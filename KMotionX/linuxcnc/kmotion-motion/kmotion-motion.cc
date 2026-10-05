/*
 * kmotion-motion - LinuxCNC's motion module as a userspace process.
 *
 * LinuxCNC's task talks to "motion" through one shared-memory block (emcmot_struct_t:
 * a command slot, a status block, the configuration and an error ring). Normally
 * motmod, a real-time HAL module, sits on the other side. This program sits there
 * instead, as a plain process, so that the moves can be handed to KMotion's
 * trajectory planner and a KFLOP/Kogna, which do the real-time work.
 *
 * This is the skeleton: it speaks the protocol completely (every command is decoded
 * and acknowledged, the status block and the HAL pins a GUI expects are kept up to
 * date) and executes the moves with a simple stand-in model - straight-line and arc
 * segments run at the commanded speed, jogs move at the jog speed, homing is
 * immediate - so that LinuxCNC's GUI is fully usable while the KMotion side is
 * plugged in step by step (see ../README.md).
 *
 * Derived from LinuxCNC's motion-logger.c (src/emc/motion-logger), which is
 * GPL-2; this file is therefore GPL-2 as well.
 *
 *   kmotion-motion [-l LOGFILE] [-p PERIOD_MS]
 *
 * Started from a HAL file with "loadusr -W kmotion-motion ...". -l logs the
 * command stream like motion-logger does.
 */

#include <cerrno>
#include <cfloat>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <map>
#include <string>
#include <signal.h>
#include <thread>
#include <chrono>
#include <unistd.h>

#include <hal.h>
#include "motion/motion.h"
#include "motion/motion_struct.h"
#include "motion/mot_priv.h"
#include "motion/homing.h"
#include <motion_types.h>

#include "kmotion-backend.h"

// ---- the shared memory block, with the names mot_priv.h's macros expect ----------
struct emcmot_struct_t *emcmotStruct = NULL;
struct emcmot_command_t *emcmotCommand = NULL;
struct emcmot_status_t *emcmotStatus = NULL;
struct emcmot_config_t *emcmotConfig = NULL;
struct emcmot_internal_t *emcmotInternal = NULL;
struct emcmot_error_t *emcmotError = NULL;
emcmot_joint_t joints[EMCMOT_MAX_JOINTS];
static int num_joints = EMCMOT_MAX_JOINTS;
static int num_spindles = EMCMOT_MAX_SPINDLES;

static struct emcmot_command_t *c = NULL;   // the command slot, as in motion-logger
static int comp_id = -1;
static int shmem_id = -1;
static volatile sig_atomic_t quit = 0;
static FILE *logfile = NULL;
static const char *logfile_name = NULL;
static double period_s = 0.005;
static KmBackend *km = NULL;                // KMotion's planner; NULL: the stand-in model below
static std::string mode = "kmotion";        // -m kmotion | standin
static double now_s();

static const int NAXES = 9;                 // x y z a b c u v w
static const int QUEUE_LIMIT = 500;         // queueFull from here: bounds task's read-ahead (motmod
                                            // takes ~2000 segments; the planner wants seconds of it)
static const char AXIS_LETTERS[NAXES + 1] = "xyzabcuvw";

static void log_print(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void log_print(const char *fmt, ...)
{
    if (logfile == NULL) {
        if (logfile_name == NULL) return;
        logfile = fopen(logfile_name, "w");
        if (logfile == NULL) {
            fprintf(stderr, "kmotion-motion: cannot open %s: %s\n", logfile_name, strerror(errno));
            logfile_name = NULL;
            return;
        }
    }
    va_list ap;
    va_start(ap, fmt);
    vfprintf(logfile, fmt, ap);
    va_end(ap);
    fflush(logfile);
}

// an operator message for task/GUI, through motion's error ring
static void report_error(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void report_error(const char *fmt, ...)
{
    char buf[EMCMOT_ERROR_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    log_print("ERROR: %s\n", buf);
    if (emcmotError) emcmotErrorPut(emcmotError, buf);
}

// ---- small vector helpers on PmCartesian ------------------------------------------
static PmCartesian vsub(const PmCartesian &a, const PmCartesian &b) { PmCartesian r = {a.x - b.x, a.y - b.y, a.z - b.z}; return r; }
static PmCartesian vadd(const PmCartesian &a, const PmCartesian &b) { PmCartesian r = {a.x + b.x, a.y + b.y, a.z + b.z}; return r; }
static PmCartesian vscale(const PmCartesian &a, double s) { PmCartesian r = {a.x * s, a.y * s, a.z * s}; return r; }
static double vdot(const PmCartesian &a, const PmCartesian &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static PmCartesian vcross(const PmCartesian &a, const PmCartesian &b)
{
    PmCartesian r = {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    return r;
}
static double vmag(const PmCartesian &a) { return sqrt(vdot(a, a)); }

static void pose_to_array(const EmcPose &p, double o[NAXES])
{
    o[0] = p.tran.x; o[1] = p.tran.y; o[2] = p.tran.z;
    o[3] = p.a; o[4] = p.b; o[5] = p.c; o[6] = p.u; o[7] = p.v; o[8] = p.w;
}
static EmcPose array_to_pose(const double o[NAXES])
{
    EmcPose p;
    p.tran.x = o[0]; p.tran.y = o[1]; p.tran.z = o[2];
    p.a = o[3]; p.b = o[4]; p.c = o[5]; p.u = o[6]; p.v = o[7]; p.w = o[8];
    return p;
}

// ---- the stand-in motion model -------------------------------------------------------
// Replaces LinuxCNC's trajectory planner for now: segments run one after the other
// at their commanded speed (feed/rapid override applied), no acceleration, no
// blending. Everything the GUI needs (positions, in-position, queue depth, the id
// of the running line, distance to go) comes out of it.
struct Segment {
    enum Kind { LINE, ARC, PROBE, RIGID_TAP } kind;
    int motion_type;        // EMC_MOTION_TYPE_*
    int id;                 // G-code line
    double vel, ini_maxvel, acc;
    double end[NAXES];
    PmCartesian center, normal;
    int turn;
    struct state_tag_t tag;
    // set when the segment starts
    double start[NAXES];
    double length;          // path length the speed applies to
    double progress;
    // arc geometry (pmCircleInit's convention)
    PmCartesian ccenter, rTan, rPerp, helix;
    double radius, angle, spiral;
};

struct Jog {
    bool active;
    bool to_target;
    double target;
    double vel;             // signed for continuous jogs
};

struct Model {
    double pos[NAXES];      // commanded = actual position, machine units
    std::deque<Segment> queue;
    bool active;            // queue.front() is running
    bool paused;
    bool stepping;          // pause again when the running segment ends
    int last_id;
    double current_vel;
    Jog jog[NAXES];
    double jog_min[NAXES], jog_max[NAXES];
    double joint_home[EMCMOT_MAX_JOINTS];
    struct HomingParams { double home, offset, final_vel, search_vel, latch_vel; int flags, sequence; };
    HomingParams hp[EMCMOT_MAX_JOINTS];       // from SET_JOINT_HOMING_PARAMS
    int home_seen;          // backend homing runs already taken into account
    unsigned home_pending;  // joints the board is homing
    bool probing;
    // with the KMotion backend
    KmState ks;             // what the backend reported last
    int errors_seen;        // backend failures already turned into the error flag
    int step_id;            // stepping: the line that was executing at the STEP command
    double last_move_time;  // when task last handed a move over (flush timing)
    int last_type;          // motion type and tag of the last move handed over
    struct state_tag_t last_tag;
};
static Model m;

static void model_init()
{
    m.queue.clear();
    m.active = m.paused = m.stepping = m.probing = false;
    m.last_id = 0;
    m.errors_seen = 0;
    m.step_id = 0;
    m.current_vel = 0;
    for (int i = 0; i < NAXES; i++) {
        m.pos[i] = 0;
        m.jog[i] = Jog();
        m.jog_min[i] = -1e9;
        m.jog_max[i] = 1e9;
    }
    for (int j = 0; j < EMCMOT_MAX_JOINTS; j++) { m.joint_home[j] = 0; m.hp[j] = Model::HomingParams(); }
    m.home_seen = 0;
    m.home_pending = 0;
    memset(&m.ks, 0, sizeof m.ks);
    m.last_move_time = 0;
    m.last_type = 0;
    memset(&m.last_tag, 0, sizeof m.last_tag);
}

static bool any_jog_active()
{
    for (int i = 0; i < NAXES; i++) if (m.jog[i].active) return true;
    return false;
}

static void stop_jogs(int which)       // -1: all
{
    for (int i = 0; i < NAXES; i++) {
        if (which >= 0 && which != i) continue;
        // only a jog that is running gets a stop on the board: a Jog<ch>=0 to an axis in
        // coordinated motion would interfere with it (ABORT comes through here too)
        if (m.jog[i].active && km && km->is_board()) km->jog_stop(i);
        m.jog[i].active = false;
    }
}

static void start_segment(Segment &s)
{
    memcpy(s.start, m.pos, sizeof s.start);
    s.progress = 0;
    if (s.kind == Segment::ARC) {
        PmCartesian start = {s.start[0], s.start[1], s.start[2]};
        PmCartesian end = {s.end[0], s.end[1], s.end[2]};
        // center projected into the plane of the start point, normal made unit,
        // negative turn flips the normal (pmCircleInit)
        double nmag = vmag(s.normal);
        PmCartesian n = nmag > 0 ? vscale(s.normal, 1.0 / nmag) : PmCartesian{0, 0, 1};
        int turn = s.turn;
        if (turn < 0) { turn = -1 - turn; n = vscale(n, -1.0); }
        PmCartesian v = vsub(start, s.center);
        PmCartesian vn = vscale(n, vdot(v, n));
        s.ccenter = vadd(s.center, vn);
        s.rTan = vsub(start, s.ccenter);
        s.radius = vmag(s.rTan);
        s.rPerp = vcross(n, s.rTan);
        PmCartesian rEndFull = vsub(end, s.ccenter);
        PmCartesian rEnd = vsub(rEndFull, vscale(n, vdot(rEndFull, n)));
        s.helix = vscale(n, vdot(rEndFull, n));
        s.spiral = vmag(rEnd) - s.radius;
        double a = 0;
        if (s.radius > 0 && vmag(rEnd) > 0) {
            double d = vdot(s.rTan, rEnd) / (s.radius * vmag(rEnd));
            if (d > 1) d = 1; else if (d < -1) d = -1;
            a = acos(d);
            if (vdot(vcross(s.rTan, rEnd), n) < 0) a = 2 * M_PI - a;
        }
        if (fabs(a) < 1e-9) a = 2 * M_PI;      // start == end: a full circle
        s.angle = a + turn * 2 * M_PI;
        s.length = fabs(s.angle) * (s.radius + s.spiral / 2) ;
        double h = vmag(s.helix);
        s.length = sqrt(s.length * s.length + h * h);
    } else {
        double d2 = 0;
        for (int i = 0; i < 3; i++) d2 += (s.end[i] - s.start[i]) * (s.end[i] - s.start[i]);
        if (d2 == 0) for (int i = 3; i < NAXES; i++) d2 += (s.end[i] - s.start[i]) * (s.end[i] - s.start[i]);
        s.length = sqrt(d2);
    }
}

static void segment_pos(const Segment &s, double f, double out[NAXES])
{
    if (f < 0) f = 0; else if (f > 1) f = 1;
    for (int i = 0; i < NAXES; i++) out[i] = s.start[i] + f * (s.end[i] - s.start[i]);
    if (s.kind == Segment::ARC && s.radius > 0) {
        double th = s.angle * f;
        double r = (s.radius + s.spiral * f) / s.radius;
        PmCartesian p = vadd(s.ccenter, vscale(vadd(vscale(s.rTan, cos(th)), vscale(s.rPerp, sin(th))), r));
        p = vadd(p, vscale(s.helix, f));
        out[0] = p.x; out[1] = p.y; out[2] = p.z;
    }
}

static double net_feed_scale()
{
    double s = 1.0;
    if (emcmotStatus->enables_new & FS_ENABLED) s *= emcmotStatus->feed_scale;
    return s;
}

static void mark_joint_homed(int j, bool homed, bool move_to_home = true);

static void model_step(double dt)
{
    m.current_vel = 0;
    // coordinated motion
    if (km) {
        km->state(m.ks);                 // the worker thread keeps it current
        if (m.ks.message[0]) report_error("%s", m.ks.message);
        if (m.stepping && emcmotStatus->id != m.step_id && m.ks.active_id) {
            km->pause(true);                  // the next line has started: the step is done
            m.stepping = false;
        }
        if (m.ks.home_serial != m.home_seen) {
            // the board's home program has finished (or was stopped)
            m.home_seen = m.ks.home_serial;
            for (int j = 0; j < EMCMOT_MAX_JOINTS && j < 8; j++) {
                if (!(m.home_pending & (1u << j))) continue;
                if (m.ks.home_ok_mask & (1u << j)) mark_joint_homed(j, true, false);
                else { mark_joint_homed(j, false); report_error("homing of joint %d failed", j); }
            }
            m.home_pending = 0;
        }
        if (m.ks.errors != m.errors_seen) {
            // the planner refused a move, or the board dropped out: the backend has stopped
            // the motion. The error flag makes task abort the program (motmod sets it when
            // tpAddLine fails); ENABLE, ABORT and the mode changes clear it
            m.errors_seen = m.ks.errors;
            SET_MOTION_ERROR_FLAG(1);
            m.queue.clear();
            m.active = m.paused = m.stepping = false;
            stop_jogs(-1);
        }
        memcpy(m.pos, m.ks.pos, sizeof m.pos);
        m.current_vel = m.ks.current_vel;
        if (m.ks.active_id) m.last_id = m.ks.active_id;
    } else if (!m.active && !m.queue.empty() && !m.paused) {
        start_segment(m.queue.front());
        m.active = true;
    }
    if (m.active) {
        Segment &s = m.queue.front();
        double v = s.vel;
        if (s.motion_type == EMC_MOTION_TYPE_TRAVERSE) v *= emcmotStatus->rapid_scale;
        else v *= net_feed_scale();
        if (m.paused) v = 0;
        s.progress += v * dt;
        m.current_vel = v;
        if (s.progress >= s.length || s.length <= 0) {
            memcpy(m.pos, s.end, sizeof m.pos);
            m.last_id = s.id;
            if (s.kind == Segment::PROBE) {
                emcmotStatus->probing = 0;
                emcmotStatus->probeTripped = 0;       // nothing to trip in the stand-in model
                emcmotStatus->probedPos = array_to_pose(m.pos);
            }
            m.queue.pop_front();
            m.active = false;
            if (m.stepping) { m.paused = true; m.stepping = false; }
        } else {
            segment_pos(s, s.progress / s.length, m.pos);
        }
    }
    // jogs (free mode: joints, teleop: axes - identical here, identity kinematics)
    for (int i = 0; i < NAXES; i++) {
        Jog &j = m.jog[i];
        if (!j.active) continue;
        double step = fabs(j.vel) * dt;
        if (j.to_target) {
            double d = j.target - m.pos[i];
            if (fabs(d) <= step) { m.pos[i] = j.target; j.active = false; }
            else m.pos[i] += d > 0 ? step : -step;
        } else {
            m.pos[i] += j.vel > 0 ? step : -step;
        }
        if (m.pos[i] > m.jog_max[i]) { m.pos[i] = m.jog_max[i]; j.active = false; }
        if (m.pos[i] < m.jog_min[i]) { m.pos[i] = m.jog_min[i]; j.active = false; }
        if (fabs(j.vel) > m.current_vel) m.current_vel = fabs(j.vel);
    }
    if (km && any_jog_active()) km->set_position(m.pos);   // the planner continues from here
}

// ---- HAL pins: what motmod offers and GUIs (AXIS at least) expect to find -----------
struct Pins {
    hal_bool_t motion_enabled, in_position, coord_mode, teleop_mode, on_soft_limit;
    hal_real_t current_vel, requested_vel, distance_to_go;
    hal_sint_t program_line, motion_type;
    struct {
        hal_real_t pos_cmd, pos_fb, motor_pos_cmd;
        hal_bool_t homed, homing, neg_lim_sw, pos_lim_sw, home_sw;
    } joint[EMCMOT_MAX_JOINTS];
    struct {
        hal_real_t pos_cmd;
    } axis[NAXES];
    struct {
        hal_bool_t on, forward, reverse, brake, at_speed;
        hal_real_t speed_out, speed_out_abs, speed_out_rps, speed_out_rps_abs, speed_cmd_rps;
    } spindle[EMCMOT_MAX_SPINDLES];
};
static Pins *pins = NULL;        // in HAL shared memory: hal_pin_new_* requires that

#define PIN(call) do { int r_ = (call); if (r_ < 0) { fprintf(stderr, "kmotion-motion: HAL pin failed: %s\n", strerror(-r_)); return r_; } } while (0)

static int create_pins()
{
    pins = (Pins *) hal_malloc(sizeof(Pins));
    if (pins == NULL) { fprintf(stderr, "kmotion-motion: hal_malloc failed\n"); return -1; }
    memset(pins, 0, sizeof(Pins));
    PIN(hal_pin_new_bool(comp_id, HAL_OUT, &pins->motion_enabled, 0, "motion.motion-enabled"));
    PIN(hal_pin_new_bool(comp_id, HAL_OUT, &pins->in_position, 1, "motion.in-position"));
    PIN(hal_pin_new_bool(comp_id, HAL_OUT, &pins->coord_mode, 0, "motion.coord-mode"));
    PIN(hal_pin_new_bool(comp_id, HAL_OUT, &pins->teleop_mode, 0, "motion.teleop-mode"));
    PIN(hal_pin_new_bool(comp_id, HAL_OUT, &pins->on_soft_limit, 0, "motion.on-soft-limit"));
    PIN(hal_pin_new_real(comp_id, HAL_OUT, &pins->current_vel, 0, "motion.current-vel"));
    PIN(hal_pin_new_real(comp_id, HAL_OUT, &pins->requested_vel, 0, "motion.requested-vel"));
    PIN(hal_pin_new_real(comp_id, HAL_OUT, &pins->distance_to_go, 0, "motion.distance-to-go"));
    PIN(hal_pin_new_si32(comp_id, HAL_OUT, &pins->program_line, 0, "motion.program-line"));
    PIN(hal_pin_new_si32(comp_id, HAL_OUT, &pins->motion_type, 0, "motion.motion-type"));
    for (int j = 0; j < EMCMOT_MAX_JOINTS; j++) {
        PIN(hal_pin_new_real(comp_id, HAL_OUT, &pins->joint[j].pos_cmd, 0, "joint.%d.pos-cmd", j));
        PIN(hal_pin_new_real(comp_id, HAL_OUT, &pins->joint[j].pos_fb, 0, "joint.%d.pos-fb", j));
        PIN(hal_pin_new_real(comp_id, HAL_OUT, &pins->joint[j].motor_pos_cmd, 0, "joint.%d.motor-pos-cmd", j));
        PIN(hal_pin_new_bool(comp_id, HAL_OUT, &pins->joint[j].homed, 0, "joint.%d.homed", j));
        PIN(hal_pin_new_bool(comp_id, HAL_OUT, &pins->joint[j].homing, 0, "joint.%d.homing", j));
        PIN(hal_pin_new_bool(comp_id, HAL_IN, &pins->joint[j].neg_lim_sw, 0, "joint.%d.neg-lim-sw-in", j));
        PIN(hal_pin_new_bool(comp_id, HAL_IN, &pins->joint[j].pos_lim_sw, 0, "joint.%d.pos-lim-sw-in", j));
        PIN(hal_pin_new_bool(comp_id, HAL_IN, &pins->joint[j].home_sw, 0, "joint.%d.home-sw-in", j));
    }
    for (int a = 0; a < NAXES; a++)
        PIN(hal_pin_new_real(comp_id, HAL_OUT, &pins->axis[a].pos_cmd, 0, "axis.%c.pos-cmd", AXIS_LETTERS[a]));
    for (int s = 0; s < EMCMOT_MAX_SPINDLES; s++) {
        PIN(hal_pin_new_bool(comp_id, HAL_OUT, &pins->spindle[s].on, 0, "spindle.%d.on", s));
        PIN(hal_pin_new_bool(comp_id, HAL_OUT, &pins->spindle[s].forward, 0, "spindle.%d.forward", s));
        PIN(hal_pin_new_bool(comp_id, HAL_OUT, &pins->spindle[s].reverse, 0, "spindle.%d.reverse", s));
        PIN(hal_pin_new_bool(comp_id, HAL_OUT, &pins->spindle[s].brake, 1, "spindle.%d.brake", s));
        PIN(hal_pin_new_bool(comp_id, HAL_IN, &pins->spindle[s].at_speed, 1, "spindle.%d.at-speed", s));
        PIN(hal_pin_new_real(comp_id, HAL_OUT, &pins->spindle[s].speed_out, 0, "spindle.%d.speed-out", s));
        PIN(hal_pin_new_real(comp_id, HAL_OUT, &pins->spindle[s].speed_out_abs, 0, "spindle.%d.speed-out-abs", s));
        PIN(hal_pin_new_real(comp_id, HAL_OUT, &pins->spindle[s].speed_out_rps, 0, "spindle.%d.speed-out-rps", s));
        PIN(hal_pin_new_real(comp_id, HAL_OUT, &pins->spindle[s].speed_out_rps_abs, 0, "spindle.%d.speed-out-rps-abs", s));
        PIN(hal_pin_new_real(comp_id, HAL_OUT, &pins->spindle[s].speed_cmd_rps, 0, "spindle.%d.speed-cmd-rps", s));
    }
    return 0;
}

// ---- the shared memory block -----------------------------------------------------------
static void config_change()
{
    if (emcmotConfig->head == emcmotConfig->tail) {
        emcmotConfig->head++;
        emcmotConfig->config_num++;
        emcmotStatus->config_num = emcmotConfig->config_num;
        emcmotConfig->tail = emcmotConfig->head;
    }
}

static int init_comm_buffers()
{
    shmem_id = rtapi_shmem_new(DEFAULT_SHMEM_KEY, comp_id, sizeof(emcmot_struct_t));
    if (shmem_id < 0) {
        fprintf(stderr, "kmotion-motion: rtapi_shmem_new failed: %d\n", shmem_id);
        return -1;
    }
    int r = rtapi_shmem_getptr(shmem_id, (void **) &emcmotStruct);
    if (r < 0) {
        fprintf(stderr, "kmotion-motion: rtapi_shmem_getptr failed: %d\n", r);
        return -1;
    }
    c = emcmotCommand = &emcmotStruct->command;
    emcmotStatus = &emcmotStruct->status;
    emcmotConfig = &emcmotStruct->config;
    emcmotInternal = &emcmotStruct->internal;
    emcmotError = &emcmotStruct->error;

    emcmotErrorInit(emcmotError);
    emcmotConfig->numJoints = num_joints;
    emcmotConfig->numSpindles = num_spindles;
    emcmotConfig->kinType = KINEMATICS_IDENTITY;
    emcmotConfig->trajCycleTime = period_s;
    emcmotConfig->servoCycleTime = period_s;
    emcmotConfig->interpolationRate = 1;
    emcmotConfig->limitVel = DEFAULT_VELOCITY;
    emcmotConfig->maxFeedScale = 1.0;
    emcmotStatus->vel = DEFAULT_VELOCITY;
    emcmotStatus->acc = DEFAULT_ACCELERATION;
    emcmotStatus->feed_scale = 1.0;
    emcmotStatus->rapid_scale = 1.0;
    for (int n = 0; n < EMCMOT_MAX_SPINDLES; n++) {
        emcmotStatus->spindle_status[n].scale = 1.0;
        emcmotStatus->spindle_status[n].net_scale = 1.0;
        emcmotStatus->spindle_status[n].at_speed = 1;
        emcmotStatus->spindle_status[n].brake = 1;
    }
    emcmotStatus->net_feed_scale = 1.0;
    // adaptive feed off, feed override, spindle override and feed hold on (motmod's defaults)
    emcmotStatus->enables_new = FS_ENABLED | SS_ENABLED | FH_ENABLED;
    emcmotStatus->enables_queued = emcmotStatus->enables_new;
    emcmotStatus->carte_pos_cmd_ok = 1;
    emcmotStatus->carte_pos_fb_ok = 1;
    SET_MOTION_INPOS_FLAG(1);
    config_change();

    for (int j = 0; j < EMCMOT_MAX_JOINTS; j++) {
        emcmot_joint_t *joint = &joints[j];
        memset(joint, 0, sizeof *joint);
        joint->max_pos_limit = 1.0;
        joint->min_pos_limit = -1.0;
        joint->vel_limit = 1.0;
        joint->acc_limit = 1.0;
        joint->min_ferror = 0.01;
        joint->max_ferror = 1.0;
        SET_JOINT_INPOS_FLAG(joint, 1);
    }
    return 0;
}

static void update_motion_state()
{
    if (!GET_MOTION_ENABLE_FLAG()) emcmotStatus->motion_state = EMCMOT_MOTION_DISABLED;
    else if (GET_MOTION_TELEOP_FLAG()) emcmotStatus->motion_state = EMCMOT_MOTION_TELEOP;
    else if (GET_MOTION_COORD_FLAG()) emcmotStatus->motion_state = EMCMOT_MOTION_COORD;
    else emcmotStatus->motion_state = EMCMOT_MOTION_FREE;
}

// everything derived from the model, once per cycle
static void update_status()
{
    bool coord_busy = km ? (m.ks.running || m.ks.homing) : (m.active || !m.queue.empty());
    bool inpos = !coord_busy && !any_jog_active();
    SET_MOTION_INPOS_FLAG(inpos ? 1 : 0);
    emcmotStatus->carte_pos_cmd = array_to_pose(m.pos);
    emcmotStatus->carte_pos_fb = emcmotStatus->carte_pos_cmd;
    int depth = km ? m.ks.depth : (int) m.queue.size();
    emcmotStatus->depth = depth;
    emcmotStatus->activeDepth = km ? (m.ks.active_id ? 1 : 0) : (m.active ? 1 : 0);
    emcmotStatus->tcqlen = (unsigned) depth;
    emcmotStatus->queueFull = depth >= QUEUE_LIMIT;
    emcmotStatus->paused = m.paused;
    emcmotStatus->stepping = m.stepping;
    emcmotStatus->jogging_active = any_jog_active();
    emcmotStatus->current_vel = m.current_vel;
    emcmotStatus->net_feed_scale = net_feed_scale();
    emcmotStatus->enables_queued = emcmotStatus->enables_new;
    if (km) {
        emcmotStatus->id = m.ks.active_id ? m.ks.active_id : m.last_id;
        emcmotStatus->motionType = m.ks.running ? m.last_type : 0;
        emcmotStatus->requested_vel = m.ks.current_vel;
        emcmotStatus->distance_to_go = m.ks.distance_to_go;
        if (m.ks.running) emcmotStatus->tag = m.last_tag;
        memset(&emcmotStatus->dtg, 0, sizeof emcmotStatus->dtg);
    } else if (m.active) {
        const Segment &s = m.queue.front();
        emcmotStatus->id = s.id;
        emcmotStatus->motionType = s.motion_type;
        emcmotStatus->requested_vel = s.vel;
        emcmotStatus->distance_to_go = s.length - s.progress;
        emcmotStatus->tag = s.tag;
        double e[NAXES];
        for (int i = 0; i < NAXES; i++) e[i] = s.end[i] - m.pos[i];
        emcmotStatus->dtg = array_to_pose(e);
    } else {
        emcmotStatus->id = m.last_id;
        emcmotStatus->motionType = 0;
        emcmotStatus->requested_vel = 0;
        emcmotStatus->distance_to_go = 0;
        memset(&emcmotStatus->dtg, 0, sizeof emcmotStatus->dtg);
    }
    for (int j = 0; j < EMCMOT_MAX_JOINTS; j++) {
        emcmot_joint_t *joint = &joints[j];
        emcmot_joint_status_t *js = &emcmotStatus->joint_status[j];
        if (j < NAXES) {                       // identity kinematics: joint j = axis j
            joint->pos_cmd = m.pos[j];
            joint->pos_fb = m.pos[j];
            joint->motor_pos_cmd = m.pos[j] + joint->motor_offset;
            joint->motor_pos_fb = joint->motor_pos_cmd;
            SET_JOINT_INPOS_FLAG(joint, m.jog[j].active ? 0 : 1);
        }
        SET_JOINT_ENABLE_FLAG(joint, GET_MOTION_ENABLE_FLAG() && (km == NULL || j >= 8 || m.ks.enabled[j]));
        js->flag = joint->flag;
        js->pos_cmd = joint->pos_cmd;
        js->pos_fb = joint->pos_fb;
        js->vel_cmd = joint->vel_cmd;
        js->acc_cmd = joint->acc_cmd;
        js->ferror = 0;
        js->ferror_high_mark = 0;
        js->backlash = joint->backlash;
        js->max_pos_limit = joint->max_pos_limit;
        js->min_pos_limit = joint->min_pos_limit;
        js->min_ferror = joint->min_ferror;
        js->max_ferror = joint->max_ferror;
    }
    for (int a = 0; a < NAXES; a++) {
        emcmotStatus->axis_status[a].max_pos_limit = m.jog_max[a];
        emcmotStatus->axis_status[a].min_pos_limit = m.jog_min[a];
        emcmotStatus->axis_status[a].teleop_vel_cmd = m.jog[a].active ? m.jog[a].vel : 0;
    }
    emcmotStatus->heartbeat++;
}

static void update_pins()
{
    hal_set_bool(pins->motion_enabled, GET_MOTION_ENABLE_FLAG());
    hal_set_bool(pins->in_position, GET_MOTION_INPOS_FLAG());
    hal_set_bool(pins->coord_mode, GET_MOTION_COORD_FLAG());
    hal_set_bool(pins->teleop_mode, GET_MOTION_TELEOP_FLAG());
    hal_set_real(pins->current_vel, emcmotStatus->current_vel);
    hal_set_real(pins->requested_vel, emcmotStatus->requested_vel);
    hal_set_real(pins->distance_to_go, emcmotStatus->distance_to_go);
    hal_set_si32(pins->program_line, emcmotStatus->id);
    hal_set_si32(pins->motion_type, emcmotStatus->motionType);
    for (int j = 0; j < EMCMOT_MAX_JOINTS; j++) {
        hal_set_real(pins->joint[j].pos_cmd, joints[j].pos_cmd);
        hal_set_real(pins->joint[j].pos_fb, joints[j].pos_fb);
        hal_set_real(pins->joint[j].motor_pos_cmd, joints[j].motor_pos_cmd);
        hal_set_bool(pins->joint[j].homed, emcmotStatus->joint_status[j].homed);
        hal_set_bool(pins->joint[j].homing, emcmotStatus->joint_status[j].homing);
    }
    for (int a = 0; a < NAXES; a++) hal_set_real(pins->axis[a].pos_cmd, m.pos[a]);
    for (int s = 0; s < EMCMOT_MAX_SPINDLES; s++) {
        const spindle_status_t &ss = emcmotStatus->spindle_status[s];
        double rpm = ss.speed * ss.net_scale;
        hal_set_bool(pins->spindle[s].on, ss.speed != 0);
        hal_set_bool(pins->spindle[s].forward, ss.speed > 0);
        hal_set_bool(pins->spindle[s].reverse, ss.speed < 0);
        hal_set_bool(pins->spindle[s].brake, ss.brake);
        hal_set_real(pins->spindle[s].speed_out, rpm);
        hal_set_real(pins->spindle[s].speed_out_abs, fabs(rpm));
        hal_set_real(pins->spindle[s].speed_out_rps, rpm / 60.0);
        hal_set_real(pins->spindle[s].speed_out_rps_abs, fabs(rpm) / 60.0);
        hal_set_real(pins->spindle[s].speed_cmd_rps, ss.speed / 60.0);
    }
}

// ---- commands --------------------------------------------------------------------------
static void mark_joint_homed(int j, bool homed, bool move_to_home)
{
    if (j < 0 || j >= EMCMOT_MAX_JOINTS) return;
    emcmot_joint_status_t *js = &emcmotStatus->joint_status[j];
    js->homing = 0;
    js->homed = homed;
    if (homed && move_to_home && j < NAXES) m.pos[j] = m.joint_home[j];   // homing ends at the HOME position
}

static void queue_segment(Segment &s)
{
    pose_to_array(c->pos, s.end);
    s.id = c->id;
    s.motion_type = c->motion_type;
    s.vel = c->vel;
    s.ini_maxvel = c->ini_maxvel;
    s.acc = c->acc;
    s.tag = c->tag;
    s.turn = c->turn;
    s.center = c->center;
    s.normal = c->normal;
    if (s.vel <= 0) s.vel = s.ini_maxvel > 0 ? s.ini_maxvel : emcmotStatus->vel;
    m.queue.push_back(s);
}

static void start_jog(int index, bool to_target, double target, double vel)
{
    if (index < 0 || index >= NAXES) return;
    if (!GET_MOTION_ENABLE_FLAG() || (km ? m.ks.running : (m.active && !m.paused))) return;
    if (km && km->is_board()) {
        // the board jogs the actuator itself, within its own axis limits
        if (to_target) km->jog_to(index, target, vel);
        else km->jog(index, vel);
        return;
    }
    Jog &j = m.jog[index];
    j.active = true;
    j.to_target = to_target;
    j.target = target;
    j.vel = vel;
}

static void handle_command()
{
    emcmotStatus->commandStatus = EMCMOT_COMMAND_OK;
    switch (c->command) {
    case EMCMOT_ABORT:
        log_print("ABORT\n");
        if (km) km->abort();
        SET_MOTION_ERROR_FLAG(0);
        m.queue.clear();
        m.active = false;
        m.paused = false;
        m.stepping = false;
        stop_jogs(-1);
        emcmotStatus->probing = 0;
        break;
    case EMCMOT_JOG_ABORT:
        log_print("JOG_ABORT joint=%d axis=%d\n", c->joint, c->axis);
        stop_jogs(GET_MOTION_TELEOP_FLAG() ? c->axis : c->joint);
        break;
    case EMCMOT_ENABLE:
        log_print("ENABLE\n");
        SET_MOTION_ENABLE_FLAG(1);
        SET_MOTION_ERROR_FLAG(0);
        if (km && km->is_board()) km->machine_on(true);
        update_motion_state();
        break;
    case EMCMOT_DISABLE:
        log_print("DISABLE\n");
        SET_MOTION_ENABLE_FLAG(0);
        if (km) km->abort();
        if (km && km->is_board()) km->machine_on(false);
        m.queue.clear();
        m.active = false;
        m.paused = false;
        stop_jogs(-1);
        update_motion_state();
        break;
    case EMCMOT_JOINT_ACTIVATE:
        log_print("JOINT_ACTIVATE joint=%d\n", c->joint);
        if (c->joint >= 0 && c->joint < EMCMOT_MAX_JOINTS) { SET_JOINT_ACTIVE_FLAG(&joints[c->joint], 1); }
        break;
    case EMCMOT_JOINT_DEACTIVATE:
        log_print("JOINT_DEACTIVATE joint=%d\n", c->joint);
        if (c->joint >= 0 && c->joint < EMCMOT_MAX_JOINTS) { SET_JOINT_ACTIVE_FLAG(&joints[c->joint], 0); }
        break;
    case EMCMOT_PAUSE:
        log_print("PAUSE\n");
        m.paused = true;
        if (km) km->pause(true);
        break;
    case EMCMOT_RESUME:
        log_print("RESUME\n");
        m.paused = false;
        m.stepping = false;
        if (km) km->pause(false);
        break;
    case EMCMOT_STEP:
        log_print("STEP\n");
        // motmod: resume the paused motion until the executing line changes, then pause
        // again; "paused" stays set throughout. With the board that pause is its feed hold,
        // so the stop lands a little way into the next line
        if (m.paused) {
            m.stepping = true;
            if (km) { m.step_id = emcmotStatus->id; km->pause(false); }
            else m.paused = false;
        }
        break;
    case EMCMOT_REVERSE:
        log_print("REVERSE (not supported here)\n");
        break;
    case EMCMOT_FORWARD:
        log_print("FORWARD\n");
        break;
    case EMCMOT_FREE:
        log_print("FREE\n");
        SET_MOTION_COORD_FLAG(0);
        SET_MOTION_TELEOP_FLAG(0);
        update_motion_state();
        break;
    case EMCMOT_COORD:
        log_print("COORD\n");
        SET_MOTION_COORD_FLAG(1);
        SET_MOTION_TELEOP_FLAG(0);
        SET_MOTION_ERROR_FLAG(0);
        stop_jogs(-1);
        update_motion_state();
        break;
    case EMCMOT_TELEOP:
        log_print("TELEOP\n");
        SET_MOTION_TELEOP_FLAG(1);
        SET_MOTION_ERROR_FLAG(0);
        update_motion_state();
        break;
    case EMCMOT_SPINDLE_SCALE:
        log_print("SPINDLE_SCALE spindle=%d scale=%.6g\n", c->spindle, c->scale);
        if (c->spindle >= 0 && c->spindle < EMCMOT_MAX_SPINDLES) {
            emcmotStatus->spindle_status[c->spindle].scale = c->scale;
            emcmotStatus->spindle_status[c->spindle].net_scale = (emcmotStatus->enables_new & SS_ENABLED) ? c->scale : 1.0;
        }
        break;
    case EMCMOT_SS_ENABLE:
        log_print("SS_ENABLE %d\n", c->mode);
        if (c->mode) emcmotStatus->enables_new |= SS_ENABLED; else emcmotStatus->enables_new &= ~SS_ENABLED;
        break;
    case EMCMOT_FEED_SCALE:
        log_print("FEED_SCALE %.6g\n", c->scale);
        emcmotStatus->feed_scale = c->scale;
        if (km) km->set_feed_override(net_feed_scale(), emcmotStatus->rapid_scale);
        break;
    case EMCMOT_RAPID_SCALE:
        log_print("RAPID_SCALE %.6g\n", c->scale);
        emcmotStatus->rapid_scale = c->scale;
        if (km) km->set_feed_override(net_feed_scale(), emcmotStatus->rapid_scale);
        break;
    case EMCMOT_FS_ENABLE:
        log_print("FS_ENABLE %d\n", c->mode);
        if (c->mode) emcmotStatus->enables_new |= FS_ENABLED; else emcmotStatus->enables_new &= ~FS_ENABLED;
        if (km) km->set_feed_override(net_feed_scale(), emcmotStatus->rapid_scale);
        break;
    case EMCMOT_FH_ENABLE:
        log_print("FH_ENABLE %d\n", c->mode);
        if (c->mode) emcmotStatus->enables_new |= FH_ENABLED; else emcmotStatus->enables_new &= ~FH_ENABLED;
        break;
    case EMCMOT_AF_ENABLE:
        log_print("AF_ENABLE %d\n", c->flags);
        if (c->flags) emcmotStatus->enables_new |= AF_ENABLED; else emcmotStatus->enables_new &= ~AF_ENABLED;
        break;
    case EMCMOT_OVERRIDE_LIMITS:
        log_print("OVERRIDE_LIMITS joint=%d\n", c->joint);
        emcmotStatus->overrideLimitMask = c->joint < 0 ? 0 : 1;
        break;
    case EMCMOT_JOINT_HOME: {
        log_print("JOINT_HOME joint=%d\n", c->joint);
        // with a home program on the board, joints with a search velocity are homed there
        // (the result comes back through the backend's state); a joint with HOME_SEARCH_VEL 0
        // is declared homed where it stands, as in LinuxCNC
        KmHomeRequest req;
        unsigned mask = 0;
        for (int j = 0; j < num_joints; j++) {
            if (c->joint >= 0 && j != c->joint) continue;
            bool on_board = km && km->has_home_program() && j < 8 && m.hp[j].search_vel != 0;
            if (!on_board) { mark_joint_homed(j, true); continue; }
            KmHomeJoint &hj = req.joint[j];
            hj.home = true;
            hj.search_vel = m.hp[j].search_vel;
            hj.latch_vel = m.hp[j].latch_vel;
            hj.final_vel = m.hp[j].final_vel;
            hj.offset = m.hp[j].offset;
            hj.home_pos = m.hp[j].home;
            hj.use_index = (m.hp[j].flags & HOME_USE_INDEX) != 0;
            hj.no_final_move = (m.hp[j].flags & HOME_NO_FINAL_MOVE) != 0;
            hj.sequence = m.hp[j].sequence;
            double travel = joints[j].max_pos_limit - joints[j].min_pos_limit;
            hj.max_travel = (travel > 0 && travel < 1e6) ? travel : 0;
            emcmotStatus->joint_status[j].homing = 1;
            emcmotStatus->joint_status[j].homed = 0;
            mask |= 1u << j;
        }
        if (mask) { m.home_pending |= mask; km->home(req); }
        else if (km) km->set_position(m.pos);
        break;
    }
    case EMCMOT_JOINT_UNHOME:
        log_print("JOINT_UNHOME joint=%d\n", c->joint);
        if (c->joint < 0) {
            for (int j = 0; j < num_joints; j++) mark_joint_homed(j, false);
        } else {
            mark_joint_homed(c->joint, false);
        }
        break;
    case EMCMOT_JOG_CONT:
        log_print("JOG_CONT joint=%d axis=%d vel=%.6g\n", c->joint, c->axis, c->vel);
        start_jog(GET_MOTION_TELEOP_FLAG() ? c->axis : c->joint, false, 0, c->vel);
        break;
    case EMCMOT_JOG_INCR: {
        int i = GET_MOTION_TELEOP_FLAG() ? c->axis : c->joint;
        log_print("JOG_INCR joint=%d axis=%d offset=%.6g vel=%.6g\n", c->joint, c->axis, c->offset, c->vel);
        if (i >= 0 && i < NAXES) {
            double from = m.jog[i].active && m.jog[i].to_target ? m.jog[i].target : m.pos[i];
            start_jog(i, true, from + c->offset, c->vel);
        }
        break;
    }
    case EMCMOT_JOG_ABS:
        log_print("JOG_ABS joint=%d axis=%d target=%.6g vel=%.6g\n", c->joint, c->axis, c->offset, c->vel);
        start_jog(GET_MOTION_TELEOP_FLAG() ? c->axis : c->joint, true, c->offset, c->vel);
        break;
    case EMCMOT_SET_LINE: {
        log_print("SET_LINE x=%.6g, y=%.6g, z=%.6g, a=%.6g, b=%.6g, c=%.6g, u=%.6g, v=%.6g, w=%.6g, id=%d, motion_type=%d, vel=%.6g, ini_maxvel=%.6g, acc=%.6g, turn=%d\n",
                  c->pos.tran.x, c->pos.tran.y, c->pos.tran.z, c->pos.a, c->pos.b, c->pos.c, c->pos.u, c->pos.v, c->pos.w,
                  c->id, c->motion_type, c->vel, c->ini_maxvel, c->acc, c->turn);
        if (!GET_MOTION_COORD_FLAG()) { emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_COMMAND; report_error("SET_LINE outside coordinated mode"); break; }
        if (km) {
            double end[NAXES];
            pose_to_array(c->pos, end);
            if (km->line(end, c->vel, c->acc, c->motion_type == EMC_MOTION_TYPE_TRAVERSE, c->id))
                emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
            m.last_move_time = now_s(); m.last_type = c->motion_type; m.last_tag = c->tag;
            break;
        }
        if (m.queue.size() >= (size_t) QUEUE_LIMIT) { emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_COMMAND; report_error("motion queue full"); break; }
        Segment s;
        memset(&s, 0, sizeof s);
        s.kind = Segment::LINE;
        queue_segment(s);
        break;
    }
    case EMCMOT_SET_CIRCLE: {
        log_print("SET_CIRCLE end x=%.6g, y=%.6g, z=%.6g, a=%.6g, b=%.6g, c=%.6g, u=%.6g, v=%.6g, w=%.6g; center %.6g %.6g %.6g; normal %.6g %.6g %.6g; id=%d, motion_type=%d, vel=%.6g, ini_maxvel=%.6g, acc=%.6g, turn=%d\n",
                  c->pos.tran.x, c->pos.tran.y, c->pos.tran.z, c->pos.a, c->pos.b, c->pos.c, c->pos.u, c->pos.v, c->pos.w,
                  c->center.x, c->center.y, c->center.z, c->normal.x, c->normal.y, c->normal.z,
                  c->id, c->motion_type, c->vel, c->ini_maxvel, c->acc, c->turn);
        if (!GET_MOTION_COORD_FLAG()) { emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_COMMAND; report_error("SET_CIRCLE outside coordinated mode"); break; }
        if (km) {
            double end[NAXES], center[3] = {c->center.x, c->center.y, c->center.z}, normal[3] = {c->normal.x, c->normal.y, c->normal.z};
            pose_to_array(c->pos, end);
            if (km->arc(end, center, normal, c->turn, c->vel, c->acc, c->id))
                emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
            m.last_move_time = now_s(); m.last_type = c->motion_type; m.last_tag = c->tag;
            break;
        }
        if (m.queue.size() >= (size_t) QUEUE_LIMIT) { emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_COMMAND; report_error("motion queue full"); break; }
        Segment s;
        memset(&s, 0, sizeof s);
        s.kind = Segment::ARC;
        queue_segment(s);
        break;
    }
    case EMCMOT_CLEAR_PROBE_FLAGS:
        log_print("CLEAR_PROBE_FLAGS\n");
        emcmotStatus->probing = 0;
        emcmotStatus->probeTripped = 0;
        break;
    case EMCMOT_PROBE: {
        log_print("PROBE to x=%.6g, y=%.6g, z=%.6g, vel=%.6g type=%d (runs as a plain move here, never trips)\n",
                  c->pos.tran.x, c->pos.tran.y, c->pos.tran.z, c->vel, c->probe_type);
        if (km) {
            double end[NAXES];
            pose_to_array(c->pos, end);
            if (km->line(end, c->vel, c->acc, false, c->id)) emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
            m.last_move_time = now_s(); m.last_type = EMC_MOTION_TYPE_PROBING; m.last_tag = c->tag;
            emcmotStatus->probing = 0;
            emcmotStatus->probeTripped = 0;
            break;
        }
        Segment s;
        memset(&s, 0, sizeof s);
        s.kind = Segment::PROBE;
        queue_segment(s);
        s.motion_type = EMC_MOTION_TYPE_PROBING;
        emcmotStatus->probing = 1;
        emcmotStatus->probeTripped = 0;
        emcmotStatus->probe_type = c->probe_type;
        break;
    }
    case EMCMOT_RIGID_TAP: {
        log_print("RIGID_TAP to z=%.6g vel=%.6g (runs as a plain move here)\n", c->pos.tran.z, c->vel);
        if (km) {
            double end[NAXES];
            pose_to_array(c->pos, end);
            if (km->line(end, c->vel, c->acc, false, c->id)) emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
            m.last_move_time = now_s(); m.last_type = c->motion_type; m.last_tag = c->tag;
            break;
        }
        Segment s;
        memset(&s, 0, sizeof s);
        s.kind = Segment::RIGID_TAP;
        queue_segment(s);
        break;
    }
    case EMCMOT_SET_JOINT_POSITION_LIMITS:
        log_print("SET_JOINT_POSITION_LIMITS joint=%d, min=%.6g, max=%.6g\n", c->joint, c->minLimit, c->maxLimit);
        if (c->joint >= 0 && c->joint < EMCMOT_MAX_JOINTS) {
            joints[c->joint].max_pos_limit = c->maxLimit;
            joints[c->joint].min_pos_limit = c->minLimit;
        }
        break;
    case EMCMOT_SET_AXIS_POSITION_LIMITS:
        log_print("SET_AXIS_POSITION_LIMITS axis=%d, min=%.6g, max=%.6g\n", c->axis, c->minLimit, c->maxLimit);
        if (c->axis >= 0 && c->axis < NAXES) { m.jog_min[c->axis] = c->minLimit; m.jog_max[c->axis] = c->maxLimit; }
        break;
    case EMCMOT_SET_AXIS_LOCKING_JOINT:
        log_print("SET_AXIS_LOCKING_JOINT axis=%d, locking_joint=%d\n", c->axis, c->joint);
        break;
    case EMCMOT_SET_AXIS_JERK_LIMIT:
        log_print("SET_AXIS_JERK_LIMIT axis=%d, jerk=%.6g\n", c->axis, c->jerk);
        break;
    case EMCMOT_SET_JOINT_BACKLASH:
        log_print("SET_JOINT_BACKLASH joint=%d, backlash=%.6g\n", c->joint, c->backlash);
        if (c->joint >= 0 && c->joint < EMCMOT_MAX_JOINTS) joints[c->joint].backlash = c->backlash;
        break;
    case EMCMOT_SET_JOINT_MIN_FERROR:
        log_print("SET_JOINT_MIN_FERROR joint=%d, minFerror=%.6g\n", c->joint, c->minFerror);
        if (c->joint >= 0 && c->joint < EMCMOT_MAX_JOINTS) joints[c->joint].min_ferror = c->minFerror;
        break;
    case EMCMOT_SET_JOINT_MAX_FERROR:
        log_print("SET_JOINT_MAX_FERROR joint=%d, maxFerror=%.6g\n", c->joint, c->maxFerror);
        if (c->joint >= 0 && c->joint < EMCMOT_MAX_JOINTS) joints[c->joint].max_ferror = c->maxFerror;
        break;
    case EMCMOT_SET_VEL:
        log_print("SET_VEL vel=%.6g, ini_maxvel=%.6g\n", c->vel, c->ini_maxvel);
        emcmotStatus->vel = c->vel;
        break;
    case EMCMOT_SET_VEL_LIMIT:
        log_print("SET_VEL_LIMIT vel=%.6g\n", c->vel);
        emcmotConfig->limitVel = c->vel;
        config_change();
        break;
    case EMCMOT_SET_AXIS_VEL_LIMIT:
        log_print("SET_AXIS_VEL_LIMIT axis=%d vel=%.6g\n", c->axis, c->vel);
        break;
    case EMCMOT_SET_JOINT_VEL_LIMIT:
        log_print("SET_JOINT_VEL_LIMIT joint=%d, vel=%.6g\n", c->joint, c->vel);
        if (c->joint >= 0 && c->joint < EMCMOT_MAX_JOINTS) joints[c->joint].vel_limit = c->vel;
        break;
    case EMCMOT_SET_AXIS_ACC_LIMIT:
        log_print("SET_AXIS_ACC_LIMIT axis=%d, acc=%.6g\n", c->axis, c->acc);
        break;
    case EMCMOT_SET_JOINT_ACC_LIMIT:
        log_print("SET_JOINT_ACC_LIMIT joint=%d, acc=%.6g\n", c->joint, c->acc);
        if (c->joint >= 0 && c->joint < EMCMOT_MAX_JOINTS) joints[c->joint].acc_limit = c->acc;
        break;
    case EMCMOT_SET_ACC:
        log_print("SET_ACC acc=%.6g\n", c->acc);
        emcmotStatus->acc = c->acc;
        break;
    case EMCMOT_SET_JERK:
        log_print("SET_JERK jerk=%.6g\n", c->jerk);
        emcmotStatus->jerk = c->jerk;
        break;
    case EMCMOT_SET_PLANNER_TYPE:
        log_print("SET_PLANNER_TYPE planner_type=%d\n", c->planner_type);
        emcmotStatus->planner_type = c->planner_type;
        break;
    case EMCMOT_SET_SCURVE_PEAK_SCALE:
        log_print("SET_SCURVE_PEAK_SCALE scale=%.6g\n", c->scurve_peak_scale);
        emcmotStatus->scurve_peak_scale = c->scurve_peak_scale;
        break;
    case EMCMOT_SET_TERM_COND:
        log_print("SET_TERM_COND termCond=%d, tolerance=%.6g\n", c->termCond, c->tolerance);
        if (km) km->set_path_mode(c->termCond, c->tolerance);     // G61/G61.1/G64 P to the planner
        break;
    case EMCMOT_SET_NUM_JOINTS:
        log_print("SET_NUM_JOINTS %d\n", c->joint);
        if (c->joint > 0 && c->joint <= EMCMOT_MAX_JOINTS) {
            num_joints = c->joint;
            emcmotConfig->numJoints = num_joints;
            config_change();
        }
        break;
    case EMCMOT_SET_NUM_SPINDLES:
        log_print("SET_NUM_SPINDLES %d\n", c->spindle);
        if (c->spindle > 0 && c->spindle <= EMCMOT_MAX_SPINDLES) {
            num_spindles = c->spindle;
            emcmotConfig->numSpindles = num_spindles;
            config_change();
        }
        break;
    case EMCMOT_SET_WORLD_HOME:
        log_print("SET_WORLD_HOME x=%.6g, y=%.6g, z=%.6g\n", c->pos.tran.x, c->pos.tran.y, c->pos.tran.z);
        emcmotStatus->world_home = c->pos;
        break;
    case EMCMOT_SET_JOINT_HOMING_PARAMS:
        log_print("SET_JOINT_HOMING_PARAMS joint=%d, offset=%.6g home=%.6g, final_vel=%.6g, search_vel=%.6g, latch_vel=%.6g, flags=0x%08x, sequence=%d, volatile=%d\n",
                  c->joint, c->offset, c->home, c->home_final_vel, c->search_vel, c->latch_vel, c->flags, c->home_sequence, c->volatile_home);
        if (c->joint >= 0 && c->joint < EMCMOT_MAX_JOINTS) {
            m.joint_home[c->joint] = c->home;
            Model::HomingParams &hp = m.hp[c->joint];
            hp.home = c->home; hp.offset = c->offset; hp.final_vel = c->home_final_vel;
            hp.search_vel = c->search_vel; hp.latch_vel = c->latch_vel; hp.flags = c->flags; hp.sequence = c->home_sequence;
        }
        break;
    case EMCMOT_SET_JOINT_JERK_LIMIT:
        log_print("SET_JOINT_JERK_LIMIT joint=%d, jerk=%.6g\n", c->joint, c->jerk);
        if (c->joint >= 0 && c->joint < EMCMOT_MAX_JOINTS) joints[c->joint].jerk_limit = c->jerk;
        break;
    case EMCMOT_UPDATE_JOINT_HOMING_PARAMS:
        log_print("UPDATE_JOINT_HOMING_PARAMS joint=%d, offset=%.6g home=%.6g home_sequence=%d\n", c->joint, c->offset, c->home, c->home_sequence);
        if (c->joint >= 0 && c->joint < EMCMOT_MAX_JOINTS) {
            m.joint_home[c->joint] = c->home;
            m.hp[c->joint].home = c->home; m.hp[c->joint].offset = c->offset; m.hp[c->joint].sequence = c->home_sequence;
        }
        break;
    case EMCMOT_SET_DEBUG:
        log_print("SET_DEBUG %d\n", c->debug);
        emcmotConfig->debug = c->debug;
        config_change();
        break;
    case EMCMOT_SET_DOUT:
        log_print("SET_DOUT out=%d start=%d end=%d now=%d\n", c->out, c->start, c->end, c->now);
        if (c->out < EMCMOT_MAX_DIO) emcmotStatus->synch_do[c->out] = c->start;
        break;
    case EMCMOT_SET_AOUT:
        log_print("SET_AOUT out=%d value=%.6g now=%d\n", c->out, c->minLimit, c->now);
        if (c->out < EMCMOT_MAX_AIO) emcmotStatus->analog_output[c->out] = c->minLimit;
        break;
    case EMCMOT_SET_SPINDLE_PARAMS:
        log_print("SET_SPINDLE_PARAMS spindle=%d max_pos=%.6g min_pos=%.6g max_neg=%.6g min_neg=%.6g\n",
                  c->spindle, c->maxLimit, c->min_pos_speed, c->minLimit, c->max_neg_speed);
        if (c->spindle >= 0 && c->spindle < EMCMOT_MAX_SPINDLES) {
            spindle_status_t &ss = emcmotStatus->spindle_status[c->spindle];
            ss.max_pos_speed = c->maxLimit; ss.min_pos_speed = c->min_pos_speed;
            ss.min_neg_speed = c->minLimit; ss.max_neg_speed = c->max_neg_speed;
        }
        break;
    case EMCMOT_SET_SPINDLESYNC:
        log_print("SET_SPINDLESYNC sync=%.6f, flags=0x%08x\n", c->spindlesync, c->flags);
        emcmotStatus->spindleSync = c->spindlesync != 0;
        break;
    case EMCMOT_SPINDLE_ON:
        log_print("SPINDLE_ON spindle=%d speed=%.6g css_factor=%.6g xoffset=%.6g wait_at_speed=%d\n",
                  c->spindle, c->vel, c->ini_maxvel, c->acc, c->wait_for_spindle_at_speed);
        if (c->spindle >= 0 && c->spindle < EMCMOT_MAX_SPINDLES) {
            spindle_status_t &ss = emcmotStatus->spindle_status[c->spindle];
            ss.speed = c->vel;
            ss.css_factor = c->ini_maxvel;
            ss.xoffset = c->acc;
            ss.direction = c->vel > 0 ? 1 : (c->vel < 0 ? -1 : 0);
            ss.brake = 0;
            ss.state = 1;
            ss.at_speed = 1;
        }
        break;
    case EMCMOT_SPINDLE_OFF:
        log_print("SPINDLE_OFF spindle=%d\n", c->spindle);
        if (c->spindle >= 0 && c->spindle < EMCMOT_MAX_SPINDLES) {
            spindle_status_t &ss = emcmotStatus->spindle_status[c->spindle];
            ss.speed = 0; ss.direction = 0; ss.state = 0;
        }
        break;
    case EMCMOT_SPINDLE_INCREASE:
        log_print("SPINDLE_INCREASE spindle=%d\n", c->spindle);
        if (c->spindle >= 0 && c->spindle < EMCMOT_MAX_SPINDLES) {
            spindle_status_t &ss = emcmotStatus->spindle_status[c->spindle];
            ss.speed += ss.speed > 0 ? 100 : (ss.speed < 0 ? -100 : 0);
        }
        break;
    case EMCMOT_SPINDLE_DECREASE:
        log_print("SPINDLE_DECREASE spindle=%d\n", c->spindle);
        if (c->spindle >= 0 && c->spindle < EMCMOT_MAX_SPINDLES) {
            spindle_status_t &ss = emcmotStatus->spindle_status[c->spindle];
            if (ss.speed > 100) ss.speed -= 100; else if (ss.speed < -100) ss.speed += 100;
        }
        break;
    case EMCMOT_SPINDLE_BRAKE_ENGAGE:
        log_print("SPINDLE_BRAKE_ENGAGE spindle=%d\n", c->spindle);
        if (c->spindle >= 0 && c->spindle < EMCMOT_MAX_SPINDLES) {
            spindle_status_t &ss = emcmotStatus->spindle_status[c->spindle];
            ss.speed = 0; ss.direction = 0; ss.state = 0; ss.brake = 1;
        }
        break;
    case EMCMOT_SPINDLE_BRAKE_RELEASE:
        log_print("SPINDLE_BRAKE_RELEASE spindle=%d\n", c->spindle);
        if (c->spindle >= 0 && c->spindle < EMCMOT_MAX_SPINDLES) emcmotStatus->spindle_status[c->spindle].brake = 0;
        break;
    case EMCMOT_SPINDLE_ORIENT:
        log_print("SPINDLE_ORIENT spindle=%d orientation=%.6g mode=%d (not supported here)\n", c->spindle, c->orientation, c->mode);
        break;
    case EMCMOT_SET_JOINT_MOTOR_OFFSET:
        log_print("SET_JOINT_MOTOR_OFFSET joint=%d offset=%.6g\n", c->joint, c->motor_offset);
        if (c->joint >= 0 && c->joint < EMCMOT_MAX_JOINTS) joints[c->joint].motor_offset = c->motor_offset;
        break;
    case EMCMOT_SET_JOINT_COMP:
        log_print("SET_JOINT_COMP joint=%d nominal=%.6g fwd=%.6g rev=%.6g (ignored)\n", c->joint, c->comp_nominal, c->comp_forward, c->comp_reverse);
        break;
    case EMCMOT_SET_OFFSET:
        log_print("SET_OFFSET x=%.6g, y=%.6g, z=%.6g, a=%.6g, b=%.6g, c=%.6g u=%.6g, v=%.6g, w=%.6g\n",
                  c->tool_offset.tran.x, c->tool_offset.tran.y, c->tool_offset.tran.z,
                  c->tool_offset.a, c->tool_offset.b, c->tool_offset.c, c->tool_offset.u, c->tool_offset.v, c->tool_offset.w);
        emcmotStatus->tool_offset = c->tool_offset;
        break;
    case EMCMOT_SET_MAX_FEED_OVERRIDE:
        log_print("SET_MAX_FEED_OVERRIDE %.6g\n", c->maxFeedScale);
        emcmotConfig->maxFeedScale = c->maxFeedScale;
        config_change();
        break;
    case EMCMOT_SETUP_ARC_BLENDS:
        log_print("SETUP_ARC_BLENDS enable=%d depth=%d fallback=%d gap=%d ramp=%.6g kink=%.6g\n",
                  c->arcBlendEnable, c->arcBlendOptDepth, c->arcBlendFallbackEnable, c->arcBlendGapCycles,
                  c->arcBlendRampFreq, c->arcBlendTangentKinkRatio);
        emcmotConfig->arcBlendEnable = c->arcBlendEnable;
        emcmotConfig->arcBlendOptDepth = c->arcBlendOptDepth;
        emcmotConfig->arcBlendFallbackEnable = c->arcBlendFallbackEnable;
        emcmotConfig->arcBlendGapCycles = c->arcBlendGapCycles;
        emcmotConfig->arcBlendRampFreq = c->arcBlendRampFreq;
        emcmotConfig->arcBlendTangentKinkRatio = c->arcBlendTangentKinkRatio;
        config_change();
        break;
    case EMCMOT_SET_PROBE_ERR_INHIBIT:
        log_print("SET_PROBE_ERR_INHIBIT %d %d\n", c->probe_jog_err_inhibit, c->probe_home_err_inhibit);
        emcmotConfig->inhibit_probe_jog_error = c->probe_jog_err_inhibit;
        emcmotConfig->inhibit_probe_home_error = c->probe_home_err_inhibit;
        config_change();
        break;
    case EMCMOT_SELECT_KINS_TYPE:
        log_print("SELECT_KINS_TYPE %d (identity kinematics only here)\n", c->switchkins_type);
        emcmotStatus->switchkins_type = 0;
        emcmotStatus->switchkins_seq++;
        break;
    default:
        log_print("unknown command %d\n", (int) c->command);
        emcmotStatus->commandStatus = EMCMOT_COMMAND_UNKNOWN_COMMAND;
        break;
    }
}

// ---- the KMotion backend's configuration, from the LinuxCNC ini -------------------------
// Minimal ini reader: "[SECTION]" headers and "KEY = value" lines, '#' and ';' comments,
// the first occurrence of a key wins (LinuxCNC's own convention).
struct Ini {
    std::map<std::string, std::string> kv;
    bool load(const char *path)
    {
        std::ifstream f(path);
        if (!f) return false;
        std::string line, sec;
        while (std::getline(f, line)) {
            size_t p = line.find_first_of("#;");
            if (p != std::string::npos) line.erase(p);
            size_t a = line.find_first_not_of(" \t\r"), b = line.find_last_not_of(" \t\r");
            if (a == std::string::npos) continue;
            line = line.substr(a, b - a + 1);
            if (line[0] == '[') { sec = line.substr(1, line.find(']') - 1); continue; }
            size_t eq = line.find('=');
            if (eq == std::string::npos) continue;
            std::string k = line.substr(0, eq), v = line.substr(eq + 1);
            k.erase(k.find_last_not_of(" \t") + 1);
            v.erase(0, v.find_first_not_of(" \t"));
            kv.insert(std::make_pair(sec + "." + k, v));
        }
        return true;
    }
    const char *get(const std::string &sec, const std::string &key) const
    {
        std::map<std::string, std::string>::const_iterator it = kv.find(sec + "." + key);
        return it == kv.end() ? NULL : it->second.c_str();
    }
    double num(const std::string &sec, const std::string &key, double dflt) const
    {
        const char *v = get(sec, key);
        return v ? atof(v) : dflt;
    }
};

static bool read_kmotion_config(KmConfig &cfg)
{
    const char *ini_path = getenv("INI_FILE_NAME");
    Ini ini;
    if (!ini_path || !ini.load(ini_path)) {
        fprintf(stderr, "kmotion-motion: cannot read the ini file (INI_FILE_NAME=%s)\n", ini_path ? ini_path : "unset");
        return false;
    }
    const char *K = "KMOTION";
    const char *mode_ini = ini.get(K, "MODE");
    if (mode_ini && !strcasecmp(mode_ini, "board")) cfg.simulate = false;
    else if (mode_ini && !strcasecmp(mode_ini, "standin")) mode = "standin";
    const char *prog = ini.get(K, "INIT_PROGRAM");
    if (prog) snprintf(cfg.init_program, sizeof cfg.init_program, "%s", prog);
    cfg.init_thread = (int) ini.num(K, "INIT_THREAD", 1);
    const char *home = ini.get(K, "HOME_PROGRAM");
    if (home && home[0]) {
        if (home[0] == '/') snprintf(cfg.home_program, sizeof cfg.home_program, "%s", home);
        else {                                        // relative to the ini file's directory
            std::string dir = ini_path;
            size_t slash = dir.rfind('/');
            dir = slash == std::string::npos ? "." : dir.substr(0, slash);
            snprintf(cfg.home_program, sizeof cfg.home_program, "%s/%s", dir.c_str(), home);
        }
    }
    cfg.home_thread = (int) ini.num(K, "HOME_THREAD", 2);
    cfg.home_timeout = ini.num(K, "HOME_TIMEOUT_S", 120);
    cfg.status_period = ini.num(K, "STATUS_PERIOD_MS", 20) * 1e-3;
    cfg.third_order = ini.num(K, "THIRD_ORDER", 1) != 0;
    cfg.cubic_knots = ini.num(K, "CUBIC_KNOTS", 1) != 0;
    cfg.actuator_limits = ini.num(K, "ACTUATOR_LIMITS", 1) != 0;
    cfg.log_segments = ini.num(K, "LOG_SEGMENTS", 0) != 0;
    cfg.break_angle = ini.num(K, "BREAK_ANGLE", cfg.break_angle);
    cfg.collinear_tol = ini.num(K, "COLLINEAR_TOL", cfg.collinear_tol);
    cfg.corner_tol = ini.num(K, "CORNER_TOL", cfg.corner_tol);
    cfg.facet_angle = ini.num(K, "FACET_ANGLE", cfg.facet_angle);
    cfg.lookahead = ini.num(K, "LOOKAHEAD", cfg.lookahead);
    const char *coords = ini.get("TRAJ", "COORDINATES");
    const char *channels = ini.get(K, "CHANNELS");
    if (channels) sscanf(channels, "%d %d %d %d %d %d %d %d", &cfg.channel[0], &cfg.channel[1], &cfg.channel[2], &cfg.channel[3],
                         &cfg.channel[4], &cfg.channel[5], &cfg.channel[6], &cfg.channel[7]);
    const char letters[] = "XYZABCUV";
    for (int i = 0; i < 8; i++) {
        KmAxisParams &ax = cfg.axis[i];
        if (coords && !strchr(coords, letters[i]) && !strchr(coords, tolower(letters[i]))) continue;
        std::string sec = std::string("AXIS_") + letters[i];
        char jsec[16];
        snprintf(jsec, sizeof jsec, "JOINT_%d", i);     // identity kinematics: joint i is axis i
        ax.counts_per_unit = fabs(ini.num(jsec, "INPUT_SCALE", 0));
        ax.max_vel = ini.num(sec, "MAX_VELOCITY", 0);
        ax.max_accel = ini.num(sec, "MAX_ACCELERATION", 0);
        ax.max_jerk = ini.num(sec, "MAX_JERK", 0);
        if (ax.counts_per_unit <= 0) ax.counts_per_unit = ini.num(K, "DEFAULT_SCALE", 1000);
    }
    return true;
}

// ---- main ----------------------------------------------------------------------------------
static void sighandler(int) { quit = 1; }

static double now_s()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int main(int argc, char *argv[])
{
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-l") && i + 1 < argc) logfile_name = argv[++i];
        else if (!strcmp(argv[i], "-p") && i + 1 < argc) period_s = atof(argv[++i]) * 1e-3;
        else if (!strcmp(argv[i], "-m") && i + 1 < argc) mode = argv[++i];
        else {
            fprintf(stderr, "usage: kmotion-motion [-l LOGFILE] [-p PERIOD_MS] [-m kmotion|standin]\n");
            return 1;
        }
    }
    if (mode != "kmotion" && mode != "standin") {
        fprintf(stderr, "kmotion-motion: -m takes kmotion or standin\n");
        return 1;
    }
    if (period_s < 0.001) period_s = 0.001;

    signal(SIGINT, sighandler);
    signal(SIGQUIT, sighandler);
    signal(SIGTERM, sighandler);
    signal(SIGHUP, sighandler);
    signal(SIGUSR1, SIG_IGN);
    signal(SIGUSR2, SIG_IGN);

    model_init();
    if ((comp_id = hal_init("kmotion-motion")) < 0) {
        fprintf(stderr, "kmotion-motion: hal_init failed: %d\n", comp_id);
        return 1;
    }
    if (create_pins() < 0 || init_comm_buffers() < 0) {
        hal_exit(comp_id);
        return 1;
    }
    KmConfig cfg;
    cfg.queue_limit = QUEUE_LIMIT;
    if (mode == "kmotion" && !read_kmotion_config(cfg)) { hal_exit(comp_id); return 1; }
    if (mode == "kmotion") {
        km = new KmBackend;
        if (!km->init(cfg, m.pos)) {
            fprintf(stderr, "kmotion-motion: the KMotion planner could not be set up\n");
            hal_exit(comp_id);
            return 1;
        }
        log_print("motion: %s; planner %s%s%s; axes", km->mode_name(), cfg.third_order ? "3rd order" : "standard",
                  cfg.third_order && cfg.cubic_knots ? ", cubic knots" : "", cfg.actuator_limits ? ", actuator limits" : ", axis limits");
        for (int i = 0; i < 8; i++)
            if (cfg.axis[i].counts_per_unit > 0 && cfg.axis[i].max_vel > 0)
                log_print(" %c: %g counts, vel %g, accel %g, jerk %g;", "XYZABCUV"[i], cfg.axis[i].counts_per_unit,
                          cfg.axis[i].max_vel, cfg.axis[i].max_accel, cfg.axis[i].max_jerk);
        log_print("\n");
    } else {
        log_print("motion: stand-in model\n");
    }
    update_motion_state();
    update_status();
    update_pins();
    int r = hal_ready(comp_id);
    if (r < 0) {
        fprintf(stderr, "kmotion-motion: hal_ready failed: %d\n", r);
        hal_exit(comp_id);
        return 1;
    }
    log_print("kmotion-motion started, period %.1f ms\n", period_s * 1e3);

    double last = now_s();
    double next = last + period_s;
    while (!quit) {
        double t = now_s();
        double dt = t - last;
        last = t;
        if (dt > 0.1) dt = 0.1;              // after a stall: don't jump the model

        rtapi_mutex_get(&emcmotStruct->command_mutex);
        emcmotStatus->head++;
        if (c->commandNum != emcmotStatus->commandNumEcho) {
            handle_command();
            emcmotStatus->commandEcho = c->command;
            emcmotStatus->commandNumEcho = c->commandNum;
        }
        model_step(dt);
        update_status();
        emcmotStatus->tail = emcmotStatus->head;
        rtapi_mutex_give(&emcmotStruct->command_mutex);
        update_pins();

        next += period_s;
        double sleep_s = next - now_s();
        if (sleep_s > 0) usleep((useconds_t) (sleep_s * 1e6));
        else next = now_s();
    }

    log_print("kmotion-motion stopping\n");
    if (km) {
        // a run in progress ends here: stop the board, give the backend a moment to finish
        // the stop, then take it down - its threads must not outlive the process's
        // static destructors (the process once hung in exit for good). A watchdog ends
        // the process if the teardown hangs anyway.
        std::thread([] { std::this_thread::sleep_for(std::chrono::seconds(3)); _exit(0); }).detach();
        km->abort();
        for (int i = 0; i < 200; i++) {
            km->state(m.ks);
            if (!m.ks.running) break;
            usleep(5000);
        }
        delete km;
        km = NULL;
    }
    rtapi_shmem_delete(shmem_id, comp_id);
    hal_exit(comp_id);
    return 0;
}
