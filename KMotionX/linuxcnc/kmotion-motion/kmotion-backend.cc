/*
 * kmotion-backend.cc - the KMotion side of kmotion-motion (see kmotion-backend.h).
 *
 * Simulate mode: CCoordMotion runs KMotion's real trajectory planner with
 * m_Simulate and m_DoTime set, which plans and times every segment exactly as
 * for a board but downloads nothing. The finalized segments are copied out of the
 * planner's buffer (the same data the TPSegLog holds) and replayed over time here,
 * so LinuxCNC shows the motion the controller would execute, 3rd order knots,
 * blending and all. The only part of CCoordMotion that talks to a board even in
 * simulate mode is FlushSegments, so the flush is composed here from its public
 * pieces instead.
 *
 * Board mode: the same planner downloads to the board through KMotionServer the
 * way KMotionCNC and kmxWeb do; positions, the executing line (ExecTime) and the
 * end of a run (CheckDoneBuf) are polled from the board.
 *
 * Everything that touches CoordMotion runs in the worker thread.
 */
#include "kmotion-backend.h"

#include <atomic>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <strings.h>

#include "GCodeInterpreterX.h"
#include "CoordMotion.h"

static std::mutex g_msg_mutex;
static std::string g_message;                   // operator messages from KMotion
static void err_handler(const char *msg)
{
    std::lock_guard<std::mutex> lock(g_msg_mutex);
    if (!g_message.empty()) g_message += " | ";
    g_message += msg;
}
static int console_handler(const char *msg)     // the board's console output (printf from C programs)
{
    fprintf(stderr, "kmotion-motion: board console: %s", msg);
    return 0;
}

static double now_s()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// a planned segment, as the controller would execute it (the segment log's view)
struct Seg {
    char kind;              // 'L' linear/rapid, 'A' arc, 'D' dwell, 'K' per-axis cubic knot
    int seq;                // our sequence number -> LinuxCNC id
    bool act_frame;         // positions in actuator units (TP3 knots, transformed linears)
    int plane;              // arcs: 0 XY, 1 XZ, 2 YZ (plane-local xc, yc)
    bool ccw;
    double dx, dwell;
    int ntrips;
    double P0[8], P1[8], xc, yc;
    double T[7], A[7], B[7], C[7], D[7];
    double K[32];
    double total;           // duration
};

struct Cmd {
    enum Kind { LINE, ARC, DWELL, ABORT, FEED, SET_POS, MACHINE_ON, JOG_TO, PATH_MODE, HOME, SPINDLE, MCODE, PROBE } kind = LINE;
    double end[9] = {}, center[3] = {}, normal[3] = {};
    double vel = 0, acc = 0, seconds = 0, feed_scale = 0, rapid_scale = 0, tolerance = 0;
    bool rapid = false, on = false;
    int turn = 0, id = 0, axis = 0, term_cond = 0;
    KmHomeRequest home;
    int sp_state = 0;             // SPINDLE: 1 CW, -1 CCW, 0 off
    double sp_rpm = 0;
    bool sp_css = false;          // ... G96, with these
    KmSpindleCss css;
    int mc = 0, ticket = 0;       // MCODE: the M code, its ticket, P and Q
    double p = 0, q = 0;
    bool away = false;            // PROBE: stop when the probe loses contact (G38.4/.5)
};

struct KmBackend::Impl {
    KmConfig cfg;
    CKMotionDLL *km = nullptr;
    CCoordMotion *cm = nullptr;
    double scale[8];                // counts per actuator unit
    bool used[8];

    // command queue, main thread -> worker
    std::mutex qmx;
    std::condition_variable qcv;
    std::deque<Cmd> q;
    std::thread worker;             // plans and downloads (may block on the board's flow control)
    std::thread poller;             // board mode: polls status so the display never freezes
    // urgent board commands from the protocol thread (feed hold, resume, the stop of an
    // abort, jogs): sent in order by a thread of their own, so the protocol loop never
    // waits on the board link (task times a command out after 1 s; the link has stalled
    // for over a second under heavy polling)
    std::thread urgent;
    std::mutex umx;
    std::condition_variable ucv;
    struct Urgent { std::string cmd; int jog_axis; };   // jog_axis >= 0: a counted jog command
    std::deque<Urgent> uq;
    std::mutex wmx;                 // the run bookkeeping below, shared by the two threads
    std::atomic<bool> stop{false};
    std::atomic<bool> paused{false};
    std::atomic<bool> aborting{false}; // abort() requested, the worker has not finished it yet
    std::atomic<int> stop_state{0};    // board: MAIN_STATUS.StopImmediateState from the last status poll
    std::atomic<unsigned> status_count{0};   // board: status polls so far
    std::atomic<int> errors{0};        // planner failures so far (each one stopped the motion)
    std::atomic<bool> homing{false};   // board: the home program is running
    int home_serial = 0;               // under smx
    unsigned home_ok_mask = 0, home_fail_mask = 0;
    std::atomic<int> sp_state{2};      // board: the spindle as last told (1 CW, -1 CCW, 0 off; 2 not known yet:
                                       // a spindle left turning by an earlier session must still get its M5)
    std::atomic<double> sp_rpm{-1};    // ... what its S action got (-1: none yet): the RPM, or in CSS mode the surface speed
    std::atomic<int> sp_css_mode{0};   // board: persist 110 as last written, 1 RPM, 2 CSS (0: not yet)
    std::atomic<bool> sp_at_speed{true};   // poller: the spindle at speed (SPINDLE_AT_SPEED) ...
    std::atomic<int> sp_at_speed_done{0};  // ... in a status read made once this many spindle commands were done
    double at_prev_dest = 0, at_prev_stamp = 0;   // poller, AXIS: the spindle channel's last Dest and its time
    bool at_have_prev = false;
    // G96 as last written to the board (w_css), for the AXIS target: the X counts of radius 0,
    // X's counts per inch, the surface speed in in/s and the maximum RPM
    std::atomic<double> css_xoff{0}, css_k{0}, css_surface{0}, css_max_rpm{0};
    std::atomic<unsigned long long> input_states{0};   // poller: KmConfig.input_bits as last read
    std::atomic<int> mcode_posted{0}, mcode_done{0}, mcode_result{0};   // user M codes: tickets
    std::atomic<bool> probe_contact{false};    // poller: the probe bit at its contact level
    // G38: the probe move is being planned and downloaded (the poller watches for a trip
    // meanwhile), whether it seeks loss of contact, and the trip seen then, the planner told
    // to abort (w_probe)
    std::mutex probe_mx;
    std::atomic<bool> probe_watch{false}, probe_away{false}, probe_trip{false};
    std::atomic<int> spindle_done{0};  // spindle commands carried out or superseded
    std::atomic<double> sp_measured{0};    // the spindle axis's filtered speed, RPM (poller)
    double sp_prev = 0;                    // poller: the last Dest/Position read, and whether there is one
    bool sp_have_prev = false;
    // board jogs, counted per axis: issued (protocol thread), written to the board or dropped
    // (urgent and worker threads), and finished (poller: the count written before a status
    // read that shows the axis done or disabled). A jog runs until finished catches up
    std::atomic<unsigned> jog_issued[8], jog_written[8], jog_finished[8];

    // published state
    std::mutex smx;
    KmState st;
    std::string messages;

    // worker state
    int serial = 0;                 // last sequence number handed to the planner
    int done_upto = 0;              // every sequence number <= this has finished
    std::deque<int> ids;            // LinuxCNC id per sequence number, ids[0] is for done_upto + 1
    bool unflushed = false;         // moves handed over since the last flush
    double last_move_t = 0;
    double last_full_t = 0;         // when task's queue was last seen full (it needs a moment to refill)
    double pos[9];
    double cmd_end[8];              // where the last move handed to the planner ends (its own
                                    // current_* lags one staged chord behind after an arc)
    bool enabled[8];
    double feed_scale = 1.0, rapid_scale = 1.0;
    double current_vel = 0;
    // simulate: replay
    int harvested = 0;              // planner segments copied so far (index into its buffer)
    std::deque<Seg> replay;
    Seg cur;
    bool have_cur = false;
    double t_in_seg = 0;
    double last_tick = 0;
    // board
    bool run_active = false;        // a run is downloaded/executing on the board
    double last_poll = 0;
    bool connected = false;

    int id_of(int seq) const
    {
        int k = seq - done_upto - 1;
        return (k >= 0 && k < (int) ids.size()) ? ids[k] : 0;
    }
    void finish_upto(int seq)
    {
        while (done_upto < seq && !ids.empty()) { ids.pop_front(); done_upto++; }
    }
    void post(const Cmd &c)
    {
        std::lock_guard<std::mutex> lock(qmx);
        q.push_back(c);
        qcv.notify_one();
    }
    void send_urgent(const std::string &cmd, int jog_axis = -1)
    {
        std::lock_guard<std::mutex> lock(umx);
        uq.push_back(Urgent{cmd, jog_axis});
        ucv.notify_one();
    }
    void message(const std::string &m)
    {
        std::lock_guard<std::mutex> lock(smx);
        if (!messages.empty()) messages += " | ";
        messages += m;
    }
};

// ---- geometry of a segment (the same reconstruction MotionLogPlotter does) ------------
static double dist_at(const Seg &s, double t)
{
    for (int i = 0; i < s.ntrips; i++) {
        if (t <= s.T[i] || i == s.ntrips - 1) {
            if (t > s.T[i]) t = s.T[i];
            return ((s.A[i] * t + s.B[i]) * t + s.C[i]) * t + s.D[i];
        }
        t -= s.T[i];
    }
    return s.dx;
}

static void point_at(const Seg &s, double dist, double o[8])
{
    double f = s.dx > 0 ? dist / s.dx : 0;
    if (f < 0) f = 0; else if (f > 1) f = 1;
    for (int i = 0; i < 8; i++) o[i] = s.P0[i] + f * (s.P1[i] - s.P0[i]);
    if (s.kind == 'A') {
        double r0 = hypot(s.P0[0] - s.xc, s.P0[1] - s.yc);
        double r1 = hypot(s.P1[0] - s.xc, s.P1[1] - s.yc);
        double th0 = atan2(s.P0[1] - s.yc, s.P0[0] - s.xc);
        double th1 = atan2(s.P1[1] - s.yc, s.P1[0] - s.xc);
        double dth = th1 - th0;
        if (s.ccw) { if (dth <= 0) dth += 2 * M_PI; }
        else { if (dth >= 0) dth -= 2 * M_PI; }
        // a planned arc longer than the end points suggest: full turns
        if (r0 > 0 && s.dx > fabs(dth) * r0 * 1.5) {
            double turns = floor((s.dx / r0 - fabs(dth)) / (2 * M_PI) + 0.5);
            dth += (s.ccw ? 1 : -1) * turns * 2 * M_PI;
        }
        double r = r0 + f * (r1 - r0);
        double th = th0 + f * dth;
        double lx = s.xc + r * cos(th), ly = s.yc + r * sin(th);
        double lz = s.P0[2] + f * (s.P1[2] - s.P0[2]);
        if (s.plane == 1) { o[2] = lx; o[0] = ly; o[1] = lz; }
        else if (s.plane == 2) { o[1] = lx; o[0] = lz; o[2] = ly; }
        else { o[0] = lx; o[1] = ly; o[2] = lz; }
    }
}

static void eval(const Seg &s, double t, double o[8])
{
    if (s.kind == 'K') {
        if (t < 0) t = 0; else if (t > s.total) t = s.total;
        for (int a = 0; a < 8; a++) {
            const double *k = &s.K[4 * a];
            o[a] = ((k[0] * t + k[1]) * t + k[2]) * t + k[3];
        }
        return;
    }
    if (s.kind == 'D') { memcpy(o, s.P0, sizeof(double) * 8); return; }
    point_at(s, dist_at(s, t), o);
}

// ---- the planner's buffer -> the replay queue (simulate mode) -------------------------
static void copy_segment(KmBackend::Impl *d, SEGMENT *p, Seg &s)
{
    memset(&s, 0, sizeof s);
    s.kind = 'L';
    if (p->type == SEG_ARC) s.kind = 'A';
    else if (p->type == SEG_DWELL || p->nTrips <= 0) s.kind = 'D';
    if (p->Cubic8) s.kind = 'K';
    s.plane = 0;
    if (p->plane == CANON_PLANE_XZ) s.plane = 1;
    else if (p->plane == CANON_PLANE_YZ) s.plane = 2;
    s.ccw = p->DirIsCCW != 0;
    s.seq = p->sequence_number;
    s.dx = std::isfinite(p->dx) ? p->dx : 0;
    s.dwell = (std::isfinite(p->dwell_time) && p->dwell_time > 0) ? p->dwell_time : 0;
    s.ntrips = p->nTrips;
    double P0[8] = {p->x0, p->y0, p->z0, p->a0, p->b0, p->c0, p->u0, p->v0};
    double P1[8] = {p->x1, p->y1, p->z1, p->a1, p->b1, p->c1, p->u1, p->v1};
    s.act_frame = p->ActSpace || p->Cubic8;
    if (!p->ActSpace && !p->Cubic8 && p->type != SEG_ARC) {
        // a legacy segment: the controller interpolates linearly in actuator space
        // between the transformed end points (SegLogSegment does the same)
        double A0[MAX_ACTUATORS], A1[MAX_ACTUATORS];
        if (!d->cm->Kinematics->TransformCADtoActuators(p->x0, p->y0, p->z0, p->a0, p->b0, p->c0, p->u0, p->v0, A0) &&
            !d->cm->Kinematics->TransformCADtoActuators(p->x1, p->y1, p->z1, p->a1, p->b1, p->c1, p->u1, p->v1, A1)) {
            for (int i = 0; i < 8; i++) {
                P0[i] = d->scale[i] != 0 ? A0[i] / d->scale[i] : 0;
                P1[i] = d->scale[i] != 0 ? A1[i] / d->scale[i] : 0;
            }
            s.act_frame = true;
        }
    }
    memcpy(s.P0, P0, sizeof P0);
    memcpy(s.P1, P1, sizeof P1);
    s.xc = p->xc; s.yc = p->yc;
    for (int i = 0; i < 7; i++) {
        s.T[i] = p->C[i].t; s.A[i] = p->C[i].a; s.B[i] = p->C[i].b; s.C[i] = p->C[i].c; s.D[i] = p->C[i].d;
        if (!std::isfinite(s.T[i]) || s.T[i] < 0) s.T[i] = 0;
        if (!std::isfinite(s.A[i])) s.A[i] = 0;
        if (!std::isfinite(s.B[i])) s.B[i] = 0;
        if (!std::isfinite(s.C[i])) s.C[i] = 0;
        if (!std::isfinite(s.D[i])) s.D[i] = 0;
    }
    if (s.kind == 'K') {
        const double *k = SEG_CUBIC8_COEFFS(p);
        for (int j = 0; j < 32; j++) s.K[j] = std::isfinite(k[j]) ? k[j] : 0;
        s.ntrips = 1;
        s.total = s.T[0] > 0 ? s.T[0] : 0;
    } else {
        s.total = 0;
        for (int i = 0; i < s.ntrips && i < 7; i++) s.total += s.T[i];
        if (s.total <= 0) s.total = s.dwell;
    }
}

static void harvest(KmBackend::Impl *d)
{
    if (!d->cfg.simulate) return;
    for (int i = d->harvested; i < d->cm->m_nsegs_downloaded; i++) {
        Seg s;
        copy_segment(d, GetSegPtr(i), s);
        d->replay.push_back(s);
    }
    d->harvested = d->cm->m_nsegs_downloaded;
}

// actuator positions (counts) back to CAD through the kinematics
static void acts_to_cad(KmBackend::Impl *d, const double acts_in[8], double out[9])
{
    double acts[MAX_ACTUATORS];
    for (int i = 0; i < 8; i++) acts[i] = acts_in[i];
    double x, y, z, a, b, c, u, v;
    if (d->cm->Kinematics->TransformActuatorstoCAD(acts, &x, &y, &z, &a, &b, &c, &u, &v) == 0) {
        out[0] = x; out[1] = y; out[2] = z; out[3] = a; out[4] = b; out[5] = c; out[6] = u; out[7] = v;
    }
    out[8] = 0;
}

static void seg_to_cad(KmBackend::Impl *d, const Seg &s, const double in[8], double out[9])
{
    if (s.act_frame) {
        double acts[8];
        for (int i = 0; i < 8; i++) acts[i] = in[i] * d->scale[i];
        acts_to_cad(d, acts, out);
        return;
    }
    for (int i = 0; i < 8; i++) out[i] = in[i];
    out[8] = 0;
}

static void set_cm_position(KmBackend::Impl *d, const double pos[9])
{
    CCoordMotion *cm = d->cm;
    cm->current_x = pos[0]; cm->current_y = pos[1]; cm->current_z = pos[2];
    cm->current_a = pos[3]; cm->current_b = pos[4]; cm->current_c = pos[5];
    cm->current_u = pos[6]; cm->current_v = pos[7];
    memcpy(d->cmd_end, pos, sizeof d->cmd_end);
}

static void w_abort(KmBackend::Impl *d);

// the planner refused a move or a flush. Our own abort (a probe trip's as well) makes it
// return failures too (that is how the worker gets out of a blocking download): those are
// not errors.
// Anything else is one: the planner is left mid-path, so the run is stopped like an
// abort and LinuxCNC gets the message and its motion error flag (what motmod does
// when tpAddLine fails)
static void fail(KmBackend::Impl *d, const char *what, int id = 0)
{
    std::string m;
    {
        std::lock_guard<std::mutex> lock(g_msg_mutex);
        m = g_message;
        g_message.clear();
    }
    if (d->aborting || d->probe_trip) return;
    if (m.empty()) m = std::string(what) + " failed in KMotion's planner";
    if (id > 0) m += " (line " + std::to_string(id) + ")";
    d->message(m);
    d->errors++;
    w_abort(d);
}

// ---- worker: planning ----------------------------------------------------------------
static int begin_move(KmBackend::Impl *d, int id)
{
    std::lock_guard<std::mutex> lk(d->wmx);
    int seq = ++d->serial;
    d->ids.push_back(id);
    d->unflushed = true;
    d->last_move_t = now_s();
    if (!d->cfg.simulate) d->run_active = true;
    return seq;
}

static void w_line(KmBackend::Impl *d, const Cmd &c)
{
    int seq = begin_move(d, c.id);
    // the planner's ID argument is its cutter-compensation hint (1/2/3), not a line number
    int r = d->cm->StraightFeedAccelRapid(c.vel, c.acc, c.rapid, true, c.end[0], c.end[1], c.end[2], c.end[3], c.end[4], c.end[5], c.end[6], c.end[7], seq, 0);
    if (!r) memcpy(d->cmd_end, c.end, sizeof d->cmd_end);
    harvest(d);
    if (r) fail(d, "StraightFeed", c.id);
}

static void w_arc(KmBackend::Impl *d, const Cmd &c)
{
    // LinuxCNC's center/normal/turn back to rs274ngc's canon arc, which KMotion's
    // ArcFeed takes. The plane from the normal's dominant axis; (first, second | helix)
    // axes in the plane's canonical order: XY (x, y | z), XZ (z, x | y), YZ (y, z | x).
    // The direction as posemath's pmCircleInit reads it: counterclockwise about the
    // normal with `turn` extra full turns when turn >= 0; turn < 0 flips the normal
    // (clockwise) and means -1 - turn extra turns. The planner takes a plain CCW flag
    // (its own canon passes rotation == 1) and knows no multiple turns: those become
    // half circles ahead of the final arc, the helix and the other axes advancing in
    // proportion to the angle. Each piece gets its own planner sequence number, like
    // G-code lines (the planner kept only one of two identical pieces under one number);
    // they all map to the one LinuxCNC id.
    int plane_axis = 2;
    if (fabs(c.normal[1]) > fabs(c.normal[plane_axis])) plane_axis = 1;
    if (fabs(c.normal[0]) > fabs(c.normal[plane_axis])) plane_axis = 0;
    CANON_PLANE plane;
    int i1, i2, i3;
    if (plane_axis == 2) { plane = CANON_PLANE_XY; i1 = 0; i2 = 1; i3 = 2; }
    else if (plane_axis == 1) { plane = CANON_PLANE_XZ; i1 = 2; i2 = 0; i3 = 1; }
    else { plane = CANON_PLANE_YZ; i1 = 1; i2 = 2; i3 = 0; }
    bool ccw = c.normal[plane_axis] >= 0;
    int extra = c.turn;
    if (c.turn < 0) { ccw = !ccw; extra = -1 - c.turn; }
    CCoordMotion *cm = d->cm;
    double cur[8];
    memcpy(cur, d->cmd_end, sizeof cur);        // not cm->current_*: see cmd_end
    int r = 0;
    if (extra > 0) {
        // the final arc's angle as the planner will see it: (0, 2 pi] CCW, [-2 pi, 0) CW
        double t0 = atan2(cur[i2] - c.center[i2], cur[i1] - c.center[i1]);
        double t1 = atan2(c.end[i2] - c.center[i2], c.end[i1] - c.center[i1]);
        double dt = t1 - t0;
        if (fabs(dt) < 1e-9) dt = 0;
        if (ccw) { if (dt <= 0) dt += 2 * M_PI; } else { if (dt >= 0) dt -= 2 * M_PI; }
        double total = fabs(dt) + 2 * M_PI * extra;
        double opp1 = 2 * c.center[i1] - cur[i1], opp2 = 2 * c.center[i2] - cur[i2];   // across the circle
        for (int k = 1; k <= 2 * extra && r == 0; k++) {
            double f = M_PI * k / total;            // fraction of the whole motion after k half circles
            double mid[8];
            for (int i = 0; i < 8; i++) mid[i] = cur[i] + (c.end[i] - cur[i]) * f;
            bool back = (k % 2) == 0;
            int seq = begin_move(d, c.id);
            r = cm->ArcFeedAccel(c.vel, c.acc, plane, back ? cur[i1] : opp1, back ? cur[i2] : opp2, c.center[i1], c.center[i2], ccw ? 1 : 0, mid[i3],
                                 mid[3], mid[4], mid[5], mid[6], mid[7], seq, 0);
        }
    }
    if (r == 0) {
        int seq = begin_move(d, c.id);
        r = cm->ArcFeedAccel(c.vel, c.acc, plane, c.end[i1], c.end[i2], c.center[i1], c.center[i2], ccw ? 1 : 0, c.end[i3],
                             c.end[3], c.end[4], c.end[5], c.end[6], c.end[7], seq, 0);
    }
    if (!r) memcpy(d->cmd_end, c.end, sizeof d->cmd_end);
    harvest(d);
    if (r) fail(d, "ArcFeed", c.id);
}

static void w_dwell(KmBackend::Impl *d, const Cmd &c)
{
    int seq = begin_move(d, c.id);
    int r = d->cm->Dwell(c.seconds, seq);
    harvest(d);
    if (r) fail(d, "Dwell", c.id);
}

static void w_abort(KmBackend::Impl *d);

static void w_flush(KmBackend::Impl *d)
{
    CCoordMotion *cm = d->cm;
    {
        std::lock_guard<std::mutex> lk(d->wmx);
        if (!d->unflushed) return;
        d->unflushed = false;
    }
    if (!d->cfg.simulate) {
        // the real thing: finishes the plan, downloads the rest, starts the buffer
        double t0 = now_s();
        if (getenv("KM_DEBUG_STOP")) fprintf(stderr, "kmotion-motion: FlushSegments begins (%d segments planned)\n", nsegs);
        int fr = cm->FlushSegments();
        if (getenv("KM_DEBUG_STOP")) fprintf(stderr, "kmotion-motion: FlushSegments returned %d after %.3f s\n", fr, now_s() - t0);
        if (fr) {
            if (cm->m_AxisDisabled) { d->message("an axis of the coordinate system is disabled: motion refused"); cm->m_AxisDisabled = false; }
            fail(d, "FlushSegments");   // stops the run; the moves are lost and LinuxCNC's queue drains
        }
        return;
    }
    // what FlushSegments does, minus the board: finish a 3rd order streaming run,
    // finalize the remaining segments, "download" them (into the planner's buffer only),
    // then reset for the next run
    if (cm->Kinematics->m_MotionParams.ThirdOrderTP && cm->TP3FlushRun("linuxcnc flush")) { fail(d, "TP3FlushRun"); return; }
    MaximizeSegments();
    for (int iseg = cm->m_nsegs_downloaded; iseg < nsegs; iseg++)
        if (cm->OutputSegment(iseg)) { fail(d, "OutputSegment"); return; }
    harvest(d);
    tp_init();
    cm->TP3ClearRun();
    cm->DownloadInit();
    d->harvested = 0;
}

static int board_query(KmBackend::Impl *d, const char *cmd, char *reply);

// bring the board to a stop the way KMotion's own Halt does (CCoordMotion::
// CheckMotionHalt): StopImmediate0 is the feed hold (every axis decelerates within its
// limits), GetStopState says when it has come to rest, StopImmediate2 then clears the
// stop state without resuming and the rest of the buffer is abandoned. StopImmediate2
// on its own does nothing to a moving board.
static void board_stop(KmBackend::Impl *d)
{
    char reply[MAX_LINE + 1];
    unsigned c0 = d->status_count;
    if (d->km->WriteLine("StopImmediate0")) d->message("StopImmediate0 failed");   // abort() has queued one too: harmless twice
    double t0 = now_s(), t_check = t0;
    bool was_moving = false;
    int last_state = -1;
    // the stop state comes with the status poll (MAIN_STATUS.StopImmediateState): polling
    // GetStopState here starved the status thread, and the display froze for the whole
    // stop and then jumped to the rest position
    for (;;) {
        int state = d->stop_state;
        if (d->status_count >= c0 + 2) {                  // two polls after the hold went out
            if (getenv("KM_DEBUG_STOP") && state != last_state) { fprintf(stderr, "kmotion-motion: stop state %d at %.3f s\n", state, now_s() - t0); last_state = state; }
            if (state != 1 && state != 2) break;          // at rest, or nothing was moving
            was_moving = true;
            // the buffer may run out while the hold is still ramping: at rest as well
            if (state == 1 && now_s() - t_check > 0.25) {
                t_check = now_s();
                if (board_query(d, "CheckDoneBuf", reply) == 0 && strcmp(reply, "0") != 0) break;
            }
        }
        if (now_s() - t0 > 10) { d->message("the board did not come to a stop within 10 s"); break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (d->km->WriteLine("StopImmediate2")) d->message("StopImmediate2 failed");
    if (was_moving) fprintf(stderr, "kmotion-motion: board stopped (%.2f s)\n", now_s() - t0);
}

static void w_abort(KmBackend::Impl *d)
{
    CCoordMotion *cm = d->cm;
    if (!d->cfg.simulate) board_stop(d);
    cm->SetAbort();
    cm->ClearAbort();                 // re-initializes the planner
    cm->ClearHalt();
    d->replay.clear();
    d->have_cur = false;
    d->t_in_seg = 0;
    d->harvested = 0;
    {
        std::lock_guard<std::mutex> lk(d->wmx);
        d->unflushed = false;
        d->run_active = false;
        d->ids.clear();
        d->done_upto = d->serial;
    }
    d->current_vel = 0;
    if (d->cfg.simulate) {
        set_cm_position(d, d->pos);
    } else {
        cm->RearmCoordLaunch();
        double x, y, z, a, b, c, u, v;
        if (cm->ReadCurAbsPosition(&x, &y, &z, &a, &b, &c, &u, &v) == 0) {
            double p[9] = {x, y, z, a, b, c, u, v, 0};
            memcpy(d->pos, p, sizeof p);
        }
        cm->ClearAbort();
        // the planner's current position is the end of the last move it was given, up to
        // the lookahead ahead of the board; the next move must start where the board
        // stands or the board jumps there first (seen: a 0.35 in leap at the restart)
        set_cm_position(d, d->pos);
    }
    memcpy(d->cmd_end, d->pos, sizeof d->cmd_end);
    d->aborting = false;
}

static void w_replay(KmBackend::Impl *d, double dt)
{
    double before[9];
    memcpy(before, d->pos, sizeof before);
    d->current_vel = 0;
    if (!d->paused) {
        // the board's feed override scales the whole coordinated motion's clock
        double adv = dt * (d->feed_scale > 0 ? d->feed_scale : 0);
        while (adv > 0) {
            if (!d->have_cur) {
                if (d->replay.empty()) break;
                d->cur = d->replay.front();
                d->replay.pop_front();
                d->have_cur = true;
                d->t_in_seg = 0;
                // a move spans many planned segments (hundreds of knots): it is finished
                // once a segment of a later move starts
                std::lock_guard<std::mutex> lk(d->wmx);
                d->finish_upto(d->cur.seq - 1);
            }
            double left = d->cur.total - d->t_in_seg;
            if (adv >= left) {
                adv -= left;
                double o[8];
                eval(d->cur, d->cur.total, o);
                seg_to_cad(d, d->cur, o, d->pos);
                d->have_cur = false;
            } else {
                d->t_in_seg += adv;
                adv = 0;
                double o[8];
                eval(d->cur, d->t_in_seg, o);
                seg_to_cad(d, d->cur, o, d->pos);
            }
        }
        double dd = 0;
        for (int i = 0; i < 3; i++) dd += (d->pos[i] - before[i]) * (d->pos[i] - before[i]);
        d->current_vel = dt > 0 ? sqrt(dd) / dt : 0;
    }
    std::lock_guard<std::mutex> lk(d->wmx);
    if (!d->have_cur && d->replay.empty() && !d->unflushed) d->finish_upto(d->serial);
}

// ---- worker: the board ------------------------------------------------------------------
static int board_query(KmBackend::Impl *d, const char *cmd, char *reply)
{
    reply[0] = 0;
    return d->km->WriteLineReadLine(cmd, reply);
}

// CoordMotion::UpdateRealTimeState is private: the same walk over the executing
// buffer, from the public planner globals - which downloaded segment the board's
// execution time T falls into, hence the sequence number being executed
static bool realtime_sequence(double T, int &seq)
{
    if (T < 0.0) return false;                      // nothing executing
    SEGMENT *segs = segments_executing;
    if (segs != segments0 && segs != segments1) return false;
    int index = (segs == segments0) ? 0 : 1;
    double BufTime = SegsDoneTime[index];
    if (SegsDone[index] == -1) {
        if (special_cmds_initial_sequence_no[index] >= 0) { seq = special_cmds_initial_sequence_no[index]; return true; }
        return false;
    }
    int i;
    for (i = SegsDone[index]; i >= 0; i--) {
        for (int k = segs[TPMOD(i)].nTrips - 1; k >= 0; k--) {
            if (BufTime <= T) break;
            BufTime -= segs[TPMOD(i)].C[k].t;
        }
        if (BufTime <= T || (T == 0.0 && BufTime < 1e-6)) break;
    }
    if (i < 0) return false;
    seq = segs[TPMOD(i)].sequence_number;
    return true;
}

// an I/O bit's state as MAIN_STATUS carries it, KMotion's numbering (KMotion.exe's Digital I/O
// pages read them the same way): 1 or 0, or -1 for a bit the status does not carry
static int status_bit(const MAIN_STATUS &st, int b)
{
    auto bit = [](int word, int n) { return (word >> n) & 1; };
    if (b >= 0 && b < 64) return bit(st.BitsState[b / 32], b % 32);              // board bits 0-47, virtual 48-63
    if (b >= 64 && b < 96) return bit(st.SnapBitsState0, b - 64);                 // SnapAmp 0
    if (b >= 96 && b < 128) return bit(st.SnapBitsState1, b - 96);                // SnapAmp 1
    if (b >= 128 && b < 144) return bit(st.KanalogBitsStateInputs, b - 128);      // Kanalog inputs
    if (b >= 144 && b < 168) return bit(st.KanalogBitsStateOutputs, b - 144);     // Kanalog outputs
    if (b >= 168 && b < 184) return bit(st.VirtualBits, b - 168 + 16);            // the second 16 virtual bits
    if (b >= 200 && b < 290) return bit(st.BitsState200[(b - 200) / 32], (b - 200) % 32);   // Kogna 200-289
    if (b >= 1024 && b < 1056) return bit(st.VirtualBitsEx0, b - 1024);           // the status carries 1024-1055
    return -1;
}

// what the spindle was told, in RPM (-1: nothing yet): its S, or in G96 what the board's
// ServiceCSS() makes of X's position (the radius from the X counts of radius 0, capped)
static double spindle_target_rpm(KmBackend::Impl *d, const MAIN_STATUS &status)
{
    int state = d->sp_state;
    if (state == 2) return -1;
    if (state == 0) return 0;
    if (d->sp_css_mode != 2) return d->sp_rpm;
    int xch = d->cfg.channel[0];
    double k = d->css_k, max_rpm = d->css_max_rpm;
    if (xch < 0 || xch >= N_CHANNELS_KOGNA || k <= 0) return -1;
    double radius = fabs((status.Dest[xch] - d->css_xoff) / k);
    double rpm = radius > 0 ? d->css_surface * 60.0 / (2 * M_PI * radius) : max_rpm;
    return rpm < max_rpm ? rpm : max_rpm;
}

bool km_status_has_bit(int bit)
{
    MAIN_STATUS z;
    memset(&z, 0, sizeof z);
    return status_bit(z, bit) >= 0;
}

// the probe watcher's persist variables (configs/kmotion-kogna/probe/ProbeService.c)
enum { P_ARM = 69, P_STATUS = 70, P_BIT = 71, P_LEVEL = 72, P_AWAY = 73, P_POS = 74 };

static void w_poll_board(KmBackend::Impl *d)
{
    CCoordMotion *cm = d->cm;
    MAIN_STATUS status;
    memset(&status, 0, sizeof status);
    unsigned jog_written[8];                 // jogs this status read comes after
    for (int i = 0; i < 8; i++) jog_written[i] = d->jog_written[i];
    const int spindle_done = d->spindle_done;    // ... and spindle commands
    if (d->km->WaitToken(false, 100, "kmotion-motion") != KMOTION_LOCKED) {
        d->connected = false;
        return;
    }
    int r = d->km->GetStatus(status, false);
    d->km->ReleaseToken();
    {
        static double last_ok = 0;
        double now = now_s();
        if (getenv("KM_DEBUG_STOP") && last_ok > 0 && now - last_ok > 0.2) fprintf(stderr, "kmotion-motion: status gap %.3f s\n", now - last_ok);
        last_ok = now;
    }
    if (r) {
        if (d->connected) d->message("lost the board: GetStatus failed");
        d->connected = false;
        return;
    }
    d->connected = true;
    d->stop_state = status.StopImmediateState;
    d->status_count++;
    // positions: the commanded destinations (open-loop machines report no other position)
    double acts[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double before[9];
    memcpy(before, d->pos, sizeof before);
    for (int i = 0; i < 8; i++) {
        int ch = d->cfg.channel[i];
        if (ch >= 0 && ch < N_CHANNELS_KOGNA) {
            acts[i] = status.Dest[ch];
            d->enabled[i] = (status.Enables >> ch) & 1;
        } else {
            d->enabled[i] = true;
        }
    }
    acts_to_cad(d, acts, d->pos);
    // the spindle at speed (SPINDLE_AT_SPEED): its bit at the level, or its axis done (a jog at
    // speed has no trajectory left; a disabled axis is not done)
    if (d->cfg.spindle_at_speed == KM_AT_SPEED_BIT) {
        d->sp_at_speed = status_bit(status, d->cfg.spindle_at_speed_bit) == d->cfg.spindle_at_speed_level;
    } else if (d->cfg.spindle_at_speed == KM_AT_SPEED_AXIS) {
        // a jogged spindle at its commanded speed, as Dynomotion's OnCWJogWait.c waits for it
        // (on the channel's velocity): its Dest's speed since the last read, on the board's own
        // clock, within 1% (at least 1 RPM) of the target; nothing told yet: no wait
        int ch = d->cfg.spindle_at_speed_axis;
        double cpr = d->cfg.spindle_counts_per_rev;
        bool at = false;
        if (ch >= 0 && ch < N_CHANNELS_KOGNA && cpr > 0) {
            double p = status.Dest[ch], dt = status.TimeStamp - d->at_prev_stamp;
            if (d->at_have_prev && dt > 0.001 && dt < 1.0) {
                double rpm = spindle_target_rpm(d, status);
                double v = fabs(p - d->at_prev_dest) / dt, target = fabs(rpm) * cpr / 60.0;
                double tol = std::max(0.01 * target, cpr / 60.0);
                at = rpm < 0 || fabs(v - target) <= tol;
            }
            d->at_prev_dest = p;
            d->at_prev_stamp = status.TimeStamp;
            d->at_have_prev = true;
        }
        d->sp_at_speed = at;
    }
    d->sp_at_speed_done = spindle_done;
    unsigned long long in = 0;
    for (size_t i = 0; i < d->cfg.input_bits.size() && i < 64; i++)
        if (status_bit(status, d->cfg.input_bits[i]) == 1) in |= 1ull << i;
    d->input_states = in;
    if (d->cfg.probe_bit >= 0) d->probe_contact = status_bit(status, d->cfg.probe_bit) == d->cfg.probe_level;
    // a probe trip while the worker still plans or downloads the probe move: the watcher's
    // feed hold freezes the board's execution, and the planner's download pacing would wait
    // for it forever. Its abort flag gets the worker out; w_probe then finishes the stop.
    // Asked when the board is held or the probe is in the state sought (a trip before the
    // buffer started holds nothing)
    if (d->probe_watch && (status.StopImmediateState != 0 || d->probe_contact != d->probe_away)) {
        char cmd[32], reply[MAX_LINE + 1];
        snprintf(cmd, sizeof cmd, "GetPersistDec %d", P_STATUS);
        if (board_query(d, cmd, reply) == 0 && atoi(reply) == 2) {
            std::lock_guard<std::mutex> lk(d->probe_mx);
            if (d->probe_watch) { d->probe_trip = true; cm->SetAbort(); }
        }
    }
    // jogs: an axis with no trajectory (or disabled) has finished every jog written before this read
    for (int i = 0; i < 8; i++) {
        int ch = d->cfg.channel[i];
        if (ch < 0 || ch >= N_CHANNELS_KOGNA || ((status.AxisDone >> ch) & 1) || !((status.Enables >> ch) & 1))
            d->jog_finished[i] = jog_written[i];
    }
    // speed from the board's own clock between two status reads (the host-side interval
    // includes the query latency and reads low)
    static double last_stamp = 0;
    double dt = status.TimeStamp - last_stamp;
    last_stamp = status.TimeStamp;
    double dd = 0;
    for (int i = 0; i < 3; i++) dd += (d->pos[i] - before[i]) * (d->pos[i] - before[i]);
    if (dt > 0.001 && dt < 1.0) d->current_vel = sqrt(dd) / dt;
    // the spindle's speed from its axis channel over the same interval: Dest (what a jogged
    // spindle is told, ramps included) or Position (its encoder), low-pass filtered. A jump far
    // beyond any spindle (Zero, EnableAxisDest) is no speed and is skipped
    if (d->cfg.spindle_speed_axis >= 0 && d->cfg.spindle_speed_axis < N_CHANNELS_KOGNA && d->cfg.spindle_counts_per_rev > 0) {
        int ch = d->cfg.spindle_speed_axis;
        double p = d->cfg.spindle_speed_from_dest ? status.Dest[ch] : status.Position[ch];
        if (d->sp_have_prev && dt > 0.001 && dt < 1.0) {
            double rpm = (p - d->sp_prev) / dt / d->cfg.spindle_counts_per_rev * 60.0;
            if (fabs(rpm) < 100000) {
                double tau = d->cfg.spindle_speed_tau;
                double a = tau > 0 ? 1.0 - exp(-dt / tau) : 1.0;
                d->sp_measured = d->sp_measured + (rpm - d->sp_measured) * a;
            }
        }
        d->sp_prev = p;
        d->sp_have_prev = true;
    }

    bool active, flushed;
    {
        std::lock_guard<std::mutex> lk(d->wmx);
        active = d->run_active;
        flushed = !d->unflushed;
    }
    if (active && cm->CoordLaunched()) {
        char reply[MAX_LINE + 1];
        // where the board is in the downloaded buffer: the executing line
        if (board_query(d, "ExecTime", reply) == 0) {
            double T;
            int seq;
            if (sscanf(reply, "%lf", &T) == 1 && realtime_sequence(T, seq)) {
                std::lock_guard<std::mutex> lk(d->wmx);
                if (seq > d->done_upto && seq <= d->serial) d->finish_upto(seq - 1);
            }
        }
        if (flushed && board_query(d, "CheckDoneBuf", reply) == 0) {
            if (strcmp(reply, "1") == 0) {
                // the buffer ran to its end: the run is complete
                std::lock_guard<std::mutex> lk(d->wmx);
                if (!d->unflushed) {
                    d->finish_upto(d->serial);
                    d->run_active = false;
                    cm->RearmCoordLaunch();
                }
            } else if (strcmp(reply, "-1") == 0) {
                d->message("an axis is disabled: motion aborted");
                d->errors++;
                d->aborting = true;
                cm->SetAbort();
                Cmd c{};
                c.kind = Cmd::ABORT;
                d->post(c);
            }
        }
    }
}

// what the main thread reads
static void publish(KmBackend::Impl *d)
{
    std::lock_guard<std::mutex> lock(d->smx);
    std::lock_guard<std::mutex> lk(d->wmx);
    KmState &s = d->st;
    memcpy(s.pos, d->pos, sizeof s.pos);
    for (int i = 0; i < 8; i++) s.enabled[i] = d->cfg.simulate ? true : d->enabled[i];
    s.depth = 0;                                  // LinuxCNC moves pending (a multi-turn arc is several sequence numbers)
    for (size_t k = 0; k < d->ids.size(); k++) if (k == 0 || d->ids[k] != d->ids[k - 1]) s.depth++;
    s.running = d->unflushed || d->run_active || d->have_cur || !d->replay.empty() || d->aborting;
    s.errors = d->errors;
    s.homing = d->homing;
    s.home_serial = d->home_serial;
    s.home_ok_mask = d->home_ok_mask;
    s.home_fail_mask = d->home_fail_mask;
    s.spindle_state = d->sp_state;
    s.spindle_rpm = d->sp_rpm;
    s.spindle_css = d->sp_css_mode == 2;
    s.spindle_done = d->spindle_done;
    s.spindle_rpm_measured = d->sp_measured;
    s.active_id = s.depth > 0 ? d->id_of(d->done_upto + 1) : 0;
    s.paused = d->paused;
    s.connected = d->cfg.simulate ? true : d->connected;
    s.current_vel = d->current_vel;
    s.distance_to_go = (d->cfg.simulate && d->have_cur && d->cur.kind != 'K' && d->cur.total > 0)
                       ? d->cur.dx * (1 - d->t_in_seg / d->cur.total) : 0;
    std::lock_guard<std::mutex> lock2(g_msg_mutex);
    if (!g_message.empty()) {
        if (!d->messages.empty()) d->messages += " | ";
        d->messages += g_message;
        g_message.clear();
    }
}

// board mode: status at its own pace, whatever the planner is waiting on
static void urgent_main(KmBackend::Impl *d)
{
    while (!d->stop) {
        std::string cmd;
        int jog_axis;
        {
            std::unique_lock<std::mutex> lock(d->umx);
            if (d->uq.empty()) { d->ucv.wait_for(lock, std::chrono::milliseconds(20)); continue; }
            cmd = d->uq.front().cmd;
            jog_axis = d->uq.front().jog_axis;
            d->uq.pop_front();
        }
        double t0 = now_s();
        if (d->km->WriteLine(cmd.c_str())) d->message(std::string("board command failed: ") + cmd);
        if (jog_axis >= 0) d->jog_written[jog_axis]++;
        if (getenv("KM_DEBUG_STOP") && cmd.compare(0, 13, "StopImmediate") == 0)
            fprintf(stderr, "kmotion-motion: %s sent, took %.3f s\n", cmd.c_str(), now_s() - t0);
    }
}

static void poller_main(KmBackend::Impl *d)
{
    while (!d->stop) {
        double t = now_s();
        w_poll_board(d);
        d->last_poll = t;
        publish(d);
        double sleep_s = d->cfg.status_period - (now_s() - t);
        if (sleep_s > 0) std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
    }
}

static int run_program(KmBackend::Impl *d, const char *path, int thread)
{
    char err[512] = "";
    if (d->km->CompileAndLoadCoff(path, thread, err, sizeof err - 1)) {
        d->message(std::string("init program failed: ") + path + ": " + err);
        return 1;
    }
    char cmd[64], reply[MAX_LINE + 1];
    snprintf(cmd, sizeof cmd, "Execute%d", thread);
    if (d->km->WriteLine(cmd)) { d->message("Execute failed for the init program"); return 1; }
    snprintf(cmd, sizeof cmd, "CheckThread%d", thread);
    double t0 = now_s();
    for (;;) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (board_query(d, cmd, reply)) { d->message("CheckThread failed"); return 1; }
        if (strcmp(reply, "0") == 0) return 0;
        if (now_s() - t0 > 30) { d->message("the init program did not finish within 30 s"); return 1; }
    }
}

// a program that is to keep running (START_PROGRAM): compiled (a .out is loaded as it is) into
// its thread, which stops whatever ran there, and executed; not waited for
static int start_program(KmBackend::Impl *d, const char *path, int thread)
{
    size_t n = strlen(path);
    if (n > 4 && !strcasecmp(path + n - 4, ".out")) {
        if (d->km->LoadCoff(thread, path, 0)) { d->message(std::string("start program failed: loading ") + path); return 1; }
    } else {
        char err[512] = "";
        if (d->km->CompileAndLoadCoff(path, thread, err, sizeof err - 1)) {
            d->message(std::string("start program failed: ") + path + ": " + err);
            return 1;
        }
    }
    char cmd[64];
    snprintf(cmd, sizeof cmd, "Execute%d", thread);
    if (d->km->WriteLine(cmd)) { d->message(std::string("board command failed: ") + cmd); return 1; }
    return 0;
}

static void w_machine_on(KmBackend::Impl *d, bool on)
{
    char cmd[64];
    MAIN_STATUS status;
    memset(&status, 0, sizeof status);
    bool have_status = d->km->GetStatus(status, true) == 0;
    for (int i = 0; i < 8; i++) {
        int ch = d->cfg.channel[i];
        if (ch < 0) continue;
        if (on) {
            // enable at the current destination: no jump for open-loop axes
            if (have_status) snprintf(cmd, sizeof cmd, "EnableAxisDest%d %.0f", ch, status.Dest[ch]);   // console form: no "="
            else snprintf(cmd, sizeof cmd, "EnableAxis%d", ch);
        } else {
            snprintf(cmd, sizeof cmd, "DisableAxis%d", ch);
        }
        if (d->km->WriteLine(cmd)) { d->message(std::string("board command failed: ") + cmd); return; }
    }
}

static void w_jog_to(KmBackend::Impl *d, const Cmd &c)
{
    // an incremental or absolute jog: a move of the one actuator to the target counts, at
    // LinuxCNC's jog speed (with none, the board's own Vel)
    if (c.axis < 0 || c.axis >= 8) return;
    int ch = d->cfg.channel[c.axis];
    if (ch < 0) return;
    double target[9];
    memcpy(target, d->pos, sizeof target);
    target[c.axis] = c.end[c.axis];
    double acts[MAX_ACTUATORS];
    if (d->cm->Kinematics->TransformCADtoActuators(target[0], target[1], target[2], target[3], target[4], target[5], target[6], target[7], acts)) return;
    char cmd[64];
    double vel = fabs(c.vel * d->scale[c.axis]);
    if (vel > 0) snprintf(cmd, sizeof cmd, "MoveAtVel%d=%.3f %.3f", ch, acts[c.axis], vel);
    else snprintf(cmd, sizeof cmd, "Move%d=%.3f", ch, acts[c.axis]);
    if (d->km->WriteLine(cmd)) d->message(std::string("board command failed: ") + cmd);
}

// ---- worker loop --------------------------------------------------------------------------
// homing through the board: LinuxCNC's parameters go to the program through the persist
// array (layout in Home.c), the program runs on its thread, and when it is done the
// planner is set to where the board stands. Aborts kill the thread and stop the jogs.
static int set_persist(KmBackend::Impl *d, int index, long value)
{
    char cmd[64];
    snprintf(cmd, sizeof cmd, "SetPersistDec %d %ld", index, value);
    if (d->km->WriteLine(cmd)) { d->message(std::string("board command failed: ") + cmd); return 1; }
    return 0;
}

static void w_home(KmBackend::Impl *d, const KmHomeRequest &req)
{
    enum { P_MASK = 120, P_OK = 121, P_FAILED = 122, P_CURRENT = 123, P_JOINT = 128, P_STRIDE = 8 };
    unsigned chmask = 0;                     // by board channel
    unsigned jmask = 0;                      // by LinuxCNC joint
    int ch_of[8];
    for (int j = 0; j < 8; j++) {
        ch_of[j] = d->cfg.channel[j];
        if (!req.joint[j].home || ch_of[j] < 0 || ch_of[j] >= 8) continue;
        const KmHomeJoint &hj = req.joint[j];
        double k = d->scale[j];
        int base = P_JOINT + P_STRIDE * ch_of[j];
        int flags = (hj.use_index ? 1 : 0) | (hj.no_final_move ? 4 : 0);
        if (set_persist(d, base + 0, lround(hj.search_vel * k)) || set_persist(d, base + 1, lround(hj.latch_vel * k)) ||
            set_persist(d, base + 2, lround(hj.offset * k)) || set_persist(d, base + 3, lround(hj.home_pos * k)) ||
            set_persist(d, base + 4, lround(fabs(hj.final_vel) * k)) || set_persist(d, base + 5, flags) ||
            set_persist(d, base + 6, lround(fabs(hj.max_travel) * k)) || set_persist(d, base + 7, hj.sequence)) {
            jmask |= 1u << j;
            goto failed_all;
        }
        chmask |= 1u << ch_of[j];
        jmask |= 1u << j;
    }
    if (chmask == 0) return;
    if (set_persist(d, P_OK, 0) || set_persist(d, P_FAILED, 0) || set_persist(d, P_CURRENT, -1) || set_persist(d, P_MASK, (long) chmask)) goto failed_all;
    {
        d->homing = true;
        char err[512] = "";
        char cmd[64], reply[MAX_LINE + 1];
        int thread = d->cfg.home_thread;
        if (d->km->CompileAndLoadCoff(d->cfg.home_program, thread, err, sizeof err - 1)) {
            d->message(std::string("home program failed: ") + d->cfg.home_program + ": " + err);
            goto finish_failed;
        }
        snprintf(cmd, sizeof cmd, "Execute%d", thread);
        if (d->km->WriteLine(cmd)) { d->message("Execute failed for the home program"); goto finish_failed; }
        snprintf(cmd, sizeof cmd, "CheckThread%d", thread);
        double t0 = now_s();
        bool killed = false;
        for (;;) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (d->aborting || d->stop) {
                killed = true;
                d->message("homing aborted");
            } else if (now_s() - t0 > d->cfg.home_timeout) {
                killed = true;
                d->message("the home program did not finish in time");
            }
            if (killed) {
                snprintf(cmd, sizeof cmd, "Kill%d", thread);
                d->km->WriteLine(cmd);
                for (int ch = 0; ch < 8; ch++)
                    if (chmask & (1u << ch)) { snprintf(cmd, sizeof cmd, "Jog%d=0", ch); d->km->WriteLine(cmd); }
                break;
            }
            if (board_query(d, cmd, reply)) { d->message("CheckThread failed"); killed = true; break; }
            if (strcmp(reply, "0") == 0) break;
        }
        unsigned ok_ch = 0;
        if (!killed) {
            snprintf(cmd, sizeof cmd, "GetPersistDec %d", (int) P_OK);
            if (board_query(d, cmd, reply) == 0) ok_ch = (unsigned) strtol(reply, NULL, 10);
        }
        // where the board stands now is where the planner continues from
        double x, y, z, a, bb, c, u, v;
        if (d->cm->ReadCurAbsPosition(&x, &y, &z, &a, &bb, &c, &u, &v) == 0) {
            double p[9] = {x, y, z, a, bb, c, u, v, 0};
            memcpy(d->pos, p, sizeof p);
        } else {
            d->cm->ClearAbort();
        }
        set_cm_position(d, d->pos);
        unsigned ok_j = 0;
        for (int j = 0; j < 8; j++)
            if ((jmask & (1u << j)) && ch_of[j] >= 0 && (ok_ch & (1u << ch_of[j]))) ok_j |= 1u << j;
        {
            std::lock_guard<std::mutex> lock(d->smx);
            d->home_ok_mask = ok_j;
            d->home_fail_mask = jmask & ~ok_j;
            d->home_serial++;
        }
        d->homing = false;
        return;
    }
finish_failed:
    d->homing = false;
failed_all:
    {
        std::lock_guard<std::mutex> lock(d->smx);
        d->home_ok_mask = 0;
        d->home_fail_mask = jmask;
        d->home_serial++;
    }
}

// ---- worker: spindle ------------------------------------------------------------------
// LinuxCNC's spindle through KMotionCNC's M3, M4, M5 and S actions ([KMOTION] SPINDLE_*).
// One action, carried out as the G-code interpreter's InvokeActionDirect does: bits and DACs
// at once (the board is idle by now: task waits for motion before a spindle command); a
// program gets its persist variable (S: the RPM as a float, M3/M4/M5: the M code's number),
// is compiled and loaded into its thread (a .out is loaded as it is), executed, and for the
// wait types waited for. issued() runs once the board has the action. Returns 0 done,
// 1 failed (message posted), 2 stopped waiting on an abort (the program carries on, as in
// KMotionCNC).
static int w_action(KmBackend::Impl *d, const KmAction &a, int mcode, double rpm, const std::function<void()> &issued,
                    const double *pq = nullptr)
{
    char cmd[640], reply[MAX_LINE + 1];
    const std::string who = mcode >= 100 ? "M" + std::to_string(mcode) : "spindle";
    switch (a.type) {
    case KM_ACTION_NONE:
        issued();
        return 0;
    case KM_ACTION_SETBIT:
    case KM_ACTION_SETTWOBITS:
        for (int k = 0; k < (a.type == KM_ACTION_SETTWOBITS ? 2 : 1); k++) {
            snprintf(cmd, sizeof cmd, "SetStateBit%d=%d", (int) a.p[2 * k], (int) a.p[2 * k + 1]);
            if (d->km->WriteLine(cmd)) { d->message(std::string("board command failed: ") + cmd); return 1; }
        }
        issued();
        return 0;
    case KM_ACTION_DAC: {
        int v = (int) floor(rpm * a.p[1] + a.p[2] + 0.5);          // scale and offset, then the limits
        if (v < (int) a.p[3]) v = (int) a.p[3];
        if (v > (int) a.p[4]) v = (int) a.p[4];
        snprintf(cmd, sizeof cmd, "DAC%d=%d", (int) a.p[0], v);
        if (d->km->WriteLine(cmd)) { d->message(std::string("board command failed: ") + cmd); return 1; }
        issued();
        return 0;
    }
    default:
        break;
    }
    // a program. One that wasn't waited for may still be running in the thread, and loading
    // over it would cut it short: let it finish (a thread that never ends is an error)
    int thread = (int) a.p[0], var = (int) a.p[1];
    snprintf(cmd, sizeof cmd, "CheckThread%d", thread);
    for (double t0 = now_s();;) {
        if (board_query(d, cmd, reply)) { d->message("CheckThread failed"); return 1; }
        if (strcmp(reply, "0") == 0) break;
        if (d->aborting || d->stop) return 2;
        if (now_s() - t0 > 10) {
            d->message(who + ": thread " + std::to_string(thread) + " is busy with another program, " + a.file + " was not run");
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (var >= 0 && pq) {
        // a user M code: P and Q as floats, in <var> and <var>+1 (LinuxCNC sends -1 for a word not given)
        for (int k = 0; k < 2; k++) {
            float f = (float) pq[k];
            unsigned bits;
            memcpy(&bits, &f, sizeof bits);
            snprintf(cmd, sizeof cmd, "SetPersistHex %d %x", var + k, bits);
            if (d->km->WriteLine(cmd)) { d->message(std::string("board command failed: ") + cmd); return 1; }
        }
    } else if (var >= 0) {
        unsigned bits = (unsigned) mcode;
        if (mcode == 10) { float f = (float) rpm; memcpy(&bits, &f, sizeof bits); }   // S: the speed as a float
        snprintf(cmd, sizeof cmd, "SetPersistHex %d %x", var, bits);
        if (d->km->WriteLine(cmd)) { d->message(std::string("board command failed: ") + cmd); return 1; }
    }
    if (a.file[0]) {
        size_t n = strlen(a.file);
        if (n > 4 && !strcasecmp(a.file + n - 4, ".out")) {
            if (d->km->LoadCoff(thread, a.file, 0)) { d->message(who + ": loading " + a.file + " failed"); return 1; }
        } else {
            char err[512] = "";
            if (d->km->CompileAndLoadCoff(a.file, thread, err, sizeof err - 1)) {
                d->message(who + " program failed: " + a.file + ": " + err);
                return 1;
            }
        }
    }
    snprintf(cmd, sizeof cmd, "Execute%d", thread);
    if (d->km->WriteLine(cmd)) { d->message(std::string("board command failed: ") + cmd); return 1; }
    issued();
    if (a.type == KM_ACTION_PROGRAM) return 0;
    snprintf(cmd, sizeof cmd, "CheckThread%d", thread);
    for (double t0 = now_s();;) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (d->aborting || d->stop) return 2;
        if (board_query(d, cmd, reply)) { d->message("CheckThread failed"); return 1; }
        if (strcmp(reply, "0") == 0) break;
        if (now_s() - t0 > 120) { d->message(who + ": " + a.file + " did not finish within 120 s"); return 1; }
    }
    if (a.type == KM_ACTION_PROGRAM_WAIT_SYNC) {
        // the program may have moved axes: carry on from where the board stands
        double x, y, z, aa, bb, cc, u, v;
        if (d->cm->ReadCurAbsPosition(&x, &y, &z, &aa, &bb, &cc, &u, &v) == 0) {
            double p[9] = {x, y, z, aa, bb, cc, u, v, 0};
            memcpy(d->pos, p, sizeof p);
            set_cm_position(d, d->pos);
        } else {
            d->cm->ClearAbort();
        }
    }
    return 0;
}

// G96/G97 on the board, as KMotionCNC's SetCSS() hands them over: for CSS the X counts of
// radius 0, the inches per count, the surface speed in in/s and the maximum RPM (floats in
// persist 111-114), then the mode in 110 (2 CSS, 1 RPM), which a ServiceCSS() loop on the
// board reads (Dynomotion's CSSJog.c: every 50 ms, RPM = surface speed / radius, capped,
// while the spindle is on). Machine units are inches here, as everywhere in the backend.
// changed: the mode differs from the one written before. Returns 0, or 1 (message posted)
static int w_css(KmBackend::Impl *d, const Cmd &c, bool &changed)
{
    char cmd[64];
    auto put = [&](int var, unsigned bits) {
        snprintf(cmd, sizeof cmd, "SetPersistHex %d %x", var, bits);
        if (d->km->WriteLine(cmd)) { d->message(std::string("board command failed: ") + cmd); return 1; }
        return 0;
    };
    auto float_bits = [](double v) { float f = (float) v; unsigned b; memcpy(&b, &f, sizeof b); return b; };
    int mode = c.sp_css ? 2 : 1;
    if (c.sp_css) {
        double k = d->scale[0];                // X's counts per inch
        if (d->cfg.channel[0] < 0 || k == 0) { d->message("G96 needs an X axis on the board"); return 1; }
        if (put(PC_COMM_CSS_X_OFFSET, float_bits(c.css.x_offset * k)) || put(PC_COMM_CSS_X_FACTOR, float_bits(1.0 / k)) ||
            put(PC_COMM_CSS_S, float_bits(c.css.surface_speed)) || put(PC_COMM_CSS_MAX_RPM, float_bits(c.css.max_rpm)))
            return 1;
        d->css_xoff = c.css.x_offset * k;
        d->css_k = k;
        d->css_surface = c.css.surface_speed;
        d->css_max_rpm = c.css.max_rpm;
    }
    // the mode every time (something else may have changed it since), last as KMotionCNC does
    if (put(PC_COMM_CSS_MODE, (unsigned) mode)) return 1;
    changed = mode != d->sp_css_mode;
    d->sp_css_mode = mode;
    return 0;
}

static void w_spindle(KmBackend::Impl *d, const Cmd &c)
{
    // a speed change superseded by the next queued one (the override slider sends a stream
    // of them) is skipped; a start, stop or reversal never is
    {
        std::lock_guard<std::mutex> lock(d->qmx);
        if (c.sp_state == d->sp_state && !d->q.empty() && d->q.front().kind == Cmd::SPINDLE && d->q.front().sp_state == c.sp_state)
            return;
    }
    if (c.sp_css && c.sp_state != 0 && !d->cfg.spindle_css) {
        if (!d->aborting) {
            d->message("G96 constant surface speed needs [KMOTION] SPINDLE_CSS = 1 and a ServiceCSS() loop "
                       "on the board: the spindle was not started");
            d->errors++;
            w_abort(d);
        }
        return;
    }
    // what S gets: the RPM, or in CSS mode the surface speed (KMotionCNC converts G96's S to
    // in/s for the S action; the bench's S variable is the CSS one, 113)
    const double sval = c.sp_css ? c.css.surface_speed : c.sp_rpm;
    if (d->cfg.simulate) {
        d->sp_state = c.sp_state;
        if (c.sp_state) { d->sp_rpm = sval; d->sp_css_mode = c.sp_css ? 2 : 1; }
        return;
    }
    const KmAction *A = d->cfg.spindle;
    int r = 0;
    bool mode_changed = false;
    if (c.sp_state != 0 && d->cfg.spindle_css) r = w_css(d, c, mode_changed);
    if (r == 0 && c.sp_state != 0) {
        // S before M3/M4, as a block "M3 S600" runs in KMotionCNC: the S program saves the
        // speed, the M3 program spins up to it. While it turns, a new speed is S alone
        if (d->sp_state == 0 || c.sp_state != d->sp_state || sval != d->sp_rpm || mode_changed)
            r = w_action(d, A[KM_SPINDLE_S], 10, sval, [&] { d->sp_rpm = sval; });
        if (r == 0 && c.sp_state != d->sp_state) {
            bool cw = c.sp_state > 0;
            r = w_action(d, A[cw ? KM_SPINDLE_M3 : KM_SPINDLE_M4], cw ? 3 : 4, sval, [&] { d->sp_state = c.sp_state; });
        }
    } else if (d->sp_state != 0) {
        r = w_action(d, A[KM_SPINDLE_M5], 5, c.sp_rpm, [&] { d->sp_state = 0; });
    }
    if (r == 1 && !d->aborting) {
        // the spindle isn't doing what LinuxCNC believes: stop, as a planner failure does
        d->errors++;
        w_abort(d);
    }
}

// G38.2-G38.5 (PROBE_BIT): the probe watcher, ServiceProbe() in the board's forever loop
// (configs/kmotion-kogna/probe/ProbeService.c has the persist layout), is armed with the probe
// bit, its contact level and the direction, and says whether the probe already is in the state
// sought (then nothing moves) or that it is watching. The move then goes to the planner like any
// line, flushed at once, so any kinematics apply. When the probe trips, the watcher records
// every coordinate-system axis's Dest, stops the coordinated motion (a feed hold) and disarms;
// here the stop is finished as an abort's is (at rest, the rest of the buffer abandoned, the
// planner where the board stands), and the recorded actuator positions become the probed
// position through the kinematics. A move that ends without a trip disarms the watcher.
// A trip can come while the move is still being planned and downloaded (a move longer than
// the planner's lookahead): the poller sees it and gets the worker out of the planner
static void w_probe(KmBackend::Impl *d, const Cmd &c)
{
    char cmd[64], reply[MAX_LINE + 1];
    double pos[9];
    memcpy(pos, d->pos, sizeof pos);
    auto finish = [&](int outcome) {
        d->probe_trip = false;
        std::lock_guard<std::mutex> lock(d->smx);
        d->st.probe_outcome = outcome;
        memcpy(d->st.probe_pos, pos, sizeof pos);
        d->st.probe_serial++;
    };
    auto status = [&]() {
        snprintf(cmd, sizeof cmd, "GetPersistDec %d", P_STATUS);
        return board_query(d, cmd, reply) ? -9 : atoi(reply);
    };
    auto disarm = [&] { set_persist(d, P_ARM, 0); };
    // tripped: where the axes were (doubles, low word first), then the stop finished
    auto tripped = [&] {
        double acts[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 8; i++) {
            unsigned w[2] = {0, 0};
            for (int k = 0; k < 2; k++) {
                snprintf(cmd, sizeof cmd, "GetPersistHex %d", P_POS + 2 * i + k);
                if (board_query(d, cmd, reply) == 0) w[k] = (unsigned) strtoul(reply, NULL, 16);
            }
            unsigned long long bits = ((unsigned long long) w[1] << 32) | w[0];
            memcpy(&acts[i], &bits, sizeof acts[i]);
        }
        w_abort(d);
        acts_to_cad(d, acts, pos);
        finish(KM_PROBE_TRIPPED);
    };
    if (set_persist(d, P_STATUS, -1) || set_persist(d, P_BIT, d->cfg.probe_bit) || set_persist(d, P_LEVEL, d->cfg.probe_level) ||
        set_persist(d, P_AWAY, c.away ? 1 : 0) || set_persist(d, P_ARM, 1)) { finish(KM_PROBE_FAILED); return; }
    int s = -1;
    for (double t0 = now_s(); (s = status()) == -1 && now_s() - t0 < 2;)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (s == 1) { finish(KM_PROBE_ALREADY); return; }
    if (s != 0) {
        d->message("the probe watcher did not answer: ServiceProbe() must run in the board's forever loop "
                   "(configs/kmotion-kogna/probe/ProbeService.c, BenchLoop.c)");
        disarm();
        finish(KM_PROBE_FAILED);
        return;
    }
    d->probe_trip = false;
    d->probe_away = c.away;
    d->probe_watch = true;
    w_line(d, c);
    if (!d->probe_trip) w_flush(d);
    {
        std::lock_guard<std::mutex> lk(d->probe_mx);
        d->probe_watch = false;
    }
    for (;;) {
        if (d->probe_trip) { tripped(); return; }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (d->aborting || d->stop) { disarm(); finish(KM_PROBE_ABORTED); return; }   // the queued ABORT stops the board
        if (status() == 2) { tripped(); return; }
        bool running;
        {
            std::lock_guard<std::mutex> lk(d->wmx);
            running = d->run_active || d->unflushed;
        }
        if (!running) {
            // the move ran to its end: stop watching (a trip the watcher is recording right now
            // still counts)
            disarm();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (status() == 2) { tripped(); return; }
            memcpy(pos, d->pos, sizeof pos);
            finish(KM_PROBE_NO_CONTACT);
            return;
        }
    }
}

static void worker_main(KmBackend::Impl *d)
{
    d->last_tick = d->last_poll = now_s();
    while (!d->stop) {
        // commands
        for (;;) {
            Cmd c;
            {
                std::unique_lock<std::mutex> lock(d->qmx);
                if (d->q.empty()) break;
                c = d->q.front();
                d->q.pop_front();
            }
            switch (c.kind) {
            case Cmd::LINE: w_line(d, c); break;
            case Cmd::ARC: w_arc(d, c); break;
            case Cmd::DWELL: w_dwell(d, c); break;
            case Cmd::ABORT: w_abort(d); break;
            case Cmd::FEED:
                d->feed_scale = c.feed_scale;
                d->rapid_scale = c.rapid_scale;
                if (!d->cfg.simulate) {
                    d->cm->SetFeedRateOverride(c.feed_scale);
                    d->cm->SetFeedRateRapidOverride(c.rapid_scale);
                }
                break;
            case Cmd::SET_POS:
                if (d->cfg.simulate && !d->have_cur && d->replay.empty() && !d->unflushed) {
                    memcpy(d->pos, c.end, sizeof d->pos);
                    set_cm_position(d, d->pos);
                }
                break;
            case Cmd::MACHINE_ON: if (!d->cfg.simulate) w_machine_on(d, c.on); break;
            case Cmd::HOME: if (!d->cfg.simulate) w_home(d, c.home); break;
            case Cmd::SPINDLE: w_spindle(d, c); d->spindle_done++; break;
            case Cmd::PROBE: w_probe(d, c); break;
            case Cmd::MCODE: {
                // a user M code: its action with P and Q (in simulate mode nothing to do)
                int r = 0;
                if (!d->cfg.simulate && c.mc >= 100 && c.mc < 200) {
                    const double pq[2] = {c.p, c.q};
                    r = w_action(d, d->cfg.mcode[c.mc - 100], c.mc, c.p, [] {}, pq);
                }
                d->mcode_result = r;
                d->mcode_done = c.ticket;
                break;
            }
            case Cmd::PATH_MODE:
                // in program order with the moves; the planner applies it per waypoint
                d->cm->SetPathMode(c.term_cond == 2 ? CANON_CONTINUOUS : CANON_EXACT_STOP,
                                   (c.term_cond == 2 && c.tolerance > 0) ? c.tolerance : -1.0);
                break;
            case Cmd::JOG_TO: if (!d->cfg.simulate) w_jog_to(d, c); d->jog_written[c.axis]++; break;
            }
        }
        double t = now_s();
        // keep the board (or the replay) fed between waypoints: the planner downloads its
        // finalized segments in bounded chunks when it gets to it, every 8th waypoint,
        // which is too rare when task trickles moves in one at a time against a full queue
        // (the replay ran dry with 19 moves pending; on the board the buffer would starve)
        if (d->unflushed && !d->aborting && nsegs > d->cm->m_nsegs_downloaded) {
            if (d->cm->DownloadDoneSegments()) fail(d, "DownloadDoneSegments");
            else harvest(d);
        }
        // the planner finalizes the last moves only when told that no more are coming:
        // flush once task has been quiet for a moment (program end, M0, tool change, dwell).
        // Not while task's queue is full and motion is under way: then task is waiting for
        // a move to finish before it hands over the next one, and a flush here would end
        // the run with a stop after every move (seen as arcs stopping at every segment)
        bool flush_now;
        {
            std::lock_guard<std::mutex> lk(d->wmx);
            int pending = 0;
            for (size_t k = 0; k < d->ids.size(); k++) if (k == 0 || d->ids[k] != d->ids[k - 1]) pending++;
            bool executing = d->cfg.simulate ? (d->have_cur || !d->replay.empty()) : (d->run_active && d->cm->CoordLaunched());
            if (pending >= d->cfg.queue_limit) d->last_full_t = t;
            // quiet = no move handed over for 50 ms. With the queue below its limit that means
            // task has nothing more (program end, M0, tool change, dwell) - but task needs a
            // few ms to refill a slot a finished move has just freed, so a full queue must
            // also be 50 ms in the past. With nothing executing, a flush is the only way to
            // get going again, full queue or not (the planner holds its tail back until
            // either more moves come or it is told that none will)
            bool quiet_moves = t - d->last_move_t > 0.05;
            bool slot_settled = t - d->last_full_t > 0.05;
            flush_now = d->unflushed && quiet_moves && ((pending < d->cfg.queue_limit && slot_settled) || !executing);
            if (flush_now && getenv("KM_DEBUG_FLUSH"))
                fprintf(stderr, "kmotion-motion: flush: %d pending (limit %d), executing %d, quiet %.3f s, serial %d done_upto %d\n",
                        pending, d->cfg.queue_limit, (int) executing, t - d->last_move_t, d->serial, d->done_upto);
        }
        if (flush_now) w_flush(d);
        if (d->cfg.simulate) {
            double dt = t - d->last_tick;
            if (dt > 0.1) dt = 0.1;
            w_replay(d, dt);
            d->last_tick = t;
            publish(d);
        }
        std::unique_lock<std::mutex> lock(d->qmx);
        if (d->q.empty()) d->qcv.wait_for(lock, std::chrono::milliseconds(5));
    }
}

// ---- KmBackend ----------------------------------------------------------------------------
KmBackend::KmBackend() : d(new Impl)
{
    memset(&d->st, 0, sizeof d->st);
    memset(d->pos, 0, sizeof d->pos);
    for (int i = 0; i < 8; i++) { d->enabled[i] = true; d->scale[i] = 1; d->used[i] = false; }
}

KmBackend::~KmBackend()
{
    d->cm->SetAbort();                // gets the worker out of a blocking download (e.g. paused mid-run)
    d->stop = true;
    d->qcv.notify_one();
    if (d->worker.joinable()) d->worker.join();
    if (d->poller.joinable()) d->poller.join();
    d->ucv.notify_one();
    if (d->urgent.joinable()) d->urgent.join();
    delete d->cm;
    delete d->km;
    delete d;
}

bool KmBackend::is_board() const { return !d->cfg.simulate; }
bool KmBackend::has_spindle() const
{
    for (const KmAction &a : d->cfg.spindle) if (a.type != KM_ACTION_NONE) return true;
    return false;
}

void KmBackend::spindle(int state, double rpm, const KmSpindleCss *css)
{
    Cmd c{};
    c.kind = Cmd::SPINDLE;
    c.sp_state = state;
    c.sp_rpm = rpm;
    c.sp_css = css != nullptr;
    if (css) c.css = *css;
    d->post(c);
}

void KmBackend::set_bit(int bit, bool on)
{
    if (d->cfg.simulate) return;
    char cmd[64];
    snprintf(cmd, sizeof cmd, "SetStateBit%d=%d", bit, on ? 1 : 0);
    d->send_urgent(cmd);
}

int KmBackend::mcode(int n, double p, double q)
{
    Cmd c{};
    c.kind = Cmd::MCODE;
    c.mc = n;
    c.p = p;
    c.q = q;
    c.ticket = ++d->mcode_posted;
    d->post(c);
    return c.ticket;
}

bool KmBackend::mcode_finished(int ticket, int &result)
{
    if (d->mcode_done < ticket) return false;
    result = d->mcode_result;
    return true;
}

bool KmBackend::has_probe() const { return !d->cfg.simulate && d->cfg.probe_bit >= 0; }

void KmBackend::probe(const double end[9], double vel, double acc, int id, bool when_clears)
{
    Cmd c{};
    c.kind = Cmd::PROBE;
    memcpy(c.end, end, sizeof c.end);
    c.vel = vel;
    c.acc = acc;
    c.id = id;
    c.away = when_clears;
    d->post(c);
}

bool KmBackend::has_home_program() const { return !d->cfg.simulate && d->cfg.home_program[0] != 0; }

void KmBackend::home(const KmHomeRequest &req)
{
    Cmd c{};
    c.kind = Cmd::HOME;
    c.home = req;
    d->post(c);
}

const char *KmBackend::mode_name() const
{
    return d->cfg.simulate ? "KMotion planner, simulated execution" : "KMotion planner and board";
}

bool KmBackend::init(const KmConfig &cfg, const double pos[9])
{
    d->cfg = cfg;
    memcpy(d->pos, pos, sizeof d->pos);
    d->km = new CKMotionDLL(0);
    d->km->SetErrMsgCallback(err_handler);
    d->cm = new CCoordMotion(d->km);
    CCoordMotion *cm = d->cm;
    cm->m_Simulate = cfg.simulate;
    cm->m_DoTime = true;            // plan with real timing even without a board

    MOTION_PARAMS *MP = cm->GetMotionParams();
    MP->BreakAngle = cfg.break_angle;
    MP->CollinearTol = cfg.collinear_tol;
    MP->CornerTol = cfg.corner_tol;
    MP->FacetAngle = cfg.facet_angle;
    MP->TPLookahead = cfg.lookahead;
    MP->MaxRapidFRO = 1.0;
    MP->DegreesA = MP->DegreesB = MP->DegreesC = true;
    double *counts[8] = {&MP->CountsPerInchX, &MP->CountsPerInchY, &MP->CountsPerInchZ, &MP->CountsPerInchA,
                         &MP->CountsPerInchB, &MP->CountsPerInchC, &MP->CountsPerInchU, &MP->CountsPerInchV};
    double *vel[8] = {&MP->MaxVelX, &MP->MaxVelY, &MP->MaxVelZ, &MP->MaxVelA, &MP->MaxVelB, &MP->MaxVelC, &MP->MaxVelU, &MP->MaxVelV};
    double *acc[8] = {&MP->MaxAccelX, &MP->MaxAccelY, &MP->MaxAccelZ, &MP->MaxAccelA, &MP->MaxAccelB, &MP->MaxAccelC, &MP->MaxAccelU, &MP->MaxAccelV};
    double *jerk[8] = {&MP->MaxJerkX, &MP->MaxJerkY, &MP->MaxJerkZ, &MP->MaxJerkA, &MP->MaxJerkB, &MP->MaxJerkC, &MP->MaxJerkU, &MP->MaxJerkV};
    double *rvel[8] = {&MP->MaxRapidVelX, &MP->MaxRapidVelY, &MP->MaxRapidVelZ, &MP->MaxRapidVelA, &MP->MaxRapidVelB, &MP->MaxRapidVelC, &MP->MaxRapidVelU, &MP->MaxRapidVelV};
    double *racc[8] = {&MP->MaxRapidAccelX, &MP->MaxRapidAccelY, &MP->MaxRapidAccelZ, &MP->MaxRapidAccelA, &MP->MaxRapidAccelB, &MP->MaxRapidAccelC, &MP->MaxRapidAccelU, &MP->MaxRapidAccelV};
    double *rjerk[8] = {&MP->MaxRapidJerkX, &MP->MaxRapidJerkY, &MP->MaxRapidJerkZ, &MP->MaxRapidJerkA, &MP->MaxRapidJerkB, &MP->MaxRapidJerkC, &MP->MaxRapidJerkU, &MP->MaxRapidJerkV};
    double *spos[8] = {&MP->SoftLimitPosX, &MP->SoftLimitPosY, &MP->SoftLimitPosZ, &MP->SoftLimitPosA, &MP->SoftLimitPosB, &MP->SoftLimitPosC, &MP->SoftLimitPosU, &MP->SoftLimitPosV};
    double *sneg[8] = {&MP->SoftLimitNegX, &MP->SoftLimitNegY, &MP->SoftLimitNegZ, &MP->SoftLimitNegA, &MP->SoftLimitNegB, &MP->SoftLimitNegC, &MP->SoftLimitNegU, &MP->SoftLimitNegV};
    for (int i = 0; i < 8; i++) {
        const KmAxisParams &ax = cfg.axis[i];
        d->used[i] = ax.counts_per_unit > 0;
        // kmxWeb's defaults for an undefined axis: the planner needs non-zero numbers
        double c = ax.counts_per_unit > 0 ? ax.counts_per_unit : 100.0;
        double v = ax.max_vel > 0 ? ax.max_vel : 0.1;
        double a = ax.max_accel > 0 ? ax.max_accel : 0.01;
        double j = ax.max_jerk > 0 ? ax.max_jerk : 10.0 * a;
        *counts[i] = c; *vel[i] = v; *acc[i] = a; *jerk[i] = j;
        *rvel[i] = v; *racc[i] = a; *rjerk[i] = j;
        *spos[i] = 1e9; *sneg[i] = -1e9;          // LinuxCNC enforces its own limits
        d->scale[i] = c;
        // one actuator per axis
        MP->ActScale[i] = d->used[i] ? c : 0;
        MP->MaxActVel[i] = d->used[i] ? v : 0;
        MP->MaxActAccel[i] = d->used[i] ? a : 0;
        MP->MaxActJerk[i] = d->used[i] ? j : 0;
        MP->ActDegrees[i] = (i >= 3 && i <= 5);
    }
    MP->ThirdOrderTP = cfg.third_order;
    MP->CubicKnots = cfg.third_order && cfg.cubic_knots;
    MP->ActuatorLimits = cfg.actuator_limits;
    MP->LogSegments = cfg.log_segments;
    cm->SetTPParams();

    if (cfg.simulate) {
        // the coordinate system, without asking the board
        cm->x_axis = cfg.channel[0]; cm->y_axis = cfg.channel[1]; cm->z_axis = cfg.channel[2];
        cm->a_axis = cfg.channel[3]; cm->b_axis = cfg.channel[4]; cm->c_axis = cfg.channel[5];
        cm->u_axis = cfg.channel[6]; cm->v_axis = cfg.channel[7];
        cm->m_DefineCS_valid = cm->m_DefineCS_known = true;
        // rapids use the board's own axis settings and GetRapidSettings would fetch them
        // (even in simulate mode it tries, which launches KMotionServer): the rapid limits
        // are set above, so mark them fresh
        cm->RapidParamsDirty = false;
        set_cm_position(d, pos);
    } else {
        int type = 0;
        if (d->km->CheckKMotionVersion(&type)) {
            std::string m;
            { std::lock_guard<std::mutex> lock(g_msg_mutex); m = g_message; g_message.clear(); }
            fprintf(stderr, "kmotion-motion: no board, or firmware mismatch: %s\n", m.c_str());
            return false;
        }
        fprintf(stderr, "kmotion-motion: connected to a %s\n", type == BOARD_TYPE_KOGNA ? "Kogna" : "KFLOP");
        d->km->SetConsoleCallback(console_handler);     // registers with the server: board mode only
        for (const std::string &prog : cfg.init_programs) {
            if (run_program(d, prog.c_str(), cfg.init_thread)) {
                std::string m;
                { std::lock_guard<std::mutex> lock(d->smx); m = d->messages; }
                fprintf(stderr, "kmotion-motion: %s\n", m.c_str());
                return false;
            }
        }
        if (cm->SetAxisDefinitions(cfg.channel[0], cfg.channel[1], cfg.channel[2], cfg.channel[3],
                                   cfg.channel[4], cfg.channel[5], cfg.channel[6], cfg.channel[7])) {
            fprintf(stderr, "kmotion-motion: DefineCS failed\n");
            return false;
        }
        double x, y, z, a, b, c, u, v;
        if (cm->ReadCurAbsPosition(&x, &y, &z, &a, &b, &c, &u, &v)) {
            fprintf(stderr, "kmotion-motion: reading the board position failed\n");
            cm->ClearAbort();
            return false;
        }
        double p[9] = {x, y, z, a, b, c, u, v, 0};
        memcpy(d->pos, p, sizeof p);
        // programs that keep running (forever loops), once the axes and the coordinate system are set
        for (const auto &sp : cfg.start_programs) {
            if (start_program(d, sp.second.c_str(), sp.first)) {
                std::string m;
                { std::lock_guard<std::mutex> lock(d->smx); m = d->messages; }
                fprintf(stderr, "kmotion-motion: %s\n", m.c_str());
                return false;
            }
        }
        d->connected = true;
    }
    cm->ClearAbort();
    cm->ClearHalt();
    {
        std::lock_guard<std::mutex> lock(d->smx);
        memcpy(d->st.pos, d->pos, sizeof d->st.pos);
        for (int i = 0; i < 8; i++) d->st.enabled[i] = true;
        d->st.connected = true;
    }
    memcpy(d->cmd_end, d->pos, sizeof d->cmd_end);
    d->worker = std::thread(worker_main, d);
    if (!cfg.simulate) {
        d->poller = std::thread(poller_main, d);
        d->urgent = std::thread(urgent_main, d);
    }
    return true;
}

void KmBackend::set_position(const double pos[9])
{
    Cmd c{};
    c.kind = Cmd::SET_POS;
    memcpy(c.end, pos, sizeof c.end);
    d->post(c);
}

int KmBackend::line(const double end[9], double vel, double acc, bool rapid, int id)
{
    Cmd c{};
    c.kind = Cmd::LINE;
    memcpy(c.end, end, sizeof c.end);
    c.vel = vel; c.acc = acc; c.rapid = rapid; c.id = id;
    d->post(c);
    return 0;
}

int KmBackend::arc(const double end[9], const double center[3], const double normal[3], int turn, double vel, double acc, int id)
{
    Cmd c{};
    c.kind = Cmd::ARC;
    memcpy(c.end, end, sizeof c.end);
    memcpy(c.center, center, sizeof c.center);
    memcpy(c.normal, normal, sizeof c.normal);
    c.turn = turn; c.vel = vel; c.acc = acc; c.id = id;
    d->post(c);
    return 0;
}

void KmBackend::set_path_mode(int term_cond, double tolerance)
{
    Cmd c{};
    c.kind = Cmd::PATH_MODE;
    c.term_cond = term_cond; c.tolerance = tolerance;
    d->post(c);
}

int KmBackend::dwell(double seconds, int id)
{
    Cmd c{};
    c.kind = Cmd::DWELL;
    c.seconds = seconds; c.id = id;
    d->post(c);
    return 0;
}

void KmBackend::abort()
{
    // the moves still queued for the worker belong to the program being dropped; the
    // planner's abort flag gets the worker out of any wait on the board; the board's
    // feed hold starts now. The worker finishes the stop (waits for the board to come
    // to rest, clears the stop state, resets the planner) and the state reads
    // "running" until then
    d->aborting = true;
    {
        std::lock_guard<std::mutex> lock(d->qmx);
        std::deque<Cmd> keep;
        for (const Cmd &x : d->q)
            if (x.kind == Cmd::FEED || x.kind == Cmd::SET_POS || x.kind == Cmd::MACHINE_ON || x.kind == Cmd::PATH_MODE ||
                x.kind == Cmd::SPINDLE) keep.push_back(x);
            else if (x.kind == Cmd::JOG_TO) d->jog_written[x.axis]++;   // dropped: nothing to wait for
            else if (x.kind == Cmd::MCODE) { d->mcode_result = 2; d->mcode_done = x.ticket; }
        d->q.swap(keep);
    }
    d->cm->SetAbort();
    if (!d->cfg.simulate) d->send_urgent("StopImmediate0");
    d->paused = false;
    Cmd c{};
    c.kind = Cmd::ABORT;
    d->post(c);
}

void KmBackend::pause(bool on)
{
    d->paused = on;
    if (!d->cfg.simulate) d->send_urgent(on ? "StopImmediate0" : "StopImmediate1");   // feed hold / resume
}

void KmBackend::set_feed_override(double feed_scale, double rapid_scale)
{
    Cmd c{};
    c.kind = Cmd::FEED;
    c.feed_scale = feed_scale; c.rapid_scale = rapid_scale;
    d->post(c);
}

void KmBackend::machine_on(bool on)
{
    Cmd c{};
    c.kind = Cmd::MACHINE_ON;
    c.on = on;
    d->post(c);
}

void KmBackend::jog(int axis, double vel)
{
    if (d->cfg.simulate || axis < 0 || axis >= 8) return;
    int ch = d->cfg.channel[axis];
    if (ch < 0) return;
    char cmd[64];
    snprintf(cmd, sizeof cmd, "Jog%d=%.3f", ch, vel * d->scale[axis]);   // counts/s, the board's limits apply
    d->jog_issued[axis]++;
    d->send_urgent(cmd, axis);
}

void KmBackend::jog_to(int axis, double target, double vel)
{
    if (d->cfg.simulate || axis < 0 || axis >= 8 || d->cfg.channel[axis] < 0) return;
    Cmd c{};
    c.kind = Cmd::JOG_TO;
    c.axis = axis;
    c.end[axis] = target;
    c.vel = vel;
    d->jog_issued[axis]++;
    d->post(c);
}

void KmBackend::jog_stop(int axis)
{
    if (d->cfg.simulate) return;
    for (int i = 0; i < 8; i++)
        if ((axis < 0 || axis == i) && d->cfg.channel[i] >= 0) jog(i, 0);
}

void KmBackend::state(KmState &out)
{
    std::lock_guard<std::mutex> lock(d->smx);
    out = d->st;
    out.jog_busy = 0;
    for (int i = 0; i < 8; i++) if (d->jog_finished[i] != d->jog_issued[i]) out.jog_busy |= 1u << i;
    out.status_count = d->status_count;
    out.input_states = d->input_states;
    out.probe_contact = d->probe_contact;
    if (d->cfg.simulate || d->cfg.spindle_at_speed == KM_AT_SPEED_NONE) {
        out.spindle_at_speed = true;              // nothing to wait for
        out.spindle_at_speed_done = d->spindle_done;
    } else {
        out.spindle_at_speed = d->sp_at_speed;
        out.spindle_at_speed_done = d->sp_at_speed_done;
    }
    out.message[0] = 0;
    if (!d->messages.empty()) {
        snprintf(out.message, sizeof out.message, "%s", d->messages.c_str());
        d->messages.clear();
    }
}
