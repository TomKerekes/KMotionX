/*
 * kmotion-pccomm.cc - the board's commands to the PC, against LinuxCNC.
 *
 * A C program on the KFLOP/Kogna drives KMotionCNC by writing a command code into persist
 * variable 100 (PC_COMM_PERSIST), its arguments into 101-107 and strings into the gather
 * buffer; the status upload carries persist 100-107, the PC does what was asked and writes the
 * result into persist 100: 0 done, negative failed (DoPC() in C Programs/
 * KflopToKMotionCNCFunctions.c spins until it is <= 0). The codes are DSP_KFLOP/PC-DSP.h's
 * PC_COMM_*; what KMotionCNC does with each is ServiceKFLOPCommands() in KMotionCNCDlg.cpp.
 *
 * Here the same commands go to LinuxCNC. Nearly all of them are task-level operations (MDI,
 * touch-off, the tool table, run/pause/abort, the overrides), so this file talks to task the
 * way a GUI does: through the NML channels, as the linuxcnc Python module does, opened as
 * process "xemc". Set-DRO is AXIS's own touch-off (G10 L20), tool table changes are G10 L1,
 * an override is EMC_TRAJ_SET_SCALE (the GUI's slider follows, and the value reaches the
 * board's FRO through the usual EMCMOT_FEED_SCALE path). A message goes through motion's
 * error ring (emcError is task's to write), so it shows as the GUI's notification; no GUI can
 * answer Yes/No, see PC_COMM_MSG below.
 *
 * This is the one file that includes both sides' headers: LinuxCNC's NML and PC-DSP.h's codes.
 * The board link itself is the backend's (KmBackend::pc_result and the persist/gather
 * primitives).
 *
 * The handshake: a status read may be on the wire while the result is written and still show
 * the command, so the next command is taken only from a read two counts after the write; a
 * command already present in the first read is refused with -2 (a program left waiting by an
 * earlier session must not fire now) as KMotionCNC does; one command runs at a time (the board
 * side is single-writer by contract). Results: 0 done; -1 failed, busy or not supported
 * (KMotionCNC's own default for an unknown code); -2 bad argument.
 */
#include "kmotion-pccomm.h"
#include "kmotion-backend.h"

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <functional>
#include <mutex>
#include <set>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <sys/time.h>

#include "config.h"                     // EMC2_DEFAULT_NMLFILE
#include "libnml/rcs/rcs.hh"
#include "libnml/rcs/rcs_print.hh"
#include "nml_intf/emc.hh"
#include "nml_intf/emc_nml.hh"
#include "tooldata/tooldata.hh"

#include "PC-DSP.h"                     // the PC_COMM_* codes

static KmBackend *km = nullptr;
static KmPcCommConfig cfg;
static std::thread th;
static std::atomic<bool> stop_flag{false};
static void (*pclog)(const char *fmt, ...) = nullptr;
static std::mutex msg_mx;
static std::deque<std::string> msgs;

static double now_s()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

static void post_message(const std::string &m)
{
    std::lock_guard<std::mutex> lock(msg_mx);
    msgs.push_back(m);
}

bool pccomm_message(std::string &out)
{
    std::lock_guard<std::mutex> lock(msg_mx);
    if (msgs.empty()) return false;
    out = msgs.front();
    msgs.pop_front();
    return true;
}

// ---- LinuxCNC through NML, as the linuxcnc Python module does it ------------------------------
static struct {
    RCS_CMD_CHANNEL *c = nullptr;
    RCS_STAT_CHANNEL *s = nullptr;
    EMC_STAT *st = nullptr;
    int serial = 0;
    double next_try = 0;
} nml;
static bool have_stat = false;         // a status has been seen since the channels were opened

// task starts after us (loadusr -W in the HAL file): open when it is there, quietly
static bool nml_open()
{
    if (nml.c) return true;
    double t = now_s();
    if (t < nml.next_try) return false;
    nml.next_try = t + 2.0;
    const char *file = cfg.nml_file.empty() ? EMC2_DEFAULT_NMLFILE : cfg.nml_file.c_str();
    set_rcs_print_destination(RCS_PRINT_TO_NULL);
    RCS_CMD_CHANNEL *c = new RCS_CMD_CHANNEL(emcFormat, "emcCommand", "xemc", file);
    RCS_STAT_CHANNEL *s = c->valid() ? new RCS_STAT_CHANNEL(emcFormat, "emcStatus", "xemc", file) : nullptr;
    set_rcs_print_destination(RCS_PRINT_TO_STDERR);
    if (!c->valid() || !s || !s->valid()) {
        delete s;
        delete c;
        return false;
    }
    nml.c = c;
    nml.s = s;
    nml.st = (EMC_STAT *) s->get_address();
    pclog("pccomm: LinuxCNC's NML channels open (%s)\n", file);
    return true;
}

static void nml_close()
{
    delete nml.s;
    delete nml.c;
    nml.s = nullptr;
    nml.c = nullptr;
    nml.st = nullptr;
    have_stat = false;
}

// the current status, or null when task is not there. peek() reports the type only when
// task has written a status since the last look (every cycle, but two looks a millisecond
// apart see nothing new): once one has been seen, the buffer is good
static EMC_STAT *snapshot()
{
    if (!nml_open()) return nullptr;
    if (nml.s->peek() == EMC_STAT_TYPE) have_stat = true;
    return have_stat ? nml.st : nullptr;
}

static bool wait_until(const std::function<bool()> &cond, double timeout)
{
    double t0 = now_s();
    while (!stop_flag) {
        if (snapshot() && cond()) return true;
        if (now_s() - t0 > timeout) return false;
        usleep(10000);
    }
    return false;
}

// send a command and wait for task to take it (the Python module's emcSendCommand +
// wait_complete): 0 done, -1 refused, failed or timed out
static int send(RCS_CMD_MSG &cmd, double timeout = 5.0)
{
    if (!nml_open() || nml.c->write(&cmd)) return -1;
    nml.serial = cmd.serial_number;
    double t0 = now_s();
    while (!stop_flag && now_s() - t0 < timeout) {
        if (snapshot()) {
            int diff = nml.st->echo_serial_number - nml.serial;
            if (diff > 0) return 0;
            if (diff == 0 && nml.st->status == RCS_STATUS::DONE) return 0;
            if (diff == 0 && nml.st->status == RCS_STATUS::ERROR) return -1;
        }
        usleep(10000);
    }
    return -1;
}

// nothing running: the equivalent of KMotionCNC's !ThreadIsExecuting
static bool idle(const EMC_STAT *st)
{
    return st->task.state == EMC_TASK_STATE::ON && st->task.interpState == EMC_TASK_INTERP::IDLE &&
           st->motion.traj.queue == 0;
}

static int set_mode(EMC_TASK_MODE mode)
{
    EMC_STAT *st = snapshot();
    if (!st) return -1;
    if (st->task.mode == mode) return 0;
    EMC_TASK_SET_MODE c;
    c.mode = mode;
    if (send(c)) return -1;
    return wait_until([mode] { return nml.st->task.mode == mode; }, 2.0) ? 0 : -1;
}

// an MDI line, as the GUI would send it, through to the end of its motion; the mode goes
// back to what it was (an operator jogging in MANUAL keeps the jog keys). -1: refused (a
// program is running, task is not on) or the interpreter reported an error
static int mdi(const std::string &line)
{
    EMC_STAT *st = snapshot();
    if (!st || !idle(st)) return -1;
    EMC_TASK_MODE prev = st->task.mode;
    if (set_mode(EMC_TASK_MODE::MDI)) return -1;
    pclog("pccomm: MDI %s\n", line.c_str());
    EMC_TASK_PLAN_EXECUTE c;
    snprintf(c.command, sizeof c.command, "%s", line.c_str());
    int r = send(c);
    bool done = wait_until([] {
        return nml.st->task.interpState == EMC_TASK_INTERP::IDLE && nml.st->motion.traj.queue == 0 &&
               nml.st->task.queuedMDIcommands == 0 && nml.st->task.execState != EMC_TASK_EXEC::WAITING_FOR_MOTION;
    }, cfg.mdi_timeout);
    if (prev != EMC_TASK_MODE::MDI) set_mode(prev);
    return (r || !done) ? -1 : 0;
}

// ---- units and positions ---------------------------------------------------------------------
// LinuxCNC's status is in machine units; KMotionCNC's values are the interpreter's current
// units. The factor machine -> program for the linear axes
static double lin_factor(const EMC_STAT *st)
{
    double k = 1.0;
    if (cfg.machine_mm) k = 1.0 / 25.4;              // mm -> inches
    if (st->task.programUnits == CANON_UNITS_MM) k *= 25.4;
    else if (st->task.programUnits == CANON_UNITS_CM) k *= 2.54;
    return k;
}

static bool linear_axis(int i) { return i < 3 || i > 5; }    // x y z, u v (w)

static void pose_to_array(const EmcPose &p, double out[8])
{
    out[0] = p.tran.x; out[1] = p.tran.y; out[2] = p.tran.z;
    out[3] = p.a; out[4] = p.b; out[5] = p.c; out[6] = p.u; out[7] = p.v;
}

// the 8 axes to the persists at dest (doubles), in program units: machine coordinates or
// the DROs (minus the G5x, G92 and tool offsets, what the GUI shows; XY rotation left aside)
static int send_coordinates(int dest, bool machine)
{
    EMC_STAT *st = snapshot();
    if (!st) return -1;
    double pos[8], g5x[8], g92[8], tool[8];
    pose_to_array(st->motion.traj.position, pos);
    pose_to_array(st->task.g5x_offset, g5x);
    pose_to_array(st->task.g92_offset, g92);
    pose_to_array(st->task.toolOffset, tool);
    double k = lin_factor(st);
    for (int i = 0; i < 8; i++) {
        double v = machine ? pos[i] : pos[i] - g5x[i] - g92[i] - tool[i];
        if (linear_axis(i)) v *= k;
        if (!km->persist_set_double(dest + i, v)) return -1;
    }
    return 0;
}

static float float_arg(int bits)
{
    float f;
    memcpy(&f, &bits, sizeof f);
    return f;
}

static bool persist_set_float(int index, double v)
{
    float f = (float) v;
    int bits;
    memcpy(&bits, &f, sizeof bits);
    return km->persist_set(index, bits);
}

// ---- the tool table: LinuxCNC's tooldata, mapped in as the Python module does ----------------
static bool tool_mmap_ready = false, tool_mmap_failed = false;
static bool tool_table_open()
{
    if (tool_mmap_ready) return true;
    if (tool_mmap_failed) return false;
    if (tool_mmap_user()) {
        tool_mmap_failed = true;
        pclog("pccomm: the tool table is not available (tool_mmap_user failed)\n");
        return false;
    }
    tool_mmap_ready = true;
    return true;
}

static bool tool_entry(int toolno, CANON_TOOL_TABLE &t)
{
    if (!tool_table_open() || toolno < 0) return false;
    int idx = tooldata_find_index_for_tool(toolno);
    return idx >= 0 && tooldata_get(&t, idx) == IDX_OK;
}

// GET/SET_TOOLTABLE_*: which field, and the G10 L1 word that sets it
enum ToolField { TF_LENGTH, TF_DIAMETER, TF_OFFSETX, TF_OFFSETY };
static int tool_get(int toolno, ToolField f, int dest)
{
    CANON_TOOL_TABLE t;
    EMC_STAT *st = snapshot();
    if (!st) return -1;
    if (!tool_entry(toolno, t)) return -2;
    double v = f == TF_LENGTH ? t.offset.tran.z : f == TF_DIAMETER ? t.diameter : f == TF_OFFSETX ? t.offset.tran.x : t.offset.tran.y;
    return km->persist_set_double(dest, v * lin_factor(st)) ? 0 : -1;
}
static int tool_set(int toolno, ToolField f, int src)
{
    CANON_TOOL_TABLE t;
    double v;
    if (!tool_entry(toolno, t)) return -2;
    if (!km->persist_get_double(src, v)) return -1;
    char line[64];
    switch (f) {
    case TF_LENGTH:   snprintf(line, sizeof line, "G10 L1 P%d Z%.6f", toolno, v); break;
    case TF_DIAMETER: snprintf(line, sizeof line, "G10 L1 P%d R%.6f", toolno, v / 2); break;
    case TF_OFFSETX:  snprintf(line, sizeof line, "G10 L1 P%d X%.6f", toolno, v); break;
    default:          snprintf(line, sizeof line, "G10 L1 P%d Y%.6f", toolno, v); break;
    }
    return mdi(line);
}

// ---- the interpreter's #variables: no NML access; the var file, which task rewrites after
// every program and MDI line, has the persistent ones ---------------------------------------
static bool read_var_file(std::map<int, double> &vars)
{
    if (cfg.var_file.empty()) return false;
    std::ifstream f(cfg.var_file);
    if (!f) return false;
    int n;
    double v;
    while (f >> n >> v) vars[n] = v;
    return true;
}

static const char *axis_letters = "XYZABCUV";

static int override_cmd(int code, int bits, const EMC_STAT *st)
{
    double f = float_arg(bits);
    if (!(f >= 0) || f > 100) return -2;
    switch (code) {
    case PC_COMM_SET_FRO: case PC_COMM_SET_FRO_INC: {
        EMC_TRAJ_SET_SCALE c;
        c.scale = code == PC_COMM_SET_FRO ? f : st->motion.traj.scale * f;
        c.scale = std::min(c.scale, cfg.max_feed_override);
        return send(c);
    }
    case PC_COMM_SET_RRO: case PC_COMM_SET_RRO_INC: {
        EMC_TRAJ_SET_RAPID_SCALE c;
        c.scale = code == PC_COMM_SET_RRO ? f : st->motion.traj.rapid_scale * f;
        c.scale = std::min(c.scale, 1.0);
        return send(c);
    }
    default: {
        EMC_TRAJ_SET_SPINDLE_SCALE c;
        c.spindle = 0;
        c.scale = code == PC_COMM_SET_SSO ? f : st->motion.spindle[0].spindle_scale * f;
        c.scale = std::max(cfg.min_spindle_override, std::min(c.scale, cfg.max_spindle_override));
        return send(c);
    }
    }
}

// ---- the commands ----------------------------------------------------------------------------
static std::set<int> unsupported_seen;

static int handle(const int pc[8])
{
    const int code = pc[0];
    EMC_STAT *st = snapshot();
    switch (code) {
    // run control
    case PC_COMM_ESTOP: {
        EMC_TASK_SET_STATE c;
        c.state = EMC_TASK_STATE::ESTOP;
        return send(c);
    }
    case PC_COMM_HALT:
    case PC_COMM_RESTART: {         // the next run starts at line 1 anyway
        EMC_TASK_ABORT c;
        return send(c);
    }
    case PC_COMM_EXECUTE: {
        if (!st) return -1;
        if (st->task.interpState == EMC_TASK_INTERP::PAUSED) {
            EMC_TASK_PLAN_RESUME c;
            return send(c);
        }
        if (!idle(st) || !st->task.file[0]) return -1;
        if (set_mode(EMC_TASK_MODE::AUTO)) return -1;
        EMC_TASK_PLAN_RUN c;
        c.line = 0;
        return send(c);
    }
    case PC_COMM_SINGLE_STEP: {
        if (!st || st->task.state != EMC_TASK_STATE::ON) return -1;
        if (st->task.interpState == EMC_TASK_INTERP::IDLE && (!st->task.file[0] || set_mode(EMC_TASK_MODE::AUTO))) return -1;
        EMC_TASK_PLAN_STEP c;
        return send(c);
    }
    case PC_COMM_HALT_NEXT_LINE: {  // the nearest thing: a feed hold, resumable
        if (!st || st->task.interpState == EMC_TASK_INTERP::IDLE) return -1;
        EMC_TASK_PLAN_PAUSE c;
        return send(c);
    }
    // overrides
    case PC_COMM_SET_FRO: case PC_COMM_SET_FRO_INC:
    case PC_COMM_SET_RRO: case PC_COMM_SET_RRO_INC:
    case PC_COMM_SET_SSO: case PC_COMM_SET_SSO_INC:
        if (!st) return -1;
        return override_cmd(code, pc[1], st);
    // the DROs: AXIS's touch-off, into the active coordinate system (P0)
    case PC_COMM_SET_X: case PC_COMM_SET_Y: case PC_COMM_SET_Z: case PC_COMM_SET_A:
    case PC_COMM_SET_B: case PC_COMM_SET_C: case PC_COMM_SET_U: case PC_COMM_SET_V: {
        char line[64];
        snprintf(line, sizeof line, "G10 L20 P0 %c%.6f", axis_letters[code - PC_COMM_SET_X], (double) float_arg(pc[1]));
        return mdi(line);
    }
    case PC_COMM_MDI: {
        std::string s;
        if (!km->gather_read_string(pc[1], 50, s)) return -1;
        int r = mdi(s);
        km->persist_set(PC_COMM_PERSIST + 2, r);        // the exit code, as KMotionCNC reports it
        return r;
    }
    case PC_COMM_MCODE: {
        char line[32];
        snprintf(line, sizeof line, "M%d", pc[1]);
        return mdi(line);
    }
    case PC_COMM_USER_BUTTON: {
        auto it = cfg.user_button.find(pc[1]);
        if (it == cfg.user_button.end()) {
            pclog("pccomm: USER_BUTTON %d: no [KMOTION] USER_BUTTON_%d in the ini\n", pc[1], pc[1]);
            return -2;
        }
        return mdi(it->second);
    }
    case PC_COMM_MSG: {
        // the GUI shows it as a notification; nobody can press a button: an MB_OK box gets
        // IDOK, a box with other buttons IDCANCEL and the command fails, so a program that
        // branches on the answer can tell
        std::string s;
        if (!km->gather_read_string(pc[1], 50, s)) return -1;
        post_message("KFLOP: " + s);
        bool ok_only = (pc[2] & 0xF) == 0;                // MB_TYPEMASK: MB_OK = 0
        km->persist_set(PC_COMM_PERSIST + 3, ok_only ? 1 : 2);   // IDOK, IDCANCEL
        return ok_only ? 0 : -1;
    }
    // data for the board
    case PC_COMM_GET_DROS:           return send_coordinates(pc[1], false);
    case PC_COMM_GET_MACHINE_COORDS: return send_coordinates(pc[1], true);
    case PC_COMM_GET_MISC_SETTINGS: {
        // units (the canon codes, as KMotionCNC's), T, H, D: LinuxCNC has no separate H and D
        // numbers, so the tool in the spindle stands for them while G43 / G41-G42 are active
        if (!st) return -1;
        bool g43 = false, g41 = false;
        for (int i = 0; i < ACTIVE_G_CODES; i++) {
            if (st->task.activeGCodes[i] == 430 || st->task.activeGCodes[i] == 431) g43 = true;
            if (st->task.activeGCodes[i] == 410 || st->task.activeGCodes[i] == 420) g41 = true;
        }
        int tool = st->io.tool.toolInSpindle;
        return km->persist_set(pc[1], (int) st->task.programUnits) && km->persist_set(pc[1] + 1, tool) &&
               km->persist_set(pc[1] + 2, g43 ? tool : 0) && km->persist_set(pc[1] + 3, g41 ? tool : 0) ? 0 : -1;
    }
    case PC_COMM_GET_TOOL_SLOT_ID: {
        if (!st) return -1;
        CANON_TOOL_TABLE t;
        int tool = st->io.tool.toolInSpindle;
        int pocket = tool_entry(tool, t) ? t.pocketno : 0;
        return km->persist_set(pc[1], pocket) && km->persist_set(pc[1] + 1, tool) ? 0 : -1;
    }
    case PC_COMM_GETAXISRES: {       // counts per inch, X..C, as floats
        for (int i = 0; i < 6; i++) {
            double c = km->axis_params(i).counts_per_unit;
            if (cfg.machine_mm && linear_axis(i)) c *= 25.4;
            if (!persist_set_float(pc[1] + i, c)) return -1;
        }
        return 0;
    }
    case PC_COMM_GET_TP_PARAM: {     // KMotionCNC's types: 0 velocity, 1 acceleration, 2 counts/inch, 3 jog speed
        int type = pc[1], axis = pc[2];
        if (type < 0 || type > 3 || axis < 0 || axis > 7 || pc[3] < 0 || pc[3] >= 100) return -2;
        const KmAxisParams &a = km->axis_params(axis);
        double k = cfg.machine_mm && linear_axis(axis) ? 1.0 / 25.4 : 1.0;    // to inches
        double v = type == 0 ? a.max_vel * k : type == 1 ? a.max_accel * k : type == 2 ? a.counts_per_unit / k : a.max_vel * k;
        return km->persist_set_double(pc[3], v) ? 0 : -1;
    }
    case PC_COMM_GET_VARS: {
        std::map<int, double> vars;
        if (!read_var_file(vars)) return -1;
        for (int i = 0; i < pc[2]; i++) {
            auto it = vars.find(pc[1] + i);
            if (!km->persist_set_double(pc[3] + i, it == vars.end() ? 0.0 : it->second)) return -1;
        }
        return 0;
    }
    case PC_COMM_SET_VARS: {
        std::string line;
        for (int i = 0; i < pc[2]; i++) {
            double v;
            if (!km->persist_get_double(pc[3] + i, v)) return -1;
            char s[48];
            snprintf(s, sizeof s, "%s#%d=%.9g", i ? " " : "", pc[1] + i, v);
            line += s;
        }
        return line.empty() ? 0 : mdi(line);
    }
    case PC_COMM_UPDATE_FIXTURE: {   // re-select the active system: the offsets its #vars now hold apply
        if (!st) return -1;
        static const char *g5x[] = {"G54", "G54", "G55", "G56", "G57", "G58", "G59", "G59.1", "G59.2", "G59.3"};
        int i = st->task.g5x_index;
        return mdi(g5x[i >= 1 && i <= 9 ? i : 1]);
    }
    // the tool table
    case PC_COMM_GET_TOOLTABLE_LENGTH:   return tool_get(pc[1], TF_LENGTH, pc[2]);
    case PC_COMM_GET_TOOLTABLE_DIAMETER: return tool_get(pc[1], TF_DIAMETER, pc[2]);
    case PC_COMM_GET_TOOLTABLE_OFFSETX:  return tool_get(pc[1], TF_OFFSETX, pc[2]);
    case PC_COMM_GET_TOOLTABLE_OFFSETY:  return tool_get(pc[1], TF_OFFSETY, pc[2]);
    case PC_COMM_SET_TOOLTABLE_LENGTH:   return tool_set(pc[1], TF_LENGTH, pc[2]);
    case PC_COMM_SET_TOOLTABLE_DIAMETER: return tool_set(pc[1], TF_DIAMETER, pc[2]);
    case PC_COMM_SET_TOOLTABLE_OFFSETX:  return tool_set(pc[1], TF_OFFSETX, pc[2]);
    case PC_COMM_SET_TOOLTABLE_OFFSETY:  return tool_set(pc[1], TF_OFFSETY, pc[2]);
    case PC_COMM_GET_TOOLTABLE_INDEX: {  // tool number and table index are one thing here
        CANON_TOOL_TABLE t;
        bool found = tool_entry(pc[1], t);
        if (!km->persist_set(pc[2], found ? pc[1] : -1)) return -1;
        return found ? 0 : -2;
    }
    case PC_COMM_G43: {
        char line[32];
        snprintf(line, sizeof line, "G43 H%d", pc[1]);
        return mdi(line);
    }
    case PC_COMM_G49: return mdi("G49");
    // strings for the board
    case PC_COMM_GET_GCODE_LINE: {
        if (!st || !st->task.file[0]) return -1;
        std::ifstream f(st->task.file);
        std::string line;
        for (int n = 0; n < st->task.motionLine && std::getline(f, line); n++) {}
        return f && km->gather_write_string(pc[1], line) ? 0 : -1;
    }
    case PC_COMM_GET_DATE_TIME: {
        char buf[64];
        time_t t = time(NULL);
        strftime(buf, sizeof buf, "%Y-%m-%d %H:%M:%S", localtime(&t));
        return km->gather_write_string(pc[1], buf) ? 0 : -1;
    }
    default:
        // KMotionCNC's own dialog: jog keys, controls, the dialog face, screen scripts, the
        // edit cell, an input box, geo correction, per-axis jog overrides, TP settings,
        // G43.4, tool comments - nothing to do here
        if (unsupported_seen.insert(code).second) pclog("pccomm: command %d is not supported\n", code);
        return -1;
    }
}

static const char *name(int code)
{
    switch (code) {
    case PC_COMM_ESTOP: return "ESTOP"; case PC_COMM_HALT: return "HALT"; case PC_COMM_EXECUTE: return "EXECUTE";
    case PC_COMM_SINGLE_STEP: return "SINGLE_STEP"; case PC_COMM_HALT_NEXT_LINE: return "HALT_NEXT_LINE";
    case PC_COMM_RESTART: return "RESTART"; case PC_COMM_SET_FRO: return "SET_FRO"; case PC_COMM_SET_FRO_INC: return "SET_FRO_INC";
    case PC_COMM_SET_RRO: return "SET_RRO"; case PC_COMM_SET_RRO_INC: return "SET_RRO_INC"; case PC_COMM_SET_SSO: return "SET_SSO";
    case PC_COMM_SET_SSO_INC: return "SET_SSO_INC"; case PC_COMM_MDI: return "MDI"; case PC_COMM_MCODE: return "MCODE";
    case PC_COMM_USER_BUTTON: return "USER_BUTTON"; case PC_COMM_MSG: return "MSG"; case PC_COMM_GET_DROS: return "GET_DROS";
    case PC_COMM_GET_MACHINE_COORDS: return "GET_MACHINE_COORDS"; case PC_COMM_GET_MISC_SETTINGS: return "GET_MISC_SETTINGS";
    case PC_COMM_GET_TOOL_SLOT_ID: return "GET_TOOL_SLOT_ID"; case PC_COMM_GETAXISRES: return "GETAXISRES";
    case PC_COMM_GET_TP_PARAM: return "GET_TP_PARAM"; case PC_COMM_GET_VARS: return "GET_VARS"; case PC_COMM_SET_VARS: return "SET_VARS";
    case PC_COMM_UPDATE_FIXTURE: return "UPDATE_FIXTURE"; case PC_COMM_GET_TOOLTABLE_INDEX: return "GET_TOOLTABLE_INDEX";
    case PC_COMM_G43: return "G43"; case PC_COMM_G49: return "G49"; case PC_COMM_GET_GCODE_LINE: return "GET_GCODE_LINE";
    case PC_COMM_GET_DATE_TIME: return "GET_DATE_TIME";
    default:
        if (code >= PC_COMM_SET_X && code <= PC_COMM_SET_V) return "SET_DRO";
        if (code >= PC_COMM_GET_TOOLTABLE_LENGTH && code <= PC_COMM_GET_TOOLTABLE_OFFSETY) return "TOOLTABLE";
        return "?";
    }
}

// ---- the dispatcher thread -------------------------------------------------------------------
static void pccomm_main()
{
    unsigned done_count = 0;        // the status count when the last result was written
    bool first = true;
    while (!stop_flag) {
        usleep(20000);
        KmState ks;
        km->state(ks);
        if (!ks.connected || ks.pc_comm_count == 0) continue;
        const int code = ks.pc_comm[0];
        if (first) {
            first = false;
            if (code > 0) {
                pclog("pccomm: command %d (%s) left over from before: refused\n", code, name(code));
                done_count = km->pc_result(-2);
            }
            continue;
        }
        if (code <= 0 || ks.pc_comm_count < done_count + 2) continue;
        int pc[8];
        memcpy(pc, ks.pc_comm, sizeof pc);
        pclog("pccomm: %s (%d) args %d %d %d\n", name(code), code, pc[1], pc[2], pc[3]);
        double t0 = now_s();
        int r = handle(pc);
        done_count = km->pc_result(r);
        pclog("pccomm: %s -> %d (%.3f s)\n", name(code), r, now_s() - t0);
    }
    nml_close();
}

void pccomm_start(KmBackend *backend, const KmPcCommConfig &c, void (*log)(const char *fmt, ...))
{
    km = backend;
    cfg = c;
    pclog = log;
    stop_flag = false;
    th = std::thread(pccomm_main);
}

void pccomm_stop()
{
    stop_flag = true;
    if (th.joinable()) th.join();
}
