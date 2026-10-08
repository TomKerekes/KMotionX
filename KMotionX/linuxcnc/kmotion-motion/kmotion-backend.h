/*
 * kmotion-backend.h - the KMotion side of kmotion-motion.
 *
 * kmotion-motion.cc includes only LinuxCNC's headers, kmotion-backend.cc only
 * KMotion's: the two code bases share rs274ngc ancestry and some type names, so
 * they meet only through the plain C++ types in this header.
 *
 * The backend runs KMotion in its own thread: CoordMotion's calls block while the
 * board's buffer is full (that is its flow control), and LinuxCNC's task times a
 * motion command out after a second, so the protocol loop must never wait on them.
 * Moves and settings are queued to the worker; feed hold, resume, the stop of an abort
 * and jogs go to the board through a small command thread of their own, in order.
 *
 * Units are LinuxCNC's machine units (the configs here are inches, which is also
 * what KMotion's planner uses; a mm machine needs the conversion added).
 * Axis order everywhere: x y z a b c u v (w has no KMotion equivalent).
 */
#pragma once

#include <string>
#include <utility>
#include <vector>

struct KmAxisParams {
    double counts_per_unit = 0;   // 0: axis not in use
    double max_vel = 0;           // units/s
    double max_accel = 0;         // units/s^2
    double max_jerk = 0;          // units/s^3, 0 = 10 x accel
};

// Spindle control as KMotionCNC's Tool Setup does it for M3, M4, M5 and S (Dynomotion's help
// page "KMotionCNC Spindle Control"): each is one of KMotion's M-code actions, carried out the
// way the G-code interpreter's InvokeActionDirect does. The numbers are GCodeInterpreter.h's
// M_Action_*.
enum KmActionType {
    KM_ACTION_NONE = 0,
    KM_ACTION_SETBIT = 1,         // p: bit state
    KM_ACTION_SETTWOBITS = 2,     // p: bit state bit state
    KM_ACTION_DAC = 3,            // p: dac scale offset min max: RPM x scale + offset, clamped
    KM_ACTION_PROGRAM = 4,        // p: thread var; file: a .c (compiled and loaded each time) or a .out
    KM_ACTION_PROGRAM_WAIT = 5,   // ... and wait until the thread is done
    KM_ACTION_PROGRAM_WAIT_SYNC = 6   // ... wait, then re-read the axis positions
};
struct KmAction {
    int type = KM_ACTION_NONE;
    double p[5] = {};             // MCODE_ACTION.dParams
    char file[512] = "";
};
enum { KM_SPINDLE_M3, KM_SPINDLE_M4, KM_SPINDLE_M5, KM_SPINDLE_S, KM_SPINDLE_ACTIONS };
// where the board tells that the spindle is at speed (SPINDLE_AT_SPEED)
enum { KM_AT_SPEED_NONE, KM_AT_SPEED_BIT, KM_AT_SPEED_AXIS };
// how a probe move (G38.2-G38.5) ended
enum { KM_PROBE_NO_CONTACT,   // ran to its end without the probe tripping
       KM_PROBE_TRIPPED,      // stopped by the probe (KmState.probe_pos: where)
       KM_PROBE_ALREADY,      // the probe already was in the state sought: no move
       KM_PROBE_FAILED,       // the program or the board failed (message posted)
       KM_PROBE_ABORTED };

struct KmConfig {
    bool simulate = true;         // true: plan and time without a board, replay the plan
                                  // false: KMotionServer and the board (KFLOP/Kogna)
    bool third_order = true;      // 3rd Order Planner (TP3)
    bool cubic_knots = true;
    bool actuator_limits = true;  // actuator-space limits, one actuator per axis
    bool log_segments = false;    // write /tmp/TPSegLog.csv for the plotter
    double break_angle = 30.0;
    double collinear_tol = 0.0002;
    double corner_tol = 0.0005;
    double facet_angle = 0.5;
    double lookahead = 3.0;
    KmAxisParams axis[8];
    int channel[8] = {0, 1, 2, -1, -1, -1, -1, -1};   // board channel per axis (DefineCS)
    int queue_limit = 500;        // LinuxCNC moves the protocol side lets task queue: while that
                                  // many are pending, task is waiting for us, not out of moves
    // board mode
    std::vector<std::string> init_programs;   // C programs run once at start, in order (axis setup)
    int init_thread = 1;
    // C programs started after the init programs and left running, {thread, file}: forever
    // loops on the board, e.g. Dynomotion's ServiceCSS() for G96
    std::vector<std::pair<int, std::string>> start_programs;
    double status_period = 0.02;  // board status poll period, seconds
    char home_program[512] = "";  // C program that homes (see configs/kmotion-kogna/Home.c), "" = none:
                                  // joints are then declared homed where they stand
    int home_thread = 2;
    double home_timeout = 120;    // seconds the program may take
    KmAction spindle[KM_SPINDLE_ACTIONS];   // LinuxCNC's spindle 0 (all NONE: the board is not told)
    // the spindle's measured speed (board mode): from the axis channel that turns it, either its
    // commanded Dest (a jogged spindle's real ramps, no encoder needed) or its encoder Position
    int spindle_speed_axis = -1;              // board channel, -1 = not measured
    bool spindle_speed_from_dest = true;      // false: Position
    double spindle_counts_per_rev = 0;
    double spindle_speed_tau = 0.1;           // seconds, low-pass filter; 0 = none
    // G96 constant surface speed, handed to the board as KMotionCNC does it: X's offset, the
    // surface speed and the maximum RPM in persist 111-114 and the mode in 110, for a
    // ServiceCSS() loop there (Dynomotion's C Programs/SpindleUsingJogs/CSS/CSSJog.c).
    // false: G96 is refused with an error
    bool spindle_css = false;
    // the spindle at speed, read on the board: an input bit at a level (e.g. the spindle
    // drive's at-speed output), or a jogged spindle axis at its commanded speed: its Dest's
    // speed between two status reads, on the board's own clock, within 1% (at least 1 RPM) of
    // what the spindle was told - S, or in G96 what ServiceCSS() makes of X (a jog's Done stays
    // false while it holds its speed, so it can't tell). NONE: the board doesn't tell
    int spindle_at_speed = KM_AT_SPEED_NONE;
    int spindle_at_speed_bit = -1;            // BIT: the I/O bit and the level meaning at speed
    int spindle_at_speed_level = 1;
    int spindle_at_speed_axis = -1;           // AXIS: the board channel (spindle_counts_per_rev converts)
    // board input bits whose states every status read publishes (KmState.input_states)
    std::vector<int> input_bits;              // at most 64, each one the status carries
    // LinuxCNC's user M codes M100-M199 (index n - 100): KMotionCNC's M-code actions; P and Q
    // go to the action's persist variable and the next one, as floats
    KmAction mcode[100];
    // G38.2-G38.5: the probe watcher ServiceProbe() (configs/kmotion-kogna/probe/ProbeService.c)
    // runs in the board's forever loop; armed for each probe move, which the planner runs like
    // any move (so any kinematics apply), it records the axes' Dests on the trip and stops the
    // coordinated motion. probe_bit -1 = probe moves run as plain moves that never trip
    int probe_bit = -1;                       // the probe's input bit (one the status carries) ...
    int probe_level = 1;                      // ... and the level that means contact
};

// the board's status carries this I/O bit (SPINDLE_AT_SPEED = BIT)
bool km_status_has_bit(int bit);

// G96 for the board (KmBackend::spindle), in machine units (inches here)
struct KmSpindleCss {
    double surface_speed = 0;     // at the tool, units/s, the spindle override included
    double max_rpm = 1e9;         // G96 D, within the [SPINDLE_0] limit for the direction
    double x_offset = 0;          // machine X of radius 0 (LinuxCNC's G5x + G92 + tool X offsets)
};

// homing through the board (HOME_PROGRAM): LinuxCNC's per-joint homing parameters, in
// machine units, as task sends them. The backend converts to counts and hands them to the
// program through the board's persist variables (layout documented in Home.c).
struct KmHomeJoint {
    bool home = false;            // this joint is to be homed
    double search_vel = 0;        // units/s, the sign is the direction toward the switch
    double latch_vel = 0;         // units/s; same sign as the search: approach again slowly,
                                  // opposite sign: the switch's trailing edge is home
    double final_vel = 0;         // units/s for the move to home_pos, 0 = the axis default
    double offset = 0;            // position the switch edge gets (HOME_OFFSET)
    double home_pos = 0;          // where to go afterwards (HOME)
    double max_travel = 0;        // search distance limit, 0 = none
    bool use_index = false;
    bool no_final_move = false;
    int sequence = 0;             // homing order, lower first
};
struct KmHomeRequest {
    KmHomeJoint joint[8];
};

struct KmState {
    double pos[9];                // x y z a b c u v w
    bool enabled[8];              // board: axis enabled (always true in simulate mode)
    int depth;                    // moves handed over and not finished yet
    int active_id;                // LinuxCNC id of the move executing (0 if none)
    bool running;                 // something is executing or still being planned
    bool paused;
    bool connected;               // board mode: status is coming from the board
    int errors;                   // planner failures so far; each one stopped the motion
    bool homing;                  // board: the home program is running
    int home_serial;              // counts finished homing runs; the masks below belong to the last
    unsigned home_ok_mask;        // joints homed by it (bit = LinuxCNC joint)
    unsigned home_fail_mask;      // joints it did not home (switch not found, aborted, program failed)
    int spindle_state;            // board: the spindle as last given to it: 1 CW, -1 CCW, 0 off, 2 not yet
    double spindle_rpm;           // ... what its S action got: the RPM, in CSS mode the surface speed
    bool spindle_css;             // board: in CSS mode (persist 110 = 2, G96)
    bool spindle_at_speed;        // at speed as of the last status read (always, without SPINDLE_AT_SPEED)
    int spindle_at_speed_done;    // ... a read made once this many spindle commands were done (cf. spindle_done)
    unsigned status_count;        // board: status reads so far
    unsigned long long input_states;  // board: bit i = KmConfig.input_bits[i] in the last status read
    bool probe_contact;           // board: the probe bit at its contact level, in the last status read
    int probe_serial;             // counts finished probe moves; the outcome and position are the last one's
    int probe_outcome;            // KM_PROBE_*
    double probe_pos[9];          // KM_PROBE_TRIPPED: where the axes were when it tripped
    int spindle_done;             // counts spindle() calls carried out (or superseded)
    double spindle_rpm_measured;  // board: the spindle axis's speed, signed RPM (0 when not measured)
    unsigned jog_busy;            // board: bit per axis, a jog the board has not finished yet
    double current_vel;
    double distance_to_go;
    char message[512];            // non-empty: a message for the operator (cleared by the call)
};

class KmBackend {
public:
    KmBackend();
    ~KmBackend();
    // start: planner parameters and the machine's current position (simulate mode);
    // in board mode this connects, runs the init program and reads the position
    bool init(const KmConfig &cfg, const double pos[9]);
    bool is_board() const;
    // simulate mode: the machine "moved" by other means (jog, homing) while idle
    void set_position(const double pos[9]);
    int line(const double end[9], double vel, double acc, bool rapid, int id);
    int arc(const double end[9], const double center[3], const double normal[3], int turn, double vel, double acc, int id);
    int dwell(double seconds, int id);
    // LinuxCNC's SET_TERM_COND (G61/G61.1/G64 P): 1 = exact stop at block ends, 2 = blend;
    // tolerance in machine units, 0 = the planner's own corner tolerance
    void set_path_mode(int term_cond, double tolerance);
    // drop everything, stop where we are
    void abort();
    void pause(bool on);
    void set_feed_override(double feed_scale, double rapid_scale);
    // board mode
    bool has_home_program() const;
    void home(const KmHomeRequest &req);         // runs HOME_PROGRAM; the result comes through state()
    bool has_spindle() const;                    // SPINDLE_* actions configured
    // LinuxCNC's spindle 0: state 1 CW, -1 CCW, 0 off; rpm = |speed| after the override and
    // the limits. Runs S and then M3/M4 when it turns on, S on a speed change, M5 when it
    // stops. css: G96 constant surface speed (SPINDLE_CSS; refused without it): the board
    // gets it through persist 110-114 first, and S gets the surface speed in units/s, as
    // KMotionCNC passes it; without css (G97) persist 110 goes back to RPM mode.
    void spindle(int state, double rpm, const KmSpindleCss *css = nullptr);
    void set_bit(int bit, bool on);              // board: an output bit, in order with the jogs and stops
    // user M code n (100-199): runs mcode[n - 100] with P and Q, in order with the other board
    // work; the ticket is finished once the action is over (thread-safe, unlike state())
    int mcode(int n, double p, double q);
    bool mcode_finished(int ticket, int &result);   // result: 0 done, 1 failed (message posted), 2 aborted
    bool has_probe() const;                      // board mode with PROBE_BIT
    // a probe move to end (G38.2/.3: until contact, when_clears G38.4/.5: until contact is
    // lost); the result comes through state() (probe_serial, probe_outcome, probe_pos)
    void probe(const double end[9], double vel, double acc, int id, bool when_clears);
    void machine_on(bool on);                    // enable/disable the axes
    void jog(int axis, double vel);              // continuous jog, units/s; 0 stops
    void jog_to(int axis, double target, double vel);
    void jog_stop(int axis);                     // -1: all
    // the latest state (the worker keeps it current)
    void state(KmState &out);
    const char *mode_name() const;

    struct Impl;                  // the KMotion-side state, defined in kmotion-backend.cc

private:
    Impl *d;
};
