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
 * Moves and settings are queued to the worker; feedhold, resume, abort and jogs go
 * to the board straight away (the KMotion pipe serializes the two threads).
 *
 * Units are LinuxCNC's machine units (the configs here are inches, which is also
 * what KMotion's planner uses; a mm machine needs the conversion added).
 * Axis order everywhere: x y z a b c u v (w has no KMotion equivalent).
 */
#pragma once

struct KmAxisParams {
    double counts_per_unit = 0;   // 0: axis not in use
    double max_vel = 0;           // units/s
    double max_accel = 0;         // units/s^2
    double max_jerk = 0;          // units/s^3, 0 = 10 x accel
};

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
    // board mode
    char init_program[512] = "";  // C program run once at start (axis setup), "" for none
    int init_thread = 1;
    double status_period = 0.02;  // board status poll period, seconds
};

struct KmState {
    double pos[9];                // x y z a b c u v w
    bool enabled[8];              // board: axis enabled (always true in simulate mode)
    int depth;                    // moves handed over and not finished yet
    int active_id;                // LinuxCNC id of the move executing (0 if none)
    bool running;                 // something is executing or still being planned
    bool paused;
    bool connected;               // board mode: status is coming from the board
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
    // drop everything, stop where we are
    void abort();
    void pause(bool on);
    void set_feed_override(double feed_scale, double rapid_scale);
    // board mode
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
