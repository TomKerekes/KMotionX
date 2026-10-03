/*
 * kmotion-backend.h - the KMotion side of kmotion-motion.
 *
 * kmotion-motion.cc includes only LinuxCNC's headers, kmotion-backend.cc only
 * KMotion's: the two code bases share rs274ngc ancestry and some type names, so
 * they meet only through the plain C++ types in this header.
 *
 * Units are LinuxCNC's machine units (the config here is inches, which is also
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
    bool simulate = true;         // plan and time without a board, replay the plan (the only mode yet)
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
};

struct KmState {
    double pos[9];                // x y z a b c u v w
    int depth;                    // moves handed over and not finished yet
    int active_id;                // LinuxCNC id of the move executing (0 if none)
    bool running;                 // something is executing or still being planned
    double current_vel;
    double distance_to_go;
    char message[512];            // non-empty: a message for the operator (cleared by the call)
};

class KmBackend {
public:
    KmBackend();
    ~KmBackend();
    // start: planner parameters and the machine's current position
    bool init(const KmConfig &cfg, const double pos[9]);
    // the machine moved by other means (jog, homing) while nothing was planned
    void set_position(const double pos[9]);
    int line(const double end[9], double vel, double acc, bool rapid, int id);
    int arc(const double end[9], const double center[3], const double normal[3], int turn, double vel, double acc, int id);
    int dwell(double seconds, int id);
    // finish planning everything handed over so far (needed before the last moves run)
    int flush();
    bool needs_flush() const;
    // drop everything, stop where we are
    void abort();
    void pause(bool on);
    void set_feed_override(double feed_scale, double rapid_scale);
    // advance the execution by dt seconds and report
    void step(double dt, KmState &out);
    const char *mode_name() const;

    struct Impl;                  // the KMotion-side state, defined in kmotion-backend.cc

private:
    Impl *d;
};
