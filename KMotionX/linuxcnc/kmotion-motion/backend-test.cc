// backend-test: exercises KmBackend alone (no LinuxCNC, no board), followed in real time
// at 5 ms as the protocol loop does. "make test" runs the first three scenarios:
//   backend-test          resume-test.ngc's moves: the planner's timing and blending
//   backend-test arcs     arc directions, planes and multiple turns as LinuxCNC encodes them
//   backend-test abort    an abort in mid-program, then a move after it
//   backend-test circles  two full circles as consecutive moves
//   backend-test quit     pause mid-run, then abort and tear the backend down (process exit)
//   backend-test gated    36 chords of a circle handed over the way task does it: only while
//                         fewer than 20 moves are pending (the planner must not stop at each)
//   backend-test letters  the first moves of LinuxCNC's axis.ngc on the kmotion-kogna settings
//                         (KM_KOGNA=1 selects those settings in simulate mode; KM_CORNER_TOL,
//                         KM_BREAK_ANGLE override the planner's corner settings)
//   backend-test arcspeed how fast the planner lets a circle of r=1 run
// Each check prints ok/FAIL; the exit status is the number of failures.
// KM_BOARD=1 in the environment runs the backend in board mode with the kmotion-kogna
// settings (2540 counts/in, 9.84 in/s, MinirouterInit.c): "abort" then measures the real
// stop on the Kogna, with KM_JOG0=1 adding the Jog<ch>=0 the protocol side used to send.
#include "kmotion-backend.h"
#include <array>
#include <chrono>
#include <deque>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <map>
#include <string>
#include <thread>

static int failures = 0;
static void check(bool ok, const char *what)
{
    printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) failures++;
}

struct Extent {
    double lo[3] = {1e9, 1e9, 1e9}, hi[3] = {-1e9, -1e9, -1e9};
    void add(const double *p)
    {
        for (int i = 0; i < 3; i++) { if (p[i] < lo[i]) lo[i] = p[i]; if (p[i] > hi[i]) hi[i] = p[i]; }
    }
};

// follows the backend at 5 ms; per-move extents, the path length, the messages
struct Follower {
    KmBackend &km;
    KmState st;
    double t = 0, vmax = 0, length = 0;
    int last_id = -1;
    bool trace = false;             // print every position change
    bool have_prev = false;
    double prev[3];
    std::map<int, Extent> extent;
    std::string messages;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    explicit Follower(KmBackend &k) : km(k) { memset(&st, 0, sizeof st); }
    // until the backend is idle (after a 0.5 s grace at the start) or until the given time
    double run(double until = 1e9, bool keep_going = false)
    {
        double last_report = t;
        for (;;) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            km.state(st);
            t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (st.message[0]) { printf("  message: %s\n", st.message); messages += st.message; messages += "\n"; }
            if (st.current_vel > vmax) vmax = st.current_vel;
            if (have_prev) length += sqrt(pow(st.pos[0] - prev[0], 2) + pow(st.pos[1] - prev[1], 2) + pow(st.pos[2] - prev[2], 2));
            if (trace && have_prev && fabs(st.pos[0] - prev[0]) + fabs(st.pos[1] - prev[1]) + fabs(st.pos[2] - prev[2]) > 1e-7)
                printf("    t=%6.3f X%.5f Y%.5f Z%.5f dX=%+.5f v=%.3f running %d\n", t, st.pos[0], st.pos[1], st.pos[2], st.pos[0] - prev[0], st.current_vel, st.running);
            memcpy(prev, st.pos, sizeof prev);
            have_prev = true;
            if (st.active_id) extent[st.active_id].add(st.pos);
            if (st.active_id != last_id) {
                printf("  t=%6.2f line %2d starts at X%.3f Y%.3f Z%.3f depth %d\n", t, st.active_id, st.pos[0], st.pos[1], st.pos[2], st.depth);
                last_id = st.active_id;
            }
            if (t - last_report >= 5) { last_report = t; printf("  t=%6.2f X%.3f Y%.3f Z%.3f v=%.3f\n", t, st.pos[0], st.pos[1], st.pos[2], st.current_vel); }
            if (t >= until) break;
            if (!keep_going && !st.running && t > 0.5) break;
        }
        return t;
    }
    bool at(double x, double y, double z, double tol = 1e-3) const
    {
        return fabs(st.pos[0] - x) < tol && fabs(st.pos[1] - y) < tol && fabs(st.pos[2] - z) < tol;
    }
};

// resume-test.ngc: G0 Z0.2, G1 X1 F20, G1 Y1, G3 full circle I-1 J0, G0 Z0.4, G0 X0 Y0
static void program_moves(KmBackend &km)
{
    double F = 20.0 / 60.0;
    double e1[9] = {0, 0, 0.2, 0,0,0,0,0,0};
    double e2[9] = {1, 0, 0.2, 0,0,0,0,0,0};
    double e3[9] = {1, 1, 0.2, 0,0,0,0,0,0};
    double e4[9] = {1, 1, 0.2, 0,0,0,0,0,0};
    double c4[3] = {0, 1, 0.2}, n4[3] = {0, 0, 1};
    double e5[9] = {1, 1, 0.4, 0,0,0,0,0,0};
    double e6[9] = {0, 0, 0.4, 0,0,0,0,0,0};
    km.line(e1, 5.657, 141.4, true, 3);
    km.line(e2, F, 100, false, 6);
    km.line(e3, F, 100, false, 7);
    km.arc(e4, c4, n4, 0, F, 100, 9);
    km.line(e5, 4, 100, true, 10);
    km.line(e6, 5.657, 141.4, true, 11);
    printf("moves handed over (the worker flushes 50 ms after the last one)\n");
}

static void scenario_program(KmBackend &km)
{
    program_moves(km);
    Follower f(km);
    f.run();
    printf("done at t=%.2f s: X%.4f Y%.4f Z%.4f, depth %d, max speed %.3f in/s, path %.3f in\n",
           f.t, f.st.pos[0], f.st.pos[1], f.st.pos[2], f.st.depth, f.vmax, f.length);
    check(f.at(0, 0, 0.4), "ends at X0 Y0 Z0.4");
    check(f.st.depth == 0, "queue empty at the end");
    check(f.extent.count(9) && f.extent[9].lo[0] > -1.02 && f.extent[9].lo[0] < -0.98, "the full circle reaches X-1");
    check(f.messages.empty(), "no messages");
}

// arcs as LinuxCNC's canon encodes them: the normal is the plane's axis, never flipped;
// turn = -1 is a clockwise arc, 0 counterclockwise, 1 counterclockwise with one extra turn
static void scenario_arcs(KmBackend &km)
{
    double V = 1.0;
    double nz[3] = {0, 0, 1}, ny[3] = {0, 1, 0};
    // 1: G2 quarter arc from (0,0) to (1,1) about (1,0): clockwise is the short way, through (0.29, 0.71)
    double e1[9] = {1, 1, 0, 0,0,0,0,0,0}, c1[3] = {1, 0, 0};
    km.arc(e1, c1, nz, -1, V, 100, 1);
    // 2: G3 quarter arc back to (0,0) about the same center: counterclockwise
    double e2[9] = {0, 0, 0, 0,0,0,0,0,0};
    km.arc(e2, c1, nz, 0, V, 100, 2);
    // 3: G3 P2 from (0,0) back to (0,0) about (0.5,0): two full circles, 2 pi long
    double c3[3] = {0.5, 0, 0};
    km.arc(e2, c3, nz, 1, V, 100, 3);
    // 4: G18 G2 from (0,0,0) to (1,0,1) about (0,0,1): clockwise about +Y is the short way, through X0.71 Z0.29
    double e4[9] = {1, 0, 1, 0,0,0,0,0,0}, c4[3] = {0, 0, 1};
    km.arc(e4, c4, ny, -1, V, 100, 4);
    printf("arcs handed over\n");
    Follower f(km);
    f.run();
    double expect = 3 * M_PI / 2 + 2 * M_PI;
    printf("done at t=%.2f s: X%.4f Y%.4f Z%.4f, path %.3f in (expected %.3f)\n", f.t, f.st.pos[0], f.st.pos[1], f.st.pos[2], f.length, expect);
    for (auto &e : f.extent)
        printf("  line %d: X %.3f..%.3f  Y %.3f..%.3f  Z %.3f..%.3f\n", e.first, e.second.lo[0], e.second.hi[0], e.second.lo[1], e.second.hi[1], e.second.lo[2], e.second.hi[2]);
    check(f.at(1, 0, 1), "ends at X1 Y0 Z1");
    check(fabs(f.length - expect) < 0.02 * expect, "path length = 3 quarter arcs + 2 circles");
    check(f.extent.count(1) && f.extent[1].lo[1] > -0.02 && f.extent[1].hi[0] < 1.02, "G2 quarter arc takes the short way (stays in 0..1, never Y-1)");
    check(f.extent.count(2) && f.extent[2].lo[1] > -0.02 && f.extent[2].hi[0] < 1.02, "G3 quarter arc takes the short way");
    check(f.extent.count(3) && f.extent[3].lo[1] < -0.48 && f.extent[3].hi[0] > 0.98 && f.extent[3].hi[0] < 1.02, "double circle covers its circle (Y-0.5, X1)");
    check(f.extent.count(4) && f.extent[4].lo[0] > -0.02 && f.extent[4].hi[2] < 1.02, "G18 G2 arc is clockwise about +Y (never X-1)");
    check(f.messages.empty(), "no messages");
}

// two full circles as two consecutive LinuxCNC moves (G3 I.5 twice): does the planner keep both?
static void scenario_circles(KmBackend &km)
{
    double V = 1.0, nz[3] = {0, 0, 1};
    double e[9] = {0, 0, 0, 0,0,0,0,0,0}, c[3] = {0.5, 0, 0};
    km.arc(e, c, nz, 0, V, 100, 1);
    km.arc(e, c, nz, 0, V, 100, 2);
    double e3[9] = {1, 0, 0, 0,0,0,0,0,0};
    km.line(e3, V, 100, false, 3);
    printf("circles handed over\n");
    Follower f(km);
    f.run();
    double expect = 2 * M_PI + 1;
    printf("done at t=%.2f s: X%.4f Y%.4f Z%.4f, path %.3f in (expected %.3f), max speed %.3f\n", f.t, f.st.pos[0], f.st.pos[1], f.st.pos[2], f.length, expect, f.vmax);
    check(f.at(1, 0, 0), "ends at X1 Y0 Z0");
    check(fabs(f.length - expect) < 0.02 * expect, "path length = two circles + 1");
    check(f.messages.empty(), "no messages");
}

// how fast does the planner let a circle run? r=1 at the axis limit (4 in/s)
static void scenario_arcspeed(KmBackend &km)
{
    double nz[3] = {0, 0, 1};
    double e[9] = {0, 0, 0, 0,0,0,0,0,0}, c[3] = {1, 0, 0};
    km.arc(e, c, nz, 0, 4.0, 100, 1);
    printf("r=1 circle at F=4 handed over\n");
    Follower f(km);
    f.run();
    printf("done at t=%.2f s: path %.3f in, max speed %.3f in/s (commanded 4)\n", f.t, f.length, f.vmax);
    check(fabs(f.length - 2 * M_PI) < 0.05, "one circle of r=1");
}

// what kmotion-motion does at exit, with a run paused: abort, wait for idle, delete
static void scenario_quit(KmBackend *&km)
{
    Follower f(*km);
    if (km->is_board()) {
        // a 30 s move: the download is still streaming (and blocked by the board's flow
        // control once paused) when the teardown comes - the case that hung the process
        f.km.state(f.st);
        double e[9]; memcpy(e, f.st.pos, sizeof e);
        e[0] += (e[0] > 5.0) ? -10.0 : 10.0;
        km->line(e, 0.333, 50, false, 5);
    } else {
        program_moves(*km);
    }
    f.run(1.0);
    km->pause(true);
    f.run(f.t + 0.5, true);
    printf("  paused at X%.4f Y%.4f Z%.4f; abort + teardown\n", f.st.pos[0], f.st.pos[1], f.st.pos[2]);
    auto t0 = std::chrono::steady_clock::now();
    km->abort();
    KmState st;
    for (int i = 0; i < 200; i++) { km->state(st); if (!st.running) break; std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
    double t_idle = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    delete km;
    km = nullptr;
    double t_down = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    printf("  idle after %.2f s, torn down after %.2f s\n", t_idle, t_down);
    check(!st.running, "idle after the abort");
    check(t_down < 2.0, "teardown while paused finishes within 2 s");
}

// the start of axis.ngc as task sent it (inches; rapids at 9.84 in/s, F100 = 0.0656 in/s,
// F400 = 0.2625 in/s): Z up, rapid to the first letter, plunge, the first strokes
static void scenario_letters(KmBackend &km)
{
    double up = 0.11811, down = -0.0787402, R = 9.84, A = 98.4, F100 = 100 / 25.4 / 60, F400 = 400 / 25.4 / 60;
    double p[9] = {1.4591, 0.601612, up, 0,0,0,0,0,0};
    km.line(p, R, A, true, 1);                                   // G0 Z up (from Z-0.0787)
    double q[9] = {0.0692051, 0.019685, up, 0,0,0,0,0,0};
    km.line(q, R, A, true, 2);                                   // G0 X Y
    q[2] = down; km.line(q, F100, A, false, 3);                  // G1 Z plunge F100
    double s4[9] = {0.234452, 0.808778, down, 0,0,0,0,0,0}; km.line(s4, F400, A, false, 4);
    double s5[9] = {0.396469, 0.808778, down, 0,0,0,0,0,0}; km.line(s5, F400, A, false, 5);
    double s6[9] = {0.259212, 0.151559, down, 0,0,0,0,0,0}; km.line(s6, F400, A, false, 6);
    double s7[9] = {0.660756, 0.151559, down, 0,0,0,0,0,0}; km.line(s7, F400, A, false, 7);
    double s8[9] = {0.633305, 0.019685, down, 0,0,0,0,0,0}; km.line(s8, F400, A, false, 8);
    double s9[9] = {0.0692051, 0.019685, down, 0,0,0,0,0,0}; km.line(s9, F400, A, false, 9);
    double s10[9] = {0.0692051, 0.019685, up, 0,0,0,0,0,0}; km.line(s10, R, A, true, 10);   // G0 Z up
    double s11[9] = {0.737189, 0.019685, up, 0,0,0,0,0,0}; km.line(s11, R, A, true, 11);     // G0 X
    double s12[9] = {0.737189, 0.019685, down, 0,0,0,0,0,0}; km.line(s12, F100, A, false, 12); // plunge
    printf("letters handed over\n");
    Follower f(km);
    f.run();
    printf("done at t=%.2f s (the moves alone: Z 0.197/0.0656 = 3.0 s, strokes 2.95 in/0.2625 = 11.2 s, rapids ~2 s)\n", f.t);
}

// task hands the next move over only when the motion queue is not full. With a small limit
// that means one new move each time one finishes; the backend must not take those gaps for
// the end of the program (it did: a flush, hence a stop, after every move)
static void scenario_gated(KmBackend &km)
{
    const int limit = 20;                 // = cfg.queue_limit set in main for this scenario
    double F = 400 / 25.4 / 60, A = 98.4;
    std::deque<std::array<double, 9>> moves;
    for (int k = 1; k <= 36; k++) {
        double a = k * 10.0 * M_PI / 180.0;
        moves.push_back({cos(a) - 1.0, sin(a), 0, 0, 0, 0, 0, 0, 0});    // a circle of r=1 about (-1,0) in 10-degree chords
    }
    Follower f(km);
    int id = 1;
    while (!moves.empty()) {
        km.state(f.st);
        if (f.st.depth < limit) {
            km.line(moves.front().data(), F, A, false, id++); moves.pop_front();
            if (getenv("KM_DEBUG_FLUSH")) printf("    t=%.3f handed over move %d (depth was %d)\n", f.t, id - 1, f.st.depth);
        }
        f.run(f.t + 0.005, true);
    }
    printf("  all 36 chords handed over at t=%.2f s\n", f.t);
    f.run();
    printf("done at t=%.2f s: X%.4f Y%.4f (a full circle at 0.2625 in/s takes 23.9 s)\n", f.t, f.st.pos[0], f.st.pos[1]);
    check(f.at(0, 0, 0, 2e-3), "ends where it started");
    check(f.t < 27, "no stops between the chords (time close to the ideal)");
}

static void scenario_abort(KmBackend &km)
{
    bool board = km.is_board();
    Follower f(km);
    if (board) {
        // one feed move along X from where the machine is, 3 s long
        f.km.state(f.st);
        double e[9]; memcpy(e, f.st.pos, sizeof e);
        e[0] += (e[0] > 1.0) ? -1.0 : 1.0;
        printf("board: X%.4f -> X%.4f at 0.333 in/s\n", f.st.pos[0], e[0]);
        km.line(e, 0.333, 50, false, 5);
    } else {
        program_moves(km);
    }
    double t_abort = f.run(board ? 1.5 : 2.0);
    double at[3];
    memcpy(at, f.st.pos, sizeof at);
    printf("  t=%.2f abort at X%.3f Y%.3f Z%.3f (line %d, depth %d)\n", t_abort, at[0], at[1], at[2], f.st.active_id, f.st.depth);
    check(f.st.running && f.st.depth > 0, "the program was running when aborted");
    km.abort();
    if (board && getenv("KM_JOG0")) km.jog_stop(-1);
    f.trace = board;
    double t_idle = f.run(t_abort + 2.0);
    printf("  t=%.2f idle: X%.3f Y%.3f Z%.3f depth %d running %d\n", t_idle, f.st.pos[0], f.st.pos[1], f.st.pos[2], f.st.depth, f.st.running);
    check(!f.st.running && f.st.depth == 0, "idle with an empty queue after the abort");
    if (board) {
        // keep watching: does the position change after the board came to rest?
        double at_rest[3]; memcpy(at_rest, f.st.pos, sizeof at_rest);
        double t_rest = f.t;
        f.run(t_idle + 2.0, true);
        printf("  t=%.2f X%.5f Y%.5f Z%.5f: moved %+.5f in X since idle (t=%.2f)\n", f.t, f.st.pos[0], f.st.pos[1], f.st.pos[2], f.st.pos[0] - at_rest[0], t_rest);
        check(fabs(f.st.pos[0] - at_rest[0]) < 1e-4, "no movement after the stop");
        // the move after the stop must start from where the board stands, not from where
        // the planner was before the abort: watch for a jump in the first 0.6 s
        double e2[9]; memcpy(e2, f.st.pos, sizeof e2);
        e2[0] += 0.3;
        printf("  move after the stop: X%.4f -> X%.4f at 0.333 in/s\n", f.st.pos[0], e2[0]);
        km.line(e2, 0.333, 50, false, 6);
        double before[3]; memcpy(before, f.st.pos, sizeof before);
        double biggest = 0, t_start = f.t;
        f.trace = true;
        while (f.t < t_start + 0.6) {
            f.run(f.t + 0.005, true);
            double step = fabs(f.st.pos[0] - before[0]);
            if (step > biggest) biggest = step;
            memcpy(before, f.st.pos, sizeof before);
        }
        f.trace = false;
        f.run();
        printf("  biggest position step while starting: %.4f in (one 20 ms status step at 0.333 in/s is 0.0067)\n", biggest);
        check(biggest < 0.03, "the move after the stop starts where the board stopped (no jump)");
        check(fabs(f.st.pos[0] - e2[0]) < 1e-3, "and ends at its target");
        return;
    }
    check(t_idle - t_abort < 0.1, "idle within 100 ms (simulate: nothing to decelerate)");
    check(f.messages.find("failed") == std::string::npos, "no failure reported for the abort");
    check(f.st.errors == 0, "no error counted for the abort");
    // the planner is usable afterwards: a rapid home from wherever we stopped
    double home[9] = {0, 0, 0, 0,0,0,0,0,0};
    km.line(home, 5.657, 141.4, true, 20);
    f.t0 = std::chrono::steady_clock::now();
    f.t = 0;
    f.run();
    check(f.extent.count(20) && f.at(0, 0, 0), "a move after the abort runs and ends at X0 Y0 Z0");
    check(f.messages.empty(), "no messages");
}

int main(int argc, char **argv)
{
    const char *scenario = argc > 1 ? argv[1] : "program";
    KmConfig cfg;
    cfg.log_segments = true;
    for (int i = 0; i < 3; i++) { cfg.axis[i].counts_per_unit = 4000; cfg.axis[i].max_vel = 4; cfg.axis[i].max_accel = 100; cfg.axis[i].max_jerk = 1000; }
    if (getenv("KM_KOGNA")) {               // the kmotion-kogna settings, simulate mode
        for (int i = 0; i < 3; i++) { cfg.axis[i].counts_per_unit = 2540; cfg.axis[i].max_vel = 9.84; cfg.axis[i].max_accel = 98.4; cfg.axis[i].max_jerk = 984; }
    }
    if (getenv("KM_CORNER_TOL")) cfg.corner_tol = atof(getenv("KM_CORNER_TOL"));
    if (getenv("KM_COLLINEAR_TOL")) cfg.collinear_tol = atof(getenv("KM_COLLINEAR_TOL"));
    if (getenv("KM_JERK")) for (int i = 0; i < 3; i++) cfg.axis[i].max_jerk = atof(getenv("KM_JERK"));
    if (getenv("KM_BREAK_ANGLE")) cfg.break_angle = atof(getenv("KM_BREAK_ANGLE"));
    if (strcmp(scenario, "gated") == 0) cfg.queue_limit = 20;     // the backend must know task's limit
    if (getenv("KM_BOARD")) {
        cfg.simulate = false;
        snprintf(cfg.init_program, sizeof cfg.init_program, "%s", "/home/tk/KMotionXCNC/settings/c-programs/MinirouterInit.c");
        cfg.init_thread = 1;
        cfg.status_period = getenv("KM_STATUS_MS") ? atof(getenv("KM_STATUS_MS")) / 1000.0 : 0.02;    // 5 ms polling stalls the Kogna link
        for (int i = 0; i < 3; i++) { cfg.axis[i].counts_per_unit = 2540; cfg.axis[i].max_vel = 9.84; cfg.axis[i].max_accel = 98.4; cfg.axis[i].max_jerk = 984; }
    }
    double pos[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    KmBackend *km = new KmBackend;
    if (!km->init(cfg, pos)) { printf("init failed\n"); return 1; }
    printf("mode: %s, scenario: %s\n", km->mode_name(), scenario);
    if (getenv("KM_PATH_TOL")) km->set_path_mode(2, atof(getenv("KM_PATH_TOL")));   // as G64 P<tol> would
    if (strcmp(scenario, "arcs") == 0) scenario_arcs(*km);
    else if (strcmp(scenario, "abort") == 0) scenario_abort(*km);
    else if (strcmp(scenario, "circles") == 0) scenario_circles(*km);
    else if (strcmp(scenario, "arcspeed") == 0) scenario_arcspeed(*km);
    else if (strcmp(scenario, "quit") == 0) scenario_quit(km);
    else if (strcmp(scenario, "letters") == 0) scenario_letters(*km);
    else if (strcmp(scenario, "gated") == 0) scenario_gated(*km);
    else scenario_program(*km);
    if (km) {
        auto t0 = std::chrono::steady_clock::now();
        delete km;
        double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        printf("  teardown took %.2f s\n", dt);
        check(dt < 2.0, "teardown finishes within 2 s");
    }
    printf("%s: %d failure(s)\n", scenario, failures);
    return failures;
}
