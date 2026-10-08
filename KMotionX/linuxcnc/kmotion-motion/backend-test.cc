// backend-test: exercises KmBackend alone (no LinuxCNC, no board), followed in real time
// at 5 ms as the protocol loop does. "make test" runs the first three scenarios:
//   backend-test          resume-test.ngc's moves: the planner's timing and blending
//   backend-test arcs     arc directions, planes and multiple turns as LinuxCNC encodes them
//   backend-test abort    an abort in mid-program, then a move after it
//   backend-test circles  two full circles as consecutive moves
//   backend-test quit     pause mid-run, then abort and tear the backend down (process exit)
//   backend-test gated    36 chords of a circle handed over the way task does it: only while
//                         fewer than 20 moves are pending (the planner must not stop at each)
//   backend-test home     (KM_BOARD=1 only) homes X through Home.c with a short search travel:
//                         on a board without a switch the search gives up after that travel
//                         and the result is "failed", which exercises the whole path;
//                         KM_HOME_ABORT=1 aborts the homing after a second instead
//   backend-test spindle  LinuxCNC's spindle through the bench's Spindle Using Jogs programs
//                         (configs/kmotion-kogna/spindle; KM_BOARD=1): channel 3 turns at
//                         RPM x 1000/60 counts/s, CW positive, CCW negative; then G96 through
//                         Dynomotion's ServiceCSS() in BenchLoop.c (thread 1) as X moves, its
//                         D cap, and G97; at speed (SPINDLE_AT_SPEED = AXIS 3) only once each
//                         ramp is over. Without a board: G96 refused without SPINDLE_CSS
//   backend-test jog      (KM_BOARD=1 only) jogs X as the protocol side does: a continuous jog
//                         runs until it is stopped and the board has slowed to a stop, an
//                         increment until the board is at the target (at the jog speed), a
//                         stop ends an increment early, and an increment of 0 ends at once
//   backend-test atbit    (KM_BOARD=1 only) SPINDLE_AT_SPEED = BIT 1030 1: the at-speed state
//                         follows virtual bit 1030 as the test sets and clears it
//   backend-test io       (KM_BOARD=1 only) an output bit (set_bit 1026) read back as an input
//                         bit, and the user M code M100 (configs/kmotion-kogna/mcodes/M100.c in
//                         thread 5, P and Q in persist 10 and 11), which sets virtual bit 1027
//                         when P is not 0
//   backend-test probe    (KM_BOARD=1 only) G38.x through the planner, with ServiceProbe()
//                         (probe/ProbeService.c) in BenchLoop.c's forever loop in thread 1
//                         watching virtual bit 1031 (contact = 1), which the test sets and
//                         clears during the moves: a trip toward, a trip away, the probe
//                         already in the state sought, and a move without contact
//   backend-test letters  the first moves of LinuxCNC's axis.ngc on the kmotion-kogna settings
//                         (KM_KOGNA=1 selects those settings in simulate mode; KM_CORNER_TOL,
//                         KM_BREAK_ANGLE override the planner's corner settings)
//   backend-test arcspeed how fast the planner lets a circle of r=1 run
// Each check prints ok/FAIL; the exit status is the number of failures.
// KM_BOARD=1 in the environment runs the backend in board mode with the kmotion-kogna
// settings (2540 counts/in, 9.84 in/s, MinirouterInit.c): "abort" then measures the real
// stop on the Kogna, with KM_JOG0=1 adding the Jog<ch>=0 the protocol side used to send.
#include "kmotion-backend.h"
#include "KMotionDLL.h"   // the spindle scenario watches channel 3 through a second connection
#undef check              // KMotionX's dbg.h has a check() macro; the harness has its own
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
#include <unistd.h>

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

static void scenario_home(KmBackend &km)
{
    if (!km.is_board()) { printf("  home: board mode only (KM_BOARD=1)\n"); return; }
    KmHomeRequest req;
    KmHomeJoint &hj = req.joint[0];
    hj.home = true;
    hj.search_vel = -0.5;         // toward negative, 0.5 in/s
    hj.latch_vel = -0.1;
    hj.offset = 0;
    hj.home_pos = 0.2;
    hj.max_travel = 0.3;          // give up after 0.3 in without a switch
    hj.sequence = 0;
    Follower f(km);
    f.km.state(f.st);
    printf("  homing X from X%.4f: search -0.5 in/s, travel limit 0.3 in\n", f.st.pos[0]);
    int serial0 = f.st.home_serial;
    km.home(req);
    bool aborted = false;
    for (;;) {
        f.run(f.t + 0.05, true);
        if (getenv("KM_HOME_ABORT") && !aborted && f.t > 1.0) { printf("  t=%.2f abort\n", f.t); km.abort(); aborted = true; }
        if (f.st.home_serial != serial0) break;
        if (f.t > 30) { printf("  no result within 30 s\n"); break; }
    }
    printf("  t=%.2f homing %d, ok mask 0x%x, fail mask 0x%x, X%.4f\n", f.t, f.st.homing, f.st.home_ok_mask, f.st.home_fail_mask, f.st.pos[0]);
    check(f.st.home_serial != serial0, "a homing result arrived");
    check(!f.st.homing, "homing flag cleared");
    if (aborted) check(f.st.home_fail_mask == 1 && f.t < 3.0, "aborted homing reports failure promptly");
    else check((f.st.home_ok_mask | f.st.home_fail_mask) == 1, "the result names joint 0");
    f.run(f.t + 1.0, true);
    check(f.messages.find("home program failed") == std::string::npos, "the program compiled and ran");
}

// configs/kmotion-kogna, found from where this program is (kmotion-motion/build/backend-test)
static std::string kogna_config_dir()
{
    char exe[4096];
    ssize_t n = readlink("/proc/self/exe", exe, sizeof exe - 1);
    if (n <= 0) return "../configs/kmotion-kogna";
    exe[n] = 0;
    std::string p = exe;
    for (int up = 0; up < 3; up++) p = p.substr(0, p.rfind('/'));     // the file, build, kmotion-motion
    return p + "/configs/kmotion-kogna";
}

static void scenario_spindle(KmBackend &km)
{
    if (!km.is_board()) {
        KmState st;
        km.state(st);
        const int errors = st.errors;
        KmSpindleCss css;
        css.surface_speed = 20;
        km.spindle(1, 191, &css);
        for (int i = 0; i < 200 && st.spindle_done == 0; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            km.state(st);
            if (st.message[0]) printf("  message: %s\n", st.message);
        }
        check(st.errors == errors + 1, "without SPINDLE_CSS, G96 is refused with an error");
        printf("  (the spindle on the board: KM_BOARD=1)\n");
        return;
    }
    CKMotionDLL board(0);
    // channel 3's speed from its Dest over half a second of the board's own time, and the
    // direction bits 1024 (CW) and 1025 (CCW)
    double pos_rate = NAN;                 // channel 3's Position speed, counts/s (encoder 4, the quadrature trick)
    auto measure = [&](double &counts_s, int &bits) {
        MAIN_STATUS a, b;
        memset(&a, 0, sizeof a);
        memset(&b, 0, sizeof b);
        if (board.GetStatus(a, true)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (board.GetStatus(b, true)) return false;
        double dt = b.TimeStamp - a.TimeStamp;
        counts_s = dt > 0 ? (b.Dest[3] - a.Dest[3]) / dt : NAN;
        pos_rate = dt > 0 ? (b.Position[3] - a.Position[3]) / dt : NAN;
        bits = b.VirtualBitsEx0 & 3;
        return true;
    };
    KmState st;
    km.state(st);
    const int done0 = st.spindle_done;
    int posted = 0;
    auto command = [&](int state, double rpm, const KmSpindleCss *css) {
        auto t0 = std::chrono::steady_clock::now();
        km.spindle(state, rpm, css);
        posted++;
        for (;;) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            km.state(st);
            if (st.message[0]) printf("  message: %s\n", st.message);
            double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (st.spindle_done - done0 >= posted) return t;
            if (t > 30) { printf("  no result within 30 s\n"); return t; }
        }
    };
    // until the board says the spindle is at speed (channel 3 done) in a read made after the
    // last spindle command was done: the seconds that took, -1 if not within the limit
    auto wait_at_speed = [&](double limit) {
        auto t0 = std::chrono::steady_clock::now();
        for (;;) {
            km.state(st);
            double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (st.spindle_at_speed && st.spindle_at_speed_done == st.spindle_done) return t;
            if (t > limit) return -1.0;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    };
    struct Step { int state; double rpm, counts_s; int bits; const char *what; };
    const Step steps[] = {
        {1, 600, 10000, 1, "M3 S600: CW at 10000 counts/s, bit 1024"},
        {1, 1200, 20000, 1, "S1200 while turning: 20000 counts/s"},
        {-1, 600, -10000, 2, "M4 S600: spins down, then CCW at -10000 counts/s, bit 1025"},
        {0, 600, 0, 0, "M5: stops, both bits clear"},
    };
    for (const Step &x : steps) {
        double took = command(x.state, x.rpm, nullptr);
        double at = wait_at_speed(5);
        printf("  at speed %.2f s after the command\n", at);
        check(x.state == 0 ? (at >= 0 && at < 0.3) : (at > 0.5 && at < 2.0),
              x.state == 0 ? "  at speed at once (OffJog.c waits for the stop)" : "  at speed once the ramp is over (~1 s at 10000 counts/s^2)");
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        double v = NAN;
        int bits = -1;
        measure(v, bits);
        km.state(st);
        double rpm = x.counts_s * 60 / 1000;           // MySpindleDefs.h: 1000 counts/rev
        printf("  the command took %.2f s; then Dest %.0f counts/s, Position %.0f counts/s, bits %d, measured %.1f RPM\n",
               took, v, pos_rate, bits, st.spindle_rpm_measured);
        check(fabs(v - x.counts_s) < 0.02 * fabs(x.counts_s) + 20, x.what);
        check(bits == x.bits, "  the direction bits");
        check(st.spindle_state == x.state, "  the backend's spindle state");
        check(fabs(st.spindle_rpm_measured - rpm) < 0.02 * fabs(rpm) + 5, "  the measured spindle speed");
    }
    // G96 through the board's ServiceCSS() loop (TestIncludingCSS.c, started in thread 4):
    // S100 ft/min = 20 in/s at the tool. X's offset puts the radius at 1 in (191.0 RPM); X then
    // moves out to 2 in (95.5 RPM); D 60 caps it; M4 reverses it in G96 (the spin-down used
    // to hang there: ServiceCSS() jogged it back up); G97 S600 goes back to RPM mode; M5
    auto persist110 = [&] {
        char reply[256] = "";
        if (board.WriteLineReadLine("GetPersistHex 110", reply)) return -1;
        return (int) strtol(reply, nullptr, 16);
    };
    auto wait_jog = [&] {
        for (int i = 0; i < 1000; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            km.state(st);
            if (!(st.jog_busy & 1)) return;
        }
        printf("  X did not finish its move within 10 s\n");
    };
    km.state(st);
    const double x0 = st.pos[0];
    KmSpindleCss css;
    css.surface_speed = 20.0;
    css.max_rpm = 1e9;
    css.x_offset = x0 - 1.0;
    struct CssStep { double x, max_rpm; bool send, css; int state; double rpm; int bits, mode; const char *what; };
    const double r1 = 20.0 * 60 / (2 * M_PI * 1.0), r2 = 20.0 * 60 / (2 * M_PI * 2.0);
    const CssStep css_steps[] = {
        {x0, 1e9, true, true, 1, r1, 1, 2, "G96 S100 M3 at radius 1 in: 191.0 RPM, persist 110 = 2"},
        {x0 + 1.0, 1e9, false, true, 1, r2, 1, 2, "X out to radius 2 in, nothing sent: the board halves it to 95.5 RPM"},
        {x0 + 1.0, 60, true, true, 1, 60, 1, 2, "G96 D60: capped at 60 RPM"},
        {x0 + 1.0, 60, true, true, -1, 60, 2, 2, "M4 in G96: spins down and back up CCW at 60 RPM"},
        {x0 + 1.0, 1e9, true, false, -1, 600, 2, 1, "G97 S600: 600 RPM CCW, persist 110 = 1"},
        {x0 + 1.0, 1e9, true, false, 0, 0, 0, 1, "M5: stops"},
    };
    for (const CssStep &x : css_steps) {
        km.state(st);
        if (fabs(st.pos[0] - x.x) > 1e-4) { km.jog_to(0, x.x, 2.0); wait_jog(); }
        css.max_rpm = x.max_rpm;
        double took = 0;
        if (x.send) took = command(x.state, x.rpm > 0 ? x.rpm : 600, x.css ? &css : nullptr);
        double at = wait_at_speed(5);
        check(at >= 0, "  at speed again");
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        double v = NAN;
        int bits = -1;
        measure(v, bits);
        km.state(st);
        double counts_s = x.state * x.rpm * 1000 / 60;
        printf("  the command took %.2f s; X%.4f; then Dest %.0f counts/s (expected %.0f), bits %d, persist 110 = %d, measured %.1f RPM\n",
               took, st.pos[0], v, counts_s, bits, persist110(), st.spindle_rpm_measured);
        check(fabs(v - counts_s) < 0.02 * fabs(counts_s) + 20, x.what);
        check(bits == x.bits, "  the direction bits");
        check(persist110() == x.mode && st.spindle_css == (x.mode == 2), "  the CSS mode on the board");
        check(fabs(st.spindle_rpm_measured - x.state * x.rpm) < 0.02 * x.rpm + 5, "  the measured spindle speed");
    }
    km.jog_to(0, x0, 2.0);
    wait_jog();
    km.state(st);
    check(fabs(st.pos[0] - x0) < 1e-3, "X back where it started");
}

static void scenario_jog(KmBackend &km)
{
    if (!km.is_board()) { printf("  jog: board mode only (KM_BOARD=1)\n"); return; }
    KmState st;
    auto secs = [](std::chrono::steady_clock::time_point a) {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - a).count();
    };
    // until the backend reports X's jog finished: the time it took (-1: not within the limit)
    auto wait_done = [&](double limit) {
        auto t0 = std::chrono::steady_clock::now();
        for (;;) {
            km.state(st);
            if (st.message[0]) printf("  message: %s\n", st.message);
            if (!(st.jog_busy & 1)) return secs(t0);
            if (secs(t0) > limit) return -1.0;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    };
    const double count = 1.0 / 2540;
    km.state(st);
    const double x0 = st.pos[0];
    printf("  X starts at %.5f\n", x0);
    check(!(st.jog_busy & 1), "no jog before the first one");

    // continuous at 1 in/s: busy while held, still busy right after the stop (the board slows
    // down), then done with X standing still
    km.jog(0, 1.0);
    km.state(st);
    check(st.jog_busy & 1, "a continuous jog is running as soon as it is issued");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    km.state(st);
    printf("  after 1 s: X%.5f\n", st.pos[0]);
    check((st.jog_busy & 1) && st.pos[0] > x0 + 0.8 && st.pos[0] < x0 + 1.1, "  still running a second later, X about 1 in further");
    km.jog_stop(0);
    km.state(st);
    check(st.jog_busy & 1, "  still running right after the stop");
    double t = wait_done(5);
    printf("  stopped at X%.5f, %.3f s after the stop\n", st.pos[0], t);
    check(t >= 0, "  finished after the stop");
    double xs = st.pos[0];
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    km.state(st);
    check(fabs(st.pos[0] - xs) < 1e-6, "  and X stands still");

    // an increment of -0.5 at 1 in/s: about half a second, finished at the target
    double target = st.pos[0] - 0.5;
    km.jog_to(0, target, 1.0);
    km.state(st);
    check(st.jog_busy & 1, "an increment is running as soon as it is issued");
    t = wait_done(5);
    printf("  increment of -0.5 at 1 in/s: finished after %.3f s at X%.5f\n", t, st.pos[0]);
    check(t > 0.45 && t < 1.0, "  it took about half a second (the jog speed, not the board's Vel)");
    check(fabs(st.pos[0] - target) < count, "  finished at the target");

    // an increment of +2 stopped after half a second
    target = st.pos[0] + 2;
    km.jog_to(0, target, 1.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    km.jog_stop(0);
    t = wait_done(5);
    printf("  increment of +2 stopped after 0.5 s: finished %.3f s later at X%.5f\n", t, st.pos[0]);
    check(t >= 0 && st.pos[0] < target - 1, "  the stop ends it short of the target");

    // an increment of 0: nothing to move, finished at once
    km.jog_to(0, st.pos[0], 1.0);
    t = wait_done(1);
    printf("  increment of 0: finished after %.3f s\n", t);
    check(t >= 0 && t < 0.2, "an increment of 0 finishes at once");

    // back to the start
    km.jog_to(0, x0, 2.0);
    t = wait_done(10);
    check(t >= 0 && fabs(st.pos[0] - x0) < count, "back at the start");
}

static void scenario_atbit(KmBackend &km)
{
    if (!km.is_board()) { printf("  atbit: board mode only (KM_BOARD=1)\n"); return; }
    CKMotionDLL board(0);
    KmState st;
    auto settle = [&](bool want) {          // until a read shows the bit as wanted (2 s at most)
        for (int i = 0; i < 200; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            km.state(st);
            if (st.spindle_at_speed == want && st.spindle_at_speed_done == st.spindle_done) return true;
        }
        printf("  still at_speed %d (read after %d of %d spindle commands), %u status reads\n",
               st.spindle_at_speed, st.spindle_at_speed_done, st.spindle_done, st.status_count);
        return false;
    };
    check(board.WriteLine("ClearBit1030") == 0 && settle(false), "bit 1030 clear: not at speed");
    check(board.WriteLine("SetBit1030") == 0 && settle(true), "bit 1030 set: at speed");
    check(board.WriteLine("ClearBit1030") == 0 && settle(false), "clear again: not at speed");
}

static void scenario_io(KmBackend &km)
{
    if (!km.is_board()) { printf("  io: board mode only (KM_BOARD=1)\n"); return; }
    KmState st;
    // until the published input bits (1026 = bit 0, 1027 = bit 1) match, within 2 s
    auto inputs = [&](unsigned want) {
        for (int i = 0; i < 200; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            km.state(st);
            if (st.message[0]) printf("  message: %s\n", st.message);
            if ((st.input_states & 3) == want) return true;
        }
        printf("  inputs read 0x%llx\n", st.input_states & 3);
        return false;
    };
    km.set_bit(1026, false);
    km.set_bit(1027, false);
    check(inputs(0), "bits 1026 and 1027 clear");
    km.set_bit(1026, true);
    check(inputs(1), "set_bit(1026): read back set");
    km.set_bit(1026, false);
    check(inputs(0), "and clear again");
    // M100 P1.5 Q2.5: the program sets bit 1027; M100 P0: clears it
    auto mcode = [&](double p, double q) {
        auto t0 = std::chrono::steady_clock::now();
        int ticket = km.mcode(100, p, q), result = -1;
        while (!km.mcode_finished(ticket, result)) {
            if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(30)) { printf("  M100 not finished in 30 s\n"); return -1; }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        printf("  M100 P%g Q%g: result %d after %.2f s\n", p, q, result,
               std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
        return result;
    };
    check(mcode(1.5, 2.5) == 0 && inputs(2), "M100 P1.5: done, bit 1027 set");
    check(mcode(0, -1) == 0 && inputs(0), "M100 P0: done, bit 1027 clear");
}

static void scenario_probe(KmBackend &km)
{
    if (!km.is_board()) { printf("  probe: board mode only (KM_BOARD=1)\n"); return; }
    CKMotionDLL board(0);
    KmState st;
    auto contact = [&](bool on) { board.WriteLine(on ? "SetBit1031" : "ClearBit1031"); };   // not 1032: Tom's simulator
    // a probe move along X to x at vel in/s; the contact changes after `after` seconds (if >= 0);
    // the outcome, -1 if none within 15 s
    auto probe = [&](double x, bool away, double after, bool set_to, double vel) {
        km.state(st);
        int s0 = st.probe_serial;
        double end[9] = {x, st.pos[1], st.pos[2], 0, 0, 0, 0, 0, 0};
        auto t0 = std::chrono::steady_clock::now();
        km.probe(end, vel, 10.0, 1, away);
        bool changed = false;
        for (;;) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (after >= 0 && !changed && t >= after) { contact(set_to); changed = true; }
            km.state(st);
            if (st.message[0]) printf("  message: %s\n", st.message);
            if (st.probe_serial != s0) {
                printf("  outcome %d after %.2f s: probed X%.5f, now X%.5f%s\n", st.probe_outcome, t, st.probe_pos[0], st.pos[0],
                       st.running ? " (still running)" : "");
                return st.probe_outcome;
            }
            if (t > 15) { printf("  no result within 15 s\n"); return -1; }
        }
    };
    auto settle = [&] {                     // the board at rest after the result
        for (int i = 0; i < 300; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            km.state(st);
            if (!st.running) return;
        }
    };
    contact(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    km.state(st);
    const double x0 = st.pos[0];
    printf("  X starts at %.5f\n", x0);

    // G38.2 toward X+1, contact after 0.6 s: tripped part way, stopped short of the end
    int o = probe(x0 + 1, false, 0.6, true, 0.5);
    settle();
    double trip = st.probe_pos[0];
    check(o == KM_PROBE_TRIPPED, "G38.2: tripped when the probe made contact");
    check(trip > x0 + 0.05 && trip < x0 + 0.6, "  probed position part way along (0.5 in/s)");
    check(st.pos[0] >= trip - 1e-4 && st.pos[0] < trip + 0.05 && !st.running, "  stopped just past the trip and at rest");

    // G38.4 back toward X0 while in contact, contact lost after 0.4 s: tripped
    o = probe(x0, true, 0.4, false, 0.5);
    settle();
    check(o == KM_PROBE_TRIPPED && st.probe_pos[0] < trip && st.probe_pos[0] > x0, "G38.4: tripped when the contact was lost");

    // G38.2 while already in contact: nothing moves
    contact(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    km.state(st);
    double before = st.pos[0];
    o = probe(before + 0.5, false, -1, false, 0.5);
    km.state(st);
    check(o == KM_PROBE_ALREADY && fabs(st.pos[0] - before) < 1e-4, "G38.2 with the probe already tripped: refused, no move");
    contact(false);

    // G38.2 without contact: runs to its end
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    km.state(st);
    double target = st.pos[0] + 0.2;
    o = probe(target, false, -1, false, 0.5);
    settle();
    check(o == KM_PROBE_NO_CONTACT && fabs(st.pos[0] - target) < 1e-3, "G38.2 without contact: ran to its end");

    // G38.2 longer than the planner's lookahead (1 in at 10 in/min, 6 s), contact after 0.8 s:
    // the trip comes while the move is still being downloaded, the planner pacing its download
    // on the held board (LinuxCNC hung there until the poller learned to see the trip)
    km.state(st);
    double start = st.pos[0];
    o = probe(start + 1, false, 0.8, true, 1.0 / 6);
    settle();
    check(o == KM_PROBE_TRIPPED && st.probe_pos[0] > start + 0.05 && st.probe_pos[0] < start + 0.3 && !st.running,
          "G38.2 longer than the lookahead: tripped part way, at rest");
    contact(false);

    // back to the start
    double back[9] = {x0, st.pos[1], st.pos[2], 0, 0, 0, 0, 0, 0};
    km.line(back, 2.0, 50.0, false, 2);
    Follower f(km);
    f.run(20);
    check(fabs(f.st.pos[0] - x0) < 1e-3, "X back at the start");
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
        cfg.init_programs.push_back("/home/tk/KMotionXCNC/settings/c-programs/MinirouterInit.c");
        cfg.init_thread = 1;
        const std::string kogna = kogna_config_dir();
        snprintf(cfg.home_program, sizeof cfg.home_program, "%s", (kogna + "/Home.c").c_str());
        if (strcmp(scenario, "spindle") == 0) {           // as kmotion-kogna.ini sets it up
            cfg.init_programs.push_back(kogna + "/spindle/SpindleAxis.c");
            cfg.start_programs.emplace_back(1, kogna + "/BenchLoop.c");   // ServiceCSS() in thread 1
            cfg.spindle_css = true;
            cfg.spindle_at_speed = KM_AT_SPEED_AXIS;
            cfg.spindle_at_speed_axis = 3;
            cfg.spindle_speed_axis = 3;
            cfg.spindle_counts_per_rev = 1000;
            cfg.spindle_speed_from_dest = !(getenv("KM_SPINDLE_FROM") && !strcasecmp(getenv("KM_SPINDLE_FROM"), "POSITION"));
            cfg.spindle_speed_tau = 0.1;
            printf("spindle speed measured from channel 3's %s\n", cfg.spindle_speed_from_dest ? "Dest" : "Position");
            const char *file[KM_SPINDLE_ACTIONS] = {"OnCWJog.c", "OnCCWJog.c", "OffJog.c", "SpindleJog.c"};
            for (int i = 0; i < KM_SPINDLE_ACTIONS; i++) {
                cfg.spindle[i].type = KM_ACTION_PROGRAM_WAIT;
                cfg.spindle[i].p[0] = 3;
                cfg.spindle[i].p[1] = i == KM_SPINDLE_S ? 113 : 1;
                snprintf(cfg.spindle[i].file, sizeof cfg.spindle[i].file, "%s/spindle/%s", kogna.c_str(), file[i]);
            }
        }
        if (strcmp(scenario, "probe") == 0) {         // as kmotion-kogna.ini sets probing up
            cfg.start_programs.emplace_back(1, kogna + "/BenchLoop.c");   // ServiceProbe() in thread 1
            cfg.probe_bit = 1031;                     // not the ini's 1032, which Tom's probe simulator may drive
            cfg.probe_level = 1;
        }
        if (strcmp(scenario, "io") == 0) {            // as kmotion-kogna.ini sets M100 up
            cfg.input_bits = {1026, 1027};
            KmAction &a = cfg.mcode[0];
            a.type = KM_ACTION_PROGRAM_WAIT;
            a.p[0] = 5;
            a.p[1] = 10;
            snprintf(a.file, sizeof a.file, "%s/mcodes/M100.c", kogna.c_str());
        }
        if (strcmp(scenario, "atbit") == 0) {
            cfg.spindle_at_speed = KM_AT_SPEED_BIT;
            cfg.spindle_at_speed_bit = 1030;
            cfg.spindle_at_speed_level = 1;
        }
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
    else if (strcmp(scenario, "home") == 0) scenario_home(*km);
    else if (strcmp(scenario, "gated") == 0) scenario_gated(*km);
    else if (strcmp(scenario, "spindle") == 0) scenario_spindle(*km);
    else if (strcmp(scenario, "jog") == 0) scenario_jog(*km);
    else if (strcmp(scenario, "atbit") == 0) scenario_atbit(*km);
    else if (strcmp(scenario, "io") == 0) scenario_io(*km);
    else if (strcmp(scenario, "probe") == 0) scenario_probe(*km);
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
