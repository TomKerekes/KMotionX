// backend-test: exercises KmBackend alone (no LinuxCNC, no board), followed in real time
// at 5 ms as the protocol loop does. "make test" runs the first four scenarios:
//   backend-test          resume-test.ngc's moves: the planner's timing and blending
//   backend-test arcs     arc directions, planes and multiple turns as LinuxCNC encodes them
//   backend-test abort    an abort in mid-program, then a move after it
//   backend-test joglimit a world jog stops where a joint reaches its soft limit: the cut on
//                         the jog's path that kmotion-motion makes at the jog's start
//                         (kmotion-joglimit.h), with identity kinematics and with the 3Link
//                         kinematics at the robot's poses, then a planner move to the cut end
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
//   backend-test limits   (KM_BOARD=1 only) limit switches read with the status: C Programs/
//                         SimulateHomes.c (started on thread 7) makes channel 0's positive
//                         switch, bit 52, above +2000 counts and its negative one, bit 49,
//                         below -1500; jogs to X1, X0, X-0.7 and back show them in the state
//   backend-test tap      rigid tapping: simulate - replayed as a move down and back; board
//                         (KM_BOARD=1) - Tap.c on thread 4 runs the bench's spindle (axis 3,
//                         its looped encoder) while the move runs in spindle-sync mode
//   backend-test measure  KMotionCNC's Measure: points appended to a geo measurement file
//   backend-test holdjog  (KM_BOARD=1 only) KMotionCNC's forward/reverse in feed hold: a move
//                         paused half way runs backwards and forwards while "held"
//   backend-test offon    (KM_BOARD=1 only) a move, three aborts, the axes disabled and
//                         enabled again (a machine off and on), a jog, an abort, then the
//                         first program move: LinuxCNC's sequence around a limit switch trip
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
//   backend-test curve    a curve of short moves (a 6.3 mm radius arc's chords, then the same arc
//                         as 1 degree G1 facets) runs at the speed the filter window and the
//                         tolerance allow, within the tolerance and the joints' limits, and a
//                         90 degree corner is still blended; KM_3LINK=1 through the 3 Link
//                         kinematics at the robot's joint limits (KM_ACT_VEL, KM_ACT_JERK vary them;
//                         KM_CURVE_FACETS=0 the old planner, every junction a corner)
//   backend-test zigzag   a zigzag of tiny moves at a sharp angle (60 degrees; KM_ZIG_DEG, KM_ZIG_LEN):
//                         the planner's own caps carry it through where the corner policy stopped at
//                         every vertex; within the tolerance and the joints' limits
//   backend-test canon    replays a program from LinuxCNC's canon dump (rs274 -g file.ngc) given
//                         as KM_CANON=<file>, at the work offset KM_OFFSET="x y z" (inches), and
//                         prints the program's time and the arcs' speeds
// Each check prints ok/FAIL; the exit status is the number of failures. The planner's logs
// (TPSegLog.csv, TP3_Timeline.log) go to /tmp/backend-test, or to KMX_LOG_DIR when set.
// KM_BOARD=1 in the environment runs the backend in board mode with the kmotion-kogna
// settings (2540 counts/in, 9.84 in/s, the config's MinirouterInit.c): "abort" then measures the real
// stop on the Kogna, with KM_JOG0=1 adding the Jog<ch>=0 the protocol side used to send.
#include "kmotion-backend.h"
#include "kmotion-joglimit.h"
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
#include <sys/stat.h>

// the least mean speed on the curve the curve scenario accepts, in/s (one chord per window ran it at 0.19)
#define KM_CURVE_MIN_SPEED 0.5

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
    std::map<int, double> t_start, t_end;   // when each move became active, and stopped being
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
                if (last_id > 0) t_end[last_id] = t;
                if (st.active_id > 0 && !t_start.count(st.active_id)) t_start[st.active_id] = t;
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

static void scenario_limits(KmBackend &km)
{
    if (!km.is_board()) { printf("  limits: board mode only (KM_BOARD=1)\n"); return; }
    // SimulateHomes.c (main started it on thread 7) makes the limit bits from the channels'
    // destinations: channel 0's positive switch (bit 52) above +2000 counts, 0.787 in at 2540
    // counts/in, its negative one (bit 49) below -1500 counts, -0.591 in
    KmState st;
    auto limits = [&](unsigned neg, unsigned pos, const char *what) {
        for (int i = 0; i < 400; i++) {            // a jog of 1 in at 2 in/s, then the status
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            km.state(st);
            if (st.message[0]) printf("  message: %s\n", st.message);
            if ((st.neg_limit & 1) == neg && (st.pos_limit & 1) == pos && !(st.jog_busy & 1)) return true;
        }
        printf("  %s: neg 0x%x pos 0x%x jog_busy 0x%x at X%.4f\n", what, st.neg_limit, st.pos_limit, st.jog_busy, st.pos[0]);
        return false;
    };
    km.state(st);
    double x0 = st.pos[0];
    printf("  X at %.4f; the positive switch above X%.4f, the negative below X%.4f (counts 0 = X0)\n", x0, 2000 / 2540.0, -1500 / 2540.0);
    check(limits(0, 0, "at the start"), "no limit switch made at the start");
    km.jog_to(0, 1.0, 2.0);
    check(limits(0, 1, "after a jog to X1"), "X1: the positive switch (bit 52) is made, in the state");
    km.jog_to(0, 0.0, 2.0);
    check(limits(0, 0, "back at X0"), "X0: made no more");
    km.jog_to(0, -0.7, 2.0);
    check(limits(1, 0, "after a jog to X-0.7"), "X-0.7: the negative switch (bit 49) is made");
    km.jog_to(0, x0, 2.0);
    check(limits(0, 0, "back at the start"), "back at the start: none made");
}

// the sequence LinuxCNC runs around a limit switch trip (or any machine off), on the board:
// a move, the aborts task sends in a row, the axes disabled and enabled again (machine off
// and on), a jog, task's abort at the mode change, then the first program move - which
// twice left kmotion-motion dead on the bench right after the planner took it up
static void scenario_offon(KmBackend &km)
{
    if (!km.is_board()) { printf("  offon: board mode only (KM_BOARD=1)\n"); return; }
    Follower f(km);
    f.km.state(f.st);
    double x0 = f.st.pos[0];
    double e[9]; memcpy(e, f.st.pos, sizeof e);
    e[0] = x0 + 0.3;
    km.line(e, 2.0, 50, true, 1);
    f.run(10);
    check(f.at(e[0], e[1], e[2], 1e-3), "a move to X+0.3 runs");
    for (int i = 0; i < 3; i++) km.abort();                   // task's aborts in a row
    km.machine_on(false);                                     // machine off (a limit trip)
    km.machine_on(true);                                      // and on again
    for (int i = 0; i < 300; i++) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); km.state(f.st); if (!f.st.running) break; }
    check(!f.st.running, "idle after the aborts, off and on");
    km.jog_to(0, x0 + 0.1, 1.0);                              // the jog off the switch
    for (int i = 0; i < 500; i++) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); km.state(f.st); if (!(f.st.jog_busy & 1) && i > 5) break; }
    km.jog_stop(0);
    check(fabs(f.st.pos[0] - (x0 + 0.1)) < 1e-3, "the jog ran to X+0.1");
    km.abort();                                               // task's abort at the mode change
    for (int i = 0; i < 300; i++) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); km.state(f.st); if (!f.st.running) break; }
    e[0] = x0;
    printf("  the first program move after the off/on: X%.4f -> X%.4f\n", f.st.pos[0], x0);
    f.t0 = std::chrono::steady_clock::now();
    f.t = 0;
    km.line(e, 2.0, 50, true, 2);
    f.run(10);
    check(f.at(x0, e[1], e[2], 1e-3), "it runs to the start");
    // the bench's axes (MinirouterInit.c: Accel 4000 at Vel 40000 counts/s) take about 20 s to
    // stop, far over 75% of the lookahead: the planner says so once, through the interpreter's
    // Translate() - which had no interpreter here and crashed the process until it was made
    // safe (GCodeInterpreter.cpp). That warning is the one message allowed
    std::string other = f.messages;
    size_t w = other.find("Warning Controller configuration takes longer to stop");
    if (w != std::string::npos) other.erase(w, other.find('\n', w) == std::string::npos ? std::string::npos : other.find('\n', w) - w + 1);
    check(other.empty(), "no messages but the bench's stop-time warning");
}

// KMotionCNC's Measure: a geo correction measurement file with its header, three points
// appended from the planner's position, the file as the kinematics read it
static void scenario_measure(KmBackend &km)
{
    const char *file = "/tmp/claude-backend-test-measure.txt";
    FILE *f = fopen(file, "w");
    check(f != NULL, "a measurement file with the header: 2 rows, 3 columns, spacing 1, offset 0");
    if (!f) return;
    fprintf(f, "2,3\n1.0,1.0\n0.0,0.0\n");
    fclose(f);
    int row, col, index, total;
    double pos[9];
    std::string err;
    KmState st;
    double p[9] = {0.5, 0.25, 0.125, 0, 0, 0, 0, 0, 0};
    km.set_position(p);
    for (int i = 0; i < 200; i++) { km.state(st); if (fabs(st.pos[0] - 0.5) < 1e-9) break; std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
    check(km.measure_point(file, row, col, index, total, pos, err), "the first point is recorded");
    check(row == 0 && col == 0 && index == 1 && total == 6 && fabs(pos[0] - 0.5) < 1e-9, "as row 0, col 0, point 1 of 6, at the planner's position");
    p[0] = 1.5;
    km.set_position(p);
    for (int i = 0; i < 200; i++) { km.state(st); if (fabs(st.pos[0] - 1.5) < 1e-9) break; std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
    check(km.measure_point(file, row, col, index, total, pos, err) && row == 0 && col == 1 && index == 2, "the second: row 0, col 1");
    check(km.measure_point(file, row, col, index, total, pos, err) && row == 0 && col == 2 && index == 3, "the third: row 0, col 2");
    check(km.measure_point(file, row, col, index, total, pos, err) && row == 1 && col == 0 && index == 4, "the fourth starts row 1 (columns first)");
    f = fopen(file, "r");
    int nrows, ncols, n = 0, r, c;
    double sx, sy, ox, oy, x, y, z, lastx = -1;
    bool hdr = f && fscanf(f, "%d,%d", &nrows, &ncols) == 2 && fscanf(f, "%lf,%lf", &sx, &sy) == 2 && fscanf(f, "%lf,%lf", &ox, &oy) == 2;
    while (hdr && fscanf(f, "%d,%d,%lf,%lf,%lf", &r, &c, &x, &y, &z) == 5) { n++; lastx = x; }
    if (f) fclose(f);
    check(hdr && nrows == 2 && ncols == 3 && n == 4 && fabs(lastx - 1.5) < 1e-6, "the file reads back: header intact, 4 points, the last at X1.5");
    check(!km.measure_point("/tmp/claude-backend-test-no-such-dir/x.txt", row, col, index, total, pos, err) && !err.empty(), "a missing file is refused with a reason");
    remove(file);
}

// KMotionCNC's forward/reverse in feed hold, on the board: a slow move, a hold half way,
// reverse while "held" for a second (the position goes back), stop, forward (it advances),
// release, resume (the move completes)
static void scenario_holdjog(KmBackend &km)
{
    if (!km.is_board()) { printf("  holdjog: board mode only (KM_BOARD=1)\n"); return; }
    Follower f(km);
    f.km.state(f.st);
    double x0 = f.st.pos[0];
    double e[9]; memcpy(e, f.st.pos, sizeof e);
    e[0] = x0 + 1.0;
    km.line(e, 0.5, 50, false, 1);                        // 2 s at 0.5 in/s
    f.run(1.0, true);
    km.pause(true);
    f.run(f.t + 1.0, true);
    double x_hold = f.st.pos[0];
    printf("  paused at X%.4f (started at X%.4f)\n", x_hold, x0);
    check(x_hold > x0 + 0.2 && x_hold < x0 + 0.9 && f.st.paused, "paused in the middle of the move");
    km.hold_jog(-0.5);
    f.run(f.t + 1.0, true);
    km.hold_jog(0);
    f.run(f.t + 0.5, true);
    double x_back = f.st.pos[0];
    printf("  after a second of reverse at 0.5: X%.4f\n", x_back);
    check(x_back < x_hold - 0.1, "reverse while held: the position went back");
    km.hold_jog(0.5);
    f.run(f.t + 0.5, true);
    km.hold_jog(0);
    f.run(f.t + 0.5, true);
    double x_fwd = f.st.pos[0];
    printf("  after half a second forward: X%.4f\n", x_fwd);
    check(x_fwd > x_back + 0.05, "forward while held: it advanced");
    km.pause(false);
    f.run(10);
    check(f.at(e[0], e[1], e[2], 1e-3), "resumed: the move completes at X+1");
    check(f.messages.empty(), "no messages");
}

// rigid tapping: simulate - the tap replayed as a move down and back, the result reported;
// board (KM_BOARD=1, the bench's spindle on axis 3 with its looped encoder, Tap.c on thread
// 4) - M3 S600, a tap of 0.5 in at 0.05 in/rev (10 revolutions), the position back at the
// start, the spindle running again
static void scenario_tap(KmBackend &km)
{
    Follower f(km);
    f.km.state(f.st);
    double start[9]; memcpy(start, f.st.pos, sizeof start);
    double bottom[9]; memcpy(bottom, start, sizeof bottom);
    bottom[2] = start[2] - 0.5;
    int serial0 = f.st.tap_serial;
    if (km.is_board()) {
        km.spindle(1, 600);
        for (int i = 0; i < 600; i++) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); km.state(f.st); if (f.st.spindle_done >= 1 && f.st.spindle_at_speed) break; }
        check(f.st.spindle_state == 1, "M3 S600: the spindle turns");
        std::this_thread::sleep_for(std::chrono::seconds(2));    // up to speed (OnCWJog ramps)
    }
    printf("  tap from Z%.4f to Z%.4f at 0.05 in/rev, 10 rev/s\n", start[2], bottom[2]);
    km.tap(bottom, 0.05, 10.0, 1.0, 7);
    double zmin = start[2];
    for (;;) {
        f.run(f.t + 0.05, true);
        if (f.st.pos[2] < zmin) zmin = f.st.pos[2];
        if (f.st.tap_serial != serial0 && !f.st.running && !f.st.tapping) break;
        if (f.t > 60) { printf("  no end within 60 s\n"); break; }
    }
    printf("  t=%.2f: result %d, deepest Z%.4f, at rest at Z%.4f, spindle state %d\n", f.t, f.st.tap_result, zmin, f.st.pos[2], f.st.spindle_state);
    check(f.st.tap_serial != serial0 && f.st.tap_result == 0, "the tap reports done");
    check(zmin <= bottom[2] + 1e-3, "it reached the bottom");
    check(f.at(start[0], start[1], start[2], km.is_board() ? 0.02 : 1e-3), "and came back to the start");
    // a G84: the rapid to the retract plane and the tap come in one block. The tap must wait
    // for the rapid (the worker holds it until the planner is idle) and start from there
    {
        double plane[9]; memcpy(plane, start, sizeof plane); plane[2] = start[2] + 0.3;
        double bottom2[9]; memcpy(bottom2, plane, sizeof bottom2); bottom2[2] = plane[2] - 0.5;
        int serial1 = f.st.tap_serial;
        printf("  a rapid to Z%.4f and, in the same breath, a tap to Z%.4f\n", plane[2], bottom2[2]);
        km.line(plane, 50.0, 1e9, true, 8);
        km.tap(bottom2, 0.05, 10.0, 1.0, 9);
        bool waited = false, early = false;
        zmin = plane[2];
        for (;;) {
            f.run(f.t + 0.05, true);
            if (f.st.running && !f.st.tapping && f.st.tap_serial == serial1) waited = true;
            if (f.st.tapping && f.st.pos[2] < plane[2] - 1e-3 && zmin == plane[2] && f.st.pos[2] < start[2] + 0.3 - 0.01 && !waited) early = true;
            if (f.st.tapping || f.st.tap_serial != serial1) { if (f.st.pos[2] < zmin) zmin = f.st.pos[2]; }
            if (f.st.tap_serial != serial1 && !f.st.running && !f.st.tapping) break;
            if (f.t > 120) { printf("  no end within 120 s\n"); break; }
        }
        printf("  t=%.2f: result %d, deepest Z%.4f, at rest at Z%.4f\n", f.t, f.st.tap_result, zmin, f.st.pos[2]);
        check(waited && !early, "the tap waited for the rapid before it");
        check(f.st.tap_serial != serial1 && f.st.tap_result == 0, "then reported done");
        check(zmin <= bottom2[2] + 1e-3, "it reached its bottom, measured from the retract plane");
        check(f.at(plane[0], plane[1], plane[2], km.is_board() ? 0.02 : 1e-3), "and came back to the retract plane");
        // an abort while the tap still waits drops it: the rapid stops, no tap runs
        if (!km.is_board()) {
            double far[9]; memcpy(far, plane, sizeof far); far[2] = plane[2] + 2.0;
            int serial2 = f.st.tap_serial;
            km.line(far, 2.0, 1e9, true, 10);
            km.tap(bottom2, 0.05, 10.0, 1.0, 11);
            f.run(f.t + 0.3, true);
            km.abort();
            for (int i = 0; i < 200 && f.st.running; i++) f.run(f.t + 0.05, true);
            f.run(f.t + 0.5, true);
            check(f.st.tap_serial == serial2 && !f.st.tapping && !f.st.running, "an abort while the tap waits drops it: no tap ran");
            km.line(plane, 50.0, 1e9, true, 12);           // back to the retract plane
            for (int i = 0; i < 200; i++) { f.run(f.t + 0.05, true); if (!f.st.running && f.t > 1) break; }
        }
    }
    if (km.is_board()) {
        check(f.st.spindle_state == 1, "the spindle runs again as before (M3)");
        km.spindle(0, 0);
        for (int i = 0; i < 300; i++) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); km.state(f.st); if (f.st.spindle_state == 0) break; }
    }
    std::string other = f.messages;
    size_t w = other.find("Warning Controller configuration takes longer to stop");
    if (w != std::string::npos) other.erase(w, other.find('\n', w) == std::string::npos ? std::string::npos : other.find('\n', w) - w + 1);
    check(other.empty(), "no messages but the bench's stop-time warning");
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

// the 3 Link robot's kinematics (simulate mode): the joints are the servo angles, a square
// runs through the planner, the state reports the joints
static void scenario_kins3link(KmBackend &km)
{
    KmState st;
    km.state(st);
    check(!km.identity_kinematics(), "KINEMATICS = 3Link is not identity");
    double j0[8], back[9];
    check(km.cad_to_joints(st.pos, j0), "the start pose (the center of travel) is in reach");
    printf("  joints at X0 Y0 Z0: Right %.2f Left %.2f Z %.2f deg\n", j0[0], j0[1], j0[2]);
    check(km.joints_to_cad(j0, back) && fabs(back[0]) < 1e-4 && fabs(back[1]) < 1e-4 && fabs(back[2]) < 1e-4,
          "CAD -> joints -> CAD comes back to the origin");
    check(fabs(st.act_pos[0] - j0[0]) < 1e-6 && fabs(st.act_pos[1] - j0[1]) < 1e-6 && fabs(st.act_pos[2] - j0[2]) < 1e-6,
          "the state reports the joints");
    // a square 1 in wide at Z0.1 (the pen up a little), then back to the center and up
    double p[5][9] = {{0.5, 0, 0.1, 0, 0, 0, 0, 0, 0}, {0.5, 0.5, 0.1, 0, 0, 0, 0, 0, 0}, {-0.5, 0.5, 0.1, 0, 0, 0, 0, 0, 0},
                      {-0.5, -0.5, 0.1, 0, 0, 0, 0, 0, 0}, {0, 0, 0.5, 0, 0, 0, 0, 0, 0}};
    Follower f(km);
    for (int i = 0; i < 5; i++) km.line(p[i], 1.0, 30.0, i == 4, i + 1);
    f.run(60);
    check(f.extent.count(5) && f.at(0, 0, 0.5), "the square and the lift ran to X0 Y0 Z0.5");
    km.state(st);
    double j[8];
    check(km.cad_to_joints(st.pos, j) && fabs(st.act_pos[0] - j[0]) < 1e-3 && fabs(st.act_pos[2] - j[2]) < 1e-3,
          "the joints follow the position");
    printf("  joints at X0 Y0 Z0.5: Right %.2f Left %.2f Z %.2f deg\n", st.act_pos[0], st.act_pos[1], st.act_pos[2]);
    check(f.messages.empty(), "no messages");
}

// a world jog stops where a joint reaches its soft limit: the cut kmotion-motion.cc makes
// at the jog's start (kmotion-joglimit.h), on the jog's path, so the board ends the move
// there by itself. Identity kinematics first (joint 0's limit 5 tighter than the axis's
// 10), then the 3Link kinematics at the poses met on the robot with test limits Right min
// -15 and Z max 84, where a joint's travel along an axis is not monotonic. Last, the board
// way: the planner move to the cut end comes to rest there, nothing aborts it
static void scenario_joglimit(KmBackend &km)
{
    check(km.identity_kinematics(), "no KINEMATICS: joint j is axis j");
    double lo[8], hi[8];
    for (int j = 0; j < 8; j++) { lo[j] = -10; hi[j] = 10; }
    hi[0] = 5;
    const unsigned active = 7;                  // joints 0-2
    auto ident = [](const double p[9], double j[8]) { for (int i = 0; i < 8; i++) j[i] = p[i]; return true; };
    auto line = [](double x0, double y0, double z0, double x1, double y1, double z1, double *f, double *t) {
        for (int i = 0; i < 9; i++) f[i] = t[i] = 0;
        f[0] = x0; f[1] = y0; f[2] = z0; t[0] = x1; t[1] = y1; t[2] = z1;
    };
    double f[9], t[9];
    line(4.9, 0, 0, 10, 0, 0, f, t);
    JogLimit c = jog_limit(f, t, ident, lo, hi, active);
    double x = f[0] + c.s * (t[0] - f[0]);
    printf("  +X from X4.9 toward X10: cut by joint %d (%s) at X%.9f\n", c.joint, c.dir > 0 ? "max" : "min", x);
    check(c.joint == 0 && c.dir == 1 && fabs(x - 5) < 1e-9, "+X stops exactly at joint 0's max limit 5");
    line(5, 0, 0, 10, 0, 0, f, t);
    c = jog_limit(f, t, ident, lo, hi, active);
    check(c.joint == 0 && c.s < 1e-9, "+X from the limit itself: refused");
    line(5.5, 0, 0, 10, 0, 0, f, t);
    c = jog_limit(f, t, ident, lo, hi, active);
    check(c.joint == 0 && c.s == 0, "+X with joint 0 already beyond: refused");
    line(5.5, 0, 0, -10, 0, 0, f, t);
    c = jog_limit(f, t, ident, lo, hi, active);
    check(c.joint < 0 && c.s == 1, "-X from beyond the limit runs all the way to the axis limit (the joint comes back)");
    line(5.5, 0, 0, 5.5, 10, 0, f, t);
    c = jog_limit(f, t, ident, lo, hi, active);
    check(c.joint < 0 && c.s == 1, "Y with joint 0 parked beyond its limit: not cut");
    line(5.5, 0, 0, 10, 0, 0, f, t);
    c = jog_limit(f, t, ident, lo, hi, active & ~1u);
    check(c.joint < 0 && c.s == 1, "an inactive joint is not looked at");
    line(0, -9.9, 0, 0, -20, 0, f, t);
    c = jog_limit(f, t, ident, lo, hi, active);
    double y = f[1] + c.s * (t[1] - f[1]);
    check(c.joint == 1 && c.dir == -1 && fabs(y + 10) < 1e-9, "-Y stops exactly at joint 1's min limit -10");

    // the 3Link kinematics (the kmotion-3link settings), test limits Right min -15, Z max 84
    KmConfig k3;
    snprintf(k3.kinematics, sizeof k3.kinematics, "3Link");
    for (int i = 0; i < 3; i++) {
        KmAxisParams &a = k3.axis[i];
        a.counts_per_unit = 11.37778; a.max_vel = 3; a.max_accel = 30; a.max_jerk = 300;
        a.act_max_vel = i == 0 ? 35 : 17.6; a.act_max_accel = 700; a.act_max_jerk = 7000; a.act_degrees = true;
    }
    double origin[9] = {0};
    KmBackend kb;
    check(kb.init(k3, origin), "a 3Link backend (simulate)");
    auto kins = [&](const double p[9], double j[8]) { return kb.cad_to_joints(p, j); };
    double lo3[8] = {-15, 90, -45, -1e9, -1e9, -1e9, -1e9, -1e9}, hi3[8] = {90, 270, 84, 1e9, 1e9, 1e9, 1e9, 1e9};
    double jp[8], p[9];
    // +Z from the center of travel: the Z servo (80.02 at Z0, 87.14 at Z1) reaches 84
    line(0, 0, 0, 0, 0, 1, f, t);
    c = jog_limit(f, t, kins, lo3, hi3, active);
    for (int i = 0; i < 9; i++) p[i] = f[i] + c.s * (t[i] - f[i]);
    kb.cad_to_joints(p, jp);
    printf("  +Z from X0 Y0 Z0: cut by joint %d at Z%.5f, Z servo %.6f\n", c.joint, p[2], jp[2]);
    check(c.joint == 2 && c.dir == 1 && fabs(jp[2] - 84) < 1e-6, "+Z stops with the Z servo exactly at 84");
    // the pose the robot was left at: Right -15.26, past its min. Along every axis one way
    // brings it in: that way runs, and the other is refused at once
    const double P[3] = {3.4185, 0.6397, 0.9998};
    const double ax_lo[3] = {-4, -2, -0.1}, ax_hi[3] = {4, 2, 1.0};
    int refused = 0, ran = 0;
    for (int a = 0; a < 3; a++)
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            for (int i = 0; i < 9; i++) f[i] = t[i] = 0;
            for (int i = 0; i < 3; i++) f[i] = t[i] = P[i];
            t[a] = sgn > 0 ? ax_hi[a] : ax_lo[a];
            c = jog_limit(f, t, kins, lo3, hi3, active);
            double q[9], j0[8], j1[8];
            for (int i = 0; i < 9; i++) q[i] = f[i] + 1e-4 * (t[i] - f[i]) / fabs(t[a] - f[a]);
            kb.cad_to_joints(f, j0);
            kb.cad_to_joints(q, j1);
            bool inward = j1[0] > j0[0];
            printf("  %c%c from X%.4f Y%.4f Z%.4f: Right %s, cut s=%.4f by joint %d\n", sgn > 0 ? '+' : '-', "XYZ"[a],
                   P[0], P[1], P[2], inward ? "comes in" : "goes out", c.s, c.joint);
            if (fabs(t[a] - f[a]) < 1e-9) continue;   // at the axis limit already (Z1 is 0.0002 away: tiny but real)
            if (inward) { if (c.s > 0) ran++; }
            else if (c.s == 0 && c.joint == 0) refused++;
        }
    printf("  of the six directions: %d ran with Right coming in, %d refused with Right going out\n", ran, refused);
    check(ran + refused == 6, "every direction that brings Right in runs, every one that takes it out is refused");
    // X4 Y-2 Z1: Right -69.8 and Z 101.4, both past; -X first takes Right further out (-69.96
    // at X3.5) before it comes in: refused, +Y brings both in and runs
    line(4, -2, 1, -4, -2, 1, f, t);
    c = jog_limit(f, t, kins, lo3, hi3, active);
    check(c.s == 0 && c.joint >= 0, "-X from X4 Y-2 Z1: refused (a joint's first step goes out)");
    line(4, -2, 1, 4, 2, 1, f, t);
    c = jog_limit(f, t, kins, lo3, hi3, active);
    check(c.s > 0.5, "+Y from X4 Y-2 Z1: runs (both joints come in)");

    // the board way: the planner move goes to the cut end and comes to rest there
    KmState st;
    double pos[9] = {0};
    km.set_position(pos);
    for (int i = 0; i < 200; i++) { km.state(st); if (fabs(st.pos[0]) < 1e-9) break; std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
    line(0, 0, 0, 10, 0, 0, f, t);
    c = jog_limit(f, t, ident, lo, hi, active);
    double end[9];
    for (int i = 0; i < 9; i++) end[i] = f[i] + c.s * (t[i] - f[i]);
    Follower fo(km);
    int serial = km.line(end, 1.0, 100, false, 1);
    check(serial > 0, "line() returns the move's serial");
    bool begun = false;
    while (fo.t < 15) {
        fo.run(fo.t + 0.005, true);
        if ((int) fo.st.lines_begun >= serial) begun = true;
        if (begun && !fo.st.running) break;
    }
    printf("  planner move to the cut end X%.4f: at rest at X%.6f, taken up %d\n", end[0], fo.st.pos[0], (int) begun);
    check(begun && !fo.st.running && fabs(fo.st.pos[0] - 5) < 1e-6, "it ends at rest exactly at joint 0's limit, no abort");
    check(fo.st.errors == 0 && fo.messages.empty(), "no errors, no messages");
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

// the 3 Link's tap path as the planner cuts it (simulate, the kmotion-3link settings, the
// segment log /tmp/TPSegLog.csv): a rapid to Z0.8, then the tap's feed from Z0.8 to Z-0.2
// (bottom Z0.1 plus the 0.3 in allowance) at 0.08333 in/s - 100 RPM at 0.05 in/rev, as on
// the robot. Knots of the feed with no motion are pauses the board will make
static void scenario_tapsegs(KmBackend &km)
{
    Follower f(km);
    double top[9] = {0, 0, 0.8, 0, 0, 0, 0, 0, 0}, bottom[9] = {0, 0, -0.2, 0, 0, 0, 0, 0, 0};
    km.line(top, 3.0, 30.0, true, 1);
    f.run();
    check(f.at(0, 0, 0.8), "the rapid ends at Z0.8");
    km.line(bottom, 0.083333, 1e9, false, 2);
    f.t0 = std::chrono::steady_clock::now(); f.t = 0;
    f.run();
    printf("  t=%.2f: the feed ended at Z%.4f\n", f.t, f.st.pos[2]);
    check(f.at(0, 0, -0.2), "the feed ends at Z-0.2");
    printf("  the planner's knots are in %s/TPSegLog.csv\n", getenv("KMX_LOG_DIR"));
}

// threading (G33): lines under spindle sync form one run, planned at pitch x rev/s (here
// 0.05 x 10 = 0.5 in/s) and launched together; the first unsynchronized move ends it and
// runs after it. Simulate: the run is replayed like any other
static void scenario_thread(KmBackend &km)
{
    Follower f(km);
    double a[9] = {0, 0, -0.5, 0, 0, 0, 0, 0, 0}, b[9] = {0.1, 0, -0.6, 0, 0, 0, 0, 0, 0}, c[9] = {0.1, 0, 0, 0, 0, 0, 0, 0, 0};
    km.sync_line(a, 0.05, 10.0, 1e9, 1);
    km.sync_line(b, 0.05, 10.0, 1e9, 2);
    bool seen = false;
    for (int i = 0; i < 6; i++) { f.run(f.t + 0.01, true); if (f.st.threading) seen = true; }
    check(seen, "two synchronized lines open a threading run");
    km.line(c, 5.0, 100.0, true, 3);           // the pull-out: joins the run and ends it
    f.run();
    check(!f.st.threading, "the rapid ended it");
    check(f.extent.count(1) && f.extent.count(2) && f.extent.count(3), "all three lines ran");
    check(f.extent[2].lo[2] < -0.59 && f.at(0.1, 0, 0), "the thread reached Z-0.6 and the rapid came back to Z0");
    check(f.messages.empty(), "no messages");
}


// ---- the planner's knots from its segment log ---------------------------------------------
// The simulate replay publishes positions at the poll period; the planner's own output is finer:
// the cubic knots (CUBIC_KNOTS) in TPSegLog.csv, each axis a cubic of time over the knot. Their
// derivatives are the speed, acceleration and jerk the board is told to run, per actuator (per
// joint), exactly; the positions sampled at 1 ms, through the kinematics, are the pen's path
struct KnotLog {
    double vmax[8] = {}, amax[8] = {}, jmax[8] = {};
    std::vector<double> t;                      // sample times (1 ms apart within a knot)
    std::vector<std::array<double, 3>> p;       // CAD positions
    double total = 0;                           // the knots' total time
    int knots = 0;
    bool load(KmBackend &km, const std::string &path)
    {
        FILE *f = fopen(path.c_str(), "r");
        if (!f) return false;
        char line[8192];
        double tt = 0;
        while (fgets(line, sizeof line, f)) {
            if (line[0] != 'K') continue;
            // kind,plane,ccw,seq,dx,dwell,ntrips, 8 x P0, 8 x P1, xc, yc, tt0, the 32 coefficients
            double v[60] = {};
            int n = 0;
            for (char *tok = strtok(line, ","); tok && n < 60; tok = strtok(NULL, ",")) v[n++] = atof(tok);
            if (n < 58) continue;
            double T = v[25];
            if (!(T > 0)) continue;
            const double *K = &v[26];
            knots++;
            for (int a = 0; a < 8; a++) {
                double ka = K[4 * a], kb = K[4 * a + 1], kc = K[4 * a + 2];
                auto vel = [&](double x) { return (3 * ka * x + 2 * kb) * x + kc; };
                double vm = std::max(fabs(vel(0)), fabs(vel(T)));
                if (ka != 0) { double x = -kb / (3 * ka); if (x > 0 && x < T) vm = std::max(vm, fabs(vel(x))); }
                double am = std::max(fabs(2 * kb), fabs(6 * ka * T + 2 * kb));
                double jm = fabs(6 * ka);
                if (vm > vmax[a]) vmax[a] = vm;
                if (am > amax[a]) amax[a] = am;
                if (jm > jmax[a]) jmax[a] = jm;
            }
            for (double x = 0; x < T - 1e-9; x += 0.001) {
                double joints[8], cad[9];
                for (int a = 0; a < 8; a++) joints[a] = ((K[4 * a] * x + K[4 * a + 1]) * x + K[4 * a + 2]) * x + K[4 * a + 3];
                if (!km.joints_to_cad(joints, cad)) continue;
                t.push_back(tt + x);
                p.push_back({cad[0], cad[1], cad[2]});
            }
            tt += T;
        }
        fclose(f);
        total = tt;
        return knots > 0;
    }
    // the speed between samples i and i+1
    double speed(size_t i) const
    {
        if (i + 1 >= p.size()) return 0;
        double dt = t[i + 1] - t[i];
        if (dt <= 0) return 0;
        return sqrt(pow(p[i + 1][0] - p[i][0], 2) + pow(p[i + 1][1] - p[i][1], 2) + pow(p[i + 1][2] - p[i][2], 2)) / dt;
    }
};

// the programmed path as geometry: lines and counterclockwise XY arcs, to measure the executed
// path's deviation from it and to tell which element a sample belongs to
struct PathGeom {
    struct El { bool arc; double a[3], b[3], c[3], r, th0, dth; };
    std::vector<El> els;
    void line(const double *a, const double *b)
    {
        El e{}; e.arc = false; memcpy(e.a, a, sizeof e.a); memcpy(e.b, b, sizeof e.b); els.push_back(e);
    }
    void arc(const double *a, const double *b, const double *c)   // CCW about c in the XY plane
    {
        El e{}; e.arc = true; memcpy(e.a, a, sizeof e.a); memcpy(e.b, b, sizeof e.b); memcpy(e.c, c, sizeof e.c);
        e.r = hypot(a[0] - c[0], a[1] - c[1]);
        e.th0 = atan2(a[1] - c[1], a[0] - c[0]);
        e.dth = atan2(b[1] - c[1], b[0] - c[0]) - e.th0;
        if (e.dth <= 0) e.dth += 2 * M_PI;
        els.push_back(e);
    }
    static double dist_el(const El &e, const double *p)
    {
        if (!e.arc) {
            double d[3] = {e.b[0] - e.a[0], e.b[1] - e.a[1], e.b[2] - e.a[2]};
            double l2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
            double f = l2 > 0 ? ((p[0] - e.a[0]) * d[0] + (p[1] - e.a[1]) * d[1] + (p[2] - e.a[2]) * d[2]) / l2 : 0;
            f = std::min(1.0, std::max(0.0, f));
            return sqrt(pow(p[0] - e.a[0] - f * d[0], 2) + pow(p[1] - e.a[1] - f * d[1], 2) + pow(p[2] - e.a[2] - f * d[2], 2));
        }
        double th = atan2(p[1] - e.c[1], p[0] - e.c[0]) - e.th0;
        while (th < 0) th += 2 * M_PI;
        double dz = p[2] - e.c[2];
        if (th <= e.dth) return hypot(hypot(p[0] - e.c[0], p[1] - e.c[1]) - e.r, dz);
        double da = sqrt(pow(p[0] - e.a[0], 2) + pow(p[1] - e.a[1], 2) + pow(p[2] - e.a[2], 2));
        double db = sqrt(pow(p[0] - e.b[0], 2) + pow(p[1] - e.b[1], 2) + pow(p[2] - e.b[2], 2));
        return std::min(da, db);
    }
    int nearest(const double *p, double *dist) const
    {
        int best = -1;
        double bd = 1e30;
        for (size_t i = 0; i < els.size(); i++) { double d = dist_el(els[i], p); if (d < bd) { bd = d; best = (int) i; } }
        if (dist) *dist = bd;
        return best;
    }
};

// what one pass over the curve path came to: the time spent on the arc element, the speeds
// there, the deviation from the programmed path everywhere, the crawl at the 90 degree corner
struct CurveResult {
    double arc_time = 0, arc_vmax = 0, arc_vmean = 0, dev_max = 0, corner_vmin = 1e9, total = 0;
    double in_vmin = 1e9, in_vmax = 0, in_amax = 0;   // the arc's interior (10..41 of its 51 degrees): the
                                                      // speed's range and the tangential acceleration
    double jv[8] = {}, ja[8] = {}, jj[8] = {};
};
static CurveResult measure_curve(KmBackend &km, const std::string &logfile, const PathGeom &path, int arc_el,
                                 double arc_len, const double *corner, const char *what)
{
    CurveResult r;
    KnotLog kl;
    check(kl.load(km, logfile), "the knot log was written");
    std::string keep = logfile.substr(0, logfile.rfind('/')) + "/curve-" + what + ".csv";
    rename(logfile.c_str(), keep.c_str());      // this pass's knots, kept for a look
    int on_arc = 0;
    const PathGeom::El &arc = path.els[arc_el];
    double vprev = -1, tprev = 0;
    int dev_el = -1; double dev_v = 0, dev_t = 0; std::array<double, 3> dev_p{};
    for (size_t i = 0; i + 1 < kl.p.size(); i++) {
        double d;
        int el = path.nearest(kl.p[i].data(), &d);
        double v = kl.speed(i);
        if (d > r.dev_max) { r.dev_max = d; dev_el = el; dev_v = v; dev_t = kl.t[i]; dev_p = kl.p[i]; }
        if (el == arc_el) {
            on_arc++;
            if (v > r.arc_vmax) r.arc_vmax = v;
            double th = atan2(kl.p[i][1] - arc.c[1], kl.p[i][0] - arc.c[0]) - arc.th0;
            while (th < 0) th += 2 * M_PI;
            if (th > 10 * M_PI / 180 && th < 41 * M_PI / 180) {
                if (v < r.in_vmin) r.in_vmin = v;
                if (v > r.in_vmax) r.in_vmax = v;
                if (vprev >= 0 && kl.t[i] > tprev) r.in_amax = std::max(r.in_amax, fabs(v - vprev) / (kl.t[i] - tprev));
                vprev = v; tprev = kl.t[i];
            }
        }
        double dc = sqrt(pow(kl.p[i][0] - corner[0], 2) + pow(kl.p[i][1] - corner[1], 2) + pow(kl.p[i][2] - corner[2], 2));
        if (dc < 0.02 && v < r.corner_vmin) r.corner_vmin = v;
    }
    r.arc_time = on_arc * 0.001;
    r.arc_vmean = r.arc_time > 0 ? arc_len / r.arc_time : 0;
    r.total = kl.total;
    memcpy(r.jv, kl.vmax, sizeof r.jv); memcpy(r.ja, kl.amax, sizeof r.ja); memcpy(r.jj, kl.jmax, sizeof r.jj);
    printf("  %s: %d knots, %.2f s; on the arc %.2f s, mean %.3f in/s, max %.3f in/s; deviation max %.5f in; corner crawl %.3f in/s\n",
           what, kl.knots, kl.total, r.arc_time, r.arc_vmean, r.arc_vmax, r.dev_max, r.corner_vmin);
    printf("    the arc's interior: speed %.3f..%.3f in/s, tangential accel up to %.1f in/s^2\n", r.in_vmin, r.in_vmax, r.in_amax);
    printf("    the largest deviation is nearest element %d (0 approach, 1 arc, 2 exit, 3 after the corner) at t=%.3f X%.4f Y%.4f, speed %.3f in/s\n",
           dev_el, dev_t, dev_p[0], dev_p[1], dev_v);
    printf("    per joint max |v| %.2f %.2f %.2f  |a| %.1f %.1f %.1f  |j| %.0f %.0f %.0f\n",
           kl.vmax[0], kl.vmax[1], kl.vmax[2], kl.amax[0], kl.amax[1], kl.amax[2], kl.jmax[0], kl.jmax[1], kl.jmax[2]);
    return r;
}

// A curve made of short moves must run at the speed the filter window and the tolerance allow,
// not at one chord per window: an approach line, a 6.3 mm radius 51 degree G3 arc at F 2.5
// in/s (the chords ArcFeedAccel makes of it), its tangent exit line, then a 90 degree corner;
// then the same path with the arc as 1 degree G1 facets (a CAM polyline). Measured from the
// planner's knots: the speed on the arc, the deviation from the programmed path (within the
// corner tolerance everywhere), each joint's v/a/j within its limits, the corner still blended
// (a crawl, no stop), and the facets planned like the arc. KM_3LINK=1 runs it through the
// 3 Link kinematics at the robot's joint limits
static void scenario_curve(KmBackend &km, const KmConfig &cfg, const std::string &logdir)
{
    const double F = getenv("KM_CURVE_F") ? atof(getenv("KM_CURVE_F")) : 2.5, Acc = 30, Rapid = 3.0, Z = 0.1;
    const double R = getenv("KM_CURVE_R") ? atof(getenv("KM_CURVE_R")) : 0.248;                 // the arc's radius, inches
    const double ang = (getenv("KM_CURVE_DEG") ? atof(getenv("KM_CURVE_DEG")) : 51) * M_PI / 180;   // and angle
    const int nfacet = (int) lround(ang * 180 / M_PI);                                          // 1 degree facets
    const double tol = cfg.corner_tol;
    printf("  arc radius %.3f in, %.0f degrees, F %.2f in/s\n", R, ang * 180 / M_PI, F);
    double A[9] = {-0.5, -0.3, Z, 0, 0, 0, 0, 0, 0}, B[9] = {0, -0.3, Z, 0, 0, 0, 0, 0, 0};
    double O[3] = {0, -0.3 + R, Z};
    double C[9] = {R * sin(ang), -0.3 + R * (1 - cos(ang)), Z, 0, 0, 0, 0, 0, 0};
    double u[2] = {cos(ang), sin(ang)};                              // the tangent at C
    double D[9] = {C[0] + 0.4 * u[0], C[1] + 0.4 * u[1], Z, 0, 0, 0, 0, 0, 0};
    double E[9] = {D[0] - 0.4 * u[1], D[1] + 0.4 * u[0], Z, 0, 0, 0, 0, 0, 0};
    double nz[3] = {0, 0, 1};
    PathGeom path;
    path.line(A, B); path.arc(B, C, O); path.line(C, D); path.line(D, E);
    const std::string logfile = logdir + "/TPSegLog.csv";
    printf("  tolerance %.4f, collinear %.4f, kinematics %s\n", tol, cfg.collinear_tol, km.identity_kinematics() ? "identity" : cfg.kinematics);

    // the arc
    Follower f(km);
    km.line(A, Rapid, Acc, true, 1);
    f.run();
    check(f.at(A[0], A[1], A[2]), "the rapid reached the start");
    km.seglog_new_run();
    auto tp0 = std::chrono::steady_clock::now();
    km.line(B, F, Acc, false, 2);
    km.arc(C, O, nz, 0, F, Acc, 3);
    km.line(D, F, Acc, false, 4);
    km.line(E, F, Acc, false, 5);
    f.t0 = std::chrono::steady_clock::now(); f.t = 0;
    // the planner's own time: the worker takes the moves as fast as it plans them (no throttle
    // without a board), so the wait until the queue is empty is the planning time
    while (km.moves_queued() > 0) std::this_thread::sleep_for(std::chrono::microseconds(200));
    printf("  the planner took %.3f s to plan the arc path\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - tp0).count());
    f.run();
    check(f.at(E[0], E[1], E[2]), "the arc path ended at E");
    CurveResult ra = measure_curve(km, logfile, path, 1, R * ang, D, "arc");

    // the same arc as 1 degree facets
    km.line(A, Rapid, Acc, true, 10);
    f.run();
    km.seglog_new_run();
    tp0 = std::chrono::steady_clock::now();
    km.line(B, F, Acc, false, 11);
    int id = 12;
    for (int k = 1; k <= nfacet; k++) {
        double th = k * ang / nfacet;
        double P[9] = {R * sin(th), -0.3 + R * (1 - cos(th)), Z, 0, 0, 0, 0, 0, 0};
        km.line(P, F, Acc, false, id++);
    }
    km.line(D, F, Acc, false, id++);
    km.line(E, F, Acc, false, id++);
    f.t0 = std::chrono::steady_clock::now(); f.t = 0;
    while (km.moves_queued() > 0) std::this_thread::sleep_for(std::chrono::microseconds(200));
    printf("  the planner took %.3f s to plan the facet path\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - tp0).count());
    f.run();
    check(f.at(E[0], E[1], E[2]), "the facet path ended at E");
    CurveResult rf = measure_curve(km, logfile, path, 1, R * ang, D, "facets");

    // the joints' limits (the actuators'; the axes' without a kinematics)
    for (int i = 0; i < 3; i++) {
        const KmAxisParams &a = cfg.axis[i];
        bool kin = cfg.kinematics[0] != 0;
        double vl = kin ? a.act_max_vel : a.max_vel, al = kin ? a.act_max_accel : a.max_accel, jl = kin ? a.act_max_jerk : a.max_jerk;
        char what[128];
        snprintf(what, sizeof what, "joint %d within its limits: v %.2f/%.2f a %.1f/%.1f j %.0f/%.0f", i,
                 std::max(ra.jv[i], rf.jv[i]), vl, std::max(ra.ja[i], rf.ja[i]), al, std::max(ra.jj[i], rf.jj[i]), jl);
        check(std::max(ra.jv[i], rf.jv[i]) <= 1.02 * vl && std::max(ra.ja[i], rf.ja[i]) <= 1.02 * al && std::max(ra.jj[i], rf.jj[i]) <= 1.1 * jl, what);
    }
    check(ra.dev_max <= tol * 1.05, "the arc path stays within the tolerance");
    check(rf.dev_max <= tol * 1.05, "the facet path stays within the tolerance");
    check(ra.arc_vmean >= KM_CURVE_MIN_SPEED, "the arc runs at the speed the window allows, not one chord per window");
    check(rf.arc_vmean >= KM_CURVE_MIN_SPEED, "the facets run at that speed too");
    check(fabs(rf.arc_time - ra.arc_time) <= 0.25 * ra.arc_time, "the facets plan like the arc (times within 25%)");
    // at this tolerance the 90 degree corner's blend would crawl below 5% of the feed, so the
    // planner stops there instead (TP3_CORNER_STOP_FRAC), with or without the facet rule
    check(ra.corner_vmin < 0.005 && rf.corner_vmin < 0.005, "the 90 degree corner stops, as before");
    check(f.messages.empty(), "no messages");
}

// A zigzag of tiny moves meeting at a sharp angle (60 degrees; KM_ZIG_DEG, KM_ZIG_LEN vary it):
// the moves are too short to hold a blend of the tolerance, so the facet rule hands the
// junctions to the planner raw and the planner's own corner caps bound them, where the blend
// of old crawled or the corner policy stopped at every vertex. Measures the treatment the
// settings select: the time over the zigzag, the deviation from the polyline, the stops, the
// joints' limits
static void scenario_zigzag(KmBackend &km, const KmConfig &cfg, const std::string &logdir)
{
    const double F = 2.5, Acc = 30, Rapid = 3.0, Z = 0.1, tol = cfg.corner_tol;
    const double turn = (getenv("KM_ZIG_DEG") ? atof(getenv("KM_ZIG_DEG")) : 60) * M_PI / 180;
    const double seg = getenv("KM_ZIG_LEN") ? atof(getenv("KM_ZIG_LEN")) : 0.008;
    const int n = 20;
    printf("  %d moves of %.4f in turning %.0f degrees at each vertex, F %.2f, tolerance %.4f, %s\n", n, seg, turn * 180 / M_PI, F, tol,
           km.identity_kinematics() ? "identity" : cfg.kinematics);
    double A[9] = {-0.5, -0.3, Z, 0, 0, 0, 0, 0, 0}, B[9] = {0, -0.3, Z, 0, 0, 0, 0, 0, 0};
    PathGeom path;
    path.line(A, B);
    std::vector<std::array<double, 9>> pts;
    std::array<double, 9> P = {0, -0.3, Z, 0, 0, 0, 0, 0, 0};
    for (int k = 0; k < n; k++) {
        double d = (k % 2 == 0 ? 1 : -1) * turn / 2;
        std::array<double, 9> Q = P;
        Q[0] += seg * cos(d); Q[1] += seg * sin(d);
        path.line(P.data(), Q.data());
        pts.push_back(Q);
        P = Q;
    }
    double E[9] = {P[0] + 0.4, P[1], Z, 0, 0, 0, 0, 0, 0};
    path.line(P.data(), E);
    Follower f(km);
    km.line(A, Rapid, Acc, true, 1);
    f.run();
    km.seglog_new_run();
    int id = 2;
    km.line(B, F, Acc, false, id++);
    for (auto &q : pts) km.line(q.data(), F, Acc, false, id++);
    km.line(E, F, Acc, false, id++);
    f.t0 = std::chrono::steady_clock::now(); f.t = 0;
    f.run();
    check(f.at(E[0], E[1], E[2]), "the path ended at E");
    KnotLog kl;
    check(kl.load(km, logdir + "/TPSegLog.csv"), "the knot log was written");
    double dev = 0, vmin = 1e9, zig_len = n * seg;
    int on_zig = 0, stops = 0;
    bool stopped = false;
    for (size_t i = 0; i + 1 < kl.p.size(); i++) {
        double d;
        int el = path.nearest(kl.p[i].data(), &d);
        if (d > dev) dev = d;
        double v = kl.speed(i);
        if (el >= 1 && el <= n) {
            on_zig++;
            if (v < vmin) vmin = v;
            if (v < 0.005) { if (!stopped) stops++; stopped = true; } else stopped = false;
        }
    }
    printf("  zigzag: %.2f s for %.3f in (mean %.3f in/s, slowest %.3f in/s), %d stop(s); deviation max %.5f in; whole path %.2f s\n",
           on_zig * 0.001, zig_len, on_zig > 0 ? zig_len / (on_zig * 0.001) : 0, vmin, stops, dev, kl.total);
    printf("    per joint max |v| %.2f %.2f %.2f  |a| %.1f %.1f %.1f  |j| %.0f %.0f %.0f\n",
           kl.vmax[0], kl.vmax[1], kl.vmax[2], kl.amax[0], kl.amax[1], kl.amax[2], kl.jmax[0], kl.jmax[1], kl.jmax[2]);
    for (int i = 0; i < 3; i++) {
        const KmAxisParams &a = cfg.axis[i];
        bool kin = cfg.kinematics[0] != 0;
        double vl = kin ? a.act_max_vel : a.max_vel, al = kin ? a.act_max_accel : a.max_accel, jl = kin ? a.act_max_jerk : a.max_jerk;
        char what[96];
        snprintf(what, sizeof what, "joint %d within its limits", i);
        check(kl.vmax[i] <= 1.02 * vl && kl.amax[i] <= 1.02 * al && kl.jmax[i] <= 1.1 * jl, what);
    }
    check(dev <= tol * 1.05, "the path stays within the tolerance");
    if (cfg.curve_facets && km.identity_kinematics() && !getenv("KM_ZIG_DEG") && !getenv("KM_ZIG_LEN")) {
        check(stops == 0, "no stop at the vertices (the corner policy stopped at all 21)");
        check(on_zig > 0 && zig_len / (on_zig * 0.001) > 0.1, "faster than 0.1 in/s (was 0.05)");
    }
    check(f.messages.empty(), "no messages");
}

// replays a canon dump of a G-code program (LinuxCNC's rs274 -g file.ngc: STRAIGHT_FEED,
// STRAIGHT_TRAVERSE, ARC_FEED in the XY plane, SET_FEED_RATE, USE_LENGTH_UNITS,
// SET_MOTION_CONTROL_MODE) as task would hand it over: KM_CANON=<file>, KM_OFFSET="x y z" the
// work offset in inches (the program's G54). Prints the program's time and each move's
static void scenario_canon(KmBackend &km, double rapid, double acc)
{
    const char *file = getenv("KM_CANON");
    if (!file) { check(false, "KM_CANON=<canon dump> names the program"); return; }
    double off[3] = {0, 0, 0};
    if (getenv("KM_OFFSET")) sscanf(getenv("KM_OFFSET"), "%lf %lf %lf", &off[0], &off[1], &off[2]);
    FILE *fp = fopen(file, "r");
    if (!fp) { check(false, "the canon dump opens"); return; }
    double scale = 1.0, feed = 0;
    KmState st;
    km.state(st);
    double cur[3] = {st.pos[0], st.pos[1], st.pos[2]};
    struct Move { int id; bool rapid, arc; double len; };
    std::vector<Move> moves;
    char line[1024];
    int id = 0;
    while (fgets(line, sizeof line, fp)) {
        char *c = strchr(line, 'N');
        if (!c) continue;
        c = strchr(c, ' ');
        if (!c) continue;
        c++;
        double a[9];
        if (strstr(c, "USE_LENGTH_UNITS")) scale = strstr(c, "MM") ? 1 / 25.4 : 1.0;
        else if (sscanf(c, "SET_FEED_RATE(%lf)", &a[0]) == 1) feed = a[0] * scale / 60;
        else if (strstr(c, "SET_MOTION_CONTROL_MODE")) {
            if (sscanf(c, "SET_MOTION_CONTROL_MODE(CANON_CONTINUOUS, %lf)", &a[0]) == 1) km.set_path_mode(2, a[0] * scale);
            else if (strstr(c, "EXACT_PATH")) km.set_path_mode(1, 0);
            else if (strstr(c, "EXACT_STOP")) km.set_path_mode(0, 0);
        } else if (sscanf(c, "STRAIGHT_TRAVERSE(%lf, %lf, %lf", &a[0], &a[1], &a[2]) == 3 || sscanf(c, "STRAIGHT_FEED(%lf, %lf, %lf", &a[0], &a[1], &a[2]) == 3) {
            bool rp = strstr(c, "TRAVERSE") != NULL;
            double e[9] = {a[0] * scale + off[0], a[1] * scale + off[1], a[2] * scale + off[2], 0, 0, 0, 0, 0, 0};
            moves.push_back({++id, rp, false, sqrt(pow(e[0] - cur[0], 2) + pow(e[1] - cur[1], 2) + pow(e[2] - cur[2], 2))});
            km.line(e, rp ? rapid : feed, acc, rp, id);
            memcpy(cur, e, sizeof cur);
        } else if (sscanf(c, "ARC_FEED(%lf, %lf, %lf, %lf, %lf, %lf", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]) == 6) {
            double e[9] = {a[0] * scale + off[0], a[1] * scale + off[1], a[5] * scale + off[2], 0, 0, 0, 0, 0, 0};
            double ctr[3] = {a[2] * scale + off[0], a[3] * scale + off[1], cur[2]}, nz[3] = {0, 0, 1};
            int rot = (int) a[4];
            int turn = rot > 0 ? rot - 1 : rot;
            double r = hypot(cur[0] - ctr[0], cur[1] - ctr[1]);
            double th = atan2(e[1] - ctr[1], e[0] - ctr[0]) - atan2(cur[1] - ctr[1], cur[0] - ctr[0]);
            if (rot > 0) { if (th <= 0) th += 2 * M_PI; } else { if (th >= 0) th -= 2 * M_PI; }
            moves.push_back({++id, false, true, r * fabs(th)});
            km.arc(e, ctr, nz, turn, feed, acc, id);
            memcpy(cur, e, sizeof cur);
        } else if (strstr(c, "PROGRAM_END")) break;
    }
    fclose(fp);
    printf("  %d moves handed over\n", id);
    auto tp0 = std::chrono::steady_clock::now();
    // a long first move through a kinematics takes the planner a while: wait for the replay
    for (int i = 0; i < 6000; i++) { km.state(st); if (st.running) break; std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
    Follower f(km);
    std::thread planwatch([&] {       // the planner's own time: the worker takes the moves as it plans them
        while (km.moves_queued() > 0) std::this_thread::sleep_for(std::chrono::milliseconds(2));
        printf("  the planner took %.2f s to plan the program\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - tp0).count());
    });
    f.run(600);
    planwatch.join();
    double feed_len = 0, feed_time = 0, arc_len = 0, arc_time = 0;
    for (const Move &m : moves) {
        if (!f.t_start.count(m.id)) continue;
        double dur = f.t_end.count(m.id) ? f.t_end[m.id] - f.t_start[m.id] : 0;
        if (!m.rapid) { feed_len += m.len; feed_time += dur; }
        if (m.arc) { arc_len += m.len; arc_time += dur; printf("  arc %3d: %.3f in in %.2f s = %.3f in/s\n", m.id, m.len, dur, dur > 0 ? m.len / dur : 0); }
    }
    printf("  program time %.2f s; feeds %.2f in in %.2f s (mean %.3f in/s); arcs %.2f in in %.2f s (mean %.3f in/s)\n",
           f.t, feed_len, feed_time, feed_time > 0 ? feed_len / feed_time : 0, arc_len, arc_time, arc_time > 0 ? arc_len / arc_time : 0);
    check(f.at(cur[0], cur[1], cur[2]), "the program ended at its last point");
    check(f.messages.empty(), "no messages");
}

int main(int argc, char **argv)
{
    const char *scenario = argc > 1 ? argv[1] : "program";
    // the planner's logs (TPSegLog.csv, TP3_Timeline.log) go to their own directory, never to
    // /tmp where a LinuxCNC running beside the test writes its own
    if (!getenv("KMX_LOG_DIR")) { mkdir("/tmp/backend-test", 0777); setenv("KMX_LOG_DIR", "/tmp/backend-test", 1); }
    const std::string logdir = getenv("KMX_LOG_DIR");
    KmConfig cfg;
    cfg.log_segments = true;
    for (int i = 0; i < 3; i++) { cfg.axis[i].counts_per_unit = 4000; cfg.axis[i].max_vel = 4; cfg.axis[i].max_accel = 100; cfg.axis[i].max_jerk = 1000; }
    if (getenv("KM_KOGNA")) {               // the kmotion-kogna settings, simulate mode
        for (int i = 0; i < 3; i++) { cfg.axis[i].counts_per_unit = 2540; cfg.axis[i].max_vel = 9.84; cfg.axis[i].max_accel = 98.4; cfg.axis[i].max_jerk = 984; }
    }
    const bool link3 = getenv("KM_3LINK") || strcmp(scenario, "kins3link") == 0 || strcmp(scenario, "tapsegs") == 0;
    if (link3) {                            // the kmotion-3link settings, simulate mode
        snprintf(cfg.kinematics, sizeof cfg.kinematics, "3Link");
        for (int i = 0; i < 3; i++) {
            KmAxisParams &a = cfg.axis[i];
            a.counts_per_unit = 11.37778; a.max_vel = 3; a.max_accel = 30; a.max_jerk = 300;
            a.act_max_vel = i == 0 ? 35 : 17.6; a.act_max_accel = 700; a.act_max_jerk = 7000; a.act_degrees = true;
            if (getenv("KM_ACT_VEL")) a.act_max_vel = atof(getenv("KM_ACT_VEL"));     // the joints' limits, all alike
            if (getenv("KM_ACT_JERK")) a.act_max_jerk = atof(getenv("KM_ACT_JERK"));
        }
    }
    if (strcmp(scenario, "curve") == 0 || strcmp(scenario, "canon") == 0 || strcmp(scenario, "zigzag") == 0) { cfg.corner_tol = 0.002; cfg.collinear_tol = 0.0005; }   // the robot's
    if (getenv("KM_CURVE_FACETS")) cfg.curve_facets = atoi(getenv("KM_CURVE_FACETS")) != 0;   // 0: every junction a corner, as before
    if (getenv("KM_CORNER_TOL")) cfg.corner_tol = atof(getenv("KM_CORNER_TOL"));
    if (getenv("KM_COLLINEAR_TOL")) cfg.collinear_tol = atof(getenv("KM_COLLINEAR_TOL"));
    if (getenv("KM_JERK")) for (int i = 0; i < 3; i++) cfg.axis[i].max_jerk = atof(getenv("KM_JERK"));
    if (getenv("KM_BREAK_ANGLE")) cfg.break_angle = atof(getenv("KM_BREAK_ANGLE"));
    if (getenv("KM_LOOKAHEAD")) cfg.lookahead = atof(getenv("KM_LOOKAHEAD"));
    const bool exact = getenv("KM_EXACT") != NULL;      // the path mode G61 for the scenario (default G64)
    if (strcmp(scenario, "gated") == 0) cfg.queue_limit = 20;     // the backend must know task's limit
    if (getenv("KM_BOARD")) {
        cfg.simulate = false;
        const std::string kogna = kogna_config_dir();
        cfg.init_programs.push_back(kogna + "/MinirouterInit.c");   // the bench's copy, as kmotion-kogna.ini runs it
        cfg.init_thread = 1;
        snprintf(cfg.home_program, sizeof cfg.home_program, "%s", (kogna + "/Home.c").c_str());
        if (strcmp(scenario, "tap") == 0) {               // Tap.c on thread 4, the bench's spindle below
            snprintf(cfg.tap_program, sizeof cfg.tap_program, "%s", (kogna + "/Tap.c").c_str());
            cfg.tap_thread = 4;
            cfg.tap_overshoot_revs = 6;
        }
        if (strcmp(scenario, "spindle") == 0 || strcmp(scenario, "tap") == 0) {   // as kmotion-kogna.ini sets it up
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
        if (strcmp(scenario, "limits") == 0) {       // the switch simulator, as the ini's commented START_PROGRAM
            cfg.start_programs.emplace_back(7, kogna + "/../../../../C Programs/SimulateHomes.c");
            for (int i = 0; i < 3; i++) { cfg.limit_neg_bit[i] = 49 + i; cfg.limit_pos_bit[i] = 52 + i; }
            cfg.limit_level = 1;
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
    if (exact) km->set_path_mode(1, 0);                 // G61: exact path, as LinuxCNC's canned cycles run
    printf("mode: %s, scenario: %s, logs in %s\n", km->mode_name(), scenario, logdir.c_str());
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
    else if (strcmp(scenario, "limits") == 0) scenario_limits(*km);
    else if (strcmp(scenario, "offon") == 0) scenario_offon(*km);
    else if (strcmp(scenario, "measure") == 0) scenario_measure(*km);
    else if (strcmp(scenario, "tap") == 0) scenario_tap(*km);
    else if (strcmp(scenario, "holdjog") == 0) scenario_holdjog(*km);
    else if (strcmp(scenario, "probe") == 0) scenario_probe(*km);
    else if (strcmp(scenario, "kins3link") == 0) scenario_kins3link(*km);
    else if (strcmp(scenario, "joglimit") == 0) scenario_joglimit(*km);
    else if (strcmp(scenario, "tapsegs") == 0) scenario_tapsegs(*km);
    else if (strcmp(scenario, "thread") == 0) scenario_thread(*km);
    else if (strcmp(scenario, "curve") == 0) scenario_curve(*km, cfg, logdir);
    else if (strcmp(scenario, "zigzag") == 0) scenario_zigzag(*km, cfg, logdir);
    else if (strcmp(scenario, "canon") == 0) scenario_canon(*km, link3 ? 3.0 : cfg.axis[0].max_vel, link3 ? 30.0 : cfg.axis[0].max_accel);
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
