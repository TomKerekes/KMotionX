// backend-test: exercises KmBackend alone (no LinuxCNC, no board) with resume-test.ngc's moves,
// followed in real time at 5 ms. Build and run with "make test".
#include "kmotion-backend.h"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <thread>
int main()
{
    KmConfig cfg;
    cfg.log_segments = true;
    for (int i = 0; i < 3; i++) { cfg.axis[i].counts_per_unit = 4000; cfg.axis[i].max_vel = 4; cfg.axis[i].max_accel = 100; cfg.axis[i].max_jerk = 1000; }
    double pos[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    KmBackend km;
    if (!km.init(cfg, pos)) { printf("init failed\n"); return 1; }
    printf("mode: %s\n", km.mode_name());
    double F = 20.0 / 60.0;                       // F20 in/min
    double e1[9] = {0, 0, 0.2, 0,0,0,0,0,0};      // G0 Z0.2
    double e2[9] = {1, 0, 0.2, 0,0,0,0,0,0};      // G1 X1
    double e3[9] = {1, 1, 0.2, 0,0,0,0,0,0};      // G1 Y1
    double e4[9] = {1, 1, 0.2, 0,0,0,0,0,0};      // G3 full circle I-1 J0
    double c4[3] = {0, 1, 0.2}, n4[3] = {0, 0, 1};
    double e5[9] = {1, 1, 0.4, 0,0,0,0,0,0};      // G0 Z0.4
    double e6[9] = {0, 0, 0.4, 0,0,0,0,0,0};      // G0 X0 Y0
    km.line(e1, 5.657, 141.4, true, 3);
    km.line(e2, F, 100, false, 6);
    km.line(e3, F, 100, false, 7);
    km.arc(e4, c4, n4, 0, F, 100, 9);
    km.line(e5, 4, 100, true, 10);
    km.line(e6, 5.657, 141.4, true, 11);
    printf("moves handed over (the worker flushes 50 ms after the last one)\n");
    KmState st;
    auto t0 = std::chrono::steady_clock::now();
    double vmax = 0, last_report = -1; int last_id = -1; double t = 0;
    for (int n = 0; n < 40000; n++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        km.state(st);
        t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (st.message[0]) printf("  message: %s\n", st.message);
        if (st.current_vel > vmax) vmax = st.current_vel;
        if (st.active_id != last_id) { printf("  t=%6.2f line %2d starts at X%.3f Y%.3f Z%.3f depth %d\n", t, st.active_id, st.pos[0], st.pos[1], st.pos[2], st.depth); last_id = st.active_id; }
        if (t - last_report >= 5) { last_report = t; printf("  t=%6.2f X%.3f Y%.3f Z%.3f v=%.3f\n", t, st.pos[0], st.pos[1], st.pos[2], st.current_vel); }
        if (!st.running && t > 0.5) break;
    }
    printf("done at t=%.2f s: X%.4f Y%.4f Z%.4f, depth %d, max speed %.3f in/s\n", t, st.pos[0], st.pos[1], st.pos[2], st.depth, vmax);
    return 0;
}
