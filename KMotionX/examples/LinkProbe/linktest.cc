// linktest: the kmotion-motion backend in board mode against the KFLOP (machine off, nothing
// moves): how long a board call from another thread waits while the poller reads status.
// persist 150 is read and written back unchanged; Jog0=0 stops a jog that is not running.
#include "kmotion-backend.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>
static double now_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static void report(const char *what, std::vector<double> t)
{
    std::sort(t.begin(), t.end());
    double sum = 0; for (double x : t) sum += x;
    printf("%-44s n=%2zu  min %6.1f  median %6.1f  mean %6.1f  max %6.1f ms\n", what, t.size(),
           t.front() * 1e3, t[t.size() / 2] * 1e3, sum / t.size() * 1e3, t.back() * 1e3);
}
int main(int argc, char **argv)
{
    KmConfig cfg;
    cfg.simulate = false;
    cfg.status_period = (argc > 1 ? atof(argv[1]) : 20) * 1e-3;
    int ch[8] = {0, 2, 3, -1, -1, -1, -1, -1};
    memcpy(cfg.channel, ch, sizeof ch);
    double cpu[3] = {3183, 2540, 10000}, vel[3] = {8, 8, 2}, acc[3] = {10, 10, 20}, jerk[3] = {7854.23, 3937.01, 1000};
    for (int i = 0; i < 3; i++) { cfg.axis[i].counts_per_unit = cpu[i]; cfg.axis[i].max_vel = vel[i]; cfg.axis[i].max_accel = acc[i]; cfg.axis[i].max_jerk = jerk[i]; }
    cfg.third_order = false;
    cfg.actuator_limits = false;
    cfg.homed_persist = -1;
    KmBackend km;
    double pos[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    if (!km.init(cfg, pos)) { fprintf(stderr, "init failed\n"); return 1; }
    printf("status period %.0f ms\n", cfg.status_period * 1e3);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    int v = 0;
    if (!km.persist_get(150, v)) { fprintf(stderr, "persist_get failed\n"); return 1; }
    printf("persist 150 = %d (written back unchanged)\n", v);
    std::vector<double> t;
    for (int i = 0; i < 40; i++) { double a = now_s(); bool ok = km.persist_set(150, v); t.push_back(now_s() - a); if (!ok) printf("persist_set failed\n"); std::this_thread::sleep_for(std::chrono::milliseconds(150)); }
    report("persist_set (a WriteLine), poller running", t);
    t.clear();
    for (int i = 0; i < 40; i++) { double a = now_s(); km.persist_get(150, v); t.push_back(now_s() - a); std::this_thread::sleep_for(std::chrono::milliseconds(150)); }
    report("persist_get (a WriteLineReadLine), poller running", t);
    for (int i = 0; i < 40; i++) { km.jog_stop(0); std::this_thread::sleep_for(std::chrono::milliseconds(150)); }   // the urgent queue (KM_DEBUG_STOP prints each send)
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    printf("done\n");
    return 0;
}
