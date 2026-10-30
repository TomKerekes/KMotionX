// linkprobe: read-only timing of the KMotion link through KMotionServer (GetStatus, Version),
// alone and while another thread polls status the way kmotion-motion's poller does.
#include "CoordMotion.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>
static double now_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static void report(const char *what, std::vector<double> t)
{
    if (t.empty()) return;
    std::sort(t.begin(), t.end());
    double sum = 0; for (double x : t) sum += x;
    printf("%-36s n=%2zu  min %6.1f  median %6.1f  mean %6.1f  max %6.1f ms\n", what, t.size(),
           t.front() * 1e3, t[t.size() / 2] * 1e3, sum / t.size() * 1e3, t.back() * 1e3);
}
int main()
{
    CKMotionDLL km(0);
    MAIN_STATUS st;
    printf("MAIN_STATUS %zu bytes = %zu words (%zu hex chars)\n", sizeof st, sizeof st / 4, sizeof st / 4 * 9);
    std::vector<double> t;
    char reply[MAX_LINE + 1] = "";
    for (int i = 0; i < 30; i++) { double a = now_s(); int r = km.GetStatus(st, true); t.push_back(now_s() - a); if (r) printf("GetStatus failed %d\n", r); }
    report("GetStatus alone", t);
    printf("  status VersionAndSize 0x%x -> %d words\n", st.VersionAndSize, st.VersionAndSize & 0xffff);
    t.clear();
    for (int i = 0; i < 30; i++) { double a = now_s(); int r = km.WriteLineReadLine("Version", reply); t.push_back(now_s() - a); if (r) printf("Version failed %d\n", r); }
    report("Version alone", t);
    printf("  reply: %s\n", reply);
    for (int mode = 0; mode < 2; mode++) {
        std::atomic<bool> stop{false};
        std::atomic<int> polls{0};
        std::vector<double> pt;
        std::thread p([&] {
            while (!stop) {
                double a = now_s();
                km.GetStatus(st, true);
                pt.push_back(now_s() - a);
                polls++;
                if (mode == 1) { double s = 0.020 - (now_s() - a); if (s > 0) std::this_thread::sleep_for(std::chrono::duration<double>(s)); }
            }
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        t.clear();
        for (int i = 0; i < 12; i++) {
            double a = now_s();
            int r = km.WriteLineReadLine("Version", reply);
            t.push_back(now_s() - a);
            if (r) printf("Version failed %d\n", r);
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
        }
        stop = true; p.join();
        report(mode == 0 ? "Version, poller flat out" : "Version, poller at 20 ms period", t);
        report(mode == 0 ? "  (poller's GetStatus, flat out)" : "  (poller's GetStatus, 20 ms)", pt);
    }
    return 0;
}
