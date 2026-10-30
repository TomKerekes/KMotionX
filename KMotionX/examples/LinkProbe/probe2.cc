// probe2: one status read taken apart - the pipe round trips to the server, the USB write, each
// line of the reply (read-only commands: GetStatus, Version).
#include "CoordMotion.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>
static double now_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static void report(const char *what, std::vector<double> t)
{
    if (t.empty()) return;
    std::sort(t.begin(), t.end());
    double sum = 0; for (double x : t) sum += x;
    printf("%-44s n=%3zu  min %6.2f  median %6.2f  mean %6.2f  max %6.2f ms\n", what, t.size(),
           t.front() * 1e3, t[t.size() / 2] * 1e3, sum / t.size() * 1e3, t.back() * 1e3);
}
int main()
{
    CKMotionDLL km(0);
    char reply[MAX_LINE + 1];
    std::vector<double> t;
    // 1. the server round trip alone: lock + release (no USB traffic once connected)
    for (int i = 0; i < 50; i++) { double a = now_s(); int r = km.WaitToken(false, 1000, "probe2"); if (r != KMOTION_LOCKED) { printf("lock failed %d\n", r); return 1; } km.ReleaseToken(); t.push_back(now_s() - a); }
    report("WaitToken + ReleaseToken (2 pipe round trips)", t);
    // 2. a write and its one-line reply, timed apart, under one token
    std::vector<double> tw, tr;
    for (int i = 0; i < 50; i++) {
        if (km.WaitToken(false, 1000, "probe2") != KMOTION_LOCKED) return 1;
        double a = now_s(); km.WriteLine("Version"); double b = now_s();
        km.ReadLineTimeOut(reply, 5000); double c = now_s();
        km.ReleaseToken();
        tw.push_back(b - a); tr.push_back(c - b);
    }
    report("WriteLine(Version): pipe + USB write", tw);
    report("ReadLineTimeOut: pipe + USB read, 1 line", tr);
    // 3. a status read line by line: how many lines, how long each
    MAIN_STATUS st;
    int n = sizeof st / sizeof(int);
    std::vector<double> tl, tall; std::vector<int> lens; int lines_per = 0;
    for (int i = 0; i < 20; i++) {
        if (km.WaitToken(false, 1000, "probe2") != KMOTION_LOCKED) return 1;
        double a = now_s();
        km.WriteLine("GetStatus");
        int words = 0, lines = 0;
        while (words < n) {
            double b = now_s();
            if (km.ReadLineTimeOut(reply, 5000)) { printf("read failed\n"); break; }
            double d = now_s() - b;
            int len = strlen(reply);
            int w = (len + 1) / 9;          // 8 hex chars + a space per word
            words += w; lines++;
            if (i == 0) { lens.push_back(len); printf("  line %d: %d chars (%d words) in %.2f ms\n", lines, len, w, d * 1e3); }
            tl.push_back(d);
        }
        km.ReleaseToken();
        tall.push_back(now_s() - a);
        lines_per = lines;
    }
    printf("status: %d words, %d lines per read\n", n, lines_per);
    report("one status line (pipe + USB read)", tl);
    report("whole status read under one token", tall);
    // 4. back-to-back ReadLineTimeOut with nothing to read (the server's empty poll)
    t.clear();
    if (km.WaitToken(false, 1000, "probe2") != KMOTION_LOCKED) return 1;
    for (int i = 0; i < 10; i++) { double a = now_s(); int r = km.ReadLineTimeOut(reply, 20); t.push_back(now_s() - a); if (i == 0) printf("  empty read returned %d after %.1f ms (timeout 20)\n", r, t.back() * 1e3); }
    km.ReleaseToken();
    report("ReadLineTimeOut with nothing to read, 20 ms timeout", t);
    return 0;
}
