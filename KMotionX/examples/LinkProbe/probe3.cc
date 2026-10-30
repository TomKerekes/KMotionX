// probe3: (1) how often another client holds the board's token (the KMotion tool, say): 300 lock
// attempts spaced 20 ms; (2) a burst of 5 GetStatus under one token: the link's sustained rate
// through the server, event character on (read-only commands).
#include "CoordMotion.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>
static double now_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
int main()
{
    CKMotionDLL km(0);
    char reply[MAX_LINE + 1];
    int waited = 0, in_use = 0; double worst = 0, sum = 0;
    for (int i = 0; i < 300; i++) {
        double a = now_s();
        int r = km.WaitToken(false, 1000, "probe3");
        double d = now_s() - a;
        if (r == KMOTION_LOCKED) km.ReleaseToken(); else in_use++;
        if (d > 0.003) waited++;
        if (d > worst) worst = d;
        sum += d;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    printf("token: 300 tries over 6 s: %d waited over 3 ms (worst %.1f ms, mean %.2f ms), %d refused\n", waited, worst * 1e3, sum / 300 * 1e3, in_use);
    MAIN_STATUS st;
    int n = sizeof st / sizeof(int);
    for (int burst = 1; burst <= 5; burst += 4) {
        if (km.WaitToken(false, 1000, "probe3") != KMOTION_LOCKED) return 1;
        double a = now_s();
        for (int b = 0; b < burst; b++) km.WriteLine("GetStatus");
        double w = now_s() - a;
        int words = 0, lines = 0, bytes = 0;
        std::vector<double> tl;
        while (words < n * burst) {
            double b0 = now_s();
            if (km.ReadLineTimeOut(reply, 5000)) { printf("read failed\n"); break; }
            tl.push_back(now_s() - b0);
            int len = strlen(reply);
            words += (len + 1) / 9; lines++; bytes += len;
        }
        double d = now_s() - a;
        km.ReleaseToken();
        std::sort(tl.begin(), tl.end());
        printf("burst of %d GetStatus: %d writes in %.1f ms; %d lines, %d bytes in %.1f ms = %.1f KB/s = %.2f ms per 64-byte packet; line min %.2f median %.2f max %.2f ms\n",
               burst, burst, w * 1e3, lines, bytes, d * 1e3, bytes / d / 1e3, d * 1e3 / (bytes / 64.0), tl.front() * 1e3, tl[tl.size() / 2] * 1e3, tl.back() * 1e3);
    }
    return 0;
}
