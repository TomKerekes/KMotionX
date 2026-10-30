// rawusb: the KFLOP's USB link through libftdi alone (no KMotionServer): the time of a GetStatus
// reply (1577 bytes: 174 hex words in 11 lines) and of a burst of them, with the FTDI event
// character on or off and a given latency timer. Each ftdi_read_data return is one USB bulk
// transfer's worth: its size shows how many packets arrive per transfer.
//   rawusb <latency_ms> <eventchar 0|1> <burst>
#include <ftdi.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
static double now_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static const int STATUS_BYTES = 1577;
int main(int argc, char **argv)
{
    int latency = argc > 1 ? atoi(argv[1]) : 2, evt = argc > 2 ? atoi(argv[2]) : 1, burst = argc > 3 ? atoi(argv[3]) : 5;
    struct ftdi_context *f = ftdi_new();
    if (ftdi_usb_open(f, 0x0403, 0x6001) < 0) { fprintf(stderr, "open: %s\n", ftdi_get_error_string(f)); return 1; }
    unsigned char old = 0; ftdi_get_latency_timer(f, &old);
    ftdi_set_latency_timer(f, latency);
    ftdi_set_event_char(f, '\n', evt);
    f->usb_read_timeout = 1000;
    unsigned char buf[8192];
    auto drain = [&] { for (;;) { int n = ftdi_read_data(f, buf, sizeof buf); if (n <= 0) break; } };
    drain();
    printf("latency timer %d ms (was %d), event char %s, chunk %u\n", latency, old, evt ? "on" : "off", f->readbuffer_chunksize);
    const char cmd[] = "\x1b\x01GetStatus\r";
    // single status reads, each read's size recorded for the first one
    std::vector<double> t;
    for (int i = 0; i < 20; i++) {
        double a = now_s();
        ftdi_write_data(f, (const unsigned char *) cmd, strlen(cmd));
        int total = 0, reads = 0;
        while (total < STATUS_BYTES) {
            int n = ftdi_read_data(f, buf, sizeof buf);
            if (n < 0) { fprintf(stderr, "read: %s\n", ftdi_get_error_string(f)); return 1; }
            if (n == 0) { if (now_s() - a > 1) { printf("timeout after %d bytes\n", total); break; } continue; }
            if (i == 0) printf("  read %d: %d bytes at %.2f ms\n", reads + 1, n, (now_s() - a) * 1e3);
            total += n; reads++;
        }
        t.push_back(now_s() - a);
        if (i == 0) printf("  first status: %d bytes in %d reads\n", total, reads);
    }
    std::sort(t.begin(), t.end());
    printf("one GetStatus (%d bytes): min %.1f median %.1f max %.1f ms\n", STATUS_BYTES, t[0] * 1e3, t[10] * 1e3, t[19] * 1e3);
    // a burst: the board answers burst commands back to back - the link's sustained rate
    std::string many; for (int i = 0; i < burst; i++) many += cmd;
    double a = now_s();
    ftdi_write_data(f, (const unsigned char *) many.c_str(), many.size());
    int total = 0, reads = 0, maxread = 0;
    while (total < burst * STATUS_BYTES) {
        int n = ftdi_read_data(f, buf, sizeof buf);
        if (n < 0) { fprintf(stderr, "read: %s\n", ftdi_get_error_string(f)); return 1; }
        if (n == 0) { if (now_s() - a > 5) { printf("burst timeout after %d bytes\n", total); break; } continue; }
        total += n; reads++; if (n > maxread) maxread = n;
    }
    double d = now_s() - a;
    printf("burst of %d: %d bytes in %.1f ms = %.1f KB/s, %d reads (largest %d bytes), %.2f ms per 64-byte packet\n",
           burst, total, d * 1e3, total / d / 1e3, reads, maxread, d * 1e3 / (total / 64.0));
    ftdi_set_latency_timer(f, old);
    ftdi_set_event_char(f, '\n', 1);
    ftdi_usb_close(f); ftdi_free(f);
    return 0;
}
