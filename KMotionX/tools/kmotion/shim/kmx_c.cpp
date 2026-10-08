/*
 * kmx_c - a C interface to KMotionX's CKMotionDLL for the KMotion tool (Python, ctypes).
 *
 * One handle = one client connection to KMotionServer, which the server serves beside
 * other clients (LinuxCNC's kmotion-motion, kmxWeb); the board token serializes the
 * commands. Console output from the board and the library's error messages are queued
 * here and read out by the Python side.
 */
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>
#include "KMotionDLL.h"

static std::mutex g_mx;
static std::deque<std::string> g_console, g_errors;

// the Linux port's MessageBox prints to the terminal and waits for Enter on stdin, which
// would freeze a GUI: route it into the error queue instead. Questions get "no"/"cancel",
// a GUI cannot answer them for the user.
extern MB_USER_CALLBACK *mb_callback;
static int messagebox_handler(const wchar_t *title, const wchar_t *msg, uint32_t uType)
{
    std::string text;
    if (title) { char b[256]; snprintf(b, sizeof b, "%ls: ", title); text += b; }
    if (msg) { char b[2048]; snprintf(b, sizeof b, "%ls", msg); text += b; }
    {
        std::lock_guard<std::mutex> lock(g_mx);
        if (g_errors.size() < 200) g_errors.push_back(text);
    }
    uint32_t options = uType & 0xf;
    if (options == MB_YESNO) return IDNO;
    if (options == MB_OKCANCEL) return IDCANCEL;
    return IDOK;
}

static int console_handler(const char *msg)
{
    std::lock_guard<std::mutex> lock(g_mx);
    if (g_console.size() < 2000) g_console.push_back(msg ? msg : "");
    return 0;
}
static void err_handler(const char *msg)
{
    std::lock_guard<std::mutex> lock(g_mx);
    if (g_errors.size() < 200) g_errors.push_back(msg ? msg : "");
}
static int pop(std::deque<std::string> &q, char *buf, int len)
{
    std::lock_guard<std::mutex> lock(g_mx);
    if (q.empty() || len <= 0) return 0;
    snprintf(buf, len, "%s", q.front().c_str());
    q.pop_front();
    return (int) strlen(buf);
}

extern "C" {

// board_id 0 = the first board the server finds (USB id, Kogna serial or static IP otherwise)
void *kmx_open(int board_id)
{
    mb_callback = messagebox_handler;
    CKMotionDLL *km = new CKMotionDLL(board_id);
    km->SetErrMsgCallback(err_handler);
    return km;
}
void kmx_close(void *h) { delete (CKMotionDLL *) h; }

// console lines (printf from C programs) are pushed by the server to the client that
// registered; kmx_console_read drains them
int kmx_set_console(void *h) { return ((CKMotionDLL *) h)->SetConsoleCallback(console_handler); }
int kmx_service_console(void *h) { return ((CKMotionDLL *) h)->ServiceConsole(); }
int kmx_console_read(char *buf, int len) { return pop(g_console, buf, len); }
int kmx_error_read(char *buf, int len) { return pop(g_errors, buf, len); }

// one console command; the reply buffer should hold MAX_LINE (2560) bytes
int kmx_write_line(void *h, const char *s) { return ((CKMotionDLL *) h)->WriteLine(s); }
int kmx_write_read(void *h, const char *s, char *reply, int len)
{
    char r[MAX_LINE + 1];
    r[0] = 0;
    int rc = ((CKMotionDLL *) h)->WriteLineReadLine(s, r);
    snprintf(reply, len, "%s", r);
    return rc;
}

// firmware check and board type (BOARD_TYPE_KFLOP / BOARD_TYPE_KOGNA as in PC-DSP.h)
int kmx_check_version(void *h, int *board_type) { return ((CKMotionDLL *) h)->CheckKMotionVersion(board_type); }
int kmx_board_type(void *h)
{
    int type = 0;
    if (((CKMotionDLL *) h)->CheckKMotionVersion(&type, true)) return -1;
    return type;
}

// the whole MAIN_STATUS block as the library fills it; Python mirrors the struct and
// checks kmx_status_size() against its own layout
int kmx_status_size(void) { return (int) sizeof(MAIN_STATUS); }
int kmx_status(void *h, void *buf, int len)
{
    MAIN_STATUS st;
    memset(&st, 0, sizeof st);
    int rc = ((CKMotionDLL *) h)->GetStatus(st, true);
    if (rc) return rc;
    if (len < (int) sizeof st) return -2;
    memcpy(buf, &st, sizeof st);
    return 0;
}

// C programs: compile with tcc67 for the connected board type, download to the thread
int kmx_compile_load(void *h, const char *path, int thread, char *err, int errlen)
{
    if (err && errlen > 0) err[0] = 0;
    return ((CKMotionDLL *) h)->CompileAndLoadCoff(path, thread, err, errlen > 0 ? errlen - 1 : 0);
}
int kmx_load(void *h, const char *path, int thread) { return ((CKMotionDLL *) h)->LoadCoff(thread, path, 0); }

// compile only (tcc67 for the given board type), to the .out file for that thread; and the
// .out name the library derives from a source name and a thread
int kmx_compile(void *h, const char *path, const char *out, int board_type, int thread, char *err, int errlen)
{
    if (err && errlen > 0) err[0] = 0;
    return ((CKMotionDLL *) h)->Compile(path, out, board_type, thread, err, errlen > 0 ? errlen - 1 : 0);
}
int kmx_out_name(void *h, int thread, const char *path, char *out, int outlen)
{
    ((CKMotionDLL *) h)->ConvertToOut(thread, path, out, outlen);
    return 0;
}

// the console's way: an echoed command, then the board's reply lines read back until its
// "Ready"; the token is held across the sequence so other clients do not interleave
int kmx_wait_token(void *h, int timeout_ms) { return ((CKMotionDLL *) h)->WaitToken(false, timeout_ms, "kmotion-tool"); }
void kmx_release_token(void *h) { ((CKMotionDLL *) h)->ReleaseToken(); }
int kmx_write_line_echo(void *h, const char *s) { return ((CKMotionDLL *) h)->WriteLineWithEcho(s); }
int kmx_read_line_timeout(void *h, char *buf, int len, int timeout_ms)
{
    char r[MAX_LINE + 1];
    r[0] = 0;
    int rc = ((CKMotionDLL *) h)->ReadLineTimeOut(r, timeout_ms);
    snprintf(buf, len, "%s", r);
    return rc;
}
int kmx_check_ready(void *h) { return ((CKMotionDLL *) h)->CheckForReady(); }
int kmx_disconnect(void *h) { return ((CKMotionDLL *) h)->Disconnect(); }
int kmx_const(int which)
{
    switch (which) {
    case 0: return KMOTION_LOCKED;
    case 1: return KMOTION_IN_USE;
    case 2: return KMOTION_READY;
    case 3: return KMOTION_NOT_CONNECTED;
    case 4: return KMOTION_TIMEOUT;
    }
    return -1;
}

// firmware: load an image "packed to flash" (PackToFlash 1) or into RAM for the KFLOP's
// boot loader recovery (2); the recovery handshake lock; and the whole New Version
// sequence as KMotion.exe does it: LoadCoff(-1, image, 1), then ProgFlashImage under the
// token and CheckForReady until READY. Returns 0 done, 1 timeout, 2 the board reported
// an error (flash in an undefined state), 3 the image download failed.
int kmx_load_pack(void *h, const char *path, int pack) { return ((CKMotionDLL *) h)->LoadCoff(-1, path, pack); }
int kmx_lock_recovery(void *h) { return ((CKMotionDLL *) h)->KMotionLockRecovery(); }
int kmx_flash_new_version(void *h, const char *path, int timeout_s)
{
    CKMotionDLL *km = (CKMotionDLL *) h;
    if (km->LoadCoff(-1, path, 1)) return 3;
    if (km->WaitToken(false, 5000, "ProgFlashImage") != KMOTION_LOCKED) return 3;
    int rc = 1;
    if (km->WriteLineWithEcho("ProgFlashImage") == 0) {
        for (int i = 0; i < timeout_s * 2; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            int r = km->CheckForReady();
            if (r == KMOTION_READY) { rc = 0; break; }
            if (r == KMOTION_ERROR) { rc = 2; break; }
        }
    } else {
        rc = 3;
    }
    km->ReleaseToken();
    return rc;
}

// the board this connection is on, as the server knows it (KMotion.exe's title): a KFLOP's
// USB location ID or a Kogna's IP address (a.b.c.d from the high byte down); -1 while the
// server has no connection to it. A local query: nothing goes to the board
int kmx_usb_location(void *h) { return ((CKMotionDLL *) h)->USBLocation(); }

// which boards the server sees
int kmx_list_locations(void *h, int *list, int max)
{
    int n = 0, tmp[256];
    if (((CKMotionDLL *) h)->ListLocations(&n, tmp)) return -1;
    if (n > max) n = max;
    memcpy(list, tmp, n * sizeof(int));
    return n;
}

} // extern "C"
