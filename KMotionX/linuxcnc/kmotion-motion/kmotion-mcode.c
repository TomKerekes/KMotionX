/*
 * kmotion-mcode: LinuxCNC's user M codes M100-M199 on the KMotion board.
 *
 * Task runs the executable M1xx it finds in [RS274NGC] USER_M_PATH as "M1xx <P> <Q>" (-1 for a
 * word not given) and waits for it to exit; a nonzero exit is an error that stops the program.
 * A config's M1xx is a wrapper: exec kmotion-mcode <n> "$@". This sends "<n> <P> <Q>" to the
 * running kmotion-motion (Unix socket, abstract name "kmotion-motion"), which carries out
 * [KMOTION] MCODE_<n> on the board, and waits for its "ok" or "error".
 */
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: kmotion-mcode <100-199> [<P> [<Q>]]\n");
        return 2;
    }
    int code = atoi(argv[1]);
    double p = argc > 2 ? atof(argv[2]) : -1.0, q = argc > 3 ? atof(argv[3]) : -1.0;

    int s = socket(AF_UNIX, SOCK_STREAM, 0);
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    static const char name[] = "\0kmotion-motion";
    memcpy(addr.sun_path, name, sizeof name - 1);
    if (s < 0 || connect(s, (struct sockaddr *) &addr, (socklen_t) (offsetof(struct sockaddr_un, sun_path) + sizeof name - 1)) < 0) {
        fprintf(stderr, "kmotion-mcode: M%d: kmotion-motion is not running (or has no user M codes)\n", code);
        return 1;
    }
    char req[128];
    int n = snprintf(req, sizeof req, "%d %.10g %.10g\n", code, p, q);
    if (write(s, req, (size_t) n) != n) {
        fprintf(stderr, "kmotion-mcode: M%d: could not send the request\n", code);
        return 1;
    }
    // the reply comes when the action is over (a program waited for may take a while)
    char reply[256];
    size_t len = 0;
    for (;;) {
        ssize_t r = read(s, reply + len, sizeof reply - 1 - len);
        if (r <= 0) break;
        len += (size_t) r;
        if (memchr(reply, '\n', len) || len == sizeof reply - 1) break;
    }
    reply[len] = 0;
    close(s);
    if (strncmp(reply, "ok", 2) == 0) return 0;
    fprintf(stderr, "kmotion-mcode: M%d: %s", code, len ? reply : "no reply from kmotion-motion\n");
    return 1;
}
