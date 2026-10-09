#!/usr/bin/env python3
"""Run PCCommTest.c on the board (MDI M101) against a running LinuxCNC on kmotion-kogna and
check what its status shows afterwards; the per-command ok/FAIL lines come from the board's
console on kmotion-motion's stderr (the terminal LinuxCNC was started from with -v -d), and
the commands and their results are in /tmp/kmotion-motion.log ("pccomm:" lines).

Start LinuxCNC first (source linuxcnc-dev's rip-environment, then `linuxcnc -v -d
kmotion-kogna.ini`), then run this. Turns the machine on and homes it if needed."""
import linuxcnc, time

c = linuxcnc.command()
s = linuxcnc.stat()
e = linuxcnc.error_channel()
t0 = time.time()
fails = 0


def log(*a):
    print("%6.2f " % (time.time() - t0) + " ".join(str(x) for x in a), flush=True)


def check(ok, what):
    global fails
    log("  %s   %s" % ("ok  " if ok else "FAIL", what))
    if not ok:
        fails += 1


def errs(tag):
    while True:
        m = e.poll()
        if not m:
            break
        log("  %s: %s %s" % (tag, m[0], m[1]))


def wait(cond, what, timeout=15):
    t = time.time()
    while time.time() - t < timeout:
        s.poll()
        if cond():
            return True
        time.sleep(0.05)
    log("  TIMEOUT waiting for", what)
    return False


def dro_x():
    s.poll()
    return s.actual_position[0] - s.g5x_offset[0] - s.g92_offset[0] - s.tool_offset[0]


s.poll()
if s.task_state != linuxcnc.STATE_ON:
    c.state(linuxcnc.STATE_ESTOP_RESET); c.wait_complete()
    c.state(linuxcnc.STATE_ON); c.wait_complete()
    wait(lambda: s.task_state == linuxcnc.STATE_ON, "machine on"); errs("on")
if not all(s.homed[:3]):
    c.mode(linuxcnc.MODE_MANUAL); c.wait_complete()
    c.teleop_enable(0); c.wait_complete()
    c.home(-1)
    wait(lambda: all(s.homed[:3]), "home all", 120); errs("home")
log("on and homed:", s.task_state == linuxcnc.STATE_ON, s.homed[:3])

x_before = dro_x()
log("X DRO before: %.4f" % x_before)

c.mode(linuxcnc.MODE_MDI); c.wait_complete()
c.mdi("M101")
wait(lambda: s.interp_state == linuxcnc.INTERP_IDLE, "M101", 10)
log("M101 sent: PCCommTest.c runs on the board; watch the console for its checks")

# the board's SET_FRO 0.5 then x1.5: 75% while it runs
wait(lambda: abs(s.feedrate - 0.75) < 0.001, "feed override 75% from the board", 20)
check(abs(s.feedrate - 0.75) < 0.001, "SET_FRO / SET_FRO_INC: feed override 75%% (%.2f)" % s.feedrate)
# its SET_X 1.25
wait(lambda: abs(dro_x() - 1.25) < 0.001, "X touched off to 1.25", 20)
check(abs(dro_x() - 1.25) < 0.001, "SET_X: X DRO 1.25 (%.4f)" % dro_x())
# the end: the override back to 100%
wait(lambda: abs(s.feedrate - 1.0) < 0.001, "the test's end (feed override 100%)", 120)
check(abs(s.feedrate - 1.0) < 0.001, "SET_FRO 1.0 at the end (%.2f)" % s.feedrate)
time.sleep(0.5)
errs("during the test")

# put the touch-off back
wait(lambda: s.interp_state == linuxcnc.INTERP_IDLE, "idle", 10)
c.mode(linuxcnc.MODE_MDI); c.wait_complete()
c.mdi("G10 L20 P0 X%.6f" % x_before)
wait(lambda: s.interp_state == linuxcnc.INTERP_IDLE, "restore", 10)
check(abs(dro_x() - x_before) < 0.001, "X DRO restored (%.4f)" % dro_x())
errs("restore")
log("pccomm-test: %d failure(s) seen from LinuxCNC; the board's own count is on the console" % fails)
