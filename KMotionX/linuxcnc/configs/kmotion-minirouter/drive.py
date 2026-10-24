#!/usr/bin/env python3
"""Drive the kmotion-sim config through LinuxCNC's Python interface and print what
task reports back: E-stop reset, machine on, home all, a continuous and an
incremental jog, MDI moves, then a program with a pause and resume halfway.

Start LinuxCNC first (source linuxcnc-dev's rip-environment, then
`linuxcnc -v -d kmotion-sim.ini`), then run this. An optional argument names the
G-code program; the default is KMotionX's tests/resume-test.ngc (ends at X0 Y0 Z0.4)."""
import linuxcnc, os, sys, time

PROGRAM = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/KMotionX/tests/resume-test.ngc')
c = linuxcnc.command()
s = linuxcnc.stat()
e = linuxcnc.error_channel()
t0 = time.time()


def log(*a):
    print("%6.2f " % (time.time() - t0) + " ".join(str(x) for x in a), flush=True)


def errs(tag):
    while True:
        m = e.poll()
        if not m:
            break
        log("  %s: %s %s" % (tag, m[0], m[1]))


def pos():
    s.poll()
    return tuple(round(v, 3) for v in s.actual_position[:3])


def wait(cond, what, timeout=15):
    t = time.time()
    while time.time() - t < timeout:
        s.poll()
        if cond():
            return True
        time.sleep(0.05)
    log("  TIMEOUT waiting for", what)
    return False


c.state(linuxcnc.STATE_ESTOP_RESET); c.wait_complete()
c.state(linuxcnc.STATE_ON); c.wait_complete()
wait(lambda: s.task_state == linuxcnc.STATE_ON, "machine on"); errs("on")
log("machine on:", s.task_state == linuxcnc.STATE_ON)

c.mode(linuxcnc.MODE_MANUAL); c.wait_complete()
c.teleop_enable(0); c.wait_complete()
c.home(-1)
wait(lambda: all(s.homed[:3]), "home all", 20); errs("home")
log("homed:", s.homed[:3], "position", pos())

# jogs in world (teleop) mode: 1 s continuous at 1 unit/s, then -0.5 incremental
c.teleop_enable(1); c.wait_complete()
c.jog(linuxcnc.JOG_CONTINUOUS, False, 0, 1.0)
time.sleep(1.0)
c.jog(linuxcnc.JOG_STOP, False, 0)
time.sleep(0.3)
log("after a 1 s jog +X at 1/s:", pos(), "(X about 1)")
c.jog(linuxcnc.JOG_INCREMENT, False, 0, 1.0, -0.5)
time.sleep(1.0)
log("after an incremental jog of -0.5:", pos(), "(X about 0.5)")
errs("jog")

c.mode(linuxcnc.MODE_MDI); c.wait_complete()
for cmd in ("G20 G90", "G0 X1 Y1", "G1 X2 F20", "G1 Y2", "G0 Z0.1"):
    c.mdi(cmd)
    wait(lambda: s.interp_state == linuxcnc.INTERP_IDLE, "MDI " + cmd, 30)
    log("MDI %-10s -> %s  inpos %s  queue %d" % (cmd, pos(), s.inpos, s.queue))
errs("mdi")

c.mode(linuxcnc.MODE_AUTO); c.wait_complete()
c.program_open(PROGRAM); c.wait_complete()
c.auto(linuxcnc.AUTO_RUN, 0)
wait(lambda: s.interp_state != linuxcnc.INTERP_IDLE, "program start", 5)
log("program running")
time.sleep(4.0)
c.auto(linuxcnc.AUTO_PAUSE)
time.sleep(0.5)
p1 = pos(); s.poll(); log("paused at", p1, "line", s.motion_line, "paused flag", s.paused)
time.sleep(1.0)
p2 = pos(); log("1 s later", p2, "- holds position:", p1 == p2)
c.auto(linuxcnc.AUTO_RESUME)
last = time.time()
while time.time() - t0 < 120:
    s.poll()
    if s.interp_state == linuxcnc.INTERP_IDLE:
        break
    if time.time() - last >= 3:
        last = time.time(); log("  line %d at %s vel %.3f" % (s.motion_line, pos(), s.current_vel))
    time.sleep(0.1)
errs("auto")
log("program done: interp %d, position %s, queue %d, inpos %s" % (s.interp_state, pos(), s.queue, s.inpos))
