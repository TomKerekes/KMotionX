#!/usr/bin/env python3
"""Machine on as soon as LinuxCNC is up, when the board's axes are still enabled from the
last session ([EMC] KEEP_ENABLED_ON_EXIT leaves them so, as KMotionCNC does): E-stop reset and
Machine on, nothing else - no init program, no homing (homing is the operator's or the
board's business, as in KMotionCNC; [TRAJ] NO_FORCE_HOMING lets moves run unhomed). With any
axis disabled, LinuxCNC is left in E-stop as it normally starts. Run by LinuxCNC right after
the display program starts ([APPLICATIONS] APP, DELAY 1): task is up by then, the GUI is not
yet, and a GUI patched for KEEP_ENABLED_ON_EXIT (linuxcnc/patches/) leaves a machine it finds
on alone. Logs to /tmp/kmotion-autostart.log, since a desktop launch shows nothing."""
import linuxcnc, subprocess, sys, time

LOG = "/tmp/kmotion-autostart.log"


def say(msg, err=False):
    line = time.strftime("%H:%M:%S ") + msg
    print(line, file=sys.stderr if err else sys.stdout)
    with open(LOG, "a") as f:
        f.write(line + "\n")


c = linuxcnc.command()
s = linuxcnc.stat()


def wait(cond, timeout):
    t = time.time()
    while time.time() - t < timeout:
        s.poll()
        if cond():
            return True
        time.sleep(0.05)
    return False


try:
    enabled = subprocess.run(["halcmd", "getp", "kmotion.all-enabled"], capture_output=True, text=True, timeout=5).stdout.strip() == "TRUE"
except Exception as e:
    say("autostart: cannot read kmotion.all-enabled: %s" % e, True)
    sys.exit(1)
if not enabled:
    say("autostart: an axis is disabled on the board: LinuxCNC starts in E-stop")
    sys.exit(0)
s.poll()
say("autostart: kmotion.all-enabled TRUE, task_state %d" % s.task_state)
c.state(linuxcnc.STATE_ESTOP_RESET); c.wait_complete()
c.state(linuxcnc.STATE_ON); c.wait_complete()
if not wait(lambda: s.task_state == linuxcnc.STATE_ON, 10):
    say("autostart: the machine did not come on (task_state %d)" % s.task_state, True)
    sys.exit(1)
# joints the board remembers as homed ([KMOTION] HOMED_PERSIST, read into kmotion.homed-mask
# at a start with every axis enabled): their homing request is answered where they stand,
# nothing moves. With [KMOTION] ALWAYS_HOMED they are homed with the machine on already
try:
    mask = int(subprocess.run(["halcmd", "getp", "kmotion.homed-mask"], capture_output=True, text=True, timeout=5).stdout.strip() or "0")
except Exception:
    mask = 0
s.poll()
njoints = len(s.joint)
todo = [j for j in range(min(njoints, 8)) if mask & (1 << j) and not s.homed[j]]
if todo:
    c.mode(linuxcnc.MODE_MANUAL); c.wait_complete()
    c.teleop_enable(0); c.wait_complete()
    for j in todo:
        c.home(j)
    wait(lambda: all(s.homed[j] for j in todo), 10)
    say("autostart: joints %s homed where they stand (the board remembered them)" % todo)
c.mode(linuxcnc.MODE_MANUAL); c.wait_complete()
c.teleop_enable(1); c.wait_complete()          # world mode: the GUI jogs the pen, not a servo
s.poll()
say("autostart: machine on (the axes were enabled on the board), homed %s" % str(s.homed[:min(njoints, 8)]))
