#!/usr/bin/env python3
"""Machine on as soon as LinuxCNC is up, when the board's axes are still enabled from the
last session (KEEP_ENABLED_ON_EXIT leaves them so, as KMotionCNC does): E-stop reset and
Machine on, nothing else - no init program, no homing (homing is the operator's or the
board's business, as in KMotionCNC; [TRAJ] NO_FORCE_HOMING lets moves run unhomed). With any
axis disabled, LinuxCNC is left in E-stop as it normally starts. Run by LinuxCNC after the GUI
starts ([APPLICATIONS] APP in the ini)."""
import linuxcnc, os, subprocess, sys, time

c = linuxcnc.command()
s = linuxcnc.stat()
ini = linuxcnc.ini(os.environ["INI_FILE_NAME"])
LOG = "/tmp/kmotion-autostart.log"       # a desktop launch shows nothing: what happened, for the record


def say(msg, err=False):
    line = time.strftime("%H:%M:%S ") + msg
    print(line, file=sys.stderr if err else sys.stdout)
    with open(LOG, "a") as f:
        f.write(line + "\n")


def wait(cond, timeout):
    t = time.time()
    while time.time() - t < timeout:
        s.poll()
        if cond():
            return True
        time.sleep(0.1)
    return False


try:
    enabled = subprocess.run(["halcmd", "getp", "kmotion.all-enabled"], capture_output=True, text=True, timeout=5).stdout.strip() == "TRUE"
except Exception as e:
    say("autostart: cannot read kmotion.all-enabled: %s" % e, True)
    sys.exit(1)
if not enabled:
    say("autostart: an axis is disabled on the board: LinuxCNC starts in E-stop")
    sys.exit(0)
# the GUI's own init puts the machine into E-stop reset if it comes up after us (hence
# [APPLICATIONS] DELAY): try a few times, a little apart
say("autostart: kmotion.all-enabled TRUE, pid %d" % os.getpid())
# act once the GUI is up, not before: its init can reset the machine (gmoccapy's does) and
# would undo a Machine on made earlier. A GUI registers a HAL component and marks it ready at
# the end of its init: gmoccapy as "gmoccapy", AXIS as "axisui"
gui = (ini.find("DISPLAY", "DISPLAY") or "").strip()
comp = {"axis": "axisui"}.get(gui, gui)


def gui_ready():
    try:
        out = subprocess.run(["halcmd", "show", "comp"], capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return False
    return any(len(f := line.split()) >= 5 and f[2] == comp and f[4] == "ready" for line in out.splitlines())


t0 = time.time()
if comp and wait(gui_ready, 60):
    say("autostart: the GUI (%s) is up after %.0f s" % (comp, time.time() - t0))
    # gmoccapy marks its component ready before its init is over, and the init can still
    # reset the machine (E-stop to E-stop reset) a few seconds later: wait for that change,
    # or 8 s
    s.poll()
    state0 = s.task_state
    if wait(lambda: s.task_state != state0, 8):
        say("autostart: the GUI's init changed the state to %d" % s.task_state)
    time.sleep(1)
else:
    say("autostart: no ready HAL component %r within 60 s, going ahead" % comp)
time.sleep(1)
for attempt in range(3):
    s.poll()
    say("autostart: attempt %d, task_state %d" % (attempt + 1, s.task_state))
    if s.task_state != linuxcnc.STATE_ON:
        c.state(linuxcnc.STATE_ESTOP_RESET); c.wait_complete()
        c.state(linuxcnc.STATE_ON); c.wait_complete()
        wait(lambda: s.task_state == linuxcnc.STATE_ON, 90)
    time.sleep(3)                                # the GUI, if it is still coming up, acts within this
    s.poll()
    if s.task_state == linuxcnc.STATE_ON:
        c.mode(linuxcnc.MODE_MANUAL); c.wait_complete()
        c.teleop_enable(1); c.wait_complete()      # world mode: the GUI jogs the pen, not a servo
        say("autostart: machine on (the axes were enabled on the board), attempt %d" % (attempt + 1))
        sys.exit(0)
say("autostart: the machine did not stay on", True)
sys.exit(1)
