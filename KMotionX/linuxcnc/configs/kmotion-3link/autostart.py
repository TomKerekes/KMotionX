#!/usr/bin/env python3
"""Machine on as soon as LinuxCNC is up, when the board's axes are still enabled from the
last session (KEEP_ENABLED_ON_EXIT leaves them so, as KMotionCNC does): E-stop reset and
Machine on, nothing else - no init program, no homing (homing is the operator's or the
board's business, as in KMotionCNC; [TRAJ] NO_FORCE_HOMING lets moves run unhomed). With any
axis disabled, LinuxCNC is left in E-stop as it normally starts. Run by LinuxCNC after the GUI
starts ([APPLICATIONS] APP in the ini)."""
import linuxcnc, subprocess, sys, time

c = linuxcnc.command()
s = linuxcnc.stat()


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
    print("autostart: cannot read kmotion.all-enabled:", e, file=sys.stderr)
    sys.exit(1)
if not enabled:
    print("autostart: an axis is disabled on the board: LinuxCNC starts in E-stop")
    sys.exit(0)
# the GUI's own init puts the machine into E-stop reset if it comes up after us (hence
# [APPLICATIONS] DELAY): try a few times, a little apart
for attempt in range(3):
    s.poll()
    if s.task_state != linuxcnc.STATE_ON:
        c.state(linuxcnc.STATE_ESTOP_RESET); c.wait_complete()
        c.state(linuxcnc.STATE_ON); c.wait_complete()
        wait(lambda: s.task_state == linuxcnc.STATE_ON, 90)
    time.sleep(3)                                # the GUI, if it is still coming up, acts within this
    s.poll()
    if s.task_state == linuxcnc.STATE_ON:
        c.mode(linuxcnc.MODE_MANUAL); c.wait_complete()
        c.teleop_enable(1); c.wait_complete()      # world mode: the GUI jogs the pen, not a servo
        print("autostart: machine on (the axes were enabled on the board)")
        sys.exit(0)
print("autostart: the machine did not stay on", file=sys.stderr)
sys.exit(1)
