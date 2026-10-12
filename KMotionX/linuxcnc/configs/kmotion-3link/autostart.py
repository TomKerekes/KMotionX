#!/usr/bin/env python3
"""Machine on and homed as soon as LinuxCNC is up, as KMotionCNC leaves a machine after its
init: E-stop reset, Machine on (the enable program sets the servos up and enables them where
they stand), Home All (the servos are absolute: every joint is homed where it is, nothing
moves). Run by LinuxCNC after the GUI starts ([APPLICATIONS] APP in the ini); gives up quietly
if the machine does not come on (the GUI shows why)."""
import linuxcnc, sys, time

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


s.poll()
if s.task_state != linuxcnc.STATE_ON:
    c.state(linuxcnc.STATE_ESTOP_RESET); c.wait_complete()
    c.state(linuxcnc.STATE_ON); c.wait_complete()
    if not wait(lambda: s.task_state == linuxcnc.STATE_ON, 90):
        print("autostart: the machine did not come on", file=sys.stderr)
        sys.exit(1)
if not all(s.homed[:3]):
    c.mode(linuxcnc.MODE_MANUAL); c.wait_complete()
    c.teleop_enable(0); c.wait_complete()
    c.home(-1)
    if not wait(lambda: all(s.homed[:3]), 30):
        print("autostart: homing did not finish", file=sys.stderr)
        sys.exit(1)
    c.teleop_enable(1); c.wait_complete()      # world mode: the GUI jogs the pen, not a servo
print("autostart: machine on and homed")
