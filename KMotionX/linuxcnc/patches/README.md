# Patches to LinuxCNC for `[EMC] KEEP_ENABLED_ON_EXIT`

LinuxCNC ends every session by disabling the machine: task's shutdown sends a motion
DISABLE, and gmoccapy's quit sends Machine off and E-stop before that. On a machine whose
axes keep their state across sessions (absolute servos, the 3 Link robot) KMotionCNC leaves
them enabled, so a restart needs no init and no homing. These two patches add the ini key
`[EMC] KEEP_ENABLED_ON_EXIT = 1`, off by default (a stock LinuxCNC, or the key absent, ends
a session as always):

- `task-keep-enabled-on-exit.patch` (`src/emc/task/`): task reads the key with its other
  `[EMC]` settings; with it set, its shutdown (`emcMotionHalt`) does not disable motion.
- `gmoccapy-keep-enabled-on-exit.patch` (`src/emc/usr_intf/gmoccapy/gmoccapy.py`): with the
  key set, the quit sends no Machine off and no E-stop, and the E-stop button's start-up
  reset leaves a machine that is already on alone (the start-up script
  `configs/kmotion-3link/autostart.py` turns it on before the GUI is up).

Other GUIs decide for themselves what they send at init and at quit: AXIS sends nothing at
either and needs no patch; another one would need its own.

On the motion side `kmotion-motion` reads the same key: the DISABLE that task sends as part
of its own start (before the session's first ENABLE) leaves the board's axes as they are;
every other DISABLE disables them at once. No timing is involved.

Apply, in the LinuxCNC run-in-place tree (the pinned commit of `linuxcnc/README.md`):

    cd ~/linuxcnc-dev
    git apply ~/KMotionX/KMotionX/linuxcnc/patches/*.patch
    cd src && make

(`milltask` is rebuilt; `bin/gmoccapy` is regenerated from the patched source.)
