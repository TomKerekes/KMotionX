# Patches to LinuxCNC

## For `[EMC] KEEP_ENABLED_ON_EXIT`

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
  `configs/kmotion-3link/autostart.py` turns it on before the GUI is up), and the exit
  button stays available while the machine is on (stock gmoccapy disables it then, since
  its exit would turn the machine off; the window's close button works either way).

The gmoccapy patch also adds `[DISPLAY] CLEAR_PLOT_AT_FIRST_MACHINE_ON = 1`: a machine
whose position is unknown until its axes are enabled (serial servos read 0 until they are
set up) shows a stand-in position at the start, and the first Machine on draws a line in
the plot from there to the real position; with the key, the first position change after the
session's first Machine on clears the plot.

A third patch, `gremlin-backplot-clear.patch` (`lib/python/gremlin.py`,
`lib/python/rs274/glcanon_gl.py`, `glcanon_scene.py`), fixes the clearing itself, in
LinuxCNC 2.10's renderer: the live plot is kept in a GPU buffer that is only re-sent when
the logger's point count shrinks, and after a clear the logger refills a two-point trail
within milliseconds, so the plot kept drawing the old segment (the eraser button has the
same flaw: it does nothing until the tool has moved). With the patch the buffer also
recognises the trail by its first point, and gremlin redraws once the logger's trail is not
the one it had before the clear. No timing is involved.

## For `[RS274NGC] RIGID_TAP_CYCLES`

`interp-rigid-tap-cycles.patch` (`src/emc/rs274ngc/`): with the key set, the interpreter's
G84 and G74 cycles are rigid taps - the cycle's rapid to the retract plane, then the same
synchronized tap as G33.1 (`START_SPEED_FEED_SYNCH` with the pitch, `RIGID_TAP` to the
bottom), which the controller runs to the bottom, reverses and brings back to the retract
plane. The pitch is F / S in G94 (F = S x pitch, exactly what the stock floating-chuck cycle
expects) and F in G95; G93 and G96 are refused; P is ignored. Without the key (the default),
or on a stock LinuxCNC, G84 is the floating-chuck sequence as documented (feed down, M5, M4,
dwell, feed up, M5, M3). LinuxCNC's remapping cannot do this: only codes that do not exist
can be remapped, and G84 exists.

Other GUIs decide for themselves what they send at init and at quit: AXIS sends nothing at
either and needs no patch; another one would need its own.

On the motion side `kmotion-motion` reads the same key: the DISABLE that task sends as part
of its own start (before the session's first ENABLE) leaves the board's axes as they are;
every other DISABLE disables them at once. No timing is involved.

Apply, in the LinuxCNC run-in-place tree (the pinned commit of `linuxcnc/README.md`):

    cd ~/linuxcnc-dev
    git apply ~/KMotionX/KMotionX/linuxcnc/patches/*.patch
    cd src && make

(`milltask`, the interpreter library and its python module are rebuilt; `bin/gmoccapy` is regenerated from the patched source.)

The gmoccapy patch also makes `_periodic` reconcile, every poll, the E-stop and power buttons with the task state and the homed state (gmoccapy's own flag and buttons, and the Combi_DROs' homed colour) with the status: those follow the status watcher's edge signals, which a machine turned on or homed from outside before the GUI's first poll never produces (E-stop shown with the power on; red DROs on a homed machine).
