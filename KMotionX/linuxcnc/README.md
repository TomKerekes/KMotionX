# LinuxCNC with KMotion's planner and boards

Work in progress (started 2026-10-02): LinuxCNC's task and GUIs on one side, KMotion's
trajectory planner (TP3) and a KFLOP or Kogna on the other, with no real-time kernel on
the PC. LinuxCNC's motion module (`motmod`, a real-time HAL module) is replaced by a
plain userspace process, `kmotion-motion`, that speaks the same task↔motion protocol
through the same shared memory block.

## Layout

- `kmotion-motion/` – the process. `kmotion-motion.cc` (LinuxCNC side) decodes every emcmot
  command, acknowledges it, keeps the status block and the HAL pins a GUI expects up to date,
  handles jogging and homing. Derived from LinuxCNC's `motion-logger.c`, hence GPL-2.
  `kmotion-backend.cc` (KMotion side) hands the moves to KMotion's trajectory planner
  (`CCoordMotion`, TP3). The two include only their own project's headers and meet through
  `kmotion-backend.h`. `-m standin` keeps the earlier stand-in model (moves at their
  commanded speed, no planner) for comparison.
  - **Simulate mode** (the current one): the planner runs with `m_Simulate` + `m_DoTime`, so
    it plans and times every segment exactly as for a board but downloads nothing; the
    finalized segments are copied out of its buffer (the TPSegLog's data) and replayed over
    time. LinuxCNC therefore shows the motion the controller would execute: accelerations,
    jerk limits, corner blending, 3rd order knots. `make test` runs the planner on the test
    program's moves without LinuxCNC or a board (`backend-test.cc`).
  - Planner settings come from the ini: `[KMOTION]` (3rd order, cubic knots, actuator
    limits, segment log, break angle, tolerances, lookahead, board channels), the axis
    limits from `[AXIS_*] MAX_VELOCITY / MAX_ACCELERATION / MAX_JERK` (jerk defaults to
    10 × accel) and counts per unit from `[JOINT_n] INPUT_SCALE`.
- `configs/kmotion-sim/` – a LinuxCNC configuration that loads `kmotion-motion` instead of
  `motmod` (the sim/axis config, inches, 3 joints, trivkins), plus `drive.py`, which runs
  the whole thing from the command line.

## Build and run

Needs a LinuxCNC run-in-place build (`~/linuxcnc-dev` here; `LINUXCNC_DIR` in the makefile
picks another one). Its headers, `liblinuxcnchal` and `libnml` are used; three small
files of its tree (the motion error ring buffer helpers) are compiled in.

    cd kmotion-motion && make
    source ~/linuxcnc-dev/scripts/rip-environment
    linuxcnc -v -d ~/KMotionX/KMotionX/linuxcnc/configs/kmotion-sim/kmotion-sim.ini
    python3 ~/KMotionX/KMotionX/linuxcnc/configs/kmotion-sim/drive.py   # in another terminal

`-v -d` keeps LinuxCNC's output on the console instead of its error dialog. The command
stream goes to `/tmp/kmotion-motion.log` (the `-l` option in the HAL file).

## Notes on the KMotion side

- `CCoordMotion` in simulate mode still asks the board for the coordinate system and the
  positions (`DefineCS`, `Dest%d`), and `FlushSegments` opens the board's buffer; the backend
  therefore sets the axis map and the current position itself and composes its flush from
  the public pieces (`TP3FlushRun`, `MaximizeSegments`, `OutputSegment`, then the reset).
- The planner finalizes the last moves only when told that no more are coming, so the
  backend flushes when task has been quiet for 50 ms (program end, M0, tool change, dwell).
- One LinuxCNC move becomes many planned segments (knots every 10 ms with TP3); a move
  counts as finished when a segment of a later move starts. The planner's sequence numbers
  are our own serials, mapped back to LinuxCNC's line ids.

## Next

1. The Kogna: simulate off, axis definitions and the init program, positions from the
   board, feed override through the board, Halt/Resume as kmxWeb does it, homing on the
   board side.
2. Kinematics on the KMotion side (TP3 plans in actuator space); LinuxCNC's kinematics
   module is bypassed and only gets joint positions to display.
3. mm configs (LinuxCNC machine units vs KMotion's inches).
