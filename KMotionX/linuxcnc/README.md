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
  - The KMotion side runs in its own thread: CoordMotion's calls block while the board's
    buffer is full (its flow control) and task times a motion command out after a second,
    so the protocol loop only queues moves and copies the published state. Feedhold,
    resume, abort and jogs go to the board directly (KMotion's pipe serializes the threads).
  - **Simulate mode** (`[KMOTION] MODE = simulate`): the planner runs with `m_Simulate` +
    `m_DoTime`, so it plans and times every segment exactly as for a board but downloads
    nothing; the finalized segments are copied out of its buffer (the TPSegLog's data) and
    replayed over time. LinuxCNC therefore shows the motion the controller would execute:
    accelerations, jerk limits, corner blending, 3rd order knots. `make test` runs the
    planner on the test program's moves without LinuxCNC or a board (`backend-test.cc`).
  - **Board mode** (`MODE = board`): the same planner downloads to the KFLOP/Kogna through
    KMotionServer as KMotionCNC and kmxWeb do. At start it checks the firmware, runs the
    `INIT_PROGRAM` (axis setup, e.g. the Minirouter init) and waits for it, sets the
    coordinate system (`DefineCSEx`) and reads the position. Then: positions and enables
    from `GetStatus` every `STATUS_PERIOD_MS`, the executing line from `ExecTime` against
    the downloaded segments, the end of a run from `CheckDoneBuf`; machine on/off =
    `EnableAxisDest`/`DisableAxis`; pause/resume = `StopImmediate0/1` (the board's feed
    hold); abort = `StopImmediate2` plus a planner reset; feed override = the board's FRO;
    jogs = `Jog<ch>=counts/s` and `Move<ch>=counts`. Homing is still immediate (board-side
    homing comes later).
  - Planner settings come from the ini: `[KMOTION]` (mode, init program, 3rd order, cubic
    knots, actuator limits, segment log, break angle, tolerances, lookahead, board
    channels), the axis limits from `[AXIS_*] MAX_VELOCITY / MAX_ACCELERATION / MAX_JERK`
    (jerk defaults to 10 × accel) and counts per unit from `[JOINT_n] INPUT_SCALE`.
- `configs/kmotion-sim/` – a LinuxCNC configuration that loads `kmotion-motion` instead of
  `motmod` in simulate mode (the sim/axis config, inches, 3 joints, trivkins), plus
  `drive.py`, which runs the whole thing from the command line.
- `configs/kmotion-kogna/` – the same for Tom's Kogna: board mode, 2540 counts/inch,
  250 mm/s, 2500 mm/s², jerk 25000 mm/s³ in inches, `MinirouterInit.c` as the init program.

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

- `GetRapidSettings` reads the rapid limits from the board even in simulate mode (a 100 ms
  token wait), which launches KMotionServer; `RapidParamsDirty = false` after setting them
  avoids that. `SetConsoleCallback` registers with the server, so it is board mode only.
- `UpdateRealTimeState` is private; `realtime_sequence()` is the same walk over the
  executing buffer from the planner's public globals.

## Next

1. Run the Kogna config; then board-side homing, in-position from the board's `AxisDone`,
   jogging while paused.
2. Kinematics on the KMotion side (TP3 plans in actuator space); LinuxCNC's kinematics
   module is bypassed and only gets joint positions to display.
3. mm configs (LinuxCNC machine units vs KMotion's inches).
