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
    so the protocol loop only queues moves and copies the published state. Feed hold,
    resume, the stop of an abort and jogs go to the board through a small command thread
    of their own, in order, so the protocol loop never waits on the board link either.
    A status poller thread reads the board every `STATUS_PERIOD_MS`.
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
    hold); abort = what KMotion's own Halt does (`CheckMotionHalt`): `StopImmediate0` at
    once, `GetStopState` until the board is at rest, then `StopImmediate2`, which clears the
    stop state and abandons the rest of the buffer, and a planner reset - the state reads
    busy until that is done (`StopImmediate2` alone does nothing to a moving board); feed
    override = the board's FRO; jogs = `Jog<ch>=counts/s` and `Move<ch>=counts`. Homing is
    still immediate (board-side homing comes later).
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

`make test` runs the harness (`backend-test.cc`) in simulate mode: the test program's moves,
arc directions/planes/multiple turns as LinuxCNC encodes them, and an abort in mid-program
(`./build/backend-test circles|arcspeed|quit|letters|gated` are five more; `KM_KOGNA=1` selects
the Kogna settings, `KM_CORNER_TOL`, `KM_COLLINEAR_TOL`, `KM_JERK`, `KM_PATH_TOL` vary the
planner settings). Each check prints ok or FAIL. The planner writes `/tmp/TPSegLog.csv` and
`/tmp/TP3_Timeline.log` during every run with logging on; both are overwritten by the
next run, LinuxCNC's included.
With `KM_BOARD=1` the harness runs against the Kogna with the kmotion-kogna settings
(`abort` then measures the real stop and the move after it; `KM_DEBUG_STOP=1` prints the
timeline). Never do that while a LinuxCNC or kmxWeb is connected to the board: the init
program zeroes the positions and the test moves the axes.

## Notes on the KMotion side

- `CCoordMotion` in simulate mode still asks the board for the coordinate system and the
  positions (`DefineCS`, `Dest%d`), and `FlushSegments` opens the board's buffer; the backend
  therefore sets the axis map and the current position itself and composes its flush from
  the public pieces (`TP3FlushRun`, `MaximizeSegments`, `OutputSegment`, then the reset).
- The planner finalizes the last moves only when told that no more are coming, so the
  backend flushes when task has been quiet for 50 ms (program end, M0, tool change, dwell).
  Not, however, while task's queue is full and motion is under way: task then hands over
  one move each time one finishes, and a flush in those gaps ended the run, with a stop,
  after every move (seen as arcs stopping at every segment; 38 of 42 run ends in one run of
  axis.ngc). The queue limit is 500 moves (motmod takes ~2000 segments; the planner wants
  seconds of lookahead), a freed slot gets 50 ms for task to refill it, and a flush with
  nothing executing is always allowed, since it is the only way to get going again.
- Between waypoints the worker calls `DownloadDoneSegments` itself: the planner downloads
  its finalized segments in bounded chunks only every 8th waypoint, too rarely when moves
  trickle in one at a time (the simulate replay ran dry with 19 moves pending; on the board
  the buffer would starve). `backend-test gated` hands 36 chords over the way task does.
- One LinuxCNC move becomes many planned segments (knots every 10 ms with TP3); a move
  counts as finished when a segment of a later move starts. The planner's sequence numbers
  are our own serials, mapped back to LinuxCNC's line ids.

- `GetRapidSettings` reads the rapid limits from the board even in simulate mode (a 100 ms
  token wait), which launches KMotionServer; `RapidParamsDirty = false` after setting them
  avoids that. `SetConsoleCallback` registers with the server, so it is board mode only.
- `UpdateRealTimeState` is private; `realtime_sequence()` is the same walk over the
  executing buffer from the planner's public globals.
- Arcs: LinuxCNC sends center, normal and `turn`. The normal is the plane's axis and is never
  flipped; `turn >= 0` is counterclockwise with `turn` extra full turns, `turn < 0` clockwise
  with `-1 - turn` extra turns (posemath's `pmCircleInit`). KMotion's `ArcFeed` takes a plain
  CCW flag in its `rotation` argument (its own canon passes `rotation == 1`) and knows no
  multiple turns, so extra turns go in as half circles ahead of the final arc, each with its
  own sequence number. Its `ID` argument is the cutter-compensation hint (1/2/3), not a line
  number: 0 here.
- After an arc the planner's `current_*` lags one chord behind (TP3's collinear stager holds
  the last chord until the next move or a flush), so the backend keeps the last commanded
  end point (`cmd_end`) itself for geometry that needs the start point.
- A planner failure (a refused move or flush) stops the run like an abort, reports the
  message and raises LinuxCNC's motion error flag so task aborts the program, as motmod does
  when `tpAddLine` fails. The planner also returns failures while it is being aborted (that is
  how the worker leaves a blocked download); those are not reported.
- Pause/resume are the board's feed hold (`StopImmediate0/1`): the motion resumes exactly
  where it stopped, mid-arc included. Stop (an abort) drops the program, as in LinuxCNC
  generally; the GUI's "Run from here" starts afresh at a line. Single step (`EMCMOT_STEP`,
  sent by task only while paused) works as in motmod: resume until the executing line
  changes, then hold again - with the board the hold starts a little way into the next line.
- Measured on the Kogna (`KM_BOARD=1 ./build/backend-test abort`, `KM_DEBUG_STOP=1` for the
  timeline): a stop during a feed move has the feed hold on the wire within a few ms, the
  board at rest 50-60 ms later, and the commanded position does not move afterwards. The
  planner's current position is then set to where the board stopped: it had stayed at the
  end of the last move handed over, up to the lookahead ahead, and the next move began with
  the board leaping there (0.35 in in the test, the "small jump" seen in AXIS). The stop
  state is read from the status poll (`MAIN_STATUS.StopImmediateState`), not by polling
  `GetStopState`, which starved the status thread and froze the display during the stop.
- The Kogna's Ethernet link stalled for 0.9-1.5 s twice when status was polled every 5 ms
  (three queries per poll); everything waited, the stop included, and the board ran on
  meanwhile. 20 ms polling has shown no stall. Keep the polling moderate.
- At exit (LinuxCNC's shutdown sends SIGTERM) the process aborts a run in progress, waits
  for the board to stop, and takes the backend down before leaving; a watchdog ends the
  process after 3 s if that hangs. Without this the process once outlived LinuxCNC for good
  (its threads ran into the library's static destructors) and the next start would have
  failed. `backend-test quit` covers the paused-mid-run case.
- `SET_TERM_COND` (G61, G61.1, G64 P) goes to the planner's `SetPathMode`: blend mode with
  the programmed tolerance (0 = the ini's `CORNER_TOL`), exact stop at block ends otherwise.
- Corner tolerance matters a lot to TP3. At a corner it either blends within the tolerance
  at a kink speed cap of 8·(tol/2)/(|Δdir|·Tw), Tw = 3·accel/jerk being its filter window,
  or stops; with `CORNER_TOL = 0.0005` and jerk 984 that cap was 0.0067 in/s, and around
  every rapid-to-feed corner the plan crawled at that speed for about 1.5 s on each side -
  seen in AXIS as a pause before the plunge, a tool that creeps up to its target, a DRO that
  keeps counting after the tool seems to have stopped. `backend-test letters` (the first
  moves of axis.ngc on the Kogna settings) measures it: 31.1 s with 9.9 s of crawl at
  0.0005, 26.9 s with none at the 0.003 KMotionCNC uses for this machine; the configs now
  carry 0.003 / 0.0005 (corner / collinear), Tom's KMotionXCNC values. Speed changes still
  take about Tw (0.3 s) each with jerk 984; ten times the jerk brings the same moves to 21 s.
- Observation, not investigated: with TP3 the planner caps the speed on arcs at roughly one
  radian per second of arc angle whatever F says (r = 1 in runs at ~1 in/s, r = 0.5 at
  ~0.42 in/s; `backend-test arcspeed`). It looks like `FACET_ANGLE` (0.5°) per 10 ms knot.

## Next

1. Retest on the Kogna: Stop mid-arc (any jump afterwards?), Pause/Resume mid-arc, Step
   while paused; then board-side homing, in-position from the board's `AxisDone`, jogging
   while paused.
2. Kinematics on the KMotion side (TP3 plans in actuator space); LinuxCNC's kinematics
   module is bypassed and only gets joint positions to display.
3. mm configs (LinuxCNC machine units vs KMotion's inches).
