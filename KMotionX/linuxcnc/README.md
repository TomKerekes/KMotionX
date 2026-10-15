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
    override = the board's FRO; jogs = `Jog<ch>=counts/s`, and an increment
    `MoveAtVel<ch>=counts counts/s` at LinuxCNC's jog speed. For LinuxCNC a jog runs (in
    position off, jogging on) until a status read made after its last command reached the
    board shows the axis done (`AxisDone`) or disabled: releasing a jog button stops the
    board, and the slow-down to rest is part of the jog.
  - **Homing** (board mode) runs a C program on the board, `[KMOTION] HOME_PROGRAM`
    (`configs/kmotion-kogna/Home.c`, after Dynomotion's `SimpleHomeIndexFunction.c`: jog to
    the switch, back off, latch slowly, optionally on to the index, set the position, move to
    HOME). LinuxCNC's `[JOINT_n]` homing parameters, as task sends them (`HOME_SEARCH_VEL`
    with its sign as the direction, `HOME_LATCH_VEL`, `HOME_OFFSET`, `HOME`, `HOME_FINAL_VEL`,
    `HOME_USE_INDEX`, `HOME_SEQUENCE`, and `MAX_LIMIT - MIN_LIMIT` as the search travel), go to
    the program in counts through the board's persist variables 120-191 (layout at the top of
    Home.c); the program runs on `HOME_THREAD` and is polled with `CheckThread`; when it ends
    the planner is set to where the board stands, the joints it reports in persist 121 become
    `homed`, the others get an error. Which input bit each switch is on, and the index bit,
    are the machine's business and sit in a table at the top of Home.c. Stop or machine-off
    during homing kills the thread and stops the jogs. A joint with `HOME_SEARCH_VEL = 0`, or
    no `HOME_PROGRAM`, is declared homed where it stands.
  - **Limit switches** are per joint, as LinuxCNC's own: `[KMOTION] LIMIT_NEG_BITS` and
    `LIMIT_POS_BITS` name each joint's switch bit (in `CHANNELS` order, -1 none; a bit the
    status carries) and `LIMIT_ACTIVE` the level at which a switch is made; the HAL pins
    `joint.N.neg-lim-sw-in` / `pos-lim-sw-in` count as well (the only source in simulate
    mode). Handled as motmod's `check_for_faults()` does: a switch made and not overridden
    sets the joint's error flag, says "joint N on limit switch error" once and takes the
    machine off (the joint stays homed); a joint being homed is exempt. The GUI's "ignore
    limits" (`OVERRIDE_LIMITS`) lets the next joint jog run - off the switch, not further
    onto it ("Can't jog joint N further past max hard limit") - and the limits count again
    when that jog ends; no program move while a switch is made. A world jog under identity
    kinematics (no joint mode in the GUI) follows the same rule for its own joint; under a
    KINEMATICS it is refused while a switch is made, as it moves every joint. A robot's
    switches sit on its actuators, so there is no CAD-space hard limit. The bench's
    `C Programs/SimulateHomes.c` makes the switches on bits 49-54 from the destinations
    (`backend-test limits`).
  - **KMotionCNC's buttons** as HAL pins, for the gladevcp panel `panels/kmotion-panel.ui`
    (embedded in gmoccapy's bottom row by `[DISPLAY] EMBED_TAB_*`, netted by
    `panels/kmotion-panel.hal`, which gladevcp runs itself with `-H` once its pins exist): `kmotion.hold-reverse` / `hold-forward` run the paused program
    backwards or forwards while held, at `kmotion.hold-rate` (0: 0.2) of the programmed
    speed - the board's temporary override within the hold (`SetFROTemp`), as KMotionCNC's
    feed hold buttons; released, or at the resume, it goes back to 0 (board mode only).
    `kmotion.measure` appends the current position to `[KMOTION] MEASURE_FILE` as the next
    grid point of a geo correction table (KMotionCNC's Measure: the header - rows,cols,
    spacing, offset - by hand, then one press per grid point, columns first; the next point
    is read from the file, so it survives a restart); `kmotion.geo-reload` reads `GEO_FILE`
    again. Each press answers with a message. The panel's handler (`kmotion-panel.py`)
    greys Back/Forward unless `kmotion.hold-allowed` says a hold jog is possible, hides Geo
    reload without a `GEO_FILE`, and shows Measure only with `[KMOTION] MEASURE_BUTTON = 1`.
  - **Rigid tapping** (G33.1, board mode): the planner's move from the start through the
    bottom, and on by `[KMOTION] TAP_OVERSHOOT_REVS` revolutions for the spindle's
    deceleration, at pitch x spindle speed, launched in the board's spindle-sync mode
    (`TrigThread`, KMotionCNC's threading mode): the board slaves the coordinated motion's
    time base to the spindle encoder, so every actuator follows through any kinematics and
    the spindle's reversal runs the path backwards. `TAP_PROGRAM` (`configs/kmotion-kogna/
    Tap.c`, on `TAP_THREAD`) drives the spindle: it waits for the launch, lets the spindle
    turn the revolutions for the depth, reverses it at G33.1's I times the speed, stops it
    once the encoder is back at the start and reports through persist 192-196; the module
    then clears the motion, sets the planner to where the board stands and restarts the
    spindle as LinuxCNC has it. The program holds what the machine is: the encoder axis,
    counts per revolution from the ini (`SPINDLE_SPEED_AXIS`, `SPINDLE_COUNTS_PER_REV`, persist
    197-198), how the spindle runs. While the tap runs the spindle's status shows the
    direction it really turns, as motmod's does. The interpreter wants the spindle turning
    (M3 S) before a G33.1; an aborted tap stops the spindle, a pause is refused (the board's
    feed hold stops the motion whatever the spindle does), the feed override is 1 for the
    tap. The firmware starts the sync when the spindle reaches a start position two
    revolutions past the trigger, and the path's end ends it, so `TAP_OVERSHOOT_REVS` must
    cover the spindle's deceleration - and no more: the allowance is `TAP_OVERSHOOT_REVS` x
    pitch of path beyond the bottom, and at a coarse pitch or a slow spindle a smaller value
    does (the spindle's deceleration takes fewer revolutions). The tap's whole path is on
    the board before the launch: a run is normally streamed, `LOOKAHEAD` seconds ahead of the
    board's executed time, and in spindle sync that time runs with the spindle (a tap longer
    than the lookahead was fed as the spindle went, the motion stopping between chunks, and
    hung once the spindle stood at the top); for a tap the lookahead is unbounded, the
    download takes a fraction of a second, and should it still wait when the program has
    ended, the poller cuts it. In simulate mode the tap is replayed as a move down
    and back. **G84 and G74** (the tapping cycles, right- and left-hand) are rigid taps too
    with `[RS274NGC] RIGID_TAP_CYCLES = 1` and the interpreter patch
    `patches/interp-rigid-tap-cycles.patch`: the cycle's rapid to the retract plane, then the
    tap to Z at the pitch F / S (G94: F = S x pitch, as the stock cycle wants it) or F in G95,
    back at the retract plane (G98 then returns to the initial level); L repeats and the next
    holes' sticky words as in any canned cycle; P (the dwell at the bottom) is ignored; G93
    and G96 are refused. Stock LinuxCNC's G84 is a floating-chuck cycle (feed down, M5, M4,
    dwell, feed up, M5, M3), which it stays without the key. Because the cycle's rapid and
    the tap come in one block, the module holds a tap until the moves before it have
    finished (the tap starts from where the axes stand), and an abort meanwhile drops it.
  - **Threading** (G33, and G76's passes; board mode): a feed under spindle sync is planned
    at pitch x the spindle's rev/s, consecutive synchronized lines in one run (a G76 pass's
    entry, thread and exit), launched together in the board's spindle-sync mode like the
    tap: the board slaves the run's time base to the spindle encoder and starts it when the
    spindle crosses a whole revolution, so every pass starts at the same spindle angle and
    the passes line up. The run begins with the first synchronized line: moves the planner
    still holds before it (G76's rapids to the pass depth) are launched as a normal run
    first, so the wait for the spindle is at the thread's start, where LinuxCNC's is.
    The first unsynchronized move after the synchronized lines - G76's
    pull-out to the drive line, a rapid right behind the cut in LinuxCNC - joins the run
    (under the spindle's time base, real time at the base speed) and ends it, so the tool
    never stands at the thread's end with the spindle turning; any other command, or the
    quiet flush, launches the run as it is; the whole path is on the board before the launch, the feed override is 1 for the
    run (as motmod ignores it on synchronized moves) and the operator's again after. The
    interpreter wants the spindle turning (M3 S) before a G33. Needs `SPINDLE_SPEED_AXIS`
    and the encoder as the tap does; in simulate mode the run is replayed like any other.
  - **Spindle** (board mode) as KMotionCNC drives one (Dynomotion's help page "KMotionCNC
    Spindle Control"): `[KMOTION] SPINDLE_M3`, `SPINDLE_M4`, `SPINDLE_M5` and `SPINDLE_S` are
    each one of KMotion's M-code actions, carried out as the G-code interpreter's
    `InvokeActionDirect` does: `SETBIT`/`SETTWOBITS` (relay bits), `DAC` (S x scale + offset,
    clamped), or `PROGRAM`/`PROGRAM_WAIT`/`PROGRAM_WAIT_SYNC <thread> <var> <file>` (a C program
    gets its persist variable, S the RPM as a float and M3/M4/M5 their number, is compiled and
    loaded into the thread and executed). LinuxCNC's spindle on runs S then M3/M4, a speed or
    override change S, spindle off M5; the speed is motmod's (override, `[SPINDLE_0]` limits),
    and "M4 S0" keeps its direction. Motion reads busy while a spindle command is under way,
    so task's next command waits for a "wait" program as KMotionCNC's interpreter does. A
    stream of override steps collapses to the latest; a start, stop or reversal never does.
    Machine off and kmotion-motion's exit stop the spindle too, and the first stop after
    start-up always runs M5.
    `INIT_PROGRAM` may be given more than once (run in order); program paths are absolute or
    relative to the ini. `START_PROGRAM = <thread> <file>` (any number) starts a program after
    the init programs and leaves it running: forever loops on the board. Every running thread
    slows down thread 0, so the continuous work belongs in one loop in thread 1 (free again
    once the init programs are done), each part `#include`d with a service call that does its
    work only when enabled, as Dynomotion's examples do: the bench's `BenchLoop.c`.
  - **G96 constant surface speed** (`[KMOTION] SPINDLE_CSS = 1`; without it G96 is refused with
    an error) as KMotionCNC does it, on the board: LinuxCNC hands motion a CSS factor, X's
    offset and the D maximum with every spindle on (motmod would recompute the RPM from X each
    cycle; here a speed change runs a C program, far too slow for that). kmotion-motion turns
    them into what KMotionCNC's `SetCSS()` writes: the X counts of radius 0, inches per count,
    the surface speed in in/s (override included) and the maximum RPM (floats in persist
    111-114), then the mode in 110 (2 for G96, 1 for G97); the S action gets the surface speed
    too. A `ServiceCSS()` loop on the board (Dynomotion's `C Programs/SpindleUsingJogs/CSS/
    CSSJog.c`) then sets the spindle's speed from X every 50 ms; Dynomotion's CSS spindle
    programs leave the speed to it in mode 2. LinuxCNC's own display (`spindle.0.speed-out`)
    follows X as motmod's does. No D: KMotionCNC's 10^9 RPM, within the `[SPINDLE_0]` limit.
  - The kmotion-kogna bench uses Dynomotion's Spindle Using Jogs programs, the CSS set
    (`configs/kmotion-kogna/spindle/`, `MySpindleDefs.h` set for channel 3, which
    `SpindleAxis.c` sets up), with `ServiceCSS()` (`CSSJog.c`) called from `BenchLoop.c`, the
    bench's forever loop in thread 1. One fix in the copies: `OnCWJog.c` and `OnCCWJog.c`
    clear the spindle state
    (`STATEVAR`) before a reversal's spin-down, and `OffJog.c` before its stop; otherwise, in
    G96, `ServiceCSS()` jogs the spindle back up while they wait for it to stop, and an M3 to M4
    reversal never finishes. `KM_BOARD=1 ./build/backend-test spindle` checks 600/1200 RPM CW,
    600 RPM CCW and off on channel 3 (10000 counts/s at 600 RPM) and the direction bits, then
    G96 S100 at a radius of 1 in (191 RPM), X moved out to 2 in with nothing sent (95.5 RPM),
    D60, M4 in G96, G97 S600 and M5.
  - **Spindle speed** measured on the board, for the GUI: the status poller takes
    `[KMOTION] SPINDLE_SPEED_AXIS`'s channel, its commanded `Dest` (a jogged spindle's real
    ramps, no encoder needed) or its encoder `Position` (`SPINDLE_SPEED_FROM = DEST|POSITION`),
    over the board's own clock, in `SPINDLE_COUNTS_PER_REV`, low-pass filtered with
    `SPINDLE_SPEED_TAU` seconds, onto the HAL pins `kmotion.spindle-rpm` (signed),
    `kmotion.spindle-rpm-abs` and `kmotion.spindle-rps`. The kmotion-kogna config runs
    gmoccapy, whose spindle bar shows it (`gmoccapy-postgui.hal`); a spin-down shows as it
    happens, where the commanded speed drops to 0 at once. On the bench, `SpindleAxis.c` also
    feeds channel 3's step/dir output back into encoder 4 as quadrature, so `POSITION` works too
    (`KM_SPINDLE_FROM=POSITION` in the harness).
  - **Spindle at speed** (`[KMOTION] SPINDLE_AT_SPEED`): what the board says, either an input
    bit at a level (`BIT <bit> [<level>]`, e.g. the spindle drive's at-speed output) or a jogged
    spindle axis at its commanded speed (`AXIS [<channel>]`): the channel's Dest speed between
    two status reads, on the board's own clock (`TimeStamp`), within 1% (at least 1 RPM) of S -
    in G96 of what `ServiceCSS()` makes of X - with `SPINDLE_COUNTS_PER_REV`, as Dynomotion's
    `OnCWJogWait.c` waits on the channel's velocity. (A jog's Done can't tell: the firmware ends
    a jog in an open-ended constant-velocity segment, so Done stays false until it stops.)
    `KM_BOARD=1 ./build/backend-test spindle` checks it after each ramp, `atbit` the bit (a
    virtual bit it sets and clears). Motion uses it as motmod uses `spindle.N.at-speed`
    (which still counts too): after an M3, M4 or M5 that asks for it, and in G96 after each
    rapid, the next feed move and everything after it wait until the spindle is at speed. A
    reading only counts from a status read made after the spindle command was done, and three
    in a row with the board idle let the moves go, so a read from before the spin-up (or before
    `ServiceCSS()`'s next update) cannot. `kmotion.spindle-at-speed` shows it (the kmotion-kogna
    config lights gmoccapy's at-speed LED with it); the bench uses `AXIS 3`.
    `SPINDLE_AT_SPEED_TOLERANCE` (default 0.01) is the fraction of the commanded speed within
    which `AXIS` counts as at speed; it leaves again at twice that, so the LED does not
    flicker with the read timing.
  - **Board I/O bits as HAL pins**: `[KMOTION] OUTPUT_BITS = <bit> ...` makes a pin
    `kmotion.out.<bit>` (HAL in) per bit, which the board bit follows (`SetStateBit`, set once
    at start, then on every change); `INPUT_BITS` makes `kmotion.in.<bit>` (HAL out) from the
    status read every `STATUS_PERIOD_MS`. Any HAL signal can then drive a board output: the
    kmotion-kogna config nets `iocontrol.0.coolant-mist` (M7) and `-flood` (M8) to virtual bits
    1026 and 1029 (M9 clears both). LinuxCNC's digital outputs `motion.digital-out-NN`
    (`[KMOTION] NUM_DIO`, default 4) follow M64/M65 P<n> at once; the config nets
    `motion.digital-out-00` to virtual bit 1028. M62/M63, which switch in step with the next
    move, are refused for now (the moves are on the board by then).
  - **User M codes M100-M199** as KMotionCNC's M-code actions: task runs the executable `M1xx`
    it finds in `[RS274NGC] USER_M_PATH` as `M1xx <P> <Q>` (-1 for a word not given) and waits
    for it. The config's `M1xx` is a two-line wrapper around `kmotion-mcode`, which sends the
    code, P and Q to kmotion-motion on a Unix socket (abstract name `kmotion-motion`);
    kmotion-motion carries out `[KMOTION] MCODE_<n>` (the spindle actions' syntax; P and Q go
    to `<var>` and `<var>+1` as floats) through the backend, in order with its other board
    work, and answers when it is over; an error stops the program. The kmotion-kogna bench has
    `mcodes/M100` and `mcodes/M100.c` (thread 5, P and Q in persist 10 and 11: virtual bit 1027
    on when P is not 0). `KM_BOARD=1 ./build/backend-test io` checks the bits and M100.
  - **Probing, G38.2-G38.5** (`[KMOTION] PROBE_BIT`, `PROBE_ACTIVE`): the planner runs the
    probe move like any move, so any kinematics apply (3Link included), while the probe
    watcher, `ServiceProbe()` (`configs/kmotion-kogna/probe/ProbeService.c`, after Dynomotion's
    `NotifyProbeMach3.c`), called from the board's forever loop (the bench's `BenchLoop.c`),
    watches the probe bit. kmotion-motion arms it for each probe move (persist 69; the layout
    is in ProbeService.c). When the probe makes contact (G38.2/.3) or loses it (G38.4/.5) it
    records every coordinate-system axis's Dest, calls `StopCoordinatedMotion()` and disarms;
    the backend finishes the stop as an abort's (at rest, the rest of the buffer abandoned,
    the planner where the board stands) and turns the recorded actuator positions into the
    probed position through the kinematics.
    kmotion-motion then answers as motmod does: `probeTripped`, `probedPos` (#5061-), the errors
    for G38.2/.4 ("finished without making contact", "already tripped"; G38.3/.5 none), and
    `probeVal` from the bit. A move that ends without a trip disarms the watcher; one that
    the watcher does not answer within 2 s (no loop running it) is an error. Not done:
    motmod's "Probe tripped during non-probe move". `KM_BOARD=1 ./build/backend-test probe`
    runs BenchLoop.c and sets and clears virtual bit 1031 (contact = 1) during real moves,
    one longer than the planner's lookahead: a trip while a move is still being downloaded
    freezes the planner's download pacing (it waits for the held board), so the status
    poller watches for the trip until the move is all downloaded and aborts the planner's
    wait. (The watcher uses if/else where `?:` would do: TCC67 before dda21c8 got a `?:`
    whose result is a double wrong.)
  - **Kinematics on the KMotion side** (`[KMOTION] KINEMATICS = <name>`, the names KMotionCNC's
    `Data/Kinematics.txt` takes: `3Link`, `Scara`, `Geppetto`, `Kinematics3Rod`, the 5-axis
    ones): the planner turns the CAD position into the actuators' positions, as KMotionCNC
    does, and LinuxCNC's joints are those actuators, in their own units (degrees for a rotary
    one), not the axes. kmotion-motion gives LinuxCNC kinematics type BOTH, so its GUI tells
    joint mode from world mode: a joint jog moves one actuator on the board (within the
    joint's `[JOINT_n]` limits - with or without a KINEMATICS, a continuous jog now runs to the
    limit, as motmod's do), a world jog is a planner move toward the axis's `[AXIS_*]` limit (or
    the increment), stopped at the button's release the way an abort stops a move. It ends
    where a joint would reach its `[JOINT_n]` limit heading out (motmod stops its teleop jogs
    there too): the end point is cut on the jog's path through the kinematics when the jog
    starts (`kmotion-joglimit.h`), so the board stops there by itself whatever the PC does,
    with one message on arrival. A joint already past its limit can be jogged back; a jog
    whose first step takes it further out is refused, with one message; the joint
    positions shown are the actuators' (from the status, or where the position puts them in
    simulate mode); homing is per joint as before (absolute actuators: `HOME_SEARCH_VEL = 0`,
    homed where they stand, nothing moves). `[KMOTION] ACTUATOR_LIMITS = 1` then takes the
    planner's limits from `[JOINT_n] MAX_VELOCITY / MAX_ACCELERATION / MAX_JERK` in the joint's
    units (`TYPE = ANGULAR` marks a rotary one), `[JOINT_n] INPUT_SCALE` being its counts per
    unit, while the feed rate and tolerances stay in CAD units. Without a KINEMATICS nothing
    changes: joint i is axis i. Not done: LinuxCNC's own soft-limit checks of the joints along
    a programmed path (the interpreter checks the `[AXIS_*]` limits, the board its own).
  - **An enable program** (`[KMOTION] ENABLE_PROGRAM = <thread> <file>`): machine on runs it in
    its thread in place of enabling the axes, and waits until every axis of the coordinate
    system is enabled (`ENABLE_TIMEOUT_S`, default 15; past it an error, the program's own
    messages are on the board's console); then the planner takes the position from the board.
    For a setup that must come first, e.g. serial servos whose torque the program turns on
    before enabling at their measured positions. Machine off disables the axes as before. Its
    thread is reserved like a START_PROGRAM's.
  - **Across sessions** (`[EMC] KEEP_ENABLED_ON_EXIT = 1`, with the two patches to LinuxCNC in
    `patches/`): the board's axes stay enabled when LinuxCNC exits, as KMotionCNC leaves them,
    and a start that finds them all enabled (HAL pin `kmotion.all-enabled`) can come up with
    Machine on and no init - `configs/kmotion-3link/autostart.py` does that. The DISABLE task
    sends as part of its own start leaves the board as it is; every other DISABLE (Machine off,
    E-stop, and with a stock LinuxCNC the GUI's quit and task's shutdown) disables the board
    at once. `[KMOTION] ALWAYS_HOMED = 1`: every joint is homed whenever the machine is on
    (absolute feedback), no homing step. `[KMOTION] HOMED_PERSIST = <n>`: the joints homed are
    remembered in persist variable n (bit = joint) across sessions, read by a start with every
    axis enabled (HAL pin `kmotion.homed-mask`; such a joint's next homing request is answered
    where it stands, once), cleared by a start with an axis disabled and by the board's power;
    a board program may set or clear it. A board that stops answering takes the machine to
    off and unhomed.
  - **Geometric correction** (`[KMOTION] GEO_FILE = <file>`, relative to the ini): KMotionCNC's
    Geo Correction File (Tool Setup), read by the kinematics at start as KMotionCNC reads it
    (`ReadGeoTable`: the XY table and any per-axis linear tables named after it); a file the
    kinematics cannot read stops the start. The board's `PC_COMM_RELOAD_GEOCORRECTION` reads
    it again.
  - **The 3 Link robot:** `configs/kmotion-3link/` (its README says what is still to be set):
    three Dynamixel XL430 servos on the Kogna's serial servo bus, `KINEMATICS = 3Link`, joints
    in degrees at 11.378 counts/degree, `ENABLE_PROGRAM = 1 DxlAxisInit3.c` (Tom's servo setup
    and watchdog), the demo `ngc/DynoMotion3Link.ngc`. `backend-test kins3link` runs a square
    through the 3Link kinematics in simulate mode; the config ran headless (LinuxCNC's `dummy`
    display, a python `linuxcnc.command` script) in simulate mode and on the bare Kogna with
    the bench's step/dir channels in place of the servos: homing, world and joint jogs in both
    modes, inch and mm MDI moves, the demo program.
  - **The board's commands to the PC** (board mode, `[KMOTION] PC_COMM = 1`, the default;
    `kmotion-pccomm.cc`): KMotionCNC's PC_COMM mechanism, by which a C program on the board
    drives the application - it writes a command code into persist 100, arguments into
    101-107 and strings into the gather buffer (`DSP_KFLOP/PC-DSP.h`, the helpers in
    `C Programs/KflopToKMotionCNCFunctions.c`: `MDI()`, `MsgBox()`, `GetDROs()`,
    `SetToolLength()`, `DoPCFloat(PC_COMM_SET_FRO, ...)` ...); the status upload carries
    persist 100-107, and the result goes back into persist 100 (0 done, negative failed).
    Nearly every command is a task-level operation in LinuxCNC, so the dispatcher talks to
    task the way a GUI does, through its NML channels (`emcCommand`/`emcStatus`, process
    `xemc`, what the `linuxcnc` Python module uses). Supported: `ESTOP`, `HALT`, `RESTART`
    (an abort; the next run starts at line 1 anyway), `EXECUTE` (resume if paused, else run the
    loaded file from the start - KMotionCNC continues a halted job, LinuxCNC does not),
    `SINGLE_STEP`, `HALT_NEXT_LINE` (a pause, the nearest thing), `SET_FRO/RRO/SSO` and their
    `_INC` forms (`EMC_TRAJ_SET_*SCALE`, clamped to `[DISPLAY] MAX_FEED_OVERRIDE` etc.; the
    GUI's slider follows and the board's FRO gets it through the usual path), `SET_X`..`SET_V`
    (AXIS's touch-off, `G10 L20 P0`), `MDI` (needs task idle, as KMotionCNC's; the mode goes
    back to what it was afterwards), `MCODE` (`M<n>`), `USER_BUTTON` (`[KMOTION]
    USER_BUTTON_<n> = <MDI line>`), `MSG` (through motion's error ring to the GUI's
    notification; no GUI can press a button: an `MB_OK` box is answered IDOK, any other
    IDCANCEL with the command failing, so a program that branches on the answer can tell),
    `GET_DROS`, `GET_MACHINE_COORDS`, `GET_MISC_SETTINGS` (units; T, and the tool in the
    spindle as H and D while G43 / G41-42 are active - LinuxCNC has no separate numbers),
    `GET_TOOL_SLOT_ID`, `GETAXISRES`, `GET_TP_PARAM`, `GET/SET_TOOLTABLE_LENGTH/DIAMETER/
    OFFSETX/OFFSETY` (reads from LinuxCNC's tool data, sets through `G10 L1`, which also
    rewrites the tool file), `GET_TOOLTABLE_INDEX` (tool number = index here), `G43`, `G49`,
    `SET_VARS` (`#n=` assignments through MDI), `GET_VARS` (NML exposes no interpreter
    parameters: read from `[RS274NGC] PARAMETER_FILE`, which task rewrites after every MDI
    line and program, so only the numbers in that file - add a `600 0` line to make #600
    persistent and readable), `UPDATE_FIXTURE` (re-selects the active G5x), `GET_GCODE_LINE`,
    `GET_DATE_TIME`. Not supported (result -1, logged once): KMotionCNC's own dialog - jog
    keys, controls, the dialog face, screen scripts, the edit cell, `INPUT`, geo correction,
    per-axis jog overrides, `SET_TP_PARAM`, `G43.4`, tool comments. Values follow
    KMotionCNC's conventions: the interpreter's current units for positions and offsets
    (LinuxCNC's status is in machine units; converted), counts per inch for `GETAXISRES`.
    The handshake: a status read can be on the wire while the result is written and still
    show the command, so the next command is taken only from a read two counts later; a
    command already present in the first read is refused with -2 (left over by an earlier
    session), as KMotionCNC does; one command runs at a time. Every command and result goes
    to the `-l` log. Test, with the Kogna and LinuxCNC on kmotion-kogna: `pccomm/pccomm-test.py`
    starts `pccomm/PCCommTest.c` on the board (MDI `M101`) and checks the override and the
    touch-off from LinuxCNC's side; the program's own ok/FAIL lines are on the board's
    console (kmotion-motion's stderr).
  - Planner settings come from the ini: `[KMOTION]` (mode, init program, 3rd order, cubic
    knots, actuator limits, segment log, break angle, tolerances, lookahead, board
    channels), the axis limits from `[AXIS_*] MAX_VELOCITY / MAX_ACCELERATION / MAX_JERK`
    (jerk defaults to 10 × accel) and counts per unit from `[JOINT_n] INPUT_SCALE`.
- `configs/kmotion-sim/` – a LinuxCNC configuration that loads `kmotion-motion` instead of
  `motmod` in simulate mode (the sim/axis config, inches, 3 joints, trivkins), plus
  `drive.py`, which runs the whole thing from the command line.
- `configs/kmotion-kogna/` – the same for Tom's Kogna: board mode, 2540 counts/inch,
  250 mm/s, 2500 mm/s², jerk 25000 mm/s³ in inches, `MinirouterInit.c` as the init program,
  homing through `Home.c`, the bench spindle on channel 3 (`spindle/`), gmoccapy as the GUI.

## Build and run

Needs a LinuxCNC run-in-place build (`~/linuxcnc-dev` here; `LINUXCNC_DIR` in the makefile
picks another one). Its headers, `liblinuxcnchal` and `libnml` are used; three small
files of its tree (the motion error ring buffer helpers) are compiled in.

    cd kmotion-motion && make
    source ~/linuxcnc-dev/scripts/rip-environment
    linuxcnc -v -d ~/KMotionX/KMotionX/linuxcnc/configs/kmotion-sim/kmotion-sim.ini
    python3 ~/KMotionX/KMotionX/linuxcnc/configs/kmotion-sim/drive.py   # in another terminal

A desktop icon that starts LinuxCNC with the kmotion-kogna or the kmotion-sim config:
`../desktop/install-desktop-icons.sh` (see `../desktop/README.md`).

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
timeline; `home`, `spindle` and `jog` run homing, the spindle actions and jogs on the board).
Never do that while a LinuxCNC or kmxWeb is connected to the board: the init
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

1. Homing on a real machine (the switch bits in Home.c); in-position from the board's
   `AxisDone` for moves as well as jogs; jogging while paused.
2. Spindle: spindle-synchronized motion (G33/G76) would need the board's threading.
3. Kinematics on the KMotion side (TP3 plans in actuator space); LinuxCNC's kinematics
   module is bypassed and only gets joint positions to display.
4. mm configs (LinuxCNC machine units vs KMotion's inches).
