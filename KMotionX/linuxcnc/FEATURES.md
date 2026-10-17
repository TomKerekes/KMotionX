# LinuxCNC on KMotion: supported features

What LinuxCNC can do with `kmotion-motion` in place of its real-time motion module: the planning on the PC by KMotion's 3rd Order trajectory planner, the execution on a KFLOP or Kogna. The G and M codes are LinuxCNC's own (its interpreter is untouched but for one optional patch, below); this lists what reaches the board and how. `README.md` has the details and the ini keys of each item, `configs/` the three configurations (`kmotion-sim`, `kmotion-kogna`, `kmotion-3link`).

## Motion

- **3rd Order, jerk-limited trajectory planner** (TP3): every move planned with velocity, acceleration and jerk limits per axis (`[AXIS_*] MAX_VELOCITY / MAX_ACCELERATION / MAX_JERK`) or per actuator (`[KMOTION] ACTUATOR_LIMITS = 1`, from `[JOINT_n]`), cubic knots, lookahead `[KMOTION] LOOKAHEAD` seconds; the plan streams to the board as it executes. `THIRD_ORDER = 0` selects KMotion's standard planner.
- **Lines, arcs, helices**: G0, G1, G2, G3 (any plane: G17, G18, G19; helical; more than one turn), G4 dwell. Rapids at the board's rapid override.
- **Path modes**: G64 (blended, the machine's corner and collinear tolerances), G64 P (the programmed tolerance), G61 exact path, G61.1 exact stop. A move through a nonlinear kinematics is subdivided into pieces; in the exact modes the planner stops at the programmed vertices only, not at the pieces.
- **Feed modes**: G93 inverse time, G94 units per minute, G95 units per revolution (LinuxCNC's interpreter; the planner gets the resulting feed).
- **Overrides**: feed override (the board's FRO, applied to a running move), rapid override, spindle override (through the spindle's S action). Every override is 1 during a tap or a threading run, as LinuxCNC's motion ignores it there.
- **Feed hold and resume**: the board's own feed hold (`StopImmediate`), so the stop is the controller's ramp whatever the PC does; abort / stop as KMotion's halt (stop, wait for rest, clear). A pause is refused during a rigid tap.
- **Back / Forward in feed hold**: KMotionCNC's buttons, as HAL pins and on the operator panel - the paused program runs backwards or forwards along its path while a button is held, at a fraction of the programmed speed (the board's temporary override).
- **Soft limits**: the interpreter checks programmed moves against the CAD axes' `[AXIS_*]` limits; `kmotion-motion` checks every move and jog against the joint limits `[JOINT_n] MIN_LIMIT / MAX_LIMIT` through the kinematics (actuator space) as well, and against the kinematics' reach; a move beyond either is refused with the value and the limit. World jogs stop where a joint would reach its limit on the jog's path.
- **Hardware limit switches** per joint (`[KMOTION] LIMIT_NEG_BITS / LIMIT_POS_BITS / LIMIT_ACTIVE`, or the HAL pins `joint.N.neg-lim-sw-in / pos-lim-sw-in`): a switch made sets the joint's error, takes the machine off (the joint stays homed); the GUI's "ignore limits" lets one jog off the switch.
- **Program run, single step, MDI**, run from line, as LinuxCNC's task provides them.
- **Units**: the configurations are in inches (the kinematics' and KMotion's units); G20 and G21 programs both run (the interpreter converts). A millimetre machine configuration is not done.

## Kinematics

- **Nonlinear kinematics on the KMotion side** (`[KMOTION] KINEMATICS = <name>`, the names KMotionCNC's `Kinematics.txt` takes: `3Link`, `Scara`, `Geppetto`, `Kinematics3Rod`, the 5-axis gimbal and table ones): the planner turns the CAD path into actuator motion; LinuxCNC's joints are the actuators in their own units (degrees for a rotary one), its axes the CAD coordinates. Kinematics type BOTH: joint mode jogs one actuator, world mode moves the CAD position.
- **Identity** without a `KINEMATICS`: joint i is axis i.
- **Geometric correction** (`[KMOTION] GEO_FILE`): KMotionCNC's Geo Correction File (the XY table and per-axis linear tables), read at start, reloaded by the panel's Geo reload button or the board's `PC_COMM_RELOAD_GEOCORRECTION`; the **Measure** button records the current position as the next grid point of such a table (`MEASURE_FILE`).

## Jogging

- Continuous and incremental jogs from any GUI or HAL (`INCREMENTS`): a joint jog is the board's own jog of that channel (`Jog`, `MoveAtVel`), stopped by the board at the joint's limit; a world jog under a kinematics is a planner move toward the axis limit, ended at the button's release like an abort. Jogging while the program is paused is not done (the hold jog above runs the program itself).

## Homing

- `[JOINT_n]` homing as LinuxCNC specifies it (`HOME_SEARCH_VEL`, `HOME_LATCH_VEL`, `HOME_OFFSET`, `HOME`, `HOME_FINAL_VEL`, `HOME_USE_INDEX`, `HOME_SEQUENCE`), carried out by a C program on the board (`[KMOTION] HOME_PROGRAM`, `configs/kmotion-kogna/Home.c`: search, back off, latch, index, set position, move to HOME); the switch and index bits are the program's table. A joint with `HOME_SEARCH_VEL = 0` is homed where it stands.
- **Absolute feedback**: `[KMOTION] ALWAYS_HOMED = 1` (homed whenever the machine is on), `HOMED_PERSIST = <n>` (the homed joints remembered on the board across sessions).
- **Sessions**: `[EMC] KEEP_ENABLED_ON_EXIT = 1` (two patches to LinuxCNC) leaves the axes enabled when LinuxCNC exits, and a start that finds them enabled can come up with Machine on and no init (`configs/kmotion-3link/autostart.py`); an **enable program** (`ENABLE_PROGRAM`) can do the machine on itself (serial servos' torque and set-up).

## Spindle

- **Spindle control as KMotionCNC's** (`[KMOTION] SPINDLE_M3 / SPINDLE_M4 / SPINDLE_M5 / SPINDLE_S`): each an action - relay bits (`SETBIT`, `SETTWOBITS`), a DAC (`DAC`: speed x scale + offset, e.g. a 0-10 V drive on a Kanalog DAC), or a C program on the board (`PROGRAM`, `PROGRAM_WAIT`, `PROGRAM_WAIT_SYNC`; Dynomotion's Spindle Using Jogs programs for a spindle that is a board axis). M3, M4, M5, S, the override; `[SPINDLE_0]` limits.
- **G96 constant surface speed** (`SPINDLE_CSS = 1`) on the board, as KMotionCNC does it: a `ServiceCSS()` loop sets the speed from X; G97 back to RPM; D the maximum.
- **Measured spindle speed** (`SPINDLE_SPEED_AXIS`, from the channel's command or its encoder) on HAL pins for the GUI's display; **spindle at speed** (`SPINDLE_AT_SPEED`, an input bit or the axis's measured speed within `SPINDLE_AT_SPEED_TOLERANCE`) holds the next feed move after a spindle command, as LinuxCNC's motion does; `spindle.N.at-speed` counts too.
- **Rigid tapping**: G33.1 (K pitch, I retract speed multiplier), and **G84 / G74** tapping cycles as rigid taps with `[RS274NGC] RIGID_TAP_CYCLES = 1` and the interpreter patch `patches/interp-rigid-tap-cycles.patch` (pitch F/S in G94, F in G95; R, L, G98 / G99 as in any canned cycle; without the key LinuxCNC's floating-chuck G84). The board's spindle-sync mode (`TrigThread`): the motion is slaved to the spindle encoder through any kinematics, a program on the board (`TAP_PROGRAM`, `Tap.c`) turns the spindle the revolutions for the depth and reverses it, the motion follows it back out, the spindle restarts as M3 had it.
- **Threading**: G33 (spindle-synchronized feed, K pitch) and the lathe threading cycle G76 (P pitch, Z, I, J, K, Q, H, E, L: depths, compound angle, spring passes, tapers), each pass a synchronized run started at a whole spindle revolution so the passes line up; the pull-out rides with the cut.
- Needs a spindle encoder on a board channel (`SPINDLE_SPEED_AXIS`, `SPINDLE_COUNTS_PER_REV`) for the measured speed, tapping and threading. Not done: peck tapping (no such LinuxCNC code), M19 orient.

## Probing

- **G38.2, G38.3, G38.4, G38.5** (`[KMOTION] PROBE_BIT`, `PROBE_ACTIVE`): the probe move runs through the planner (any kinematics), a watcher on the board (`ProbeService.c`) stops the motion at the contact and records the actuators; the probed position (#5061-#5069), `probeTripped` and the errors as LinuxCNC's motion gives them.

## I/O, M codes, the board's programs

- **Board I/O as HAL pins**: `[KMOTION] OUTPUT_BITS` (`kmotion.out.<bit>`, written on change), `INPUT_BITS` (`kmotion.in.<bit>`, read every status period); any HAL signal can drive a board output: the configurations net **M7 / M8 / M9** coolant (`iocontrol`) to bits. **M64 / M65 P<n>** digital outputs (`NUM_DIO`) are set at once; M62 / M63 (in step with the next move) are refused for now. **M66** wait-for-input is LinuxCNC's own on HAL pins.
- **Custom M codes M100-M199**: KMotionCNC's M-code actions (`[KMOTION] MCODE_<n>`: bits, DAC, a C program with P and Q) through LinuxCNC's `USER_M_PATH` executables.
- **Init and background programs**: `INIT_PROGRAM` (any number, run in order at the session's first Machine on: the axes' set-up), `START_PROGRAM` (left running: the board's service loops - CSS, probe watcher, limit simulation). LinuxCNC's start only connects to the board and checks its firmware; a missing board or a firmware mismatch is reported in plain words when Machine on is pressed, which tries the board again.
- **The board's commands to the PC** (`PC_COMM = 1`): a C program on the board drives LinuxCNC through KMotionCNC's PC_COMM mechanism - E-stop, halt, run, single step, MDI, M codes, overrides, touch-off (`SET_X`..), tool table reads and writes, G43 / G49, variables, messages to the operator, user buttons (`USER_BUTTON_<n>`), DROs and settings back to the board. KMotionCNC's own dialog commands (jog keys, screen scripts, input boxes) are not supported.

## Operator interface

- **GUIs**: gmoccapy (the configurations; two small patches for the enabled-on-exit session and the start-up state of its buttons and DROs) and AXIS (no patch); any LinuxCNC GUI should work, as the task-motion protocol is unchanged.
- **The KMotion panel** on gmoccapy's main screen (gladevcp, `panels/`): Back / Forward in feed hold, Measure, Geo reload, Continue (run again from the line after a halt, as KMotionCNC's Go).
- **HAL pins** of LinuxCNC's motion that GUIs use (`motion.*`, `joint.N.*`, `axis.L.*`, `spindle.N.*`), plus `kmotion.*` (the panel's, the spindle's measured speed and at-speed, the board bits, `all-enabled`, `homed-mask`).
- **Simulate mode** (`[KMOTION] MODE = simulate`): the same planner without a board, the plan replayed in time - LinuxCNC shows the controller's motion.

## Motors and feedback

The board's channels are configured by the init program, as in KMotionCNC; LinuxCNC needs only the channel of each joint (`[KMOTION] CHANNELS`) and its counts per unit (`[JOINT_n] INPUT_SCALE`). Whatever KMotion drives on a channel runs under LinuxCNC:

- **Step and direction**: open loop (`Step Dir`), closed loop with an encoder (`CL Step`), microstepping on the board's own outputs (`Microstep`, `CL Micro`).
- **Analog servo drives**: +/-10 V through a Kanalog DAC (`DAC Servo`), the board closing the position loop on the encoder.
- **Brushed DC servo** (PWM, `DC Servo`) and **brushless 3-phase / 4-phase** commutated by the board (`3PH Servo`, `4PH Servo`), on SnapAmps or external drives.
- **Serial servos** (`Serial Servo`, `CL Serial Servo`: the Kogna's bus, Dynamixel on the 3 Link robot), set up and watched by the enable program.
- **Feedback**: quadrature encoders, analog (ADC), resolvers, serial servo position, or a program's own (`User Input`); absolute feedback with `ALWAYS_HOMED` / `HOMED_PERSIST`.

## Boards and modes

- **Kogna** (Ethernet) and **KFLOP** (USB) through KMotionServer, like KMotionCNC and kmxWeb; one board per LinuxCNC. No real-time kernel on the PC.
- The 3 Link pen robot (`configs/kmotion-3link`, three serial servos, 3Link kinematics) and the bench (`configs/kmotion-kogna`, step/dir, a spindle axis with a looped encoder) are the two machines the above has run on; `kmotion-sim` runs it all without a board.

## Not done

Jogging while paused; M62 / M63; a millimetre machine configuration; LinuxCNC's "probe tripped during non-probe move"; M19 spindle orient; peck tapping; KMotionCNC's dialog commands over PC_COMM; the legacy (non-TP3) planner's exact-mode stops at the pieces of a move.
