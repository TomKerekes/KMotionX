# LinuxCNC with KMotion's planner and boards

Work in progress (started 2026-10-02): LinuxCNC's task and GUIs on one side, KMotion's
trajectory planner (TP3) and a KFLOP or Kogna on the other, with no real-time kernel on
the PC. LinuxCNC's motion module (`motmod`, a real-time HAL module) is replaced by a
plain userspace process, `kmotion-motion`, that speaks the same task↔motion protocol
through the same shared memory block.

## Layout

- `kmotion-motion/` – the process. `kmotion-motion.cc` decodes every emcmot command,
  acknowledges it, keeps the status block and the HAL pins a GUI expects up to date, and
  for now executes the moves with a stand-in model (segments run at their commanded speed,
  no acceleration, no blending; jogs; immediate homing). Derived from LinuxCNC's
  `motion-logger.c`, hence GPL-2.
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

## Next

1. Hand `SET_LINE`/`SET_CIRCLE` to `CCoordMotion` (`StraightFeedAccelRapid`, `ArcFeed`, TP3)
   with `CKMotionDLL` in simulate mode; positions back from the planner/board.
2. The Kogna: axis definitions, feed override through the board, Halt/Resume as kmxWeb
   does it, homing on the board side.
3. Kinematics on the KMotion side (TP3 plans in actuator space); LinuxCNC's kinematics
   module is bypassed and only gets joint positions to display.
