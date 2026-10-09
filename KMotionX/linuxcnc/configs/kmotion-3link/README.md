# kmotion-3link: LinuxCNC on the 3 Link pen robot

Dynomotion's 3 Link robot: three Dynamixel XL430 servos (Right, Left, Z) on the Kogna's serial
servo bus. The kinematics (`GCodeInterpreter/Kinematics3Link.cpp`) and the 3rd order planner
run on the KMotion side; LinuxCNC runs the G-code and the GUI, through `kmotion-motion` in the
motion module's place (`KMotionX/linuxcnc/README.md`).

    source ~/linuxcnc-dev/scripts/rip-environment
    linuxcnc kmotion-3link.ini

`ngc/DynoMotion3Link.ngc` is the demo program (in mm, G21).

## How it hangs together

- `[KMOTION] KINEMATICS = 3Link` gives the planner the 3Link kinematics (the names are those
  of KMotionCNC's `Data/Kinematics.txt`). The planner turns the pen's X Y Z (inches) into the
  servo angles; LinuxCNC is told the kinematics are not trivial (kinematics type BOTH), so
  its **joints are the servos**, in degrees, and its **axes the pen**:
  - joint mode (gmoccapy's Joint/World button) jogs one servo, on the board, within the
    joint's `[JOINT_n]` limits;
  - world mode jogs the pen: a planner move toward the axis's `[AXIS_*]` limit, stopped when
    the button is released;
  - gmoccapy shows the joints in joint mode and the pen position in world mode.
- Homing: the servos are absolute (they remember their position, multi-turn), so every joint
  is homed where it stands (`HOME_SEARCH_VEL = 0`). Home All just declares them homed, after
  which MDI, programs and world jogs are allowed.
- Machine on runs `DxlAxisInit3.c` (`[KMOTION] ENABLE_PROGRAM`, thread 1): it sets the servos
  up (their PID gains, which reset at power up; the position offsets; torque on), enables the
  axes at their measured positions, and keeps running as a watchdog: when an axis stays
  disabled it turns the torque off on every servo and ends. Machine off (and E-stop) disables
  the axes, so that is what happens; the next machine on runs the program again. If a servo
  does not answer, the program enables nothing and kmotion-motion reports it after
  `ENABLE_TIMEOUT_S`; the program's own messages are on the board's console, printed by
  kmotion-motion on its terminal.
- `[KMOTION] ACTUATOR_LIMITS = 1`: the planner limits the servos by `[JOINT_n]`'s
  MAX_VELOCITY / MAX_ACCELERATION / MAX_JERK (degrees), not the pen by the axis limits.
- The file is a copy of Tom's `C Programs/SerialServo/Dynamixel/DxlAxisInit3.c` (KMotionSrcKogna);
  it must agree with `[JOINT_n]`: channel = servo ID = joint (0 Right, 1 Left, 2 Z), the
  Vel/Accel/Jerk in counts/s are the joint limits in degrees x 11.378.

## Still to set for the machine

1. `[JOINT_n] INPUT_SCALE`: 11.37778 counts per degree (4096/rev), with the sign of each servo's
   direction: Right and Left count counterclockwise seen from above, Z counterclockwise seen
   from the right (+X) side, as `Kinematics3Link.cpp` defines the angles. A wrong sign shows
   as the pen moving the wrong way, or the inverse kinematics failing to converge.
2. The servos' zero: `DxlAxisInit3.c`'s `InputOffset0`/`OutputOffset` make a chosen pose read 0;
   the kinematics' `OFFSET_*_DEG` are 0, so that pose has to be the arm at its geometric 0
   (Right/Left along +X, Z toward the paper). To calibrate: at any pose read the counts and
   estimate the arm angle; OFFSET = counts / 11.378 - angle (comment at the top of
   `Kinematics3Link.cpp`).
3. `[JOINT_n] MIN_LIMIT / MAX_LIMIT`: the arms' real range (joint jogs stop there).
4. `[AXIS_*] MIN_LIMIT / MAX_LIMIT`: a box inside the reachable workspace (world jogs run to
   them; programmed moves are checked against them).
5. The speeds: `[AXIS_*] MAX_VELOCITY` 3 in/s and `[TRAJ]` match the demo's F4000 mm/min;
   raise `[JOINT_n] MAX_VELOCITY` together with the servos' `Vel` in `DxlAxisInit3.c`.
6. M100 (the demo program's first line) and M3/M5: nothing is assigned; `mcodes/M100.c` only
   prints. KMotionCNC's Tool Setup says what they did there.

## In a virtual machine, or on another PC

kmotion-motion needs no real-time kernel: the Kogna does the real time, the PC only plans
ahead. A VirtualBox VM is fine. What Tom's laptop runs, to reproduce it (Debian 13, XFCE):

- **LinuxCNC:** the `master` branch of github.com/LinuxCNC/linuxcnc at commit `514be4f657`
  (2026-09-20, "Merge pull request #4575 from grandixximo/merge-2.9"), which calls itself
  2.10.0~pre1 (`git describe`: v2.9.10-5543). A run-in-place build, `--with-realtime=uspace`,
  no documentation, not setuid (uspace warns that it cannot get real-time priority; it does
  not need it here).

      sudo apt install git build-essential autoconf automake dh-python desktop-file-utils \
          intltool bwidget libboost-python-dev libepoxy-dev libgl-dev libglu1-mesa-dev \
          libgtk-3-dev libcap-dev libmodbus-dev libgpiod-dev libeditreadline-dev libtirpc-dev \
          libudev-dev libusb-1.0-0-dev libxmu-dev libfmt-dev netcat-openbsd procps psmisc \
          python3 python3-dev python3-pybind11 python3-tk python3-xlib python3-opengl \
          python3-numpy python3-configobj python3-cairo python3-gi python3-gi-cairo \
          gir1.2-gtk-3.0 gir1.2-gtksource-4 python3-pyqt5 python3-pyqt5.qsci \
          python3-pyqt5.qtsvg python3-pyqt5.qtopengl python3-dbus python3-qtpy python3-zmq \
          tcl tcl8.6-dev tclx tk8.6-dev yapps2
      git clone https://github.com/LinuxCNC/linuxcnc.git ~/linuxcnc-dev
      cd ~/linuxcnc-dev && git checkout 514be4f657
      cd src && ./autogen.sh && ./configure --with-realtime=uspace && make -j$(nproc)

  Then `source ~/linuxcnc-dev/scripts/rip-environment` in every terminal that runs it
  (`linuxcnc` alone opens the config picker: `configs/sim/gmoccapy` is a good first check).

- **KMotionX:** Tom's fork, `master` (github.com/TomKerekes/KMotionX). Plain `make`, never
  `make -j` (the subdirectory builds race) and never inside a subdirectory; the libraries
  and KMotionServer go to `/usr/local` and `~/.kmotionx` with `sudo make install`.

      sudo apt install libftdi1-dev
      git clone https://github.com/TomKerekes/KMotionX.git ~/KMotionX
      cd ~/KMotionX && ./configure && make && sudo make install
      make -C ~/KMotionX/KMotionX/linuxcnc/kmotion-motion      # kmotion-motion, kmotion-mcode

  Optional, the KMotion tool (`KMotionX/tools/kmotion`): `sudo apt install
  python3-pyside6.qtwidgets python3-pyside6.qtgui` and `make -C ~/KMotionX/KMotionX/tools/kmotion/shim`.

- **This config:** `cd ~/KMotionX/KMotionX/linuxcnc/configs/kmotion-3link && linuxcnc
  kmotion-3link.ini` (after the rip-environment). The paths in `kmotion-3link.hal` and
  `mcodes/M100` are relative to the tree, so the two clones must sit as above.

- **The Kogna:** the VM needs a network path to it. The Kogna hands out the addresses on its
  own link (192.168.113.x), so give the VM a bridged adapter on the host's interface that
  connects to the Kogna (the USB-Ethernet adapter on the laptop); KMotionServer finds the
  board by itself, and `/tmp/kmotion-motion.log` starts with "connected to a Kogna". A KFLOP
  on USB would need the VM's USB filter for FTDI 0403:6001 instead.
