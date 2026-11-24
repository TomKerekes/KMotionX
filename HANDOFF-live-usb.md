# Hand-over: KMotionX / LinuxCNC on KMotion - state of the repository, and the live-USB goal (2026-11-08)

A fresh Claude Code session in ~/KMotionX can start from this file: "read HANDOFF-live-usb.md and continue".
Written on Tom's VM (hostname LinuxCNCdeb, user tk, Debian 13.7, kernel 6.12 rt, 38 GB disk with 19 GB free,
8 GB RAM, 2 CPUs; a VirtualBox guest on Tom's Windows PC; shared folder C:\ShareVM\kmotion = /mnt/kmotion).

## The repository

~/KMotionX is Tom's fork TomKerekes/KMotionX (github), branch master, pushed through 842e6b9. It is the
Dynomotion KMotion library ported to Linux (KMotionDLL, GCodeInterpreter, KMotionServer, TCC67, the
DSP_KFLOP / DSP_KOGNA firmware images at KMotion 5.5.1) plus, under KMotionX/:
- linuxcnc/kmotion-motion/: kmotion-motion, a userspace replacement for LinuxCNC's motion module. LinuxCNC's
  task talks to it through the usual shared memory; it runs KMotion's TP3 trajectory planner and drives a
  KFLOP (USB) or Kogna (Ethernet) through KMotionServer. Files: kmotion-motion.cc (LinuxCNC side: HAL pins,
  the command protocol, homing/limits/jogs model), kmotion-backend.cc/.h (the KMotion side: worker, poller
  and urgent threads, CoordMotion, board programs, spindle/M-code actions), kmotion-pccomm.cc (the
  board's commands to the PC), kmotion-mcode.c (user M codes M100-M199 and the M6 tool change reach the
  module over a Unix socket), backend-test.cc (make test: 10 simulated scenarios; KM_BOARD=1 adds ones
  that touch the real board - never while LinuxCNC runs). Built with make in that folder (LINUXCNC_DIR).
- linuxcnc/configs/: kmotion-minirouter (the bench: 3 step/dir axes, a jogged spindle axis, simulated
  home/limit/probe switches in BenchLoop.c, M100-M102, M6 ToolChange.c; works on a KFLOP or a Kogna
  unchanged), kmotion-3link (a Kogna pen robot with a kinematics), kmotion-sim (no board). Each ini's
  [KMOTION] section is documented in the minirouter ini. configs/<user's> (default, carl...) are untracked.
- linuxcnc/ngc/kmotion_m6.ngc (the M6 remap), linuxcnc/panels/ (gladevcp panel, gamepad-input),
  linuxcnc/patches/ (4 patches on LinuxCNC: gmoccapy keep-enabled-on-exit + limit-override fixes,
  gremlin backplot clear, interp rigid-tap cycles, task keep-enabled-on-exit), linuxcnc/tools/
  migrate-kmotioncnc (KMotionCNC Tool Setup -> a LinuxCNC config), linuxcnc/README.md (the full design
  notes), linuxcnc/FEATURES.md.
- linuxcnc/install/: QUICKSTART.md (step by step for a Windows PC), INSTALL.md, vm-create.cmd / .sh
  (VirtualBox VM: VBoxSVGA, OHCI USB with filters for the KFLOP, the Kogna COM port and the F310 game
  pad, bridged adapter to the Kogna, shared folder), install-vm.sh (everything after the OS install,
  rerunnable; see below).
- desktop/: install-desktop-icons.sh and the launchers it makes: KMotion (tools/kmotion, a PySide6
  tool), Kogna COM Port (tio), Pull KMotionX, Re-install LinuxCNC, Migrate from KMotionCNC, LinuxCNC
  (Minirouter / 3 Link / Simulator); clipboard-png (XDG autostart: a Windows bitmap on the shared
  clipboard offered as PNG, so browser-based apps can paste it).
- tools/kmotion/: the Linux KMotion tool (Digital I/O, Axis, C Programs, Config & Flash, Console).
- examples/LinkProbe/: timing tools for the PC-board link (built into bin/).
- usb/etc/udev/rules.d/: 90-kflop.rules (0403:f231 and 0403:6001 Dynomotion, MODE 0666, uaccess),
  11.joystick.rules.

Other trees on this VM: ~/linuxcnc-dev = LinuxCNC run-in-place, clone of github LinuxCNC/linuxcnc pinned
at commit 514be4f657 with the 4 patches applied, built with --with-realtime=uspace (no RT kernel needed:
the board does the real time; no "make setuid"). ~/KMotionSrcKogna = Dynomotion's source clone (reference
only). ~/.kmotionx = the installed KMotion home (bin/KMotionServer, tcc67, DSP_KFLOP, DSP_KOGNA, C
Programs, Data). /usr/local/lib/kmx = the installed libraries (libKMotion, libKMotionX, libGCodeInterpreter).

## How an install is made today (install-vm.sh, the model for the live image)

1. apt: deb-src on; the package list PACKAGES (build tools, LinuxCNC's build needs - its own
   `apt build-dep` adds the rest - libftdi1-dev, PySide6, matplotlib, tio, geany, network-manager,
   dkms + headers for the running kernel). brltty purged (it grabs FTDI 0403:6001).
2. KMotionX: clone to ~/KMotionX, ./configure, make (plain, never -j), sudo make install
   (/usr/local/lib/kmx, ~/.kmotionx), then make again (ownership), sudo chown -R ~/.kmotionx.
3. udev rules copied to /etc/udev/rules.d, udevadm reload; user into plugdev, dialout, vboxsf.
4. LinuxCNC: clone, checkout 514be4f657, the patches (a changed patch replaces its old version),
   debian/configure + apt build-dep, autogen + configure --with-realtime=uspace
   --disable-build-documentation, make -j.
5. kmotion-motion: make, make test (~5 min).
6. VirtualBox Guest Additions (skipped when not a VM).
7. The KMotion tool's C shim (tools/kmotion/shim), the desktop icons (install-desktop-icons.sh:
   ~/Desktop, the applications menu, the clipboard helper's autostart), the Kogna's network profile.
Paths are absolute into the user's home (launchers, rpath of kmotion-motion into ~/KMotionX/bin, the
ini's PROGRAM_PREFIX ...): an image needs one fixed user name and home.

## Recent work (all pushed; details in ~/vm-build-notes.md, sections 1-51)
- 42-46: a start without a board is clean; Machine on's first click connects (a stray byte in the FTDI
  handshake, KmotionIOX.cpp); a board swapped while running gets its init programs again; "Ignore
  limits" switches task to manual (gmoccapy patch); install/update icons; USB filters by ids only.
- 47: migrate-kmotioncnc; 48-50: jog ordering and the poller yielding the link (Carl's laptop VM: a
  status read 55 ms over VirtualBox's OHCI USB; the probes in examples/LinkProbe); 51: M6 on the board.
- The firmware images are the KMotion 5.5.1 release's (Kogna build Sep 29 2026).

## Standing rules
No timing heuristics in the module. Commits end with "Co-Authored-By: Claude Fable 5.1
<noreply@anthropic.com>". ~/vm-build-notes.md gets a numbered section per piece of work. Never start
LinuxCNC (or the probes) while a LinuxCNC runs (pgrep -x milltask); the user's LinuxCNC on this VM
owns the board. Keep tool calls short while the user drives; never pkill -f with a pattern that
matches your own command line; run commands over ~2 min in the background. gmoccapy runs from a build
copy: patch ~/linuxcnc-dev/src/emc/usr_intf/gmoccapy/gmoccapy.py, make -C ~/linuxcnc-dev/src
../bin/gmoccapy, regenerate linuxcnc/patches/gmoccapy-keep-enabled-on-exit.patch with git diff.

## Next goal: a bootable live USB (Debian 13 live, installable) with everything preinstalled

Wanted: an ISO/USB image built with Debian live-build in this VM, booting to the Xfce desktop with
LinuxCNC (the RIP tree at 514be4f657 + patches), kmotion-motion, KMotionX (libraries, KMotionServer,
tcc67, the KMotion tool, the configs, the desktop icons, udev rules, clipboard helper) and
KMotionXCNC (TomKerekes/KMotionXCNC on github: the web UI, Angular "ng2" + kmxWeb; how it is built
and launched is to be worked out - not in install-vm.sh today) preinstalled, plus an "install to
disk" option (live-build's debian-installer integration or Calamares).

Approach to take: live-build (apt install live-build) in ~/live; `lb config` for Debian 13 (trixie),
amd64, iso-hybrid, Xfce task, --debian-installer live (or calamares), the package list from
install-vm.sh's PACKAGES plus LinuxCNC's build-dep set; a chroot hook (config/hooks/live/) that runs
the same steps as install-vm.sh non-interactively as root for a fixed user (no sudo prompts, no Guest
Additions, no network profile), building LinuxCNC and KMotionX inside the chroot (an hour or more in
the VM: cache the clones, or build once and copy the trees in through config/includes.chroot/), the
user's home with the icons (install-desktop-icons.sh as that user), udev rules, autostart. Decide: the
live user's name and home (every absolute path in the install depends on it; the installer-to-disk
should create the same user), persistence for a live USB (configs, ~/.kmotionx, position files),
stock kernel (fine: no RT needed) and firmware packages, image size (~3-4 GB). Disk: live-build wants
10-15 GB free; this VM has 19 GB. Build in the background, log to a file, check in short calls.
