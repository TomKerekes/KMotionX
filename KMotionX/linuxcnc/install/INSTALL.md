# Installing LinuxCNC on KMotion from scratch

`QUICKSTART.md` is the same as a bare list of steps for a Windows PC; this file says why. A procedure, with scripts, for a fresh machine: a VirtualBox VM here, but step 3 on is the same on any Debian 13 PC. Nothing needs a real-time kernel: the KFLOP or Kogna does the real time, the PC plans ahead. What you get at the end is what `FEATURES.md` lists: LinuxCNC (the run-in-place tree, 2.10 pre-release, with the four small patches in `patches/`), KMotionX (Tom's fork: the libraries, KMotionServer, the planner, kmotion-motion), the KMotion tool, the desktop icons and the three configurations.

The scripts are in this folder: `vm-create.cmd` / `vm-create.sh` make the VM on the host, `install-vm.sh` does everything inside it. They were written from the notes of building the existing VM by hand and have not yet been run on a clean machine: the first clean install is the test.

## 1. What you need

- A host whose processor virtualization (Intel VT-x, AMD-V) is enabled in the BIOS: Task Manager > Performance > CPU shows "Virtualization: Enabled". Disabled, the VM stops at once with "VT-x is disabled in the BIOS"; the switch is in the BIOS setup under Advanced > CPU or, on Lenovo, Security > Virtualization (`QUICKSTART.md`, step 1).
- A host with Oracle VirtualBox 7 installed (skip this if it already is): the installer for your host from https://www.virtualbox.org/wiki/Downloads ("Windows hosts" on Windows; on Linux the distribution's package or Oracle's), installed with its defaults. `vm-create.cmd` runs it from `C:\Program Files\Oracle\VirtualBox`, `vm-create.sh` needs `VBoxManage` on the PATH. The Extension Pack is not needed: the KFLOP's USB interface (FTDI 0403:6001, the FT232/FT245 family) is a USB 2.0 full-speed device, 12 Mbit/s, and VirtualBox's built-in OHCI controller (labelled USB 1.1) serves full-speed devices; the pack only adds the high-speed and USB 3 controllers, which the KFLOP cannot use. A VM that has the pack and an xHCI controller works as well.
- The Debian 13 "netinst" ISO, `debian-13.x.x-amd64-netinst.iso`, from debian.org (about 700 MB). The VM downloads the rest during the install, so the host needs internet.
- For the Kogna: the host's network interface on the Kogna's network, which the VM gets a bridged adapter on. With its IP setting 0 the Kogna looks for a DHCP server: on a network with a router it takes the router's address (then the interface is the PC's own connection to that network), alone on a cable to a spare port it finds none and becomes the DHCP server itself, handing out 192.168.113.x (a USB-Ethernet adapter on the laptop, here). With a fixed IP set in the Kogna, the interface on that network. For a KFLOP: a USB port; the VM takes the board through a USB filter.
- The board must run the firmware this tree carries (`DSP_KOGNA/DSPKOGNA.out`, `DSP_KFLOP/DSPKFLOP.out`): kmotion-motion checks the build stamp when it connects and refuses Machine on with both builds named until they match. The KMotion tool's Config & Flash screen flashes it (or KMotion on Windows).
- Time: the Debian install takes 15 minutes, `install-vm.sh` 30 to 60 (LinuxCNC's build is most of it).

## 2. The VM

With the script, on the host. Get the one file for your host (no checkout is needed yet; the VM gets its own in step 3):

    https://raw.githubusercontent.com/TomKerekes/KMotionX/master/KMotionX/linuxcnc/install/vm-create.cmd
    https://raw.githubusercontent.com/TomKerekes/KMotionX/master/KMotionX/linuxcnc/install/vm-create.sh

and run it from a command prompt (Windows: Start, type `cmd`) in the folder you saved it to:

    vm-create.cmd  C:\path\debian-13.1.0-amd64-netinst.iso  "<host adapter to the Kogna>"  [C:\kmotion-share]
    bash vm-create.sh  ~/debian-13.1.0-amd64-netinst.iso  "<host interface to the Kogna>"  [~/kmotion-share]

Run it without the adapter name once to see the host's adapters as VirtualBox names them (`VBoxManage list bridgedifs`). The optional third argument is a shared folder, mounted at `/mnt/kmotion` in the VM once the Guest Additions are in.

By hand instead: New VM, type Linux / Debian (64-bit), 4 GB memory, 2 CPUs, a 40 GB disk; Display: VMSVGA, 64 MB; Network: adapter 1 NAT (internet), adapter 2 bridged to the host's interface to the Kogna, cable connected; USB: enabled, the OHCI (USB 1.1) controller is enough, a filter with vendor 0403 and product 6001 (the KFLOP); Storage: the ISO in the optical drive. The script does exactly this.

Then install Debian: start the VM, "Graphical install", your language and keyboard, hostname of your choice, a user (the scripts assume you work as that user, in its home), no root password (then the user gets sudo, which the scripts need), guided partitioning on the whole disk, and in the software selection tick "Debian desktop environment" and "Xfce" (the desktop icons support Xfce and GNOME; Xfce is what the existing VM runs), plus "standard system utilities". Reboot into the desktop. `UNATTENDED=1 bash vm-create.sh ...` lets VirtualBox do the install by itself instead (user `tk`, password `kmotion`, Debian's default desktop).

The Guest Additions (the shared clipboard and folder, drag and drop, a display that follows the window) are installed by `install-vm.sh` from VirtualBox's own disc, which the VM script attaches as a second drive (`--medium additions`). A VM made by hand gets the disc with Devices > Insert Guest Additions CD Image before the script runs; by hand the install is `sudo sh /media/cdrom/VBoxLinuxAdditions.run` after step 3 (it needs `build-essential linux-headers-amd64 dkms`, which the script installs).

## 3. Everything else: `install-vm.sh`

In the VM, in a terminal:

    curl -fsSL https://raw.githubusercontent.com/TomKerekes/KMotionX/master/KMotionX/linuxcnc/install/install-vm.sh -o install-vm.sh
    bash install-vm.sh

(`curl` is on a fresh Debian; or copy the file in through the shared folder.) It asks for your password at its `sudo` steps and does, in order:

1. Turns on the `deb-src` sources (LinuxCNC's build dependencies come from `apt build-dep`) and installs the packages (LinuxCNC's, `libftdi1-dev` for KMotionX, PySide6 for the KMotion tool, matplotlib for the MotionLogPlotter, the kernel headers for the Guest Additions).
2. Clones KMotionX (`TomKerekes/KMotionX`, `master`) to `~/KMotionX`, `./configure`, `make` (plain, never `-j`), `sudo make install` (the libraries to `/usr/local/lib/kmx`, KMotionServer, tcc67 and the firmware images to `~/.kmotionx`), `make` once more as yourself (the install leaves `bin/` root-owned), `chown` of `~/.kmotionx`, the KFLOP udev rule, and adds you to `plugdev`, `dialout` and `vboxsf`.
3. Clones LinuxCNC to `~/linuxcnc-dev`, checks out the pinned commit `514be4f657`, applies the patches, `apt build-dep`, `./autogen.sh && ./configure --with-realtime=uspace --disable-build-documentation`, `make -j$(nproc)`. No `make setuid`: not needed.
4. Builds `kmotion-motion` and runs its tests (`make test`: the planner in simulate mode, every check prints ok or FAIL).
5. Installs the VirtualBox Guest Additions from their disc when the machine is a VM and they are not in yet (skipped otherwise), builds the KMotion tool's C shim and installs the desktop icons (KMotion, LinuxCNC on the Kogna, on the 3 Link, in simulate mode).
6. With `KOGNA_IF=<interface> bash install-vm.sh`, makes the DHCP profile for the bridged interface (`ip link` shows the names; the one without the internet). Without it, the command to do it by hand is printed.

It can be run again at any time: each step skips what is already done, and after a `git pull` in `~/KMotionX` it rebuilds what changed (if a patch in `patches/` changed, `git -C ~/linuxcnc-dev checkout -- src lib` first, then run it again). `--dry-run` prints the commands without running anything; `--no-linuxcnc` and `--no-icons` leave those out. Log out and in once afterwards for the groups.

## 4. The board

- **Kogna**: with the bridged adapter's cable connected (Devices > Network > Connect Network Adapter - the VM's virtual cable, not the physical one), the interface gets an address within seconds (`ip -4 addr`: 192.168.113.x from a Kogna that is its own DHCP server, the router's addresses with the Kogna on a router's network). KMotionServer finds the board by itself; `printf 'Version\n' | ~/KMotionX/bin/kflopConsole` prints its firmware build. If a KMotionServer was started before the link existed, `killall KMotionServer` and let the next client start a fresh one.
- **KFLOP**: plug it in; with the USB filter the VM takes it (Devices > USB shows it ticked) and the udev rule makes it accessible. Do not have Windows KMotion hold it at the same time.
- **Firmware**: the build stamp the board reports must be the one in `~/.kmotionx/DSP_KOGNA/DSPKOGNA.out` (or `DSP_KFLOP/`). Otherwise LinuxCNC starts but Machine on is refused with a message naming the board's build and the file's; flash the board from the KMotion tool (desktop icon KMotion > Config & Flash > the .out image), then press Machine on again - no restart needed.

## 5. First runs

1. Desktop icon **LinuxCNC (KMotion simulate)**: AXIS with the planner and no board. Load a program and run it; this proves LinuxCNC and the planner.
2. **KMotion** (the tool): the board's console, I/O, axes, config: proves the board link.
3. **LinuxCNC (Kogna)**: Tom's bench configuration (`configs/kmotion-kogna`: step/dir channels 0-2, `MinirouterInit.c`, homing through `Home.c` with simulated switches). The GUI is gmoccapy. **LinuxCNC (3 Link)** is the pen robot.
4. From a terminal instead: `source ~/linuxcnc-dev/scripts/rip-environment`, then `linuxcnc -v -d <ini>`; the motion module's log is `/tmp/kmotion-motion.log`.

For your own machine, copy `configs/kmotion-kogna` and change the channels, the init program, the scales and limits, the homing and the spindle (the ini's comments say what each key does; `README.md` the details); a copy of a LinuxCNC launcher with the new ini path makes its icon.

## 6. Things that went wrong on the existing VM, so you recognise them

- `make -j` in `~/KMotionX` stops partway: the subdirectories race. Plain `make`.
- After `sudo make install` the files in `~/KMotionX/bin` are root's: `make` once more as yourself (the script does).
- `apt build-dep` needs `deb-src` sources (the script turns them on).
- "realtime scheduling unavailable ... Falling back to POSIX non-realtime" when LinuxCNC starts: normal and harmless here.
- A LinuxCNC that was killed leaves `/tmp/linuxcnc.lock` and HAL components behind: the next start says "LinuxCNC is still running. Restart it?" - answer Y, or `halrun -U` and remove the lock. Quit from the GUI, not with Ctrl-C in a pipeline.
- The bridged interface shows no carrier: the VM's virtual cable is disconnected (Devices > Network), not the real one. NetworkManager made no profile for it: `nmcli connection add type ethernet ifname <IF> con-name Kogna ipv4.method auto autoconnect yes`.
- A KFLOP on USB gets captured by the VM as soon as Windows releases it; KMotionServer numbers boards as it finds them, so untick it (Devices > USB) when the Kogna is the target.
- The shared folder is unreadable until you are in group `vboxsf` and have logged in again.
- KMotionServer stopped answering after a LinuxCNC session ended badly: `killall KMotionServer`; the next client respawns it.
