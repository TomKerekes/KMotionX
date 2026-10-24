#!/bin/bash
# install-vm.sh - LinuxCNC on KMotion, from a fresh Debian 13 (XFCE) to a working system:
# the packages, KMotionX (Tom's fork) built and installed, LinuxCNC's run-in-place tree at
# the pinned commit with the patches, kmotion-motion and its tests, the KMotion tool, the
# desktop icons, the board's network profile and USB access. Everything after the OS
# install; see INSTALL.md for the VM and the board.
#
# Run it as yourself in a terminal (sudo asks for your password a few times):
#     bash install-vm.sh
# It can be run again at any time: every step skips what is already done. Options:
#     --dry-run        print the commands, run nothing
#     --no-linuxcnc    skip the LinuxCNC tree (KMotionX and the tool only)
#     --no-icons       no desktop icons
# Settings (environment): KMOTIONX_REPO, KMOTIONX_BRANCH, KMOTIONX_DIR (~/KMotionX),
#     LINUXCNC_COMMIT (514be4f657), LINUXCNC_DIR (~/linuxcnc-dev),
#     KOGNA_IF (the bridged interface to the Kogna, e.g. enp0s8: a DHCP profile is made for it)
set -Eeuo pipefail   # -E: the ERR message below also fires for a command run inside a function

# everything also goes to ~/install.log (appended: a rerun follows the first run), so the
# command to type is just "bash install-vm.sh" - a mistyped "| tee install-vm.sh" once
# overwrote the script with its own output
if [ -z "${INSTALL_VM_LOGGING:-}" ]; then
    export INSTALL_VM_LOGGING=1
    echo "(the output is also kept in $HOME/install.log)"
    exec > >(tee -a "$HOME/install.log") 2>&1
fi

KMOTIONX_REPO=${KMOTIONX_REPO:-https://github.com/TomKerekes/KMotionX.git}
KMOTIONX_BRANCH=${KMOTIONX_BRANCH:-master}
KMOTIONX_DIR=${KMOTIONX_DIR:-$HOME/KMotionX}
LINUXCNC_REPO=${LINUXCNC_REPO:-https://github.com/LinuxCNC/linuxcnc.git}
LINUXCNC_COMMIT=${LINUXCNC_COMMIT:-514be4f657}
LINUXCNC_DIR=${LINUXCNC_DIR:-$HOME/linuxcnc-dev}
KOGNA_IF=${KOGNA_IF:-}
DRY=0; DO_LINUXCNC=1; DO_ICONS=1
for a in "$@"; do
    case $a in
        --dry-run) DRY=1;;
        --no-linuxcnc) DO_LINUXCNC=0;;
        --no-icons) DO_ICONS=0;;
        -h|--help) sed -n '2,20p' "$0"; exit 0;;
        *) echo "unknown option $a"; exit 2;;
    esac
done
run() { echo "+ $*"; [ $DRY = 1 ] || "$@"; }
# apt-get runs with DEBIAN_FRONTEND=noninteractive: a package's configuration question (which
# login manager when GNOME's and Xfce's are both in, say) keeps the current choice instead of
# stopping the install for an answer
# a failed command stops the script (set -e): say so plainly, the output may be going through tee
trap 'echo; echo "*** install-vm.sh STOPPED: the command above failed (line $LINENO). Fix what it says and run"; echo "*** the script again: the steps already done are skipped."' ERR
step() { echo; echo "==== $*"; }
note() { echo "   - $*"; }

# the packages: LinuxCNC's build needs (its own apt build-dep adds the rest), KMotionX's
# (libftdi1-dev), the KMotion tool's (PySide6), the Guest Additions' (headers, dkms)
PACKAGES="git build-essential autoconf automake libtool pkg-config dh-python desktop-file-utils
intltool bwidget libboost-python-dev libepoxy-dev libgl-dev libglu1-mesa-dev libgtk-3-dev
libcap-dev libmodbus-dev libgpiod-dev libeditreadline-dev libtirpc-dev libudev-dev
libusb-1.0-0-dev libxmu-dev libfmt-dev netcat-openbsd procps psmisc python3 python3-dev
python3-pybind11 python3-tk python3-xlib python3-opengl python3-numpy python3-matplotlib python3-configobj
python3-cairo python3-gi python3-gi-cairo gir1.2-gtk-3.0 gir1.2-gtksource-4 python3-pyqt5
python3-pyqt5.qsci python3-pyqt5.qtsvg python3-pyqt5.qtopengl python3-dbus python3-qtpy
python3-zmq tcl tcl8.6-dev tclx tk8.6-dev yapps2 libftdi1-dev python3-pyside6.qtwidgets
python3-pyside6.qtgui python3-pyside6.qtcore network-manager linux-headers-amd64 dkms tio"

step "Debian"
. /etc/os-release
if [ "${VERSION_ID:-}" != "13" ]; then
    echo "this is $PRETTY_NAME, not Debian 13: the package list is for 13, continuing anyway"
fi
if [ "$(id -u)" = 0 ]; then echo "run this as yourself, not as root (it uses sudo where needed)"; exit 2; fi

step "apt sources: deb-src (LinuxCNC's build dependencies come from 'apt build-dep')"
if [ -f /etc/apt/sources.list.d/debian.sources ]; then
    if ! grep -q '^Types:.*deb-src' /etc/apt/sources.list.d/debian.sources; then
        run sudo sed -i 's/^Types: deb$/Types: deb deb-src/' /etc/apt/sources.list.d/debian.sources
    else note "already on"; fi
elif [ -f /etc/apt/sources.list ] && ! grep -q '^deb-src' /etc/apt/sources.list; then
    run sudo sh -c "grep '^deb ' /etc/apt/sources.list | sed 's/^deb /deb-src /' >> /etc/apt/sources.list"
else note "already on"; fi
run sudo DEBIAN_FRONTEND=noninteractive apt-get update -q

step "packages"
# shellcheck disable=SC2086
run sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q $PACKAGES
# brltty (the Braille display daemon every Debian desktop brings) claims FTDI 0403:6001 USB
# devices - some Braille displays use that chip - and takes the Kogna's COM port (an FT232R)
# away from the serial driver seconds after it appears; a KFLOP would lose its port the same way
if dpkg -s brltty >/dev/null 2>&1; then run sudo DEBIAN_FRONTEND=noninteractive apt-get purge -y -q brltty
else note "brltty not installed"; fi

step "KMotionX: $KMOTIONX_REPO ($KMOTIONX_BRANCH) in $KMOTIONX_DIR"
if [ ! -d "$KMOTIONX_DIR/.git" ]; then
    run git clone -b "$KMOTIONX_BRANCH" "$KMOTIONX_REPO" "$KMOTIONX_DIR"
else note "already cloned ($(git -C "$KMOTIONX_DIR" log -1 --format=%h 2>/dev/null)); not pulled - git pull yourself when you want the latest"; fi
if [ $DRY = 0 ]; then cd "$KMOTIONX_DIR"; fi
[ -f "$KMOTIONX_DIR/config.mak" ] || run bash -c "cd '$KMOTIONX_DIR' && ./configure"
# plain make, never -j: the subdirectory builds race (see the build notes)
run make -C "$KMOTIONX_DIR"
# a tree built before the runpath fix (include.mk: realpath of an install dir that did not
# exist yet gave the binaries no runpath, and KMotionServer could not find libKMotionX.so):
# relink everything
if [ -x "$KMOTIONX_DIR/bin/KMotionServer" ] && ! readelf -d "$KMOTIONX_DIR/bin/KMotionServer" 2>/dev/null | grep -q 'RUNPATH\|RPATH'; then
    note "the binaries have no library runpath (an older build): relinking"
    run make -C "$KMOTIONX_DIR" clean
    run make -C "$KMOTIONX_DIR"
fi
run sudo make -C "$KMOTIONX_DIR" install
# the install relinks bin/ as root: one more make as yourself makes the files yours again
run make -C "$KMOTIONX_DIR"
run sudo chown -R "$USER:" "$HOME/.kmotionx"
if [ ! -f /etc/udev/rules.d/90-kflop.rules ]; then
    # (90-: after Debian's 50-udev-default, which overrode the old 10.kflop.rules' mode)
    run sudo cp "$KMOTIONX_DIR/KMotionX/usb/etc/udev/rules.d/90-kflop.rules" /etc/udev/rules.d/
    [ -f /etc/udev/rules.d/10.kflop.rules ] && run sudo rm -f /etc/udev/rules.d/10.kflop.rules
    run sudo udevadm control --reload-rules
    run sudo udevadm trigger
else note "KFLOP udev rule already installed"; fi
if [ ! -f /etc/udev/rules.d/11.joystick.rules ]; then
    # a game pad readable by the logged-in user, for the configs' gamepad.hal
    run sudo cp "$KMOTIONX_DIR/KMotionX/usb/etc/udev/rules.d/11.joystick.rules" /etc/udev/rules.d/
    run sudo udevadm control --reload-rules
    run sudo udevadm trigger
else note "joystick udev rule already installed"; fi
for g in plugdev dialout vboxsf; do
    if getent group $g > /dev/null && ! id -nG | grep -qw $g; then run sudo usermod -aG $g "$USER"; GROUPS_CHANGED=1; fi
done

if [ $DO_LINUXCNC = 1 ]; then
    step "LinuxCNC run-in-place: $LINUXCNC_REPO at $LINUXCNC_COMMIT in $LINUXCNC_DIR"
    if [ ! -d "$LINUXCNC_DIR/.git" ]; then
        run git clone "$LINUXCNC_REPO" "$LINUXCNC_DIR"
    else note "already cloned"; fi
    run git -C "$LINUXCNC_DIR" checkout -q "$LINUXCNC_COMMIT"
    step "LinuxCNC patches ($KMOTIONX_DIR/KMotionX/linuxcnc/patches)"
    for p in "$KMOTIONX_DIR"/KMotionX/linuxcnc/patches/*.patch; do
        if git -C "$LINUXCNC_DIR" apply --reverse --check "$p" 2>/dev/null; then note "$(basename "$p"): already applied"
        else run git -C "$LINUXCNC_DIR" apply "$p"; fi
    done
    step "LinuxCNC build dependencies (apt build-dep; the documentation tools come along)"
    # apt build-dep reads the tree's debian/control, which LinuxCNC does not ship: its
    # debian/configure generates it (and lists what is missing, which build-dep then installs)
    if [ ! -f "$LINUXCNC_DIR/debian/control" ]; then
        run "$LINUXCNC_DIR/debian/configure"
    else note "debian/control exists"; fi
    run sudo DEBIAN_FRONTEND=noninteractive apt-get build-dep -y -q "$LINUXCNC_DIR"
    step "LinuxCNC build (--with-realtime=uspace, no documentation; a while)"
    if [ ! -f "$LINUXCNC_DIR/src/Makefile.inc" ]; then
        run bash -c "cd '$LINUXCNC_DIR/src' && ./autogen.sh && ./configure --with-realtime=uspace --disable-build-documentation"
    else note "already configured"; fi
    run make -C "$LINUXCNC_DIR/src" -j"$(nproc)"
    note "no 'make setuid': kmotion-motion needs no real-time priority (the board does the real time)"

    step "kmotion-motion (LinuxCNC's motion module replacement) and its tests"
    run make -C "$KMOTIONX_DIR/KMotionX/linuxcnc/kmotion-motion" LINUXCNC_DIR="$LINUXCNC_DIR"
    run make -C "$KMOTIONX_DIR/KMotionX/linuxcnc/kmotion-motion" LINUXCNC_DIR="$LINUXCNC_DIR" test
fi

step "VirtualBox Guest Additions (shared clipboard, drag and drop, a resizable display)"
# from the disc vm-create attached (or one inserted by hand: Devices > Insert Guest Additions CD),
# wherever the desktop mounted it or not mounted at all; the packages step put the headers and dkms in
if ! grep -q '^flags.*hypervisor' /proc/cpuinfo 2>/dev/null; then note "not a VM: skipped"
elif command -v VBoxClient >/dev/null 2>&1; then note "already installed ($(VBoxClient --version 2>/dev/null || echo installed))"
else
    GA=""
    for d in /media/*/VBox_GAs* /media/cdrom* /run/media/*/VBox_GAs*; do [ -f "$d/VBoxLinuxAdditions.run" ] && { GA=$d; break; }; done
    if [ -z "$GA" ]; then
        for dev in /dev/sr0 /dev/sr1 /dev/sr2; do
            [ -b "$dev" ] || continue
            GAMNT=$(mktemp -d)
            if sudo mount -o ro "$dev" "$GAMNT" 2>/dev/null; then
                if [ -f "$GAMNT/VBoxLinuxAdditions.run" ]; then GA=$GAMNT; break; fi
                sudo umount "$GAMNT"
            fi
            rmdir "$GAMNT" 2>/dev/null
        done
    fi
    if [ -n "$GA" ]; then
        # the installer returns 2 when it only asks for a reboot: not a failure
        run sudo sh "$GA/VBoxLinuxAdditions.run" --nox11 || note "the Guest Additions installer ended with $? (a reboot wanted, or see /var/log/vboxadd-install.log)"
        case "$GA" in /tmp/*) sudo umount "$GA" 2>/dev/null; rmdir "$GA" 2>/dev/null;; esac
        note "in effect after a log out and in (a reboot for the display driver)"
        # the vboxsf group (a shared folder's readers) exists only now: the groups step ran before
        if getent group vboxsf > /dev/null && ! id -nG | grep -qw vboxsf; then run sudo usermod -aG vboxsf "$USER"; GROUPS_CHANGED=1; fi
    else
        note "no Guest Additions disc found: in the VM window, Devices > Insert Guest Additions CD Image, then run this script again"
    fi
fi

step "the KMotion tool (tools/kmotion)"
run make -C "$KMOTIONX_DIR/KMotionX/tools/kmotion/shim"

if [ $DO_ICONS = 1 ]; then
    step "desktop icons"
    run env LINUXCNC_DIR="$LINUXCNC_DIR" "$KMOTIONX_DIR/KMotionX/desktop/install-desktop-icons.sh"
    # GNOME shows nothing on the desktop without its desktop-icons extension (Xfce needs none);
    # the launchers are in its applications menu either way
    if [ -d /usr/share/gnome-shell ]; then
        run sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q gnome-shell-extension-desktop-icons-ng
        run gnome-extensions enable ding@rastersoft.com || note "the desktop-icons extension is enabled at the next login"
        note "GNOME: the launchers are in the applications menu (the Windows key, type LinuxCNC); on the desktop after a log out and in"
    fi
fi

step "the Kogna's network"
# KOGNA_IF not given: a wired interface without an IPv4 address is one nothing manages yet -
# the bridge to a Kogna that is its own DHCP server (NetworkManager makes no profile for a link
# that answers no DHCP at install time); an interface that has an address (the internet's, or
# the bridge on a router's network where the Kogna takes the router's address) needs nothing
if [ -z "$KOGNA_IF" ]; then
    for i in /sys/class/net/*; do
        i=$(basename "$i")
        [ "$i" = lo ] && continue
        [ -d "/sys/class/net/$i/wireless" ] && continue
        [ -e "/sys/class/net/$i/device" ] || continue
        if ip -4 addr show "$i" 2>/dev/null | grep -q ' inet '; then note "$i has an address: managed already"; continue; fi
        KOGNA_IF=$i; note "the Kogna's interface: $i (wired, no address yet)"; break
    done
fi
if [ -n "$KOGNA_IF" ]; then
    if nmcli -t -f NAME connection show 2>/dev/null | grep -qx Kogna; then note "profile 'Kogna' exists"
    else
        run nmcli connection add type ethernet ifname "$KOGNA_IF" con-name Kogna ipv4.method auto ipv6.method disabled autoconnect yes
        run nmcli connection up Kogna
    fi
else
    note "every wired interface has an address: nothing to do (a Kogna on a router's network is reached through the interface on that network)"
fi

step "done"
cat <<END
Next:
  - log out and in once if groups were added (${GROUPS_CHANGED:-no change}: plugdev/dialout/vboxsf)
  - the board must run the firmware this tree carries (DSP_KOGNA/DSPKOGNA.out, DSP_KFLOP/DSPKFLOP.out):
    kmotion-motion refuses a mismatch at start; the KMotion tool's Config & Flash screen flashes it
  - first run: the desktop icon "LinuxCNC (KMotion simulate)" (no board), then "LinuxCNC (Kogna)"
    or from a terminal:  source $LINUXCNC_DIR/scripts/rip-environment
                         linuxcnc $KMOTIONX_DIR/KMotionX/linuxcnc/configs/kmotion-sim/kmotion-sim.ini
  - later updates: git -C $KMOTIONX_DIR pull, then run this script again (it rebuilds what changed)
END
