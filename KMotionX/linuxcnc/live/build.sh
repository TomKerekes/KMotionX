#!/bin/bash
# build.sh - the bootable live image (Debian 13 live, Xfce, installable) with LinuxCNC on KMotion
# preinstalled: KMotionX, kmotion-motion, LinuxCNC, the KMotion tool, the configurations, the
# desktop icons - everything install-vm.sh puts on a VM, built into the image by live-build.
#
#     sudo bash build.sh [work folder]        (default: ~/live of the user who ran sudo)
#
# Needs root (debootstrap, chroots), the network, 10-15 GB free in the work folder, and an hour or
# more: LinuxCNC and KMotionX are built inside the image. The result: <work>/kmotion-linuxcnc-<date>.iso,
# an iso-hybrid for a DVD or a USB stick (dd, Rufus in dd mode, balenaEtcher). It boots to the Xfce
# desktop as user "cnc" (password cnc, sudo without a password) with the icons, or installs to disk
# ("Install" in the boot menu: name the user cnc - every path in the install is in /home/cnc).
# The log: <work>/build.log. A second run reuses the downloaded packages (the cache) and starts
# the image afresh.
set -euo pipefail
here=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
[ "$(id -u)" = 0 ] || { echo "run with sudo: sudo bash $0"; exit 2; }
owner=${SUDO_USER:-root}
WORK=${1:-$(getent passwd "$owner" | cut -d: -f6)/live}
mkdir -p "$WORK"
cd "$WORK"
exec > >(tee -a build.log) 2>&1
echo "==== $(date) live image build in $WORK"
command -v lb >/dev/null || { apt-get update -q; apt-get install -y -q live-build; }
df -h "$WORK" | tail -1
# the image is about 5 GB and the chroot 14 GB: 22 GB free to be safe (the first run on a 38 GB VM
# ran out at the ISO step; finish-iso.sh completes such a build)
free_gb=$(df -BG --output=avail "$WORK" | tail -1 | tr -dc 0-9)
[ "$free_gb" -ge 22 ] || echo "note: $free_gb GB free; the build wants about 22 GB - if the ISO step runs out, finish-iso.sh completes it"
# a fresh configuration each time (lb config is not idempotent across option changes); the
# cache/ of downloaded packages and the bootstrap stays
if [ -d config ]; then lb clean; rm -rf config; fi
lb config \
    --distribution trixie \
    --architectures amd64 \
    --binary-images iso-hybrid \
    --archive-areas "main contrib non-free non-free-firmware" \
    --debian-installer live \
    --debian-installer-gui true \
    --firmware-chroot false \
    --firmware-binary false \
    --memtest none \
    --bootappend-live "boot=live components username=cnc hostname=kmotion locales=en_US.UTF-8 keyboard-layouts=us" \
    --iso-application "LinuxCNC on KMotion" \
    --iso-volume "KMotion-LinuxCNC" \
    --iso-publisher "Dynomotion" \
    --iso-preparer "KMotionX live/build.sh"
mkdir -p config/package-lists config/hooks/live
cp "$here/kmotion.list.chroot" config/package-lists/
cp "$here/0100-kmotion.hook.chroot" config/hooks/live/
chmod +x config/hooks/live/0100-kmotion.hook.chroot
echo "==== $(date) lb build"
lb build
iso="kmotion-linuxcnc-$(date +%Y%m%d).iso"
mv -f live-image-amd64.hybrid.iso "$iso"
chown "$owner": "$iso" build.log 2>/dev/null || true
echo "==== $(date) done: $WORK/$iso ($(du -h "$iso" | cut -f1))"
if [ -d /mnt/kmotion ] && [ -w /mnt/kmotion ]; then mv -f "$iso" /mnt/kmotion/ && echo "moved to /mnt/kmotion/$iso (the shared folder)"; fi
echo "==== removing the chroot (14 GB); the package cache stays for the next build"
lb clean --chroot 2>/dev/null || rm -rf chroot
