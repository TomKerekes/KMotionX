#!/bin/bash
# finish-iso.sh - write the ISO again when build.sh got everything built and ran out of disk at the
# last step (xorriso: "Image size ... exceeds free space"). Frees what can go (live-build's package
# cache, the host's apt cache), reruns only the ISO stage, and moves the result to the shared
# folder (or the folder given), then removes the 14 GB chroot.
#
#     sudo bash finish-iso.sh [work folder] [destination folder]
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo: sudo bash $0"; exit 2; }
owner=${SUDO_USER:-root}
WORK=${1:-$(getent passwd "$owner" | cut -d: -f6)/live}
DEST=${2:-}
if [ -z "$DEST" ]; then for d in /mnt/kmotion "$(getent passwd "$owner" | cut -d: -f6)"; do [ -d "$d" ] && [ -w "$d" ] && { DEST=$d; break; }; done; fi
cd "$WORK"
exec > >(tee -a build.log) 2>&1
echo "==== $(date) finish-iso in $WORK, destination $DEST"
rm -rf cache/packages.* 2>/dev/null || true
apt-get clean
df -h "$WORK" | tail -1
rm -f live-image-amd64.hybrid.iso
lb binary_iso
iso="kmotion-linuxcnc-$(date +%Y%m%d).iso"
mv -f live-image-amd64.hybrid.iso "$DEST/$iso"
chown "$owner": "$DEST/$iso" 2>/dev/null || true
echo "==== $(date) done: $DEST/$iso ($(du -h "$DEST/$iso" | cut -f1))"
echo "==== removing the chroot (14 GB); the cache of downloaded packages is gone too, a rebuild downloads again"
lb clean --chroot 2>/dev/null || rm -rf chroot
df -h "$WORK" | tail -1
