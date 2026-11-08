#!/bin/bash
# finish-iso.sh - write the ISO when build.sh got everything built and failed at the last step
# (xorriso: "Image size ... exceeds free space"). live-build runs xorriso inside the chroot after
# moving the image tree in there, and moves it back only on success, so a failed run leaves
# chroot/binary and chroot/binary.sh behind. This restores that, then runs the very same xorriso
# command in the chroot with the ISO written onto the destination folder, bind-mounted into the
# chroot (the shared folder: the ISO never touches this VM's disk), and removes the 14 GB chroot.
#
#     sudo bash finish-iso.sh [work folder] [destination folder]
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo: sudo bash $0"; exit 2; }
owner=${SUDO_USER:-root}
home=$(getent passwd "$owner" | cut -d: -f6)
WORK=${1:-$home/live}
DEST=${2:-}
if [ -z "$DEST" ]; then for d in /mnt/kmotion "$home"; do [ -d "$d" ] && [ -w "$d" ] && { DEST=$d; break; }; done; fi
cd "$WORK"
exec > >(tee -a build.log) 2>&1
iso="kmotion-linuxcnc-$(date +%Y%m%d).iso"
echo "==== $(date) finish-iso in $WORK -> $DEST/$iso"
# the state a failed ISO step leaves: the image tree in chroot/binary, the command in chroot/binary.sh
if [ -d chroot/binary ] && [ "$(ls -A chroot/binary 2>/dev/null)" ]; then
    echo "the image tree is in chroot/binary (left by the failed ISO step): using it there"
    rm -rf binary
elif [ -d binary ] && [ "$(ls -A binary)" ]; then
    mv binary chroot/binary
else
    echo "no image tree: run build.sh first"; exit 1
fi
[ -f chroot/binary.sh ] || { echo "no chroot/binary.sh (the xorriso command live-build composed): run build.sh"; exit 1; }
echo "--- the command live-build composed:"; cat chroot/binary.sh
# the output onto the destination, through a bind mount in the chroot
mkdir -p chroot/mnt/out
mountpoint -q chroot/mnt/out || mount --bind "$DEST" chroot/mnt/out
sed -e "s|-o [^ ]*\.iso|-o /mnt/out/$iso|" chroot/binary.sh > chroot/binary-out.sh
echo "--- running:"; grep -o '\-o [^ ]*' chroot/binary-out.sh
rm -f "$DEST/$iso"
chroot chroot sh binary-out.sh
chroot chroot sh -c "command -v implantisomd5 >/dev/null && implantisomd5 /mnt/out/$iso" || echo "(no md5 implant)"
umount chroot/mnt/out
rm -f chroot/binary-out.sh chroot/binary.sh
mv chroot/binary binary
chown "$owner": "$DEST/$iso" 2>/dev/null || true
echo "==== $(date) done: $DEST/$iso ($(du -h "$DEST/$iso" | cut -f1))"
size=$(stat -c %s "$DEST/$iso")
if [ "$size" -gt 3000000000 ]; then
    echo "==== removing the chroot (14 GB) and the binary tree; the package cache stays"
    rm -rf chroot binary
    df -h "$WORK" | tail -1
fi
