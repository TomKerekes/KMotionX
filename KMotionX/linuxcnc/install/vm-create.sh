#!/bin/bash
# vm-create.sh - makes the VirtualBox VM for LinuxCNC on KMotion, on a Linux or macOS host
# (vm-create.cmd is the same for Windows). Needs VirtualBox 7 (VBoxManage on the PATH) and
# the Debian 13 netinst ISO. Usage:
#     bash vm-create.sh <debian-13 iso> [bridged host interface] [shared folder]
# e.g. bash vm-create.sh ~/Downloads/debian-13.1.0-amd64-netinst.iso "enp3s0" ~/kmotion-share
# Settings (environment): VM_NAME (KMotion-LinuxCNC), VM_RAM_MB (4096), VM_CPUS (2),
# VM_DISK_MB (40000), VM_DIR (VirtualBox's default). Then start the VM and install Debian
# (INSTALL.md, step 2) - or UNATTENDED=1 lets VirtualBox install it (user "tk", password
# "kmotion"; Debian's default desktop).
set -euo pipefail
ISO=${1:?the Debian 13 netinst ISO}
BRIDGE=${2:-}
SHARE=${3:-}
NAME=${VM_NAME:-KMotion-LinuxCNC}
RAM=${VM_RAM_MB:-4096}; CPUS=${VM_CPUS:-2}; DISK=${VM_DISK_MB:-40000}
[ -f "$ISO" ] || { echo "no such ISO: $ISO"; exit 2; }
if [ -z "$BRIDGE" ]; then echo "host interfaces for the bridge to the Kogna (give one as the 2nd argument, or none for now):"; VBoxManage list bridgedifs | grep '^Name:'; fi

VBoxManage createvm --name "$NAME" --ostype Debian_64 --register ${VM_DIR:+--basefolder "$VM_DIR"}
VMDIR=$(VBoxManage showvminfo "$NAME" --machinereadable | sed -n 's/^CfgFile="\(.*\)\/[^/]*"$/\1/p')
VBoxManage modifyvm "$NAME" --memory "$RAM" --cpus "$CPUS" --vram 64 --graphicscontroller vmsvga \
    --usb on --usbohci on --clipboard-mode bidirectional --draganddrop bidirectional --audio-enabled off \
    --nic1 nat
if [ -n "$BRIDGE" ]; then
    # adapter 2 bridged to the interface the Kogna is on: the Kogna is its own DHCP server
    VBoxManage modifyvm "$NAME" --nic2 bridged --bridgeadapter2 "$BRIDGE" --cableconnected2 on
fi
VBoxManage createmedium disk --filename "$VMDIR/$NAME.vdi" --size "$DISK"
VBoxManage storagectl "$NAME" --name SATA --add sata --controller IntelAhci
VBoxManage storageattach "$NAME" --storagectl SATA --port 0 --device 0 --type hdd --medium "$VMDIR/$NAME.vdi"
VBoxManage storageattach "$NAME" --storagectl SATA --port 1 --device 0 --type dvddrive --medium "$ISO"
# VirtualBox's Guest Additions disc in a second drive: install-vm.sh installs them (shared clipboard, display).
# The file ships with VirtualBox (its place differs by host); an install without it only loses the automatic step
GAISO=""
for f in /usr/share/virtualbox/VBoxGuestAdditions.iso /usr/lib/virtualbox/additions/VBoxGuestAdditions.iso \
         /opt/VirtualBox/additions/VBoxGuestAdditions.iso /Applications/VirtualBox.app/Contents/MacOS/VBoxGuestAdditions.iso; do
    [ -f "$f" ] && { GAISO=$f; break; }
done
if [ -n "$GAISO" ]; then
    VBoxManage storageattach "$NAME" --storagectl SATA --port 2 --device 0 --type dvddrive --medium "$GAISO"
else
    echo "note: no VBoxGuestAdditions.iso found - insert it later from the VM window: Devices > Insert Guest Additions CD Image"
fi
# a KFLOP on USB (FTDI 0403:6001, a full-speed device: the OHCI controller serves it) goes to the VM when it is plugged in
VBoxManage usbfilter add 0 --target "$NAME" --name KFLOP --vendorid 0403 --productid 6001
if [ -n "$SHARE" ]; then
    mkdir -p "$SHARE"
    VBoxManage sharedfolder add "$NAME" --name kmotion --hostpath "$SHARE" --automount --auto-mount-point /mnt/kmotion
fi
if [ "${UNATTENDED:-0}" = 1 ]; then
    VBoxManage unattended install "$NAME" --iso="$ISO" --user=tk --password=kmotion --full-user-name="KMotion" \
        --hostname=kmotion.local --time-zone=UTC --install-additions --start-vm=gui
    echo "Debian is installing by itself (user tk, password kmotion); when it is done, log in and go on with INSTALL.md step 3"
else
    echo "VM '$NAME' made. Start it and install Debian 13 (INSTALL.md, step 2):"
    echo "    VBoxManage startvm '$NAME'"
fi
