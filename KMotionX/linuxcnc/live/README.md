# live/: the bootable image

`sudo bash build.sh` makes a Debian 13 live ISO (Xfce) with LinuxCNC on KMotion preinstalled, by
live-build: `kmotion.list.chroot` is the image's own package list, `0100-kmotion.hook.chroot` runs
inside the image at build time - it creates the user `cnc` and runs the repository's own
`install/install-vm.sh --image`, so the image is exactly a VM install (minus the VM-only steps:
Guest Additions, the Kogna network profile, the host kernel's headers). Boot it as a live system
(user cnc, password cnc) or choose "Install" in the boot menu; name the user `cnc` there, since the
paths of the install are in /home/cnc. The build takes about 40 minutes on the VM and
22 GB in `~/live` (the chroot alone is 14 GB; the first run on a 38 GB disk failed writing the ISO:
`finish-iso.sh` completes such a build, writing the ISO onto the shared folder through a bind mount
and removing the chroot). The ISO is 4.7 GB - a USB stick, not a DVD. `vm-test-iso.cmd` on the
Windows host boots it in a throwaway VirtualBox VM. `~/live/build.log` has everything.
KMotionXCNC (the web CNC application) is built into the image by install-vm.sh's KMotionXCNC step
(Node, an npm build) and has its own icon. Known: the module's `make test` prints nothing inside
the chroot (to be looked at); the disk installer must be given the user name `cnc`; in a VirtualBox VM the
Guest Additions' programs (VBoxService, VBoxClient) are in the image, from Debian's `virtualbox-guest-additions-iso`,
on the kernel's own vboxguest module (shared clipboard, window-sized display); `vm-test-iso.cmd` gives the test VM a USB-tablet mouse. Real hardware is unaffected.

**For users: [LIVE-USB.md](LIVE-USB.md)** - download, writing the stick, starting from it, keeping
changes, installing to disk. The stick keeps changes through live-boot's persistence: the boot line
carries `persistence` (build.sh's `--bootappend-live`), and the desktop's "Keep Changes on USB"
(`desktop/usb-persistence`, a launcher only in the image) adds an ext4 volume labelled `persistence`
with `/ union` in the stick's unused space behind the ISO image (MBR entry 3; the ISO's own
entries are 1, type 0, and 2, the EFI image inside it). `vm-test-stick.cmd` tries exactly that in
VirtualBox: the ISO converted to a virtual disk and enlarged, the VM started from the disk.
`install-vm.sh` treats a live session (`boot=live`) as `--image`, so "Re-install LinuxCNC" works on
a stick that keeps changes.

A machine is switched off at the mains, and a stick that keeps changes must stand that: the image
writes data out after 2 s instead of 30 (`90-kmotion-live.conf`, a sysctl file) and commits the
volume's journal every second (`kmotion-persistence-commit.service`), both installed by
`install-vm.sh` in image mode; the KMotion tool saves a C program through a flushed temporary file
renamed over the old one. (First trial: a C program saved by All, the VM powered off seconds
later, came back as an empty file.)

**Publishing**: the build leaves `kmotion-linuxcnc-<date>.iso` and `.iso.sha256` on the shared
folder; both go to https://dynomotion.com/Software/LinuxCNC/ (LIVE-USB.md names that address). The
ISO is over 4 GB: the web server must serve files that large (and should support resumed downloads).
