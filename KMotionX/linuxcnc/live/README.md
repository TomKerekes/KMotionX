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
Guest Additions are built into the image from Debian's `virtualbox-guest-additions-iso` for the image's
kernel (shared clipboard, window-sized display); `vm-test-iso.cmd` gives the test VM a USB-tablet mouse. Real hardware is unaffected.
