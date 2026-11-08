# live/: the bootable image

`sudo bash build.sh` makes a Debian 13 live ISO (Xfce) with LinuxCNC on KMotion preinstalled, by
live-build: `kmotion.list.chroot` is the image's own package list, `0100-kmotion.hook.chroot` runs
inside the image at build time - it creates the user `cnc` and runs the repository's own
`install/install-vm.sh --image`, so the image is exactly a VM install (minus the VM-only steps:
Guest Additions, the Kogna network profile, the host kernel's headers). Boot it as a live system
(user cnc, password cnc) or choose "Install" in the boot menu; name the user `cnc` there, since the
paths of the install are in /home/cnc. The build takes an hour or more and 10-15 GB in `~/live`;
`~/live/build.log` has everything. Not in the image yet: KMotionXCNC (the web UI).
