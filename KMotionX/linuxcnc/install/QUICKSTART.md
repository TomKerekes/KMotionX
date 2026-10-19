# Quick start: LinuxCNC on KMotion in a VirtualBox VM, step by step

Only the steps, for a Windows PC. `INSTALL.md` says why and what each one does.

## On the Windows PC

1. Make a folder `C:\InstallLinuxCNC`.
2. If VirtualBox is not installed: open https://www.virtualbox.org/wiki/Downloads, click **Windows hosts**, run the downloaded installer, keep every default.
3. Open https://www.debian.org/download, click the **debian-13.x.x-amd64-netinst.iso** link, save the file to `C:\InstallLinuxCNC`. Note its exact name: the version number changes (13.7.0 in October 2026).
4. Open PowerShell: Start, type `powershell`, Enter. Fetch the VM script into the folder (Windows has `curl.exe` built in):

       cd C:\InstallLinuxCNC
       curl.exe -fsSL -o vm-create.cmd https://raw.githubusercontent.com/TomKerekes/KMotionX/master/KMotionX/linuxcnc/install/vm-create.cmd

   (Saving it from the browser works too, but the browser tends to add `.txt`; if that happened, `Rename-Item vm-create.cmd.txt vm-create.cmd`.)
5. Find the name of the adapter the Kogna is plugged into (with a KFLOP, skip this step). Power the Kogna and connect its cable to the PC, then:

       ipconfig /all

   The PC's adapters are listed one by one, each with a `Description` line and, when connected, an `IPv4 Address` line. The Kogna gives the adapter it is plugged into an address starting `192.168.113.` (the Kogna is a DHCP server on that cable), so look for that address: the `Description` line of that adapter is the name to use, for example `Intel(R) Ethernet Controller I226-V #2`. The internet adapter (Wi-Fi, or another Ethernet port) is not it; the VM reaches the internet through the PC on its own.
   The same names appear after `Name:` in `& "C:\Program Files\Oracle\VirtualBox\VBoxManage.exe" list bridgedifs`; the `VBoxNetworkName:` lines there, with their `HostInterfaceNetworking-` prefix, are not accepted.
   If the name is wrong, the script makes nothing and prints the accepted names; run it again with one of those.
6. Make the VM (one line; the ISO name exactly as downloaded, the adapter name in quotes, or leave it out for a KFLOP):

       .\vm-create.cmd C:\InstallLinuxCNC\debian-13.7.0-amd64-netinst.iso "Intel(R) Ethernet Controller I226-V #2"

   The script stops with a message if the ISO or the adapter name is wrong, before it makes anything.

7. Start it:

       & "C:\Program Files\Oracle\VirtualBox\VBoxManage.exe" startvm KMotion-LinuxCNC

## In the VM: Debian

8. In the installer menu choose **Graphical install**. Then, screen by screen:
   - Language, location, keyboard: yours.
   - Primary network interface (asked when the VM has the Kogna adapter): **enp0s3**. Inside the VM, Debian names the VM's two virtual network cards by their slot, the same on every PC: enp0s3 is VirtualBox adapter 1, the NAT one with the internet (through whatever the PC uses, Wi-Fi included); enp0s8 is adapter 2, the VM's own link to the Kogna over the Ethernet port named in step 6. The Windows adapter names play no part inside the VM.
   - Hostname: `kmotion`. Domain name: leave empty.
   - Root password: leave both fields empty (your user then gets `sudo`, which the install script needs).
   - Full name, username (for example `tk`), password: yours. Remember them.
   - Partition disks: **Guided - use entire disk**, the only disk, **All files in one partition**, **Finish partitioning**, then **Yes** to write the changes.
   - Package manager: the suggested mirror; proxy: empty.
   - Participate in the package usage survey: No.
   - Software selection: tick **Debian desktop environment**, **Xfce** and **standard system utilities**; untick **GNOME**. Continue.
   - Install GRUB: **Yes**, choose `/dev/sda`.
   - Finish: Continue. The VM reboots into the login screen.
9. Log in. Open a terminal: Applications menu (top left) > Terminal Emulator.

## In the VM: everything else

10. In the terminal, type these two lines (the second asks for your password; it takes 30 to 60 minutes and keeps a log in `install.log`):

        curl -fsSL https://raw.githubusercontent.com/TomKerekes/KMotionX/master/KMotionX/linuxcnc/install/install-vm.sh -o install-vm.sh
        bash install-vm.sh 2>&1 | tee install.log

    It ends with a summary. If it stops with an error, send `install.log`.
11. Log out (top right menu) and log in again.

## The board

12. Kogna: in the VM window's menu, Devices > Network > Connect Network Adapter (adapter 2); the Kogna link is up within seconds. KFLOP: plug it in; Devices > USB shows it ticked.
13. The board's firmware must be the one this tree carries. If Machine on is refused with a message naming two builds: desktop icon **KMotion**, Config & Flash, flash `~/.kmotionx/DSP_KOGNA/DSPKOGNA.out` (or `DSP_KFLOP/DSPKFLOP.out`), then press Machine on again.

## First run

14. Double-click the desktop icon **LinuxCNC (KMotion simulate)**: AXIS starts with the planner and no board. Load a program and run it.
15. Then **LinuxCNC (Kogna)** or **LinuxCNC (3 Link)** with the board connected.

Optional, for a shared folder and a resizable display: Devices > Insert Guest Additions CD, then in a terminal `sudo sh /media/cdrom/VBoxLinuxAdditions.run`, `sudo usermod -aG vboxsf $USER`, log out and in.
