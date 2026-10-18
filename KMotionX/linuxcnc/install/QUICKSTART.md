# Quick start: LinuxCNC on KMotion in a VirtualBox VM, step by step

Only the steps, for a Windows PC. `INSTALL.md` says why and what each one does.

## On the Windows PC

1. Make a folder `C:\InstallLinuxCNC`.
2. If VirtualBox is not installed: open https://www.virtualbox.org/wiki/Downloads, click **Windows hosts**, run the downloaded installer, keep every default.
3. Open https://www.debian.org/download, click the **debian-13.x.x-amd64-netinst.iso** link, save the file to `C:\InstallLinuxCNC`.
4. Open https://raw.githubusercontent.com/TomKerekes/KMotionX/master/KMotionX/linuxcnc/install/vm-create.cmd in the browser, press Ctrl+S, save it as `vm-create.cmd` in `C:\InstallLinuxCNC` (file type "All files", so the browser does not add `.txt`).
5. Open PowerShell: Start, type `powershell`, Enter. Then:

       cd C:\InstallLinuxCNC
       & "C:\Program Files\Oracle\VirtualBox\VBoxManage.exe" list bridgedifs

   In the output, note the `Name:` of the network adapter that connects to the Kogna (the USB-Ethernet adapter). With a KFLOP there is none to note.
6. Make the VM (one line; the ISO name as downloaded, the adapter name in quotes, or leave it out for a KFLOP):

       .\vm-create.cmd C:\InstallLinuxCNC\debian-13.1.0-amd64-netinst.iso "Realtek USB GbE Family Controller"

7. Start it:

       & "C:\Program Files\Oracle\VirtualBox\VBoxManage.exe" startvm KMotion-LinuxCNC

## In the VM: Debian

8. In the installer menu choose **Graphical install**. Then, screen by screen:
   - Language, location, keyboard: yours.
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
