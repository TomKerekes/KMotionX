# Quick start: LinuxCNC on KMotion in a VirtualBox VM, step by step

Only the steps, for a Windows PC. `INSTALL.md` says why and what each one does.

## On the Windows PC

Nothing else is installed on Windows: no Python, no compiler. Everything else goes into the VM in step 11.

1. Check that the processor's virtualization is on, which VirtualBox needs: Task Manager (Ctrl+Shift+Esc), Performance tab, CPU; at the bottom right, "Virtualization: Enabled". If it says Disabled, it is switched off in the BIOS: restart, enter the BIOS setup (F1, F2, F10, F12 or Del while the PC starts, by maker), find "Intel Virtualization Technology" or "VT-x" (under Advanced > CPU, or on Lenovo under Security > Virtualization; a page that shows it in a different colour and skips it with the arrow keys only displays it, the switch is on the other page), set it to Enabled, save with F10. Without it the VM stops at once with "VT-x is disabled in the BIOS".
2. Make a folder `C:\InstallLinuxCNC`.
3. If VirtualBox is not installed: open https://www.virtualbox.org/wiki/Downloads, click **Windows hosts**, run the downloaded installer, keep every default.
4. Open https://www.debian.org/download, click the **debian-13.x.x-amd64-netinst.iso** link, save the file to `C:\InstallLinuxCNC`. Note its exact name: the version number changes (13.7.0 in October 2026).
5. Open PowerShell: Start, type `powershell`, Enter. Fetch the VM script into the folder (Windows has `curl.exe` built in):

       cd C:\InstallLinuxCNC
       curl.exe -fsSL -o vm-create.cmd https://raw.githubusercontent.com/TomKerekes/KMotionX/master/KMotionX/linuxcnc/install/vm-create.cmd

   (Saving it from the browser works too, but the browser tends to add `.txt`; if that happened, `Rename-Item vm-create.cmd.txt vm-create.cmd`.)
6. Find the name of the adapter the Kogna is plugged into (with a KFLOP, skip this step). Power the Kogna and connect its cable to the PC, then:

       ipconfig /all

   The PC's adapters are listed one by one, each with a `Description` line and, when connected, an `IPv4 Address` line. The one to use is the adapter on the same network as the Kogna, and that depends on how the Kogna gets its address (its IP setting):
   - **Kogna on its own cable** to a spare Ethernet port, IP setting 0, no router on that cable: the Kogna finds no DHCP server and becomes one, so that adapter gets an address starting `192.168.113.`. Use that adapter.
   - **Kogna on your network** (a switch or the router), IP setting 0: the router's DHCP server gives the Kogna an address on your network. Use the Ethernet adapter that connects the PC to that network, the one whose `IPv4 Address` is on your network, for example `192.168.20.2`. That adapter may carry the PC's internet as well; that is fine.
   - **A fixed IP address** set in the Kogna: the adapter on that address's network.

   The `Description` line of that adapter is the name to use, for example `Intel(R) Ethernet Controller I226-V #2`. The VM gets its internet through the PC on its own, so the internet adapter matters only if the Kogna is on that same network.
   The same names appear after `Name:` in `& "C:\Program Files\Oracle\VirtualBox\VBoxManage.exe" list bridgedifs`; the `VBoxNetworkName:` lines there, with their `HostInterfaceNetworking-` prefix, are not accepted.
   If the name is wrong, the script makes nothing and prints the accepted names; run it again with one of those.
7. Make the VM (one line; the ISO name exactly as downloaded, the adapter name in quotes, or leave it out for a KFLOP):

       .\vm-create.cmd C:\InstallLinuxCNC\debian-13.7.0-amd64-netinst.iso "Intel(R) Ethernet Controller I226-V #2"

   The script stops with a message if the ISO or the adapter name is wrong, before it makes anything.

8. Start it:

       & "C:\Program Files\Oracle\VirtualBox\VBoxManage.exe" startvm KMotion-LinuxCNC

## In the VM: Debian

9. In the installer menu choose **Graphical install**. Then, screen by screen:
   - Language, location, keyboard: yours.
   - Primary network interface (asked when the VM has the Kogna adapter): **enp0s3**. Inside the VM, Debian names the VM's two virtual network cards by their slot, the same on every PC: enp0s3 is VirtualBox adapter 1, the NAT one with the internet (through whatever the PC uses, Wi-Fi included); enp0s8 is adapter 2, the VM's own link to the Kogna over the Ethernet port named in step 7. The Windows adapter names play no part inside the VM.
   - Hostname: `kmotion`. Domain name: leave empty.
   - Root password: leave both fields empty (your user then gets `sudo`, which the install script needs).
   - Full name, username (for example `tk`), password: yours. Remember them.
   - Partition disks: **Guided - use entire disk**, the only disk, **All files in one partition**, **Finish partitioning**, then **Yes** to write the changes.
   - Package manager: the suggested mirror; proxy: empty. A warning "An attempt to configure apt to install additional packages from the media failed" can appear (the installer looked at the VM's second disc, the Guest Additions): Continue, the packages come from the internet.
   - Participate in the package usage survey: No.
   - Software selection: tick **Debian desktop environment**, **Xfce** and **standard system utilities**; untick **GNOME**. Continue. (Missed it? GNOME works too. After logging in, `sudo apt install -y task-xfce-desktop` adds Xfce, chosen with the gear icon on the login screen, and `sudo tasksel install standard` the utilities.)
   - Install the GRUB boot loader to your primary drive: **Yes** (older installers then ask which disk: `/dev/sda`). This is the VM's own virtual disk, the only disk the installer can see; Windows and the PC's disks are untouched.
   - Finish: Continue. The VM reboots into the login screen.
10. Log in. Open the VM's web browser (Firefox, in the applications menu) and go to this page, so the commands below can be copied instead of typed:

       https://github.com/TomKerekes/KMotionX/blob/master/KMotionX/linuxcnc/install/QUICKSTART.md

   Then open a terminal (Applications menu > Terminal Emulator, or Ctrl+Alt+T). Paste into a terminal with **Ctrl+Shift+V** (Ctrl+V alone does nothing there).

## In the VM: everything else

11. In the terminal, these three lines, one at a time (the first and third ask for your password; the third takes 30 to 60 minutes and keeps a log in `install.log`):

        sudo apt install -y curl
        curl -fsSL https://raw.githubusercontent.com/TomKerekes/KMotionX/master/KMotionX/linuxcnc/install/install-vm.sh -o install-vm.sh
        bash install-vm.sh 2>&1 | tee install.log

    It ends with a summary. If it stops with an error, send `install.log`.
12. Log out (top right menu) and log in again. The launchers are then on the desktop; on GNOME also under the Windows key (type LinuxCNC).

## The board

13. Kogna: in the VM window's menu, Devices > Network > Connect Network Adapter (adapter 2). In a terminal, `ip -4 addr show enp0s8` shows an `inet` address within a few seconds, on the Kogna's network (`192.168.113.`something with the Kogna on its own cable, your network's addresses with the Kogna on your router): the VM is on the Kogna's network. If no `inet` line appears, the link has no DHCP profile (a VM installed with an older script): make it with

        nmcli connection add type ethernet ifname enp0s8 con-name Kogna ipv4.method auto ipv6.method disabled autoconnect yes

    KFLOP: plug it in; Devices > USB shows it ticked.
14. The board's firmware must be the one this tree carries. If Machine on is refused with a message naming two builds: desktop icon **KMotion**, Config & Flash, flash `~/.kmotionx/DSP_KOGNA/DSPKOGNA.out` (or `DSP_KFLOP/DSPKFLOP.out`), then press Machine on again.

## First run

15. Double-click the desktop icon **LinuxCNC (KMotion simulate)**: AXIS starts with the planner and no board. Load a program and run it.
16. Then **LinuxCNC (Kogna)** or **LinuxCNC (3 Link)** with the board connected.
17. For diagnostics, the Kogna's own console (what the board prints) is the **Kogna COM Port** icon. It needs the Kogna's USB cable in the PC and, in a VM, the device attached to the VM: VM window menu, Devices > USB, tick the Kogna. Quit the console with Ctrl-t then q.

The shared clipboard between Windows and the VM, drag and drop and a display that follows the window size come with VirtualBox's Guest Additions, which install-vm.sh installs from the disc the VM script attached (they work after step 12). For a VM made without that disc: in the VM window, Devices > Insert Guest Additions CD Image, then run install-vm.sh again.
