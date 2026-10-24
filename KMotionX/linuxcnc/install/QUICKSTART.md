# Quick start: LinuxCNC on KMotion in a VirtualBox VM, step by step

Only the steps, for a Windows PC. `INSTALL.md` says why and what each one does.

## On the Windows PC

Nothing else is installed on Windows: no Python, no compiler. Everything else goes into the VM in step 11.

1. Check that the processor's virtualization is on, which VirtualBox needs: Task Manager (Ctrl+Shift+Esc), Performance tab, CPU; at the bottom right, "Virtualization: Enabled". If it says Disabled, it is switched off in the BIOS: restart, enter the BIOS setup (F1, F2, F10, F12 or Del while the PC starts, by maker), find "Intel Virtualization Technology" or "VT-x" (under Advanced > CPU, or on Lenovo under Security > Virtualization; a page that shows it in a different colour and skips it with the arrow keys only displays it, the switch is on the other page), set it to Enabled, save with F10. Without it the VM stops at once with "VT-x is disabled in the BIOS".
   Also on Windows: if VirtualBox shows a green turtle icon at the bottom right of the VM window, or the Debian installer is extremely slow (minutes on "Starting up the partitioner"), Windows' own hypervisor has the processor and VirtualBox is running on top of it. It is switched on by Virtual Machine Platform, Hyper-V or Windows Hypervisor Platform (used by WSL2, Docker Desktop and the Windows Subsystem for Android). Untick those in "Turn Windows features on or off", run `bcdedit /set hypervisorlaunchtype off` in an administrator PowerShell, and restart. (A later "Windows Subsystem for Android couldn't start" message is that feature missing its hypervisor; uninstall it if unused.)
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
   If instead the screen stops on kernel messages (white text, a "RETBleed" or similar warning) and no login screen comes within a minute: the VM's graphics controller does not suit this host (a VirtualBox 7.0 and Debian 13 problem; the VM script now chooses VBoxSVGA, which worked where VMSVGA did not). Power the VM off (close the window, Power off the machine), then Machine > Settings > Display: Graphics Controller **VBoxSVGA**, Video Memory 128 MB, OK, and start it again.
10. Log in. Open the VM's web browser (Firefox, in the applications menu) and go to this page, so the commands below can be copied instead of typed:

       https://github.com/TomKerekes/KMotionX/blob/master/KMotionX/linuxcnc/install/QUICKSTART.md

   Then open a terminal (Applications menu > Terminal Emulator, or Ctrl+Alt+T). Paste into a terminal with **Ctrl+Shift+V** (Ctrl+V alone does nothing there), or right-click in the terminal and choose Paste.

## In the VM: everything else

11. In the terminal, these three lines, one at a time (the first and third ask for your password, typed without anything showing; the third takes 30 to 60 minutes and keeps its own log in `~/install.log`):

        sudo apt install -y curl
        curl -fsSL https://raw.githubusercontent.com/TomKerekes/KMotionX/master/KMotionX/linuxcnc/install/install-vm.sh -o install-vm.sh
        bash install-vm.sh

    Copy them from Firefox inside the VM (step 10); text copied on Windows cannot be pasted into the VM until the install is done, since the shared clipboard is part of what it installs. If a paste leaves junk like `^[[200~` on the line, press Ctrl+C and type the line.
    It ends with a summary ("==== done" and what to do next). A red line "gdm.service is not active, cannot reload" on the way is harmless. If it stops with `*** install-vm.sh STOPPED`, send `~/install.log`.
12. Log out (top right menu) and log in again. The launchers are then on the desktop; on GNOME also under the Windows key (type LinuxCNC).

## The board

13. Kogna: in the VM window's menu, Devices > Network > Connect Network Adapter (adapter 2). In a terminal, `ip -4 addr show enp0s8` shows an `inet` address within a few seconds, on the Kogna's network (`192.168.113.`something with the Kogna on its own cable, your network's addresses with the Kogna on your router): the VM is on the Kogna's network. If no `inet` line appears, the link has no DHCP profile (a VM installed with an older script): make it with

        nmcli connection add type ethernet ifname enp0s8 con-name Kogna ipv4.method auto ipv6.method disabled autoconnect yes

    KFLOP: close KMotion on Windows if it is open, plug the board in, and Devices > USB in the VM window shows it ticked (the VM script's USB filter takes it, and takes it back after a firmware flash or a power cycle). If it is not ticked, tick it.
14. The board's firmware must be the build this tree carries, which is not the same as the Windows 5.5.1 release even when both say 5.5.1 (the module compares the build stamp, date and time). The first Machine on in LinuxCNC (Kogna) will therefore be refused with a message naming the board's build and the file's. Then: desktop icon **KMotion**, Config & Flash, flash `~/.kmotionx/DSP_KOGNA/DSPKOGNA.out` (or `DSP_KFLOP/DSPKFLOP.out`), and press Machine on again. Going back to Windows later means flashing the Windows release's image from KMotion there, the same way.

## First run

15. Double-click the desktop icon **LinuxCNC (KMotion simulate)**: AXIS starts with the planner and no board. LinuxCNC starts in E-stop, so first the two round buttons at the top left of the toolbar, in order: E-stop off (F1), then Machine on (F2); on a laptop whose F keys are media keys, click the buttons or hold Fn. Then **Home All** on the Manual Control tab (Ctrl+Home). The Run button (the blue arrow, R) is grey until then. Load a program (File > Open; the examples are under ~/linuxcnc-dev/nc_files) and run it.
16. Then **LinuxCNC (Kogna)** or **LinuxCNC (3 Link)** with the board connected. Both are configurations for a Kogna: Tom's bench machine and the 3 Link pen robot. There is no KFLOP configuration yet; a bare KFLOP with the Kogna configuration reports a limit switch and will not switch on, because its unconnected inputs read as tripped. Until a KFLOP configuration ships, the simulate configuration is the test for a KFLOP install; the KMotion icon verifies the board link and flashes the firmware.
17. For diagnostics, the Kogna's own console (what the board prints) is the **Kogna COM Port** icon. It needs the Kogna's USB cable in the PC and, in a VM, the device attached to the VM: VM window menu, Devices > USB, tick the Kogna. Quit the console with Ctrl-t then q.

The shared clipboard between Windows and the VM, drag and drop and a display that follows the window size come with VirtualBox's Guest Additions, which install-vm.sh installs from the disc the VM script attached (they work after step 12). For a VM made without that disc: in the VM window, Devices > Insert Guest Additions CD Image, then run install-vm.sh again.
