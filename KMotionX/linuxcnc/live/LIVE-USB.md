# LinuxCNC on KMotion from a USB stick

A ready-made Debian 13 system with LinuxCNC, the KMotion motion module, the KMotion tool and
KMotionXCNC already installed. Start a PC from the stick and it runs: nothing is installed on the
PC and its disk is not touched. The stick can keep your settings and machine configuration from one
start to the next, and the same stick can install the system onto the PC's disk if you decide to.

For a KFLOP (USB) or a Kogna (Ethernet). No real-time kernel is needed: the board does the real time.

## What you need

- A 64-bit PC (Intel or AMD) with 4 GB of memory or more; 8 GB is comfortable.
- A USB stick: 8 GB runs it; **16 GB or more to keep changes** (32 GB if you will also update the
  software on it). A USB 3 stick in a USB 3 port makes a large difference in speed.
- The image, about 4.5 GB: **https://dynomotion.com/Software/LinuxCNC/**
  (`kmotion-linuxcnc-<date>.iso`, and `kmotion-linuxcnc-<date>.iso.sha256` beside it).

Optional, to check the download arrived whole: compare the SHA-256 of the file with the one in the
`.sha256` file.

    Windows (PowerShell):   Get-FileHash kmotion-linuxcnc-<date>.iso
    Linux:                  sha256sum -c kmotion-linuxcnc-<date>.iso.sha256
    macOS:                  shasum -a 256 kmotion-linuxcnc-<date>.iso

## 1. Write the stick

Everything on the stick is erased. The image must be written *as an image*, not copied as a file.

**balenaEtcher** (Windows, macOS, Linux; https://etcher.balena.io): Flash from file, pick the ISO;
Select target, pick the stick; Flash.

**Rufus** (Windows; https://rufus.ie): pick the stick under Device, SELECT the ISO, START. Rufus asks
how to write it: choose **Write in DD Image mode**. Leave its "Persistent partition size" at 0 (the
stick's own "Keep Changes on USB" icon does that, step 3).

**Linux**: find the stick's device with `lsblk` (the whole stick, `/dev/sdX`, not `/dev/sdX1`), then

    sudo dd if=kmotion-linuxcnc-<date>.iso of=/dev/sdX bs=4M status=progress conv=fsync

Afterwards Windows may offer to format the stick, or show it as a small or unreadable drive. Do not
format it: that is how a Linux stick looks to Windows.

## 2. Start the PC from the stick

Plug the stick in, switch the PC on, and press its boot-menu key as it starts: usually **F12** (Dell,
Lenovo, Acer), **F9** or **Esc** (HP), **F8** or **Esc** (Asus), **F11** (MSI, ASRock), **F2/F10** on
some others. Pick the USB stick, then **Live system (amd64)** in the menu that follows.

- Secure Boot can normally stay on: the image starts through Debian's signed boot loader. If the PC
  refuses the stick, switch Secure Boot off in the PC's setup and try again.
- You are logged in as the user **cnc**; the password, where one is asked for, is **cnc**.

On the desktop: **KMotion** (the KMotion tool: console, C programs, axis screens),
**LinuxCNC on KMotion** launchers (the Minirouter bench configuration, the 3-link robot, and a
simulator that needs no board), **KMotionXCNC** (the web CNC application), **Migrate from
KMotionCNC**, **Pull KMotionX** and **Re-install LinuxCNC** (updating), **Keep Changes on USB**.

Connect a KFLOP by USB. Connect a Kogna by Ethernet with its IP setting at 0, in one of two ways:

- **on its own cable** to a spare Ethernet port of the PC: the Kogna finds no DHCP server on that
  cable and becomes one, and the port gets an address starting `192.168.113.` a few seconds later;
- **on your network** (a switch or the router): the router gives the Kogna and the PC their addresses.

Either way the wired connection comes up by itself, nothing to set. `ip -4 addr` in a terminal shows
the addresses.

## 3. Keep your changes on the stick

As written, the stick is read-only: whatever you change is held in memory and gone at power-off.
To make the stick keep it:

1. Double-click **Keep Changes on USB**.
2. It shows the stick and the unused space behind the image; type `yes`.
3. Restart the PC from the stick.

From then on everything is kept: machine configurations, the files in your home folder, desktop and
network settings, updates you install. The live system itself on the stick is never modified; the
changes go to a separate volume in the stick's unused space.

- Shut down from the menu (Applications > Log Out > Shut Down) before pulling the stick or switching
  the PC off: then nothing is lost. If the power is simply cut, what was saved in the last few
  seconds may be missing; the KMotion tool's C programs are on the stick the moment they are saved.
- **To start fresh again**: at the boot menu, with *Live system* highlighted, press **Tab** (on a
  UEFI machine **e**), add the word `nopersistence` to the end of the line that starts with `linux`,
  and start (Enter; **F10** on UEFI). Run **Keep Changes on USB** in that session: it offers to empty
  the volume.
- The stick works in any PC, with its changes. Copy what matters off it now and then: USB sticks
  wear out.

## 4. Updating

With changes kept (and the PC on the internet): **Pull KMotionX**, then **Re-install LinuxCNC**,
exactly as on an installed system. A new image from the download page is the other way: write it to
the stick again (that erases the kept changes; copy your configuration folder off first).

## 5. Installing onto the PC's disk (optional)

In the stick's boot menu choose **Start installer** instead of *Live system*. It is the Debian
installer and puts the same system, with everything preinstalled, on the PC.

- It **erases the disk** you let it partition.
- When it asks for the user to create, give the user name **cnc** (any full name and password): the
  preinstalled software lives in `/home/cnc`.

## Trying it without a stick: VirtualBox on Windows

Two scripts in this folder make a throwaway VirtualBox VM around the ISO:

    vm-test-iso.cmd   C:\path\kmotion-linuxcnc-<date>.iso  ["host adapter the Kogna is on"]
    vm-test-stick.cmd C:\path\kmotion-linuxcnc-<date>.iso  [GB]  ["host adapter the Kogna is on"]

`vm-test-iso.cmd` starts the ISO as a DVD (nothing can be kept). `vm-test-stick.cmd` writes the image
onto a virtual disk as onto a stick, so **Keep Changes on USB** works there too. Both pass a KFLOP,
the Kogna's COM port and a Logitech F310 game pad through by USB; the optional adapter name (from
`VBoxManage list bridgedifs`) bridges a second network card to the Kogna.
