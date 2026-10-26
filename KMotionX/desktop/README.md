# Desktop icons for KMotion and LinuxCNC

Icons on the desktop and entries in the application menu for the KMotion tool
(`tools/kmotion`) and for LinuxCNC running on a KMotion board (`linuxcnc/`). One script,
run as yourself, nothing outside your home directory:

    ~/KMotionX/KMotionX/desktop/install-desktop-icons.sh

It puts six launchers on the desktop and in the menu (Science / Engineering):

- **KMotion** – the Linux KMotion tool (Digital I/O, Axis, C Programs, Config, Console).
- **LinuxCNC (Minirouter)** – LinuxCNC with `configs/kmotion-minirouter/kmotion-minirouter.ini`: the bench, a KFLOP or a Kogna with nothing attached.
- **Kogna COM Port** – the Kogna's virtual COM port in a terminal (`desktop/kogna-console`: tio on /dev/ttyACM0; the Kogna's USB attached to the machine, or to the VM with Devices > USB).
- **Pull KMotionX** – brings this checkout up to date from GitHub in a terminal window (`desktop/pull-kmotionx`: git pull, and what came in).
- **Re-install LinuxCNC** – runs the installer again (`desktop/reinstall-linuxcnc`: `linuxcnc/install/install-vm.sh`, which skips what is done and rebuilds and installs what changed; asks for your password). Close LinuxCNC and the KMotion tool first.

It also puts `desktop/clipboard-png` into the session's autostart (`~/.config/autostart`): VirtualBox's shared clipboard brings a Windows screenshot in as a bitmap only, which browser-based programs (Claude Code, Firefox, Chromium) cannot paste; the helper re-offers such an image as PNG too. `--remove` takes it away with the launchers.
- **LinuxCNC (3 Link)** – LinuxCNC on the 3 Link pen robot, `configs/kmotion-3link/kmotion-3link.ini`.
- **LinuxCNC (KMotion simulate)** – the same planner with no board,
  `configs/kmotion-sim/kmotion-sim.ini`.

What it does along the way:

- Builds `bin/libkmx_c.so` (the KMotion tool's C interface) if it is missing, and warns if
  python3 has no PySide6 (`sudo apt install python3-pyside6.qtwidgets python3-pyside6.qtgui`,
  the one thing it cannot do for you).
- Finds the LinuxCNC that `kmotion-motion` was built against (its run path), else the
  run-in-place tree in `LINUXCNC_DIR` (default `~/linuxcnc-dev`), else the `linuxcnc` on
  PATH; builds `kmotion-motion` if it is missing. Without any LinuxCNC the two LinuxCNC
  launchers are skipped.
- Installs the icons into `~/.local/share/icons/hicolor` (KMotion.exe's icon at six sizes,
  LinuxCNC's icon from its tree) and marks the desktop copies as trusted, so Xfce and GNOME
  run them on a double-click without the "untrusted launcher" question.

The launchers hold absolute paths into this checkout and the LinuxCNC tree, so run the
script again after moving either; it overwrites its own files. `--remove` takes everything
away again.

Another machine configuration: copy one of the LinuxCNC launchers in
`~/.local/share/applications/` under a new name, change `Name=` and the `.ini` path on its
`Exec=` line, and copy it to the desktop (make it executable; the first double-click may
ask once to trust it).

The KMotion launcher runs `kmotion-tool` (next to this file), which sends the tool's
output to `~/.cache/kmotionx/kmotion-tool.log` and shows the end of it in a window if the
tool fails to start, since there is no terminal to read it in.
