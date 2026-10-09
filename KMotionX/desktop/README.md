# Desktop icons for KMotion and LinuxCNC

Icons on the desktop and entries in the application menu for the KMotion tool
(`tools/kmotion`) and for LinuxCNC running on a KMotion board (`linuxcnc/`). One script,
run as yourself, nothing outside your home directory:

    ~/KMotionX/KMotionX/desktop/install-desktop-icons.sh

It puts three launchers on the desktop and in the menu (Science / Engineering):

- **KMotion** – the Linux KMotion tool (Digital I/O, Axis, C Programs, Config, Console).
- **LinuxCNC (Kogna)** – LinuxCNC with `configs/kmotion-kogna/kmotion-kogna.ini`.
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
