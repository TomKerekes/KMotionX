#!/bin/bash
# install-desktop-icons.sh: desktop icons and application-menu entries for KMotion (the
# Linux KMotion tool, tools/kmotion) and for LinuxCNC on a KMotion board, for the user who
# runs it. No root needed; nothing outside the user's home is touched.
#
#     KMotionX/desktop/install-desktop-icons.sh              install (or refresh)
#     KMotionX/desktop/install-desktop-icons.sh --remove     take them away again
#
# The launchers point into this checkout, so run it again after moving the tree.
# LinuxCNC: the run-in-place tree kmotion-motion was built against (its makefile's
# LINUXCNC_DIR, default ~/linuxcnc-dev), or LINUXCNC_DIR in the environment, or the
# linuxcnc on PATH.
set -euo pipefail

here=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)   # .../KMotionX/KMotionX/desktop
root=$(cd "$here/../.." && pwd)                        # the checkout
tool="$root/KMotionX/tools/kmotion"
lcnc_tree="$root/KMotionX/linuxcnc"
data="${XDG_DATA_HOME:-$HOME/.local/share}"
apps="$data/applications"
icons="$data/icons/hicolor"
desktop=$(xdg-user-dir DESKTOP 2>/dev/null || true)
[ -n "$desktop" ] || desktop="$HOME/Desktop"

launchers="kmotion-tool linuxcnc-kmotion-kogna linuxcnc-kmotion-sim"
sizes="16 32 48 64 128 256"

note() { printf '%s\n' "$*"; }
warn() { printf 'warning: %s\n' "$*" >&2; }

refresh() {
    command -v update-desktop-database >/dev/null && update-desktop-database "$apps" 2>/dev/null || true
    command -v gtk-update-icon-cache >/dev/null && gtk-update-icon-cache -q -t -f "$icons" 2>/dev/null || true
}

if [ "${1:-}" = "--remove" ]; then
    for l in $launchers; do rm -f "$apps/$l.desktop" "$desktop/$l.desktop"; done
    for n in $sizes; do rm -f "$icons/${n}x${n}/apps/kmotion-tool.png"; done
    rm -f "$icons/scalable/apps/linuxcncicon.svg" "$icons/48x48/apps/linuxcncicon.png"
    refresh
    note "removed the KMotion and LinuxCNC launchers from $desktop and $apps"
    exit 0
fi
[ $# -eq 0 ] || { echo "usage: $0 [--remove]" >&2; exit 2; }

mkdir -p "$apps" "$desktop"

# A launcher on the desktop is run without the file manager's "untrusted launcher"
# question only if it is executable and, on Xfce 4.18+ and GNOME, carries the file
# manager's own trust mark.
mark_trusted() {
    chmod +x "$1"
    command -v gio >/dev/null || return 0
    gio set -t string "$1" metadata::xfce-exe-checksum "$(sha256sum "$1" | cut -d' ' -f1)" 2>/dev/null || true
    gio set -t string "$1" metadata::trusted true 2>/dev/null || true
}

# write_launcher <name>: the text on stdin becomes ~/.local/share/applications/<name>.desktop
# (the menu) and a copy on the desktop.
write_launcher() {
    local f="$apps/$1.desktop"
    cat >"$f"
    chmod 644 "$f"
    cp "$f" "$desktop/$1.desktop"
    mark_trusted "$desktop/$1.desktop"
    note "  $desktop/$1.desktop"
}

# --- KMotion ---------------------------------------------------------------------------
note "KMotion (tools/kmotion)"
if [ ! -e "$root/bin/libkmx_c.so" ]; then
    note "  building bin/libkmx_c.so (the tool's C interface)..."
    make -s -C "$tool/shim" || warn "bin/libkmx_c.so did not build; see tools/kmotion/README.md"
fi
python3 -c 'import PySide6.QtWidgets' 2>/dev/null ||
    warn "python3 has no PySide6: the KMotion launcher will not start until you run
         sudo apt install python3-pyside6.qtwidgets python3-pyside6.qtgui"
for n in $sizes; do
    install -Dm644 "$tool/icons/kmotion-$n.png" "$icons/${n}x${n}/apps/kmotion-tool.png"
done
write_launcher kmotion-tool <<END
[Desktop Entry]
Version=1.0
Type=Application
Name=KMotion
Comment=KMotion's setup and troubleshooting screens for a KFLOP or Kogna (KMotionX)
Exec="$here/kmotion-tool"
Path=$tool
Icon=kmotion-tool
Terminal=false
StartupNotify=true
StartupWMClass=kmotion-tool
Categories=Science;Engineering;
Keywords=kmotion;kflop;kogna;dynomotion;cnc;
END

# --- LinuxCNC --------------------------------------------------------------------------
note "LinuxCNC with KMotion's planner (linuxcnc/)"
motion="$lcnc_tree/kmotion-motion/kmotion-motion"
lcnc_dir=""
# the tree kmotion-motion was linked against is in its run path
if [ -x "$motion" ] && command -v readelf >/dev/null; then
    for p in $(readelf -d "$motion" 2>/dev/null | sed -n 's/.*R\(UN\)\?PATH.*\[\(.*\)\]/\2/p' | tr ':' ' '); do
        [ -x "${p%/lib}/scripts/linuxcnc" ] && { lcnc_dir="${p%/lib}"; break; }
    done
fi
[ -n "$lcnc_dir" ] || lcnc_dir="${LINUXCNC_DIR:-$HOME/linuxcnc-dev}"
if [ -x "$lcnc_dir/scripts/linuxcnc" ]; then
    linuxcnc="$lcnc_dir/scripts/linuxcnc"
    [ -x "$motion" ] || {
        note "  building linuxcnc/kmotion-motion..."
        make -s -C "$lcnc_tree/kmotion-motion" LINUXCNC_DIR="$lcnc_dir" ||
            warn "kmotion-motion did not build; see linuxcnc/README.md"
    }
elif command -v linuxcnc >/dev/null; then
    linuxcnc=$(command -v linuxcnc)
    warn "no LinuxCNC run-in-place tree at $lcnc_dir; the launchers use $linuxcnc.
         kmotion-motion must be built against the same LinuxCNC (linuxcnc/README.md)."
else
    warn "no LinuxCNC found (no $lcnc_dir/scripts/linuxcnc, none on PATH): LinuxCNC launchers skipped."
    linuxcnc=""
fi
if [ -n "$linuxcnc" ]; then
    # the LinuxCNC icon, by its usual name, from the tree or the installed package
    svg="$lcnc_dir/debian/extras/usr/share/icons/hicolor/scalable/apps/linuxcncicon.svg"
    [ -e "$svg" ] && install -Dm644 "$svg" "$icons/scalable/apps/linuxcncicon.svg"
    png="$lcnc_dir/linuxcncicon.png"
    [ -e "$png" ] || png=/usr/share/linuxcnc/linuxcncicon.png
    [ -e "$png" ] && install -Dm644 "$png" "$icons/48x48/apps/linuxcncicon.png"
    write_launcher linuxcnc-kmotion-kogna <<END
[Desktop Entry]
Version=1.0
Type=Application
Name=LinuxCNC (Kogna)
Comment=LinuxCNC with KMotion's trajectory planner on a Kogna (kmotion-kogna.ini)
Exec="$linuxcnc" "$lcnc_tree/configs/kmotion-kogna/kmotion-kogna.ini"
Icon=linuxcncicon
Terminal=false
StartupNotify=false
Categories=Science;Engineering;X-CNC;
Keywords=cnc;linuxcnc;kmotion;kogna;
END
    write_launcher linuxcnc-kmotion-sim <<END
[Desktop Entry]
Version=1.0
Type=Application
Name=LinuxCNC (KMotion simulate)
Comment=LinuxCNC with KMotion's trajectory planner, no board (kmotion-sim.ini)
Exec="$linuxcnc" "$lcnc_tree/configs/kmotion-sim/kmotion-sim.ini"
Icon=linuxcncicon
Terminal=false
StartupNotify=false
Categories=Science;Engineering;X-CNC;
Keywords=cnc;linuxcnc;kmotion;simulate;
END
fi

refresh
note "done: icons on $desktop and in the application menu (Science / Engineering)."
