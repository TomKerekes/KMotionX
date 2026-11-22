#!/usr/bin/env python3
"""The Create tool dialog on its own, for a GUI that is not gmoccapy (AXIS's Stock menu runs it):

    tool_dialog.py --images DIR [--table FILE] [--tool N]

The dialog is gmoccapy_stock's (GTK): a basic tool's sizes in, a tool image file in DIR out; with a
table and a tool, it starts from that tool's image or diameter, and "Use it for tool N" writes the
image's name into the tool's comment in the table file and has LinuxCNC read the table again (when
one runs)."""
import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# LinuxCNC's python modules (rs274.glcanon_scene, which the Stock builds on): on the path already in a
# run-in-place environment (AXIS passes its own), else LINUXCNC_DIR's lib/python (default ~/linuxcnc-dev)
if not any(os.path.exists(os.path.join(p, "rs274", "glcanon_scene.py")) for p in sys.path if p):
    sys.path.insert(0, os.path.join(os.path.expanduser(os.environ.get("LINUXCNC_DIR", "~/linuxcnc-dev")), "lib", "python"))


def table_tool_defaults(table, tool):
    """What the dialog starts from for tool number tool of a tool table file: its image's type and
    sizes when the image is one the dialog made, else the table's diameter (the D word)."""
    import gmoccapy_stock
    import kmotion_stock
    if not table or not tool:
        return None
    name = kmotion_stock.read_tool_table_images(table).get(int(tool), "")
    found = gmoccapy_stock.image_defaults(name) if name else None
    if found:
        return found
    diameter = 0.0
    try:
        with open(table, encoding="utf-8", errors="replace") as f:
            for raw in f:
                line = raw.partition(";")[0]
                words = line.split()
                if any(w[:1] in "Tt" and w[1:] == str(int(tool)) for w in words):
                    for w in words:
                        if w[:1] in "Dd":
                            try:
                                diameter = float(w[1:])
                            except ValueError:
                                pass
                    break
    except OSError:
        pass
    return (None, {"D": gmoccapy_stock.number_text(diameter)}) if diameter > 0 else None


def main():
    ap = argparse.ArgumentParser(description="the Create tool dialog on its own")
    ap.add_argument("--images", required=True, help="the tool image folder")
    ap.add_argument("--table", default="", help="the LinuxCNC tool table file")
    ap.add_argument("--tool", type=int, default=0, help="the tool to give the image to")
    a = ap.parse_args()
    import gi
    gi.require_version("Gtk", "3.0")
    from gi.repository import Gtk
    import gmoccapy_stock
    import kmotion_stock
    tool = a.tool if a.tool > 0 and a.table else None
    dlg = gmoccapy_stock.CreateToolDialog(None, a.images, tool, table_tool_defaults(a.table, a.tool))
    while True:
        if dlg.run() != Gtk.ResponseType.OK:
            break
        path = dlg.create()
        if path is None:
            continue                                           # the reason is in the dialog
        if tool and dlg.assign.get_active():
            name = os.path.basename(path)
            try:
                if not kmotion_stock.table_file_with_image(a.table, tool, name):
                    dlg.message.set_text("Tool %d is not in the tool table." % tool)
                    continue
            except OSError as e:
                dlg.message.set_text(str(e))
                continue
            try:
                import linuxcnc
                linuxcnc.command().load_tool_table()
            except Exception:
                pass                                           # no LinuxCNC running: the file is written
        break
    dlg.destroy()


if __name__ == "__main__":
    main()
