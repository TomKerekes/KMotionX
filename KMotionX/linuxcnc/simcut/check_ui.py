# Copyright (c) 2026 DynoMotion Incorporated.  SPDX-License-Identifier: MIT
"""make check-ui: gmoccapy_stock (the Tool image and Create tool buttons, the dialog) and the tool table
comments, with stand-ins for gmoccapy's widgets.  Needs a display (DISPLAY; Xvfb will do) and the
run-in-place tree's lib/python (LINUXCNC_DIR, default ~/linuxcnc-dev); LinuxCNC need not run."""
import os, sys, tempfile, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.environ.get("LINUXCNC_DIR", os.path.expanduser("~/linuxcnc-dev")), "lib", "python"))
sys.path.insert(0, HERE)
import gi; gi.require_version("Gtk", "3.0"); from gi.repository import Gtk
import kmotion_stock, gmoccapy_stock, kmotion_simcut as sim

fails = 0
def check(ok, text):
    global fails
    print("  %s  %s" % ("pass" if ok else "FAIL", text)); fails += 0 if ok else 1

# the comments
check(kmotion_stock.image_in_comment("1/8 end mill [EndMill-z 1in .125.wrl]") == "EndMill-z 1in .125.wrl", "image read from a comment")
check(kmotion_stock.image_in_comment("1/8 end mill") == "", "no image in a plain comment")
check(kmotion_stock.comment_with_image("1/8 end mill [old.wrl]", "new.wrl") == "1/8 end mill [new.wrl]", "image replaced in a comment")
check(kmotion_stock.comment_with_image("1/8 end mill [old.wrl]", "") == "1/8 end mill", "image taken out of a comment")
check(kmotion_stock.comment_with_image("", "x.wrl") == "[x.wrl]", "image into an empty comment")

td = tempfile.mkdtemp()
table = os.path.join(td, "tool.tbl")
open(table, "w").write("T1 P1 Z0.511 D0.125 ;1/8 end mill [EndMill-z 1in .125.wrl]\nT2 P2 D0.0625 ;1/16 end mill\n")
images = os.path.join(td, "images"); os.mkdir(images)
open(os.path.join(td, "toolimages.txt"), "w").write("T2 other.wrl\nT1 fromfile.wrl\n")
ti = kmotion_stock.ToolImages(os.path.join(td, "toolimages.txt"), images, table)
check(ti.name(1) == "EndMill-z 1in .125.wrl" and ti.name(2) == "other.wrl" and ti.name(3) == "", "the table's comment wins, TOOL_IMAGES next (%s, %s)" % (ti.name(1), ti.name(2)))
check(not ti.changed(), "nothing changed yet")
time.sleep(0.05); open(table, "a").write("T3 P3 D0.2 ;drill\n"); os.utime(table, None)
check(ti.changed(), "a change of the tool table is seen")

# gmoccapy stand-ins
class WTree:
    def __init__(self): self.box = Gtk.ButtonBox()
    def get_object(self, name): return self.box if name == "buttonbox" else None
class ToolEdit:
    def __init__(self):
        self.wTree = WTree(); self.edited = False
        self.model = Gtk.ListStore(int, int, int, str, str, str, str, str, str, str, str, str, str, str, str, int, str)
        self.model.append([0, 1, 1] + ["0"] * 12 + [0, "1/8 end mill"])
        self.model.append([0, 2, 2] + ["0"] * 12 + [0, "1/16 end mill [x.wrl]"])
    def get_selected_row(self): return 2
class Widgets: pass
class Dialogs:
    def warning_dialog(self, *a): print("   (warning dialog)", a[1:])
class Prefs:
    def __init__(self): self.d = {"view_stock": False}
    def getpref(self, k, default, kind): return self.d.get(k, default)
    def putpref(self, k, v, kind=None): self.d[k] = v
class Gremlin(Gtk.DrawingArea):
    draws = 0
    show_program = True
    def queue_draw(self): self.draws += 1
class App:
    def __init__(self):
        self.widgets = Widgets(); self.widgets.tooledit1 = ToolEdit(); self.widgets.window1 = Gtk.Window()
        self.widgets.gremlin = Gremlin(); self.prefs = Prefs()
        self.widgets.box_gremlin_buttons = Gtk.Box()
        self.widgets.tbtn_view_dimension = Gtk.ToggleButton(label="dim"); self.widgets.tbtn_view_dimension.set_size_request(48, 48)
        self.widgets.box_gremlin_buttons.pack_start(Gtk.Button(label="zoom"), False, False, 0)
        self.widgets.box_gremlin_buttons.pack_start(self.widgets.tbtn_view_dimension, False, False, 0)
        self.widgets.box_gremlin_buttons.pack_start(Gtk.Button(label="clear"), False, False, 0)
        self.dialogs = Dialogs(); self.saved = 0
    def on_btn_save_tool_changes_clicked(self, w): self.saved += 1
class Part:
    def __init__(self): self.images = ti; self.refed = []; self.visible = True
    def refeed(self, why=""): self.refed.append(why); return True
    def set_visible(self, on): self.visible = bool(on)
app, part = App(), Part()
gmoccapy_stock.install(app, part)
check(len(app.widgets.tooledit1.wTree.box.get_children()) == 2, "two buttons on the tool table page")
kids = app.widgets.box_gremlin_buttons.get_children()
tog = [k for k in kids if isinstance(k, Gtk.ToggleButton) and k is not app.widgets.tbtn_view_dimension]
d = kids.index(app.widgets.tbtn_view_dimension)
check(len(tog) == 2 and kids.index(tog[0]) == d + 1 and kids.index(tog[1]) == d + 2 and all(t.get_size_request() == (48, 48) for t in tog),
      "the Stock and Paths toggles sit right after the dimensions toggle, the same size")
check(not part.visible and not tog[0].get_active(), "the Stock toggle starts as the preference says (hidden here)")
tog[0].set_active(True)
check(part.visible and app.prefs.d["view_stock"] is True and app.widgets.gremlin.draws == 1, "toggling shows the Stock, saves the preference and redraws")
check(tog[1].get_active() and app.widgets.gremlin.show_program is True, "the Paths toggle starts on (no preference yet)")
tog[1].set_active(False)
check(app.widgets.gremlin.show_program is False and app.prefs.d["view_paths"] is False, "toggling it off hides the program's lines and saves the preference")
ok = gmoccapy_stock.set_tool_image(app, part, 2, os.path.join(images, "V D0.5 A60 T0 L2.wrl"))
row = [r for r in app.widgets.tooledit1.model if r[1] == 2][0]
check(ok and row[16] == "1/16 end mill [V D0.5 A60 T0 L2.wrl]" and app.saved == 1 and part.refed, "the image goes into tool 2's comment by its name, the table is saved, the Stock refed: %r" % row[16])
gmoccapy_stock.set_tool_image(app, part, 2, "/elsewhere/odd.wrl")
check(row[16] == "1/16 end mill [/elsewhere/odd.wrl]", "an image outside the folder by its full path")

# the Create tool dialog
dlg = gmoccapy_stock.CreateToolDialog(None, images, tool=2)
check(dlg.file_entry.get_text() == "EndMill D0.25 L1.5", "the name follows the sizes: %r" % dlg.file_entry.get_text())
dlg.combo.set_active(2)                                   # V bit
pic = dlg.picture_box.get_children()[0]
check(isinstance(pic, Gtk.Image) and pic.get_pixbuf() is not None and pic.get_pixbuf().get_width() == gmoccapy_stock.PICTURE_WIDTH, "the V picture is shown")
letters = [c.get_text() for c in dlg.grid.get_children() if isinstance(c, Gtk.Label) and len(c.get_text()) == 1]
check(sorted(letters) == ["A", "D", "L", "T"], "the sizes are lettered as on the picture: %s" % letters)
for k in range(len(gmoccapy_stock.TOOL_TYPES)):
    dlg.combo.set_active(k)
    assert dlg.picture_box.get_children()[0].get_pixbuf() is not None, k
dlg.combo.set_active(2)
check(len(dlg.entries) == 4 and dlg.file_entry.get_text() == "V D0.5 A60 T0 L2", "a V bit has four sizes and its name: %r" % dlg.file_entry.get_text())
dlg.entries[0].set_text("0.25"); dlg.entries[1].set_text("90")
path = dlg.create()
check(path and os.path.exists(path) and path.endswith("V D0.25 A90 T0 L2.wrl"), "created %s" % path)
h, r, open_ = sim.read_tool_image(path)
check(len(h) == 3 and abs(max(r) - 0.125) < 1e-6 and abs(h[1] - 0.125) < 1e-4, "read back: a 90 degree point of radius 0.125 reaches full radius at height 0.125 (%s)" % list(zip(h, r)))
dlg.entries[3].set_text("0.01")
check(dlg.create() is None and dlg.message.get_text(), "sizes that make no tool are refused with a reason: %r" % dlg.message.get_text())
dlg.destroy()

# the tool drawn from its image: the mesh of revolution
import math, numpy as np
f = os.path.join(td, "dove.wrl")
assert sim.write_tool_image(f, sim.SIMTOOL_DOVETAIL, [0.75, 60, 0.3125, 0.375, 2.0], 36, "") is None
h, r, open_ = sim.read_tool_image(f)
pts = kmotion_stock.simplify_profile(h, r, 0.01 * max(r))
check(4 <= len(pts) <= 12, "a dovetail's %d profile points simplify to %d corners" % (len(h), len(pts)))
m = kmotion_stock.revolve_profile(h, r, open_)
tri = m.reshape(-1, 3, 6)
a, b, cc = tri[:, 0, :3], tri[:, 1, :3], tri[:, 2, :3]
face = np.cross(b - a, cc - a)
n = tri[:, :, 3:].mean(axis=1)
okk = np.linalg.norm(face, axis=1) > 1e-9
check(m.dtype == np.float32 and m.shape[1] == 6 and (np.einsum("ij,ij->i", face[okk], n[okk]) > 0).all(),
      "the dovetail's mesh: %d triangles, all wound counter-clockwise seen from outside" % (len(m) // 3))
check(abs(m[:, 2].min()) < 1e-6 and abs(m[:, 2].max() - 2.0) < 1e-6 and abs(np.hypot(m[:, 0], m[:, 1]).max() - 0.375) < 1e-4,
      "from the tip at z 0 to the length 2, radius 0.375 at most")
m2 = kmotion_stock.revolve_profile([0, 1], [0.125, 0.125], False, slices=8)
m3 = kmotion_stock.revolve_profile([0, 1], [0.125, 0.125], True, slices=8)
check(len(m2) == 8 * 6 + 8 * 3 * 2 and len(m3) == 8 * 6 + 8 * 3, "a closed end mill has two caps, an open one the bottom only")
tp = kmotion_stock.ImageToolPart(ti)
check(tp.mesh_for(3) is None, "a tool without an image draws LinuxCNC's own marker")
print("ui test", "passed" if not fails else "FAILED (%d)" % fails)
sys.exit(1 if fails else 0)
