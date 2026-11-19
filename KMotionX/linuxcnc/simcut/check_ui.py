# Copyright (c) 2026 DynoMotion Incorporated.  SPDX-License-Identifier: MIT
"""make check-ui: gmoccapy_stock (the Tool image and Create tool buttons, the dialog) and the tool table
comments, with stand-ins for gmoccapy's widgets, and the executed path drawn by kmotion_stock's
DonePathPart into an offscreen GL context (Mesa's EGL on a pbuffer, software rendering; skipped where
there is none).  Needs a display (DISPLAY; Xvfb will do) and the run-in-place tree's lib/python
(LINUXCNC_DIR, default ~/linuxcnc-dev); LinuxCNC need not run."""
import os, sys, tempfile, time
# PyOpenGL keeps some state per GL context, asking the window system binding which context is current;
# the drawing test below makes its context through EGL, so that is the binding to ask (before OpenGL is imported)
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
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

# the executed path, drawn: a GL 3.3 core context offscreen, LinuxCNC's renderer and a stand-in frame; a
# path of ten unit segments along X with a rapid among them, the frontier half way along the fifth;
# the pixels say what was drawn
import rs274.glcanon
from rs274 import glcanon_gl, glcanon_scene
from OpenGL.GL import glViewport, glClearColor, glClear, glReadPixels, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_RGBA, GL_UNSIGNED_BYTE


def egl_context(width, height):
    """A GL 3.3 core context on a pbuffer of width x height, through EGL's surfaceless platform."""
    os.environ.setdefault("EGL_PLATFORM", "surfaceless")
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    from OpenGL import EGL, arrays
    dpy = EGL.eglGetDisplay(EGL.EGL_DEFAULT_DISPLAY)
    major, minor = EGL.EGLint(), EGL.EGLint()
    if not EGL.eglInitialize(dpy, major, minor):
        raise RuntimeError("eglInitialize failed")
    attribs = arrays.GLintArray.asArray([EGL.EGL_SURFACE_TYPE, EGL.EGL_PBUFFER_BIT, EGL.EGL_RENDERABLE_TYPE, EGL.EGL_OPENGL_BIT,
                                         EGL.EGL_RED_SIZE, 8, EGL.EGL_GREEN_SIZE, 8, EGL.EGL_BLUE_SIZE, 8, EGL.EGL_ALPHA_SIZE, 8,
                                         EGL.EGL_DEPTH_SIZE, 24, EGL.EGL_NONE])
    configs = (EGL.EGLConfig * 1)()
    count = EGL.EGLint()
    if not EGL.eglChooseConfig(dpy, attribs, configs, 1, count) or count.value < 1:
        raise RuntimeError("no EGL config with a depth buffer")
    EGL.eglBindAPI(EGL.EGL_OPENGL_API)
    context = EGL.eglCreateContext(dpy, configs[0], EGL.EGL_NO_CONTEXT, arrays.GLintArray.asArray(
        [EGL.EGL_CONTEXT_MAJOR_VERSION, 3, EGL.EGL_CONTEXT_MINOR_VERSION, 3,
         EGL.EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL.EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT, EGL.EGL_NONE]))
    surface = EGL.eglCreatePbufferSurface(dpy, configs[0], arrays.GLintArray.asArray([EGL.EGL_WIDTH, width, EGL.EGL_HEIGHT, height, EGL.EGL_NONE]))
    if not context or not surface or not EGL.eglMakeCurrent(dpy, surface, surface, context):
        raise RuntimeError("no GL 3.3 core context")


W, H = 120, 24
try:
    egl_context(W, H)
except Exception as e:
    print("  skip  the executed path drawn: no offscreen GL context here (%s)" % e)
else:
    glViewport(0, 0, W, H)
    renderer = glcanon_gl.GlCanonRenderer()
    renderer.set_viewport(W, H)
    # X -1..11 across the pixels (a unit is 10 columns, X 0 at column 10), Y -1..1 down them: the path lies at Y 0
    proj = np.array([[2 / 12, 0, 0, -10 / 12], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]], dtype=np.float64)

    class Ini:
        def find(self, section, key): return None
    class Stat:
        task_mode, interp_state, motion_line = 1, 1, 0
    class Canon: pass
    stock = kmotion_stock.StockPart(kmotion_stock.Settings(Ini(), td))
    tips = np.array([[x, 0.0, 0.0] for x in range(11)], np.float32)          # ten unit segments along X
    flags = np.full(11, sim.SIMPOSE_MOVE, np.int32); flags[0] = 0; flags[3] |= sim.SIMPOSE_RAPID   # X 2..3 a rapid
    stock._tips, stock._flags, stock._lines = tips, flags, np.arange(11)
    stock._canon, stock._path_version, stock._frontier = Canon(), 1, (4, 0.5)   # X 0..4 whole, then half of X 4..5
    fields = {name: None for name in glcanon_scene.FrameContext.__slots__}
    fields.update(mv=glcanon_scene.MatrixStack(projection=proj), prim=glcanon_scene.Primitives(), renderer=renderer, caps=renderer.caps,
                  colors=rs274.glcanon.GlCanonDraw.colors, stat=Stat(), canon=stock._canon, width=W, height=H)
    ctx = glcanon_scene.FrameContext(**fields)
    part = kmotion_stock.DonePathPart(stock)

    def frame():
        """One frame of the part alone: the brightest colour in each column, and how many rows are lit in it."""
        glcanon_scene.Scene().apply_baseline()
        glClearColor(0, 0, 0, 1); glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        with part.scope(ctx):
            part.draw(ctx)
        px = np.frombuffer(glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, outputType=bytes), np.uint8).reshape(H, W, 4)[:, :, :3]
        return px.max(axis=0).astype(int), (px.max(axis=2) > 0).sum(axis=0)
    feed = lambda col, x: col[int(10 * (x + 1))][0] > 150 and col[int(10 * (x + 1))][1] < 100     # backplotfeed 0.75 0.25 0.25
    rapid = lambda col, x: col[int(10 * (x + 1))][1] > 100 and col[int(10 * (x + 1))][0] < 100    # backplottraverse 0.30 0.50 0.50
    dark = lambda col, x: col[int(10 * (x + 1))].max() == 0

    def looks_right(col, rows, how):
        check(feed(col, 0.5) and feed(col, 1.5) and rapid(col, 2.5) and feed(col, 3.5),
              "%s: the whole segments in the feed colour, the rapid among them in the traverse colour" % how)
        check(feed(col, 4.2) and dark(col, 4.8) and dark(col, 7) and dark(col, 9.5), "%s: the piece the tool is on as far as the tool (X 4.5), nothing beyond" % how)
        check(dark(col, -0.5) and rows[15] >= 2 and rows[52] >= 2, "%s: nothing before the first move; two pixels wide (%d rows lit at X 0.5)" % (how, rows[15]))
    col, rows = frame()
    looks_right(col, rows, "drawn with GL's line width")
    check(part._segments == 10 and part._vao is not None and part._wide_vao is None, "the path sits in one buffer of %d segments" % part._segments)
    granted = glcanon_gl._MAX_LINE_WIDTH
    glcanon_gl._MAX_LINE_WIDTH = 1.0                                        # a driver that refuses wide lines
    col, rows = frame()
    looks_right(col, rows, "drawn as quads where the driver refuses the width")
    check(part._wide_vao is not None, "... through the buffer bound as endpoint pairs")
    glcanon_gl._MAX_LINE_WIDTH = granted
    stock._frontier = (0, 0.0)
    col, rows = frame()
    check(col.max() == 0, "nothing executed: nothing drawn")
    stock._frontier = (10, 0.0)
    col, rows = frame()
    check(feed(col, 0.5) and rapid(col, 2.5) and feed(col, 9.5) and dark(col, 10.5), "the whole path executed: all of it drawn, to X 10")
    stock._path_version = 2
    stock._tips, stock._flags, stock._lines = tips[:6], flags[:6], np.arange(6)   # another program, half as long
    stock._frontier = (5, 0.0)
    col, rows = frame()
    check(part._segments == 5 and feed(col, 4.5) and dark(col, 5.5), "a new program: the buffer is made again (%d segments)" % part._segments)
print("ui test", "passed" if not fails else "FAILED (%d)" % fails)
sys.exit(1 if fails else 0)
