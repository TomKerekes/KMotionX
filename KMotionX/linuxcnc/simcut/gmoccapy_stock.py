# gmoccapy_stock.py : the Stock's buttons in gmoccapy
# Copyright (c) 2026 DynoMotion Incorporated.  SPDX-License-Identifier: MIT
"""gmoccapy's buttons for the Stock (kmotion_stock.py).  Under the preview, beside the show/hide
dimensions toggle, two toggles: the Stock shown or hidden, and the program's path lines shown or
hidden (KMotionCNC's Stock and Paths buttons; gmoccapy's own pencil toggle is the live plot).  Both
are kept in gmoccapy's preferences, view_stock and view_paths, like the others.  On the tool table
page:

  Tool image...   a tool image (KMotionCNC's VRML tool images) for the tool selected in the table.
                  The file's name goes into the tool's comment as "[name.wrl]", the table is saved
                  (LinuxCNC reloads it) and the Stock cuts with that shape from then on.
  Create tool...  a basic tool - end mill, ball nose, V bit, T slot, dovetail - from its sizes, written
                  as a tool image into the tool image folder, as KMotionCNC's Create Tool dialog does,
                  and given to the selected tool if asked.

stock_rc.py calls install(gmoccapy, part) when it runs under gmoccapy; kmotion_stock stays
GUI-independent.  The image name is read from the table by kmotion_stock.ToolImages, so what
these buttons do shows in any GUI."""
import logging
import os
import re

import gi
gi.require_version("Gtk", "3.0")
gi.require_version("GdkPixbuf", "2.0")
from gi.repository import Gtk, Gdk, GdkPixbuf  # noqa: E402
import cairo  # noqa: E402

import kmotion_simcut as sim  # noqa: E402
import kmotion_stock  # noqa: E402

log = logging.getLogger("kmotion_stock")

HERE = os.path.dirname(os.path.abspath(__file__))

#: The basic tools, as KMotionCNC's Create Tool has them: label, the engine's type, its sizes in the
#: order SetBasic takes them as (letter, name, default) - the letters are the ones on the picture -
#: the picture (KMotionCNC's, pictures/) and the pattern of the file name
TOOL_TYPES = [
    ("End mill", sim.SIMTOOL_ENDMILL, [("D", "Diameter", "0.25"), ("L", "Length", "1.5")], "ToolEndMill", "EndMill D{0} L{1}"),
    ("Ball nose", sim.SIMTOOL_BALL, [("D", "Diameter", "0.25"), ("L", "Length", "1.5")], "ToolBallNose", "Ball D{0} L{1}"),
    ("V", sim.SIMTOOL_V, [("D", "Diameter", "0.5"), ("A", "Angle (degrees)", "60"), ("T", "Tip diameter (0: a point)", "0"),
                          ("L", "Length", "2")], "ToolV", "V D{0} A{1} T{2} L{3}"),
    ("T slot", sim.SIMTOOL_TSLOT, [("D", "Diameter", "0.75"), ("H", "Height", "0.25"), ("N", "Neck diameter", "0.3125"),
                                  ("L", "Length", "2")], "ToolTSlot", "TSlot D{0} H{1} N{2} L{3}"),
    ("Dovetail", sim.SIMTOOL_DOVETAIL, [("D", "Diameter", "0.75"), ("A", "Angle (degrees)", "60"), ("H", "Height", "0.3125"),
                                        ("N", "Neck diameter", "0.375"), ("L", "Length", "2")], "ToolDovetail",
     "Dovetail D{0} A{1} H{2} N{3} L{4}"),
]
PICTURE_WIDTH = 240


def picture(name):
    """KMotionCNC's picture of a basic tool with its sizes lettered, as a Gtk.Image (empty if missing)."""
    path = os.path.join(HERE, "pictures", name + ".png")
    try:
        pb = GdkPixbuf.Pixbuf.new_from_file(path)
        pb = pb.scale_simple(PICTURE_WIDTH, int(pb.get_height() * PICTURE_WIDTH / pb.get_width()), GdkPixbuf.InterpType.BILINEAR)
        return Gtk.Image.new_from_pixbuf(pb)
    except Exception as e:
        log.error("tool picture %s: %s", path, e)
        return Gtk.Image()


COMMENT_COLUMN = 16         # of the tool table widget's model (gladevcp tooledit: KEYWORDS' ';')
TOOL_COLUMN = 1
DIAMETER_COLUMN = 12        # KEYWORDS' 'D'


def number_text(v):
    """0.25 -> "0.25", 2.0 -> "2": for file names."""
    t = ("%g" % float(v))
    return t


def image_defaults(name):
    """The type and sizes a tool image's file name encodes, as the dialog names its files
    ("Dovetail D0.75 A60 H0.3125 N0.375 L2.wrl"): (type index, {letter: text}), or None for a name
    of another making."""
    base = os.path.splitext(os.path.basename(name or ""))[0]
    for k, (_label, _kind, sizes, _picture, pattern) in enumerate(TOOL_TYPES):
        parts = re.split(r"\{\d+\}", pattern)
        rx = "^" + r"(-?\d+\.?\d*)".join(re.escape(p) for p in parts) + "$"
        m = re.match(rx, base, re.IGNORECASE)
        if m and len(m.groups()) == len(sizes):
            try:
                return k, {letter: number_text(float(v)) for (letter, _n, _d), v in zip(sizes, m.groups())}
            except ValueError:
                return None
    return None


def tool_defaults(app, part, tool):
    """What the Create tool dialog starts from for the selected tool: its image's type and sizes
    when the image is one the dialog made (its name says so), else the table's diameter."""
    if not tool:
        return None
    name, diameter = "", 0.0
    try:
        for row in app.widgets.tooledit1.model:
            if int(row[TOOL_COLUMN]) == int(tool):
                name = kmotion_stock.image_in_comment(row[COMMENT_COLUMN] or "")
                try:
                    diameter = float(row[DIAMETER_COLUMN] or 0)
                except (TypeError, ValueError):
                    diameter = 0.0
                break
    except Exception:
        log.exception("the selected tool's row")
    if not name:
        name = part.images.name(tool)
    found = image_defaults(name) if name else None
    if found:
        return found
    return (None, {"D": number_text(diameter)}) if diameter > 0 else None


class CreateToolDialog(Gtk.Dialog):
    """Sizes of a basic tool in, a tool image file out (create()).  defaults: (type index or None,
    {letter: text}) to start from - the selected tool's image, or its diameter (tool_defaults)."""

    def __init__(self, parent, image_dir, tool=None, defaults=None):
        Gtk.Dialog.__init__(self, title="Create tool", transient_for=parent, modal=True)
        self.image_dir = image_dir
        self.tool = tool
        self.add_button("_Cancel", Gtk.ResponseType.CANCEL)
        self.add_button("_Create", Gtk.ResponseType.OK)
        self.set_default_response(Gtk.ResponseType.OK)
        outer = self.get_content_area()
        outer.set_spacing(6)
        outer.set_border_width(10)
        columns = Gtk.Box(spacing=16)
        outer.pack_start(columns, True, True, 0)
        self.picture_box = Gtk.Box()                        # the picture of the type, KMotionCNC's
        columns.pack_start(self.picture_box, False, False, 0)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        columns.pack_start(box, True, True, 0)

        row = Gtk.Box(spacing=6)
        row.pack_start(Gtk.Label(label="Tool", xalign=0), False, False, 0)
        self.combo = Gtk.ComboBoxText()
        for label, _kind, _sizes, _picture, _pattern in TOOL_TYPES:
            self.combo.append_text(label)
        self.combo.set_active(0)
        self.combo.connect("changed", self._on_type_changed)
        row.pack_start(self.combo, True, True, 0)
        box.pack_start(row, False, False, 0)

        self.grid = Gtk.Grid(row_spacing=4, column_spacing=8)
        box.pack_start(self.grid, False, False, 0)
        self.entries = []
        box.pack_start(Gtk.Label(label="Sizes in the machine's units, the letters as on the picture; the tip is at height 0, "
                                       "the tool closed at its length.", xalign=0, wrap=True, max_width_chars=44), False, False, 0)

        row = Gtk.Box(spacing=6)
        row.pack_start(Gtk.Label(label="File", xalign=0), False, False, 0)
        self.file_entry = Gtk.Entry()
        self.file_entry.set_hexpand(True)
        row.pack_start(self.file_entry, True, True, 0)
        row.pack_start(Gtk.Label(label=".wrl in " + os.path.basename(image_dir.rstrip("/") or image_dir), xalign=0), False, False, 0)
        box.pack_start(row, False, False, 0)

        self.assign = Gtk.CheckButton(label="Use it for tool %s (the tool selected in the table)" % tool if tool else
                                      "Select a tool in the table to give it the image")
        self.assign.set_sensitive(bool(tool))
        self.assign.set_active(bool(tool))
        box.pack_start(self.assign, False, False, 0)

        self.message = Gtk.Label(label="", xalign=0, wrap=True, max_width_chars=44)
        box.pack_start(self.message, False, False, 0)
        self._name_edited = False
        self.file_entry.connect("changed", self._on_name_edited)
        self._on_type_changed(self.combo)
        if defaults:
            kind, values = defaults
            if kind is not None and 0 <= kind < len(TOOL_TYPES):
                self.combo.set_active(kind)
            sizes = TOOL_TYPES[self.combo.get_active()][2]
            for (letter, _n, _d), e in zip(sizes, self.entries):
                if letter in values:
                    e.set_text(str(values[letter]))
        self.show_all()

    # -- the form ---------------------------------------------------------------

    def _on_type_changed(self, combo):
        for child in self.grid.get_children():
            self.grid.remove(child)
        for child in self.picture_box.get_children():
            self.picture_box.remove(child)
        self.entries = []
        _label, _kind, sizes, picture_name, _pattern = TOOL_TYPES[combo.get_active()]
        self.picture_box.pack_start(picture(picture_name), False, False, 0)
        self.picture_box.show_all()
        for r, (letter, name, default) in enumerate(sizes):
            self.grid.attach(Gtk.Label(label="<b>%s</b>" % letter, use_markup=True, xalign=0.5), 0, r, 1, 1)
            self.grid.attach(Gtk.Label(label=name, xalign=0), 1, r, 1, 1)
            e = Gtk.Entry(text=default)
            e.set_width_chars(10)
            e.set_activates_default(True)
            e.connect("changed", self._on_size_changed)
            self.grid.attach(e, 2, r, 1, 1)
            self.entries.append(e)
        self.grid.show_all()
        self._name_edited = False
        self._suggest_name()

    def _on_size_changed(self, _entry):
        if not self._name_edited:
            self._suggest_name()

    def _on_name_edited(self, _entry):
        if not getattr(self, "_suggesting", False):
            self._name_edited = True

    def sizes(self):
        return [float(e.get_text().strip().replace(",", ".")) for e in self.entries]

    def _suggest_name(self):
        _label, _kind, _sizes, _picture, pattern = TOOL_TYPES[self.combo.get_active()]
        try:
            name = pattern.format(*[number_text(v) for v in self.sizes()])
        except ValueError:
            return
        self._suggesting = True
        self.file_entry.set_text(name)
        self._suggesting = False

    # -- the result ----------------------------------------------------------------

    def path(self):
        name = self.file_entry.get_text().strip()
        if not name:
            return ""
        if not name.lower().endswith(".wrl"):
            name += ".wrl"
        return name if os.path.isabs(name) else os.path.join(self.image_dir, name)

    def create(self):
        """Write the tool image.  The file's path, or None with the reason shown in the dialog."""
        label, kind, size_labels, _picture, _pattern = TOOL_TYPES[self.combo.get_active()]
        try:
            sizes = self.sizes()
        except ValueError:
            self.message.set_text("Every size must be a number.")
            return None
        path = self.path()
        if not path:
            self.message.set_text("The file needs a name.")
            return None
        comment = "%s: %s" % (label, ", ".join("%s %s" % (sl[1].split(" (")[0], number_text(v)) for sl, v in zip(size_labels, sizes)))
        try:
            err = sim.write_tool_image(path, kind, sizes, 36, comment)
        except Exception as e:
            err = str(e)
        if err:
            self.message.set_text(err)
            return None
        log.info("tool image written: %s (%s)", path, comment)
        return path


# ---------------------------------------------------------------------------- the buttons

def selected_tool(app):
    tool = app.widgets.tooledit1.get_selected_row()
    try:
        return int(tool) if tool else None
    except (TypeError, ValueError):
        return None


def set_tool_image(app, part, tool, path):
    """Put the image's name into the tool's comment in the table, save the table (LinuxCNC reloads it)
    and have the Stock cut with it.  path "" takes the image away."""
    te = app.widgets.tooledit1
    image_dir = os.path.normpath(part.images.image_dir)
    name = path
    if path and os.path.normpath(os.path.dirname(os.path.abspath(path))) == image_dir:
        name = os.path.basename(path)
    done = False
    for row in te.model:
        if int(row[TOOL_COLUMN]) == int(tool):
            row[COMMENT_COLUMN] = kmotion_stock.comment_with_image(row[COMMENT_COLUMN], name)
            done = True
    if not done:
        log.error("tool %s is not in the table", tool)
        return False
    te.edited = True
    app.on_btn_save_tool_changes_clicked(None)
    log.info("tool %s: image %s", tool, name or "none")
    part.refeed("tool %s given the image %s" % (tool, name or "none"))
    return True


def on_tool_image(_button, app, part):
    tool = selected_tool(app)
    if tool is None:
        app.dialogs.warning_dialog(app, "Tool image", "Select a tool in the table first.")
        return
    dlg = Gtk.FileChooserDialog(title="Tool image for tool %d" % tool, transient_for=app.widgets.window1,
                                action=Gtk.FileChooserAction.OPEN)
    dlg.add_button("_Cancel", Gtk.ResponseType.CANCEL)
    dlg.add_button("_None", Gtk.ResponseType.REJECT)
    dlg.add_button("_Open", Gtk.ResponseType.OK)
    f = Gtk.FileFilter()
    f.set_name("Tool images (*.wrl)")
    f.add_pattern("*.wrl")
    f.add_pattern("*.WRL")
    dlg.add_filter(f)
    current = part.images.name(tool)
    if current and os.path.exists(part.images.path(current)):
        dlg.set_filename(part.images.path(current))
    else:
        dlg.set_current_folder(part.images.image_dir)
    try:
        response = dlg.run()
        if response == Gtk.ResponseType.OK and dlg.get_filename():
            set_tool_image(app, part, tool, dlg.get_filename())
        elif response == Gtk.ResponseType.REJECT:
            set_tool_image(app, part, tool, "")
    finally:
        dlg.destroy()


def on_create_tool(_button, app, part):
    tool = selected_tool(app)
    dlg = CreateToolDialog(app.widgets.window1, part.images.image_dir, tool, tool_defaults(app, part, tool))
    try:
        while True:
            response = dlg.run()
            if response != Gtk.ResponseType.OK:
                break
            path = dlg.create()
            if path is None:
                continue                                   # the reason is in the dialog
            if tool and dlg.assign.get_active():
                set_tool_image(app, part, tool, path)
            else:
                part.refeed("a tool image was written")
            break
    finally:
        dlg.destroy()


def stock_icon(size=32):
    """A small picture of a cut block, for the toggle under the preview: drawn here, so that it
    follows no theme file."""
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, size, size)
    c = cairo.Context(surface)
    s = size / 32.0
    c.scale(s, s)
    # the block, seen a little from above: top face lighter, front darker
    c.move_to(4, 12); c.line_to(22, 12); c.line_to(28, 6); c.line_to(10, 6); c.close_path()
    c.set_source_rgb(0.72, 0.74, 0.77); c.fill()
    c.rectangle(4, 12, 18, 14); c.set_source_rgb(0.55, 0.57, 0.60); c.fill()
    c.move_to(22, 12); c.line_to(28, 6); c.line_to(28, 20); c.line_to(22, 26); c.close_path()
    c.set_source_rgb(0.45, 0.47, 0.50); c.fill()
    # a slot cut across the top, blue as a feed cut
    c.move_to(7, 10); c.line_to(25, 10); c.line_to(26.5, 8.5); c.line_to(8.5, 8.5); c.close_path()
    c.set_source_rgb(0.25, 0.56, 0.92); c.fill()
    c.rectangle(4, 12, 18, 2.5); c.set_source_rgb(0.20, 0.45, 0.80); c.fill()
    return Gtk.Image.new_from_pixbuf(Gdk.pixbuf_get_from_surface(surface, 0, 0, size, size))


def paths_icon(size=32):
    """A small picture of a tool path, for the toggle that shows or hides the program's lines."""
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, size, size)
    c = cairo.Context(surface)
    s = size / 32.0
    c.scale(s, s)
    c.set_line_width(2.2)
    c.set_line_join(cairo.LINE_JOIN_ROUND)
    c.set_source_rgb(0.22, 0.22, 0.22)                 # dark on the button's light face (white vanished on it)
    c.move_to(4, 26)
    for i, x in enumerate(range(7, 29, 4)):
        c.line_to(x, 6 if i % 2 == 0 else 26)
    c.stroke()
    c.set_source_rgb(0.92, 0.26, 0.22)                 # a rapid to the start
    c.set_dash([2.5, 2.0])
    c.move_to(2, 4); c.line_to(4, 26); c.stroke()
    return Gtk.Image.new_from_pixbuf(Gdk.pixbuf_get_from_surface(surface, 0, 0, size, size))


def _toggle_after(app, anchor, icon, tooltip, pref, default, apply):
    """A toggle button like anchor (gmoccapy's dimensions toggle), after it in its row; its state
    is gmoccapy's preference pref, applied through apply(state), which also runs at the start."""
    box = anchor.get_parent()
    btn = Gtk.ToggleButton()
    btn.set_image(icon)
    btn.set_always_show_image(True)
    btn.set_tooltip_text(tooltip)
    w, h = anchor.get_size_request()
    if w > 0 and h > 0:
        btn.set_size_request(w, h)
    btn.set_halign(anchor.get_halign())
    btn.set_valign(anchor.get_valign())
    state = app.prefs.getpref(pref, default, bool)
    apply(state)
    btn.set_active(state)

    def toggled(b):
        apply(b.get_active())
        app.prefs.putpref(pref, b.get_active())
        app.widgets.gremlin.queue_draw()
    btn.connect("toggled", toggled)
    box.pack_start(btn, False, False, 0)
    try:
        box.reorder_child(btn, box.get_children().index(anchor) + 1)
    except (ValueError, AttributeError):
        pass
    btn.show_all()
    return btn


def install_view_toggles(app, part):
    """The Stock and Paths toggles under the preview, after the dimensions toggle."""
    dim = getattr(app.widgets, "tbtn_view_dimension", None)
    if dim is None or dim.get_parent() is None:
        return None
    gremlin = app.widgets.gremlin

    def show_program(on):
        try:
            gremlin.set_property("show_program", on)
        except TypeError:                                  # not a property of this widget: the attribute then
            gremlin.show_program = on
    paths = _toggle_after(app, dim, paths_icon(), "Show or hide the program's path lines (the cut surface shows better without them)",
                          "view_paths", True, show_program)
    stock = _toggle_after(app, dim, stock_icon(), "Show or hide the Stock (the material removal)", "view_stock", True, part.set_visible)
    return stock, paths


def _button(label, tooltip, callback, *args):
    b = Gtk.Button(label=label)
    b.set_size_request(56, 56)
    b.set_tooltip_text(tooltip)
    b.connect("clicked", callback, *args)
    b.show_all()
    return b


def install(app, part):
    """gmoccapy's buttons for the Stock (app: the gmoccapy object): the show/hide toggle under the
    preview and the two on the tool table page."""
    try:
        install_view_toggles(app, part)
    except Exception:
        log.exception("the Stock and Paths toggles under the preview")
    te = app.widgets.tooledit1
    buttonbox = te.wTree.get_object("buttonbox")
    buttonbox.pack_start(_button("Tool\nimage", "A tool image (shape) for the tool selected in the table: the Stock cuts with it",
                                 on_tool_image, app, part), False, False, 0)
    buttonbox.pack_start(_button("Create\ntool", "A basic tool's image from its sizes, as KMotionCNC's Create Tool", on_create_tool, app, part),
                         False, False, 0)
    log.info("gmoccapy: the Stock and Paths toggles under the preview, Tool image and Create tool on the tool table page")
