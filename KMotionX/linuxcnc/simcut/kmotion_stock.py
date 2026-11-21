# kmotion_stock.py : the Stock (material removal) in a LinuxCNC preview, cut by KMotionCNC's engine
# Copyright (c) 2026 DynoMotion Incorporated.  SPDX-License-Identifier: MIT (see SimCut-LICENSE.txt
# beside SimCut.cpp)
"""The Stock of KMotionCNC's G Code Viewer for LinuxCNC's preview: a block of material that the
program cuts, drawn in the preview of AXIS, gmoccapy or any GUI that draws through
rs274.glcanon (the shared GL 3.3 core / ES 3.0 renderer).  The cutting is KMotionCNC's engine,
SimCut.cpp, through the kmotion_simcut module (simcut_module.cc); this file feeds it and draws
what it hands over.

    import kmotion_stock
    part = kmotion_stock.install(gremlin_widget, inifile)

install() adds a StockPart to the widget's scene, before the program lines, and times redraws
while the engine cuts.  stock_rc.py does that from [DISPLAY] USER_COMMAND_FILE; README.md has the
ini keys ([KMOTION] STOCK, STOCK_RESOLUTION, STOCK_BOX, STOCK_FILE, STOCK_FOLLOW, TOOL_IMAGES,
TOOL_IMAGE_DIR, STOCK_LINE_WIDTH, STOCK_DEFAULT_DIAMETER, STOCK_PIECE, STOCK_TOOL_SHAPE,
STOCK_DONE_PATH).

What the Stock is cut with
  - The block: the program's (WORKPIECE,BOX,...) comment (the first one; a cylinder or tube is
    taken by its bounding box), else [KMOTION] STOCK_BOX, else a Stock file (STOCK_FILE), else
    nothing is drawn.  Machine coordinates, as the preview draws them.
  - The path: the preview's vertices as poses, the tool tip in machine coordinates (the preview
    draws the tip), a move longer than STOCK_PIECE (0.05 inch) in pieces of that length so that the
    cut can follow the tool along it (KMotionCNC plots long moves in pieces of 0.05 s).  A traverse
    cuts as a rapid (red), a feed or arc as a feed (blue); dwell and tool change markers are poses
    without a move.  Arcs come already segmented.
  - The tool of each vertex: the tool the preview recorded there (its tool changes, M6; with the
    gcodemodule patch of KMotionX/linuxcnc/patches also M61, which the M6 remap uses), else the
    tool in the spindle.  Its shape is the tool image named for it in [KMOTION] TOOL_IMAGES
    (KMotionCNC's VRML tool images, at scale 1), else a flat end mill of the tool table's diameter.
  - The tool direction (4 and 5 axis): from the A, B and C positions the renderer recorded at each
    vertex, turned as KMotionCNC's viewer turns the tool: about X by A, then about Y by B, then
    about Z by C.  The part is taken as fixed and the tool as pivoting (RTCP), whatever the
    machine's kinematics do to achieve that.

When the Stock is cut
  - A program loaded: the whole program is cut at once, the finished part is shown.
  - A run (STOCK_FOLLOW = 1, the default): when the program starts the Stock goes back to uncut
    and is cut as far as the tool has come: every vertex of the lines before the executing line
    (stat.motion_line), and on that line the piece the tool is on.  At the end, or an abort, the
    whole program is cut again.
  - Jogging and MDI do not cut.

The executed path (STOCK_DONE_PATH = 1, the default)
  The part of the program's path a run has executed is drawn over the program's lines: from the
  first move to the piece being executed, and along that piece as far as the tool, feeds in the
  live plot's feed colour (glcanon's backplotfeed) and rapids in its traverse colour, a little wider
  than the program's lines.  It is taken from the program's own path - the poses above, and the
  progress that feeds the engine - and never from samples of the position: the board executes the
  trajectory the PC planned, so one status read gives the right picture, and a stall of the status
  pauses the frontier where gmoccapy's live plot (the pencil toggle), drawn from the samples, draws
  a straight line across the segments it missed.  Cleared when a run starts; it stays after the run
  ends, until the next start or a program load.  DonePathPart draws it, after the program's lines.

Drawing
  The engine hands the surface over in blocks (meshes) of 20 byte vertices (position, kind,
  direction) and 16 bit indices; each lives in a VBO and EBO of its own, and is drawn with a
  shader that colours by kind (grey uncut, blue feed, red rapid) and shades by the surface
  direction, as KMotionCNC's Stock3d.cpp does.  The Stock is pushed back in depth by the width of
  the path lines so that the lines show on the cut surface (KMotionCNC pulls the lines nearer
  instead; the result is the same), and drawn with glPolygonOffset(0, 1), a slope factor of 0 -
  a slope factor made a Stock seen from afar look speckled.
"""
import ctypes
import logging
import re
from contextlib import contextmanager
import math
import os
import time

import numpy as np
from OpenGL.GL import *  # noqa: F401,F403 - the renderer is PyOpenGL

import linuxcnc
import rs274.glcanon
from rs274 import glcanon_bake, glcanon_gl, glcanon_scene

import kmotion_simcut as sim

log = logging.getLogger("kmotion_stock")
#: The part's own log, /tmp/kmotion-stock.log (KMOTION_STOCK_LOG names another file, an empty
#: value none): what it did with each program and run, and its errors, whatever the GUI does
#: with Python's logging.
_log_file = os.environ.get("KMOTION_STOCK_LOG", "/tmp/kmotion-stock.log")
if _log_file and not any(getattr(h, "_kmotion_stock", False) for h in log.handlers):
    try:
        _h = logging.FileHandler(_log_file)
        _h.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s", "%H:%M:%S"))
        _h._kmotion_stock = True
        log.addHandler(_h)
        log.setLevel(logging.INFO)
    except OSError:
        pass

_debug_file = os.environ.get("KMOTION_STOCK_DEBUG")


def _debug(text):
    if _debug_file:
        with open(_debug_file, "a") as f:
            f.write(text + "\n")

#: Colours of the surface by what made it (Stock3d.cpp)
GREY = (0.62, 0.64, 0.67)
BLUE = (0.25, 0.56, 0.92)
RED = (0.92, 0.26, 0.22)
#: The light, fixed in view space (Stock3d.cpp)
LIGHT = (0.408, 0.408, 0.816)
#: The executed path: the live plot's colours (glcanon's backplotfeed and backplottraverse, these
#: when the GUI's colour table has no others) and its width in pixels, a little more than the
#: program's lines (1) so that it reads on top of them
DONE_FEED = (0.75, 0.25, 0.25)
DONE_RAPID = (0.30, 0.50, 0.50)
DONE_PATH_WIDTH = 2.0
#: How near the tool has to come to a piece of the path for that piece to count as cut, in
#: inches (KMotionCNC's TOOL_ON_PATH_TOL)
TOOL_ON_PATH_TOL_INCH = 0.01
LOOKAHEAD_POSES = 600                   # how far past the executing line the tool is looked for on the path (the
                                        # status's line trails the tool by about a second here; a piece is 0.05 inch)

# The shaders.  No #version line: rs274.glcanon_gl.compile_shader prepends the one of the live
# API (GLSL 330 core, or GLSL ES 3.00 with precision statements).  A port of Stock3d.cpp's: the
# face normal comes from the derivatives of the eye space position, turned toward the viewer, and
# the direction the engine gives each point is blended in by its interpolated length (a centroid
# varying), so near a fixed point - an edge, a corner, a face of the Stock - the triangle shows
# its own face and edges stay sharp.  The eye space here is the modelview's alone: glnav folds the
# camera's distance into the projection, so a face is turned toward the viewer by its eye space Z
# and the Stock is pushed back in world space, along the view direction (see push_distance).
VERTEX_SHADER = """
layout(location = 0) in vec3 a_pos;
layout(location = 1) in float a_kind;
layout(location = 2) in vec4 a_dir;
uniform mat4 u_mvp;
uniform mat4 u_mv;
uniform mat3 u_normal;
uniform vec3 u_away;        // the view direction in world space, away from the viewer
uniform float u_push;       // how far back the Stock goes along it (world units)
out vec3 v_eye;
centroid out vec3 v_dir;
out float v_kind;
void main() {
    vec3 p = a_pos + u_away * u_push;
    v_eye = (u_mv * vec4(p, 1.0)).xyz;
    v_dir = u_normal * a_dir.xyz;
    v_kind = a_kind;
    gl_Position = u_mvp * vec4(p, 1.0);
}
"""

FRAGMENT_SHADER = """
in vec3 v_eye;
centroid in vec3 v_dir;
in float v_kind;
uniform vec3 u_light;
uniform vec3 u_grey;
uniform vec3 u_blue;
uniform vec3 u_red;
out vec4 frag;
void main() {
    vec3 face = cross(dFdx(v_eye), dFdy(v_eye));
    float fl = length(face);
    face = fl > 0.0 ? face / fl : vec3(0.0, 0.0, 1.0);
    if (face.z < 0.0) face = -face;                  // toward the viewer, which looks along -Z
    float dl = length(v_dir);
    vec3 n = normalize(mix(face, v_dir / max(dl, 1e-6), clamp(dl, 0.0, 1.0)));
    vec3 c = v_kind < 1.0 ? mix(u_grey, u_blue, clamp(v_kind, 0.0, 1.0))
                          : mix(u_blue, u_red, clamp(v_kind - 1.0, 0.0, 1.0));
    vec3 l = normalize(u_light);
    float diff = 0.3 + 0.7 * max(dot(n, l), 0.0);
    vec3 h = normalize(l + vec3(0.0, 0.0, 1.0));
    float spec = pow(max(dot(n, h), 0.0), 24.0) * 0.15;
    frag = vec4(c * diff + vec3(spec), 1.0);
}
"""

VERTEX_BYTES = 20


# ---------------------------------------------------------------------------- settings

class Settings:
    """The [KMOTION] keys of the ini file that concern the Stock (README.md)."""

    def __init__(self, ini, ini_dir):
        self.ini_dir = ini_dir

        def get(key, default=None):
            v = ini.find("KMOTION", key)
            return v if v not in (None, "") else default

        def path(key, default=None):
            v = get(key, default)
            if not v:
                return None
            return os.path.normpath(os.path.join(ini_dir, os.path.expanduser(v)))

        self.enabled = get("STOCK", "1").strip() not in ("0", "no", "off")
        self.resolution = float(get("STOCK_RESOLUTION", "0"))
        box = get("STOCK_BOX")
        self.box = None
        if box:
            vals = [float(v) for v in box.replace(",", " ").split()]
            if len(vals) == 6:
                self.box = (tuple(vals[:3]), tuple(vals[3:]))
            else:
                log.error("[KMOTION] STOCK_BOX needs six numbers (xmin ymin zmin xmax ymax zmax), not %r", box)
        self.stock_file = path("STOCK_FILE")
        self.follow = get("STOCK_FOLLOW", "1").strip() not in ("0", "no", "off")
        self.tool_images = path("TOOL_IMAGES")
        tt = ini.find("EMCIO", "TOOL_TABLE")
        self.tool_table = os.path.normpath(os.path.join(ini_dir, os.path.expanduser(tt))) if tt else None
        here = os.path.dirname(os.path.abspath(__file__))
        self.tool_image_dir = path("TOOL_IMAGE_DIR") or os.path.normpath(
            os.path.join(here, "..", "..", "..", "KMotion", "Data", "ToolImages"))
        self.line_width = float(get("STOCK_LINE_WIDTH", "1.5"))
        self.default_diameter = float(get("STOCK_DEFAULT_DIAMETER", "0"))
        self.tool_shape = get("STOCK_TOOL_SHAPE", "1").strip() not in ("0", "no", "off")
        self.done_path = get("STOCK_DONE_PATH", "1").strip() not in ("0", "no", "off")
        self.piece = get("STOCK_PIECE")                      # None: 0.05 inch in the machine's units
        coords = (ini.find("TRAJ", "COORDINATES") or "XYZ").upper()
        self.rotary = any(a in coords for a in "ABC")
        geometry = (ini.find("DISPLAY", "GEOMETRY") or "XYZ").upper().replace(" ", "")
        if geometry.lstrip("-") != "XYZ" and geometry != "XYZ":
            log.warning("[DISPLAY] GEOMETRY is %r: the Stock takes the preview's coordinates as machine "
                        "coordinates, which they are only for XYZ", geometry)


IMAGE_TAG = re.compile(r"\[([^\[\]]*\.wrl)\]", re.IGNORECASE)


def image_in_comment(comment):
    """The tool image named in a tool table comment, as "... [EndMill-z 1in .125.wrl]" names one; "" if none."""
    found = IMAGE_TAG.findall(comment or "")
    return found[-1].strip() if found else ""


def comment_with_image(comment, name):
    """The comment with [name] as its tool image (in place of one it had); name "" takes it out."""
    base = IMAGE_TAG.sub("", comment or "").rstrip()
    if not name:
        return base
    return (base + " " if base else "") + "[%s]" % name


def read_tool_table_images(file):
    """{tool number: image name} from the comments of a LinuxCNC tool table file."""
    out = {}
    if not file:
        return out
    try:
        with open(file, encoding="utf-8", errors="replace") as f:
            for raw in f:
                line, _, comment = raw.partition(";")
                name = image_in_comment(comment)
                if not name:
                    continue
                for word in line.split():
                    if word[:1] in "Tt":
                        try:
                            out[int(word[1:])] = name
                        except ValueError:
                            pass
                        break
    except OSError as e:
        log.error("tool table: %s", e)
    return out


class ToolImages:
    """Which tool image (KMotionCNC VRML) each tool number has, from two places: the tool table's
    comments ("[name.wrl]" in a tool's comment, which gmoccapy's tool table page edits: the Tool
    image and Create tool buttons of gmoccapy_stock.py put it there), and [KMOTION] TOOL_IMAGES, one
    tool a line: the tool number (T1 or 1) and then the file name, which may have blanks.  A name
    without a folder is looked for in TOOL_IMAGE_DIR.  # starts a comment.  The table's comment wins.
    changed() says whether either file has changed since reload()."""

    def __init__(self, file, image_dir, tool_table=None):
        self.file = file
        self.tool_table = tool_table
        self.image_dir = image_dir
        self.by_number = {}
        self.from_table = {}
        self._missing = set()
        self._mtimes = None
        self.reload()

    @staticmethod
    def _mtime(path):
        try:
            return os.path.getmtime(path) if path else None
        except OSError:
            return None

    def changed(self):
        return (self._mtime(self.file), self._mtime(self.tool_table)) != self._mtimes

    def reload(self):
        self._mtimes = (self._mtime(self.file), self._mtime(self.tool_table))
        self._missing = set()
        self.from_table = read_tool_table_images(self.tool_table)
        self.by_number = {}
        if not self.file:
            return
        try:
            with open(self.file, encoding="utf-8") as f:
                for n, raw in enumerate(f, 1):
                    line = raw.split("#", 1)[0].strip()
                    if not line:
                        continue
                    parts = line.split(None, 1)
                    try:
                        number = int(parts[0].lstrip("Tt"))
                        name = parts[1].strip()
                    except (ValueError, IndexError):
                        log.error("%s line %d: expected a tool number and a file name, not %r", self.file, n, raw.rstrip())
                        continue
                    self.by_number[number] = name
        except OSError as e:
            log.error("tool images: %s", e)

    def name(self, number):
        """The image's name as the tables give it, "" for none."""
        number = int(number)
        return self.from_table.get(number) or self.by_number.get(number) or ""

    def path(self, name):
        return name if os.path.isabs(name) else os.path.join(self.image_dir, name)

    def image(self, number):
        """The file of the tool's image, or "" when it has none (or the file is missing)."""
        name = self.name(number)
        if not name:
            return ""
        path = self.path(name)
        if not os.path.exists(path):
            if path not in self._missing:
                self._missing.add(path)
                log.error("tool %d: no tool image %s (a flat end mill of the tool table diameter is used)", number, path)
            return ""
        return path


# ---------------------------------------------------------------------------- the path as poses

def tool_direction(a, b, c):
    """Which way the tool points for A, B, C in degrees, as KMotionCNC's viewer turns it:
    (0, 0, 1) turned about X by A, then about Y by B, then about Z by C.  Arrays in, (n, 3) out."""
    ra, rb, rc = np.radians(a), np.radians(b), np.radians(c)
    x0 = np.cos(ra) * np.sin(rb)
    y0 = -np.sin(ra)
    z0 = np.cos(ra) * np.cos(rb)
    x = x0 * np.cos(rc) - y0 * np.sin(rc)
    y = x0 * np.sin(rc) + y0 * np.cos(rc)
    return np.stack([x, y, z0], axis=1).astype(np.float32)


def tip_positions(geom):
    """The tool tip at every vertex, (n, 3) float32.  The preview draws the tip: the renderer takes
    the tool length offset out of the positions (check_feed.py checks that against a program with
    a length offset), so this is a copy of them and nothing else."""
    return np.array(geom.positions(0), dtype=np.float32)


def pose_flags(kinds):
    """SIMPOSE flags from the preview's vertex kinds: a drawn segment ending at the vertex is a
    move (a traverse at the rapid rate); an event marker is a pose the tool only appears at."""
    kinds = np.asarray(kinds)
    drawn = kinds <= glcanon_bake.LAST_DRAWN_KIND
    flags = np.where(drawn, sim.SIMPOSE_MOVE, 0)
    flags = np.where(drawn & (kinds == glcanon_bake.KIND_TRAVERSE), flags | sim.SIMPOSE_RAPID, flags)
    flags = flags.astype(np.int32)
    if len(flags):
        flags[0] = 0                                           # nothing travels to the first vertex
    return flags


def tool_axes(geom):
    """(n, 3) tool directions from the recorded A, B, C positions, or None when the machine has
    none, they were not recorded, or the tool stands upright throughout."""
    axis_pos = getattr(geom, "axis_positions", None)
    letters = getattr(geom, "axis_letters", "") or ""
    if axis_pos is None or axis_pos.shape[1] == 0:
        return None
    cols = {}
    for letter in "ABC":
        k = letters.find(letter)
        cols[letter] = axis_pos[:, k] if k >= 0 else None
    if all(v is None for v in cols.values()):
        return None
    n = axis_pos.shape[0]
    a = cols["A"] if cols["A"] is not None else np.zeros(n, np.float32)
    b = cols["B"] if cols["B"] is not None else np.zeros(n, np.float32)
    c = cols["C"] if cols["C"] is not None else np.zeros(n, np.float32)
    if not (np.any(a) or np.any(b) or np.any(c)):
        return None
    return tool_direction(a, b, c)


def split_moves(tips, lines, tools, flags, axis, piece):
    """The same path with every move longer than piece given in pieces of at most that length (the
    tool direction turned evenly along the move), so that the cut can follow the tool along a long
    move instead of taking the whole move at once.  piece <= 0: as it is."""
    if piece <= 0 or len(tips) < 2:
        return tips, lines, tools, flags, axis
    seg = tips[1:] - tips[:-1]
    length = np.linalg.norm(seg, axis=1)
    pieces = np.ones(len(tips), dtype=np.int64)
    moving = (flags[1:] & sim.SIMPOSE_MOVE) != 0
    pieces[1:] = np.where(moving, np.maximum(1, np.ceil(length / piece).astype(np.int64)), 1)
    pieces = np.minimum(pieces, 500)
    if pieces.max() <= 1:
        return tips, lines, tools, flags, axis
    total = int(pieces.sum())
    out_tips = np.empty((total, 3), dtype=np.float32)
    out_lines = np.empty(total, dtype=np.int64)
    out_tools = np.empty(total, dtype=np.int32)
    out_flags = np.empty(total, dtype=np.int32)
    out_axis = np.empty((total, 3), dtype=np.float32) if axis is not None else None
    at = 0
    for i in range(len(tips)):
        n = int(pieces[i])
        if n == 1:
            out_tips[at] = tips[i]
            if out_axis is not None:
                out_axis[at] = axis[i]
        else:
            f = (np.arange(1, n + 1, dtype=np.float32) / n)[:, None]
            out_tips[at:at + n] = tips[i - 1] + f * (tips[i] - tips[i - 1])
            if out_axis is not None:
                d = axis[i - 1] + f * (axis[i] - axis[i - 1])
                out_axis[at:at + n] = d / np.maximum(np.linalg.norm(d, axis=1), 1e-9)[:, None]
        out_lines[at:at + n] = lines[i]
        out_tools[at:at + n] = tools[i]
        out_flags[at:at + n] = flags[i]
        at += n
    return out_tips, out_lines, out_tools, out_flags, out_axis


def vertex_tools(geom, spindle_tool=0):
    """The tool number at every vertex, (n,) int64.  The renderer records at each vertex an
    ordinal into its list of tool changes (geom.tool_numbers: entry 0 is "none yet"), not the
    number itself.  Where no change has been recorded the tool is the one in the spindle: a
    program that changes no tool cuts with it, and so do the moves before its first change."""
    ordinals = np.asarray(geom.tools, dtype=np.int64)
    numbers = list(getattr(geom, "tool_numbers", None) or [None])
    table = np.array([int(n) if n is not None else 0 for n in numbers] or [0], dtype=np.int64)
    known = (ordinals >= 0) & (ordinals < len(table))
    tools = table[np.where(known, ordinals, 0)]
    if spindle_tool:
        tools = np.where(tools == 0, int(spindle_tool), tools)
    return tools


def program_poses(geom, stock, images, diameters, default_diameter=0.0, piece=0.0, spindle_tool=0):
    """Feed the program's vertices to the engine as poses.  Returns the per-pose line numbers
    (int64), tip positions and SIMPOSE flags, or None when the program has no vertices.

    geom: rs274.glcanon_bake.ProgramGeometry; stock: kmotion_simcut.Stock; images: ToolImages;
    diameters: {tool number: diameter} from the tool table (machine units); default_diameter for a tool
    number that is not in it (0: such a tool cuts nothing); piece: a move longer than this is given in
    pieces (split_moves), 0 for none; spindle_tool: the tool in the spindle, for vertices before any
    tool change (vertex_tools)."""
    if geom is None or getattr(geom, "is_empty", True) or len(geom) == 0:
        return None
    tips = tip_positions(geom)
    kinds = geom.kinds
    tools = vertex_tools(geom, spindle_tool)
    index_of = {}
    for number in np.unique(tools):
        number = int(number)
        dia = float(diameters.get(number, default_diameter) or 0.0)
        index_of[number] = stock.add_tool_image(images.image(number), 1.0, (0.0, 0.0, 0.0), dia)
    tool_index = np.vectorize(index_of.__getitem__, otypes=[np.int32])(tools)
    tips, lines, tool_index, flags, axis = split_moves(tips, np.asarray(geom.lines, dtype=np.int64), tool_index,
                                                        pose_flags(kinds), tool_axes(geom), piece)
    stock.add_poses(np.ascontiguousarray(tips), tool_index, flags, axis)
    return lines, tips, flags


def done_path_vertices(tips, flags, feed, rapid):
    """The path as the executed path is drawn from: GL_LINES in the renderer's per-vertex-colour
    layout, (2 (n - 1), 8) float32 - position, rgba, a spare - segment i joining pose i to pose
    i + 1, in rapid (rgba) for a traverse and feed for any other move.  A pose the tool only appears
    at (a dwell, a tool change, a jump) gets a segment of no length, so that segment i stays the
    move into pose i + 1 whatever comes before."""
    n = len(tips)
    out = np.zeros((2 * max(n - 1, 0), 8), dtype=np.float32)
    if n < 2:
        return out
    moving = (flags[1:] & sim.SIMPOSE_MOVE) != 0
    traverse = (flags[1:] & sim.SIMPOSE_RAPID) != 0
    out[0::2, :3] = np.where(moving[:, None], tips[:-1], tips[1:])
    out[1::2, :3] = tips[1:]
    colour = np.where(traverse[:, None], np.asarray(rapid, np.float32), np.asarray(feed, np.float32))
    out[0::2, 3:7] = colour
    out[1::2, 3:7] = colour
    return out


UNIT_WORD = re.compile(r"(?<![A-Z0-9._#<\[])G0*2([01])(?![0-9.])", re.IGNORECASE)   # a G20 or G21 word
AXIS_WORD = re.compile(r"(?<![A-Z0-9_])[XYZABCUVW]\s*[-+\[#.0-9]", re.IGNORECASE)      # a move's (or a G10's) coordinate
NAMED_PARAM = re.compile(r"#<[^>]*>")                                                # #<name>: its letters are not words


def _without_comments(line):
    """A G-code line without its ( ) and ; comments."""
    out, depth = [], 0
    for ch in line:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth = max(0, depth - 1)
        elif depth == 0:
            if ch == ";":
                break
            out.append(ch)
    return "".join(out)


def workpiece_units(canon, stat, geom):
    """The program's (WORKPIECE,...) comments in the program's units.

    The preview parses a program in the machine's units (G20 on an inch machine) and reads a
    WORKPIECE comment's numbers in the units in force at its line.  A metric program usually has
    its G21 below the comment, so on an inch machine a 50 mm block came out as 50 inches.  Here a
    comment with no G20/G21 above it in the file and no UNITS key of its own gets the units the
    program moves in (its last G20/G21 at or before the first move): its points are scaled about
    the frame's origin, in place, so the preview's own stock outline follows.  Nothing changes
    for a comment below the program's unit word, one saying UNITS=MM or UNITS=INCH, or a
    program that never says.  Returns the comments changed."""
    path = getattr(stat, "file", None)
    wps = list(getattr(canon, "workpieces", None) or ())
    if not path or not wps or geom is None or getattr(geom, "is_empty", True):
        return 0
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            text = f.read().splitlines()
    except OSError as e:
        log.warning("%s: not read for its G20/G21 (%s)", path, e)
        return 0
    code = [NAMED_PARAM.sub("#", _without_comments(line)) for line in text]
    words = [(n, m.group(1) == "1") for n, line in enumerate(code, 1) for m in UNIT_WORD.finditer(line)]
    # the first move's line, from the text: the first line with a coordinate word (the geometry's
    # line labels are not the file's lines for every file, so they are not used for this)
    first_move = next((n for n, line in enumerate(code, 1) if AXIS_WORD.search(line)), len(code))
    before_move = [w for w in words if w[0] <= first_move]
    if not before_move:
        log.info("WORKPIECE: the program says no G20/G21 before its first move (line %d): its comments as the parse read them", first_move)
        return 0
    at_line, program_mm = before_move[-1]
    lu = float(getattr(stat, "linear_units", 1 / 25.4) or 1 / 25.4) * 25.4    # machine units per inch
    machine_mm = abs(lu - 25.4) < 1e-6
    if program_mm == machine_mm:
        log.info("WORKPIECE: the program moves in %s (G2%d at line %d, the first move at line %d), as the parse began: its comments as read",
                 "mm" if program_mm else "inches", 1 if program_mm else 0, at_line, first_move)
        return 0
    r = (1 / 25.4 if program_mm else 1.0) / (1 / 25.4 if machine_mm else 1.0)
    # where the program's origin is on the machine: the g5x offset, then the g92 offset rotated with
    # the program (what the preview's transform did to the comment's points, from the status)
    g5x = np.asarray((getattr(stat, "g5x_offset", None) or (0.0,) * 9)[:3], dtype=float)
    g92 = np.asarray((getattr(stat, "g92_offset", None) or (0.0,) * 9)[:3], dtype=float)
    a = math.radians(float(getattr(stat, "rotation_xy", 0.0) or 0.0))
    rot = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
    origin = (g5x + rot @ g92) / lu
    changed = 0
    for wp in wps:
        lineno = int(getattr(wp, "lineno", -1))
        if any(n < lineno for n, _ in words):
            continue                                            # the program set its units above the comment
        if 0 < lineno <= len(text) and "UNITS" in text[lineno - 1].upper():
            continue                                            # the comment says
        m = np.asarray(wp.machine_points, dtype=np.float64)
        p = np.asarray(wp.points, dtype=np.float64)
        m2 = origin + r * (m - origin)
        wp.points = m2 + (p - m) if p.shape == m.shape else p.mean(axis=0) + r * (p - p.mean(axis=0))
        wp.machine_points = m2
        for k, v in list(wp.params.items()):
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                wp.params[k] = v * r
        changed += 1
        log.info("WORKPIECE at line %d: read in %s (the parse begins in the machine's units) but the program moves in %s (G2%d at line %d, "
                 "the first move at line %d): its numbers taken as %s", lineno, "mm" if machine_mm else "inches", "mm" if program_mm else "inches",
                 1 if program_mm else 0, at_line, first_move, "mm" if program_mm else "inches")
    return changed


def program_key(path, geom):
    """What tells one loaded program from another: its path, its vertex count, its line numbers and
    a checksum of its vertices from the fourth on (the same file edited and loaded again differs;
    the display reloaded does not). The first vertices are left out: the path begins where the
    machine stood when the program was loaded, and a first move that gives only some axes ends
    there too, so they change from one load to the next."""
    if geom is None or getattr(geom, "is_empty", True):
        return (path, -1, 0.0)
    try:
        p = np.asarray(geom.positions(0), dtype=np.float64)[3:]
        lines = np.asarray(geom.lines, dtype=np.float64)
        digest = float(np.round(p.sum() + (p[:, 0] * np.arange(1, len(p) + 1)).sum() * 1e-3
                                + (lines * np.arange(1, len(lines) + 1)).sum() * 1e-6, 6)) if len(p) else 0.0
    except Exception:
        digest = 0.0
    return (path, int(len(geom)), digest)


def stock_box(canon, settings):
    """(lo, hi) of the Stock: the program's first (WORKPIECE,...) comment, else STOCK_BOX."""
    for wp in getattr(canon, "workpieces", ()) or ():
        pts = np.asarray(getattr(wp, "machine_points", None))
        if pts is not None and pts.ndim == 2 and len(pts):
            lo, hi = pts.min(axis=0), pts.max(axis=0)
            if np.all(hi > lo):
                return tuple(float(v) for v in lo), tuple(float(v) for v in hi)
    return settings.box


# ---------------------------------------------------------------------------- the part

class StockPart(glcanon_scene.Part):
    """The Stock as a part of the preview's scene (rs274.glcanon_scene)."""

    def __init__(self, settings):
        self.settings = settings
        self.visible = settings.enabled
        self.images = ToolImages(settings.tool_images, settings.tool_image_dir, settings.tool_table)
        self._ticks = 0
        self.stock = sim.Stock()
        self._canon = None
        self._program_key = None                               # (file, vertices) of the program loaded: the same one again is known
        self._lines = None
        self._tips = None
        self._flags = None
        self._frontier = (0, 0.0)                              # the executed path: whole segments, how far along the next
        self._clear_from = 0                                   # ... drawn from this segment on (the GUI's clear button)
        self._path_version = 0                                 # counts the programs given (DonePathPart's buffer follows)
        self._box = None
        self._running = False
        self._eof = False                                      # the run read its program to the end: its last moves in flight
        self._ignore_line = None                               # the status's executing line at a run's start: the last run's, until it changes
        self._ended_at = 0.0                                   # when the last run ended (a reload right after is gmoccapy's own)
        self._done = 0
        self._shader = None
        self._slots = {}
        self._generation = None
        self._was_busy = False
        self._stat = None                                      # the GUI's status, as last drawn with (tick)
        self._loaded = False
        if settings.stock_file:
            err, lo, hi, res = self.stock.load(settings.stock_file)
            if err:
                log.error("%s: %s", settings.stock_file, err)
            else:
                self._loaded = True
                self._box = (lo, hi)
                log.info("Stock file %s: %s to %s, resolution %g", settings.stock_file, lo, hi, res)

    # -- the program and the run --------------------------------------------

    def _define(self, box):
        if self._loaded:
            return                                             # the file's Stock is the Stock
        if box is None:
            if self._box is not None:
                self.stock.remove()
                self._box = None
            return
        if box != self._box:
            self.stock.define(box[0], box[1], self.settings.resolution)
            self._box = box

    def load_program(self, canon, stat):
        """A program was loaded in the widget: its path is the path to cut with."""
        new = canon is not self._canon
        geom = getattr(canon, "program_geometry", None)
        empty = geom is None or getattr(geom, "is_empty", True)
        key = program_key(getattr(stat, "file", None), geom)
        # the same program loaded again while its run stands (paused, the GUI gone to manual for a
        # jog; gmoccapy reloads the display then): the run goes on, nothing is cut again. Loaded
        # again as its run ends (an abort: the interpreter idle, gmoccapy reloads at once): the
        # run ends as it would have in _follow, stopped or complete
        # the same program again, during its run or within moments of its end: gmoccapy reloads
        # the display whenever the interpreter goes idle (a pause, an abort, the end). Loaded
        # again later, while idle, it is the user's reload: a fresh load, the whole program cut
        same = key == self._program_key and self._lines is not None and (self._running or time.time() - self._ended_at < 2.0)
        self._program_key = key
        if same:
            # the engine keeps its poses and its cut, as it would have shown the block whole and
            # then cut it again; the new canon's comments get their units; a run goes on, or ends
            # as it would have in _follow
            if new:
                try:
                    workpiece_units(canon, stat, geom)
                except Exception:
                    log.exception("the WORKPIECE comments' units")
            self._canon = canon
            if not self._running:
                log.info("the same program loaded again while idle: the Stock kept as it is")
            elif self._run_held(stat):
                log.info("the program loaded again while its run stands (line %s): it goes on from %d poses",
                         getattr(stat, "motion_line", "?"), self._done)
            elif self._run_on(stat):
                log.info("the program loaded again during its run (line %s): it goes on from %d poses",
                         getattr(stat, "motion_line", "?"), self._done)
                self._follow(stat)
            else:
                log.info("the program loaded again as its run ended")
                self._end_run(stat)
            return
        if new:
            self._frontier = (0, 0.0)                          # a new program: none of it is executed
            self._clear_from = 0
        self._canon = canon
        self._running = False
        self._done = 0
        self.stock.clear_poses()
        self._lines = self._tips = self._flags = None
        self._path_version += 1
        if empty:
            log.info("program: none or empty (canon %s); the Stock is %s", type(canon).__name__,
                     "the ini's box" if self.settings.box else "not defined")
            self._define(self.settings.box)
            return
        if new:
            try:
                workpiece_units(canon, stat, geom)             # once per program: it scales the comments in place
            except Exception:
                log.exception("the WORKPIECE comments' units")
        box = stock_box(canon, self.settings)
        spindle = int(getattr(stat, "tool_in_spindle", 0) or 0)
        log.info("program: %d vertices, lines %s..%s, tool changes %s, tool in the spindle %d, box %s", len(geom),
                 int(geom.lines.min()), int(geom.lines.max()), list(getattr(geom, "tool_numbers", []))[1:], spindle, box)
        self._define(box)
        diameters = {}
        for t in getattr(canon, "tools", ()) or ():
            try:
                if t.id > 0:
                    diameters[int(t.id)] = float(t.diameter)
            except AttributeError:
                pass
        try:
            fed = program_poses(geom, self.stock, self.images, diameters, self.settings.default_diameter,
                                self._piece(stat), spindle)
        except Exception:
            log.exception("the program could not be given to the Stock")
            fed = None
        if fed is None:
            log.info("program: nothing given to the Stock")
            return
        self._lines, self._tips, self._flags = fed
        log.info("program: %d poses given (pieces of %.3g) with tools %s, tool table diameters %s, defined %s", self.stock.nb_poses(),
                 self._piece(stat), sorted(set(vertex_tools(geom, spindle).tolist())), diameters, self.stock.defined())
        if self.settings.follow and self._run_on(stat):
            log.info("a run is on: following it")
            self._start_run(stat)
            self._follow(stat)
        else:
            self.stock.cut_all()
            log.info("cut all")

    def _piece(self, stat):
        """The longest piece of a move: STOCK_PIECE, else 0.05 inch in the machine's units."""
        if self.settings.piece is not None:
            try:
                return float(self.settings.piece)
            except ValueError:
                log.error("[KMOTION] STOCK_PIECE is not a number: %r", self.settings.piece)
        lu = getattr(stat, "linear_units", 0) or 0
        return 0.05 * (25.4 * lu if lu else 1.0)

    @staticmethod
    def _motion_pending(stat):
        """Moves of a program still queued, or a feed hold: the run is not over (an abort empties
        the queue; the executing line number is no sign, it keeps its last value after an abort)."""
        try:
            return int(getattr(stat, "queue", 0) or 0) > 0 or bool(getattr(stat, "paused", False))
        except (TypeError, ValueError):
            return False

    @classmethod
    def _run_on(cls, stat):
        """A program is executing, or paused in auto: task in MODE_AUTO with the interpreter not
        idle, or with its moves still queued (task reports the interpreter idle for long stretches
        of a run here)."""
        try:
            return stat.task_mode == linuxcnc.MODE_AUTO and (stat.interp_state != linuxcnc.INTERP_IDLE or cls._motion_pending(stat))
        except AttributeError:
            return False

    @classmethod
    def _run_held(cls, stat):
        """A run standing outside auto: paused with the GUI gone to manual for a jog (task then
        reports MODE_MANUAL and the interpreter idle), its moves still queued."""
        try:
            return stat.task_mode != linuxcnc.MODE_AUTO and cls._motion_pending(stat)
        except AttributeError:
            return False

    _is_running = _run_on

    def _start_run(self, stat=None):
        self._running = True
        self._eof = False
        self._done = 0
        self._frontier = (0, 0.0)
        self._clear_from = 0
        # task reports the last run's executing line until the new run's first move: not progress
        self._ignore_line = (int(getattr(stat, "motion_line", 0) or 0) or None) if stat is not None else None
        self.stock.uncut_from(0)

    def _end_run(self, stat=None):
        """The run is over: complete (the program read to its end, or the path at its last
        segments) and the whole program is cut; stopped otherwise, and the cut and the executed
        path stay where they are - after a last look at where the tool came to rest, since the
        status's line and the tool's position catch up with each other only once it stands."""
        self._running = False
        self._ended_at = time.time()
        if stat is not None and self._lines is not None:
            try:
                done, front = self._progress(stat)
                self._frontier = max(self._frontier, front)
                if done > self._done:
                    self._done = done
                    self.stock.cut_to(done)
            except Exception:
                log.exception("the last look at the run")
        n = len(self._tips) if self._tips is not None else 0
        if self._eof or (n and self._frontier[0] >= n - 2):
            self.stock.cut_all()                               # the moves after the last line seen are cut too
            log.info("run complete: cut all (it had come to %d poses of %d)", self._done, n)
        else:
            log.info("run stopped at %d poses of %d: left as cut, the executed path to there", self._done, n)

    def clear_executed(self):
        """The GUI's clear-plot button: the executed path drawn so far is hidden (a run that goes
        on draws from here), the cut stays as it is."""
        if self._clear_from != self._frontier[0]:
            self._clear_from = self._frontier[0]
            log.info("the executed path cleared (hidden up to segment %d)", self._clear_from)
            return True
        return False

    def executed_range(self):
        """What of the executed path is drawn: (from segment, segments, fraction of the next)."""
        segments, fraction = self._frontier
        return min(self._clear_from, segments), segments, fraction

    def _tolerance(self, stat):
        # TOOL_ON_PATH_TOL in the machine's units (stat.linear_units: machine units per mm)
        lu = getattr(stat, "linear_units", 0) or 0
        return TOOL_ON_PATH_TOL_INCH * (25.4 * lu if lu else 1.0)

    def _tool_on_line(self, stat, ml):
        """Whether the tool tip stands on one of the pieces of line ml (within the tolerance)."""
        try:
            pos, off = stat.actual_position, stat.tool_offset
            tip = np.array([pos[0] - off[0], pos[1] - off[1], pos[2] - off[2]], dtype=np.float32)
        except (AttributeError, TypeError, IndexError):
            return False
        idx = np.nonzero(self._lines == ml)[0]
        if not len(idx):
            return False
        lo, hi = max(int(idx[0]) - 1, 0), int(idx[-1])
        if hi <= lo:
            return float(np.linalg.norm(self._tips[hi] - tip)) <= self._tolerance(stat)
        a, b = self._tips[lo:hi], self._tips[lo + 1:hi + 1]
        ab = b - a
        ab2 = np.einsum("ij,ij->i", ab, ab)
        t = np.clip(np.einsum("ij,ij->i", tip - a, ab) / np.where(ab2 > 0, ab2, 1), 0, 1)
        return float(np.linalg.norm(a + ab * t[:, None] - tip, axis=1).min()) <= self._tolerance(stat)

    def _progress(self, stat):
        """How far the run has come, from the status: (poses, frontier).  poses: how many poses the
        run has cut - all of the lines before the executing one, and on it the piece that the tool
        is on.  frontier: how far along the path the tool is, for the executed path - (segments,
        fraction), the whole segments behind it (segment i joins pose i to pose i + 1) and how far
        along the next one it is.  Neither goes back: a line number seen again (a loop, a late
        status) takes nothing away."""
        lines = self._lines
        n = len(lines)
        done, front = self._done, self._frontier
        ml = int(getattr(stat, "motion_line", 0) or 0)
        if ml <= 0:
            return done, front
        if self._ignore_line is not None:
            if ml != self._ignore_line:
                self._ignore_line = None
            elif not self._tool_on_line(stat, ml):
                return done, front                             # the last run's line, still reported, the tool elsewhere
            else:
                self._ignore_line = None                       # the same line again, and the tool on it: a real one
        # the executing line's poses, from the piece the tool was last on (done - 1 is its end):
        # the tool may still be on it
        d = max(done - 1, 0)
        hits = np.nonzero(lines[d:] == ml)[0]
        if not len(hits):
            return done, front                                 # not reached, or already past
        first = d + int(hits[0])
        last = first
        while last + 1 < n and lines[last + 1] == ml:
            last += 1
        count = first                                          # the lines before are done ...
        front = max(front, (max(first - 1, 0), 0.0))           # ... and the path up to this line's start
        try:
            pos = stat.actual_position
            off = stat.tool_offset
            tip = np.array([pos[0] - off[0], pos[1] - off[1], pos[2] - off[2]], dtype=np.float32)
        except (AttributeError, TypeError, IndexError):
            return max(done, count), front
        # the pieces of the line: from the vertex before its first one to its last
        found = False
        lo = max(first - 1, 0)
        if last > lo:
            a = self._tips[lo:last]
            b = self._tips[lo + 1:last + 1]
            ab = b - a
            ab2 = np.einsum("ij,ij->i", ab, ab)
            t = np.clip(np.einsum("ij,ij->i", tip - a, ab) / np.where(ab2 > 0, ab2, 1), 0, 1)
            near = a + ab * t[:, None]
            dist = np.linalg.norm(near - tip, axis=1)
            k = int(np.argmin(dist))
            if dist[k] <= self._tolerance(stat):
                count = lo + k + 2                             # the piece the tool is on is done
                front = max(front, (lo + k, float(t[k])))      # the path is executed as far as the tool
                found = True
        elif np.linalg.norm(self._tips[first] - tip) <= self._tolerance(stat):
            count = first + 1
            found = True
        # the status's executing line trails the tool (about a second here, with the planner's
        # knots): a tool not on that line but on a piece a little further along the path has
        # executed the path to there - the nearest piece within the tolerance among the next ones
        lo = max(count - 1, 0)
        hi = min(lo + LOOKAHEAD_POSES, n - 1)
        if not found and hi > lo:
            a = self._tips[lo:hi]
            b = self._tips[lo + 1:hi + 1]
            ab = b - a
            ab2 = np.einsum("ij,ij->i", ab, ab)
            t = np.clip(np.einsum("ij,ij->i", tip - a, ab) / np.where(ab2 > 0, ab2, 1), 0, 1)
            dist = np.linalg.norm(a + ab * t[:, None] - tip, axis=1)
            k = int(np.argmin(dist))
            if dist[k] <= self._tolerance(stat):
                count = max(count, lo + k + 2)
                front = max(front, (lo + k, float(t[k])))
        return max(done, min(count, n)), front

    @property
    def frontier(self):
        """How far along the path the run has come: (segments, fraction) - the whole segments
        executed (segment i joins pose i to pose i + 1) and how far along the next one the tool is."""
        return self._frontier

    def frontier_point(self):
        """Where on the path the run has come to - the end of the executed path - or None before it
        has moved."""
        segments, f = self._frontier
        tips = self._tips
        if tips is None or len(tips) == 0 or (segments == 0 and f <= 0):
            return None
        if segments + 1 >= len(tips):
            return tips[-1]
        a, b = tips[segments], tips[segments + 1]
        return a + f * (b - a)

    def _follow(self, stat):
        """The run as the status tells it: True when the engine was told something."""
        if not self.settings.follow or self._lines is None:
            return False
        told = False
        running = self._run_on(stat)
        if running and not self._running:
            self._start_run(stat)
            log.info("run started (the status still at line %s): the Stock is uncut again", getattr(stat, "motion_line", "?"))
            told = True
        if running:
            done, front = self._progress(stat)
            if front != self._frontier:
                self._frontier = front                         # the executed path grew: draw again
                told = True
            if done > self._done:
                self._done = done
                self.stock.cut_to(done)
                told = True
            # the program read to its end (task clears the line numbers there and reports the
            # interpreter waiting for the last moves): the run that ends now has completed, not been
            # stopped. An abort clears the lines too, with the interpreter idle
            if (self._done > 0 and not self._eof and int(getattr(stat, "motion_line", 0) or 0) == 0
                    and int(getattr(stat, "read_line", 1) or 0) == 0
                    and getattr(stat, "interp_state", None) == linuxcnc.INTERP_WAITING):
                self._eof = True
        elif self._running and self._run_held(stat):
            pass                                               # paused, the GUI in manual (a jog): the run stands
        elif self._running:
            self._end_run(stat)
            told = True
        return told

    def tick(self):
        """Called on a timer by install(): True when the picture should be drawn again (the
        engine is cutting, has meshes waiting to be drawn, or the run changed).  The run is followed here
        too, not only when the GUI redraws: a run that ends, or is refused before it moves,
        changes nothing a GUI redraws on, and the Stock would stay as it was."""
        told = False
        self._ticks += 1
        if self._ticks % 10 == 0 and self.images.changed():
            told = self.refeed("the tool table or the tool images table changed") or told
        if self._stat is not None:
            told = self._follow(self._stat) or told
        busy = self.stock.busy()
        pending = self.stock.pending() > 0                     # meshes handed over since the last frame
        if self._was_busy and not busy:
            try:
                st = self.stock.stats()
                log.info("engine idle: %d poses cut, lattice %s (step %.4g), %.0f triangles, %.1f MB, volume %.4f, failed %s, %d meshes waiting",
                         st["applied"], st["nodes"], st["step"][0], st["triangles"], st["bytes"] / 1e6, st["volume"][2],
                         self.stock.failed(), self.stock.pending())
            except Exception:
                log.exception("stats")
        again = busy or self._was_busy or told or pending
        self._was_busy = busy
        return again

    # -- drawing --------------------------------------------------------------

    @contextmanager
    def scope(self, ctx):
        glEnable(GL_POLYGON_OFFSET_FILL)
        glPolygonOffset(0.0, 1.0)
        glDisable(GL_CULL_FACE)
        try:
            yield
        finally:
            glEnable(GL_CULL_FACE)
            glDisable(GL_POLYGON_OFFSET_FILL)

    def invalidate(self):
        """The GL context is new: forget the buffers and ask for the surface again."""
        self._delete_slots()
        self._shader = None
        self._generation = None
        self.stock.republish()

    def _ensure_shader(self, caps):
        if self._shader is None:
            self._shader = glcanon_gl.ShaderProgram(VERTEX_SHADER, FRAGMENT_SHADER, caps)
        return self._shader

    def _delete_slots(self):
        for slot in list(self._slots.values()):
            self._delete_slot(slot)
        self._slots.clear()

    @staticmethod
    def _delete_slot(slot):
        vao, vbo, ebo, _count = slot
        try:
            glDeleteVertexArrays(1, [vao])
            glDeleteBuffers(2, [vbo, ebo])
        except Exception:
            pass

    def _upload(self):
        """Take the meshes the engine has changed since the last frame into GL buffers.  Raw GL
        calls: the renderer's buffer helper takes a plain array as float32, and these are 20 byte
        records and 16 bit indices."""
        gen = self.stock.generation()
        if gen != self._generation:
            self._delete_slots()
            self._generation = gen
        taken = 0
        for index, verts, tris in self.stock.take_meshes():
            taken += 1
            old = self._slots.pop(index, None)
            if old is not None:
                self._delete_slot(old)
            if len(tris) == 0:
                continue
            vbytes = np.ascontiguousarray(verts, dtype=np.uint8)
            ibytes = np.ascontiguousarray(tris, dtype=np.uint16)
            vao = glGenVertexArrays(1)
            vbo, ebo = glGenBuffers(2)
            glBindVertexArray(vao)
            glBindBuffer(GL_ARRAY_BUFFER, vbo)
            glBufferData(GL_ARRAY_BUFFER, vbytes.nbytes, vbytes, GL_STATIC_DRAW)
            glEnableVertexAttribArray(0)
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTES, ctypes.c_void_p(0))
            glEnableVertexAttribArray(1)
            glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, VERTEX_BYTES, ctypes.c_void_p(12))
            glEnableVertexAttribArray(2)
            glVertexAttribPointer(2, 4, GL_INT_2_10_10_10_REV, GL_TRUE, VERTEX_BYTES, ctypes.c_void_p(16))
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo)
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, ibytes.nbytes, ibytes, GL_STATIC_DRAW)
            glBindVertexArray(0)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)
            self._slots[index] = (vao, vbo, ebo, len(ibytes))
            if _debug_file:
                v = np.frombuffer(vbytes.tobytes(), dtype=np.dtype([("xyz", np.float32, 3), ("kind", np.float32), ("dir", np.uint32)]))
                _debug("slot %d: %d vertices, %d indices (max %d), xyz min %s max %s, kinds %s" % (
                    index, len(v), len(ibytes), int(ibytes.max()), v["xyz"].min(axis=0), v["xyz"].max(axis=0),
                    sorted(set(np.round(v["kind"]).astype(int).tolist()))))

    @staticmethod
    def push_distance(projection, modelview, height, line_width):
        """How far back the Stock goes, in world units, so that path lines of this width drawn at
        the surface show on it: 1.5 line widths, as a distance at the Stock.  KMotionCNC pulls the
        lines nearer by the same amount (Path3d.cpp)."""
        p = np.asarray(projection, dtype=np.float64)
        mv = np.asarray(modelview, dtype=np.float64)
        height = max(float(height), 1.0)
        if abs(p[3, 2]) > 0.5:                                 # perspective: units per pixel grow with the distance
            dist = abs(p[3, 3]) + abs(mv[2, 3])                # (glnav folds the eye's distance into the projection)
            per_pixel = 2.0 * max(dist, 1e-6) / (abs(p[1, 1]) * height)
        else:                                                  # orthographic
            per_pixel = 2.0 / (abs(p[1, 1]) * height)
        return 1.5 * line_width * per_pixel

    def sync(self, ctx):
        """The program and the run as the frame has them.  draw() does this, and so does
        DonePathPart's: with the Stock hidden by its toggle the run is still followed."""
        self._stat = ctx.stat
        if ctx.canon is not self._canon:
            self.load_program(ctx.canon, ctx.stat)
        self._follow(ctx.stat)

    def draw(self, ctx):
        self.sync(ctx)
        before = len(self._slots)
        self._upload()
        self._frames = getattr(self, "_frames", 0) + 1
        if len(self._slots) != before or self._frames in (1, 10, 100):
            log.info("frame %d: %d meshes drawn (generation %s)", self._frames, len(self._slots), self._generation)
        if not self._slots:
            return
        shader = self._ensure_shader(ctx.caps)
        mv = np.asarray(ctx.mv.top(), dtype=np.float64)
        proj = np.asarray(ctx.mv.projection, dtype=np.float64)
        m3 = mv[:3, :3]
        try:
            normal = np.linalg.inv(m3).T
        except np.linalg.LinAlgError:
            normal = np.identity(3)
        push = self.push_distance(proj, mv, ctx.height, self.settings.line_width)
        away = -mv[2, :3]
        away = away / (np.linalg.norm(away) or 1.0)
        if _debug_file and not getattr(self, "_debug_frames", 0) % 50:
            _debug("frame: height %s, push %g along %s\n  mv %s\n  proj %s" % (ctx.height, push, away.round(3).tolist(), mv.tolist(), proj.tolist()))
        if _debug_file and not getattr(self, "_debug_frames", 0) % 10:
            _debug("state %.1f: generation %s, %d slots, poses %d, running %s, done %d, busy %s" % (
                time.time() % 1000, self._generation, len(self._slots), self.stock.nb_poses(), self._running, self._done, self.stock.busy()))
        self._debug_frames = getattr(self, "_debug_frames", 0) + 1
        shader.use()
        shader.set_mat4("u_mvp", proj @ mv)
        shader.set_mat4("u_mv", mv)
        glUniformMatrix3fv(shader.uniform("u_normal"), 1, GL_TRUE, np.ascontiguousarray(normal, dtype=np.float32))
        glUniform3f(shader.uniform("u_away"), *away)
        glUniform1f(shader.uniform("u_push"), float(push))
        glUniform3f(shader.uniform("u_light"), *LIGHT)
        glUniform3f(shader.uniform("u_grey"), *GREY)
        glUniform3f(shader.uniform("u_blue"), *BLUE)
        glUniform3f(shader.uniform("u_red"), *RED)
        for vao, _vbo, _ebo, count in self._slots.values():
            glBindVertexArray(vao)
            glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_SHORT, None)
        glBindVertexArray(0)
        glUseProgram(0)

    def refeed(self, why=""):
        """The tool images may have changed: read the tables again, forget the image files the engine
        has read, and give the program to the engine once more.  True when a program was given."""
        self.images.reload()
        self.stock.forget_tool_images()
        log.info("tools read again (%s): table %s, TOOL_IMAGES %s", why, self.images.from_table, self.images.by_number)
        if self._canon is None or self._stat is None:
            return False
        self.load_program(self._canon, self._stat)
        return True

    # -- for a GUI's buttons -----------------------------------------------------

    def set_visible(self, on):
        self.visible = bool(on)

    def save(self, file, wait=6):
        """Save the Stock as it is cut now (None, or what went wrong)."""
        return self.stock.save(file, wait)

    def load(self, file):
        """A Stock file in place of the Stock: every path cuts into it from now on."""
        err, lo, hi, res = self.stock.load(file)
        if not err:
            self._loaded = True
            self._box = (lo, hi)
        return err

    def unload(self):
        self.stock.unload()
        self._loaded = False
        self._box = None
        if self._canon is not None:
            self._define(stock_box(self._canon, self.settings))


# ---------------------------------------------------------------------------- the executed path

class DonePathPart(glcanon_scene.Part):
    """The executed path: the part of the program's path the run has come to, drawn over the
    program's lines (after the program and its highlight in the scene) from the Stock's poses and
    its frontier - the path's own geometry, not samples of the position, so a stall of the status
    pauses it instead of cutting across what was missed.  Feeds in the live plot's feed colour,
    rapids in its traverse colour (the GUI's backplotfeed and backplottraverse), a little wider than
    the program's lines, at their depth with the live plot's bias so that it wins the tie.

    The whole path sits in one buffer in path order, as GL_LINES in the renderer's per-vertex-colour
    layout (done_path_vertices), made when the program or the colours change.  A frame draws its
    first segments, as many as the frontier says, through the renderer's line shader - or, where the
    driver refused the width, its quad-expanding one, the buffer bound as endpoint pairs, as the
    renderer draws its own wide lines - and the piece the tool is on up to the tool as a two point
    line of its own."""

    def __init__(self, stock):
        self.stock = stock
        self._buffer = None                                    # glcanon_gl.GLBuffer: the segments
        self._vao = None                                       # ... as GL_LINES
        self._wide_vao = None                                  # ... as endpoint pairs, one instance a segment
        self._wide_first = 0                                   # ... from this segment on
        self._segments = 0
        self._version = None                                   # (the Stock's program, the colours) the buffer holds

    @contextmanager
    def scope(self, ctx):
        glDepthFunc(GL_LEQUAL)
        glcanon_gl.set_line_width(DONE_PATH_WIDTH)
        try:
            yield
        finally:
            glUseProgram(0)
            glBindVertexArray(0)
            glcanon_gl.set_line_width(1.0)
            glDepthFunc(GL_LESS)

    def invalidate(self):
        """The GL context is new: forget the buffer."""
        self._delete()

    def _delete(self):
        for obj in (self._vao, self._wide_vao, self._buffer):
            if obj is not None:
                try:
                    obj.delete()
                except Exception:
                    pass
        self._vao = self._wide_vao = self._buffer = None
        self._segments = 0
        self._version = None

    @staticmethod
    def colours(colors):
        """(feed, rapid) as rgba from the GUI's colour table: the live plot's colours, opaque."""
        return (tuple(colors.get("backplotfeed", DONE_FEED))[:3] + (1.0,),
                tuple(colors.get("backplottraverse", DONE_RAPID))[:3] + (1.0,))

    def _ensure(self, ctx):
        """The buffer holding the program's path, made again when the program or the colours changed."""
        stock = self.stock
        colours = self.colours(ctx.colors)
        version = (stock._path_version, colours)
        if version == self._version:
            return
        self._delete()
        self._version = version
        if stock._tips is None or len(stock._tips) < 2:
            return
        verts = done_path_vertices(stock._tips, stock._flags, *colours)
        self._buffer = glcanon_gl.GLBuffer()
        self._buffer.set_data(verts)
        self._vao = glcanon_gl.VertexArray()
        self._vao.configure(self._buffer)
        self._segments = len(verts) // 2
        log.info("executed path: %d segments in the buffer", self._segments)

    def _wide(self, first=0):
        """The buffer as segment endpoint pairs for the quad-expanding shader, from segment first
        on: bound twice, a vertex apart, one instance a segment (as glcanon_gl.CategoryBuffers
        presents its own)."""
        if self._wide_vao is None or self._wide_first != first:
            if self._wide_vao is not None:
                self._wide_vao.delete()
            self._wide_vao = glcanon_gl.VertexArray()
            self._wide_first = first
            step = 2 * glcanon_gl.VERTEX_STRIDE
            self._wide_vao.configure(self._buffer, glcanon_gl.WIDE_LINE_A_ATTRIBUTES, step, divisor=1, base=first * step)
            self._wide_vao.configure(self._buffer, glcanon_gl.WIDE_LINE_B_ATTRIBUTES, step, divisor=1,
                                     base=first * step + glcanon_gl.VERTEX_STRIDE)
        return self._wide_vao

    @staticmethod
    def mvp(ctx):
        """The frame's MVP with the live plot's small eye space bias toward the viewer
        (glcanon_scene.BackplotPart.mvp), so that the path wins the depth tie with the program's lines."""
        bias = np.identity(4)
        bias[2, 3] = 0.003
        return ctx.mv.projection @ bias @ ctx.mv.top()

    def draw(self, ctx):
        stock = self.stock
        stock.sync(ctx)
        self._ensure(ctx)
        first, segments, fraction = stock.executed_range()
        n = min(segments, self._segments)
        first = min(first, n)
        mvp = self.mvp(ctx)
        renderer = ctx.renderer
        if n > first:
            width = renderer.expansion_width()
            if width:
                wide = renderer.wide_line_program()
                wide.begin(mvp)
                wide.set_expansion(renderer.viewport, width)
                vao = self._wide(first)
                vao.bind()
                glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, n - first)
                vao.unbind()
            else:
                renderer.line_program().begin(mvp)
                self._vao.bind()
                glDrawArrays(GL_LINES, 2 * first, 2 * (n - first))
                self._vao.unbind()
        end = stock.frontier_point()
        if end is not None and fraction > 0 and segments < self._segments and segments >= first:
            flags = int(stock._flags[segments + 1])
            if flags & sim.SIMPOSE_MOVE:
                feed, rapid = self.colours(ctx.colors)
                colour = rapid if flags & sim.SIMPOSE_RAPID else feed
                ctx.prim.draw_lines(ctx, [stock._tips[segments], end], colour[:3], mvp=mvp)


# ---------------------------------------------------------------------------- the tool, drawn from its image

def simplify_profile(h, r, tol):
    """The profile with points that lie within tol of the line between their neighbours left out
    (Douglas-Peucker), so that a tool image of hundreds of points draws as a few dozen rings."""
    pts = list(zip(h, r))
    if len(pts) <= 2:
        return pts
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        a, b = stack.pop()
        (ha, ra), (hb, rb) = pts[a], pts[b]
        dx, dy = hb - ha, rb - ra
        length = math.hypot(dx, dy) or 1e-12
        far, at = 0.0, -1
        for i in range(a + 1, b):
            hi, ri = pts[i]
            d = abs(dx * (ri - ra) - dy * (hi - ha)) / length
            if d > far:
                far, at = d, i
        if at >= 0 and far > tol:
            keep[at] = True
            stack.append((a, at))
            stack.append((at, b))
    return [pt for pt, k in zip(pts, keep) if k]


def revolve_profile(h, r, open_above, slices=24, tol=None):
    """A tool's profile (heights above the tip, radii) as a surface of revolution about Z: float32
    (N, 6) position and normal triangles, wound counter-clockwise seen from outside, as LinuxCNC's
    glcanon_bake.cylinder_mesh is.  The tip is at z = 0.  A flat tip and the top are capped: an
    open tool (one the engine takes as going on upward) ends at its last profile point, the image's
    length, and is drawn solid to there - uncapped, its inside faces were culled and the tool was
    see-through from above."""
    h = [float(v) for v in h]
    r = [float(v) for v in r]
    if len(h) < 2:
        return np.zeros((0, 6), dtype=np.float32)
    if tol is None:
        tol = 0.01 * max(max(r), 1e-9)
    pts = simplify_profile(h, r, tol)
    cs = [(math.cos(2 * math.pi * i / slices), math.sin(2 * math.pi * i / slices)) for i in range(slices + 1)]
    out = []

    def ring(z, rad, i):
        c, sn = cs[i]
        return rad * c, rad * sn, z

    for (h0, r0), (h1, r1) in zip(pts[:-1], pts[1:]):
        dh, dr = h1 - h0, r1 - r0
        length = math.hypot(dh, dr)
        if length <= 0:
            continue
        nr, nz = dh / length, -dr / length            # outward: the segment's direction turned a quarter
        for i in range(slices):
            c0, s0 = cs[i]
            c1, s1 = cs[i + 1]
            n0 = (nr * c0, nr * s0, nz)
            n1 = (nr * c1, nr * s1, nz)
            b0 = ring(h0, r0, i) + n0
            b1 = ring(h0, r0, i + 1) + n1
            t0 = ring(h1, r1, i) + n0
            t1 = ring(h1, r1, i + 1) + n1
            if r1 > 0:
                out.append(b0); out.append(t1); out.append(t0)
            if r0 > 0:
                out.append(b0); out.append(b1); out.append(t1)
    if pts[0][1] > 0:                                   # a flat tip: a disk facing down
        z, rad = pts[0]
        for i in range(slices):
            out.append((0.0, 0.0, z, 0.0, 0.0, -1.0))
            out.append(ring(z, rad, i + 1) + (0.0, 0.0, -1.0))
            out.append(ring(z, rad, i) + (0.0, 0.0, -1.0))
    if pts[-1][1] > 0:                                  # the top: a disk facing up
        z, rad = pts[-1]
        for i in range(slices):
            out.append((0.0, 0.0, z, 0.0, 0.0, 1.0))
            out.append(ring(z, rad, i) + (0.0, 0.0, 1.0))
            out.append(ring(z, rad, i + 1) + (0.0, 0.0, 1.0))
    return np.array(out, dtype=np.float32).reshape(-1, 6)


class ImageToolPart(glcanon_scene.ToolPart):
    """LinuxCNC's tool marker, drawing the tool in the spindle as its tool image where it has one:
    the image's profile spun about the tool's axis, at the marker's place, with the marker's
    lighting and transparency.  A tool without an image, and a lathe, get LinuxCNC's own marker."""

    def __init__(self, images):
        self.images = images
        self._meshes = {}

    def mesh_for(self, tool):
        path = self.images.image(tool) if tool else ""
        if not path:
            return None
        try:
            key = (path, os.path.getmtime(path))
        except OSError:
            return None
        if key not in self._meshes:
            try:
                h, r, open_above = sim.read_tool_image(path)
                self._meshes[key] = revolve_profile(h, r, open_above)
                log.info("tool %s drawn from %s: %d profile points", tool, os.path.basename(path), len(h))
            except Exception:
                log.exception("the tool image %s could not be drawn", path)
                self._meshes[key] = None
        return self._meshes[key]

    def _draw_at_tool(self, ctx, pos, rx, ry, rz):
        mesh = None
        if not ctx.is_lathe:
            try:
                mesh = self.mesh_for(int(getattr(ctx.stat, "tool_in_spindle", 0) or 0))
            except Exception:
                mesh = None
        if mesh is None or len(mesh) == 0:
            return glcanon_scene.ToolPart._draw_at_tool(self, ctx, pos, rx, ry, rz)
        ctx.mv.translate(*pos)
        self._apply_rotary(ctx, rx, ry, rz)
        # solid, not LinuxCNC's translucent cone: the image is a real tool, and a see-through cap
        # over its own inside read as a hole. The tool's scope blends source plus tool_alpha times
        # what is behind (GL_ONE, GL_CONSTANT_ALPHA), a brightening translucency; a constant alpha
        # of 1 there would add the background whole and turn the tool white over the Stock, so the
        # source replaces instead. A shade darker than the cone's colour, as the shader's light
        # (ambient 0.6 plus diffuse 0.6) saturates a white material on every face turned even half
        # toward the light, top and sides alike flat white
        glBlendFunc(GL_ONE, GL_ZERO)
        ctx.prim.draw_cone(ctx, tuple(0.72 * c for c in ctx.colors["cone"][:3]), mesh_verts=mesh)


# ---------------------------------------------------------------------------- hooking in

def _want_axis_positions():
    """Have every parse record the machine's axis positions at each vertex (the renderer does so
    only when asked, before the parse): the tool direction comes from the A, B, C columns."""
    cls = rs274.glcanon.GLCanon
    if getattr(cls, "_kmotion_stock_axes", False):
        return
    original = cls.configure_program_geometry

    def configure(self, geometry, ro, is_foam, want_axis_positions=False):
        return original(self, geometry, ro, is_foam, want_axis_positions=True)

    cls.configure_program_geometry = configure
    cls._kmotion_stock_axes = True


def install(widget, inifile, ini_dir=None):
    """Put the Stock into the preview widget (an rs274.glcanon.GlCanonDraw: gremlin, AXIS's
    canvas, QtVCP's graphics).  Returns the StockPart, also kept as widget.kmotion_stock."""
    settings = Settings(inifile, ini_dir or os.getcwd())
    if settings.rotary:
        _want_axis_positions()
    part = StockPart(settings)
    scene = widget.scene
    scene.insert_before(scene.program, part, lambda ctx: part.visible)
    widget.kmotion_stock = part
    clear = getattr(widget, "clear_live_plotter", None)       # the GUI's clear-plot button clears the executed path too
    if callable(clear):
        def clear_plots(*a, **k):
            if part.clear_executed():
                redraw = getattr(widget, "queue_draw", None) or getattr(widget, "tkRedraw", None)
                if redraw:
                    redraw()
            return clear(*a, **k)
        widget.clear_live_plotter = clear_plots
    if settings.tool_shape and getattr(scene, "tool", None) is not None:
        part.tool_part = ImageToolPart(part.images)
        scene.replace(scene.tool, part.tool_part)      # the gate (show_tool) stays
    if settings.enabled and settings.done_path:
        part.done_path = DonePathPart(part)            # over the program's lines, whatever the Stock's toggle says
        scene.insert_after(getattr(scene, "highlight", None) or scene.program, part.done_path)
    # the dimension lines follow their toggle while the Stock shows the part. LinuxCNC draws
    # them, and a plain box, whatever the toggle says once the program's lines are hidden (the
    # Paths toggle), as a stand-in for the hidden program (PreviewScene.program_outline); with
    # the part itself in view the stand-in only overrides the dimensions toggle
    outline = getattr(type(scene), "program_outline", None)
    extents, box = getattr(scene, "extents", None), getattr(scene, "bounding_box", None)
    if outline is not None and extents is not None and box is not None:
        def dimensions(ctx):
            return bool(ctx.show_extents) if part.visible and ctx.canon is not None else outline(ctx)[0]

        def plain_box(ctx):
            return False if part.visible else outline(ctx)[1]
        scene.regate(extents, dimensions)
        scene.regate(box, plain_box)

    # redraws while the engine cuts: GTK (gremlin) or Tk (AXIS); another toolkit calls part.tick()
    # on a timer of its own and redraws when it says so
    if hasattr(widget, "queue_draw"):
        from gi.repository import GLib

        def gtk_tick():
            try:
                if part.tick():
                    widget.queue_draw()
            except Exception:
                log.exception("tick")                          # the timer stays (an exception would end it)
            return True
        GLib.timeout_add(100, gtk_tick)
    elif hasattr(widget, "after") and hasattr(widget, "tkRedraw"):
        def tk_tick():
            try:
                if part.tick():
                    widget.tkRedraw()
            except Exception:
                log.exception("tick")
            widget.after(100, tk_tick)
        widget.after(100, tk_tick)
    log.info("Stock installed in %s (%s): resolution %g, follow %s, tool images %s, tool drawn from its image %s, executed path %s, redraws by %s",
             type(widget).__name__, type(widget).__module__, settings.resolution, settings.follow, settings.tool_images or "none",
             settings.tool_shape, settings.done_path, "GLib" if hasattr(widget, "queue_draw") else "Tk" if hasattr(widget, "after") else "nobody")
    return part
