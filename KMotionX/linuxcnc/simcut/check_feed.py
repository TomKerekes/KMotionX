# Copyright (c) 2026 DynoMotion Incorporated.  SPDX-License-Identifier: MIT
"""make check-feed: stock-demo.ngc parsed by LinuxCNC's interpreter (the gcode module, as the
preview parses it) and given to the engine through kmotion_stock, without a GUI or a running
LinuxCNC.  Checks that the preview positions are the tool tip (a tool length offset in force), the Stock box from
the WORKPIECE comment, the material removed, and a followed run: the cut and the executed path's
frontier as the run proceeds, after it ends, and at a new start.  LINUXCNC_DIR (or ~/linuxcnc-dev) gives the
run-in-place tree whose lib/python has gcode and linuxcnc.  Run it while LinuxCNC is not running: the
interpreter attaches to a running LinuxCNC's tool table memory and crashes in it."""
import os
import sys
import tempfile
from collections import namedtuple

HERE = os.path.dirname(os.path.abspath(__file__))
LINUXCNC_DIR = os.environ.get("LINUXCNC_DIR", os.path.expanduser("~/linuxcnc-dev"))
sys.path.insert(0, os.path.join(LINUXCNC_DIR, "lib", "python"))
sys.path.insert(0, HERE)

import numpy as np
import gcode
import rs274.glcanon
from rs274 import glcanon_bake
import kmotion_stock
import kmotion_simcut as sim

Tool = namedtuple("Tool", "id xoffset yoffset zoffset aoffset boffset coffset uoffset voffset woffset "
                          "diameter frontangle backangle orientation")
EMPTY = Tool(-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
TOOLS = [EMPTY, Tool(1, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0)]      # T1: 1/4 inch, length offset 0.5


class Progress:
    def nextphase(self, *a): pass
    def update(self, *a): pass
    def done(self, *a): pass


class Canon(rs274.glcanon.GLCanon):
    """The preview's canon with a tool table and machine facts of its own (what rs274.interpret.StatMixin
    takes from a running LinuxCNC)."""

    def __init__(self):
        rs274.glcanon.GLCanon.__init__(self, rs274.glcanon.GlCanonDraw.colors, "XYZ")
        self.tools = list(TOOLS)
        self.progress = Progress()

    def change_tool(self, pocket):
        rs274.glcanon.GLCanon.change_tool(self, pocket)
        self.tools[0] = self.tools[pocket] if 0 < pocket < len(self.tools) else EMPTY

    def get_tool(self, idx):
        return tuple(self.tools[idx]) if 0 <= idx < len(self.tools) else tuple(EMPTY)

    def get_external_angular_units(self): return 1.0
    def get_external_length_units(self): return 1.0
    def get_axis_mask(self): return 7
    def get_block_delete(self): return False


def main():
    fails = 0

    def check(ok, text):
        nonlocal fails
        print("  %s  %s" % ("pass" if ok else "FAIL", text))
        if not ok: fails += 1

    canon = Canon()
    td = tempfile.mkdtemp()
    canon.parameter_file = os.path.join(td, "sim.var")
    import shutil
    shutil.copy(os.path.join(HERE, "..", "configs", "kmotion-sim", "sim.var"), canon.parameter_file)
    canon.configure_program_geometry("XYZ", glcanon_bake.RotationOffsets(), False, want_axis_positions=False)
    # Without a running LinuxCNC the interpreter has no tool table (it lives in shared memory), so the
    # demo is parsed with the tool's length offset given as G43.1 in place of T1 M6 G43 H1
    program = os.path.join(td, "stock-demo.ngc")
    with open(os.path.join(HERE, "stock-demo.ngc")) as src, open(program, "w") as dst:
        for line in src:
            dst.write("G43.1 Z0.5\n" if line.startswith("T1 M6") else line)
    result, seq = gcode.parse(program, canon, "G20", "G53 G0 X0 Y0 Z0 G17 G90 G94 G54 G49 G40")
    check(result <= gcode.MIN_ERROR, "stock-demo.ngc (with G43.1) parsed (result %d, line %d)" % (result, seq))
    geom = canon.program_geometry
    n = len(geom)
    print("  %d vertices, lines %s..%s, tools %s, offsets %s" % (n, geom.lines.min(), geom.lines.max(),
          sorted(set(int(t) for t in geom.tools)), geom.tool_offsets))

    # the tool tip: the move of line 6 (G1 Z-0.25) ends at Z -0.25
    pos = geom.positions(0)
    i = int(np.nonzero(geom.lines == 7)[0][-1])
    tip = kmotion_stock.tip_positions(geom)
    print("  line 7 ends at drawn Z %.4f, tip Z %.4f, with a length offset of %.2f in force" % (pos[i][2], tip[i][2], geom.tool_offset_at(i)[2]))
    check(abs(tip[i][2] - (-1.25)) < 1e-4, "the tool tip is where the program says (Z -1.25): the preview draws the tip")

    box = kmotion_stock.stock_box(canon, type("S", (), {"box": None})())
    check(box is not None and np.allclose(box[0], (0, 0, -2)) and np.allclose(box[1], (9, 6, -1)), "the Stock box from the WORKPIECE comment: %s" % (box,))

    stock = sim.Stock()
    stock.define(box[0], box[1], 0.01)
    images = kmotion_stock.ToolImages(None, HERE)
    fed = kmotion_stock.program_poses(geom, stock, images, {0: 0.25, 1: 0.25})
    check(fed is not None and stock.nb_poses() == n, "%d poses given to the engine (one a vertex: no pieces asked for)" % stock.nb_poses())
    stock.cut_all()
    check(stock.wait_idle(180000), "cut")
    st = stock.stats()
    block = 9.0 * 6.0 * 1.0
    removed = block - st["volume"][2]
    # slot 1 (0.25 wide, 0.25 deep, through the block, 9 long), slot 2 (0.125 deep, through, 6 long) less its
    # crossing of slot 1, and the rapid inside the block (0.1 deep, from X2 to X7 with round ends) less its
    # crossing of slot 2
    import math
    expect = (0.25 * 0.25 * 9.0 + 0.25 * 0.125 * 6.0 - 0.25 * 0.25 * 0.125
              + (0.25 * 5.0 + math.pi * 0.125 ** 2) * 0.1 - 0.25 * 0.25 * 0.1)
    check(abs(removed - expect) < 0.05 * expect, "removed %.4f cubic inch of %.4f expected (lattice %s)" % (removed, expect, st["nodes"]))
    meshes = stock.take_meshes()
    kinds = set()
    for _slot, v, t in meshes:
        verts = np.frombuffer(bytes(v), dtype=np.dtype([("xyz", np.float32, 3), ("kind", np.float32), ("dir", np.uint32)]))
        kinds.update(np.unique(np.round(verts["kind"])).astype(int).tolist())
    check(kinds >= {0, 1, 2}, "the surface has uncut, feed-cut and rapid-cut parts (kinds %s) in %d meshes" % (sorted(kinds), len(meshes)))

    flags = kmotion_stock.pose_flags(geom.kinds)
    check(flags[0] == 0 and (flags[1:] & sim.SIMPOSE_MOVE).all(), "every vertex but the first is a move")

    # the executed path's lines, from the poses given
    _lines, tips, flags = fed
    rapid = (flags[1:] & sim.SIMPOSE_RAPID) != 0
    verts = kmotion_stock.done_path_vertices(tips, flags, (1, 0, 0, 1), (0, 1, 0, 1))
    check(verts.shape == (2 * (n - 1), 8) and np.array_equal(verts[0::2, :3], tips[:-1]) and np.array_equal(verts[1::2, :3], tips[1:])
          and np.array_equal(verts[0::2, 4] == 1, rapid) and np.array_equal(verts[1::2, 3] == 1, ~rapid) and 0 < rapid.sum() < n - 1,
          "the executed path's lines: a segment a move in path order, %d rapids in the traverse colour and %d feeds in the feed colour"
          % (rapid.sum(), (~rapid).sum()))
    v = kmotion_stock.done_path_vertices(np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]], np.float32),
                                         np.array([0, sim.SIMPOSE_MOVE, 0], np.int32), (1, 0, 0, 1), (0, 1, 0, 1))
    check(np.array_equal(v[2, :3], v[3, :3]) and np.array_equal(v[0, :3], [0, 0, 0]) and np.array_equal(v[1, :3], [1, 0, 0]),
          "a pose without a move (a dwell, a tool change) gets a segment of no length")

    # a run, followed: the StockPart with a stand-in ini and status
    class Ini:
        def find(self, section, key):
            return {("KMOTION", "STOCK_RESOLUTION"): "0.01", ("KMOTION", "STOCK_DEFAULT_DIAMETER"): "0.25",
                    ("TRAJ", "COORDINATES"): "XYZ"}.get((section, key))   # (tool 0 here: no T1 M6 without a tool table)

    class Stat:
        task_mode = 1                                   # MODE_MANUAL
        interp_state = 1                                # INTERP_IDLE
        motion_line = 0
        queue = 0                                       # moves of a program queued in motion
        paused = False
        read_line = 1                                   # the line the interpreter reads (0 at the end of the file)
        actual_position = (0.0,) * 9
        tool_offset = (0.0,) * 9
        linear_units = 1 / 25.4                         # an inch machine

    canon.tools = list(TOOLS)
    part = kmotion_stock.StockPart(kmotion_stock.Settings(Ini(), td))
    stat = Stat()
    part.load_program(canon, stat)
    given = part.stock.nb_poses()                       # the vertices, the long moves in pieces
    check(part.stock.wait_idle(180000) and abs((block - part.stock.stats()["volume"][2]) - expect) < 0.04 * expect,
          "the part cuts the whole program when it is loaded while the machine is idle")
    stat.task_mode, stat.interp_state = 2, 2            # MODE_AUTO, running
    stat.queue = 10
    stat.motion_line = 8                                # G1 X9.5: the first slot, the tool half way along it
    stat.actual_position = (4.5, 3.0, -1.25 + 0.5, 0, 0, 0, 0, 0, 0)
    stat.tool_offset = (0, 0, 0.5, 0, 0, 0, 0, 0, 0)
    part._follow(stat)
    check(part._running and 0 < part._done < part.stock.nb_poses() and abs(part._tips[part._done - 1][0] - 4.5) <= 0.06,
          "a run started: the Stock is cut again up to the piece the tool is on (%d poses of %d, the last at X %.3f for the tool at X 4.5)"
          % (part._done, part.stock.nb_poses(), part._tips[part._done - 1][0]))
    tip = np.array(stat.actual_position[:3]) - np.array(stat.tool_offset[:3])
    segs, frac = part.frontier
    end = part.frontier_point()
    check(0 < segs <= part._done - 1 and end is not None and np.allclose(end, tip, atol=1e-3) and abs(part._tips[segs][0] - 4.5) <= 0.06,
          "the executed path: %d whole segments, then the piece the tool is on as far as the tool (its end %s for the tool at %s)"
          % (segs, end.round(3), tip))
    stat.actual_position = (4.52, 3.0, -1.25 + 0.5, 0, 0, 0, 0, 0, 0)      # a little further along the same piece
    part._follow(stat)
    segs2, frac2 = part.frontier
    end2 = part.frontier_point()
    check((segs2, frac2) > (segs, frac) and 0 < frac2 < 1 and abs(end2[0] - 4.52) < 1e-3 and part._done == part._progress(stat)[0],
          "... and follows the tool within the piece (%.2f of segment %d, the end at X %.3f for the tool at X 4.52)" % (frac2, segs2, end2[0]))
    part.stock.wait_idle(180000)
    st = part.stock.stats()
    check(st["applied"] == part._done and 0.22 < block - st["volume"][2] < 0.34,
          "... and the engine has cut %d poses, %.4f cubic inch (half of the first slot: 0.28)" % (st["applied"], block - st["volume"][2]))
    stat.motion_line = 12                               # the second slot, G1 Y6.5, the tool a little way in
    stat.actual_position = (4.5, 2.5, -1.125 + 0.5, 0, 0, 0, 0, 0, 0)
    part._follow(stat)
    check(abs(part._tips[part._done - 1][1] - 2.5) <= 0.06, "later in the run: up to the piece of line 12 the tool is on (%d poses, the last at Y %.3f for the tool at Y 2.5)" % (part._done, part._tips[part._done - 1][1]))
    end = part.frontier_point()
    check(part.frontier > (segs2, frac2) and np.allclose(end, (4.5, 2.5, -1.125), atol=1e-3),
          "the executed path has come to line 12, its end at %s for the tool there" % (end.round(3),))
    stat.motion_line = 8
    stat.actual_position = (6.0, 3.0, -0.5 + 0.5, 0, 0, 0, 0, 0, 0)
    before = part._done
    front = part.frontier
    part._follow(stat)
    check(part._done == before and part.frontier == front, "a line number seen again (a loop, a late status) never takes cuts, or the executed path, back")
    stat.task_mode, stat.interp_state = 1, 1            # paused and the GUI in manual to jog: task reports MANUAL and the interpreter idle ...
    part._stat = stat                                   # as draw() leaves it
    part.tick()
    check(part._running and part._done == before and part.frontier == front,
          "... but the program's moves stay queued (%d): the run only stands, nothing is cut again" % stat.queue)

    class Reloaded:                                     # gmoccapy reloads the display meanwhile: the same program, a new canon
        program_geometry, workpieces, tools = canon.program_geometry, canon.workpieces, canon.tools
    part.load_program(Reloaded(), stat)
    check(part._running and part._done == before and part.frontier == front and part.stock.nb_poses() == given,
          "the same program loaded again while the run stands keeps the run, its cuts and the executed path")
    stat.task_mode, stat.queue = 2, 0                   # an abort: the queue emptied (the executing line keeps its last value) ...
    part.load_program(Reloaded(), stat)                 # ... and gmoccapy reloads the display at once, before any tick
    again = part.tick()
    removed = block - part.stock.stats()["volume"][2] if part.stock.wait_idle(180000) else 0
    check(not part._running and part._done == before and 0.3 * expect < removed < 0.8 * expect,
          "the run stopped, the display reloaded in the same moment: the cut stays as it was (%.3f of %.3f cubic inch)" % (removed, expect))
    check(part.frontier == front and np.allclose(part.frontier_point(), end, atol=1e-6), "... and the executed path stays as it was, to where it stopped")
    check(part.clear_executed() and part.executed_range()[0] == front[0] and part.frontier == front and part._done == before,
          "the GUI's clear-plot button hides the executed path drawn so far and leaves the cut")
    stat.task_mode, stat.interp_state = 2, 2            # a new start, the tool at X 1 on the first slot
    stat.queue = 10
    stat.motion_line = 8
    stat.actual_position = (1.0, 3.0, -1.25 + 0.5, 0, 0, 0, 0, 0, 0)
    part._follow(stat)
    end3 = part.frontier_point()
    check(part._running and part.frontier < front and part._done < before and end3 is not None and abs(end3[0] - 1.0) < 1e-3,
          "a new start clears it: the executed path begins again, up to the tool (X %.3f)" % end3[0])
    stat.motion_line, stat.read_line = 0, 0             # cleared lines with the interpreter idle: an abort in progress, not the end
    stat.interp_state = 1
    part._follow(stat)
    check(part._running and not part._eof, "cleared line numbers with the interpreter idle (an abort draining) are not the program's end")
    stat.interp_state = 4                               # the program read to its end: task clears the lines, the interpreter waits for the last moves
    part._follow(stat)
    check(part._running and part._eof, "the end of the program read, its last moves in flight: noted")
    stat.task_mode, stat.interp_state, stat.queue = 2, 1, 0   # ... and done
    again = part.tick()
    check(again and not part._running and part.stock.wait_idle(180000) and abs((block - part.stock.stats()["volume"][2]) - expect) < 0.04 * expect,
          "the run complete: the whole program is cut (the moves after the last line seen too)")
    stat.task_mode = 1

    class Loaded:                                       # the same program loaded again, the machine idle: a new canon object
        program_geometry, workpieces, tools = canon.program_geometry, canon.workpieces, canon.tools
    kept = part.frontier
    part.load_program(Loaded(), stat)
    check(part.frontier == kept and part.stock.nb_poses() == given and part._canon is not canon,
          "the same program loaded again while idle keeps the Stock and the executed path as they are")
    # a metric program on the inch machine with its WORKPIECE comment above its G21: the preview begins
    # the parse in the machine's units and reads the comment's 50 as inches; the part takes the
    # program's units for it (and leaves a comment that says UNITS= alone)
    mm = os.path.join(td, "mm.ngc")
    with open(mm, "w") as f:
        # shaped like the 3D_Chips sample: parameters between the inch preamble and the G21 (the path's
        # start vertex was labelled with one of their lines, above the G21)
        f.write("(WORKPIECE,BOX,XMIN=-50,YMIN=-50,ZMIN=-40,XMAX=50,YMAX=50,ZMAX=0)\nG20 G90 G17 G54 G94\n#<xs> = 1.0\n#<zs> = 1.0\n"
                "#<fs> = 1.0\n#<rpm> = 1600\n\nN30 G21\nN40 G90\nG64\nS#<rpm> M3\n"
                "(WORKPIECE,BOX,UNITS=INCH,XMIN=0,YMIN=0,ZMIN=0,XMAX=1,YMAX=1,ZMAX=1)\nG0 Z[#<zs>*10.]\nG0 X[#<xs>*53.] Y-56.128\n"
                "G1 Z[#<zs>*-5] F[#<fs>*100]\nM2\n")
    canon2 = Canon()
    canon2.parameter_file = canon.parameter_file
    canon2.configure_program_geometry("XYZ", glcanon_bake.RotationOffsets(), False, want_axis_positions=False)
    result, seq = gcode.parse(mm, canon2, "G20", "G53 G0 X0 Y0 Z0 G17 G90 G94 G54 G49 G40")
    big = canon2.workpieces[0].machine_points.max(axis=0) if len(canon2.workpieces) == 2 else (0,)
    check(result <= gcode.MIN_ERROR and len(canon2.workpieces) == 2 and abs(big[0] - 50.0) < 1e-6,
          "a metric program parsed as the preview does on an inch machine: its WORKPIECE above the G21 read as inches (X to %.1f; "
          "the path's vertices labelled with lines %s, %s..)" % (big[0], canon2.program_geometry.lines[0], canon2.program_geometry.lines[1]))
    stat2 = Stat()
    stat2.file = mm
    part.load_program(canon2, stat2)                   # another program into the first part: a fresh load
    check(part.frontier == (0, 0.0) and part.frontier_point() is None and part.stock.nb_poses() != given,
          "another program loaded clears the executed path and cuts it whole (%d poses)" % part.stock.nb_poses())
    part.stock.wait_idle(60000)
    part2 = kmotion_stock.StockPart(kmotion_stock.Settings(Ini(), td))
    part2.load_program(canon2, stat2)
    box = kmotion_stock.stock_box(canon2, part2.settings)
    check(np.allclose(box[0], (-50 / 25.4, -50 / 25.4, -40 / 25.4)) and np.allclose(box[1], (50 / 25.4, 50 / 25.4, 0.0)),
          "the Stock takes the comment in the program's units, mm: %s" % (tuple(round(v, 3) for v in box[1]),))
    wp = canon2.workpieces
    check(np.allclose(wp[0].points, wp[0].machine_points) and abs(wp[0].params["XMAX"] - 50 / 25.4) < 1e-9,
          "the preview's outline and the comment's params follow")
    check(abs(wp[1].machine_points.max() - 1.0) < 1e-9, "a comment that says UNITS=INCH is left alone")
    part2.stock.wait_idle(60000)

    print("check-feed", "passed" if not fails else "FAILED (%d)" % fails)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
