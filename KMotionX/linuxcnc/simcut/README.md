# The Stock: material removal in LinuxCNC's preview

KMotionCNC's Cut Simulator (the *Stock* of its G Code Viewer) for LinuxCNC: a block of material
that the loaded program cuts, drawn in the preview of gmoccapy, AXIS or any GUI that draws
through LinuxCNC's shared preview renderer (`rs274.glcanon`, LinuxCNC 2.10). Grey is uncut
material, blue a surface cut by a feed, red one cut by a rapid - a crash is visible.

The cutting is KMotionCNC's engine, `SimCut.cpp` (in `PC VC Examples/KMotionCNC/`, with
`SimCut.h` and `SimCutPort.h`; MIT licence, `SimCut-LICENSE.txt` there), unchanged: the same
file serves KMotionCNC. `SimCut_LinuxCNC_Handoff.md` beside it describes the engine, and
`SimCutTest/` is its test (`make run` there: 285 checks).

## Files

- `simcut_module.cc`, `Makefile` - the engine as a Python module, `kmotion_simcut` (pybind11):
  class `Stock` with the poses and meshes as numpy arrays. `make` builds it here; `make check`
  cuts a slot; `make check-feed` runs `stock-demo.ngc` through LinuxCNC's interpreter into it.
- `kmotion_stock.py` - the Stock as a part of the preview's scene: feeds the engine from the
  preview's geometry and the machine's status, draws what the engine hands over.
  `kmotion_stock.install(widget, inifile)` is all a GUI has to call.
- `stock_rc.py` - does that call as the GUI's `[DISPLAY] USER_COMMAND_FILE`.
- `gmoccapy_stock.py` - under gmoccapy, two toggles under the preview beside the dimensions
  toggle, the Stock and the program's path lines shown or hidden (KMotionCNC's Stock and Paths;
  kept as `view_stock` and `view_paths` in gmoccapy's preferences), and two buttons on the tool table page: **Tool image...**
  gives the selected tool a tool image, **Create tool...** makes one from a basic tool's sizes
  (KMotionCNC's Create Tool). `tool_image.py` does the latter from the command line.
- `axis_stock.py` - under AXIS, a `Stock` menu before `Help`: **Show stock**, **Tool image...**
  and **Remove tool image...** for a tool asked for by number (the one in the spindle offered;
  they edit the tool table file and have LinuxCNC read it again), **Create tool...**, which runs
  `tool_dialog.py`: the Create tool dialog as a process of its own (`--images DIR --table FILE
  --tool N`), for a GUI that is not GTK.
- `3D_Chips with stock.ngc` - LinuxCNC's 3D Chips sample with a `WORKPIECE` comment: a 100 x 100
  x 40 mm block cut with T2, a 1/4 inch ball nose, then a slot round its base with T3, a T-slot
  cutter. `configs/kmotion-sim` opens it at start; its G54 (Z -3 inch) keeps the 60 mm retract
  and the T-slot's 2 inch length offset under the Z limit.
- `stock-demo.ngc` - a 9 x 6 x 1 inch block, two slots and a rapid that crashes into it, cut
  with T1 and its length offset. Every move lies between X-0.5 and 9.5 and Y-0.5 and 6.5, inside
  soft limits of 10. The block's top is at Z-1 and the retracts at Z-0.5, so that a length offset
  of up to half an inch stays under a Z limit near 0 (kmotion-sim's is 0.12). The Minirouter
  bench's simulated limit switches (`BenchLoop.c`) are made at 12 inches for it.

## Setting it up

1. `make` in this folder (needs `python3-pybind11`, `python3-dev`, g++; the module is built
   against the Python that LinuxCNC runs with).
2. In the ini file:

   ```ini
   [DISPLAY]
   USER_COMMAND_FILE = ../../simcut/stock_rc.py    ; from the config folder to this one

   [KMOTION]
   STOCK = 1                       ; the Stock is on
   STOCK_RESOLUTION = 0            ; lattice spacing in machine units; 0: chosen by the Stock
   STOCK_FOLLOW = 1                ; a run cuts as far as the tool has come (0: always the finished part)
   TOOL_IMAGES = toolimages.txt    ; which tool image each tool has (below); optional
   ;TOOL_IMAGE_DIR = /path/to/ToolImages    ; where image names without a folder are; default <KMotionX>/KMotion/Data/ToolImages
   ;STOCK_BOX = 0 0 -1 4 2 0       ; the block when the program has no WORKPIECE comment (machine units)
   ;STOCK_FILE = part.stock        ; a KMotionCNC Stock file to start every path from
   ;STOCK_LINE_WIDTH = 1.5         ; the path lines are kept in front of the surface by this many pixels
   ;STOCK_DEFAULT_DIAMETER = 0     ; a tool number that is not in the tool table cuts as a flat end mill of this diameter (0: nothing)
   ;STOCK_PIECE = 0.05             ; a move longer than this (machine units) is cut in pieces of it, so the cut follows the tool along it
   ;STOCK_TOOL_SHAPE = 1           ; the tool marker drawn from the tool's image (0: LinuxCNC's cone or cylinder)
   ;STOCK_DONE_PATH = 1            ; the path the run has executed, drawn over the program's lines (0: not)
   ```

   `configs/kmotion-sim` (AXIS), `configs/kmotion-minirouter` and `configs/kmotion-3link`
   (gmoccapy) have these. The one `USER_COMMAND_FILE` serves both GUIs: in gmoccapy it adds the
   Stock and Paths toggles under the preview and the Tool image and Create tool buttons on the
   tool table page; in AXIS a `Stock` menu before `Help` (show or hide the Stock, a tool image
   picked for a tool or taken away, the Create tool dialog), the program's lines and the live
   plot being in AXIS's own View menu. The ini is the only place the Stock is set up; a config
   without the two lines above has none of it.
3. Load a program whose first lines declare the stock, as `stock-demo.ngc` does:

   ```
   (WORKPIECE,BOX,XMIN=0,YMIN=0,ZMIN=-2,XMAX=9,YMAX=6,ZMAX=-1)
   ```

   The numbers are in the program's units. LinuxCNC reads them in the units in force at the
   comment's line, and its preview begins every program in the machine's units; a metric program
   on an inch machine, with the comment above its G21, would give a block 25.4 times too big. So
   a comment with no G20/G21 above it takes the units the program moves in (its last G20/G21
   before the first move), and the preview's outline is corrected with it. A comment can also say
   so itself: `(WORKPIECE,BOX,UNITS=MM,XMIN=-50,...)` (UNITS=MM or UNITS=INCH, LinuxCNC's key).

   LinuxCNC reads the comment in the coordinate system active at that line (G5x, G92), so the
   block moves with the work offset - unlike KMotionCNC's Stock, which is fixed in machine
   coordinates. A cylinder or tube is taken by its bounding box. Without the comment the block
   is `[KMOTION] STOCK_BOX`, else a Stock file, else nothing is drawn.

### Tool shapes

A tool cuts with its tool image, named in either of two places. In the tool table, as
`[name.wrl]` at the end of the tool's comment - which is what the **Tool image...** button on
gmoccapy's tool table page writes for the selected tool (it saves the table, LinuxCNC reloads
it, and the Stock cuts the loaded program again with the new shape):

```
T1 P1 Z0.511 D0.125 ;1/8 end mill [EndMill-z 1in .125.wrl]
```

Or in `TOOL_IMAGES`, one tool a line (the table's comment wins when both name one):

```
# tool number, then the file (KMotion/Data/ToolImages unless a folder is given)
T1 EndMill-z 1in .125.wrl
T5 Dovetail-z .750 60deg.wrl
```

New shapes: the **Create tool...** button on gmoccapy's tool table page writes an end mill, ball
nose, V bit, T slot or dovetail from its sizes into the tool image folder, as KMotionCNC's Create
Tool dialog does, and gives it to the selected tool if asked. It shows KMotionCNC's picture of the tool
with its sizes lettered (`pictures/`, from KMotionCNC's resources), the letters beside the sizes. It starts from the selected tool's image when that is one it made (the file name says the type and sizes), its file name the image's so the tool is edited in place, else from the table's diameter. `tool_image.py` does the same from
the command line (sizes in the machine's units, the tip at height 0):

```
./tool_image.py endmill  0.125 1.0                      "EndMill D0.125 L1 in.wrl"
./tool_image.py ball     0.25  1.5                      ball.wrl
./tool_image.py v        0.5   60 0 2                   v60.wrl        # diameter, point angle, tip diameter, length
./tool_image.py tslot    0.75  0.25 0.3125 2            tslot.wrl      # diameter, cutter height, neck diameter, length
./tool_image.py dovetail 0.75  60 0.3125 0.375 2        dovetail.wrl   # diameter, side angle, cutter height, neck, length
./tool_image.py --read   dovetail.wrl                                  # the profile the engine makes of any image
```

Put the file in `KMotion/Data/ToolImages` (or name its folder) and the tool's line in `TOOL_IMAGES`.
Any VRML tool image KMotionCNC displays works the same way: the engine spins it about Z.

The tool marker in the preview is drawn from the image as well, where the tool in the spindle has
one: the profile spun about the tool's axis, at the marker's place, in the marker's colour and
transparency. A tool without an image keeps LinuxCNC's cone or cylinder. `STOCK_TOOL_SHAPE = 0`
keeps LinuxCNC's marker for every tool.

These are KMotionCNC's VRML tool images, spun about Z by the engine, at scale 1 (as a Tool
Table Image in KMotionCNC). The tool of a move is the one the preview's tool changes give it,
else the tool in the spindle when the program was loaded. The preview's interpreter records an
M6, but not an M61 - which is how KMotionX's M6 remap (`ngc/kmotion_m6.ngc`) sets the tool - so
with that remap the patch `patches/gcodemodule-m61-tool-change.patch` is needed, else every move
is cut with the tool that was in the spindle. A tool without an image cuts as a flat end mill of the tool table's
diameter (`D`); a tool number the table does not have cuts as `STOCK_DEFAULT_DIAMETER` (0: nothing). A Create Tool dialog like KMotionCNC's is not
written yet; its images can be made there and copied.

### When the Stock is cut

- A loaded program is cut at once, as a whole: the finished part.
- With `STOCK_FOLLOW = 1`, a run starts from the uncut block and the cut follows the tool: every
  vertex of the lines before the executing line, and on that line the piece the tool is on
  (within 0.01 inch). A long move is cut in pieces of `STOCK_PIECE` (0.05 inch) for that, as
  KMotionCNC plots long moves in pieces of 0.05 s. At the end of the run, or an abort, the whole program is cut again.
- Jogging and MDI moves do not cut (as in KMotionCNC).

### The executed path

With `STOCK_DONE_PATH = 1` (the default) the part of the program's path a run has executed is
drawn over the program's lines as the run proceeds: from the first move to the piece being
executed, and along that piece as far as the tool. Feeds are in the live plot's feed colour
(glcanon's `backplotfeed`, 0.75 0.25 0.25), rapids in its traverse colour (0.30 0.50 0.50), a
little wider than the program's lines. It is cleared when a run starts, and stays after the run
ends until the next start or a program load.

It is taken from the program's own path - the poses the Stock cuts with, and the progress that
feeds the engine - never from samples of the position. The board executes exactly the trajectory
the PC planned, so one status read gives the right picture, and a stall of the status (the board's
status over USB stalls for up to a second now and then) only pauses the frontier. gmoccapy's live
plot (the pencil toggle) is drawn from the samples, so a stall makes it jump in a straight line
across the segments it missed; with the executed path on, the pencil toggle can be left off.

### 4 and 5 axis

The tool tip is where the preview draws the vertex (the tool length offset taken out), and the
tool points as KMotionCNC's viewer turns it: (0, 0, 1) about X by A, then about Y by B, then
about Z by C, from the A, B, C positions the renderer records at each vertex. The part is taken
as fixed and the tool as pivoting about its tip (RTCP, `G43.4`), whatever the machine does to
achieve it - which is right for KMotion's kinematics, where the planner turns the CAD path into
actuator motion. `[DISPLAY] GEOMETRY` must be `XYZ` (the default): the Stock takes the preview's
coordinates as machine coordinates.

## How it hooks in

Nothing in LinuxCNC is patched. `install()` adds a `StockPart` to the widget's scene
(`rs274.glcanon_scene`), before the program lines, and a timer that redraws while the engine
cuts. With `STOCK_DONE_PATH` on it adds a `DonePathPart` after the program's highlight, so that
the executed path lies over the program's lines (at their depth, with the live plot's bias)
whatever the Stock's own toggle says: it keeps the program's path as one `GL_LINES` buffer in the
renderer's per-vertex-colour layout and draws as many of its first segments as the Stock's frontier
says, plus the piece the tool is on up to the tool, through the renderer's line shader (or its
quad-expanding one where the driver refuses the width). The part reads the loaded program from `ctx.canon.program_geometry` (positions, line
numbers, kinds, tools, axis positions), the stock from `ctx.canon.workpieces`, the run from
`ctx.stat`. When the machine has a rotary axis, `install()` wraps
`rs274.glcanon.GLCanon.configure_program_geometry` so that every parse records the axis
positions (the renderer does that only when asked before the parse).

Drawing: each block the engine hands over (20 byte vertices - position, kind, direction;
16 bit indices) goes into a VBO and EBO of its own, drawn with a shader that colours by kind
and shades by the surface direction, as KMotionCNC's `Lib3D/Stock3d.cpp` does. The Stock is
pushed back in depth by the width of the path lines so that they show on the cut surface (the
handoff's section 4: KMotionCNC pulls the lines nearer instead), and drawn with
`glPolygonOffset(0, 1)`.

Debugging: with the environment variable `KMOTION_STOCK_DEBUG=<file>` the part appends what it
uploads (each block's vertices, their extent and kinds) and the matrices of a frame now and then.

## Not yet

- Buttons to save, load and unload Stock files (the part has `save`, `load`, `unload` for a GUI
  to call); the Stock toggle is gmoccapy's and AXIS's (`set_visible` for another GUI), the Paths
  toggle gmoccapy's (AXIS has it in View). gmoccapy's own pencil toggle is the live plot of the
  actual position, which the executed path stands in for.
- The tool image lives in the tool's comment, not in a column of the table (LinuxCNC's table
  has none). gmoccapy's buttons edit its tool table page; AXIS's menu asks for the tool's number
  (its tool table is the separate tooledit program) and edits the table file, then has LinuxCNC
  read it again; another GUI edits the comment or `TOOL_IMAGES`.
- Several `(WORKPIECE,...)` pieces, cylinders and tubes as such (one box for now).
- Jogging does not cut; after an abort the whole program is cut, not the part up to the tool.
- The engine's automatic resolution starts at 0.001 inch; on a millimetre machine that is
  finer than meant (the memory budget still coarsens it).
- One draw call per block from Python; a very fine Stock (millions of triangles) may want the
  blocks merged or drawn from C++.
