#!/usr/bin/env python3
# Copyright (c) 2026 DynoMotion Incorporated.  SPDX-License-Identifier: MIT
"""Make a tool image (VRML, as KMotionCNC's Create Tool dialog does) of one of the basic tools, for the
Stock's TOOL_IMAGES table and KMotionCNC's Tool Table Image:

    tool_image.py endmill  DIAMETER LENGTH                          out.wrl
    tool_image.py ball     DIAMETER LENGTH                          out.wrl
    tool_image.py v        DIAMETER ANGLE TIP_DIAMETER LENGTH       out.wrl   (ANGLE of the point, side to side)
    tool_image.py tslot    DIAMETER CUTTER_HEIGHT NECK_DIAMETER LENGTH            out.wrl
    tool_image.py dovetail DIAMETER ANGLE CUTTER_HEIGHT NECK_DIAMETER LENGTH      out.wrl   (ANGLE side to bottom)

Sizes in the units the machine uses (inches here). The tip is at height 0, the tool closed at LENGTH.
--read FILE prints the profile the engine makes of any tool image instead."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kmotion_simcut as sim

TYPES = {"endmill": (sim.SIMTOOL_ENDMILL, 2), "ball": (sim.SIMTOOL_BALL, 2), "v": (sim.SIMTOOL_V, 4),
         "tslot": (sim.SIMTOOL_TSLOT, 4), "dovetail": (sim.SIMTOOL_DOVETAIL, 5)}


def main(argv):
    if len(argv) == 3 and argv[1] == "--read":
        h, r, open_ = sim.read_tool_image(argv[2])
        print("%s: %d profile points, radius %.4f at most, from height %.4f to %.4f%s" % (
            argv[2], len(h), max(r), h[0], h[-1], ", open above" if open_ else ""))
        for hh, rr in zip(h, r):
            print("  h %9.4f  r %9.4f" % (hh, rr))
        return 0
    if len(argv) < 2 or argv[1] not in TYPES:
        print(__doc__)
        return 2
    kind, count = TYPES[argv[1]]
    if len(argv) != count + 3:
        print("%s takes %d sizes and the file name" % (argv[1], count))
        return 2
    sizes = [float(v) for v in argv[2:2 + count]]
    file = argv[-1]
    comment = "%s %s" % (argv[1], " ".join(argv[2:2 + count]))
    err = sim.write_tool_image(file, kind, sizes, 36, comment)
    if err:
        print("not written:", err)
        return 1
    h, r, _ = sim.read_tool_image(file)
    print("%s: %s, read back as %d profile points, radius %.4f at most, height %.4f" % (file, comment, len(h), max(r), h[-1]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
