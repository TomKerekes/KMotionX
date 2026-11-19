# Copyright (c) 2026 DynoMotion Incorporated.  SPDX-License-Identifier: MIT
"""make check: the kmotion_simcut module cuts a slot through a block and hands its surface over."""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kmotion_simcut as sim

S = sim.Stock()
S.define((0, 0, -1), (4, 2, 0), 0.05)
T = S.add_tool_flat(0.5)
xyz = np.array([[-1, 1, 0.5], [-1, 1, -0.5], [5, 1, -0.5], [5, 1, 0.5]], dtype=np.float32)
flags = np.array([0, sim.SIMPOSE_MOVE | sim.SIMPOSE_RAPID, sim.SIMPOSE_MOVE, sim.SIMPOSE_MOVE | sim.SIMPOSE_RAPID])
S.add_poses(xyz, np.full(4, T), flags)
S.cut_all()
ok = S.wait_idle(20000)
st = S.stats()
block = 4 * 2 * 1
slot = 0.5 * 0.5 * 4			# the slot: 0.5 wide, 0.5 deep, through the block
removed = block - st["volume"][2]
meshes = S.take_meshes()
tris = sum(len(t) // 3 for _, _, t in meshes)
print("idle %s, lattice %s, left %.4f of %.1f: removed %.4f (slot %.4f), %d meshes, %d triangles, generation %d, %d threads"
      % (ok, st["nodes"], st["volume"][2], block, removed, slot, len(meshes), tris, S.generation(), sim.threads()))
good = ok and abs(removed - slot) < 0.05 * slot and tris > 100 and all(len(v) % sim.VERTEX_BYTES == 0 for _, v, _ in meshes)
print("check", "passed" if good else "FAILED")
sys.exit(0 if good else 1)
