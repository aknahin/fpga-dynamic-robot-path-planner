#!/usr/bin/env python3
"""Check a recorded run (sim/ff_webots_*.npz) against the scenario's ground truth.

  python3 ref/check_ff_run.py sim/ff_webots_ref.npz --seed 3

Reports: reached the goal, clearance to every rack / box / moving box over the
whole run, whether any cell the robot saw as blocked is not a real obstacle
(false obstacles), and whether every real box was found. Also replays all
frames through ref/brain_ff.py and checks the wheel commands match.
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from brain_ff import BrainFF  # noqa: E402
from sim_world import BOX_HALF_MM, RACK_HALF_MM, ROBOT_R_MM, World, late_drop_from_poses  # noqa: E402
from warehouse import CELLS, RACK_MASK, cell_xy  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
ap.add_argument('run')
ap.add_argument('--seed', type=int, default=3)
args = ap.parse_args()

d = dict(np.load(args.run))
world = World(seed=args.seed)
t, pose, cams, wheels = d['t'], d['pose'], d['cams'], d['wheels']
status = d['status']
late = late_drop_from_poses(world, t, pose)     # (cell, time) of the box dropped mid-run
boxes = list(world.boxes) + ([(late[0], late[1])] if late else [])
box_cells = {c for c, _ in boxes}

min_clear = 1e9
worst = None
for i in range(len(t)):
    x, y = pose[i][0], pose[i][1]
    squares = [(*cell_xy(c), RACK_HALF_MM) for c in range(CELLS) if (RACK_MASK >> c) & 1]
    squares += [(*cell_xy(c), BOX_HALF_MM) for c, t_on in boxes if t[i] >= t_on]
    squares += [(*m.pos(t[i]), BOX_HALF_MM) for m in world.movers]
    for cx, cy, h in squares:
        clear = math.hypot(max(abs(x - cx) - h, 0.0), max(abs(y - cy) - h, 0.0)) - ROBOT_R_MM
        if clear < min_clear:
            min_clear, worst = clear, (t[i], cx, cy)

brain = BrainFF()
mismatch = 0
false_seen = set()
for i in range(len(t)):
    w = brain.step(cams[i].tolist(), *[int(v) for v in pose[i]])
    if tuple(w) != tuple(int(v) for v in wheels[i]):
        mismatch += 1
    seen = brain.memory.blocked_mask() & ~RACK_MASK
    truth = 0
    for c, t_on in boxes:
        if t[i] >= t_on:
            truth |= 1 << c
    near_mover = 0
    for m in world.movers:   # a moving box may be seen in a cell it just left or entered
        mx, my = m.pos(t[i])
        for c in range(CELLS):
            if abs(cell_xy(c)[0] - mx) < 230 and abs(cell_xy(c)[1] - my) < 230:
                near_mover |= 1 << c
    for c in range(CELLS):
        in_lane = c // 16 == 1 or c % 16 == 7      # ghosts of the moving boxes stay until looked at again
        if (seen >> c) & 1 and not (truth >> c) & 1 and not (near_mover >> c) & 1 and not in_lane:
            false_seen.add(c)
final = brain.memory.blocked_mask()
found = {c for c in box_cells if (final >> c) & 1}
print(f'frames {len(t)}, final status {status[-1]}, reached goal at t={t[-1]:.1f} s')
print(f'min clearance to any rack/box/mover: {min_clear:.1f} mm' +
      (f' (t={worst[0]:.1f} near ({worst[1]:.0f},{worst[2]:.0f}))' if min_clear < 20 else ''))
print(f'wheel commands replayed through brain_ff: {mismatch} mismatches of {len(t)}')
print(f'cells ever seen as blocked that are not boxes or on a mover lane: {sorted(false_seen) or "none"}')
print(f'real boxes still remembered at the end: {len(found)} of {len(box_cells)} '
      f'(boxes behind racks or out of range may be unseen)')
ok = status[-1] == 'at_goal' and min_clear > 0 and mismatch == 0 and not false_seen
print('RESULT:', 'OK' if ok else 'CHECK ABOVE')
sys.exit(0 if ok else 1)
