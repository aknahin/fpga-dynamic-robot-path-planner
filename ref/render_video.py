#!/usr/bin/env python3
"""Render a recorded run as a top-down video (what the robot did and what the FPGA believed).

  python3 ref/render_video.py sim/ff_webots_hw.npz --seed 3 -o media/hardware_run.mp4

Left: the warehouse. Grey = racks (known to the FPGA), brown squares = real boxes, red
squares = real moving boxes (ground truth from the scenario seed), orange outline = cells the
FPGA believes are blocked, pink = cells it saw something leave (mover timers, plus the halo),
thin arrows = the flow field, blue fan = the four cameras' rays, line = the robot's trail.
Right: status, time and the cost to the goal from the robot's cell. Needs matplotlib + ffmpeg.
"""
import argparse
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.animation as animation  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection, PatchCollection  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
from flowfield import cost_field, dilate, flow_step, weights  # noqa: E402
from perception import column_ranges, ray_vector  # noqa: E402
from sim_world import BOX_HALF_MM, World, late_drop_from_poses  # noqa: E402
from warehouse import (CAM_COLS, CELL_MM, CELLS, GOAL, GRID, HALF_MM, INF, LANE_MASK,  # noqa: E402
                       MOVER_LANES, RACK_MASK, cell_xy)


def cells_of(mask):
    return [c for c in range(CELLS) if (mask >> c) & 1]


def square(cx, cy, half, **kw):
    return Rectangle((cx - half, cy - half), 2 * half, 2 * half, **kw)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('run')
    ap.add_argument('--seed', type=int, required=True, help='scenario seed (for ground truth)')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--stride', type=int, default=2, help='draw every Nth frame')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--title', default='FPGA flow-field warehouse robot')
    args = ap.parse_args()

    d = dict(np.load(args.run))
    n = len(d['pose'])
    t = d['t'] if 't' in d else np.arange(n) * 0.032
    world = World(seed=args.seed)
    late = late_drop_from_poses(world, t, d['pose'])
    boxes = list(world.boxes) + ([(late[0], late[1])] if late else [])
    blocked_all = [int.from_bytes(b, 'little') for b in d['blocked']] if 'blocked' in d else None
    movers_all = [int.from_bytes(b, 'little') for b in d['movers']] if 'movers' in d else None
    if blocked_all is None:   # a recording without masks: replay the reference to get them
        from brain_ff import BrainFF
        brain = BrainFF()
        blocked_all, movers_all = [], []
        for i in range(n):
            brain.step(d['cams'][i].tolist(), *[int(v) for v in d['pose'][i]])
            blocked_all.append(brain.blocked_mask())
            movers_all.append(brain.mover_mask())

    fig = plt.figure(figsize=(9.6, 7.2), dpi=100)
    ax = fig.add_axes([0.03, 0.05, 0.70, 0.88])
    info = fig.add_axes([0.75, 0.05, 0.23, 0.88])
    info.axis('off')
    fig.suptitle(args.title, fontsize=13, x=0.38, y=0.985)

    def setup_static():
        ax.set_xlim(-HALF_MM, HALF_MM)
        ax.set_ylim(-HALF_MM, HALF_MM)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        ax.add_patch(Rectangle((-HALF_MM, -HALF_MM), 2 * HALF_MM, 2 * HALF_MM, fc='#f4f1ea', ec='#444'))
        gx, gy = cell_xy(GOAL)
        ax.add_patch(square(gx, gy, 62, fc='#f2cc3c', ec='none', zorder=1))
        for a, b in MOVER_LANES:   # painted cart lanes
            (xa, ya), (xb, yb) = cell_xy(a), cell_xy(b)
            ax.plot([xa, xb], [ya, yb], '--', color='#e6c229', lw=1.4, zorder=1)
        for c in range(CELLS):
            if (RACK_MASK >> c) & 1:
                x0, y0 = cell_xy(c)
                ax.add_patch(square(x0, y0, CELL_MM / 2, fc='#7d86a6', ec='#2f5aa8', lw=1.0, zorder=2))
                ax.add_patch(square(x0, y0, CELL_MM / 2 - 28, fc='#a9926f', ec='#e08a1e', lw=0.8, zorder=2))

    setup_static()
    belief_pc = PatchCollection([], zorder=3)
    ax.add_collection(belief_pc)
    dyn = []   # artists redrawn every frame
    trail, = ax.plot([], [], '-', color='#2b6cb0', lw=1.3, alpha=0.7, zorder=6)
    xs, ys = [], []
    text = info.text(0, 1, '', va='top', ha='left', family='monospace', fontsize=9)

    def frame(k):
        nonlocal belief_pc
        for a in dyn:
            a.remove()
        dyn.clear()
        i = min(k * args.stride, n - 1)
        ti = float(t[i])
        x, y, h16 = (int(v) for v in d['pose'][i])
        heading = h16 / 65536.0 * 2 * math.pi
        blocked, movers = blocked_all[i], movers_all[i]
        keep = (1 << ((int(max(0, min(15, (HALF_MM - y) // CELL_MM))) * GRID)
                      + int(max(0, min(15, (x + HALF_MM) // CELL_MM))))) | (1 << GOAL)
        halo = movers
        for _ in range(3):
            halo = dilate(halo)
        shapes, colors = [], []
        for c in cells_of(halo & ~movers & ~blocked & ~RACK_MASK):
            shapes.append(square(*cell_xy(c), CELL_MM / 2)); colors.append('#fbe3e0')
        for c in cells_of(movers & ~blocked & ~RACK_MASK):
            shapes.append(square(*cell_xy(c), CELL_MM / 2)); colors.append('#f6b9b3')
        pc = PatchCollection(shapes, facecolors=colors, edgecolors='none', zorder=1)
        ax.add_collection(pc); dyn.append(pc)
        bl = PatchCollection([square(*cell_xy(c), CELL_MM / 2 - 4) for c in cells_of(blocked & ~RACK_MASK)],
                             facecolors='none', edgecolors='#e08a1e', linewidths=2, zorder=5)
        ax.add_collection(bl); dyn.append(bl)

        # ground truth
        truth = [(*cell_xy(c), '#a67c52') for c, t_on in boxes if ti >= t_on]
        truth += [(*m.pos(ti), '#d9453a') for m in world.movers]
        for cx, cy, col in truth:
            p = square(cx, cy, BOX_HALF_MM, fc=col, ec='#333', lw=0.6, zorder=4)
            ax.add_patch(p); dyn.append(p)

        # flow arrows from the masks the FPGA reported
        field, _ = cost_field(weights((blocked | RACK_MASK) & ~keep, movers))
        qx, qy, qu, qv = [], [], [], []
        for c in range(CELLS):
            if field[c] >= INF or c == GOAL or (RACK_MASK >> c) & 1:
                continue
            nx = flow_step(field, c)
            if nx is None:
                continue
            x0, y0 = cell_xy(c); x1, y1 = cell_xy(nx)
            qx.append(x0 - (x1 - x0) * 0.2); qy.append(y0 - (y1 - y0) * 0.2)
            qu.append((x1 - x0) * 0.5); qv.append((y1 - y0) * 0.5)
        q = ax.quiver(qx, qy, qu, qv, angles='xy', scale_units='xy', scale=1, color='#5a6577',
                      width=0.0028, headwidth=4, headlength=4, zorder=3)
        dyn.append(q)

        # camera rays (nearest return per column)
        ray_segs = []
        s, c_ = math.sin(heading), math.cos(heading)
        for cam in range(4):
            r = column_ranges(d['cams'][i][cam])
            for col in range(0, CAM_COLS, 2):
                vx, vy, hit = ray_vector(cam, col, r[col])
                ray_segs.append([(x, y), (x + vx * c_ - vy * s, y + vx * s + vy * c_)])
        rays = LineCollection(ray_segs, colors='#4a90d9', linewidths=0.4, alpha=0.5, zorder=4)
        ax.add_collection(rays); dyn.append(rays)

        # robot
        body = Circle((x, y), 37, fc='#2b6cb0', ec='white', lw=1, zorder=7)
        ax.add_patch(body); dyn.append(body)
        nose, = ax.plot([x, x + 55 * math.cos(heading)], [y, y + 55 * math.sin(heading)],
                        color='white', lw=2, zorder=8)
        dyn.append(nose)
        xs.append(x); ys.append(y)
        trail.set_data(xs, ys)

        rc = int(max(0, min(15, (HALF_MM - y) // CELL_MM))) * GRID + int(max(0, min(15, (x + HALF_MM) // CELL_MM)))
        text.set_text(
            f'time      {ti:6.1f} s\n'
            f'status    {d["status"][i]}\n'
            f'cell      {rc}\n'
            f'cost left {field[rc] if field[rc] < INF else "-"}\n'
            f'wheels    {int(d["wheels"][i][0]):+4d}/{int(d["wheels"][i][1]):+4d}\n\n'
            f'seen blocked  {bin(blocked & ~RACK_MASK).count("1")}\n'
            f'mover cells   {bin(movers).count("1")}\n\n'
            'grey   racks (known)\n'
            'brown  real box\n'
            'red    real mover\n'
            'orange FPGA: blocked\n'
            'pink   FPGA: recent motion\n'
            'arrows flow field\n'
            'blue   camera rays')
        return []

    frames = (n + args.stride - 1) // args.stride
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    writer = animation.FFMpegWriter(fps=args.fps, bitrate=2500)
    with writer.saving(fig, args.out, dpi=100):
        for k in range(frames):
            frame(k)
            writer.grab_frame()
            if k % 100 == 0:
                print(f'{k}/{frames}', flush=True)
    print(f'{args.out}: {frames} frames, {frames / args.fps:.1f} s')


main()
