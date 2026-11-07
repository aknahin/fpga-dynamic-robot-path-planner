#!/usr/bin/env python3
"""Draw the result graphs from the logs and recordings in sim/ (skips what is missing).

  python3 ref/make_results.py            # writes media/*.png

  regression.png    40+30 random warehouse runs in the Python world: time to goal, clearance
  clocks.png        clocks per frame in the Verilog brain (from tb_ff_brain +cycles) vs the 32 ms budget
  trajectories.png  the robot's path in the real Webots runs, over the warehouse map
  hardware_rtt.png  round trip Mac -> board -> Mac per frame on the real Nexys 3
"""
import glob
import math
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
from warehouse import CELL_MM, CELLS, GOAL, HALF_MM, RACK_MASK, cell_xy  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SIM, MEDIA = os.path.join(REPO, 'sim'), os.path.join(REPO, 'media')
os.makedirs(MEDIA, exist_ok=True)
BLUE, ORANGE, GREY = '#2b6cb0', '#e08a1e', '#8a8fa3'


def regression():
    rows = []
    for f in ('ff_regress_v3lanes.log',):
        p = os.path.join(SIM, f)
        if not os.path.exists(p):
            continue
        for line in open(p):
            m = re.match(r'(ok |BAD)\s+(\d+) (\w+)\s+t=\s*([\d.]+) coll (\d+) clr=\s*(-?[\d.]+)', line)
            if m:
                rows.append((int(m[2]), m[3] == 'at_goal', float(m[4]), int(m[5]), float(m[6])))
    if not rows:
        return
    rows.sort()
    seeds = [r[0] for r in rows]
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
    colors = [BLUE if r[1] and r[4] > 0 else '#d9453a' for r in rows]
    ax[0].bar(seeds, [r[2] for r in rows], color=colors)
    ax[0].set(xlabel='random warehouse (seed); red = brief contact', ylabel='time to goal (s)', title='Time to the delivery zone')
    ax[1].hist([r[4] for r in rows if r[4] > -100], bins=20, color=BLUE)
    ax[1].axvline(0, color='#d9453a', lw=1)
    ax[1].set(xlabel='closest approach, robot edge to any obstacle (mm)', ylabel='runs',
              title='Clearance (left of red line = contact)')
    ok = sum(1 for r in rows if r[1] and r[4] > 0)
    ax[2].axis('off')
    ax[2].text(0, 0.9, f'{len(rows)} random runs\n\nreached the goal: {sum(1 for r in rows if r[1])}\n'
                       f'no contact at all: {ok}\n'
                       f'median time: {np.median([r[2] for r in rows]):.0f} s\n'
                       f'median clearance: {np.median([r[4] for r in rows]):.0f} mm\n\n'
                       'each run: random start, 6 boxes,\n1 box dropped at 8 s, 2 moving boxes',
               va='top', family='monospace', fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(MEDIA, 'regression.png'), dpi=130)
    plt.close(fig)
    print('regression.png', len(rows), 'runs,', ok, 'clean')


def clocks():
    p = os.path.join(SIM, 'ff3_cycles_webots.csv')
    if not os.path.exists(p):
        return
    a = np.loadtxt(p, delimiter=',')
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
    ax[0].plot(a[:, 0] * 0.032, a[:, 1] / 1e3, color=BLUE, lw=0.8)
    ax[0].set(xlabel='run time (s)', ylabel='clocks per frame (thousands)',
              title='Verilog brain: clocks per frame, real Webots run')
    ax[1].hist(a[:, 1] / 1e3, bins=30, color=BLUE)
    ax[1].set(xlabel='clocks per frame (thousands)', ylabel='frames',
              title=f'max {a[:, 1].max():.0f} clocks = {a[:, 1].max() / 100e6 * 1e3:.2f} ms at 100 MHz '
                    f'(budget 32 ms = 3,200,000 clocks)')
    fig.tight_layout()
    fig.savefig(os.path.join(MEDIA, 'clocks.png'), dpi=130)
    plt.close(fig)
    print('clocks.png max', a[:, 1].max())


def map_axes(ax):
    ax.set_xlim(-HALF_MM, HALF_MM); ax.set_ylim(-HALF_MM, HALF_MM); ax.set_aspect('equal')
    ax.set_xticks([]); ax.set_yticks([])
    for c in range(CELLS):
        if (RACK_MASK >> c) & 1:
            x, y = cell_xy(c)
            ax.add_patch(Rectangle((x - 75, y - 75), 150, 150, fc=GREY, ec='none'))
    gx, gy = cell_xy(GOAL)
    ax.add_patch(Rectangle((gx - 62, gy - 62), 124, 124, fc='#f2cc3c', ec='none'))


def trajectories():
    runs = []
    for p, seed, title in (('ff2_webots_hw_s3.npz', 3, 'real FPGA board, seed 3'),
                           ('ff2_webots_hw_s8.npz', 8, 'real FPGA board, seed 8'),
                           ('ff2_webots_hw_s53.npz', 53, 'real FPGA board, seed 53')):
        f = os.path.join(SIM, p)
        if os.path.exists(f):
            runs.append((f, seed, title))
    if not runs:
        return
    from sim_world import World, late_drop_from_poses
    fig, axs = plt.subplots(1, len(runs), figsize=(4.6 * len(runs), 5.2))
    axs = np.atleast_1d(axs)
    for ax, (f, seed, title) in zip(axs, runs):
        d = np.load(f)
        map_axes(ax)
        w = World(seed=seed)
        pose = d['pose']
        late = late_drop_from_poses(w, d['t'], pose) if 't' in d else None
        for c, t_on in list(w.boxes) + ([(late[0], late[1])] if late else []):
            x, y = cell_xy(c)
            ax.add_patch(Rectangle((x - 55, y - 55), 110, 110, fc='#a67c52' if t_on == 0 else '#c9a27a',
                                   ec='#333', lw=0.5))
        ax.plot(pose[:, 0], pose[:, 1], color=BLUE, lw=1.4)
        ax.plot(pose[0, 0], pose[0, 1], 'o', color='#2b8a3e')
        ax.set_title(f'{title}: {len(pose) * 0.032:.0f} s to the goal', fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(MEDIA, 'trajectories.png'), dpi=130)
    plt.close(fig)
    print('trajectories.png', len(runs), 'runs')


def hardware_rtt():
    p = os.path.join(SIM, 'hw3_rtt.csv')
    if not os.path.exists(p):
        return
    a = np.loadtxt(p, delimiter=',', skiprows=1)
    if a.ndim < 2 or len(a) < 2:
        return
    ms = a[:, 1] * 1e3
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
    ax[0].plot(a[:, 0], ms, color=BLUE, lw=0.7)
    ax[0].set(xlabel='frame', ylabel='round trip (ms)', title='Mac -> Nexys 3 -> Mac, per frame')
    ax[1].hist(ms, bins=40, color=BLUE)
    ax[1].set(xlabel='round trip (ms)', ylabel='frames',
              title=f'median {np.median(ms):.2f} ms, 99% under {np.percentile(ms, 99):.2f} ms, '
                    f'max {ms.max():.2f} ms')
    fig.tight_layout()
    fig.savefig(os.path.join(MEDIA, 'hardware_rtt.png'), dpi=130)
    plt.close(fig)
    print('hardware_rtt.png median', np.median(ms))


def layout():
    from warehouse import LAYOUT, MOVER_LANES
    fig, ax = plt.subplots(figsize=(5.4, 5.4))
    map_axes(ax)
    for a, b in MOVER_LANES:
        (xa, ya), (xb, yb) = cell_xy(a), cell_xy(b)
        ax.plot([xa, xb], [ya, yb], '--', color='#e6c229', lw=2)
    gx, gy = cell_xy(GOAL)
    ax.text(gx, gy, 'goal', ha='center', va='center', fontsize=8)
    ax.set_title('The warehouse: 16x16 cells of 150 mm. Grey = racks (known to the FPGA),\n'
                 'dashed = the two cart lanes (known), everything else is found by the cameras', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(MEDIA, 'layout.png'), dpi=130)
    plt.close(fig)
    print('layout.png')


def fpga_journey():
    names = ['v1\nfirst RTL', 'v2\nRAMs, but two\nwrite ports', 'v3\none write\nport per RAM', 'final\n(placed & routed)']
    luts = [159, 154, 37, 30]
    period = [12.18, 12.33, 11.72, 9.88]
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    ax[0].bar(names, luts, color=[ORANGE, ORANGE, BLUE, BLUE])
    ax[0].axhline(100, color='#d9453a', lw=1)
    ax[0].text(3.4, 103, 'device full', color='#d9453a', ha='right', fontsize=8)
    ax[0].set(ylabel='LUTs used (% of XC6SLX16)', title='Area: v1 did not fit, final uses 30%')
    ax[1].bar(names, period, color=[ORANGE, ORANGE, ORANGE, BLUE])
    ax[1].axhline(10, color='#d9453a', lw=1)
    ax[1].text(3.4, 10.15, '10 ns = 100 MHz', color='#d9453a', ha='right', fontsize=8)
    ax[1].set(ylabel='clock period (ns)', ylim=(8, 13), title='Clock period (last bar: place & route)')
    for a in ax:
        a.tick_params(axis='x', labelsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(MEDIA, 'fpga_journey.png'), dpi=130)
    plt.close(fig)
    print('fpga_journey.png')


if __name__ == '__main__':
    layout()
    fpga_journey()
    regression()
    clocks()
    trajectories()
    hardware_rtt()
