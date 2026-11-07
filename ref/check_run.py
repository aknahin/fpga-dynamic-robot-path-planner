"""Check a recorded Webots run (the .npz written by epuck_bridge).

  python3 ref/check_run.py [sim/ref_run.npz] [--walls 24,25,...]

Reports the cells the robot drove through, the closest it came to a wall,
where it stopped, and replays every recorded frame through ref/brain.py to
confirm the recorded wheel speeds are reproduced exactly (the property that
makes the recording usable as test vectors for the Verilog).
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import brain as B  # noqa: E402

BOX_HALF_MM = 63.75   # walls are 0.85 * 150 mm boxes
ROBOT_RADIUS_MM = 37


def wall_gap(x, y, cells):
    best = float('inf')
    for c in cells:
        cx, cy = B.center_mm(c)
        dx = max(abs(x - cx) - BOX_HALF_MM, 0)
        dy = max(abs(y - cy) - BOX_HALF_MM, 0)
        best = min(best, (dx * dx + dy * dy) ** 0.5)
    return best - ROBOT_RADIUS_MM


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', nargs='?', default='sim/ref_run.npz')
    args = parser.parse_args()

    d = dict(np.load(args.run))
    t, pose, depth, wheels = d['t'], d['pose'], d['depth'], d['wheels']
    obstacles = [int.from_bytes(bytes(b), 'little') for b in d['obstacles']]
    print(f'{len(t)} frames, {t[0]:.2f} s to {t[-1]:.2f} s, final status {d["status"][-1]}')

    cells = [B.cell_of(int(x), int(y)) for x, y, _ in pose]
    route = [cells[0]] + [c for prev, c in zip(cells, cells[1:]) if c != prev]
    print('cells driven:', ' '.join(map(str, route)))

    gaps = [wall_gap(int(x), int(y), [c for c in range(64) if (m >> c) & 1])
            for (x, y, _), m in zip(pose, obstacles)]
    print(f'closest robot edge to a wall: {min(gaps):.1f} mm')
    gx, gy = B.center_mm(B.GOAL)
    x, y = int(pose[-1][0]), int(pose[-1][1])
    print(f'stopped at ({x}, {y}) mm, {((x - gx) ** 2 + (y - gy) ** 2) ** 0.5:.1f} mm from goal centre')

    brain = B.Brain()
    bad = [i for i in range(len(t))
           if tuple(brain.step(depth[i], *map(int, pose[i]))) != tuple(wheels[i])]
    print('replay through ref/brain.py:', 'all frames match' if not bad
          else f'{len(bad)} mismatches, first at frame {bad[0]}')
    sys.exit(1 if bad or min(gaps) <= 0 else 0)


main()
