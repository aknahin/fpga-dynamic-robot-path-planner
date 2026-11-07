#!/usr/bin/env python3
"""Run a warehouse scenario and write test vectors for the flow-field RTL.

  python3 ref/make_ff_vectors.py [--seed 3] [--frames 600] [-o sim/ff_vectors.txt]
  python3 ref/make_ff_vectors.py --from-run sim/ff_webots.npz   (a recorded Webots run)

Each line is one packet or one expected reply, bytes in hex:
  P n b0 b1 ...   send this UDP payload to port 5006
  B n b0 b1 ...   send it with a bad Ethernet CRC (the FPGA must ignore it)
  W n b0 b1 ...   send it to port 5007 (the FPGA must ignore it)
  R b0 ... b71    the B6 reply ref/brain_ff.py expects for the C2 just sent

The expected replies come from ref/brain_ff.py fed exactly what the FPGA gets
(the latest strip of every camera, the pose, the edits). On top of the frames
the script mixes in the cases the hardware has to get right: ignored packets,
a lost camera strip, a wall click on the route and its removal, and a reset
in the middle of a run.
"""

import argparse
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import packets_ff as P  # noqa: E402
from brain_ff import RUNNING, BrainFF  # noqa: E402
from sim_world import World  # noqa: E402
from warehouse import CAM_COLS, CAM_ROWS, CAMERAS, GOAL, RACK_MASK  # noqa: E402


def line(kind, payload):
    return f'{kind} {len(payload)} ' + ' '.join(f'{b:02x}' for b in payload)


def frames_from_world(seed, n):
    """Generator of (images, pose) from the Python warehouse simulator."""
    world = World(seed=seed)
    brain = BrainFF()
    for _ in range(n):
        images, pose = world.depth_images(), world.pose()
        yield images, pose
        # Drive the simulation with the reference brain so the run is realistic.
        world.step(brain.step(images, *pose))


def frames_from_run(path):
    d = dict(np.load(path))
    for images, pose in zip(d['cams'], d['pose']):
        yield [img for img in images], tuple(int(v) for v in pose)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seed', type=int, default=3)
    parser.add_argument('--frames', type=int, default=600)
    parser.add_argument('--from-run')
    parser.add_argument('--cage', action='store_true',
                        help='wall in the goal for a while (no-route case), then open it')
    parser.add_argument('--stress', action='store_true',
                        help='click random walls every few frames (blocked targets, no-route cases)')
    parser.add_argument('-o', '--out', default='sim/ff_vectors.txt')
    args = parser.parse_args()

    source = frames_from_run(args.from_run) if args.from_run else frames_from_world(args.seed, args.frames)
    brain = BrainFF()
    held = [np.full((CAM_ROWS, CAM_COLS), 255, dtype=np.uint8) for _ in range(CAMERAS)]
    lines = [line('P', P.pack_reset())]
    notes = []
    edit_cell = None
    n = args.frames
    rng = random.Random(args.seed)
    clicked = set()

    for i, (images, (x, y, h)) in enumerate(source):
        frame = (i + 1) & 0xFF
        if i == 5:   # look like a wall everywhere, but must be ignored
            fake = P.pack_camera(frame, 1, np.zeros((CAM_ROWS, CAM_COLS), dtype=np.uint8))
            lines.append(line('W', fake))
            lines.append(line('B', fake))
            notes.append('frame 5: wrong-port and bad-CRC strips')
        if i == n // 4 and brain.status == RUNNING:
            path_cells = [c for c in range(256) if c not in (GOAL, brain.robot_cell, brain.target)
                          and not (RACK_MASK >> c) & 1]
            # A cell two steps ahead of the robot, on its way.
            ahead = [c for c in path_cells if abs(c // 16 - brain.target // 16) +
                     abs(c % 16 - brain.target % 16) == 2]
            if ahead:
                edit_cell = ahead[0]
                lines.append(line('P', P.pack_edit(True, edit_cell)))
                brain.edit(True, edit_cell)
                notes.append(f'frame {i}: wall clicked on cell {edit_cell}')
        if i == n // 2 and edit_cell is not None:
            lines.append(line('P', P.pack_edit(False, edit_cell)))
            brain.edit(False, edit_cell)
            notes.append(f'frame {i}: wall on cell {edit_cell} removed')
        if args.cage and i == n // 3:
            for cell in (239, 254):
                lines.append(line('P', P.pack_edit(True, cell)))
                brain.edit(True, cell)
            notes.append(f'frame {i}: goal walled in')
        if args.cage and i == (2 * n) // 3:
            lines.append(line('P', P.pack_edit(False, 254)))
            brain.edit(False, 254)
            notes.append(f'frame {i}: goal opened again')
        if args.stress and i % 12 == 6 and i > 20:
            cell = rng.choice([c for c in range(256) if not (RACK_MASK >> c) & 1 and c != GOAL])
            add = cell not in clicked
            clicked.symmetric_difference_update({cell})
            lines.append(line('P', P.pack_edit(add, cell)))
            brain.edit(add, cell)
        if i == (3 * n) // 4 and not args.cage:
            lines.append(line('P', P.pack_reset()))
            brain.reset()
            notes.append(f'frame {i}: reset')
        for cam in range(CAMERAS):
            if i == n // 3 and cam == 2:
                notes.append(f'frame {i}: back camera strip lost')
                continue
            payload = P.pack_camera(frame, cam, images[cam])
            lines.append(line('P', payload))
            held[cam] = np.asarray(images[cam], dtype=np.uint8).reshape(CAM_ROWS, CAM_COLS)
        lines.append(line('P', P.pack_pose(frame, x, y, h)))
        wheels = brain.step([h_.tolist() for h_ in held], x, y, h)
        lines.append('R ' + ' '.join(f'{b:02x}' for b in P.reply_from_brain(frame, brain, wheels)))

    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    with open(args.out, 'w') as f:
        f.write('\n'.join(lines) + '\nE\n')
    print(f'{n} frames -> {args.out} ({len(lines)} lines, {brain.replans} plans '
          f'since the last reset, final status {brain.status})')
    for note in notes:
        print('  ' + note)


main()
