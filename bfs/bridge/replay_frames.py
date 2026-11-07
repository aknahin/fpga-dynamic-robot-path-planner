#!/usr/bin/env python3
"""Send a recorded run to the FPGA (or the mock) and check every reply.

Replays the depth images and poses of a recording (sim/*_run.npz) over UDP,
without Webots, and compares each B5 reply with ref/brain.py fed the same
frame: wheels, status, robot cell, target cell, obstacles and route must all
match. This is the bit-exact test for the Verilog brain on the real board.

  python3 bridge/replay_frames.py                         # mock on this Mac
  python3 bridge/replay_frames.py --target 192.168.10.255 # the board
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ref'))
sys.path.insert(0, os.path.dirname(__file__))
import brain as ref_brain  # noqa: E402
from udp_link import UdpLink  # noqa: E402

FIELDS = ('left', 'right', 'status', 'robot_cell', 'target_cell', 'obstacles', 'route')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run', nargs='?', default='sim/ref_run.npz')
    parser.add_argument('--target', default='127.0.0.1')
    parser.add_argument('--show', type=int, default=5, help='mismatches to print')
    args = parser.parse_args()

    d = dict(np.load(args.run))
    depth, pose = d['depth'], d['pose']
    link = UdpLink(args.target)
    link.reset()
    brain = ref_brain.Brain()
    bad = shown = 0
    for i in range(len(depth)):
        p = tuple(int(v) for v in pose[i])
        reply = link.exchange(depth[i], p)
        wheels = brain.step(depth[i], *p)
        want = dict(left=wheels[0], right=wheels[1], status=brain.status,
                    robot_cell=brain.robot_cell, target_cell=brain.target_cell(),
                    obstacles=brain.obstacles, route=brain.route_mask())
        diff = ['no reply'] if reply is None else \
            [f'{k} got {reply[k]} want {want[k]}' for k in FIELDS if reply[k] != want[k]]
        if diff:
            bad += 1
            if shown < args.show:
                shown += 1
                print(f'frame {i}: ' + ', '.join(diff))
    print(f'{len(depth)} frames sent to {args.target}: '
          + ('all replies match ref/brain.py' if not bad else f'{bad} frames differ')
          + f' ({link.timeouts} timeouts)')
    sys.exit(1 if bad else 0)


main()
