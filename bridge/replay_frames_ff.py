#!/usr/bin/env python3
"""Send a recorded run to the FPGA (or the mock) and check every reply.

Replays the camera strips and poses of a recording (sim/*.npz with arrays
`cams` (n,4,2,32) and `pose` (n,3)) over UDP, without Webots, and compares
each B6 reply with ref/brain_ff.py fed the same frame: wheels, status, robot
cell, target cell, blocked and mover masks must all match. This is the
bit-exact test for the Verilog brain on the real board.

  python3 bridge/replay_frames_ff.py                         # mock on this Mac
  python3 bridge/replay_frames_ff.py --target 192.168.10.255 # the board
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ref'))
sys.path.insert(0, os.path.dirname(__file__))
import brain_ff  # noqa: E402
from udp_link_ff import UdpLink  # noqa: E402

FIELDS = ('left', 'right', 'status', 'robot_cell', 'target_cell', 'blocked', 'movers')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run', nargs='?', default='sim/ff_webots.npz')
    parser.add_argument('--target', default='127.0.0.1')
    parser.add_argument('--show', type=int, default=5, help='mismatches to print')
    parser.add_argument('--rtt', help='write the round trip of every frame (frame,rtt_s) to this csv')
    args = parser.parse_args()

    d = dict(np.load(args.run))
    cams, pose = d['cams'], d['pose']
    link = UdpLink(args.target)
    link.reset()
    brain = brain_ff.BrainFF()
    bad = shown = 0
    rtts = []
    for i in range(len(cams)):
        p = tuple(int(v) for v in pose[i])
        t0 = time.perf_counter()
        reply = link.exchange(cams[i], p)
        rtts.append((i, time.perf_counter() - t0))
        wheels = brain.step(cams[i].tolist(), *p)
        want = dict(left=wheels[0], right=wheels[1], status=brain.status,
                    robot_cell=brain.robot_cell, target_cell=brain.target_cell(),
                    blocked=brain.blocked_mask(), movers=brain.mover_mask())
        diff = ['no reply'] if reply is None else \
            [f'{k} got {reply[k]} want {want[k]}' for k in FIELDS if reply[k] != want[k]]
        if diff:
            bad += 1
            if shown < args.show:
                shown += 1
                print(f'frame {i}: ' + ', '.join(diff))
    if args.rtt:
        os.makedirs(os.path.dirname(args.rtt) or '.', exist_ok=True)
        with open(args.rtt, 'w') as f:
            f.write('frame,rtt_s\n' + ''.join(f'{i},{r:.6f}\n' for i, r in rtts))
    print(f'{len(cams)} frames sent to {args.target}: '
          + ('all replies match ref/brain_ff.py' if not bad else f'{bad} frames differ')
          + f' ({link.timeouts} timeouts)')
    sys.exit(1 if bad else 0)


main()
