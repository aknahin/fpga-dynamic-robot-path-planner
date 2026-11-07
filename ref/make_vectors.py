#!/usr/bin/env python3
"""Turn a recorded Webots run into test vectors for tb/tb_nexys3_hil.v.

  python3 ref/make_vectors.py [sim/ref_run.npz] [-o sim/hil_vectors.txt]

Each line is one packet or one expected reply, bytes in hex:
  P n b0 b1 ...   send this UDP payload to port 5006
  B n b0 b1 ...   send it with a bad Ethernet CRC (the FPGA must ignore it)
  W n b0 b1 ...   send it to port 5007 (the FPGA must ignore it)
  R b0 ... b23    the B5 reply ref/brain.py expects for the C2 just sent

The expected replies come from ref/brain.py fed exactly what the FPGA gets,
band by band like bridge/mock_fpga.py. On top of the recorded frames the
script mixes in the cases the hardware has to get right: ignored packets,
a lost band, wall clicks on the route, and a reset in the middle of a run.
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import brain as B  # noqa: E402
import packets as P  # noqa: E402


def line(kind, payload):
    return f'{kind} {len(payload)} ' + ' '.join(f'{b:02x}' for b in payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run', nargs='?', default='sim/ref_run.npz')
    parser.add_argument('-o', '--out', default='sim/hil_vectors.txt')
    parser.add_argument('--frames', type=int, default=0, help='limit (0 = all)')
    args = parser.parse_args()

    d = dict(np.load(args.run))
    depth, pose = d['depth'], d['pose']
    n = len(depth) if not args.frames else min(args.frames, len(depth))
    brain = B.Brain()
    camera_grid = 0
    lines = [line('P', P.pack_reset())]
    edit_cell = None
    notes = []

    def send_band(payload):
        nonlocal camera_grid
        lines.append(line('P', payload))
        _, _, band, pixels = P.parse_command(payload)
        row = B.detect_band(pixels)
        camera_grid = (camera_grid & ~(0xFF << band * 8)) | (row << band * 8)

    for i in range(n):
        frame = (i + 1) & 0xFF
        bands = P.pack_camera(frame, depth[i])
        if i == 5:   # look like a wall everywhere, but must be ignored
            fake = bands[2][:3] + bytes(512)
            lines.append(line('W', fake))
            lines.append(line('B', fake))
            notes.append('frame 5: wrong-port and bad-CRC bands')
        if i == n // 4 and brain.status == B.RUNNING:
            route = [c for c in range(63) if (brain.remaining_mask() >> c) & 1
                     and c not in (0, brain.robot_cell)]
            if route:
                edit_cell = route[-1]
                lines.append(line('P', P.pack_edit(True, edit_cell)))
                brain.edit(True, edit_cell)
                notes.append(f'frame {i}: wall clicked on cell {edit_cell}')
        if i == n // 2 and edit_cell is not None:
            lines.append(line('P', P.pack_edit(False, edit_cell)))
            brain.edit(False, edit_cell)
            notes.append(f'frame {i}: wall on cell {edit_cell} removed')
        if i == (3 * n) // 4:
            lines.append(line('P', P.pack_reset()))
            brain.reset()
            camera_grid = 0
            notes.append(f'frame {i}: reset')
        for band, payload in enumerate(bands):
            if i == n // 3 and band == 3:
                notes.append(f'frame {i}: band 3 lost')
                continue
            send_band(payload)
        x, y, h = (int(v) for v in pose[i])
        lines.append(line('P', P.pack_pose(frame, x, y, h)))
        wheels = brain.step_grid(camera_grid, x, y, h)
        lines.append('R ' + ' '.join(f'{b:02x}' for b in P.reply_from_brain(frame, brain, wheels)))

    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    with open(args.out, 'w') as f:
        f.write('\n'.join(lines) + '\nE\n')
    print(f'{n} frames -> {args.out} ({len(lines)} lines, {brain.replans} plans '
          f'since the last reset, final status {brain.status})')
    for note in notes:
        print('  ' + note)


main()
