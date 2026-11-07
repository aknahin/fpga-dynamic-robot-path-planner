"""e-puck side of the bridge.

Each step it collects the sensor data the FPGA gets: the overhead depth
image (from the Receiver) and the robot pose (GPS + Compass). It formats
them with ref/hil_format.py, hands them to the brain, and applies the
wheel speeds that come back without changing them. It makes no decisions.

Brains:
  ref    the Python golden reference in ref/brain.py, run in-process
  udp    over the network (docs/packets.md): bridge/mock_fpga.py or the board
  none   wheels stay still; only record sensor data

Every frame (inputs and outputs) is recorded to an .npz, which serves as
test vectors for the Verilog and can be checked with ref/check_run.py.

controllerArgs (all optional; the env vars set the defaults, so a run can be
switched from the terminal without editing the world):
  --brain ref|udp|none   (env HIL_BRAIN, default ref)
  --target IP            (env HIL_TARGET, default 127.0.0.1 = mock on this
                          Mac; use 192.168.10.255 for the board)
  --record PATH          (default sim/<brain>_run.npz)
"""

import argparse
import os
import sys

import numpy as np
from controller import Robot

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'ref'))
sys.path.insert(0, os.path.join(REPO, 'bridge'))
from hil_format import (IMAGE_SIZE, WHEEL_UNIT, encode_depth, encode_pose,  # noqa: E402
                        heading_deg_from_compass)
import brain as ref_brain  # noqa: E402


def cells(mask):
    return [c for c in range(64) if (mask >> c) & 1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--brain', default=os.environ.get('HIL_BRAIN', 'ref'),
                        choices=['ref', 'udp', 'none'])
    parser.add_argument('--target', default=os.environ.get('HIL_TARGET', '127.0.0.1'))
    parser.add_argument('--record')
    args = parser.parse_args()
    record = args.record or os.path.join(REPO, 'sim', f'{args.brain}_run.npz')

    robot = Robot()
    step = int(robot.getBasicTimeStep()) * 2
    gps = robot.getDevice('gps')
    compass = robot.getDevice('compass')
    receiver = robot.getDevice('depth receiver')
    for dev in (gps, compass, receiver):
        dev.enable(step)
    left = robot.getDevice('left wheel motor')
    right = robot.getDevice('right wheel motor')
    for motor in (left, right):
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)

    brain = link = None
    if args.brain == 'ref':
        brain = ref_brain.Brain()
    elif args.brain == 'udp':
        from udp_link import UdpLink
        link = UdpLink(args.target)
        link.reset()
        print(f'[bridge] brain over UDP, sending to {args.target}')

    log = {k: [] for k in ('t', 'depth', 'pose', 'wheels', 'obstacles', 'robot_cell', 'status')}
    depth = None
    last_status = None
    last_replans = None
    recorded = False
    next_save = 5.0
    next_timeout_note = 0.0

    def save():
        os.makedirs(os.path.dirname(record), exist_ok=True)
        np.savez_compressed(record, **{k: np.array(v) for k, v in log.items()})

    while robot.step(step) != -1:
        while receiver.getQueueLength() > 0:
            data = receiver.getBytes()
            receiver.nextPacket()
            raw = np.frombuffer(data, dtype=np.float32).reshape(IMAGE_SIZE, IMAGE_SIZE)
            depth = encode_depth(raw)
        if depth is None:
            continue

        x, y, _ = gps.getValues()
        pose = encode_pose(x, y, heading_deg_from_compass(compass.getValues()))
        t = robot.getTime()

        replans, route = last_replans, 0
        if brain is not None:
            wheels = brain.step(depth, *pose)
            status, obstacles, cell = brain.status, brain.obstacles, brain.robot_cell
            replans, route = brain.replans, brain.route_mask()
        elif link is not None:
            reply = link.exchange(depth, pose)
            if reply is None:
                # Lost frame: stop for one step. Logged as 'timeout', but it
                # does not count as a status change on the console.
                wheels, status, obstacles, cell = (0, 0), 'timeout', 0, -1
                if t >= next_timeout_note:
                    print(f'[bridge] t={t:6.2f} no reply for frame {link.frame} '
                          f'({link.timeouts} so far), wheels stopped for one step')
                    next_timeout_note = t + 1.0
            else:
                wheels = (reply['left'], reply['right'])
                status, obstacles, cell = reply['status'], reply['obstacles'], reply['robot_cell']
                replans, route = reply['replans'], reply['route']
        else:
            wheels, status, obstacles, cell = (0, 0), 'none', 0, -1

        if replans != last_replans:
            print(f'[brain] t={t:6.2f} plan #{replans} from cell {cell}, route cells {cells(route)}')
            last_replans = replans
        changed = status != last_status and status != 'timeout'
        if changed:
            print(f'[brain] t={t:6.2f} status {status}')
            last_status = status

        left.setVelocity(wheels[0] * WHEEL_UNIT)
        right.setVelocity(wheels[1] * WHEEL_UNIT)
        if recorded:
            continue

        log['t'].append(t)
        log['depth'].append(depth)
        log['pose'].append(pose)
        log['wheels'].append(wheels)
        log['obstacles'].append(obstacles.to_bytes(8, 'little'))
        log['robot_cell'].append(cell)
        log['status'].append(status)
        if t >= next_save or changed:
            next_save = t + 5.0
            save()
        if status == 'at_goal':
            recorded = True  # the run is over; keep the file small
            extra = f', {link.timeouts} timeouts' if link else ''
            print(f'[bridge] recording saved: {len(log["t"])} frames in {record}{extra}')


main()
