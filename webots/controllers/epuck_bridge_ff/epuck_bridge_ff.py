"""e-puck side of the bridge for the flow-field warehouse robot.

Each step it collects the sensor data the FPGA gets: four depth strips from
the robot's own range finders (front, left, back, right) and the robot pose
(GPS + Compass). It only changes units (metres to depth bytes, see
ref/warehouse.py), hands them to the brain, and applies the wheel speeds that
come back without changing them. It makes no decisions.

Brains:
  ref    the Python golden reference in ref/brain_ff.py, run in-process
  udp    over the network (docs/packets_ff.md): bridge/mock_fpga_ff.py or the board
  none   wheels stay still; only record sensor data

Every frame (inputs and outputs) is recorded to an .npz with arrays `cams`
(n,4,2,32), `pose`, `wheels`, `status`, `blocked`, `movers`; replay it with
bridge/replay_frames_ff.py or turn it into RTL vectors with
ref/make_ff_vectors.py --from-run.

controllerArgs (all optional; the env vars set the defaults, so a run can be
switched from the terminal without editing the world):
  --brain ref|udp|none   (env HIL_BRAIN, default ref)
  --target IP            (env HIL_TARGET, default 127.0.0.1 = mock on this
                          Mac; use 192.168.10.255 for the board)
  --record PATH          (env HIL_RECORD; default sim/ff_webots_<brain>.npz)

Env HIL_EYE=1 shows the robot's-eye camera as Webots' overlay (live runs).
Env HIL_EYE_DIR=dir saves the robot's-eye colour camera (a plain forward camera used only
for the video, never given to the brain) as dir/f00000.png ... every step.
"""

import argparse
import os
import sys

import numpy as np
from controller import Robot

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'ref'))
sys.path.insert(0, os.path.join(REPO, 'bridge'))
from warehouse import CAM_COLS, CAM_ROWS, CAMERAS, DEPTH_UNIT_MM  # noqa: E402
import brain_ff  # noqa: E402

WHEEL_UNIT = 0.05    # rad/s per wheel-speed step (ref/sim_world.py, docs/packets_ff.md)


def encode_strip(raw):
    """Range-finder metres (row-major, inf = nothing in range) -> uint8 depth bytes."""
    mm = np.asarray(raw, dtype=np.float64).reshape(CAM_ROWS, CAM_COLS) * 1000.0
    mm = np.where(np.isfinite(mm), mm, 1e9)
    return np.clip(np.rint(mm / DEPTH_UNIT_MM), 0, 255).astype(np.uint8)


def encode_pose(x, y, north):
    """GPS (m) and Compass vector -> x_mm int16, y_mm int16, heading u16 (0 = east, CCW)."""
    heading_deg = np.degrees(np.arctan2(north[0], north[1]))
    return (int(round(x * 1000)), int(round(y * 1000)),
            int(round(heading_deg / 360.0 * 65536)) & 0xFFFF)


def cells(mask):
    return [c for c in range(256) if (mask >> c) & 1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--brain', default=os.environ.get('HIL_BRAIN', 'ref'),
                        choices=['ref', 'udp', 'none'])
    parser.add_argument('--target', default=os.environ.get('HIL_TARGET', '127.0.0.1'))
    parser.add_argument('--record')
    args = parser.parse_args()
    record = args.record or os.environ.get('HIL_RECORD') or os.path.join(REPO, 'sim', f'ff_webots_{args.brain}.npz')

    robot = Robot()
    step = int(robot.getBasicTimeStep()) * 2
    gps = robot.getDevice('gps')
    compass = robot.getDevice('compass')
    cams = [robot.getDevice(f'cam{i}') for i in range(CAMERAS)]
    for dev in [gps, compass] + cams:
        dev.enable(step)
    eye_dir = os.environ.get('HIL_EYE_DIR')
    show_eye = os.environ.get('HIL_EYE') == '1'   # live runs: show the robot's-eye view as Webots' camera overlay
    eye = robot.getDevice('eye') if (eye_dir or show_eye) else None
    if eye:
        if eye_dir:
            os.makedirs(eye_dir, exist_ok=True)
        eye.enable(step)
    eye_n = 0
    left = robot.getDevice('left wheel motor')
    right = robot.getDevice('right wheel motor')
    for motor in (left, right):
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)

    brain = link = None
    if args.brain == 'ref':
        brain = brain_ff.BrainFF()
    elif args.brain == 'udp':
        from udp_link_ff import UdpLink
        link = UdpLink(args.target)
        link.reset()
        print(f'[bridge] brain over UDP, sending to {args.target}')

    log = {k: [] for k in ('t', 'cams', 'pose', 'wheels', 'status', 'blocked', 'movers')}
    last_status = None
    last_replans = None
    recorded = False
    next_save = 5.0
    next_timeout_note = 0.0

    def save():
        os.makedirs(os.path.dirname(record), exist_ok=True)
        np.savez_compressed(record, t=np.array(log['t']), cams=np.array(log['cams']),
                            pose=np.array(log['pose']), wheels=np.array(log['wheels']),
                            status=np.array(log['status']),
                            blocked=np.array([b.to_bytes(32, 'little') for b in log['blocked']]),
                            movers=np.array([m.to_bytes(32, 'little') for m in log['movers']]))

    while robot.step(step) != -1:
        strips = np.array([encode_strip(c.getRangeImage()) for c in cams])
        x, y, _ = gps.getValues()
        pose = encode_pose(x, y, compass.getValues())
        t = robot.getTime()

        replans, blocked, movers, cell = last_replans, 0, 0, -1
        if brain is not None:
            wheels = brain.step(strips.tolist(), *pose)
            status, cell = brain.status, brain.robot_cell
            replans, blocked, movers = brain.replans, brain.blocked_mask(), brain.mover_mask()
        elif link is not None:
            reply = link.exchange(strips, pose)
            if reply is None:
                # Lost frame: stop for one step. Logged as 'timeout', but it
                # does not count as a status change on the console.
                wheels, status = (0, 0), 'timeout'
                if t >= next_timeout_note:
                    print(f'[bridge] t={t:6.2f} no reply for frame {link.frame} '
                          f'({link.timeouts} so far), wheels stopped for one step')
                    next_timeout_note = t + 1.0
            else:
                wheels = (reply['left'], reply['right'])
                status, cell = reply['status'], reply['robot_cell']
                replans, blocked, movers = reply['replans'], reply['blocked'], reply['movers']
        else:
            wheels, status = (0, 0), 'none'

        if replans != last_replans and status != 'timeout':
            print(f'[brain] t={t:6.2f} new waypoint decision #{replans} in cell {cell}')
            last_replans = replans
        changed = status != last_status and status != 'timeout'
        if changed:
            print(f'[brain] t={t:6.2f} status {status}')
            last_status = status

        left.setVelocity(wheels[0] * WHEEL_UNIT)
        right.setVelocity(wheels[1] * WHEEL_UNIT)
        if eye and eye_dir and not recorded:
            eye.saveImage(os.path.join(eye_dir, f'f{eye_n:05d}.png'), 90)
            eye_n += 1
        if recorded:
            continue

        log['t'].append(t)
        log['cams'].append(strips)
        log['pose'].append(pose)
        log['wheels'].append(wheels)
        log['status'].append(status)
        log['blocked'].append(blocked)
        log['movers'].append(movers)
        if t >= next_save or changed:
            next_save = t + 5.0
            save()
        if status == 'at_goal':
            recorded = True  # the run is over; keep the file small
            save()
            extra = f', {link.timeouts} timeouts' if link else ''
            print(f'[bridge] recording saved: {len(log["t"])} frames in {record}{extra}')


main()
