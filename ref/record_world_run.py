#!/usr/bin/env python3
"""Record a run of the Python warehouse (ref/sim_world.py) driven by ref/brain_ff.py.

  python3 ref/record_world_run.py --seed 3 --frames 500 -o sim/ff_pyworld.npz

Same arrays as the Webots bridge records, so bridge/replay_frames_ff.py and
ref/make_ff_vectors.py --from-run accept either.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from brain_ff import BrainFF  # noqa: E402
from sim_world import World  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
ap.add_argument('--seed', type=int, default=3)
ap.add_argument('--frames', type=int, default=500)
ap.add_argument('-o', '--out', default='sim/ff_pyworld.npz')
args = ap.parse_args()

world, brain = World(seed=args.seed), BrainFF()
cams, poses, wheels_log, status = [], [], [], []
for _ in range(args.frames):
    images, pose = world.depth_images(), world.pose()
    wheels = brain.step(images, *pose)
    cams.append(np.array(images, dtype=np.uint8))
    poses.append(pose)
    wheels_log.append(wheels)
    status.append(brain.status)
    if brain.status == 'at_goal':
        break
    world.step(wheels)
os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
np.savez_compressed(args.out, cams=np.array(cams), pose=np.array(poses),
                    wheels=np.array(wheels_log), status=np.array(status))
print(f'{len(cams)} frames -> {args.out}, final status {brain.status}, collisions {world.collisions}')
