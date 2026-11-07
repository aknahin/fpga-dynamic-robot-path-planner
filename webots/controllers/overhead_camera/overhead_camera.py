"""Overhead depth camera and scenario supervisor.

Every step it sends the raw 64x64 depth image (float32, metres, row-major,
row 0 = north) to the e-puck over the Emitter. It also builds the scenario:
static walls at start, and one obstacle that drops in later. It makes no
driving or planning decisions; those belong to the FPGA.

controllerArgs (all optional):
  --walls 24,25,26          cells that hold a wall from the start
  --spawn 59@c36            drop an obstacle on cell 59 when the robot enters
                            cell 36 (the default); use 59@t20 to drop it at t = 20 s instead
"""

import argparse
import os
import sys

import numpy as np
from controller import Supervisor

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'ref'))
from hil_format import CELL_SIZE, WALL_HEIGHT, cell_at, cell_center  # noqa: E402

DEFAULT_WALLS = '24,25,26,27,28,29,42,43,44,45,46,47'
DEFAULT_SPAWN = '59@c36'


def block_string(cell, name, color):
    x, y = cell_center(cell)
    side = CELL_SIZE * 0.85
    return (
        f'Solid {{ translation {x:.4f} {y:.4f} {WALL_HEIGHT / 2:.4f} '
        f'name "{name}" children [ Shape {{ '
        f'appearance PBRAppearance {{ baseColor {color} roughness 1 metalness 0 }} '
        f'geometry Box {{ size {side:.4f} {side:.4f} {WALL_HEIGHT:.4f} }} }} ] '
        f'boundingObject Box {{ size {side:.4f} {side:.4f} {WALL_HEIGHT:.4f} }} }}'
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--walls', default=DEFAULT_WALLS)
    parser.add_argument('--spawn', default=DEFAULT_SPAWN)
    args = parser.parse_args()

    sup = Supervisor()
    step = int(sup.getBasicTimeStep()) * 2
    camera = sup.getDevice('depth camera')
    camera.enable(step)
    emitter = sup.getDevice('depth emitter')
    root = sup.getRoot().getField('children')
    epuck = sup.getFromDef('EPUCK')

    for cell in [int(c) for c in args.walls.split(',') if c]:
        root.importMFNodeFromString(-1, block_string(cell, f'wall {cell}', '0.55 0.55 0.6'))

    spawn_cell, trigger = None, None
    if args.spawn:
        cell, trigger = args.spawn.split('@')
        spawn_cell = int(cell)

    while sup.step(step) != -1:
        if spawn_cell is not None:
            x, y, _ = epuck.getPosition()
            due = (sup.getTime() >= float(trigger[1:]) if trigger[0] == 't'
                   else cell_at(x, y) == int(trigger[1:]))
            if due:
                root.importMFNodeFromString(
                    -1, block_string(spawn_cell, 'new obstacle', '0.85 0.3 0.25'))
                print(f'[overhead] obstacle dropped on cell {spawn_cell} at t={sup.getTime():.2f} s')
                spawn_cell = None

        image = camera.getRangeImage()
        if image:
            emitter.send(np.asarray(image, dtype=np.float32).tobytes())


main()
