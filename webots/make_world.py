#!/usr/bin/env python3
"""Write the Webots warehouse world from the same scenario ref/sim_world.py uses.

  python3 webots/make_world.py --seed 3 -o webots/worlds/warehouse.wbt

16x16 cells of 0.15 m, 2.4 m arena. Racks come from ref/warehouse.py (the same
map the FPGA has built in). The seed picks the robot's start cell and heading,
the random boxes, one box that appears mid-run, and the phases of the two
moving boxes. The warehouse_supervisor controller reads the moving parts from
the supervisor robot's customData.
"""

import argparse
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ref'))
from sim_world import BOX_HALF_MM, LATE_MIN_DIST_MM, World  # noqa: E402
from warehouse import (CAM_COLS, CAM_ROWS, CELL_MM, GOAL, GRID, MOVER_LANES, RACK_MASK,  # noqa: E402
                       cell_xy)

PROTO = ('https://raw.githubusercontent.com/cyberbotics/webots/'
         '45dc6873b06b04de5edfab04c7d82f72cc0653ca/projects/')
RACK_HEIGHT = 0.20
BOX_HEIGHT = 0.12            # taller than the cameras (0.07 m), shorter than the racks
CAMERA_Z = 0.07              # the lower pixel row meets the floor beyond maxRange there
CAMERA_RANGE = 1.5


def col(r, g, b):
    return f'{r:.3f} {g:.3f} {b:.3f}'


def part(color, size, pos):
    """One coloured box of size (x, y, z) metres centred at pos, relative to its Solid."""
    return (f'Transform {{ translation {pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f} children [ Shape {{ '
            f'appearance PBRAppearance {{ baseColor {color} roughness 0.85 metalness 0 }} '
            f'geometry Box {{ size {size[0]:.4f} {size[1]:.4f} {size[2]:.4f} }} }} ] }}')


def solid(name, x_mm, y_mm, parts, bound, defname=None):
    """A Solid at (x, y) on the floor; bound = (sx, sy, sz) collision box standing on the floor."""
    head = f'DEF {defname} ' if defname else ''
    body = ' '.join(parts)
    return (f'{head}Solid {{\n  translation {x_mm / 1000.0:.4f} {y_mm / 1000.0:.4f} 0\n  name "{name}"\n'
            f'  children [ {body} ]\n'
            f'  boundingObject Transform {{ translation 0 0 {bound[2] / 2:.4f} children [ '
            f'Box {{ size {bound[0]:.4f} {bound[1]:.4f} {bound[2]:.4f} }} ] }}\n}}\n')


CARTONS = [(0.69, 0.53, 0.37), (0.78, 0.63, 0.46), (0.85, 0.76, 0.60), (0.60, 0.45, 0.30),
           (0.90, 0.88, 0.82), (0.72, 0.56, 0.40), (0.55, 0.62, 0.70), (0.80, 0.70, 0.52)]


def rack_unit(cell, x_mm, y_mm):
    """A steel shelving bay that fills its cell: blue posts, orange beams, cartons on every level.

    Everything stays inside the 0.15 m cell, and the two lower levels are filled solid up to
    0.134 m, so the depth cameras (7 cm high) see it as a solid block, as the FPGA assumes."""
    rnd = random.Random(1000 + cell)
    parts = [part(col(0.22, 0.22, 0.25), (0.15, 0.15, 0.008), (0, 0, 0.004))]       # plinth
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append(part(col(0.16, 0.36, 0.70), (0.013, 0.013, 0.20), (sx * 0.0685, sy * 0.0685, 0.10)))
    levels = [(0.008, 0.060, True), (0.078, 0.056, True), (0.144, 0.052, False)]
    for z0, h, full in levels:
        for ix in (-1, 1):
            for iy in (-1, 1):
                if not full and rnd.random() < 0.35:
                    continue                                                         # empty slot, top level only
                ch = h * rnd.uniform(0.82, 1.0)
                c = rnd.choice(CARTONS)
                parts.append(part(col(*c), (0.065, 0.065, ch), (ix * 0.0325, iy * 0.0325, z0 + ch / 2)))
                parts.append(part(col(c[0] * 0.8, c[1] * 0.8, c[2] * 0.8), (0.02, 0.0655, 0.002),
                                  (ix * 0.0325, iy * 0.0325, z0 + ch + 0.001)))     # tape stripe
        zb = z0 + h + 0.005
        for sy in (-1, 1):                                                           # front and back beams
            parts.append(part(col(0.93, 0.52, 0.08), (0.15, 0.008, 0.012), (0, sy * 0.0695, zb)))
    return solid(f'rack {cell}', x_mm, y_mm, parts, (0.15, 0.15, 0.20))


def box_node(name, x_mm, y_mm, rnd_seed, defname=None):
    """A carton on a wooden pallet, 110 mm square and 120 mm tall (what the FPGA sees as a box)."""
    rnd = random.Random(rnd_seed)
    c = rnd.choice(CARTONS)
    parts = [part(col(0.52, 0.38, 0.22), (0.11, 0.11, 0.02), (0, 0, 0.010)),
             part(col(*c), (0.108, 0.108, 0.098), (0, 0, 0.071)),
             part(col(c[0] * 0.75, c[1] * 0.75, c[2] * 0.75), (0.024, 0.109, 0.002), (0, 0, 0.121)),
             part(col(0.95, 0.95, 0.95), (0.05, 0.001, 0.03), (0, -0.0545, 0.07))]
    return solid(name, x_mm, y_mm, parts, (0.11, 0.11, 0.12), defname)


def cart_node(name, x_mm, y_mm, defname):
    """A moving obstacle: a yellow cart with a carton and a red beacon (110 mm square, 125 mm tall)."""
    parts = [part(col(0.95, 0.78, 0.10), (0.11, 0.11, 0.05), (0, 0, 0.035)),
             part(col(0.08, 0.08, 0.09), (0.112, 0.03, 0.03), (0, -0.04, 0.015)),
             part(col(0.08, 0.08, 0.09), (0.112, 0.03, 0.03), (0, 0.04, 0.015)),
             part(col(0.78, 0.63, 0.46), (0.09, 0.09, 0.05), (0, 0, 0.085)),
             part(col(0.90, 0.15, 0.12), (0.016, 0.016, 0.02), (0.03, 0.03, 0.115))]
    return solid(name, x_mm, y_mm, parts, (0.11, 0.11, 0.125), defname)


def floor_marks():
    """Painted lane markings: dashed yellow centre lines on the two cart lanes, a hatched delivery zone."""
    out = []
    for lane, (a, b) in enumerate(MOVER_LANES):
        (ax, ay), (bx, by) = cell_xy(a), cell_xy(b)
        n = int(max(abs(bx - ax), abs(by - ay)) // 100) + 1
        for i in range(n + 1):
            x = ax + (bx - ax) * i / n
            y = ay + (by - ay) * i / n
            horiz = abs(bx - ax) > abs(by - ay)
            size = (0.05, 0.008, 0.001) if horiz else (0.008, 0.05, 0.001)
            out.append(f'Solid {{ translation {x / 1000:.4f} {y / 1000:.4f} 0.0006 name "lane {lane}.{i}" children [ '
                       f'Shape {{ appearance PBRAppearance {{ baseColor 0.95 0.80 0.10 roughness 1 metalness 0 }} '
                       f'geometry Box {{ size {size[0]} {size[1]} {size[2]} }} }} ] }}')
    gx, gy = cell_xy(GOAL)
    for k, off in enumerate((-0.045, -0.015, 0.015, 0.045)):
        out.append(f'Solid {{ translation {gx / 1000:.4f} {gy / 1000 + off:.4f} 0.0012 name "zone stripe {k}" children [ '
                   f'Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 roughness 1 metalness 0 }} '
                   f'geometry Box {{ size 0.12 0.008 0.001 }} }} ] }}')
    return '\n'.join(out) + '\n'


def camera(index, yaw):
    return f'''    Pose {{
      translation 0.0095 0 {CAMERA_Z}
      rotation 0 0 1 {yaw:.5f}
      children [ RangeFinder {{
        name "cam{index}"
        fieldOfView {math.pi / 2:.5f}
        width {CAM_COLS}
        height {CAM_ROWS}
        minRange 0.05
        maxRange {CAMERA_RANGE}
      }} ]
    }}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seed', type=int, default=3)
    parser.add_argument('--quit-at-goal', action='store_true',
                        help='end the simulation when the robot has stopped on the delivery zone (headless runs)')
    parser.add_argument('--movie', help='record the 3D view to this mp4 (Webots recorder), headless runs')
    parser.add_argument('-o', '--out', default='webots/worlds/warehouse.wbt')
    args = parser.parse_args()

    w = World(seed=args.seed)
    sx, sy = cell_xy(w.start_cell)
    sx, sy = sx / 1000.0, sy / 1000.0
    gx, gy = cell_xy(GOAL)

    out = [f'''#VRML_SIM R2025a utf8
# Warehouse world (seed {args.seed}), generated by webots/make_world.py. Do not edit by hand.
# 16x16 cells of 0.15 m, 2.4 m arena. Cell = row*16 + col, row 0 north (+y), col 0 west (-x).
# Racks are known to the FPGA; boxes and the two moving boxes are found from the cameras.

EXTERNPROTO "{PROTO}objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "{PROTO}objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "{PROTO}objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "{PROTO}robots/gctronic/e-puck/protos/E-puck.proto"

WorldInfo {{
  info [ "FPGA flow-field warehouse robot" ]
  basicTimeStep 16
}}
Viewpoint {{
  orientation 0.5 0.5 -0.7071 1.7
  position -0.80 2.18 4.65
}}
TexturedBackground {{ }}
TexturedBackgroundLight {{ }}
RectangleArena {{
  floorSize 2.4 2.4
  floorTileSize 0.3 0.3
  floorAppearance PBRAppearance {{ baseColor 0.50 0.51 0.53 roughness 0.95 metalness 0 }}
  wallHeight 0.15
  wallAppearance PBRAppearance {{ baseColor 0.78 0.78 0.80 roughness 0.9 metalness 0 }}
}}
# Delivery zone on the south-east corner cell. Flat, so the cameras do not see it.
Solid {{
  translation {gx / 1000.0:.4f} {gy / 1000.0:.4f} 0.001
  children [ Shape {{ appearance PBRAppearance {{ baseColor 1 0.85 0.1 roughness 1 metalness 0 }}
    geometry Box {{ size 0.13 0.13 0.002 }} }} ]
  name "delivery zone"
}}
''']
    out.append(floor_marks())
    for c in range(GRID * GRID):
        if (RACK_MASK >> c) & 1:
            x, y = cell_xy(c)
            out.append(rack_unit(c, x, y))
    for cell, t_on in w.boxes:
        if t_on == 0.0:
            x, y = cell_xy(cell)
            out.append(box_node(f'box {cell}', x, y, 5000 + cell))
    for i, m in enumerate(w.movers):
        x, y = m.pos(0.0)
        out.append(cart_node(f'cart {i}', x, y, f'MOVER{i}'))

    scenario = dict(
        movers=[dict(a=list(m.a), b=list(m.b), speed=m.speed, phase=m.phase) for m in w.movers],
        late=dict(t=w.appear_at, min_dist_mm=LATE_MIN_DIST_MM,
                  candidates=[dict(cell=c, xy=list(cell_xy(c)), node=box_node(f'late box {c}', *cell_xy(c), 5000 + c))
                              for c in w.late_candidates]),
        goal=list(cell_xy(GOAL)), box_side_mm=2 * BOX_HALF_MM, box_height=BOX_HEIGHT)
    parts = (['"--quit-at-goal"'] if args.quit_at_goal else []) + \
            ([f'"--movie" "{os.path.abspath(args.movie)}"'] if args.movie else [])
    controller_args = ' '.join(parts)
    out.append(f'''# Starts on cell {w.start_cell}.
DEF EPUCK E-puck {{
  translation {sx:.4f} {sy:.4f} 0
  rotation 0 0 1 {w.heading:.5f}
  controller "epuck_bridge_ff"
  turretSlot [
    # The turret slot sits 9.5 mm behind the wheel axle; shift the GPS forward
    # so it reports the robot's centre.
    Pose {{
      translation 0.0095 0 0
      children [ GPS {{ }} ]
    }}
    Compass {{ }}
{camera(0, 0.0)}{camera(1, math.pi / 2)}{camera(2, math.pi)}{camera(3, -math.pi / 2)}    # Colour camera for the robot's-eye video only: the brain never sees it.
    Pose {{
      translation 0.04 0 0.075
      children [ Camera {{ name "eye" width 480 height 270 fieldOfView 1.2 near 0.01 }} ]
    }}
  ]
}}
# Moves the two moving boxes and drops the late box (no decisions, no sensing).
Robot {{
  name "warehouse rig"
  controller "warehouse_supervisor"
  controllerArgs [ {controller_args} ]
  supervisor TRUE
  customData {json.dumps(json.dumps(scenario))}
}}
''')
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    with open(args.out, 'w') as f:
        f.write(''.join(out))
    print(f'{args.out}: start cell {w.start_cell}, {len(w.boxes)} boxes, 1 late box from t={w.appear_at} s '
          f'(>= {LATE_MIN_DIST_MM:.0f} mm from the robot), {len(w.movers)} movers')


main()
