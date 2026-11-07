"""Warehouse layout and fixed numbers shared by ref/, bridge/, Webots and the RTL.

16x16 cells of 150 mm, arena centred on the origin (2.4 m square).
Cell = row*16 + col. Row 0 is the north edge (+y), column 0 the west edge (-x).
The goal (delivery zone) is the south-east corner, cell 255.

Only the racks are known to the FPGA ahead of time. Boxes and movers are found
from the four robot cameras. `python3 ref/warehouse.py` writes the generated
Verilog table module (rtl/ff_tables.v) so the RTL cannot drift
from this file.
"""

import math
import os

GRID = 16
CELLS = GRID * GRID
CELL_MM = 150
HALF_MM = GRID * CELL_MM // 2          # 1200
GOAL = CELLS - 1

LAYOUT = [
    "................",
    ".##.####.##.###.",
    ".##.####.##.###.",
    ".##......##.....",
    ".##.##.##...##..",
    ".##.##.##...##.#",
    ".......##...##.#",
    ".###.##.........",
    ".###.##...####..",
    ".....##...####..",
    ".##.###.##......",
    ".##.###.##.##...",
    ".##........##.#.",
    "..####..###...#.",
    "..####..###.....",
    "................",
]

# Two moving boxes (other robots / carts) patrol straight runs of free cells, as (cell A, cell B).
# Both lanes run through areas at least two cells wide (column 9 from row 4 to 9, and row 14 from
# column 11 to 15 in the loading area), so the robot can always pass a cart. In one-cell corridors a
# patrolling cart is a moving wall and causes livelocks and contact (see docs/RESULTS.md).
MOVER_LANES = [(4 * 16 + 9, 9 * 16 + 9), (14 * 16 + 11, 14 * 16 + 15)]

RACK_MASK = 0
for _r, _line in enumerate(LAYOUT):
    assert len(_line) == GRID
    for _c, _ch in enumerate(_line):
        if _ch == '#':
            RACK_MASK |= 1 << (_r * GRID + _c)

LANE_MASK = 0
for _a, _b in MOVER_LANES:   # every lane must be a straight run of free cells
    (_ra, _ca), (_rb, _cb) = divmod(_a, GRID), divmod(_b, GRID)
    assert _ra == _rb or _ca == _cb
    for _r in range(min(_ra, _rb), max(_ra, _rb) + 1):
        for _c in range(min(_ca, _cb), max(_ca, _cb) + 1):
            assert not (RACK_MASK >> (_r * GRID + _c)) & 1, ('lane crosses a rack', _r, _c)
            LANE_MASK |= 1 << (_r * GRID + _c)

# Cameras: front, left, back, right (yaw 0, +90, 180, -90 degrees, CCW).
CAMERAS = 4
CAM_COLS = 32
CAM_ROWS = 2
CAM_FOV_DEG = 90
DEPTH_UNIT_MM = 6                       # depth byte = depth / 6 mm
NO_HIT = 250                            # byte >= this means "nothing in range"
RANGE_MM = 1500                         # assumed free distance on a no-hit ray
HIT_PENETRATION_MM = 12                 # push the hit point a little into the obstacle
CAM_BYTES = CAM_COLS * CAM_ROWS

# Division by 150 as a multiply-shift, exact for 0..2399.
DIV150_MUL, DIV150_SHIFT = 437, 16

# Flow field weights.
INF = 1023
HALO_COSTS = (10, 6, 3)                  # extra cost 1, 2, 3 cells from recent motion
MOVER_TTL = 63                           # frames (~2 s) a vacated cell stays flagged
OCC_BLOCK = 2
OCC_MAX = 3
AGE_PERIOD = 32                          # unobserved cells fade 1 occupancy step every AGE_PERIOD frames, but only
                                         # on known cart-lane cells, or anywhere while there is no route at all
                                         # (deadlock breaker). Boxes elsewhere are remembered while a route exists.
HIT_EDGE_MM = 14                         # a hit this close to a cell edge is a rack face or wall, not a box
MISS_EDGE_MM = 20                        # a ray only proves a cell free if it passes at least this far from its edges
                                         # (a box is inset 20 mm, so a ray along the edge margin proves nothing)
SWEEP_PAIRS = 64
REVERSE_MARGIN = 8                       # stepping back to the cell just left must beat the best other step by this

SIN_BITS = 10                            # sin/cos are Q10 (1024 = 1.0)
ANGLE_STEPS = 1024                       # heading resolution used for sin/cos (0.35 degrees)


def cell_xy(cell):
    """Centre of a cell in mm."""
    row, col = divmod(cell, GRID)
    return -HALF_MM + CELL_MM // 2 + col * CELL_MM, HALF_MM - CELL_MM // 2 - row * CELL_MM


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def cell_of(x_mm, y_mm):
    """Cell under a pose, clamped to the arena (as the FPGA does)."""
    col = clamp(((x_mm + HALF_MM) * DIV150_MUL) >> DIV150_SHIFT if x_mm >= -HALF_MM else 0, 0, GRID - 1)
    row = clamp(((HALF_MM - y_mm) * DIV150_MUL) >> DIV150_SHIFT if y_mm <= HALF_MM else 0, 0, GRID - 1)
    return row * GRID + col


def cell_in_arena(x_mm, y_mm):
    """Cell under a world point, or None when the point is outside the arena."""
    if not (-HALF_MM <= x_mm < HALF_MM and -HALF_MM < y_mm <= HALF_MM):
        return None
    col = ((x_mm + HALF_MM) * DIV150_MUL) >> DIV150_SHIFT
    row = ((HALF_MM - y_mm) * DIV150_MUL) >> DIV150_SHIFT
    return row * GRID + col


QUARTER = [int(round(math.sin(2 * math.pi * i / ANGLE_STEPS) * (1 << SIN_BITS)))
           for i in range(ANGLE_STEPS // 4 + 1)]       # 0..90 degrees, 257 entries


def sin_q10(index):
    """sin of (index * 360/1024 degrees) in Q10, built from the quarter-wave ROM."""
    i = index & (ANGLE_STEPS - 1)
    half = ANGLE_STEPS // 2
    neg = i >= half
    i &= half - 1
    v = QUARTER[i] if i <= ANGLE_STEPS // 4 else QUARTER[half - i]
    return -v if neg else v


def cell_offsets(x_mm, y_mm):
    """(cell, rx, ry): the cell under a world point and the point's offset from the cell's
    west and north edges in mm, or None outside the arena."""
    cell = cell_in_arena(x_mm, y_mm)
    if cell is None:
        return None
    return (cell, (x_mm + HALF_MM) - cell % GRID * CELL_MM, (HALF_MM - y_mm) - cell // GRID * CELL_MM)


def hit_cell_clear_of_edges(x_mm, y_mm):
    """Cell of a hit point, or None if it is outside the arena or within
    HIT_EDGE_MM of a cell edge (racks and the perimeter wall sit on edges)."""
    cell = cell_in_arena(x_mm, y_mm)
    if cell is None:
        return None
    rx = (x_mm + HALF_MM) - cell % GRID * CELL_MM
    ry = (HALF_MM - y_mm) - cell // GRID * CELL_MM
    if HIT_EDGE_MM <= rx <= CELL_MM - HIT_EDGE_MM and HIT_EDGE_MM <= ry <= CELL_MM - HIT_EDGE_MM:
        return cell
    return None


def neighbors(cell):
    """Neighbours in the fixed tie order: up, down, left, right."""
    row, col = divmod(cell, GRID)
    out = []
    if row > 0:
        out.append(cell - GRID)
    if row < GRID - 1:
        out.append(cell + GRID)
    if col > 0:
        out.append(cell - 1)
    if col < GRID - 1:
        out.append(cell + 1)
    return out


def write_tables(repo):
    """Writes rtl/ff_tables.v: the rack map and the quarter-wave sine ROM as plain modules."""
    path = os.path.join(repo, 'rtl', 'ff_tables.v')
    with open(path, 'w') as f:
        f.write('// Generated by ref/warehouse.py. Do not edit.\n`timescale 1ns/1ps\n\n')
        f.write('// Racks: the only part of the map the FPGA knows ahead of time (bit = row*16 + col).\n')
        f.write('module ff_rack_map(output wire [255:0] mask);\n')
        f.write(f"    assign mask = {CELLS}'h{RACK_MASK:0{CELLS // 4}x};\n")
        f.write('endmodule\n\n')
        f.write('// Cart lanes: known traffic infrastructure. Obstacles seen there are forgotten quickly when not observed.\n')
        f.write('module ff_lane_map(output wire [255:0] mask);\n')
        f.write(f"    assign mask = {CELLS}'h{LANE_MASK:0{CELLS // 4}x};\n")
        f.write('endmodule\n\n')
        f.write('// sin(i * 360/1024 degrees) in Q10 for i = 0..256 (0..90 degrees).\n')
        f.write('module ff_sin_quarter(input wire [8:0] i, output reg [10:0] v);\n')
        f.write('    always @* case (i)\n')
        for i, q in enumerate(QUARTER):
            f.write(f"        9'd{i}: v = 11'd{q};\n")
        f.write("        default: v = 11'd0;\n    endcase\nendmodule\n")
    return path


if __name__ == '__main__':
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    write_tables(repo)
    print('rack cells:', bin(RACK_MASK).count('1'), 'goal:', GOAL)
