"""Grid geometry and sensor data formats shared by Webots, ref/ and bridge/.

8x8 cells, 0.15 m each, arena centred on the origin. Cell = row*8 + col.
Row 0 is the north edge (+y), column 0 the west edge (-x), so cell 0 is the
start corner and cell 63 the goal corner, as on the FPGA.

The encode_* functions are the sensor output format: what a real depth
camera and odometry unit would hand to the FPGA. They only change units and
rounding; they make no decisions.
"""

import math

import numpy as np

GRID = 8
CELL_SIZE = 0.15
HALF = GRID * CELL_SIZE / 2
CELL_MM = 150
HALF_MM = 600
GOAL = 63

CAMERA_HEIGHT = 3.0   # overhead depth camera above the floor, metres
IMAGE_SIZE = 64       # depth image is IMAGE_SIZE x IMAGE_SIZE
WALL_HEIGHT = 0.25    # walls and obstacles; the e-puck is only ~0.05 m tall

DEPTH_UNIT = 0.02     # one depth byte step is 2 cm, so the floor (3 m) reads 150
WHEEL_UNIT = 0.05     # one wheel-speed step is 0.05 rad/s (int8, +-125 max)


def cell_center(cell):
    row, col = divmod(cell, GRID)
    return (-HALF + (col + 0.5) * CELL_SIZE, HALF - (row + 0.5) * CELL_SIZE)


def cell_at(x, y):
    """Cell under world point (x, y) in metres, or None outside the arena."""
    col = int((x + HALF) // CELL_SIZE)
    row = int((HALF - y) // CELL_SIZE)
    if 0 <= row < GRID and 0 <= col < GRID:
        return row * GRID + col
    return None


def encode_depth(depth):
    """64x64 float depth in metres -> 64x64 uint8, 2 cm per step, 255 = far."""
    return np.clip(np.rint(np.asarray(depth) / DEPTH_UNIT), 0, 255).astype(np.uint8)


def encode_pose(x, y, heading_deg):
    """Pose -> (x_mm, y_mm, heading) as int16, int16, uint16.

    heading is a 16-bit binary angle, counter-clockwise from east:
    0 = east, 16384 = north, 32768 = west, 49152 = south.
    """
    h = int(round(heading_deg / 360.0 * 65536)) & 0xFFFF
    return int(round(x * 1000)), int(round(y * 1000)), h


def heading_deg_from_compass(north):
    """Webots Compass vector (robot frame) -> heading in degrees, 0 = east."""
    return math.degrees(math.atan2(north[0], north[1]))
