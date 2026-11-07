"""Depth-only obstacle recognition from the four robot cameras (golden model).

Input per frame: 4 camera images of CAM_ROWS x CAM_COLS depth bytes (front,
left, back, right) and the robot pose. Output: the memory of what is where.

Per camera column the nearest of the rows is the range. The column's hit
point is turned into a world point with the pose, and the ray from the robot
to it is sampled in 16 equal steps. Cells the ray crosses are "seen free",
the cell at the end is "seen occupied". After all rays, each cell is updated
once, so the result does not depend on ray order:

  hit this frame     occupancy + 1 (max 3)
  only crossed       (through the central part of the cell, MISS_EDGE_MM from its edges)
                     occupancy - 1 (min 0); if it falls from >= 2 to < 2 the
                     thing that was there has gone, so the cell gets a
                     mover timer (MOVER_TTL frames, counts down)
  not observed       once per AGE_PERIOD frames, occupancy - 1 on cart-lane cells
                     (age_lane), or on every cell while the robot has no route at
                     all (age_all, a deadlock breaker). So the ghost of a cart that
                     left view cannot block a lane, but boxes elsewhere are never
                     forgotten while a route exists.
  racks              known map, never updated

A cell is blocked while occupancy >= 2. A box stays at 3, so it stays
blocked; a mover leaves a trail of timers behind it. The planner puts extra
cost around timer cells. This is how "static or moving" is told apart from
depth alone.
"""

from warehouse import (LANE_MASK, CAM_COLS, CAMERAS, CELLS, DEPTH_UNIT_MM, HIT_PENETRATION_MM,
                       MOVER_TTL, NO_HIT, OCC_BLOCK, OCC_MAX, RACK_MASK, RANGE_MM,
                       MISS_EDGE_MM, CELL_MM, cell_offsets, hit_cell_clear_of_edges, sin_q10)

SAMPLES = 16          # the ray is sampled at k/16 of its length
MISS_LAST = 13        # a hit ray only marks "free" up to sample 13


def trig(heading):
    """(sin, cos) of the heading in Q10 from the 1024-step quarter-wave table."""
    index = ((heading + 32) >> 6) & 1023
    return sin_q10(index), sin_q10(index + 256)


def column_ranges(image):
    """image: CAM_ROWS x CAM_COLS depth bytes -> CAM_COLS bytes (nearest row)."""
    return [min(int(image[r][c]) for r in range(len(image))) for c in range(CAM_COLS)]


def ray_vector(cam, col, byte):
    """Robot-frame vector (x forward, y left) in mm of one camera column, and
    whether it ended on an obstacle."""
    hit = byte < NO_HIT
    d = byte * DEPTH_UNIT_MM + HIT_PENETRATION_MM if hit else RANGE_MM
    lat = (d * (CAM_COLS - 1 - 2 * col)) >> 5     # left of the camera axis
    vx, vy = ((d, lat), (-lat, d), (-d, -lat), (lat, -d))[cam]
    return vx, vy, hit


def cast(images, x_mm, y_mm, heading):
    """Returns (hit_mask, miss_mask) for one frame."""
    s, c = trig(heading)
    hit_mask = miss_mask = 0
    for cam in range(CAMERAS):
        ranges = column_ranges(images[cam])
        for col in range(CAM_COLS):
            vx, vy, hit = ray_vector(cam, col, ranges[col])
            dx = (vx * c - vy * s) >> 10
            dy = (vx * s + vy * c) >> 10
            last_miss = MISS_LAST if hit else SAMPLES
            for k in range(last_miss + 1):
                off = cell_offsets(x_mm + ((dx * k) >> 4), y_mm + ((dy * k) >> 4))
                if off is not None and MISS_EDGE_MM <= off[1] <= CELL_MM - MISS_EDGE_MM \
                        and MISS_EDGE_MM <= off[2] <= CELL_MM - MISS_EDGE_MM:
                    miss_mask |= 1 << off[0]
            if hit:
                cell = hit_cell_clear_of_edges(x_mm + dx, y_mm + dy)
                if cell is not None:
                    hit_mask |= 1 << cell
    return hit_mask & ~RACK_MASK, miss_mask & ~RACK_MASK


class Memory:
    """Per-cell occupancy count and mover timer."""

    def __init__(self):
        self.occ = [0] * CELLS
        self.ttl = [0] * CELLS

    def update(self, hit_mask, miss_mask, age_all=False, age_lane=False):
        for c in range(CELLS):
            if self.ttl[c]:
                self.ttl[c] -= 1
            if (hit_mask >> c) & 1:
                self.occ[c] = min(self.occ[c] + 1, OCC_MAX)
            elif (miss_mask >> c) & 1:
                new = max(self.occ[c] - 1, 0)
                if self.occ[c] >= OCC_BLOCK and new < OCC_BLOCK:
                    self.ttl[c] = MOVER_TTL
                self.occ[c] = new
            elif (age_all or (age_lane and (LANE_MASK >> c) & 1)) and self.occ[c] > 0:
                self.occ[c] -= 1     # not observed for a while: fade

    def blocked_mask(self):
        mask = 0
        for c in range(CELLS):
            if self.occ[c] >= OCC_BLOCK:
                mask |= 1 << c
        return mask

    def mover_mask(self):
        mask = 0
        for c in range(CELLS):
            if self.ttl[c]:
                mask |= 1 << c
        return mask
