"""Small stand-in for Webots, used to test the brain on many seeds quickly.

It builds the warehouse (racks, random boxes, a box that appears mid-run,
moving boxes), ray-casts the four robot cameras the way a planar-depth
RangeFinder reports them, and drives an e-puck-like differential robot.
It makes no decisions; it only produces sensor data and applies wheel speeds.

Webots remains the reference simulator for the demo; this is for fast
regression runs and for generating test vectors for the RTL.
"""

import math
import random

from warehouse import (CAM_COLS, CAM_ROWS, CAMERAS, CELL_MM, CELLS, DEPTH_UNIT_MM, GOAL, GRID,
                       HALF_MM, LAYOUT, MOVER_LANES, NO_HIT, RACK_MASK, RANGE_MM, cell_xy, neighbors)

DT = 0.032                 # one frame, seconds
WHEEL_R = 0.0205
AXLE = 0.052
WHEEL_UNIT = 0.05
ROBOT_R_MM = 37.0
BOX_HALF_MM = 55.0         # boxes and movers are 110 mm cubes
RACK_HALF_MM = CELL_MM / 2.0


LATE_MIN_DIST_MM = 800.0


def pick_late_cell(candidates, x_mm, y_mm):
    """The cell the late box is dropped on: the first candidate at least LATE_MIN_DIST_MM from
    the robot (None means wait a frame). The same rule runs in the Webots supervisor."""
    for cell in candidates:
        cx, cy = cell_xy(cell)
        if math.hypot(cx - x_mm, cy - y_mm) >= LATE_MIN_DIST_MM:
            return cell
    return None


def late_drop_from_poses(world, t, poses):
    """Which cell the late box was dropped on and when, for a recorded run (t seconds, poses in mm)."""
    for ti, p in zip(t, poses):
        if ti >= world.appear_at:
            cell = pick_late_cell(world.late_candidates, p[0], p[1])
            if cell is not None:
                return cell, float(ti)
    return None


class Mover:
    """A box that ping-pongs between two cells at a constant speed."""

    def __init__(self, a, b, speed_mm_s, phase=0.0):
        self.a, self.b = cell_xy(a), cell_xy(b)
        self.length = math.dist(self.a, self.b)
        self.speed = speed_mm_s
        self.phase = phase

    def pos(self, t):
        s = (self.phase + t * self.speed) % (2 * self.length)
        if s > self.length:
            s = 2 * self.length - s
        f = s / self.length
        return (self.a[0] + (self.b[0] - self.a[0]) * f, self.a[1] + (self.b[1] - self.a[1]) * f)


def free_connected(blocked_mask):
    """True when every non-blocked cell can reach the goal (4-connected)."""
    seen = {GOAL}
    stack = [GOAL]
    while stack:
        c = stack.pop()
        for n in neighbors(c):
            if n not in seen and not (blocked_mask >> n) & 1:
                seen.add(n)
                stack.append(n)
    free = CELLS - bin(blocked_mask).count('1')
    return len(seen) == free


class World:
    def __init__(self, seed=0, n_boxes=6, with_movers=True, appear_at=8.0, mover_speed=45.0):
        rng = random.Random(seed)
        self.rng = rng
        free = [c for c in range(CELLS) if not (RACK_MASK >> c) & 1 and c != GOAL]
        # Movers ping-pong along the lanes in warehouse.MOVER_LANES; the robot never starts in a lane.
        lane_cells = set()
        for a, b in MOVER_LANES:
            (ra, ca), (rb, cb) = divmod(a, GRID), divmod(b, GRID)
            lane_cells |= {r * GRID + c for r in range(min(ra, rb), max(ra, rb) + 1)
                           for c in range(min(ca, cb), max(ca, cb) + 1)}
        far = [c for c in free if math.dist(cell_xy(c), cell_xy(GOAL)) > 1300
               and (c not in lane_cells or not with_movers)]
        self.start_cell = rng.choice(far)
        taboo = {self.start_cell, GOAL} | set(neighbors(self.start_cell)) | set(neighbors(GOAL))
        # Lanes of the movers must stay clear of static boxes.
        self.movers = []
        if with_movers:
            self.movers = [Mover(a, b, mover_speed, rng.uniform(0, 1500)) for a, b in MOVER_LANES]
        choices = [c for c in free if c not in taboo and (c not in lane_cells or not with_movers)]
        blocked = RACK_MASK
        self.boxes = []          # static boxes: (cell, 0.0)
        tries = 0
        while len(self.boxes) < n_boxes and tries < 500:
            tries += 1
            c = rng.choice(choices)
            if any(c == b[0] for b in self.boxes):
                continue
            if not free_connected(blocked | (1 << c)):
                continue
            blocked |= 1 << c
            self.boxes.append((c, 0.0))
        self.static_mask = blocked
        # One more box is dropped at appear_at, never closer than LATE_MIN_DIST_MM to the robot:
        # the first of these candidates that is far enough (each keeps the goal reachable).
        self.appear_at = appear_at
        self.late_candidates = []
        tries = 0
        while len(self.late_candidates) < 8 and tries < 800:
            tries += 1
            c = rng.choice(choices)
            if c in self.late_candidates or any(c == b[0] for b in self.boxes):
                continue
            if free_connected(blocked | (1 << c)):
                self.late_candidates.append(c)
        self.late = None         # (cell, time) once dropped
        x, y = cell_xy(self.start_cell)
        self.x, self.y = float(x), float(y)
        self.heading = rng.uniform(0, 2 * math.pi)
        self.t = 0.0
        self.collisions = 0
        self.min_clear_mm = 1e9

    # ---- geometry ---------------------------------------------------------

    def squares(self):
        """All obstacles as (cx, cy, half) squares at the current time."""
        sq = []
        for c in range(CELLS):
            if (RACK_MASK >> c) & 1:
                sq.append((*cell_xy(c), RACK_HALF_MM))
        for cell, t_on in self.boxes:
            if self.t >= t_on:
                sq.append((*cell_xy(cell), BOX_HALF_MM))
        if self.late is not None:
            sq.append((*cell_xy(self.late[0]), BOX_HALF_MM))
        for m in self.movers:
            sq.append((*m.pos(self.t), BOX_HALF_MM))
        return sq

    @staticmethod
    def _hit_square(ox, oy, ux, uy, sq):
        cx, cy, h = sq
        tmin, tmax = 0.0, 1e9
        for o, u, lo, hi in ((ox, ux, cx - h, cx + h), (oy, uy, cy - h, cy + h)):
            if abs(u) < 1e-12:
                if o < lo or o > hi:
                    return None
            else:
                t1, t2 = (lo - o) / u, (hi - o) / u
                if t1 > t2:
                    t1, t2 = t2, t1
                tmin, tmax = max(tmin, t1), min(tmax, t2)
                if tmin > tmax:
                    return None
        return tmin

    def depth_images(self):
        """4 cameras of CAM_ROWS x CAM_COLS depth bytes (planar depth / 6 mm)."""
        sq = self.squares()
        images = []
        for cam in range(CAMERAS):
            yaw = self.heading + cam * math.pi / 2
            fx, fy = math.cos(yaw), math.sin(yaw)       # camera forward
            lx, ly = -math.sin(yaw), math.cos(yaw)      # camera left
            row = []
            for col in range(CAM_COLS):
                s = (CAM_COLS - 1 - 2 * col) / CAM_COLS
                ux, uy = fx + s * lx, fy + s * ly       # axial component is exactly 1
                best = None
                for q in sq:
                    t = self._hit_square(self.x, self.y, ux, uy, q)
                    if t is not None and (best is None or t < best):
                        best = t
                # Arena wall: the first boundary crossed.
                for o, u in ((self.x, ux), (self.y, uy)):
                    if u > 1e-12:
                        t = (HALF_MM + 1 - o) / u
                    elif u < -1e-12:
                        t = (-HALF_MM - 1 - o) / u
                    else:
                        continue
                    if best is None or t < best:
                        best = t
                if best is None or best > RANGE_MM:
                    byte = 255
                else:
                    byte = min(254, int(round(best / DEPTH_UNIT_MM)))
                row.append(byte)
            images.append([row[:] for _ in range(CAM_ROWS)])
        return images

    def pose(self):
        """(x_mm, y_mm, heading16) as the bridge reports them."""
        h = int(round(math.degrees(self.heading) / 360.0 * 65536)) & 0xFFFF
        return int(round(self.x)), int(round(self.y)), h

    # ---- dynamics ---------------------------------------------------------

    def step(self, wheels):
        wl, wr = wheels[0] * WHEEL_UNIT, wheels[1] * WHEEL_UNIT
        v = WHEEL_R * (wl + wr) / 2.0
        w = WHEEL_R * (wr - wl) / AXLE
        self.x += v * math.cos(self.heading) * DT * 1000.0
        self.y += v * math.sin(self.heading) * DT * 1000.0
        self.heading = (self.heading + w * DT) % (2 * math.pi)
        self.t += DT
        if self.late is None and self.t >= self.appear_at:
            cell = pick_late_cell(self.late_candidates, self.x, self.y)
            if cell is not None:
                self.late = (cell, self.t)
        for cx, cy, h in self.squares():
            dx = max(abs(self.x - cx) - h, 0.0)
            dy = max(abs(self.y - cy) - h, 0.0)
            clear = math.hypot(dx, dy) - ROBOT_R_MM
            self.min_clear_mm = min(self.min_clear_mm, clear)
            if clear < 0:
                self.collisions += 1
