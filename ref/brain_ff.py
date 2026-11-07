"""Golden reference for the flow-field brain (what rtl/ff_brain.v must match).

Each frame: four depth images + pose in, left/right wheel speeds out.

  1. perception: update the occupancy / mover memory from the cameras
  2. cost field from the goal over the known racks, seen obstacles, user walls
  3. flow: the robot drives to the next cell given by the flow field and
     picks the next one on arrival; if the cell it is heading to becomes
     blocked it picks again from where it is (counted in `replans`)
  4. the same waypoint controller as ref/brain.py steers to the next cell

Integers only; the motor constants are imported from brain.py unchanged.
"""

from brain import (ARRIVE_MM, DRIVE_SPEED, LAT_LIMIT, LAT_SHIFT, MAX_SPEED, STEER_SHIFT,
                   TURN_LIMIT, TURN_SPEED, clamp, wrap16)
from flowfield import cost_field, flow_step, weights
from perception import Memory, cast
from warehouse import AGE_PERIOD, CELLS, GOAL, GRID, INF, RACK_MASK, cell_of, cell_xy

WAIT_MAP, RUNNING, AT_GOAL, NO_ROUTE = 'wait_map', 'running', 'at_goal', 'no_route'

# Travel direction from one cell to the next: (dx, dy, heading16).
DIRS = {1: (1, 0, 0), -GRID: (0, 1, 16384), -1: (-1, 0, 32768), GRID: (0, -1, 49152)}


class BrainFF:
    def __init__(self):
        self.user_walls = 0   # clicked in the viewer (5A); kept over resets
        self.reset()

    def reset(self):
        self.status = WAIT_MAP
        self.memory = Memory()
        self.robot_cell = 0
        self.prev = 0
        self.target = 0
        self.replans = 0
        self.no_route_frames = 0
        self.frames = 0
        self.field = [INF] * CELLS
        self.events = []
        self._weights_key = None

    def edit(self, add, cell):
        if cell in (GOAL, self.robot_cell) or (RACK_MASK >> cell) & 1:
            return
        if add:
            self.user_walls |= 1 << cell
        else:
            self.user_walls &= ~(1 << cell)

    # Masks the viewer and the reply show (racks are drawn from the known map).
    def blocked_mask(self):
        return self.memory.blocked_mask() | self.user_walls

    def mover_mask(self):
        return self.memory.mover_mask()

    def target_cell(self):
        return self.target if self.status == RUNNING else self.robot_cell

    def step(self, images, x_mm, y_mm, heading):
        """One frame. Returns (left, right) wheel speeds."""
        self.events = []
        self.robot_cell = cell_of(x_mm, y_mm)
        hit, miss = cast(images, x_mm, y_mm, heading)
        # Deadlock breaker: only while there is no route, unobserved cells fade once per AGE_PERIOD frames.
        self.no_route_frames = self.no_route_frames + 1 if self.status == NO_ROUTE else 0
        age_all = self.status == NO_ROUTE and self.no_route_frames % AGE_PERIOD == 0
        self.frames += 1
        age_lane = self.frames % AGE_PERIOD == 0
        self.memory.update(hit, miss, age_all, age_lane)
        return self.step_masks(x_mm, y_mm, heading)

    def step_masks(self, x_mm, y_mm, heading):
        """Everything after perception (the memory is already up to date)."""
        blocked = (self.blocked_mask() | RACK_MASK) & ~((1 << self.robot_cell) | (1 << GOAL))
        mover = self.mover_mask()
        key = (blocked, mover)
        if key != self._weights_key:     # same inputs give the same field; skip the sweeps
            self._weights_key = key
            self.field, _ = cost_field(weights(blocked, mover))

        if self.status in (WAIT_MAP, NO_ROUTE):
            self.pick(self.robot_cell)
        elif self.status == RUNNING and self.field[self.target] >= INF:
            self.events.append(('blocked', self.target))
            self.pick(self.robot_cell)
        if self.status != RUNNING:
            return 0, 0
        return self.drive(x_mm, y_mm, heading)

    def pick(self, cell):
        """Start from `cell`: follow the flow one step (a new plan)."""
        self.replans += 1
        if cell == GOAL:
            self.status, self.prev, self.target = AT_GOAL, cell, cell
            return
        nxt = flow_step(self.field, cell)
        if nxt is None:
            self.status = NO_ROUTE
        else:
            self.status, self.prev, self.target = RUNNING, cell, nxt
        self.events.append(('pick', cell, nxt))

    def drive(self, x_mm, y_mm, heading):
        while True:
            dx, dy, base = DIRS[self.target - self.prev]
            cx, cy = cell_xy(self.target)
            along = (cx - x_mm) * dx + (cy - y_mm) * dy
            if along > ARRIVE_MM:
                break
            self.events.append(('reached', self.target))
            if self.target == GOAL:
                self.status = AT_GOAL
                return 0, 0
            nxt = flow_step(self.field, self.target, avoid=self.prev)
            self.prev = self.target
            if nxt is None:
                self.status = NO_ROUTE
                return 0, 0
            self.target = nxt

        # lat > 0: robot is left of the lane centre, so aim a little right.
        lat = (x_mm - cx) * -dy + (y_mm - cy) * dx
        aim = (base - clamp(lat << LAT_SHIFT, -LAT_LIMIT, LAT_LIMIT)) & 0xFFFF
        err = wrap16(aim - heading)
        if err > TURN_LIMIT:
            return -TURN_SPEED, TURN_SPEED
        if err < -TURN_LIMIT:
            return TURN_SPEED, -TURN_SPEED
        steer = err >> STEER_SHIFT
        return (clamp(DRIVE_SPEED - steer, -MAX_SPEED, MAX_SPEED),
                clamp(DRIVE_SPEED + steer, -MAX_SPEED, MAX_SPEED))
