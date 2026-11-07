"""Golden reference for the FPGA brain (Phase 2).

Input each frame: the 64x64 depth image (uint8, see hil_format.encode_depth)
and the robot pose (x_mm, y_mm, heading16). Output: left/right wheel speeds
(int8, WHEEL_UNIT rad/s per step) plus the grid and route for the viewer.

Everything here uses integers, shifts and comparisons only, so the Verilog
can match it bit for bit. bfs() follows rtl/bfs_engine.v exactly (same
neighbour order: up, down, left, right), so the routes are identical.
"""

GRID = 8
GOAL = 63
CELL_MM = 150
HALF_MM = 600

# Obstacle detection: a pixel is "high" when its depth byte is below
# DEPTH_HIGH (2.86 m, floor reads 150, wall tops 137/138, e-puck ~147).
# A cell is blocked when at least CELL_BLOCKED of its 64 pixels are high.
DEPTH_HIGH = 143
CELL_BLOCKED = 32

# Motion control, in heading16 units (65536 = 360 degrees) and mm.
ARRIVE_MM = 15         # waypoint reached when this close along the travel axis
TURN_LIMIT = 2731      # turn in place while heading error is above ~15 degrees
LAT_SHIFT = 6          # steer back to the lane centre: lat_mm * 64
LAT_LIMIT = 4096       # ... clamped to ~22.5 degrees
STEER_SHIFT = 6        # forward steering: error >> 6 wheel steps
DRIVE_SPEED = 100      # 5 rad/s, about 0.1 m/s on the e-puck
TURN_SPEED = 40        # 2 rad/s
MAX_SPEED = 125

# Travel direction from one cell to the next: (dx, dy, heading16).
DIRS = {1: (1, 0, 0), -GRID: (0, 1, 16384), -1: (-1, 0, 32768), GRID: (0, -1, 49152)}

WAIT_MAP, RUNNING, AT_GOAL, NO_ROUTE = 'wait_map', 'running', 'at_goal', 'no_route'


def detect_band(pixels):
    """One band (8 image rows x 64 px) -> 8 bits, bit c = cell column c.

    This is what the FPGA does while a C1 packet streams in: one counter
    per cell column, then a compare at the end of the packet.
    """
    bits = 0
    for col in range(GRID):
        if int((pixels[:, col * 8:col * 8 + 8] < DEPTH_HIGH).sum()) >= CELL_BLOCKED:
            bits |= 1 << col
    return bits


def detect_grid(depth_u8):
    """64x64 uint8 depth -> 64-bit obstacle mask (bit n = cell n)."""
    mask = 0
    for band in range(GRID):
        mask |= detect_band(depth_u8[band * 8:band * 8 + 8]) << (band * GRID)
    return mask


def neighbor(cell, direction):
    """Same as rtl/neighbor_check.v: 0 up, 1 down, 2 left, 3 right."""
    row, col = cell >> 3, cell & 7
    if direction == 0 and row != 0:
        return cell - 8
    if direction == 1 and row != 7:
        return cell + 8
    if direction == 2 and col != 0:
        return cell - 1
    if direction == 3 and col != 7:
        return cell + 1
    return None


def bfs(start, obstacles):
    """Mirror of rtl/bfs_engine.v. Returns (found, path start..goal, explored)."""
    visited = {start}
    parent = {}
    queue = [start]
    head = 0
    explored = 0
    while head < len(queue):
        cell = queue[head]
        head += 1
        explored += 1
        if cell == GOAL:
            path = [GOAL]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return True, path[::-1], explored
        for d in range(4):
            n = neighbor(cell, d)
            if n is not None and not (obstacles >> n) & 1 and n not in visited:
                visited.add(n)
                parent[n] = cell
                queue.append(n)
    return False, [], explored


def cell_of(x_mm, y_mm):
    col = min(max((x_mm + HALF_MM) // CELL_MM, 0), GRID - 1)
    row = min(max((HALF_MM - y_mm) // CELL_MM, 0), GRID - 1)
    return row * GRID + col


def center_mm(cell):
    row, col = divmod(cell, GRID)
    return -HALF_MM + CELL_MM // 2 + col * CELL_MM, HALF_MM - CELL_MM // 2 - row * CELL_MM


def wrap16(a):
    """Signed difference of two heading16 values, in -32768..32767."""
    return ((a + 32768) & 0xFFFF) - 32768


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class Brain:
    def __init__(self):
        self.reset()
        self.user_walls = 0   # walls clicked in the viewer (5A); kept over resets

    def reset(self):
        """C0 packet: forget the route and wait for the next frame."""
        self.status = WAIT_MAP
        self.obstacles = 0
        self.path = []        # route cells, robot's cell first, goal last
        self.index = 0        # path[index] is the waypoint being driven to
        self.robot_cell = 0
        self.explored = 0
        self.replans = 0
        self.events = []      # (kind, details) since the last step, for logs

    def edit(self, add, cell):
        """5A packet. Same rule as Phase 1: start, goal and robot cell ignored."""
        if cell in (0, GOAL, self.robot_cell):
            return
        if add:
            self.user_walls |= 1 << cell
        else:
            self.user_walls &= ~(1 << cell)

    def target_cell(self):
        return self.path[self.index] if self.status == RUNNING else self.robot_cell

    def route_mask(self):
        """Route still to drive, from the cell just left to the goal (for B5)."""
        mask = 0
        for cell in self.path[max(self.index - 1, 0):]:
            mask |= 1 << cell
        return mask

    def remaining_mask(self):
        mask = 0
        for cell in self.path[self.index:]:
            mask |= 1 << cell
        return mask

    def plan(self):
        found, path, self.explored = bfs(self.robot_cell, self.obstacles)
        self.replans += 1
        if found and len(path) > 1:
            self.path, self.index, self.status = path, 1, RUNNING
        elif found:
            self.path, self.index, self.status = path, 1, AT_GOAL
        else:
            self.path, self.index, self.status = [], 0, NO_ROUTE
        self.events.append(('plan', self.robot_cell, found, list(self.path)))

    def step(self, depth_u8, x_mm, y_mm, heading):
        """One frame. Returns (left, right) wheel speeds."""
        return self.step_grid(detect_grid(depth_u8), x_mm, y_mm, heading)

    def step_grid(self, camera_grid, x_mm, y_mm, heading):
        """Same as step(), with the camera grid already detected (band by
        band, as the FPGA does while the C1 packets stream in)."""
        self.events = []
        self.robot_cell = cell_of(x_mm, y_mm)
        seen = camera_grid | self.user_walls
        # The robot's own cell and the goal can never be walls for planning.
        seen &= ~((1 << self.robot_cell) | (1 << GOAL))
        new_walls = seen & ~self.obstacles
        changed = seen != self.obstacles
        self.obstacles = seen

        if self.status == WAIT_MAP:
            self.plan()
        elif self.status == RUNNING and new_walls & self.remaining_mask():
            self.events.append(('blocked', new_walls & self.remaining_mask()))
            self.plan()
        elif self.status == NO_ROUTE and changed:
            self.plan()

        if self.status != RUNNING:
            return 0, 0
        return self.drive(x_mm, y_mm, heading)

    def drive(self, x_mm, y_mm, heading):
        target = self.path[self.index]
        dx, dy, base = DIRS[target - self.path[self.index - 1]]
        cx, cy = center_mm(target)
        along = (cx - x_mm) * dx + (cy - y_mm) * dy
        if along <= ARRIVE_MM:
            self.index += 1
            self.events.append(('reached', target))
            if self.index == len(self.path):
                self.status = AT_GOAL
                return 0, 0
            return self.drive(x_mm, y_mm, heading)

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
