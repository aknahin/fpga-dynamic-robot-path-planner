"""Cost field and flow field (golden model for rtl/flow_engine.v).

The idea comes from the PCB router (pcb-ff3d-routing/pcb-3d-ff.cpp):
compute_integration_field_3d is a reverse Dijkstra from the goal and
compute_flow_field_3d points every cell at its cheapest neighbour. Here it
is 2D, one goal, integers only.

  weight(c)  cost of stepping onto c: 1 on free floor, plus HALO_COSTS
             1, 2 or 3 cells from a cell with recent motion, INF if blocked.
  field(c)   cost to reach the goal from c: weight(c) + min over neighbours.
  flow(c)    the neighbour with the lowest field value (tie order: up, down,
             left, right, as in rtl/neighbor_check.v).

The field is built by raster sweeps over the whole grid, forward then
backward, repeated until nothing changes (at most SWEEP_PAIRS pairs). That is
exactly the order the hardware uses. When it converges the result equals a
plain Dijkstra; test_flowfield.py checks that.
"""

from warehouse import (CELLS, GOAL, GRID, HALO_COSTS, INF, REVERSE_MARGIN, SWEEP_PAIRS,
                       neighbors)

NEIGHBORS = [neighbors(c) for c in range(CELLS)]


_COL0 = sum(1 << (r * GRID) for r in range(GRID))            # column 0 of every row
_COL15 = _COL0 << (GRID - 1)
_ALL = (1 << CELLS) - 1


def dilate(mask):
    """mask plus its 4-neighbours (what the RTL does with shifts and ORs)."""
    return (mask | ((mask << 1) & ~_COL0) | ((mask >> 1) & ~_COL15)
            | (mask << GRID) | (mask >> GRID)) & _ALL


def weights(blocked, mover):
    """blocked, mover: CELLS-bit masks. Returns a list of per-cell weights."""
    rings = []
    m = mover
    for _ in HALO_COSTS:
        m = dilate(m)
        rings.append(m)
    w = [0] * CELLS
    for c in range(CELLS):
        if (blocked >> c) & 1:
            w[c] = INF
            continue
        extra = 0
        for ring, cost in zip(rings, HALO_COSTS):
            if (ring >> c) & 1:
                extra = cost
                break
        w[c] = 1 + extra
    return w


def cost_field(w, goal=GOAL):
    """Returns (field, sweeps). The goal is always reachable, even if marked blocked."""
    field = [INF] * CELLS
    field[goal] = 0
    sweeps = 0
    for _ in range(SWEEP_PAIRS):
        changed = False
        for order in (range(CELLS), range(CELLS - 1, -1, -1)):
            sweeps += 1
            for c in order:
                if c == goal or w[c] >= INF:
                    continue
                best = min(field[n] for n in NEIGHBORS[c])
                value = min(INF, w[c] + best)
                if value < field[c]:
                    field[c] = value
                    changed = True
        if not changed:
            break
    return field, sweeps


def flow_step(field, cell, avoid=None):
    """Next cell from `cell`: the lowest-cost neighbour, or None if none is reachable.

    `avoid` is the cell the robot just came from. Stepping back to it is only
    allowed when it is cheaper than the best other step by REVERSE_MARGIN (or
    when there is no other way). Without this, two routes of nearly equal cost
    whose costs swap as obstacles move make the robot turn back and forth.
    """
    best, best_cost = None, INF
    for n in NEIGHBORS[cell]:
        if n != avoid and field[n] < best_cost:
            best, best_cost = n, field[n]
    if avoid is not None and field[avoid] < INF and field[avoid] + REVERSE_MARGIN < best_cost:
        return avoid
    return best
