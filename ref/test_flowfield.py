"""Tests for the flow-field brain. Run: python3 ref/test_flowfield.py [--runs N]

  1. the cost field equals an independent heapq Dijkstra on random maps
  2. following the flow from every reachable cell reaches the goal
  3. perception: a static box stays blocked, a mover is cleared and flagged,
     racks are ignored
  4. whole-robot runs in sim_world from random seeds
"""

import argparse
import heapq
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

from brain_ff import AT_GOAL, BrainFF  # noqa: E402
from flowfield import cost_field, flow_step, weights  # noqa: E402
from perception import Memory, cast  # noqa: E402
from sim_world import DT, World  # noqa: E402
from warehouse import CAM_COLS, CAM_ROWS, CELLS, GOAL, INF, RACK_MASK, cell_xy, neighbors  # noqa: E402


def dijkstra(w, goal=GOAL):
    dist = [INF] * CELLS
    dist[goal] = 0
    heap = [(0, goal)]
    while heap:
        d, c = heapq.heappop(heap)
        if d > dist[c]:
            continue
        for n in neighbors(c):
            if w[n] >= INF:
                continue
            nd = min(INF, d + w[n])
            if nd < dist[n]:
                dist[n] = nd
                heapq.heappush(heap, (nd, n))
    return dist


def test_field_vs_dijkstra():
    rng = random.Random(1)
    for trial in range(60):
        density = rng.choice([0.05, 0.15, 0.3])
        blocked = RACK_MASK
        mover = 0
        for c in range(CELLS):
            if c != GOAL and rng.random() < density:
                blocked |= 1 << c
            if rng.random() < 0.03:
                mover |= 1 << c
        blocked &= ~(1 << GOAL)
        w = weights(blocked, mover)
        field, sweeps = cost_field(w)
        assert field == dijkstra(w), f'trial {trial}: field differs from Dijkstra'
    print('ok  field == Dijkstra on 60 random maps')


def test_flow_reaches_goal():
    rng = random.Random(2)
    for trial in range(30):
        blocked = RACK_MASK
        for c in range(CELLS):
            if c != GOAL and rng.random() < 0.1:
                blocked |= 1 << c
        field, _ = cost_field(weights(blocked, 0))
        for start in range(CELLS):
            if field[start] >= INF or start == GOAL:
                continue
            c, steps = start, 0
            while c != GOAL:
                n = flow_step(field, c)
                assert n is not None and field[n] < field[c], f'flow stuck at {c}'
                c, steps = n, steps + 1
                assert steps < CELLS
    print('ok  flow from every reachable cell reaches the goal (30 maps)')


def blank_images():
    return [[[255] * CAM_COLS for _ in range(CAM_ROWS)] for _ in range(4)]


def test_perception():
    from sim_world import BOX_HALF_MM
    from warehouse import cell_in_arena
    w = World(seed=3, n_boxes=0, with_movers=False)
    box_cell = cell_in_arena(375, 75)       # free cell three cells east of the robot, along row 7
    bx, by = cell_xy(box_cell)
    racks = [(*cell_xy(c), 75.0) for c in range(CELLS) if (RACK_MASK >> c) & 1]

    def frame(box_present):
        w.x, w.y, w.heading = -75.0, 75.0, 0.0
        w.squares = lambda: racks + ([(bx, by, BOX_HALF_MM)] if box_present else [])
        return w.depth_images()

    pose = (-75, 75, 0)
    mem = Memory()
    for _ in range(4):
        mem.update(*cast(frame(True), *pose))
    assert (mem.blocked_mask() >> box_cell) & 1, 'static box not detected'
    assert not (mem.mover_mask() >> box_cell) & 1
    assert not (mem.blocked_mask() & RACK_MASK), 'racks must not be recorded'
    for _ in range(10):
        mem.update(*cast(frame(True), *pose))
    assert (mem.blocked_mask() >> box_cell) & 1 and mem.occ[box_cell] == 3, 'static box lost'
    for _ in range(3):                       # the box leaves
        mem.update(*cast(frame(False), *pose))
    assert not (mem.blocked_mask() >> box_cell) & 1, 'box not cleared after it left'
    assert mem.ttl[box_cell] > 0, 'cleared box not flagged as mover'
    for _ in range(70):
        mem.update(*cast(frame(False), *pose))
    assert mem.ttl[box_cell] == 0, 'mover timer did not expire'
    print('ok  perception: static box kept, mover cleared and flagged, racks ignored')


def run_robot(seed, max_s=300.0, verbose=False):
    world = World(seed=seed)
    brain = BrainFF()
    frames = 0
    last_replans = 0
    while world.t < max_s:
        wheels = brain.step(world.depth_images(), *world.pose())
        if verbose and brain.replans != last_replans:
            print(f'  t={world.t:6.1f} replan #{brain.replans} at cell {brain.robot_cell}')
            last_replans = brain.replans
        if brain.status == AT_GOAL:
            break
        world.step(wheels)
        frames += 1
    return dict(seed=seed, ok=brain.status == AT_GOAL, t=world.t, replans=brain.replans,
                collisions=world.collisions, clear=world.min_clear_mm, status=brain.status,
                start=world.start_cell)


def test_robot_runs(seeds):
    bad = 0
    for seed in seeds:
        t0 = time.time()
        r = run_robot(seed)
        flag = 'ok ' if r['ok'] and r['collisions'] == 0 else 'BAD'
        bad += flag == 'BAD'
        print(f"{flag} seed {seed:3d} start {r['start']:3d} {r['status']:9s} t={r['t']:6.1f}s "
              f"replans={r['replans']:3d} collisions={r['collisions']} "
              f"min clearance={r['clear']:6.1f} mm  ({time.time() - t0:4.1f}s wall)")
    return bad


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--first', type=int, default=0)
    ap.add_argument('--quick', action='store_true', help='unit tests only')
    args = ap.parse_args()
    test_field_vs_dijkstra()
    test_flow_reaches_goal()
    test_perception()
    if not args.quick:
        failures = test_robot_runs(range(args.first, args.first + args.runs))
        sys.exit(1 if failures else 0)
