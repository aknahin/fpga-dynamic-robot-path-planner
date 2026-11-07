"""Scenario supervisor for the warehouse world.

Moves the two moving boxes along their lanes (the same ping-pong as
ref/sim_world.py), drops the late box, and can end a headless run. It does
no sensing and makes no driving decisions; those belong to the FPGA.

The scenario comes from the supervisor robot's customData (written by
webots/make_world.py). controllerArgs (all optional):
  --quit-at-goal    end the simulation when the robot has stopped on the delivery zone
  --max-time S      end the simulation after S seconds (default 300)
  --movie PATH      record the 3D view to an mp4 (Webots' own recorder)
"""

import argparse
import json
import math

from controller import Supervisor


def mover_pos(m, t):
    length = math.dist(m['a'], m['b'])
    s = (m['phase'] + t * m['speed']) % (2 * length)
    if s > length:
        s = 2 * length - s
    f = s / length
    return (m['a'][0] + (m['b'][0] - m['a'][0]) * f, m['a'][1] + (m['b'][1] - m['a'][1]) * f)


def box_string(cell_xy, side_mm, height, name):
    x, y, s = cell_xy[0] / 1000.0, cell_xy[1] / 1000.0, side_mm / 1000.0
    return (f'Solid {{ translation {x:.4f} {y:.4f} {height / 2:.4f} name "{name}" children [ '
            f'Shape {{ appearance PBRAppearance {{ baseColor 0.75 0.55 0.3 roughness 1 metalness 0 }} '
            f'geometry Box {{ size {s:.4f} {s:.4f} {height:.4f} }} }} ] '
            f'boundingObject Box {{ size {s:.4f} {s:.4f} {height:.4f} }} }}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--quit-at-goal', action='store_true')
    parser.add_argument('--max-time', type=float, default=300.0)
    parser.add_argument('--movie')
    args = parser.parse_args()

    sup = Supervisor()
    step = int(sup.getBasicTimeStep()) * 2
    scenario = json.loads(sup.getSelf().getField('customData').getSFString())
    root = sup.getRoot().getField('children')
    epuck = sup.getFromDef('EPUCK')
    movers = [(sup.getFromDef(f'MOVER{i}'), m) for i, m in enumerate(scenario['movers'])]
    late = scenario['late']      # dropped once, on the first candidate far enough from the robot
    late_done = False
    goal = scenario['goal']
    still_since = None
    last_pos = None
    recording = False

    def finish(code):
        if recording:
            sup.movieStopRecording()
            while not sup.movieIsReady():
                sup.step(0)
            print(f'[warehouse] movie saved to {args.movie}')
        sup.simulationQuit(code)

    while sup.step(step) != -1:
        t = sup.getTime()
        if args.movie and not recording:
            sup.movieStartRecording(args.movie, 1280, 720, 0, 90, 1, False)
            recording = True
        for node, m in movers:
            x, y = mover_pos(m, t)
            node.getField('translation').setSFVec3f(
                [x / 1000.0, y / 1000.0, scenario['box_height'] / 2])
            node.resetPhysics()
        if not late_done and t >= late['t']:
            rx, ry, _ = epuck.getPosition()      # same rule as ref/sim_world.pick_late_cell
            for item in late['candidates']:
                if math.hypot(item['xy'][0] - rx * 1000.0, item['xy'][1] - ry * 1000.0) >= late['min_dist_mm']:
                    root.importMFNodeFromString(-1, item['node'])
                    print(f'[warehouse] box dropped on cell {item["cell"]} at t={t:.2f} s')
                    late_done = True
                    break

        x, y, _ = epuck.getPosition()
        near_goal = math.hypot(x * 1000 - goal[0], y * 1000 - goal[1]) < 40
        moved = last_pos is None or math.hypot(x - last_pos[0], y - last_pos[1]) > 0.0005
        last_pos = (x, y)
        still_since = (t if still_since is None else still_since) if (near_goal and not moved) else None
        if args.quit_at_goal and still_since is not None and t - still_since > 1.0:
            print(f'[warehouse] robot stopped on the delivery zone at t={still_since:.2f} s')
            finish(0)
        if t >= args.max_time:
            print(f'[warehouse] time limit {args.max_time:.0f} s reached')
            finish(1)


main()
