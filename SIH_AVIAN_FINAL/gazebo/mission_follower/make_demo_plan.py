#!/usr/bin/env python3
"""SIH_AVIAN_FINAL -- cut a short, loopable demo plan out of the v5 coverage plan.

OFFLINE MISSION PLANNING ONLY (same rule as plan_gazebo_mission.py): this is the
one step that may read the scene geometry. It writes mission/gazebo_demo_plan.json,
which the runtime follower reads together with its own sensors -- nothing else.

Why a cut instead of the first N waypoints: the live detector fires only near a
handful of the 1,162 waypoints (full_pass_05: 87 detections, all in 8 waypoints,
47 of them at V5_1015). The first 40 waypoints produced 0 detections in two demo
attempts, so a demo of them shows a camera window with no boxes. This cut is the
contiguous stretch V5_1005..V5_1027 (metro pier 6/7 area), which holds V5_1009,
V5_1015 and V5_1024 -- chosen from that logged detector output, i.e. it is a
choice of WHERE to demo, not a change to the detector or its threshold.

The plan keeps the v5 waypoints, ids, headings, gimbal pitches and inter-waypoint
routes untouched. Only two routes are new, planned with the project's own planner
(plan_gazebo_mission.route: straight / L / A* on the 1 m grid at 3.5 m clearance):
  * home -> first cut waypoint   (the v5 route_in[0] started from the previous one)
  * last cut waypoint -> home    (plan['rth'])
Every flown segment is re-checked independently at the end.

Usage: python3 make_demo_plan.py [--first 1004] [--last 1026] [--src ...] [--out ...]
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plan_gazebo_mission as P  # noqa: E402

ROOT = P.ROOT


def length(pts):
    return round(sum(float(np.linalg.norm(np.subtract(q, p))) for p, q in zip(pts[:-1], pts[1:])), 3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--first', type=int, default=1004)
    ap.add_argument('--last', type=int, default=1026)
    ap.add_argument('--src', default=os.path.join(ROOT, 'mission', 'gazebo_mission_plan_v5.json'))
    ap.add_argument('--out', default=os.path.join(ROOT, 'mission', 'gazebo_demo_plan.json'))
    a = ap.parse_args()
    t0 = time.time()
    plan = json.load(open(a.src))
    W = [dict(w) for w in plan['waypoints'][a.first:a.last + 1]]
    assert all(w.get('reachable_in_plan', True) for w in W), 'unreachable waypoint inside the cut'
    prims, _, _ = P.load_obstacles()
    grid = P.Grid(prims)
    home_air = plan['home_air_world_m']

    via, method, mc = P.route(home_air, W[0]['position_m'], prims, grid)
    assert via is not None, f'no route home -> {W[0]["waypoint_id"]}: {method}'
    W[0]['route_in'], W[0]['route_method'] = via, method
    W[0]['route_min_clearance_m'] = None if mc is None else round(mc, 3)
    W[0]['route_length_m'] = length([home_air] + via + [W[0]['position_m']])
    rvia, rmethod, rmc = P.route(W[-1]['position_m'], home_air, prims, grid)
    assert rvia is not None, f'no route {W[-1]["waypoint_id"]} -> home: {rmethod}'

    # independent re-check of every segment that can be flown, in both directions
    n_seg = n_bad = 0
    raw_min = math.inf
    chains = [[home_air] + W[0]['route_in'] + [W[0]['position_m']]]
    for w0, w1 in zip(W[:-1], W[1:]):
        chains.append([w0['position_m']] + w1['route_in'] + [w1['position_m']])
    chains.append([W[-1]['position_m']] + rvia + [home_air])
    for pts in chains:
        for p_, q_ in zip(pts[:-1], pts[1:]):
            n_seg += 1
            n_bad += not P.seg_ok(p_, q_, prims)
            raw_min = min(raw_min, P.segment_min_clearance(p_, q_, prims))
    print(f'segment re-check: {n_seg} segments, {n_bad} violate slope/clearance, '
          f'raw min clearance {raw_min:.3f} m', flush=True)

    out = {k: v for k, v in plan.items() if k not in ('waypoints', 'legs', 'rth', 'planned_total_path_m',
                                                       'n_waypoints', 'route_failures', 'min_route_clearance_m')}
    total = sum(w['route_length_m'] for w in W)
    out.update({
        'generated_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'planner': 'gazebo/mission_follower/make_demo_plan.py (offline; geometry only, no defect data)',
        'source_plan': os.path.relpath(a.src, ROOT), 'cut': {'first_index': a.first, 'last_index': a.last,
                                                            'first_id': W[0]['waypoint_id'], 'last_id': W[-1]['waypoint_id']},
        'n_waypoints': len(W),
        'rth': {'route': rvia, 'method': rmethod, 'min_clearance_m': None if rmc is None else round(rmc, 3),
                'length_m': length([W[-1]['position_m']] + rvia + [home_air])},
        'segment_recheck': {'n_segments': n_seg, 'n_violations': n_bad, 'raw_min_clearance_m': round(raw_min, 3)},
        'planned_total_path_m': round(total, 2), 'waypoints': W})
    json.dump(out, open(a.out, 'w'), indent=1)
    print(f'{W[0]["waypoint_id"]}..{W[-1]["waypoint_id"]}: {len(W)} waypoints, home->first {method} '
          f'{W[0]["route_length_m"]} m, one pass {total - W[0]["route_length_m"]:.1f} m, '
          f'RTH {rmethod} {out["rth"]["length_m"]} m', flush=True)
    print(f'wrote {a.out} in {time.time() - t0:.0f} s')
    return 1 if n_bad else 0


if __name__ == '__main__':
    sys.exit(main())
