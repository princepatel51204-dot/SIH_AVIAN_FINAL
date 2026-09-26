#!/usr/bin/env python3
"""Cut a short demo plan out of the per-column orbital plan (mission/gazebo_columns_plan.json).

OFFLINE MISSION PLANNING (same rule as plan_columns.py: the only step that reads scene geometry). The chosen
columns' waypoints -- positions, headings, gimbal pitches, look-up dwells and the routes BETWEEN them -- are
copied unchanged from the columns plan. Only two routes are new, planned with the same router and the same
exact clearance (covlib, strict 3.5 m, no relaxation) that plan_columns.py used:
  * home -> first waypoint of the cut
  * last waypoint of the cut -> home   (plan['rth'])
Every flown segment is then re-checked (run gazebo/coverage_v5/verify_columns_plan.py on the output).

Usage: make_columns_demo_plan.py [--columns RP03,RP04] [--out mission/gazebo_columns_demo_plan.json]
"""
import argparse, json, math, os, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'mission_follower'))
import covlib as C  # noqa: E402
import plan_gazebo_mission as PG  # noqa: E402


def length(pts):
    return round(sum(float(np.linalg.norm(np.subtract(q, p))) for p, q in zip(pts[:-1], pts[1:])), 3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--columns', default='RP03,RP04')
    ap.add_argument('--src', default=os.path.join(C.ROOT, 'mission', 'gazebo_columns_plan.json'))
    ap.add_argument('--out', default=os.path.join(C.ROOT, 'mission', 'gazebo_columns_demo_plan.json'))
    a = ap.parse_args()
    t0 = time.time()
    want = a.columns.split(',')
    plan = json.load(open(a.src))
    W = [dict(w) for w in plan['waypoints'] if w['column_id'] in want]
    assert W and all(w['reachable_in_plan'] for w in W), 'empty cut or unreachable waypoint'
    order = [w['column_id'] for w in W]
    assert order == sorted(order, key=lambda c: order.index(c)), 'columns are not contiguous in the source plan'
    S = C.Solids()
    PG.clearance = lambda pts, prims: C.fast_clearance(S, pts)
    PG.TIER3_CLEARANCE_M = 3.5
    grid = PG.Grid(None)
    home = plan['home_air_world_m']
    via, method, _ = PG.route(home, W[0]['position_m'], None, grid)
    assert via is not None, f'no route home -> {W[0]["waypoint_id"]}'
    W[0]['route_in'], W[0]['route_method'] = via, 'transit_' + method
    W[0]['route_length_m'] = length([home] + via + [W[0]['position_m']])
    rvia, rmethod, _ = PG.route(W[-1]['position_m'], home, None, grid)
    assert rvia is not None, f'no route {W[-1]["waypoint_id"]} -> home'
    out = {k: v for k, v in plan.items() if k not in ('waypoints', 'rth', 'n_waypoints', 'n_look_up_dwells',
                                                       'planned_total_path_m', 'segment_recheck', 'route_failures')}
    total = sum(w['route_length_m'] for w in W) + length([W[-1]['position_m']] + rvia + [home])
    out.update({'generated_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                'planner': 'gazebo/coverage_v5/make_columns_demo_plan.py (cut of the columns plan; offline; geometry only)',
                'source_plan': os.path.relpath(a.src, C.ROOT), 'columns': want,
                'n_waypoints': len(W), 'n_look_up_dwells': sum(1 for w in W if w.get('dwell')), 'route_failures': 0,
                'rth': {'route': rvia, 'method': rmethod, 'length_m': length([W[-1]['position_m']] + rvia + [home])},
                'planned_total_path_m': round(total, 2), 'waypoints': W})
    json.dump(out, open(a.out, 'w'), indent=1)
    est = (total / 1.9 + 3.9 * (len(W) - out['n_look_up_dwells']) + 2.0 * out['n_look_up_dwells']) / 60
    print(f"{want}: {len(W)} waypoints ({out['n_look_up_dwells']} dwells), path {total:.0f} m incl. home->first "
          f"{W[0]['route_length_m']} m and RTH {out['rth']['length_m']} m; est {est:.1f} sim-min -> {a.out} ({time.time() - t0:.0f} s)")


if __name__ == '__main__':
    main()
