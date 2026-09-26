#!/usr/bin/env python3
"""Independent checks of a per-column orbital plan (mission/gazebo_columns_plan.json).

OFFLINE. Does not use plan_columns.py's own bookkeeping; re-derives everything from the
plan file and the scene solids (covlib):

  1. AIMING   for every viewpoint, cast the planned camera axis (camera_world_m, heading,
              gimbal pitch) into the scene and report the first solid it hits. A viewpoint
              "frames a column" when that solid is a pier column (BR_/MB_PIER_COL).
  2. AIRFRAME base_link clearance to every solid, sampled every 0.25 m along EVERY route
              piece (takeoff-to-home included), must be >= 3.5 m.
  3. ROUTE    every route piece is flat (<= 12 deg elevation) or steep (>= 50 deg).
  4. CAMERA   pivot clearance at each viewpoint (>= 3.5 m required) and, informationally,
              along the route pieces at the destination heading (the film-era rule is not
              part of the sensed safety layer; see the report).

Usage: verify_columns_plan.py [plan.json] [--json out.json] [--first N]
"""
import argparse
import collections
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import covlib as C  # noqa: E402

PIV = np.array([0.35, 0.0, 0.05])
COL = re.compile(r'^(BR|MB)_PIER_COL_')


def first_hit(S, o, d, tmax=30.0, step=0.05):
    ts = np.arange(0.2, tmax, step)
    pts = o + ts[:, None] * d
    c = C.fast_clearance(S, pts)
    m = c < 0.03
    if not m.any():
        return None, None
    i = int(np.argmax(m))
    p = pts[i]
    best, name = 1e9, None
    for k in range(len(S.rows)):
        s = C._sdf(p[0], p[1], p[2], int(S.typ[k]), S.c[k, 0], S.c[k, 1], S.c[k, 2],
                   S.h[k, 0], S.h[k, 1], S.h[k, 2], S.cyaw[k], S.syaw[k])
        if s < best:
            best, name = s, S.names[k]
    return float(ts[i]), name


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('plan', nargs='?', default=os.path.join(C.ROOT, 'mission', 'gazebo_columns_plan.json'))
    ap.add_argument('--json')
    ap.add_argument('--first', type=int, default=0)
    a = ap.parse_args()
    S = C.Solids()
    plan = json.load(open(a.plan))
    W = plan['waypoints'][:a.first] if a.first else plan['waypoints']
    out = {'plan': os.path.basename(a.plan), 'n_waypoints': len(W)}

    # 1. aiming
    per = collections.OrderedDict()
    miss = []
    for w in W:
        cam = np.array(w['camera_world_m'])
        f, _, _ = C.cam_axes(w['heading_rad'], w['gimbal_pitch_rad'])
        t, name = first_hit(S, cam, np.asarray(f, float))
        ok = bool(name and COL.match(name))
        d = per.setdefault(w['column_id'], {'n': 0, 'on_column': 0, 'range_m': []})
        d['n'] += 1
        d['on_column'] += ok
        if t is not None and ok:
            d['range_m'].append(t)
        if not ok:
            miss.append({'waypoint': w['waypoint_id'], 'ring': w['ring'], 'hit': name,
                         'range_m': None if t is None else round(t, 2),
                         'pitch_deg': round(math.degrees(w['gimbal_pitch_rad']), 1)})
    tot = sum(v['on_column'] for v in per.values())
    out['aiming'] = {'viewpoints_axis_hits_a_column': tot, 'of': len(W),
                     'by_subject': {k: {'on_column': v['on_column'], 'of': v['n'],
                                        'range_min_m': round(min(v['range_m']), 2) if v['range_m'] else None,
                                        'range_max_m': round(max(v['range_m']), 2) if v['range_m'] else None}
                                    for k, v in per.items()},
                     'misses': miss}
    # 2/3. airframe clearance + route pieces
    p0 = plan['home_air_world_m']
    segs = []
    for w in W:
        chain = [p0] + w['route_in'] + [w['position_m']]
        for x, y in zip(chain[:-1], chain[1:]):
            segs.append((np.array(x, float), np.array(y, float), w['heading_rad']))
        p0 = w['position_m']
    if not a.first:
        chain = [p0] + plan['rth']['route'] + [plan['home_air_world_m']]
        for x, y in zip(chain[:-1], chain[1:]):
            segs.append((np.array(x, float), np.array(y, float), None))
    B, Pv, bad_el = [], [], 0
    for x, y, yaw in segs:
        L = float(np.linalg.norm(y - x))
        if L < 1e-6:
            continue
        el = math.degrees(math.atan2(abs(y[2] - x[2]), math.hypot(*(y - x)[:2])))
        if not (el <= 12.0 + 1e-6 or el >= 50.0):
            bad_el += 1
        q = x + (y - x) * np.linspace(0, 1, max(2, int(math.ceil(L / 0.25)) + 1))[:, None]
        B.append(q)
        if yaw is not None:
            Pv.append(q + np.array([math.cos(yaw) * PIV[0], math.sin(yaw) * PIV[0], PIV[2]]))
    B, Pv = np.vstack(B), np.vstack(Pv)
    cb, cp = C.fast_clearance(S, B), C.fast_clearance(S, Pv)
    out['airframe'] = {'samples': int(len(B)), 'min_clearance_m': round(float(cb.min()), 3),
                       'samples_below_3.5': int((cb < 3.5 - 1e-6).sum()), 'samples_below_3.0': int((cb < 3.0).sum())}
    out['route_pieces'] = {'segments': len(segs), 'neither_flat_nor_steep': bad_el}
    wp = np.array([w['position_m'] for w in W])
    pv = np.array([w['camera_world_m'] for w in W])
    out['at_viewpoints'] = {'base_min_m': round(float(C.fast_clearance(S, wp).min()), 3),
                            'camera_pivot_min_m': round(float(C.fast_clearance(S, pv).min()), 3),
                            'z_min_m': round(float(wp[:, 2].min()), 2)}
    out['camera_pivot_along_routes_informational'] = {'min_m': round(float(cp.min()), 3), 'samples_below_3.5': int((cp < 3.5 - 1e-6).sum())}
    print(json.dumps(out, indent=1))
    if a.json:
        json.dump(out, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
