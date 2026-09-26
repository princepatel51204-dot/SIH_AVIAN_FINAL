#!/usr/bin/env python3
"""Post-flight analysis of a column-orbit run: per-column MEASURED coverage vs the planner's claim, and camera-axis hits.

OFFLINE (simulator truth used only to score the finished flight). Coverage uses the PLANNER'S OWN visibility model
(gazebo/coverage_v5 covlib) evaluated at the TRUE camera pose of every reached waypoint, so it measures how much
pose/aim error erodes the plan - it is not an independent measure of what the camera really saw.

Usage: analyse_columns_flight.py <run_dir> [--plan mission/gazebo_columns_plan.json] [--json out.json]
"""
import argparse, collections, json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(HERE, 'viz')); sys.path.insert(0, os.path.join(ROOT, 'gazebo', 'coverage_v5'))
import viz_frames as V  # noqa: E402
import covlib as C, plan_columns as PC  # noqa: E402
import verify_columns_plan as VP  # noqa: E402

ap = argparse.ArgumentParser(); ap.add_argument('run'); ap.add_argument('--plan', default=os.path.join(ROOT, 'mission', 'gazebo_columns_plan.json')); ap.add_argument('--json'); a = ap.parse_args()
plan = json.load(open(a.plan)); W = {w['waypoint_id']: w for w in plan['waypoints']}
audit = {r['waypoint_id']: r for r in json.load(open(os.path.join(a.run, 'pose_audit.json')))['records'] if r['result'] == 'reached'}
rep = json.load(open(os.path.join(ROOT, 'mission', 'columns_plan_report.json')))['per_subject']
S = C.Solids(); subs = {s['id']: s for s in PC.inventory(S)}


def true_cam(r):
    p = np.array(r['true_world_m']); R = V.rot_from_quat_wxyz(r['true_quat_wxyz'])
    cp, cR = V.camera_pose_in_base(r['true_gimbal_joint_rad'])
    return p + R @ cp, R @ cR


out = {'run': os.path.basename(os.path.normpath(a.run)), 'note': "planner's own visibility model on TRUE camera poses (measures pose/aim error, not independent visibility)", 'per_column': {}}
tot = collections.Counter()
for sid, sub in subs.items():
    ids = [i for i, w in W.items() if w['column_id'] == sid and i in audit]
    if not ids:
        continue
    pat = PC.subject_patches(S, sub); Vv = C.Visibility(S, pat, pat['reason'] == 0); vis = pat['reason'] == 0; A = pat['area']
    col = np.isin(pat['solid'], sub['columns']) & vis
    seen_true = np.zeros(len(A), bool); seen_plan = np.zeros(len(A), bool)
    hits = collections.Counter(); ring_ok = ring_n = dw_ok = dw_n = 0; rng = []
    for i in ids:
        w, r = W[i], audit[i]
        cam, Rc = true_cam(r)
        seen_true[Vv.visible(cam, Rc[:, 0], Rc[:, 1], Rc[:, 2])] = True
        f, l, u = C.cam_axes(w['heading_rad'], w['gimbal_pitch_rad'])
        seen_plan[Vv.visible(np.array(w['camera_world_m']), f, l, u)] = True
        t, name = VP.first_hit(S, cam, Rc[:, 0])
        dwell = bool(w.get('dwell'))
        ok = bool(name and (VP.COL.match(name) or (dwell and VP.CAP.match(name))))
        if dwell: dw_n += 1; dw_ok += ok
        else: ring_n += 1; ring_ok += ok; rng += [t] if (ok and t) else []
        if not ok: hits[f"{i}->{name}"] += 1

    def pct(seen, m):
        m = vis & m
        return None if not m.any() else round(100 * float(A[m & seen].sum()) / float(A[m].sum()), 1)
    masks = {'shaft': col & (pat['face'] == 'side'), 'cap_or_head': np.isin(pat['solid'], sub['caps']), 'footing': np.isin(pat['solid'], sub['footings']), 'whole': np.ones(len(A), bool)}
    claim = rep[sid]['coverage_pct']
    out['per_column'][sid] = {'waypoints_flown': len(ids), 'ring_viewpoints': ring_n, 'dwells': dw_n,
        'planned_claim_pct': {'shaft': claim['shaft'], 'cap_or_head': claim['cap_or_head'], 'footing': claim['footing'], 'whole': claim['whole_subject']},
        'planner_model_on_planned_poses_pct': {k: pct(seen_plan, m) for k, m in masks.items()},
        'planner_model_on_TRUE_poses_pct': {k: pct(seen_true, m) for k, m in masks.items()},
        'axis_hits_column_ring': f'{ring_ok}/{ring_n}', 'axis_hits_cap_or_column_dwell': f'{dw_ok}/{dw_n}', 'axis_range_m': [round(min(rng), 2), round(max(rng), 2)] if rng else None,
        'axis_misses': dict(hits)}
    tot['ring_ok'] += ring_ok; tot['ring_n'] += ring_n; tot['dw_ok'] += dw_ok; tot['dw_n'] += dw_n
    print(sid, json.dumps(out['per_column'][sid]['planner_model_on_TRUE_poses_pct']), out['per_column'][sid]['axis_hits_column_ring'], out['per_column'][sid]['axis_hits_cap_or_column_dwell'], flush=True)
out['axis_totals'] = dict(tot)
whole = [v['planner_model_on_TRUE_poses_pct']['whole'] for v in out['per_column'].values()]
out['mean_whole_true'] = round(float(np.mean(whole)), 1)
print(json.dumps({'axis_totals': out['axis_totals'], 'mean_whole_true': out['mean_whole_true']}))
if a.json: json.dump(out, open(a.json, 'w'), indent=1)
