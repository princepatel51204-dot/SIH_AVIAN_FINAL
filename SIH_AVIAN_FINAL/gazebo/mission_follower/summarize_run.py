#!/usr/bin/env python3
"""Summarise one mission_follower run from its own output files only
(mission_log.json, pose_audit.json, contact_summary.json,
camera/camera_stats.json) -> <run>/task1_summary.json + stdout.

Usage: summarize_run.py results/<run>
"""
import json
import os
import sys

import numpy as np

run = sys.argv[1]
L = json.load(open(os.path.join(run, 'mission_log.json')))
A = json.load(open(os.path.join(run, 'pose_audit.json')))
C = json.load(open(os.path.join(run, 'contact_summary.json')))
cam_p = os.path.join(run, 'camera', 'camera_stats.json')
CAM = json.load(open(cam_p)) if os.path.exists(cam_p) else None
plan = json.load(open(L['plan']))

wl = L['waypoints']
ids = [w['waypoint_id'] for w in wl]
res = {}
for w in wl:
    res[w['result']] = res.get(w['result'], 0) + 1
recs = [r for r in A['records'] if r['result'] == 'reached']
te = np.array([r['true_err_m'] for r in recs]) if recs else np.zeros(0)
off = np.array([r['true_minus_target_m'] for r in recs]) if recs else np.zeros((0, 3))
ev = np.array([r['ekf_vs_true_err_m'] for r in recs]) if recs else np.zeros(0)
nudged = {w['waypoint_id'] for w in plan['waypoints'] if w['nudged']}
te_orig = np.array([r['true_err_m'] for r in recs if r['waypoint_id'] not in nudged])


def st(a):
    if not len(a):
        return None
    return {'n': int(len(a)), 'mean': round(float(a.mean()), 4), 'median': round(float(np.median(a)), 4),
            'p95': round(float(np.percentile(a, 95)), 4), 'max': round(float(a.max()), 4)}


legs = [w for w in wl if 'leg_dist_m' in w]
out = {
    'run': os.path.basename(os.path.normpath(run)),
    'final': L.get('final'),
    'waypoints_in_source_mission': len(plan['waypoints']),
    'logged': len(wl),
    'unique_logged': len(set(ids)),
    'duplicates': len(ids) - len(set(ids)),
    'results': res,
    'reached': res.get('reached', 0),
    'nudged_in_plan': len(nudged),
    'mapping_true_minus_target_m': st(te),
    'mapping_unnudged_only_m': st(te_orig),
    'mapping_mean_offset_xyz_m': off.mean(0).round(4).tolist() if len(off) else None,
    'mapping_std_offset_xyz_m': off.std(0).round(4).tolist() if len(off) else None,
    'ekf_vs_true_m': st(ev),
    'yaw_err_deg': (A.get('summary') or {}).get('yaw_err_deg'),
    'speed_calc': L['speed_calc'],
    'brake_test': L.get('brake_test'),
    'sense_stats': L['sense_stats'],
    'min_sensed_range_at_legs_m': min((w['min_sensed_range_m'] for w in legs if w.get('min_sensed_range_m') is not None), default=None),
    'contacts': {'episodes': C['contact_episodes'], 'samples': C['contact_samples']},
    'camera': None if CAM is None else {k: CAM[k] for k in ('frames', 'duration_wall_s', 'rate_hz', 'max_gap_s', 'waypoint_snapshots')},
    'summary_follower': L.get('summary'),
    'distance_true_m': (A.get('summary') or {}).get('distance_true_m'),
    'planned_total_path_m': plan['planned_total_path_m'],
}
json.dump(out, open(os.path.join(run, 'task1_summary.json'), 'w'), indent=1)
print(json.dumps(out, indent=1))
