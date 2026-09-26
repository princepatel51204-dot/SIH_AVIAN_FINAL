#!/usr/bin/env python3
"""Write a small, TRACKED summary of a column-orbit flight (results/ run folders are gitignored).

Every number is copied from the run's own files (mission_log.json, pose_audit.json, pose_audit_track.csv,
contact_summary.json, detection/detection_stats.json) or from the committed analysis in detection_eval/;
nothing is typed in. The dashboard reads this file.

Usage: make_columns_flight_summary.py <run_dir> <out.json>
"""
import csv, json, os, subprocess, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
run, out = sys.argv[1], sys.argv[2]
name = os.path.basename(os.path.normpath(run))
L = json.load(open(f'{run}/mission_log.json')); S = L['summary']; W = L['waypoints']
A = json.load(open(f'{run}/pose_audit.json')); R = [r for r in A['records'] if r['result'] == 'reached']
K = json.load(open(f'{run}/contact_summary.json'))
D = json.load(open(f'{run}/detection/detection_stats.json'))
cov = json.load(open(f'{HERE}/detection_eval/{name}_coverage_measured.json'))
prec = json.load(open(f'{HERE}/detection_eval/{name}_precision_strict.json'))['summary']
tgt = json.load(open(f'{HERE}/detection_eval/{name}_box_targets.json'))
rec = json.load(open(f'{HERE}/detection_eval/{name}_snapshot_precision_recall.json'))['summary']
plan = json.load(open(os.path.join(ROOT, L['plan'].split('SIH_AVIAN_FINAL/')[-1])))

track = []
for row in csv.DictReader(open(f'{run}/pose_audit_track.csv')):
    track.append((float(row['x']), float(row['y']), float(row['z'])))
dec, last = [], None
for p in track:
    if last is None or np.linalg.norm(np.subtract(p, last)) >= 1.0:
        dec.append([round(v, 2) for v in p]); last = p
ms = np.array([w['min_sensed_range_m'] for w in W])
mis = [t[6][0] for t in L['track'] if t[5] == 'MISSION' and t[6] and t[6][0] is not None]
e = np.array([r['true_err_m'] for r in R])
dw = [w['leg_time_s'] for w in W if '_D' in w['waypoint_id']]
res = {w['result'] for w in W}
summary = {
    'run': name, 'plan': os.path.relpath(os.path.join(ROOT, L['plan'].split('SIH_AVIAN_FINAL/')[-1]), ROOT),
    'sources': ['gazebo/mission_follower/results/%s/{mission_log.json,pose_audit.json,pose_audit_track.csv,contact_summary.json,detection/detection_stats.json}' % name,
                'gazebo/mission_follower/detection_eval/%s_*.json' % name],
    'mission': {'waypoints_in_plan': S['waypoints_in_plan'], 'attempted': S['attempted'], 'reached': S['reached'],
                'results': {r: sum(1 for w in W if w['result'] == r) for r in sorted(res)},
                'look_up_dwells': sum(1 for w in W if '_D' in w['waypoint_id']), 'columns': sorted({w['waypoint_id'].split('_')[0] for w in W}),
                'contact_episodes': K['contact_episodes'],
                'mission_time_sim_s': S['mission_time_sim_s'], 'mission_time_wall_s': S['mission_time_wall_s'],
                'rtf': round(S['mission_time_sim_s'] / S['mission_time_wall_s'], 3),
                'distance_true_m': A['summary'].get('distance_true_m'), 'distance_ekf_m': S['distance_ekf_m'],
                'true_pos_err_m': {'mean': round(float(e.mean()), 3), 'p95': round(float(np.percentile(e, 95)), 3), 'max': round(float(e.max()), 3)},
                'yaw_err_deg': A['summary']['yaw_err_deg'], 'gimbal_err_deg': A['summary']['gimbal_err_deg'],
                'min_sensed_range_mission_legs_m': round(float(ms.min()), 3), 'legs_min_below_3p5': int((ms < 3.5).sum()),
                'legs_min_below_3p0': int((ms < 3.0).sum()),
                'mission_samples': len(mis), 'mission_samples_inside_3p0_ring': int(sum(1 for d in mis if d < 3.0)),
                'legs_speed_limited': int(sum(1 for w in W if w['speed_limited_frac'] > 0)),
                'dwell_time_s_mean': round(float(np.mean(dw)), 2) if dw else None},
    'aiming': {'ring_axis_on_column': f"{cov['axis_totals']['ring_ok']}/{cov['axis_totals']['ring_n']}",
               'dwell_axis_on_cap_or_column': f"{cov['axis_totals']['dw_ok']}/{cov['axis_totals']['dw_n']}"},
    'coverage_note': cov['note'], 'coverage_mean_whole_true_pct': cov['mean_whole_true'],
    'coverage_per_column': {k: {'planned': v['planned_claim_pct'], 'measured_true_poses': v['planner_model_on_TRUE_poses_pct'],
                                'waypoints': v['waypoints_flown'], 'dwells': v['dwells']} for k, v in cov['per_column'].items()},
    'detector': {'frames_processed': D['frames_processed'], 'achieved_hz': D['achieved_hz'], 'boxes': prec['boxes'],
                 'box_precision_location': prec['precision_location'], 'box_tp_location': prec['tp_location'],
                 'box_tp_defects': prec['tp_boxes_by_defect'], 'box_centre_on': tgt['box_centre_on'],
                 'precision_vs_confidence_floor': prec['precision_vs_confidence_floor'],
                 'snapshot_recall_instances': rec['recall_instances_location'], 'snapshot_usable_instances': rec['usable_defect_instances'],
                 'note': 'Gazebo world is untextured SDF primitives; these numbers are pipeline behaviour, not detection accuracy.'},
    'orbit_viewpoints_xy': [[round(w['position_m'][0], 2), round(w['position_m'][1], 2)] for w in plan['waypoints'] if not w.get('dwell')],
    'track_xyz_1m': dec,
    'commit_at_build': subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=ROOT, capture_output=True, text=True).stdout.strip(),
}
json.dump(summary, open(out, 'w'))
print(json.dumps({k: v for k, v in summary.items() if k not in ('track_xyz_1m', 'orbit_viewpoints_xy', 'coverage_per_column')}, indent=1))
