#!/usr/bin/env python3
"""OFFLINE ACCURACY EVALUATION of the defect register. Ground truth is used HERE ONLY.

Nothing in the live path (live_detector_node, map_viz_node, defect_register, the
follower) reads any of the files this script reads. It is run after a flight, on
the register the drone's own sensors produced, and answers: how far is each
localized defect from where the digital twin says defects actually are?

Three numbers per localized defect, all in metres, all against the Blender/collision
frame (== Gazebo world frame, verified elsewhere in this project):
  d_surface       distance from the localized point to the nearest structure surface
                  (collision primitives + Gazebo visual-only boxes + ground plane).
                  Measures the RANGING / MAP accuracy on its own: a point that lands
                  on real structure has ~0; a large value means the ray-cast hit
                  something that is not there (or a mis-registered map).
  d_gt_any        distance to the nearest ground-truth defect of ANY type.
  d_gt_family     distance to the nearest ground-truth defect of the SAME family the
                  detector reported (labels_final.json's type -> family map).
The detector is a real model running on flat-shaded, untextured Gazebo geometry, so
a detection need not correspond to a real defect at all; d_gt_* therefore mixes
localization error with detection error, and the report says so. A detection is
called MATCHED when d_gt_family <= match_m (default 2.0 m, the register's own gate).

Usage: python3 eval_localization_offline.py <run_dir_or_defect_register.json> [--match 2.0]
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'source'))
sys.path.insert(0, HERE)
import pathfinding_final as PF  # noqa: E402
import plan_gazebo_mission as PGM  # noqa: E402

GT_FILES = ('AVIAN_defect_ground_truth_FINAL.json', 'AVIAN_metro_ground_truth_FINAL.json',
            'AVIAN_steel_ground_truth_FINAL.json')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('path')
    ap.add_argument('--match', type=float, default=2.0)
    a = ap.parse_args()
    p = a.path
    if os.path.isdir(p):
        cand = [os.path.join(p, 'viz', 'defect_register.json'), os.path.join(p, 'defect_register.json')]
        p = next(c for c in cand if os.path.exists(c))
    reg = json.load(open(p))
    out_path = os.path.join(os.path.dirname(p), 'localization_eval_OFFLINE.json')

    gt = []
    for f in GT_FILES:
        gt += json.load(open(os.path.join(ROOT, 'scene', f)))['defects']
    t2f = json.load(open(os.path.join(ROOT, 'dataset', 'labels_final.json')))['type_to_family']
    gpos = np.array([d['position_m'] for d in gt])
    gfam = [t2f.get(d['type']) for d in gt]

    prims, _, _ = PGM.load_obstacles()
    rows = []
    for d in reg['defects']:
        q = np.array([[d['x_m'], d['y_m'], d['z_m']]])
        d_surf = float(min(PF._clearance_batch(q, prims)[0], abs(q[0, 2])))
        dist = np.linalg.norm(gpos - q[0], axis=1)
        i_any = int(np.argmin(dist))
        fam_idx = [i for i, f in enumerate(gfam) if f == d['class']]
        i_fam = min(fam_idx, key=lambda i: dist[i]) if fam_idx else None
        rows.append({
            'id': d['id'], 'class': d['class'], 'n_obs': d['n_obs'], 'n_viewpoints': d['n_viewpoints'],
            'best_conf': d['best_conf'], 'pos_sigma_m_reported': d['pos_sigma_m'],
            'x_m': d['x_m'], 'y_m': d['y_m'], 'z_m': d['z_m'],
            'd_surface_m': round(d_surf, 3),
            'd_gt_any_m': round(float(dist[i_any]), 3), 'nearest_gt_any': gt[i_any]['defect_id'],
            'd_gt_family_m': None if i_fam is None else round(float(dist[i_fam]), 3),
            'nearest_gt_family': None if i_fam is None else gt[i_fam]['defect_id'],
            'gt_size_mm_nearest_family': None if i_fam is None else gt[i_fam].get('feature_size_mm'),
            'matched': i_fam is not None and float(dist[i_fam]) <= a.match,
            'within_reported_1sigma': i_fam is not None and float(dist[i_fam]) <= d['pos_sigma_m']})

    def st(v):
        v = [x for x in v if x is not None]
        return None if not v else {'n': len(v), 'median': round(float(np.median(v)), 3),
                                   'mean': round(float(np.mean(v)), 3), 'max': round(float(np.max(v)), 3)}
    res = {
        'LABEL': 'OFFLINE ACCURACY EVALUATION -- uses ground truth; never part of the live path',
        'register': os.path.relpath(p, ROOT), 'match_radius_m': a.match,
        'register_summary': reg['summary'],
        'n_localized_defects': len(rows), 'n_matched_to_gt_of_same_family': sum(r['matched'] for r in rows),
        'd_surface_m': st([r['d_surface_m'] for r in rows]),
        'd_gt_any_m': st([r['d_gt_any_m'] for r in rows]),
        'd_gt_family_m': st([r['d_gt_family_m'] for r in rows]),
        'd_gt_family_m_matched_only': st([r['d_gt_family_m'] for r in rows if r['matched']]),
        'note': ('d_surface is the clean localization/ranging accuracy. d_gt_* also contains detection error: '
                 'the detector runs on flat-shaded untextured geometry and a box need not sit on a real defect.'),
        'per_defect': rows}
    json.dump(res, open(out_path, 'w'), indent=1)
    print(f"{len(rows)} localized defects, {res['n_matched_to_gt_of_same_family']} within {a.match} m of a same-family ground-truth defect")
    for k in ('d_surface_m', 'd_gt_any_m', 'd_gt_family_m', 'd_gt_family_m_matched_only'):
        print(f'  {k:28s} {res[k]}')
    for r in rows:
        print(f"  {r['id']} {r['class']:12s} n={r['n_obs']:3d} sigma_rep={r['pos_sigma_m_reported']:.2f}  "
              f"d_surface={r['d_surface_m']:.2f}  d_gt_family={r['d_gt_family_m']}  ({r['nearest_gt_family']})")
    print('wrote', out_path)


if __name__ == '__main__':
    main()
