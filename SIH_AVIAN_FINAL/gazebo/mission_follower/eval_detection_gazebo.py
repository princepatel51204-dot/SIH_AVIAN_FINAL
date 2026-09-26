#!/usr/bin/env python3
"""Precision of the live detector's boxes against ground-truth defects, for a Gazebo run.

OFFLINE ANALYSIS. Uses simulator truth (pose_audit_track.csv) and the ground-truth defect lists (all 192)
purely to SCORE detections after the flight; nothing here feeds navigation.

RULES (stated up front, identical for every run compared):
  * A ground-truth defect is USABLE in a frame when its centre projects inside the 640x480 image,
    is in front of the camera, lies between RMIN and RMAX metres from the camera, and the line of
    sight from the camera to it is not blocked by scene solids (the last 0.6 m before the defect
    is ignored, since the defect sits on a surface). Camera pose = TRUE base pose + gimbal at the
    detection's simulation timestamp (5 Hz audit track: position interpolated, attitude nearest).
  * A detector box is a TRUE POSITIVE (location) when it contains the projected centre of >= 1
    USABLE defect. It is a TRUE POSITIVE (location + class) when, additionally, the box's detector
    family equals that defect's family (dataset/labels_final.json type_to_family).
  * Every other box is a FALSE POSITIVE. No box is dropped or re-scored.
  * Precision = TP / all boxes, reported for both TP definitions.

Usage: eval_detection_gazebo.py <run_dir> [--rmin 3.5] [--rmax 10] [--no-los] [--json out.json]
       <run_dir> needs detection/detections.json and pose_audit_track.csv
"""
import argparse
import bisect
import collections
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(HERE, 'viz'))
import viz_frames as V  # noqa: E402


GT_FILES = ('AVIAN_defect_ground_truth_FINAL.json',      # 96 road-bridge defects
            'AVIAN_metro_ground_truth_FINAL.json',       # 20 metro-viaduct defects
            'AVIAN_steel_ground_truth_FINAL.json')       # 76 steel-truss defects (bolts, gussets, ...)


def load_gt():
    """All 192 ground-truth defects (the Gazebo defects model has 192 spheres)."""
    fam = json.load(open(os.path.join(ROOT, 'dataset', 'labels_final.json')))['type_to_family']
    out = []
    for f in GT_FILES:
        for d in json.load(open(os.path.join(ROOT, 'scene', f)))['defects']:
            family = fam.get(d['type']) or fam.get(d.get('base_type', ''), 'OTHER')
            out.append({'id': d['defect_id'], 'type': d['type'], 'family': family, 'p': np.array(d['position_m'], float)})
    return out


class Track:
    def __init__(self, path):
        t, xyz, q, j = [], [], [], []
        for r in csv.DictReader(open(path)):
            t.append(float(r['t_sim']))
            xyz.append([float(r['x']), float(r['y']), float(r['z'])])
            q.append([float(r['qw']), float(r['qx']), float(r['qy']), float(r['qz'])])
            j.append(float(r['gimbal_joint_rad']))
        self.t, self.xyz, self.q, self.j = np.array(t), np.array(xyz), np.array(q), np.array(j)

    def pose(self, ts):
        i = bisect.bisect_left(self.t, ts)
        if i <= 0 or i >= len(self.t):
            i = min(max(i, 1), len(self.t) - 1)
        t0, t1 = self.t[i - 1], self.t[i]
        a = 0.0 if t1 <= t0 else min(max((ts - t0) / (t1 - t0), 0.0), 1.0)
        p = self.xyz[i - 1] * (1 - a) + self.xyz[i] * a
        k = i - 1 if a < 0.5 else i
        return p, self.q[k], float(self.j[k]), float(min(abs(ts - t0), abs(t1 - ts)))


def camera(track, ts):
    p, q, j, dt = track.pose(ts)
    R = V.rot_from_quat_wxyz(q)
    cp, cR = V.camera_pose_in_base(j)
    return p + R @ cp, R @ cR, dt


def project(gt, cam, Rc):
    P = np.array([g['p'] for g in gt])
    v = (P - cam) @ Rc                       # rows: defect in camera frame (x fwd, y left, z up)
    x = v[:, 0]
    ok = x > 0.05
    xs = np.where(ok, x, 1.0)
    u = V.CAM_CX - V.CAM_FX * v[:, 1] / xs
    w = V.CAM_CY - V.CAM_FY * v[:, 2] / xs
    return u, w, np.linalg.norm(P - cam, axis=1), ok


def los_clear(S, C, cam, target, skip_m=0.6, step=0.15):
    d = target - cam
    L = float(np.linalg.norm(d))
    if L <= skip_m + step:
        return True
    ts = np.arange(step, L - skip_m, step)
    pts = cam + (d / L) * ts[:, None]
    return bool((C.fast_clearance(S, pts) > 0.03).all())


def usable_defects(gt, cam, Rc, rmin, rmax, S=None, C=None):
    u, w, r, ok = project(gt, cam, Rc)
    out = []
    for i in np.flatnonzero(ok & (u >= 0) & (u < V.CAM_W) & (w >= 0) & (w < V.CAM_H) & (r >= rmin) & (r <= rmax)):
        if S is not None and not los_clear(S, C, cam, gt[i]['p']):
            continue
        out.append((i, float(u[i]), float(w[i]), float(r[i])))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--rmin', type=float, default=3.5)
    ap.add_argument('--rmax', type=float, default=10.0)
    ap.add_argument('--no-los', action='store_true')
    ap.add_argument('--json')
    a = ap.parse_args()
    gt = load_gt()
    D = json.load(open(os.path.join(a.run, 'detection', 'detections.json')))['detections']
    track = Track(os.path.join(a.run, 'pose_audit_track.csv'))
    S = C = None
    if not a.no_los:
        sys.path.insert(0, os.path.join(ROOT, 'gazebo', 'coverage_v5'))
        import covlib as C  # noqa: E402
        S = C.Solids()
    rows = []
    for d in D:
        cam, Rc, dt = camera(track, d['sim_time_s'])
        x0, y0, x1, y1 = d['bbox_xyxy_px']
        us = usable_defects(gt, cam, Rc, a.rmin, a.rmax, S, C)
        inside = [(i, r) for i, u, w, r in us if x0 <= u <= x1 and y0 <= w <= y1]
        fam_ok = [(i, r) for i, r in inside if gt[i]['family'] == d['class']]
        rows.append({'waypoint': d['waypoint_id'], 'sim_time_s': d['sim_time_s'], 'class': d['class'], 'confidence': d['confidence'],
                     'usable_defects_in_frame': len(us), 'tp_location': bool(inside), 'tp_location_and_class': bool(fam_ok),
                     'defects_in_box': [{'id': gt[i]['id'], 'type': gt[i]['type'], 'family': gt[i]['family'], 'range_m': round(r, 2)} for i, r in inside],
                     'audit_time_gap_s': round(dt, 3)})
    n = len(rows)
    tp, tpc = sum(r['tp_location'] for r in rows), sum(r['tp_location_and_class'] for r in rows)
    confs = np.array([r['confidence'] for r in rows]) if rows else np.zeros(0)

    def prec_at(th):
        sel = [r for r in rows if r['confidence'] >= th]
        return (len(sel), sum(r['tp_location'] for r in sel), sum(r['tp_location_and_class'] for r in sel))
    summ = {'run': os.path.basename(os.path.normpath(a.run)), 'rules': {'rmin_m': a.rmin, 'rmax_m': a.rmax, 'line_of_sight': not a.no_los,
            'tp': 'box contains the projected centre of >= 1 usable ground-truth defect', 'tp_class': 'and detector family == defect family'},
            'boxes': n, 'tp_location': tp, 'tp_location_and_class': tpc, 'false_positives_location': n - tp,
            'precision_location': None if not n else round(tp / n, 4), 'precision_location_and_class': None if not n else round(tpc / n, 4),
            'confidence': None if not n else {'min': round(float(confs.min()), 3), 'max': round(float(confs.max()), 3)},
            'precision_vs_confidence_floor': {str(th): dict(zip(('boxes', 'tp_location', 'tp_class'), prec_at(th))) for th in (0.65, 0.75, 0.85, 0.9, 0.95)},
            'boxes_by_waypoint': collections.Counter(r['waypoint'] for r in rows).most_common(),
            'tp_boxes_by_defect': collections.Counter(x['id'] for r in rows for x in r['defects_in_box']).most_common(),
            'max_audit_time_gap_s': max((r['audit_time_gap_s'] for r in rows), default=None)}
    print(json.dumps(summ, indent=1))
    if a.json:
        json.dump({'summary': summ, 'boxes': rows}, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
