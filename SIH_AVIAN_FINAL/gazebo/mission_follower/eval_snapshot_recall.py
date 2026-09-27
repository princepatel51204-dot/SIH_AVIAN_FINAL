#!/usr/bin/env python3
"""Precision AND recall of the real detector on a run's per-waypoint camera snapshots.

OFFLINE scoring (uses simulator truth only after the flight). For every camera/waypoints/<id>.jpg of a
run: run the real trained detector (same model/threshold as live_detector_node: Faster R-CNN
MobileNetV3-Large FPN, weights v2, score >= 0.65), then apply the SAME rules as eval_detection_gazebo.py:
  usable defect  = ground-truth defect centre inside the 640x480 frame, RMIN..RMAX m from the camera, line of
                   sight clear (all 192 ground-truth defects; TRUE camera pose at the waypoint event)
  box TP         = box contains the projected centre of >= 1 usable defect (location); +class family match
  recall         = usable defect INSTANCES (snapshot, defect) whose centre lies inside >= 1 box  / all instances;
                   also per UNIQUE defect (usable in >= 1 snapshot, detected in >= 1 snapshot)
Nothing is dropped or re-scored; the threshold is never lowered.

Run with the venv that has torch:  /home/prince/avian_rev_c/.venv/bin/python3 eval_snapshot_recall.py <run_dir>
       [--rmin 3.5] [--rmax 10] [--thresh 0.65] [--max N] [--only-dir subdir] [--json out.json]
"""
import argparse, collections, json, os, sys, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, 'scene', 'film')); sys.path.insert(0, os.path.join(ROOT, 'gazebo', 'coverage_v5'))
import eval_detection_gazebo as E  # noqa: E402
import covlib as C  # noqa: E402
from detect import load_model, detect  # noqa: E402  (scene/film/detect.py: the same model construction as the live node)

ap = argparse.ArgumentParser()
ap.add_argument('run'); ap.add_argument('--rmin', type=float, default=3.5); ap.add_argument('--rmax', type=float, default=10.0)
ap.add_argument('--thresh', type=float, default=0.65); ap.add_argument('--max', type=int, default=0); ap.add_argument('--json')
ap.add_argument('--cam', default='wide', choices=['wide', 'narrow'],
                help='wide: camera/waypoints (80 deg front_camera); narrow: inspect/waypoints (16 deg inspect_camera)')
ap.add_argument('--also-wide-usable', action='store_true',
                help='narrow only: also score recall against the defects usable in the WIDE frame at the same pose '
                     '(same 3.5-10 m / line-of-sight rules; a defect outside the narrow frame counts as missed)')
a = ap.parse_args()
import torch; torch.set_num_threads(int(os.environ.get('TORCH_THREADS', '4')))
gt = E.load_gt(); S = C.Solids(); model = load_model()
SNAPDIR = 'camera' if a.cam == 'wide' else 'inspect'
HFOV = E.V.CAM_HFOV if a.cam == 'wide' else E.V.INSPECT_HFOV
WIDE_HFOV = 1.3962634
audit = {r['waypoint_id']: r for r in json.load(open(os.path.join(a.run, 'pose_audit.json')))['records']}
track = E.Track(os.path.join(a.run, 'pose_audit_track.csv'))
snaps = sorted(f for f in os.listdir(os.path.join(a.run, SNAPDIR, 'waypoints')) if f.endswith('.jpg'))
if a.max: snaps = snaps[:a.max]
rows, inst, inst_hit, inst_hit_class = [], 0, 0, 0
uniq_usable, uniq_hit, uniq_hit_class = set(), set(), set()
n_boxes = n_tp = n_tpc = 0
w_inst = w_hit = 0; w_uniq, w_uniq_hit = set(), set()
t0 = time.time()
for k, f in enumerate(snaps):
    wid = f[:-4]
    r = audit.get(wid)
    if not r or r.get('result') != 'reached':
        continue
    dets, _, _ = detect(model, os.path.join(a.run, SNAPDIR, 'waypoints', f), a.thresh)
    cam, Rc, _ = E.camera(track, r['t_sim'])
    E.V.use_camera_hfov(HFOV)
    us = E.usable_defects(gt, cam, Rc, a.rmin, a.rmax, S, C)
    boxes = [(d['family'], d['score'], d['bbox']) for d in dets]
    tp_boxes = tpc_boxes = 0
    for fam, sc, (x0, y0, x1, y1) in boxes:
        inside = [i for i, u, w, rr in us if x0 <= u <= x1 and y0 <= w <= y1]
        tp_boxes += bool(inside); tpc_boxes += any(gt[i]['family'] == fam for i in inside)
    for i, u, w, rr in us:
        inst += 1; uniq_usable.add(gt[i]['id'])
        hit = [b for b in boxes if b[2][0] <= u <= b[2][2] and b[2][1] <= w <= b[2][3]]
        if hit: inst_hit += 1; uniq_hit.add(gt[i]['id'])
        if any(b[0] == gt[i]['family'] for b in hit): inst_hit_class += 1; uniq_hit_class.add(gt[i]['id'])
    if a.also_wide_usable and a.cam == 'narrow':
        E.V.use_camera_hfov(WIDE_HFOV); wus = E.usable_defects(gt, cam, Rc, a.rmin, a.rmax, S, C); E.V.use_camera_hfov(HFOV)
        pn = {i: (u, w) for i, u, w, rr in us}
        for i, *_ in wus:
            w_inst += 1; w_uniq.add(gt[i]['id'])
            if i in pn and any(b[2][0] <= pn[i][0] <= b[2][2] and b[2][1] <= pn[i][1] <= b[2][3] for b in boxes):
                w_hit += 1; w_uniq_hit.add(gt[i]['id'])
    n_boxes += len(boxes); n_tp += tp_boxes; n_tpc += tpc_boxes
    rows.append({'waypoint': wid, 'boxes': [{'family': b[0], 'score': round(b[1], 3), 'bbox': b[2]} for b in boxes], 'usable_defects': [gt[i]['id'] for i, *_ in us], 'tp_boxes': tp_boxes})
    if (k + 1) % 100 == 0: print(f'{k + 1}/{len(snaps)} snapshots, {time.time() - t0:.0f} s', flush=True)
res = {'run': os.path.basename(os.path.normpath(a.run)), 'snapshots_scored': len(rows), 'rules': {'rmin_m': a.rmin, 'rmax_m': a.rmax, 'threshold': a.thresh, 'line_of_sight': True, 'camera': a.cam, 'camera_hfov_deg': round(float(np.degrees(HFOV)), 2)},
       'boxes': n_boxes, 'box_tp_location': n_tp, 'box_tp_location_and_class': n_tpc,
       'precision_location': None if not n_boxes else round(n_tp / n_boxes, 4), 'precision_location_and_class': None if not n_boxes else round(n_tpc / n_boxes, 4),
       'usable_defect_instances': inst, 'instances_detected_location': inst_hit, 'instances_detected_location_and_class': inst_hit_class,
       'recall_instances_location': None if not inst else round(inst_hit / inst, 4), 'recall_instances_location_and_class': None if not inst else round(inst_hit_class / inst, 4),
       'unique_defects_usable': len(uniq_usable), 'unique_detected_location': len(uniq_hit), 'unique_detected_location_and_class': len(uniq_hit_class),
       'recall_unique_location': None if not uniq_usable else round(len(uniq_hit) / len(uniq_usable), 4),
       'recall_unique_location_and_class': None if not uniq_usable else round(len(uniq_hit_class) / len(uniq_usable), 4),
       'usable_defects_by_family': dict(collections.Counter(gt[[g['id'] for g in gt].index(d)]['family'] for d in uniq_usable))}
if a.also_wide_usable and a.cam == 'narrow':
    res['vs_wide_usable'] = {'instances': w_inst, 'instances_detected': w_hit, 'recall_instances': None if not w_inst else round(w_hit / w_inst, 4),
                             'unique': len(w_uniq), 'unique_detected': len(w_uniq_hit), 'recall_unique': None if not w_uniq else round(len(w_uniq_hit) / len(w_uniq), 4)}
print(json.dumps(res, indent=1))
if a.json: json.dump({'summary': res, 'snapshots': rows}, open(a.json, 'w'), indent=1)
