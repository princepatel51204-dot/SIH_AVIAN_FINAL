#!/usr/bin/env python3
"""What scene object lies under the centre of each detector box? (OFFLINE scoring aid.)

For every box in <run>/detection/detections.json, cast the ray through the box centre from the TRUE
camera pose (pose_audit_track.csv at the detection timestamp) into the scene solids (covlib) and name
the first solid it hits. Uses simulator truth only to explain what the detector responded to; nothing
here feeds navigation.

Usage: detection_box_targets.py <run_dir> [--json out.json]
"""
import argparse, collections, json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, 'gazebo', 'coverage_v5'))
import eval_detection_gazebo as E  # noqa: E402
import covlib as C  # noqa: E402


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('run'); ap.add_argument('--json'); a = ap.parse_args()
    S = C.Solids(); track = E.Track(os.path.join(a.run, 'pose_audit_track.csv'))
    D = json.load(open(os.path.join(a.run, 'detection', 'detections.json')))['detections']

    def first_hit(o, d, tmax=60, step=0.1):
        ts = np.arange(0.3, tmax, step); pts = o + ts[:, None] * d
        m = C.fast_clearance(S, pts) < 0.05
        if not m.any():
            return None, None
        i = int(np.argmax(m)); p = pts[i]; best, name = 1e9, None
        for k in range(len(S.rows)):
            s = C._sdf(p[0], p[1], p[2], int(S.typ[k]), S.c[k, 0], S.c[k, 1], S.c[k, 2], S.h[k, 0], S.h[k, 1], S.h[k, 2], S.cyaw[k], S.syaw[k])
            if s < best: best, name = s, S.names[k]
        return float(ts[i]), name
    rows, cnt = [], collections.Counter()
    for d in D:
        cam, Rc, _ = E.camera(track, d['sim_time_s']); x0, y0, x1, y1 = d['bbox_xyxy_px']
        t, name = first_hit(cam, Rc @ E.V.pixel_ray((x0 + x1) / 2, (y0 + y1) / 2))
        fam = 'none' if name is None else '_'.join(name.split('_')[:2])
        cnt[fam] += 1
        rows.append({'waypoint': d['waypoint_id'], 'confidence': d['confidence'], 'class': d['class'], 'solid': name, 'range_m': None if t is None else round(t, 2)})
    out = {'run': os.path.basename(os.path.normpath(a.run)), 'boxes': len(D), 'box_centre_on': dict(cnt.most_common()), 'rows': rows}
    print(json.dumps({k: out[k] for k in ('run', 'boxes', 'box_centre_on')}, indent=1))
    if a.json: json.dump(out, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
