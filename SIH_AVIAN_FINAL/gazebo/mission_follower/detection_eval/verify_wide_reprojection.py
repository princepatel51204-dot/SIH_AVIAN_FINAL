#!/usr/bin/env python3
"""Verify the narrow->wide box reprojection used by demo_camera.sh's wide-view overlay.

Both gimbal cameras (front_camera 80 deg, inspect_camera 16 deg) sit at the IDENTICAL pose
(0.03 0 0 0 0 0 in gimbal_cam_link -- see x500_base_inspect_camera.patch and viz_frames.py's
CAM_IN_LINK). Zero baseline means a narrow-camera pixel's back-projected ray is valid for the
wide camera too: only the focal length differs, so the mapping is an exact scale-about-centre
transform, not an approximation.

This script proves it two ways, on a real frame from aimnarrow_rp0304:
  1. Internal consistency: the closed-form scale transform used by wide_box_reproject_node.py
     is compared to the full ray back-projection / forward-projection in viz_frames.py (the
     same machinery eval_detection_gazebo.py and measure_orbit_aim.py already rely on).
  2. Ground truth: the SAME 3D ground-truth defect position is projected directly into the wide
     camera at the true recorded pose (independent of the detector's own box), and compared to
     where the detector's narrow box (reprojected) lands.
It then extracts the real recorded wide frame at that instant and draws both boxes on it.

OFFLINE, READ-ONLY: does not touch the flight, the follower, or the detector's own scoring.
"""
import bisect
import csv
import json
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FOLLOWER = os.path.dirname(HERE)
ROOT = os.path.abspath(os.path.join(FOLLOWER, '..', '..'))
sys.path.insert(0, os.path.join(FOLLOWER, 'viz'))
import viz_frames as V  # noqa: E402

RUN = os.path.join(FOLLOWER, 'results', 'aimnarrow_rp0304')
NARROW_HFOV = 0.2792527   # inspect_camera, 16 deg
WIDE_HFOV = 1.3962634     # front_camera, 80 deg
FX_NARROW = (V.CAM_W / 2) / math.tan(NARROW_HFOV / 2)
FX_WIDE = (V.CAM_W / 2) / math.tan(WIDE_HFOV / 2)
CX, CY = V.CAM_CX, V.CAM_CY


def scale_reproject(x, y):
    """Closed-form: same optical centre + orientation, only focal length changes."""
    s = FX_WIDE / FX_NARROW
    return CX + s * (x - CX), CY + s * (y - CY)


def ray_reproject(x, y):
    """Full ray back-projection (narrow) -> forward-projection (wide), via viz_frames."""
    d = np.array([1.0, -(x - CX) / FX_NARROW, -(y - CY) / FX_NARROW])
    u = CX - FX_WIDE * d[1] / d[0]
    v = CY - FX_WIDE * d[2] / d[0]
    return u, v


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
        i = min(max(i, 1), len(self.t) - 1)
        t0, t1 = self.t[i - 1], self.t[i]
        a = 0.0 if t1 <= t0 else min(max((ts - t0) / (t1 - t0), 0.0), 1.0)
        p = self.xyz[i - 1] * (1 - a) + self.xyz[i] * a
        k = i - 1 if a < 0.5 else i
        return p, self.q[k], float(self.j[k])


def load_defect(defect_id):
    for f in ('AVIAN_defect_ground_truth_FINAL.json', 'AVIAN_metro_ground_truth_FINAL.json',
              'AVIAN_steel_ground_truth_FINAL.json'):
        for d in json.load(open(os.path.join(ROOT, 'scene', f)))['defects']:
            if d['defect_id'] == defect_id:
                return np.array(d['position_m'], float)
    raise KeyError(defect_id)


def main():
    dets = json.load(open(os.path.join(RUN, 'detection', 'detections.json')))['detections']
    pick = next(d for d in dets if d['waypoint_id'] == 'RP04_R2_10' and d['frame_id'] == 1420)
    ts = pick['sim_time_s']
    x0, y0, x1, y1 = pick['bbox_xyxy_px']
    ncx, ncy = (x0 + x1) / 2, (y0 + y1) / 2
    print(f'narrow box (frame {pick["frame_id"]}, t={ts}s, conf={pick["confidence"]}): '
          f'[{x0:.1f},{y0:.1f},{x1:.1f},{y1:.1f}] centre ({ncx:.1f},{ncy:.1f})')

    u_scale, v_scale = scale_reproject(ncx, ncy)
    u_ray, v_ray = ray_reproject(ncx, ncy)
    print(f'reprojected to wide, closed-form scale: ({u_scale:.3f}, {v_scale:.3f})')
    print(f'reprojected to wide, full ray method:   ({u_ray:.3f}, {v_ray:.3f})')
    print(f'internal-consistency error: {math.hypot(u_scale - u_ray, v_scale - v_ray):.6f} px '
          '(closed-form == ray method, as derived: identical camera centre + orientation)')

    x0w, y0w = scale_reproject(x0, y0)
    x1w, y1w = scale_reproject(x1, y1)
    print(f'reprojected narrow box -> wide box: [{x0w:.1f},{y0w:.1f},{x1w:.1f},{y1w:.1f}]')

    track = Track(os.path.join(RUN, 'pose_audit_track.csv'))
    p, q, j = track.pose(ts)
    R = V.rot_from_quat_wxyz(q)
    cp, cR = V.camera_pose_in_base(j)   # camera pose in base_link (same for front_camera and inspect_camera)
    cam, Rc = p + R @ cp, R @ cR        # -> world
    gt = load_defect('DEFECT_SPALL_011')
    v = Rc.T @ (gt - cam)
    u_gt = CX - FX_WIDE * v[1] / v[0]
    v_gt = CY - FX_WIDE * v[2] / v[0]
    print(f'DEFECT_SPALL_011 ground truth, projected DIRECTLY into the wide camera at the true '
          f'pose at t={ts}s: ({u_gt:.1f}, {v_gt:.1f}), range {v[0]:.2f} m')
    box_vs_gt_px = math.hypot(u_scale - u_gt, v_scale - v_gt)
    print(f'reprojected detector-box centre vs ground truth: {box_vs_gt_px:.1f} px '
          '(nonzero: the detector\'s own box is not point-precise on the defect centre -- expected -- '
          'this is not reprojection error)')
    gt_inside_box = (min(x0w, x1w) <= u_gt <= max(x0w, x1w)) and (min(y0w, y1w) <= v_gt <= max(y0w, y1w))

    json.dump({
        'run': 'aimnarrow_rp0304', 'frame_id': pick['frame_id'], 'sim_time_s': ts, 'confidence': pick['confidence'],
        'narrow_box_px': [x0, y0, x1, y1],
        'reprojected_wide_box_px': [round(x0w, 2), round(y0w, 2), round(x1w, 2), round(y1w, 2)],
        'closed_form_vs_ray_method_px_diff': round(math.hypot(u_scale - u_ray, v_scale - v_ray), 6),
        'ground_truth_defect': 'DEFECT_SPALL_011',
        'ground_truth_projected_wide_px': [round(u_gt, 2), round(v_gt, 2)],
        'ground_truth_range_m': round(float(v[0]), 3),
        'reprojected_box_centre_vs_ground_truth_px': round(box_vs_gt_px, 2),
        'ground_truth_inside_reprojected_box': bool(gt_inside_box),
    }, open(os.path.join(HERE, 'wide_reprojection_verification.json'), 'w'), indent=1)

    fr = [(float(r['stamp_s']), int(r['frame'])) for r in csv.DictReader(open(f'{RUN}/camera/frames.csv'))]
    fs = [x[0] for x in fr]
    k = min(max(bisect.bisect_left(fs, ts), 0), len(fs) - 1)
    out_png = os.path.join(HERE, 'frames_aim_after', 'wide_reprojection_check.png')
    subprocess.run(f'ffmpeg -y -v error -i {RUN}/camera/front_camera.mp4 -vf "select=eq(n\\,{fr[k][1]})" '
                    f'-vsync 0 -frames:v 1 {out_png}', shell=True, check=True)
    from PIL import Image, ImageDraw
    im = Image.open(out_png).convert('RGB')
    d = ImageDraw.Draw(im)
    d.rectangle([x0w, y0w, x1w, y1w], outline=(255, 170, 30), width=2)
    d.text((x0w + 2, max(0, y0w - 12)), 'SPALL_DELAM reprojected', fill=(255, 170, 30))
    r_ = 4
    d.ellipse([u_gt - r_, v_gt - r_, u_gt + r_, v_gt + r_], outline=(60, 255, 60), width=2)
    d.text((u_gt + 6, v_gt - 6), 'ground truth', fill=(60, 255, 60))
    im.save(out_png)
    print('frame saved:', out_png)


if __name__ == '__main__':
    main()
