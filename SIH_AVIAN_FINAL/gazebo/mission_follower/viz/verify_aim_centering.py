#!/usr/bin/env python3
"""Verify, from a finished run, that the gimbal camera really points at each waypoint's target.

For every reached waypoint of a run this takes what the run recorded:
  * the true airframe pose and the true gimbal joint angle at the waypoint event
    (pose_audit.json -- simulator truth, used for VERIFICATION ONLY, never for flying)
  * the plan's target_m for that waypoint
  * the camera frame saved at that event (camera/waypoints/<id>.jpg)
projects the target through the real camera model (640x480, 80 deg HFOV, gimbal pivot
and camera offsets from the airframe model) and reports where it lands in the image,
plus the angle between the camera axis and the direction to the target. Nothing is
inferred from what was commanded: a command that was sent but not achieved shows up
as an error here.

The annotated frames draw a white cross at the image centre and a ring at the
projected target pixel (an arrow at the border when the target is outside the frame).

Usage:
  verify_aim_centering.py <run_dir> <plan.json> <out_dir> [--label TEXT]
  verify_aim_centering.py --before <run_dir> --after <run_dir> <plan.json> <out_dir>
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import viz_frames as V  # noqa: E402


def measure(run, plan_path):
    plan = {w['waypoint_id']: w for w in json.load(open(plan_path))['waypoints']}
    audit = json.load(open(os.path.join(run, 'pose_audit.json')))
    rows = []
    for r in audit['records']:
        if r.get('result') != 'reached' or r.get('true_gimbal_joint_rad') is None:
            continue
        w = plan.get(r['waypoint_id'])
        if not w or 'target_m' not in w:
            continue
        p = np.array(r['true_world_m'])
        R = V.rot_from_quat_wxyz(r['true_quat_wxyz'])
        theta = r['true_gimbal_joint_rad']
        cp, cR = V.camera_pose_in_base(theta)
        cam_pos, R_cam = p + R @ cp, R @ cR
        v_world = np.array(w['target_m']) - cam_pos
        v_cam = R_cam.T @ v_world
        px = V.project_to_pixel(v_cam)
        ang = math.degrees(math.acos(max(-1.0, min(1.0, v_cam[0] / np.linalg.norm(v_cam)))))
        # elevation / azimuth of the target relative to the camera axis
        az = math.degrees(math.atan2(v_cam[1], v_cam[0]))
        el = math.degrees(math.atan2(v_cam[2], math.hypot(v_cam[0], v_cam[1])))
        in_frame = bool(px is not None and 0 <= px[0] < V.CAM_W and 0 <= px[1] < V.CAM_H)
        rows.append({'waypoint_id': r['waypoint_id'], 'target_m': w['target_m'],
                     'distance_to_target_m': round(float(np.linalg.norm(v_world)), 2),
                     'gimbal_joint_rad_true': theta, 'gimbal_cmd_target_rad': r.get('target_gimbal_pitch_up_rad'),
                     'pixel_uv': None if px is None else [round(px[0], 1), round(px[1], 1)],
                     'offset_px': None if px is None else [round(px[0] - V.CAM_CX, 1), round(px[1] - V.CAM_CY, 1)],
                     'off_axis_deg': round(ang, 2), 'azimuth_deg': round(az, 2), 'elevation_deg': round(el, 2),
                     'target_in_frame': in_frame,
                     'target_pitch_up_needed_deg': round(math.degrees(math.atan2(v_world[2], math.hypot(v_world[0], v_world[1]))), 1)})
    return rows


def summarize(rows):
    if not rows:
        return {'n': 0}
    a = np.array([r['off_axis_deg'] for r in rows])
    return {'n': len(rows), 'target_in_frame': int(sum(r['target_in_frame'] for r in rows)),
            'off_axis_deg': {'mean': round(float(a.mean()), 2), 'median': round(float(np.median(a)), 2),
                             'max': round(float(a.max()), 2)},
            'within_5deg': int((a <= 5).sum()), 'within_10deg': int((a <= 10).sum())}


def annotate(img, row, tag):
    im = img.convert('RGB').copy()
    d = ImageDraw.Draw(im)
    cx, cy = V.CAM_CX, V.CAM_CY
    d.line([(cx - 14, cy), (cx + 14, cy)], fill=(255, 255, 255), width=1)
    d.line([(cx, cy - 14), (cx, cy + 14)], fill=(255, 255, 255), width=1)
    px = row['pixel_uv']
    if px and row['target_in_frame']:
        u, v = px
        d.ellipse([u - 14, v - 14, u + 14, v + 14], outline=(255, 60, 60), width=3)
        d.line([(u - 20, v), (u + 20, v)], fill=(255, 60, 60), width=1)
        d.line([(u, v - 20), (u, v + 20)], fill=(255, 60, 60), width=1)
        note = f"target at ({u:.0f},{v:.0f})  {row['off_axis_deg']:.1f} deg off axis"
    else:
        # arrow at the border pointing toward where the target is
        az, el = row['azimuth_deg'], row['elevation_deg']
        dx, dy = -math.sin(math.radians(az)), -math.sin(math.radians(el))
        n = math.hypot(dx, dy) or 1.0
        ex, ey = cx + dx / n * 200, cy + dy / n * 150
        d.line([(cx, cy), (ex, ey)], fill=(255, 60, 60), width=3)
        d.ellipse([ex - 6, ey - 6, ex + 6, ey + 6], fill=(255, 60, 60))
        note = f"TARGET OUT OF FRAME  {row['off_axis_deg']:.1f} deg off axis"
    d.rectangle([0, 0, V.CAM_W, 34], fill=(0, 0, 0))
    d.text((6, 4), f"{tag}  {row['waypoint_id']}  needs pitch {row['target_pitch_up_needed_deg']:+.0f} deg, "
                   f"joint {-math.degrees(row['gimbal_joint_rad_true']):+.0f} deg up", fill=(255, 255, 255))
    d.text((6, 18), note, fill=(255, 200, 80))
    return im


def run_one(run, plan, out, tag):
    os.makedirs(out, exist_ok=True)
    rows = measure(run, plan)
    for r in rows:
        f = os.path.join(run, 'camera', 'waypoints', r['waypoint_id'] + '.jpg')
        if os.path.exists(f):
            annotate(Image.open(f), r, tag).save(os.path.join(out, r['waypoint_id'] + f'_{tag}.jpg'), quality=88)
            r['frame'] = os.path.relpath(f, run)
    s = summarize(rows)
    json.dump({'run': os.path.basename(os.path.normpath(run)), 'summary': s, 'waypoints': rows},
              open(os.path.join(out, f'centering_{tag}.json'), 'w'), indent=1)
    return rows, s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run', nargs='?')
    ap.add_argument('plan')
    ap.add_argument('out')
    ap.add_argument('--label', default='run')
    ap.add_argument('--before')
    ap.add_argument('--after')
    a = ap.parse_args()
    if a.before and a.after:
        rb, sb = run_one(a.before, a.plan, a.out, 'before')
        ra, sa = run_one(a.after, a.plan, a.out, 'after')
        ids = [r['waypoint_id'] for r in ra if any(x['waypoint_id'] == r['waypoint_id'] for x in rb)]
        for wid in ids:
            fb = os.path.join(a.out, f'{wid}_before.jpg')
            fa = os.path.join(a.out, f'{wid}_after.jpg')
            if os.path.exists(fb) and os.path.exists(fa):
                b, c = Image.open(fb), Image.open(fa)
                sheet = Image.new('RGB', (b.width + c.width + 6, max(b.height, c.height)), (20, 20, 20))
                sheet.paste(b, (0, 0))
                sheet.paste(c, (b.width + 6, 0))
                sheet.save(os.path.join(a.out, f'{wid}_before_after.jpg'), quality=88)
        print(json.dumps({'before': sb, 'after': sa}, indent=1))
        print(f"{'waypoint':<10}{'needs':>7}{'BEFORE off-axis':>18}{'in frame':>10}{'AFTER off-axis':>17}{'in frame':>10}")
        bm = {r['waypoint_id']: r for r in rb}
        for r in ra:
            b = bm.get(r['waypoint_id'])
            if b:
                print(f"{r['waypoint_id']:<10}{r['target_pitch_up_needed_deg']:>+6.0f}°{b['off_axis_deg']:>16.1f}° {str(b['target_in_frame']):>9}"
                      f"{r['off_axis_deg']:>15.1f}° {str(r['target_in_frame']):>9}")
    else:
        rows, s = run_one(a.run, a.plan, a.out, a.label)
        print(json.dumps(s, indent=1))


if __name__ == '__main__':
    main()
