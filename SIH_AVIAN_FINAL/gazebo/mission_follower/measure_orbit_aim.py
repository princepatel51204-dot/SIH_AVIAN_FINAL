#!/usr/bin/env python3
"""Continuous camera-aim measurement over a column orbit (OFFLINE; uses simulator truth only to score a flight).

The per-waypoint checks (verify_aim_centering.py, analyse_columns_flight.py) look at the camera only when the drone
has SETTLED on a viewpoint. This script measures it THROUGHOUT the flight: every sample of the true pose/gimbal track
(pose_audit_track.csv, ~16 Hz, resampled to --hz) is assigned to the leg being flown (leg of waypoint i = the time
between the waypoint events i-1 and i in pose_audit.json), and for each sample it computes where the leg's target
column is relative to the camera:
  * target column = the nearest column axis of the pier/column the leg belongs to (plan column_id)
  * az_err_deg    = horizontal angle between the camera axis and the bearing to that column axis
  * el_err_deg    = vertical angle between the camera axis and the nearest point of the column axis segment
  * in_fov        = some part of the column axis segment (z_base..z_top) projects inside the image
                    (camera model per --cam: wide 640x480 HFOV 80 deg, or narrow 640x480 HFOV --narrow-hfov deg)
Results are reported per leg type (ring / link / transit / vertical / dwell) and per selected columns.
Optionally extracts recorded frames from mid-transit (the middle of ring legs) with the column axis drawn.

Usage: measure_orbit_aim.py <run_dir> [--columns RP04] [--hz 10] [--frames N] [--json out.json] [--outdir dir]
                            [--cam wide|narrow --narrow-hfov 20]
"""
import argparse, bisect, collections, csv, json, math, os, subprocess, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(HERE, 'viz'))
import viz_frames as V  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('run'); ap.add_argument('--columns', default=''); ap.add_argument('--hz', type=float, default=10.0)
ap.add_argument('--frames', type=int, default=0); ap.add_argument('--json'); ap.add_argument('--outdir')
ap.add_argument('--cam', default='wide', choices=['wide', 'narrow']); ap.add_argument('--narrow-hfov', type=float, default=20.0)
a = ap.parse_args()

run = a.run.rstrip('/')
L = json.load(open(f'{run}/mission_log.json'))
plan_path = os.path.join(ROOT, L['plan'].split('SIH_AVIAN_FINAL/')[-1])
plan = {w['waypoint_id']: w for w in json.load(open(plan_path))['waypoints']}
inv = {s['id']: s for s in json.load(open(f'{ROOT}/mission/columns_plan_report.json'))['inventory']}
audit = [r for r in json.load(open(f'{run}/pose_audit.json'))['records'] if r.get('result') == 'reached']
want = set(filter(None, a.columns.split(',')))

T, X, Q, J = [], [], [], []
for r in csv.DictReader(open(f'{run}/pose_audit_track.csv')):
    T.append(float(r['t_sim'])); X.append([float(r['x']), float(r['y']), float(r['z'])])
    Q.append([float(r['qw']), float(r['qx']), float(r['qy']), float(r['qz'])]); J.append(float(r['gimbal_joint_rad']))
T, X, Q, J = map(np.array, (T, X, Q, J))

if a.cam == 'wide':
    W_, H_, FX = V.CAM_W, V.CAM_H, V.CAM_FX
    cam_off = None
else:
    W_, H_ = 640, 480
    FX = (W_ / 2) / math.tan(math.radians(a.narrow_hfov) / 2)
CX, CY = W_ / 2, H_ / 2
hfov, vfov = 2 * math.degrees(math.atan(CX / FX)), 2 * math.degrees(math.atan(CY / FX))


def leg_type(w):
    if w.get('dwell'):
        return 'dwell'
    m = w.get('route_method', '')
    if m == 'ring':
        return 'ring'
    if m == 'vertical':
        return 'vertical'
    if m.startswith('link'):
        return 'link'
    return 'transit'


def sample(t):
    i = min(max(bisect.bisect_left(T, t), 1), len(T) - 1)
    t0, t1 = T[i - 1], T[i]; u = 0.0 if t1 <= t0 else min(max((t - t0) / (t1 - t0), 0), 1)
    k = i - 1 if u < 0.5 else i
    return X[i - 1] * (1 - u) + X[i] * u, Q[k], J[k]


def evaluate(p, q, j, sid):
    R = V.rot_from_quat_wxyz(q); cp, cR = V.camera_pose_in_base(j)
    cam, Rc = p + R @ cp, R @ cR
    sub = inv[sid]; axes = np.array(sub['axes_xy'])
    ax = axes[np.argmin(np.linalg.norm(axes - cam[:2], axis=1))]
    f = Rc[:, 0]
    bear = math.atan2(ax[1] - cam[1], ax[0] - cam[0]); head = math.atan2(f[1], f[0])
    az = math.degrees((head - bear + math.pi) % (2 * math.pi) - math.pi)
    zc = min(max(cam[2], sub['z_base']), sub['z_top'])
    v = np.array([ax[0], ax[1], zc]) - cam
    el = math.degrees(math.atan2(f[2], math.hypot(f[0], f[1])) - math.atan2(v[2], math.hypot(v[0], v[1])))
    inside = False
    for z in np.linspace(sub['z_base'], sub['z_top'], 25):
        c = Rc.T @ (np.array([ax[0], ax[1], z]) - cam)
        if c[0] > 0.05:
            uu, vv = CX - FX * c[1] / c[0], CY - FX * c[2] / c[0]
            if 0 <= uu < W_ and 0 <= vv < H_:
                inside = True; break
    return az, el, inside, cam, Rc, ax


rows = []
for prev, cur in zip(audit[:-1], audit[1:]):
    w = plan.get(cur['waypoint_id'])
    if not w or w.get('column_id') is None:
        continue
    if want and w['column_id'] not in want:
        continue
    t0, t1 = prev['t_sim'], cur['t_sim']
    for t in np.arange(t0, t1, 1.0 / a.hz):
        p, q, j = sample(t)
        az, el, ins, *_ = evaluate(p, q, j, w['column_id'])
        rows.append({'t': round(float(t), 3), 'wp': cur['waypoint_id'], 'col': w['column_id'], 'type': leg_type(w),
                     'u': round(float((t - t0) / max(t1 - t0, 1e-9)), 3), 'az': az, 'el': el, 'in_fov': ins})


def stats(sel):
    if not sel:
        return None
    az = np.abs([r['az'] for r in sel]); el = np.abs([r['el'] for r in sel])
    return {'samples': len(sel), 'az_err_deg': {'mean': round(float(az.mean()), 2), 'p95': round(float(np.percentile(az, 95)), 2), 'max': round(float(az.max()), 2)},
            'el_err_deg': {'mean': round(float(el.mean()), 2), 'p95': round(float(np.percentile(el, 95)), 2), 'max': round(float(el.max()), 2)},
            'column_in_fov_pct': round(100 * sum(r['in_fov'] for r in sel) / len(sel), 1),
            'column_outside_fov_pct': round(100 * sum(not r['in_fov'] for r in sel) / len(sel), 1)}


res = {'run': os.path.basename(run), 'plan': os.path.relpath(plan_path, ROOT), 'camera': a.cam, 'hfov_deg': round(hfov, 2), 'vfov_deg': round(vfov, 2),
       'hz': a.hz, 'columns': sorted(want) or 'all', 'by_leg_type': {k: stats([r for r in rows if r['type'] == k]) for k in ('ring', 'link', 'transit', 'vertical', 'dwell')},
       'ring_mid_transit_only': stats([r for r in rows if r['type'] == 'ring' and 0.2 <= r['u'] <= 0.8]),
       'all_orbit_legs': stats([r for r in rows if r['type'] in ('ring', 'vertical', 'dwell')])}
print(json.dumps(res, indent=1))
if a.json:
    json.dump(res, open(a.json, 'w'), indent=1)

if a.frames and a.outdir:
    os.makedirs(a.outdir, exist_ok=True)
    fr = [(float(r['stamp_s']), int(r['frame'])) for r in csv.DictReader(open(f'{run}/camera/frames.csv'))]
    fs = [x[0] for x in fr]
    mids = [r for r in rows if r['type'] == 'ring' and abs(r['u'] - 0.5) < 0.06]
    seen, picks = set(), []
    for r in mids:
        if r['wp'] not in seen:
            seen.add(r['wp']); picks.append(r)
    picks = picks[:: max(1, len(picks) // a.frames)][:a.frames]
    from PIL import Image, ImageDraw
    for r in picks:
        k = min(max(bisect.bisect_left(fs, r['t']), 0), len(fs) - 1)
        out = f"{a.outdir}/{r['wp']}_mid.png"
        subprocess.run(f'ffmpeg -y -v error -i {run}/camera/front_camera.mp4 -vf "select=eq(n\\,{fr[k][1]})" -vsync 0 -frames:v 1 {out}', shell=True)
        if os.path.exists(out):
            im = Image.open(out).convert('RGB'); d = ImageDraw.Draw(im)
            d.rectangle([0, 0, im.width, 16], fill=(0, 0, 0))
            d.text((4, 2), f"{r['wp']} mid-leg t={r['t']:.1f}s  column az err {r['az']:+.0f} deg  el err {r['el']:+.0f} deg  in FOV: {r['in_fov']}", fill=(255, 255, 255))
            im.save(out)
    print('frames:', [p['wp'] for p in picks])
