"""Offline checks for defect_register.py (no ROS graph, no simulator).

A known planar wall is painted with LiDAR-like stripes (15 rings 2 deg apart, 1 deg
azimuth spacing), the map is quantised to 0.25 m voxels exactly as map_viz_node does,
and detections are cast into it. Everything the checks compare against is the
synthetic scene's own geometry, defined in this file -- not the simulator.

Run:  python3 gazebo/mission_follower/viz/test_defect_register.py
"""
import json
import math
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import viz_frames as V            # noqa: E402
import defect_register as D       # noqa: E402

VOX = 0.25
fails = 0


def check(name, ok, detail=''):
    global fails
    fails += (not ok)
    print(('PASS ' if ok else 'FAIL ') + name + (f'  {detail}' if detail else ''))


def stripes_on_plane(origin, n, c, az_span=50, el_span=14):
    """LiDAR-style rays from `origin` (level, +x forward) hitting the plane (n, c)."""
    pts = []
    for el in np.deg2rad(np.arange(-el_span, el_span + 1, 2.0)):
        for az in np.deg2rad(np.arange(-az_span, az_span + 1, 1.0)):
            d = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
            den = d @ n
            if abs(den) < 1e-6:
                continue
            t = (c - origin) @ n / den
            if 0.5 < t < 29.5:
                pts.append(origin + d * t)
    return np.array(pts)


def keys_of(p):
    return np.unique(V.voxel_keys(p, VOX))


def cam(pos, gimbal=0.0):
    R = np.eye(3)                                    # base FLU == map ENU (drone facing +x)
    cp, cR = V.camera_pose_in_base(gimbal)
    return pos + R @ cp, R @ cR


def obs(o, Rc, bbox=(300, 220, 340, 260), fam='SPALL_DELAM', score=0.9, wp='W1', stamp=1.0):
    return {'family': fam, 'score': score, 'bbox': list(bbox), 'stamp': stamp, 'wp': wp, 'o': o, 'R_cam': Rc,
            'sigma_gimbal': D.SIGMA_GIMBAL_MEASURED, 'eph': 0.1, 'epv': 0.1}


pos = np.array([0.0, 0.0, 8.0])
o, Rc = cam(pos)
wall_x = 10.0 + o[0]
nx = np.array([1.0, 0.0, 0.0])
wall_pts = stripes_on_plane(pos, nx, np.array([wall_x, 0, 0]))
keys = keys_of(wall_pts)

# 1. ranging onto a wall straight ahead
loc = D.localize(keys, VOX, o, Rc @ V.pixel_ray(320, 240))
check('plane wall dead ahead is found', loc is not None)
check('plane_fit method chosen for a planar wall', loc['method'] == 'plane_fit', loc['method'])
check('range error < 0.10 m', abs(loc['point'][0] - wall_x) < 0.10, f"x {loc['point'][0]:.3f} vs {wall_x:.3f}")
check('surface normal recovered (|nx| > 0.98)', abs(loc['normal'][0]) > 0.98, str(np.round(loc['normal'], 3)))

# 2. size on a fronto-parallel wall: w = w_px * range / fx
reg = D.Register()
b = (270, 210, 370, 270)                                    # 100 x 60 px
reg.observe(obs(o, Rc, bbox=b), keys, VOX, 1.0)
c = reg.localized()[0]
w_exp, h_exp = 100 * 10.0 / V.CAM_FX, 60 * 10.0 / V.CAM_FY
check('size w x h matches range*px/fx on a fronto-parallel wall (3 %)',
      abs(c['size_w_m'] - w_exp) / w_exp < 0.03 and abs(c['size_h_m'] - h_exp) / h_exp < 0.03,
      f"{c['size_w_m']:.2f} x {c['size_h_m']:.2f} vs {w_exp:.2f} x {h_exp:.2f}")

# 3. oblique wall (normal 60 deg off the ray): plane size is ~2x the fronto-parallel guess
ang = math.radians(60)
n60 = np.array([math.cos(ang), math.sin(ang), 0.0])
p0 = np.array([wall_x, 0.0, 0.0])
keys60 = keys_of(stripes_on_plane(pos, n60, p0))
reg60 = D.Register()
reg60.observe(obs(o, Rc, bbox=b), keys60, VOX, 1.0)
c60 = reg60.localized()[0]
check('oblique wall: plane_fit used', c60['size_method'] == 'plane_fit', c60['size_method'])
check('oblique wall: width ~ 2x the fronto-parallel estimate (within 15 %)',
      abs(c60['size_w_m'] / (100 * c60['median_range_m'] / V.CAM_FX) - 2.0) < 0.3,
      f"ratio {c60['size_w_m'] / (100 * c60['median_range_m'] / V.CAM_FX):.2f}")

# 4. grazing surface: no plane range, size falls back and says so
ang = math.radians(80)
n80 = np.array([math.cos(ang), math.sin(ang), 0.0])
keys80 = keys_of(stripes_on_plane(pos, n80, np.array([wall_x, 0.0, 0.0])))
reg80 = D.Register()
ok = reg80.observe(obs(o, Rc), keys80, VOX, 1.0)
if reg80.localized():
    check('grazing surface (80 deg): does not claim plane_fit',
          reg80.localized()[0]['size_method'] == 'fronto_parallel', reg80.localized()[0]['size_method'])
else:
    check('grazing surface (80 deg): kept unlocalized rather than guessed', len(reg80.pending) == 1)

# 5. no return on the ray: unlocalized, retried, then localized when the map grows
o2, Rc2 = cam(pos, gimbal=-0.6)                              # camera tilted UP 34 deg: ray misses the striped band
reg2 = D.Register(retry_s=10.0)
res = reg2.observe(obs(o2, Rc2, stamp=5.0), keys, VOX, 5.0)
check('ray with no map return is NOT localized', res is False and len(reg2.localized()) == 0)
check('...and no position is invented for it', all('pos' not in c for c in reg2.clusters))
ray2 = Rc2 @ V.pixel_ray(320, 240)
ceiling = o2 + ray2 * 12.0
grown = np.unique(np.concatenate([keys, V.voxel_keys(ceiling + np.random.default_rng(0).normal(0, 0.15, (60, 3)), VOX)]))
reg2.retry(grown, VOX, 6.0)
check('retry succeeds once the map has a return there', len(reg2.localized()) == 1 and not reg2.pending,
      f'{len(reg2.localized())} localized, {len(reg2.pending)} pending')
reg3 = D.Register(retry_s=10.0)
reg3.observe(obs(o2, Rc2, stamp=5.0), keys, VOX, 5.0)
reg3.retry(keys, VOX, 30.0)
s = reg3.snapshot()['summary']
check('expired retry -> reported unlocalized, never localized', s['unlocalized_detections'] == 1 and s['distinct_localized_defects'] == 0, str(s))

# 6. clustering: repeat sightings collapse; classes and distant defects stay separate
rng = np.random.default_rng(3)
reg4 = D.Register()
for i in range(30):
    dx = rng.normal(0, 0.10, 2)
    bb = (300 + dx[0] * 20, 220 + dx[1] * 20, 340 + dx[0] * 20, 260 + dx[1] * 20)
    reg4.observe(obs(o, Rc, bbox=bb, wp=f'W{i // 10}', stamp=float(i), score=0.6 + 0.01 * i), keys, VOX, float(i))
check('30 raw detections of one defect -> 1 distinct defect', len(reg4.localized()) == 1, f'{len(reg4.localized())}')
c4 = reg4.localized()[0]
check('observation count, viewpoints and best confidence reported',
      c4['n_obs'] == 30 and c4['n_viewpoints'] == 3 and abs(c4['best_conf'] - 0.89) < 1e-6,
      f"n_obs {c4['n_obs']} viewpoints {c4['n_viewpoints']} best {c4['best_conf']:.2f}")
reg4.observe(obs(o, Rc, fam='CRACK', stamp=40.0), keys, VOX, 40.0)
check('same place, different class -> separate defect', len(reg4.localized()) == 2)
far = obs(o, Rc, bbox=(60, 220, 100, 260), stamp=41.0)          # ~ -5 m sideways on the wall
reg4.observe(far, keys, VOX, 41.0)
check('same class 4+ m away -> separate defect', len([c for c in reg4.localized() if c['fam'] == 'SPALL_DELAM']) == 2)
sm = reg4.snapshot()['summary']
check('raw detections and distinct defects are reported as different numbers',
      sm['raw_detections'] == 32 and sm['distinct_localized_defects'] == 3, str(sm))

# 7. uncertainty is reported, positive and grows with range
check('single-observation sigma reported and > 0.1 m floor', all(c['pos_sigma_m'] > 0.1 for c in reg4.localized()))
near_pos = np.array([0.0, 0.0, 8.0]); on, Rn = cam(near_pos)
keys_n = keys_of(stripes_on_plane(near_pos, nx, np.array([on[0] + 4.0, 0, 0])))
regn = D.Register(); regn.observe(obs(on, Rn), keys_n, VOX, 1.0)
check('sigma at 4 m < sigma at 10 m', regn.localized()[0]['pos_sigma_m'] < reg.localized()[0]['pos_sigma_m'],
      f"{regn.localized()[0]['pos_sigma_m']:.3f} < {reg.localized()[0]['pos_sigma_m']:.3f}")

# 8. output files
out = tempfile.mkdtemp(prefix='avian_reg_test_')
snap = reg4.write(out)
j = json.load(open(os.path.join(out, 'defect_register.json')))
rows = open(os.path.join(out, 'defect_register.csv')).read().strip().splitlines()
check('JSON register written with summary + defects', j['summary']['distinct_localized_defects'] == 3 and len(j['defects']) == 3)
check('CSV has header + one row per defect', len(rows) == 4 and rows[0].startswith('id,class,x_m,y_m,z_m'), rows[0])
check('register states its data source (no ground truth)', 'no ground truth' in j['source'])
print('\nRESULT:', 'ALL PASS' if not fails else f'{fails} FAILED')
sys.exit(1 if fails else 0)
