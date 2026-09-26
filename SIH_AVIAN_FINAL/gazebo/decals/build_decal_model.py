#!/usr/bin/env python3
"""Build the decal override of the Gazebo defects model (VISUALISATION ONLY).

Writes gazebo/models_decals/avian_final_defects/: a copy of the original defects model in which the
chosen defects' flat-coloured 0.25 m spheres are replaced by textured decals rendered from the Blender
twin (gazebo/decals/render_decals.py). Every other sphere is kept unchanged. The mission world is not
edited: launch_sih_sitl.sh puts gazebo/models_decals first on GZ_SIM_RESOURCE_PATH only when
AVIAN_DECALS=1, so model://avian_final_defects resolves to this copy; default runs are unchanged.

Decal geometry, all visual-only (no <collision>; nothing here feeds navigation):
  * defect on a pier COLUMN (cylinder): the orthographic render is re-projected onto the cylinder as
    N_STRIPS thin vertical strips, strip k at angle theta_k around the column axis, textured with the
    image slice x = r*sin(theta - phi) (phi = the defect's normal direction), 12 mm off the surface
  * any other host (cap, deck, web): one flat thin box on the tangent plane, 12 mm off the surface
Positions and normals come from the ground-truth JSON; decal size from the render's crop width.

Usage: build_decal_model.py ID [ID ...]
"""
import json, math, os, re, shutil, sys
import numpy as np
from PIL import Image

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
SRC_MODEL = f'{ROOT}/gazebo/models/avian_final_defects'
TEX = f'{ROOT}/gazebo/models/avian_final_decals/materials/textures'
OUT = f'{ROOT}/gazebo/models_decals/avian_final_defects'
N_STRIPS = 8
MAX_HALF_ANGLE = math.radians(70)       # beyond this the orthographic render is too foreshortened
OFFSET = 0.012
THICK = 0.004

ids = sys.argv[1:]
meta = json.load(open(f'{TEX}/decals_meta.json'))
gt = {}
for f in ('AVIAN_defect_ground_truth_FINAL.json', 'AVIAN_metro_ground_truth_FINAL.json', 'AVIAN_steel_ground_truth_FINAL.json'):
    for d in json.load(open(f'{ROOT}/scene/{f}'))['defects']:
        gt[d['defect_id']] = d
col_json = json.load(open(f'{ROOT}/mission/columns_plan_report.json'))['inventory']
axes = {}
for sub in col_json:
    for name, xy in zip(sub['column_names'], sub['axes_xy']):
        axes[name] = (np.array(xy, float), sub['radius_m'])

if os.path.exists(OUT):
    shutil.rmtree(OUT)
os.makedirs(f'{OUT}/materials/textures')
sdf = open(f'{SRC_MODEL}/model.sdf').read()
cfg = open(f'{SRC_MODEL}/model.config').read()
open(f'{OUT}/model.config', 'w').write(cfg)

# drop the converted spheres
for did in ids:
    sdf, n = re.subn(r'\s*<visual name="%s">.*?</visual>' % re.escape(did), '', sdf, flags=re.S)
    assert n == 1, did


def rot_from_axes(x, y):
    x = x / np.linalg.norm(x); y = y - (y @ x) * x; y /= np.linalg.norm(y); z = np.cross(x, y)
    R = np.column_stack([x, y, z])
    roll = math.atan2(R[2, 1], R[2, 2]); pitch = -math.asin(max(-1, min(1, R[2, 0]))); yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def visual(name, pos, rpy, sy, sz, tex_rel):
    return f'''
      <visual name="{name}">
        <pose>{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f} {rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}</pose>
        <geometry><box><size>{THICK} {sy:.4f} {sz:.4f}</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal><albedo_map>model://avian_final_defects/materials/textures/{tex_rel}</albedo_map><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr></material>
      </visual>'''


added, report = '', {}
up = np.array([0.0, 0.0, 1.0])
for did in ids:
    d, m = gt[did], meta[did]
    img = Image.open(f"{TEX}/{m['file']}").convert('RGB')
    W = m['width_m']; px = img.width
    p = np.array(d['position_m'], float); n = np.array(d['surface_normal'], float); n /= np.linalg.norm(n)
    host = d.get('host_object', '')
    if host in axes and abs(n[2]) < 0.2:
        c, r = axes[host]
        phi = math.atan2(n[1], n[0])
        half = min(MAX_HALF_ANGLE, math.asin(min(1.0, (W / 2) / r)))
        edges = np.linspace(-half, half, N_STRIPS + 1)
        for k in range(N_STRIPS):
            a0, a1 = edges[k], edges[k + 1]; am = 0.5 * (a0 + a1)
            # image x (0..W) of the orthographic render for these angles; image right = increasing angle
            x0, x1 = (r * math.sin(a0) + W / 2) / W * px, (r * math.sin(a1) + W / 2) / W * px
            fn = f'{did}_s{k}.png'
            img.crop((int(math.floor(x0)), 0, int(math.ceil(x1)), px)).save(f'{OUT}/materials/textures/{fn}')
            th = phi + am; nr = np.array([math.cos(th), math.sin(th), 0.0])
            pos = np.array([c[0], c[1], p[2]]) + nr * (r + OFFSET)
            rpy = rot_from_axes(nr, np.cross(up, nr))
            added += visual(f'{did}_decal_s{k}', pos, rpy, r * (a1 - a0) * 1.03, W, fn)
        report[did] = {'mode': 'cylinder_strips', 'host': host, 'strips': N_STRIPS, 'arc_deg': round(math.degrees(2 * half), 1), 'height_m': round(W, 3)}
    else:
        fn = f'{did}.png'
        img.save(f'{OUT}/materials/textures/{fn}')
        yv = np.cross(up, n) if abs(n[2]) < 0.95 else np.array([0.0, 1.0, 0.0])
        rpy = rot_from_axes(n, yv)
        added += visual(f'{did}_decal', p + n * OFFSET, rpy, W, W, fn)
        report[did] = {'mode': 'flat', 'host': host, 'width_m': round(W, 3)}

sdf = sdf.replace('    </link>', added + '\n    </link>', 1)
open(f'{OUT}/model.sdf', 'w').write(sdf)
json.dump({'converted': report, 'note': 'visual-only decals; spheres of converted defects removed; all other spheres unchanged'},
          open(f'{OUT}/decals_built.json', 'w'), indent=1)
print(json.dumps(report, indent=1))
print('visuals in model:', sdf.count('<visual '))
