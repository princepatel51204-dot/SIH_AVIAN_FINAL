"""Top-down plan views of the column-orbit flight for the dashboard gallery (reads committed files only).
Run with the project venv: /home/prince/avian_rev_c/.venv/bin/python3 dashboard/figures/plot_columns_plan.py"""
import json, os, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
S = json.load(open(f'{ROOT}/gazebo/mission_follower/columns_flight_summary.json'))
coll = json.load(open(f'{ROOT}/scene/collision/avian_bridge_collision.json'))['primitives']
tr = np.array(S['track_xyz_1m']); vp = np.array(S['orbit_viewpoints_xy'])
OUT = f'{ROOT}/dashboard/assets'


def draw(ax, xlim, ylim, title):
    for p in coll:
        if p['kind'] in ('ground', 'water', 'train_car'):
            continue
        c = p['centre']
        if p['type'] == 'CYLINDER':
            ax.add_patch(Circle((c[0], c[1]), p['radius'], color='#6b7686', zorder=2))
        elif p['type'] == 'BOX' and abs(p.get('yaw', 0)) < 1e-3:
            hx, hy = p['half_extents'][:2]
            ax.add_patch(Rectangle((c[0] - hx, c[1] - hy), 2 * hx, 2 * hy, color='#c7cfda', alpha=.55, zorder=1, lw=0))
    ax.plot(tr[:, 0], tr[:, 1], '-', color='#1D4ED8', lw=0.8, alpha=.85, zorder=3, label='flown track (simulator truth, 1 m decimation)')
    ax.scatter(vp[:, 0], vp[:, 1], s=7, color='#15803D', zorder=4, label='orbit viewpoints in the plan')
    ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_aspect('equal'); ax.set_title(title, fontsize=11)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.legend(loc='upper right', fontsize=8)


m = S['mission']
f, a = plt.subplots(figsize=(14, 4.2), dpi=110)
draw(a, (-20, 385), (-40, 45), f"Column-orbit flight: {m['reached']}/{m['waypoints_in_plan']} viewpoints reached, {len(m['columns'])} columns, {m['contact_episodes']} contacts")
f.tight_layout(); f.savefig(f'{OUT}/columns_plan_overview.png'); plt.close(f)
f, a = plt.subplots(figsize=(9, 5.2), dpi=110)
draw(a, (78, 147), (-14, 14), 'Road piers RP03 (x = 90 m) and RP04 (x = 135 m): orbit viewpoints and flown track')
f.tight_layout(); f.savefig(f'{OUT}/columns_plan_rp0304.png'); plt.close(f)
print('wrote', OUT)
