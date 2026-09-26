#!/usr/bin/env python3
"""SIH_AVIAN_FINAL -- per-column orbital inspection plan.  OFFLINE PLANNING ONLY.

Reads the digital twin's collision JSON (+ the Gazebo-only visual boxes) through covlib
and writes a plan JSON in the follower's existing (v5) schema. Nothing here is imported
by the follower; at flight time the drone still navigates and avoids on sensed returns
only.

Pattern: STACKED HORIZONTAL RINGS joined by VERTICAL transitions (every route piece is
flat or vertical, so it stays inside the LiDAR ring / up-down cone sensing argument).
  per subject (a pier = twin road columns + cap, or one metro column + pier head):
    ring at z1 (full loop or the free arcs) -> vertical descent -> ring at z2 (reverse
    direction) -> ... -> lowest ring;  yaw always faces the subject's spine (inward);
    one gimbal pitch per ring, chosen to maximise new surface coverage (real camera model).

Constants that MUST NOT change (same as the follower / v5): 3.0 m sensed ring, 3.5 m planned
clearance (airframe base_link AND camera pivot, to every solid incl. the ground: z >= 3.5),
1.9 m/s cruise, flat (<=12 deg) or vertical route pieces only.

Usage: python3 plan_columns.py [--subjects RP03,MC06] [--out ...] [--no-route]
"""
import argparse
import json
import math
import os
import re
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'mission_follower'))
import covlib as C  # noqa: E402
import numpy as np  # noqa: E402
import plan_v5 as PV  # noqa: E402

PLAN_CLEARANCE_M = 3.5
Z_MIN = 3.6                       # lowest ring: 3.5 m AGL + 0.1 m margin
STANDOFF_M = 4.0                  # base_link -> column surface. The camera pivot sits 0.35 m nearer,
                                  # so pivot-to-surface = 3.65 m >= 3.5 m as required.
PIVOT = np.array([0.35, 0.0, 0.05])
S_STEP_M = 3.8                    # viewpoint spacing along the ring (~45 deg on a 4.85 m radius)
VIA_STEP_M = 1.3                  # pass-through points along a ring (keeps chord clearance >= 3.5 m)
DENSE_M = 0.25                    # sampling for blocked-arc analysis and clearance checks
OVERLAP_V = 0.40                  # vertical footprint overlap between rings
MIN_FREE_FRAC_TOP = 0.20          # highest ring altitude: at least this share of the ring must be free (partial rings allowed)
PITCH_CANDS = np.radians(np.arange(-60, 76, 5))
CRUISE = 1.9
S_PER_WAYPOINT = 3.9              # measured in full_pass_05: (9375 s - 9162 m/1.9) / 1161 waypoints
DWELLS = True                     # look-up dwells on road piers (see plan_dwells)
DWELL_PITCHES = np.radians(np.arange(-60, 91, 10))
DWELL_GAIN_FRAC = 0.95            # stop when 95 % of the reachable cap/head gain is captured
DWELL_S = 3.0                     # ESTIMATED sim seconds per dwell (gimbal settle 1.0 + hold 1.0 + margin);
                                  # not measured until a plan with dwells is flown
HOME_AIR = PV.HOME_AIR


def log(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


# ----------------------------------------------------------------- perimeter --
class Capsule:
    """Offset curve at distance `d` from a spine segment A-B thickened by `rad` (rad = column
    radius). A == B gives a circle. Also used for a rectangle: see RoundedRect."""
    def __init__(self, A, B, rad, d):
        self.A, self.B = np.asarray(A, float), np.asarray(B, float)
        self.R = rad + d
        v = self.B - self.A
        self.Ls = float(np.linalg.norm(v))
        self.u = v / self.Ls if self.Ls > 1e-9 else np.array([1.0, 0.0])
        self.n = np.array([-self.u[1], self.u[0]])
        self.L = 2 * self.Ls + 2 * math.pi * self.R
        self.centre = (self.A + self.B) / 2

    def point(self, s):
        s = np.mod(np.atleast_1d(s), self.L)
        out = np.zeros((len(s), 2))
        a1, a2, a3 = self.Ls, self.Ls + math.pi * self.R, 2 * self.Ls + math.pi * self.R
        for i, t in enumerate(s):
            if t < a1:
                out[i] = self.A + self.n * self.R + self.u * t
            elif t < a2:
                th = (t - a1) / self.R
                out[i] = self.B + self.R * (math.cos(th) * self.n + math.sin(th) * self.u)
            elif t < a3:
                out[i] = self.B - self.n * self.R - self.u * (t - a2)
            else:
                th = (t - a3) / self.R
                out[i] = self.A + self.R * (-math.cos(th) * self.n - math.sin(th) * self.u)
        return out

    def inward(self, xy):
        """Nearest point of the spine (what the camera faces)."""
        xy = np.atleast_2d(xy)
        if self.Ls < 1e-9:
            return np.tile(self.A, (len(xy), 1))
        t = np.clip((xy - self.A) @ self.u, 0.0, self.Ls)
        return self.A + t[:, None] * self.u

    def aim(self, xy):
        """Where the camera looks: the NEAREST COLUMN AXIS (A or B). For a single column A == B, so
        this is the same point as inward(). For a two-column pier, aiming at the spine point
        (inward) made broadside viewpoints look through the gap between the columns (measured in
        flight: 6 of 25 frames hit the abutment / a footing / the deck instead of a column)."""
        xy = np.atleast_2d(xy)
        if self.Ls < 1e-9:
            return np.tile(self.A, (len(xy), 1))
        dA = np.linalg.norm(xy - self.A, axis=1)
        dB = np.linalg.norm(xy - self.B, axis=1)
        return np.where((dA <= dB)[:, None], self.A[None, :], self.B[None, :])


class RoundedRect:
    """Offset at distance d around a rectangular column (half extents hx,hy, yaw). No such column
    exists in this scene (every pier column is a cylinder); kept so a box column is handled."""
    def __init__(self, c, hx, hy, yaw, d):
        self.c, self.hx, self.hy, self.yaw, self.d = np.asarray(c, float), hx, hy, yaw, d
        self.Ls = 0.0
        self.L = 4 * (hx + hy) + 2 * math.pi * d
        self.centre = self.c
        self.R = d + math.hypot(hx, hy)

    def _loc(self, p):
        ca, sa = math.cos(-self.yaw), math.sin(-self.yaw)
        q = np.atleast_2d(p) - self.c
        return np.stack([q[:, 0] * ca - q[:, 1] * sa, q[:, 0] * sa + q[:, 1] * ca], 1)

    def _glob(self, q):
        ca, sa = math.cos(self.yaw), math.sin(self.yaw)
        return np.stack([q[:, 0] * ca - q[:, 1] * sa, q[:, 0] * sa + q[:, 1] * ca], 1) + self.c

    def point(self, s):
        s = np.mod(np.atleast_1d(s), self.L)
        hx, hy, d = self.hx, self.hy, self.d
        segs = [(2 * hy, 'E'), (math.pi * d / 2, 'NE'), (2 * hx, 'N'), (math.pi * d / 2, 'NW'),
                (2 * hy, 'W'), (math.pi * d / 2, 'SW'), (2 * hx, 'S'), (math.pi * d / 2, 'SE')]
        out = []
        for t in s:
            for ln, nm in segs:
                if t < ln:
                    break
                t -= ln
            f = t / ln if ln > 0 else 0
            th = t / d if nm in ('NE', 'NW', 'SW', 'SE') else 0
            p = {'E': (hx + d, -hy + t), 'N': (hx - t, hy + d), 'W': (-hx - d, hy - t), 'S': (-hx + t, -hy - d),
                 'NE': (hx + d * math.cos(th), hy + d * math.sin(th)),
                 'NW': (-hx - d * math.sin(th), hy + d * math.cos(th)),
                 'SW': (-hx - d * math.cos(th), -hy - d * math.sin(th)),
                 'SE': (hx + d * math.sin(th), -hy - d * math.cos(th))}[nm]
            out.append(p)
        return self._glob(np.array(out))

    def inward(self, xy):
        q = self._loc(xy)
        q = np.stack([np.clip(q[:, 0], -self.hx, self.hx), np.clip(q[:, 1], -self.hy, self.hy)], 1)
        return self._glob(q)

    def aim(self, xy):
        return self.inward(xy)      # a single column: nearest point of the column


# ------------------------------------------------------------------ inventory --
def inventory(S):
    rows = S.rows
    cyl_kinds = {}
    for r in rows:
        if r['type'] == 'CYLINDER':
            cyl_kinds[r['kind']] = cyl_kinds.get(r['kind'], 0) + 1
    cols, caps, foots = {}, {}, {}
    for k, r in enumerate(rows):
        m = re.match(r'(BR|MB)_PIER_(COL|CAP|HEAD|FOOTING)_(\d+)(?:_(\d))?$', r['name'])
        if not m or r['kind'] not in ('pier_column', 'pier_cap', 'pier_footing'):
            continue
        fam, what, n, sub = m.group(1), m.group(2), int(m.group(3)), m.group(4)
        key = ('RP' if fam == 'BR' else 'MC', n)
        {'COL': cols, 'CAP': caps, 'HEAD': caps, 'FOOTING': foots}[what].setdefault(key, []).append(k)
    subjects = []
    for key in sorted(cols, key=lambda kk: (kk[0] != 'RP', kk[1])):
        ck = cols[key]
        cyl = [rows[k] for k in ck]
        z0 = [r['centre'][2] - (r['half_height'] if r['type'] == 'CYLINDER' else r['half_extents'][2]) for r in cyl]
        z1 = [r['centre'][2] + (r['half_height'] if r['type'] == 'CYLINDER' else r['half_extents'][2]) for r in cyl]
        pts = np.array([r['centre'][:2] for r in cyl])
        if cyl[0]['type'] == 'CYLINDER':
            rad = cyl[0]['radius']
            shape = 'circular'
        else:
            hx, hy = cyl[0]['half_extents'][:2]
            rad = math.hypot(hx, hy)          # half-diagonal
            shape = 'rectangular'
        sid = f'{key[0]}{key[1]:02d}'
        subjects.append({'id': sid, 'kind': 'road_pier_pair' if key[0] == 'RP' else 'metro_column',
                         'columns': ck, 'column_names': [rows[k]['name'] for k in ck], 'shape': shape,
                         'radius_m': rad, 'axes_xy': pts.tolist(), 'z_base': float(min(z0)), 'z_top': float(max(z1)),
                         'caps': caps.get(key, []), 'footings': foots.get(key, []),
                         'spine': [pts[0].tolist(), pts[-1].tolist()],
                         'half_extents': cyl[0].get('half_extents'), 'yaw': cyl[0].get('yaw', 0.0),
                         'centre_xy': pts.mean(0).tolist(), 'cyl_kinds_in_scene': cyl_kinds})
    return subjects


# -------------------------------------------------------------- surface patches --
def subject_patches(S, sub):
    P = {k: [] for k in ('p', 'n', 'area', 'solid', 'face')}
    for k in sub['columns'] + sub['caps'] + sub['footings']:
        r = S.rows[k]
        step = 0.25 if k in sub['columns'] else 0.5
        pt = C.build_patches(types.SimpleNamespace(rows=[r]), step)
        pt['solid'] = np.full(len(pt['p']), k, np.int64)
        for key in P:
            P[key].append(pt[key])
    pt = {k: np.concatenate(v) for k, v in P.items()}
    reason, _ = C.classify_exclusions(S, pt)
    pt['reason'] = reason
    pt['kind'] = np.array([S.kinds[k] for k in pt['solid']])
    return pt


# ------------------------------------------------------------------ feasibility --
class Feas:
    def __init__(self, S):
        self.S = S
        _, self.reach = PV.free_space(S)

    def ok(self, xy, z, yaw):
        """Boolean per point: base_link and camera pivot clear >= 3.5 m of every solid and the ground
        (z >= 3.5). Connectivity to home is not tested here (the 1 m reach grid rounds a point that sits
        exactly at 3.5 m onto a node that fails); the router proves it when it links the waypoints."""
        xy = np.atleast_2d(xy)
        base = np.column_stack([xy, np.full(len(xy), z)])
        piv = base + np.column_stack([np.cos(yaw) * PIVOT[0], np.sin(yaw) * PIVOT[0],
                                      np.full(len(xy), PIVOT[2])])
        okb = C.fast_clearance(self.S, base) >= PLAN_CLEARANCE_M - 1e-9
        okp = C.fast_clearance(self.S, piv) >= PLAN_CLEARANCE_M - 1e-9
        return okb & okp

    def blockers(self, pt):
        """Name of the solid (or 'ground') that limits clearance at pt."""
        S = self.S
        best, name = pt[2], 'ground'
        for k in range(len(S.rows)):
            d = C._sdf(pt[0], pt[1], pt[2], int(S.typ[k]), S.c[k, 0], S.c[k, 1], S.c[k, 2],
                       S.h[k, 0], S.h[k, 1], S.h[k, 2], S.cyaw[k], S.syaw[k])
            if d < best:
                best, name = d, S.names[k]
        return name, float(best)


def cam_pose(base, yaw, pitch_up):
    c, s = math.cos(yaw), math.sin(yaw)
    cam = np.asarray(base) + np.array([c * PIVOT[0], s * PIVOT[0], PIVOT[2]])
    return cam, C.cam_axes(yaw, pitch_up)


# ----------------------------------------------------------------- ring design --
def design_subject(S, sub, feas, patches, log_=log):
    if sub['shape'] == 'circular':
        sh = Capsule(sub['spine'][0], sub['spine'][1], sub['radius_m'], STANDOFF_M)
    else:                                   # rectangular column: rounded-rectangle offset (none exist in this scene)
        sh = RoundedRect(sub['centre_xy'], sub['half_extents'][0], sub['half_extents'][1], sub['yaw'], STANDOFF_M)
    M = max(6, int(round(sh.L / S_STEP_M)))
    s_grid = np.arange(M) * sh.L / M
    P = sh.point(s_grid)
    yaw = np.arctan2(*(sh.aim(P) - P)[:, ::-1].T)
    nd = int(math.ceil(sh.L / DENSE_M))
    s_dense = np.arange(nd) * sh.L / nd
    Pd = sh.point(s_dense)
    yaw_d = np.arctan2(*(sh.aim(Pd) - Pd)[:, ::-1].T)

    def free_dense(z):
        return np.array([feas.ok(Pd[i:i + 1], z, yaw_d[i])[0] for i in range(nd)])

    def free_grid(z):
        return np.array([feas.ok(P[i:i + 1], z, yaw[i])[0] for i in range(M)])

    zs = np.round(np.arange(Z_MIN, sub['z_top'] + 1.0, 0.2), 2)
    frac = np.array([free_grid(z).mean() for z in zs])
    good = zs[frac >= MIN_FREE_FRAC_TOP]
    info = {'id': sub['id'], 'perimeter_m': sh.L, 'orbit_radius_m': sh.R, 'M': M}
    if not len(good):
        info['status'] = 'NO_FEASIBLE_RING'
        info['reason'] = 'no altitude has >= 20 % of the orbit clear at 3.5 m'
        info['free_fraction_by_z'] = {float(z): float(f) for z, f in zip(zs, frac)}
        return info, []
    z_top = float(good.max())
    d_cam = STANDOFF_M - PIVOT[0]
    H = 2 * d_cam * C.TAN_V
    dz = H * (1 - OVERLAP_V)
    rings_z = [z_top]
    while rings_z[-1] - dz >= Z_MIN - 1e-6:
        rings_z.append(round(rings_z[-1] - dz, 2))
    if rings_z[-1] - Z_MIN > 0.5 * dz:
        rings_z.append(Z_MIN)
    info.update({'status': 'OK', 'z_top_ring': z_top, 'vertical_footprint_m': H, 'ring_spacing_m': dz,
                 'overlap_v': OVERLAP_V, 'ring_z': rings_z})

    K = len(rings_z)
    z_lo_s, z_hi_s = 0.0, sub['z_top']            # bands run from the ground (footing) to the column top
    band_c = [z_hi_s - (k + 0.5) * (z_hi_s - z_lo_s) / K for k in range(K)]
    vis_mask = patches['reason'] == 0
    V = C.Visibility(S, patches, vis_mask)
    area = patches['area']
    covered = np.zeros(len(area), bool)
    rings = []
    for kr, zr in enumerate(rings_z):
        fg, fd = free_grid(zr), free_dense(zr)
        # a viewpoint pair is connected when every dense sample between them is free
        conn = np.zeros(M, bool)             # conn[i]: i -> i+1 free
        for i in range(M):
            a, b = int(round(s_grid[i] / sh.L * nd)), int(round(s_grid[(i + 1) % M] / sh.L * nd))
            idx = np.arange(a, b + 1) if b > a else np.concatenate([np.arange(a, nd), np.arange(0, b + 1)])
            conn[i] = bool(fd[idx % nd].all())
        # blocked arcs from the dense mask (cyclic runs of ~free)
        blocked = []
        if not fd.all():
            b = ~fd
            start = next(i for i in range(nd) if b[i] and not b[i - 1]) if not b.all() else 0
            i, seen = start, 0
            while seen < nd:
                if b[i % nd]:
                    j = i
                    while b[j % nd] and (j - i) < nd:
                        j += 1
                    mid = (i + j - 1) // 2 % nd
                    zmid = zr
                    who, clr = feas.blockers(np.array([Pd[mid][0], Pd[mid][1], zmid]))
                    ang = lambda idx_: math.degrees(math.atan2(Pd[idx_ % nd][1] - sh.centre[1],
                                                                Pd[idx_ % nd][0] - sh.centre[0]))
                    blocked.append({'from_bearing_deg': round(ang(i), 1), 'to_bearing_deg': round(ang(j - 1), 1),
                                    'length_m': round((j - i) * sh.L / nd, 2), 'limited_by': who,
                                    'clearance_there_m': round(clr, 2)})
                    seen += j - i
                    i = j
                else:
                    i += 1
                    seen += 1
        # best pitch for this ring: maximise newly covered area
        best = (-1.0, 0.0, None)
        fidx = np.flatnonzero(fg)
        if len(fidx):
            cam_sets = {}
            d_aim = float(np.mean(np.linalg.norm(sh.aim(P[fidx]) - P[fidx], axis=1))) - sub['radius_m'] - PIVOT[0]
            aim = math.atan2(band_c[kr] - zr, d_aim)      # = d_cam for a single column (4.0 - 0.35)
            for th in np.clip(aim + np.radians(np.arange(-35, 36, 5)), -math.pi / 2, math.pi / 2):
                u = np.zeros(0, np.int64)
                sets = []
                for i in fidx:
                    base = np.array([P[i][0], P[i][1], zr])
                    cam, (f, l, up) = cam_pose(base, yaw[i], th)
                    sets.append(V.visible(cam, f, l, up))
                union = np.unique(np.concatenate(sets)) if sets else u
                gain = float(area[union][~covered[union]].sum())
                cam_sets[th] = (gain, union)
                if gain > best[0]:
                    best = (gain, th, union)
            covered[best[2]] = True
        rings.append({'z': zr, 'free_grid': fg, 'conn': conn, 'blocked_arcs': blocked, 'pitch': float(best[1]),
                      'gain_m2': max(best[0], 0.0), 'n_free': int(fg.sum())})
    dropped = [r for r in rings if r['n_free'] == 0]
    rings = [r for r in rings if r['n_free'] > 0]
    info['dropped_rings_no_free_viewpoint'] = [{'z': r['z'], 'blocked_arcs': r['blocked_arcs']} for r in dropped]
    info['ring_z'] = [r['z'] for r in rings]
    info['rings'] = [{'z': r['z'], 'free_viewpoints': r['n_free'], 'of': M, 'pitch_deg': round(math.degrees(r['pitch']), 1),
                      'blocked_arcs': r['blocked_arcs']} for r in rings]
    shape = {'sh': sh, 'P': P, 'yaw': yaw, 's_grid': s_grid, 'M': M, 'nd': nd, 'Pd': Pd}
    return info, (rings, shape)


def ring_order(ring, prev_idx, prev_dir, M):
    """Visit order over a ring's free viewpoints. Returns list of (index) and the direction used."""
    free, conn = ring['free_grid'], ring['conn']
    if not free.any():
        return [], prev_dir
    if free.all() and conn.all():
        s = prev_idx if (prev_idx is not None and free[prev_idx]) else 0
        d = -prev_dir if prev_dir else 1
        return [(s + d * j) % M for j in range(M)], d
    # partial ring: arcs = cyclic runs of free viewpoints linked by conn
    arcs, seen = [], np.zeros(M, bool)
    for st in range(M):
        if free[st] and not seen[st] and not (free[st - 1] and conn[st - 1]):
            run, i = [st], st
            seen[st] = True
            while conn[i] and free[(i + 1) % M] and not seen[(i + 1) % M]:
                i = (i + 1) % M
                run.append(i)
                seen[i] = True
            arcs.append(run)
    for st in range(M):                        # fully closed free loop with a broken link handled above
        if free[st] and not seen[st]:
            arcs.append([st])
            seen[st] = True
    d = -prev_dir if prev_dir else 1
    order = []
    cur = prev_idx if prev_idx is not None else arcs[0][0]
    remaining = [list(a) for a in arcs]
    while remaining:
        def dist(a):
            return min(min((cur - a[0]) % M, (a[0] - cur) % M), min((cur - a[-1]) % M, (a[-1] - cur) % M))
        a = min(remaining, key=dist)
        remaining.remove(a)
        if min((cur - a[-1]) % M, (a[-1] - cur) % M) < min((cur - a[0]) % M, (a[0] - cur) % M):
            a = a[::-1]
        order += a
        cur = a[-1]
    return order, d


def ring_vias(sh, s_a, s_b, d, z):
    """Pass-through points along the ring from arclength s_a to s_b in direction d (excluding both ends)."""
    span = (s_b - s_a) * d % sh.L
    n = max(1, int(math.ceil(span / VIA_STEP_M)))
    ss = s_a + d * span * np.arange(1, n) / n
    return [[float(p[0]), float(p[1]), float(z)] for p in sh.point(ss)]


def plan_dwells(sub, pat, V, sub_wps):
    """Extra gimbal-pitch dwells at EXISTING viewpoints to see the pier cap / head.

    Each ring uses one pitch, chosen for the shaft; the cap's side faces sit above the top ring and
    need a look-up angle. Positions and headings are unchanged, so base_link and camera-pivot
    clearance, routing and the flat/vertical rule are exactly those of the parent viewpoint; only
    the gimbal pitch differs (+/-90 deg joint limit is not approached). Greedy set cover on cap/head
    area over (viewpoint, pitch) candidates until DWELL_GAIN_FRAC of the reachable gain is captured.
    Returns [(index into the reachable sub_wps, pitch_rad)]."""
    vis = pat['reason'] == 0
    A = pat['area']
    cap = np.isin(pat['solid'], sub['caps']) & vis
    if not cap.any():
        return [], sub_wps

    def vmask(cam, f, l, u):
        m = np.zeros(len(A), bool)
        m[V.visible(cam, f, l, u)] = True
        return m

    reach = [w for w in sub_wps if w['reachable_in_plan']]
    seen = np.zeros(len(A), bool)
    for w in reach:
        seen |= vmask(np.array(w['camera_world_m']), *C.cam_axes(w['heading_rad'], w['gimbal_pitch_rad']))
    cands, bound = [], seen.copy()
    for wi, w in enumerate(reach):
        for th in DWELL_PITCHES:
            cam, (f, l, u) = cam_pose(np.array(w['position_m']), w['heading_rad'], th)
            m = vmask(cam, f, l, u)
            cands.append((wi, float(th), m))
            bound |= m

    def capcov(m):
        return 100.0 * float(A[cap & m].sum()) / float(A[cap].sum())
    target = capcov(seen) + DWELL_GAIN_FRAC * (capcov(bound) - capcov(seen))
    chosen, cur = [], seen.copy()
    while capcov(cur) < target - 1e-9:
        best = max(cands, key=lambda c: float(A[cap & c[2] & ~cur].sum()))
        if float(A[cap & best[2] & ~cur].sum()) <= 1e-6:
            break
        chosen.append((best[0], best[1]))
        cur |= best[2]
    return chosen, reach


def insert_dwells(sub_wps, chosen, reach):
    """Place each dwell immediately after its parent viewpoint (same position, zero-length leg)."""
    after = {}
    for wi, th in chosen:
        after.setdefault(id(reach[wi]), []).append(th)
    out = []
    for w in sub_wps:
        out.append(w)
        for k, th in enumerate(sorted(after.get(id(w), [])), 1):
            d = dict(w)
            pos = np.array(w['position_m'])
            dcam = math.hypot(w['target_m'][0] - pos[0], w['target_m'][1] - pos[1])
            d.update({'waypoint_id': f"{w['waypoint_id']}_D{k}", 'gimbal_pitch_rad': round(float(th), 4),
                      'target_m': [w['target_m'][0], w['target_m'][1], round(float(pos[2] + dcam * math.tan(th)), 3)],
                      'route_in': [], 'route_method': 'dwell', 'route_length_m': 0.0,
                      'dwell': True, 'dwell_of': w['waypoint_id']})
            out.append(d)
    return out


def build(designs, S, feas, do_route=True):
    import plan_gazebo_mission as PG
    PG.clearance = lambda pts, prims: C.fast_clearance(S, pts)      # same exact clearance the v5 router uses
    PG.TIER3_CLEARANCE_M = PLAN_CLEARANCE_M                         # no 3.2 m relaxation anywhere
    grid = PG.Grid(None)
    order = [d for d in designs if d[2].get('status') == 'OK']
    order.sort(key=lambda d: (d[0]['id'][:2] != 'RP', d[0]['centre_xy'][0] * (1 if d[0]['id'][:2] == 'RP' else -1)))
    wps, prev, total = [], HOME_AIR.tolist(), PG.TAKEOFF_ALT_M
    raw_min, n_fail = math.inf, 0
    per_sub = {}
    for sub, pat, info, (rings, sh_) in order:
        sh, P, yaw, s_grid, M = sh_['sh'], sh_['P'], sh_['yaw'], sh_['s_grid'], sh_['M']
        sid = sub['id']
        mine, n_wp0 = [], len(wps)
        prev_idx, prev_dir, orbit_len, transit_len = None, 0, 0.0, 0.0
        first = True
        for kr, ring in enumerate(rings):
            visit, dirn = ring_order(ring, prev_idx, prev_dir, M)
            last = None
            for j, i in enumerate(visit):
                pos = [float(P[i][0]), float(P[i][1]), float(ring['z'])]
                step = None if last is None else (i - last) % M
                adj = step in (1, M - 1)
                linked = adj and (ring['conn'][last] if step == 1 else ring['conn'][i])
                if not first and last is not None and linked:
                    via = ring_vias(sh, s_grid[last], s_grid[i], 1 if step == 1 else -1, ring['z'])
                    method = 'ring'
                elif not first and last is None and np.allclose(np.array(prev[:2]), pos[:2], atol=0.05):
                    via, method = [], 'vertical'
                else:
                    via, method, _mc = PG.route(prev, pos, None, grid)
                    method = ('transit_' if first else 'link_') + method
                ok = via is not None
                if not ok:
                    n_fail += 1
                    via = []
                pts = [prev] + via + [pos]
                leg = sum(float(np.linalg.norm(np.subtract(q, p))) for p, q in zip(pts[:-1], pts[1:]))
                for p, q in zip(pts[:-1], pts[1:]):
                    raw_min = min(raw_min, PG.segment_min_clearance(p, q, None))
                yw = float(yaw[i])
                cam, _ = cam_pose(np.array(pos), yw, ring['pitch'])
                inw = sh.aim(np.array(pos[:2])[None])[0]
                dcam = math.hypot(inw[0] - pos[0], inw[1] - pos[1])
                wps.append({
                    'waypoint_id': f'{sid}_R{kr + 1}_{len([w for w in wps[n_wp0:] if w["ring"] == kr + 1]) + 1:02d}',
                    'position_m': [round(v, 4) for v in pos], 'heading_rad': round(yw, 4),
                    'gimbal_pitch_rad': round(float(ring['pitch']), 4),
                    'camera_world_m': [round(float(v), 4) for v in cam],
                    'target_m': [round(float(inw[0]), 3), round(float(inw[1]), 3), round(float(pos[2] + dcam * math.tan(ring['pitch'])), 3)],
                    'seed_member': sub['column_names'][0], 'seed_component': 'piers', 'lane': 'columns',
                    'zone': int(sid[2:]), 'column_id': sid, 'ring': kr + 1, 'ring_z_m': ring['z'], 'ring_index': int(i),
                    'route_in': via, 'route_method': method, 'route_length_m': round(leg, 3),
                    'reachable_in_plan': ok, 'candidate_index': -1})
                if ok:
                    total += leg
                    prev = pos
                    if first or method.startswith(('transit', 'link')):
                        transit_len += leg
                    else:
                        orbit_len += leg
                first = False
                last = i
            if visit:
                prev_idx, prev_dir = visit[-1], dirn
        # planned coverage of this subject from the viewpoints actually kept
        V = C.Visibility(S, pat, pat['reason'] == 0)
        n_dw = 0
        if DWELLS and sub['kind'] == 'road_pier_pair':
            chosen, reach_ = plan_dwells(sub, pat, V, wps[n_wp0:])
            if chosen:
                wps[n_wp0:] = insert_dwells(wps[n_wp0:], chosen, reach_)
                n_dw = len(chosen)
        n_ring = len(wps) - n_wp0 - n_dw
        seen = np.zeros(len(pat['area']), bool)
        for w in wps[n_wp0:]:
            if not w['reachable_in_plan']:
                continue
            cam = np.array(w['camera_world_m'])
            f, l, u = C.cam_axes(w['heading_rad'], w['gimbal_pitch_rad'])
            seen[V.visible(cam, f, l, u)] = True
        vis = pat['reason'] == 0
        A = pat['area']

        def pct(mask):
            m = vis & mask
            return None if not m.any() else round(100 * float(A[m & seen].sum()) / float(A[m].sum()), 1)
        col = np.isin(pat['solid'], sub['columns'])
        per_sub[sid] = {
            'rings': len(rings), 'viewpoints': n_ring, 'look_up_dwells': n_dw, 'orbit_path_m': round(orbit_len, 1),
            'transit_in_and_links_m': round(transit_len, 1),
            'est_time_min': round(((orbit_len + transit_len) / CRUISE + S_PER_WAYPOINT * n_ring + DWELL_S * n_dw) / 60, 1),
            'coverage_pct': {'shaft': pct(col & (pat['face'] == 'side')), 'column_all_faces': pct(col),
                             'cap_or_head': pct(np.isin(pat['solid'], sub['caps'])),
                             'footing': pct(np.isin(pat['solid'], sub['footings'])), 'whole_subject': pct(np.ones(len(A), bool))},
            'visible_by_face': {f'{k}/{fc}': round(float(A[vis & (pat['kind'] == k) & (pat['face'] == fc)].sum()), 1)
                                for k in sorted(set(pat['kind'])) for fc in sorted(set(pat['face']))
                                if (vis & (pat['kind'] == k) & (pat['face'] == fc)).any()},
            'visible_area_m2': round(float(A[vis].sum()), 1), 'uncovered_area_m2': round(float(A[vis & ~seen].sum()), 1),
            'uncovered_by_face': {f'{k}/{fc}': round(float(A[vis & ~seen & (pat['kind'] == k) & (pat['face'] == fc)].sum()), 1)
                                  for k in sorted(set(pat['kind'])) for fc in sorted(set(pat['face']))
                                  if (vis & ~seen & (pat['kind'] == k) & (pat['face'] == fc)).any()}}
    rth, rth_m, _ = PG.route(prev, HOME_AIR.tolist(), None, grid)
    pts = [prev] + (rth or []) + [HOME_AIR.tolist()]
    rth_len = sum(float(np.linalg.norm(np.subtract(q, p))) for p, q in zip(pts[:-1], pts[1:]))
    total += rth_len + PG.TAKEOFF_ALT_M
    # independent re-check of every flown segment at the strict 3.5 m rule (no tier-3 relaxation)
    n_seg = n_bad = 0
    raw_min = math.inf
    p0 = HOME_AIR.tolist()
    worst = []
    for w in wps:
        if not w['reachable_in_plan']:
            continue
        pts = [p0] + w['route_in'] + [w['position_m']]
        for a_, b_ in zip(pts[:-1], pts[1:]):
            n_seg += 1
            ok = PG.seg_ok(a_, b_, None)
            mc_ = PG.segment_min_clearance(a_, b_, None)
            raw_min = min(raw_min, mc_)
            if not ok or mc_ < PLAN_CLEARANCE_M - 1e-3:
                n_bad += 1
                worst.append((w['waypoint_id'], round(mc_, 3)))
        p0 = w['position_m']
    piv_min = min(C.fast_clearance(S, np.array([w['camera_world_m']]))[0] for w in wps if w['reachable_in_plan'])
    plan = {'generated_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'planner': 'gazebo/coverage_v5/plan_columns.py (offline; geometry only, no defect data)',
            'mode': 'per_column_orbital_inspection', 'camera': 'gimbal', 'camera_mount_m': PIVOT.tolist(),
            'sensed_clearance_m': 3.0, 'plan_clearance_m': PLAN_CLEARANCE_M, 'min_agl_m': 3.5,
            'home_ground_world_m': [20.0, -30.0, 0.0], 'home_air_world_m': HOME_AIR.tolist(),
            'n_waypoints': len(wps), 'n_look_up_dwells': sum(1 for w in wps if w.get('dwell')), 'route_failures': n_fail,
            'segment_recheck': {'n_segments': n_seg, 'n_violations': n_bad, 'raw_min_clearance_m': round(raw_min, 3),
                                'min_camera_pivot_clearance_m': round(float(piv_min), 3), 'worst': worst[:10]},
            'rth': {'route': rth or [], 'method': rth_m, 'length_m': round(rth_len, 3)},
            'planned_total_path_m': round(total, 2), 'waypoints': wps}
    return plan, per_sub


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--subjects', default='')
    ap.add_argument('--out', default=os.path.join(C.ROOT, 'mission', 'gazebo_columns_plan.json'))
    ap.add_argument('--report', default=os.path.join(C.ROOT, 'mission', 'columns_plan_report.json'))
    ap.add_argument('--no-route', action='store_true', help='design + coverage only (no routing, no plan JSON)')
    ap.add_argument('--no-dwells', action='store_true', help='do not add look-up dwells (the plan flown in columns_smoke25)')
    a = ap.parse_args()
    global DWELLS
    DWELLS = not a.no_dwells
    t0 = time.time()
    S = C.Solids()
    subs = inventory(S)
    want = set(filter(None, a.subjects.split(',')))
    log(f'{len(subs)} subjects; cylinder kinds in scene: {subs[0]["cyl_kinds_in_scene"]}')
    feas = Feas(S)
    designs = []
    for sub in subs:
        if want and sub['id'] not in want:
            continue
        pat = subject_patches(S, sub)
        info, res = design_subject(S, sub, feas, pat)
        designs.append((sub, pat, info, res))
        log(f"{sub['id']}: {info.get('status')} rings={len(info.get('rings', []))} z={info.get('ring_z')}")
    inv = [{k: v for k, v in s.items() if k != 'cyl_kinds_in_scene'} for s, _, _, _ in designs]
    json.dump({'inventory': inv, 'design': [d[2] for d in designs]}, open(a.report + '.design.json', 'w'), indent=1, default=float)
    if a.no_route:
        return
    log('design done in %.0f s; routing' % (time.time() - t0))
    plan, per_sub = build(designs, S, feas)
    json.dump(plan, open(a.out, 'w'), indent=1)
    R_ = STANDOFF_M + 0.85
    helix = []
    d_cam = STANDOFF_M - PIVOT[0]
    H = 2 * d_cam * C.TAN_V
    for alpha in (3, 5, 6, 8, 10, 12):
        dz = 2 * math.pi * R_ * math.tan(math.radians(alpha))
        helix.append({'path_pitch_deg': alpha, 'descent_per_revolution_m': round(dz, 2),
                      'vertical_footprint_m': round(H, 2), 'vertical_overlap_frac': round(1 - dz / H, 2),
                      'lidar_band_margin_deg': round(14 - alpha, 1)})
    rep = {'helix_analysis': {'orbit_radius_m': R_, 'note': 'ring lidar +/-14 deg; follower flies <=12 deg as flat', 'rows': helix},
           'inventory': inv, 'design': [d[2] for d in designs], 'per_subject': per_sub,
           'totals': {'waypoints': plan['n_waypoints'], 'look_up_dwells': plan['n_look_up_dwells'],
                      'est_time_min_sum_of_subjects': round(sum(v['est_time_min'] for v in per_sub.values()), 1),
                      'est_total_sim_min_path_based': round((plan['planned_total_path_m'] / CRUISE + S_PER_WAYPOINT * (plan['n_waypoints'] - plan['n_look_up_dwells'])
                                                             + DWELL_S * plan['n_look_up_dwells']) / 60, 1),
                      'est_note': 'dwell time (%.1f s each) is an estimate, not yet measured in flight' % DWELL_S,
                      'path_m': plan['planned_total_path_m'],
                      'route_failures': plan['route_failures'], 'segment_recheck': plan['segment_recheck']}}
    json.dump(rep, open(a.report, 'w'), indent=1, default=float)
    log(f"plan: {plan['n_waypoints']} viewpoints, path {plan['planned_total_path_m']:.0f} m, failures {plan['route_failures']}, "
        f"recheck {plan['segment_recheck']} -> {a.out}  ({time.time() - t0:.0f} s)")


if __name__ == '__main__':
    main()
