"""3D defect localisation and the defect register. Pure numpy, no ROS.

VISUALISATION AND REPORTING ONLY. Nothing here feeds the mission follower, and
nothing here reads the collision JSON, the Blender model, the SDF world or any
ground-truth defect position. Inputs are exactly what the drone itself has:
  * a detection (family, score, pixel box) from the live detector,
  * the EKF pose and the true gimbal angle at the frame time (via viz_frames),
  * the voxel map accumulated from the drone's own LiDAR / up / down returns.

Ranging method (per detection)
  1. Cast the ray through the bbox centre (viz_frames.pixel_ray, camera pose from
     the EKF + gimbal) into the voxel map, 0.125 m steps, first occupied voxel.
       exact voxel hit             -> 'voxel_hit'      (surface is inside that voxel)
       hit only within +/-1 voxel  -> 'voxel_hit_tol'  (looser: the LiDAR paints
                                        stripes 2 deg apart, so the ray can pass
                                        between two returns)
  2. Fit a plane (PCA) to the occupied voxel centres within 0.9 m of that hit. If
     it is planar enough and not grazing, intersect the ray with the plane
     -> 'plane_fit' (sub-voxel range, plus a surface normal used for size).
  3. No occupied voxel anywhere along the ray -> the detection is UNLOCALIZED. It
     is kept (origin + ray) and retried for retry_s seconds as the map grows; if
     the map never gets a return on that ray it stays unlocalized. No position is
     ever guessed.

Uncertainty (1 sigma, metres, per observation) -- ASSUMPTIONS, not measurements:
  range     sqrt(sigma_quantisation^2 + plane_rms^2), quantisation = voxel/sqrt(12)
            for exact hits and voxel/2 for tolerance hits
  lateral   range * sqrt(sigma_gimbal^2 + sigma_att^2 + (sigma_px/fx)^2)
            sigma_gimbal 0.5 deg (bridged joint state) or 3 deg (last command
            fallback), sigma_att 1 deg, sigma_px = 10 % of the box size (>= 3 px:
            where inside the defect the box centre falls)
  position  the EKF's own eph/epv (>= 0.1 m)
  total     sqrt(range^2 + lateral^2 + position^2)
A cluster reports the MEDIAN single-observation sigma (no 1/sqrt(n) shrinkage:
consecutive frames share the same gimbal, attitude and EKF errors, so they are
not independent) and the empirical scatter of its own observations.

Size: the bbox edge mid-points are intersected with the fitted plane and
measured (w x h, metres). Without a usable plane (grazing angle, no fit) it falls
back to the fronto-parallel estimate w = w_px * range / fx, which OVERSTATES the
size on an oblique surface. The result is the extent of the DETECTOR'S BOX, not
a measured defect boundary.

Clustering: same family and within gate_m (2.0 m) of an existing localized
cluster -> the same physical defect. Position is the inverse-variance weighted
mean. Consecutive frames from one hold are highly correlated, so n_obs counts
frames; n_viewpoints (distinct waypoint ids) is the better independence proxy.
Two different defects closer than gate_m WILL be merged.
"""
import csv
import itertools
import json
import math
import os

import numpy as np

import viz_frames as V

NEIGHBOUR_OFFSETS = np.array([(dx << 42) + (dy << 21) + dz
                              for dx, dy, dz in itertools.product((-1, 0, 1), repeat=3)], dtype=np.int64)

SIGMA_GIMBAL_MEASURED = math.radians(0.5)
SIGMA_GIMBAL_COMMAND = math.radians(3.0)
SIGMA_ATT = math.radians(1.0)
SIGMA_POS_FLOOR = 0.1
PLANE_RADIUS_M = 0.9
PLANE_MIN_POINTS = 8
PLANE_MAX_RMS_M = 0.20
PLANE_MIN_COS = 0.35       # |n . ray| below this (>~70 deg off normal) = grazing: no plane range/size (size error goes as 1/cos)


def _occupied(keys, q):
    idx = np.clip(np.searchsorted(keys, q), 0, len(keys) - 1)
    return keys[idx] == q


def ball_points(keys, vox, centre, radius):
    """Centres of occupied voxels within `radius` of `centre`."""
    r = int(math.ceil(radius / vox))
    ck = V.voxel_keys(np.asarray(centre, float)[None, :], vox)[0]
    offs = np.array([(dx << 42) + (dy << 21) + dz
                     for dx, dy, dz in itertools.product(range(-r, r + 1), repeat=3)], dtype=np.int64)
    q = ck + offs
    q = q[_occupied(keys, q)]
    if len(q) == 0:
        return np.zeros((0, 3))
    c = V.keys_to_centres(q, vox)
    return c[np.linalg.norm(c - np.asarray(centre), axis=1) <= radius + vox]


def fit_plane(p):
    """(centre, unit normal, rms residual) of the best-fit plane, or None."""
    if len(p) < PLANE_MIN_POINTS:
        return None
    c = p.mean(0)
    u, s, vt = np.linalg.svd(p - c, full_matrices=False)
    n = vt[2]
    rms = float(np.sqrt(np.mean(((p - c) @ n) ** 2)))
    # need real extent in two directions, otherwise the plane is undetermined (a line)
    if s[1] < 1e-6 or s[1] / max(s[0], 1e-9) < 0.15:
        return None
    return c, n, rms


def localize(keys, vox, o, d, r0=1.0, r1=30.0, step=0.125):
    """Range along ray (o, d) into the voxel map `keys` (sorted int64).
    Returns None if no occupied voxel is on the ray, else a dict."""
    if keys is None or len(keys) == 0:
        return None
    o = np.asarray(o, float)
    d = np.asarray(d, float) / np.linalg.norm(d)
    ts = np.arange(r0, r1, step)
    k0 = V.voxel_keys(o + ts[:, None] * d, vox)
    hit = _occupied(keys, k0)
    method, t_hit = 'voxel_hit', None
    if hit.any():
        t_hit = float(ts[int(np.argmax(hit))])
    else:
        hit = _occupied(keys, (k0[:, None] + NEIGHBOUR_OFFSETS[None, :]).ravel()).reshape(len(ts), -1).any(1)
        if not hit.any():
            return None
        t_hit, method = float(ts[int(np.argmax(hit))]), 'voxel_hit_tol'
    sig_q = vox / math.sqrt(12.0) if method == 'voxel_hit' else vox / 2.0
    out = {'range_m': t_hit, 'method': method, 'sigma_range_m': sig_q, 'normal': None,
           'plane_points': 0, 'plane_rms_m': None}
    p_hit = o + d * t_hit
    pts = ball_points(keys, vox, p_hit, PLANE_RADIUS_M)
    out['plane_points'] = int(len(pts))
    pl = fit_plane(pts)
    if pl is not None:
        c, n, rms = pl
        cosang = abs(float(n @ d))
        if rms <= PLANE_MAX_RMS_M and cosang >= PLANE_MIN_COS:
            t_p = float((c - o) @ n / (d @ n))
            if abs(t_p - t_hit) < 1.0 and t_p > 0.3:
                if n @ d > 0:
                    n = -n                         # face the camera
                out.update(range_m=t_p, method='plane_fit', normal=[float(x) for x in n],
                           sigma_range_m=max(rms, vox / math.sqrt(12.0)), plane_rms_m=rms)
    out['point'] = [float(x) for x in (o + d * out['range_m'])]
    return out


def plane_size(o, R_cam, bbox, plane_n, plane_p):
    """(w, h) metres of the bbox on a plane, from its four edge-midpoint rays."""
    x0, y0, x1, y1 = bbox
    xm, ym = (x0 + x1) / 2, (y0 + y1) / 2
    pts = []
    for (u, v) in ((x0, ym), (x1, ym), (xm, y0), (xm, y1)):
        d = R_cam @ V.pixel_ray(u, v)
        den = float(d @ plane_n)
        if abs(den) < 1e-6:
            return None
        t = float((plane_p - o) @ plane_n / den)
        if t <= 0:
            return None
        pts.append(o + d * t)
    return float(np.linalg.norm(pts[1] - pts[0])), float(np.linalg.norm(pts[3] - pts[2]))


def observation_sigma(rng_m, sigma_range, sigma_gimbal, bbox, eph, epv):
    x0, y0, x1, y1 = bbox
    s_px = max(3.0, 0.1 * max(x1 - x0, y1 - y0))
    s_ang = math.sqrt(sigma_gimbal ** 2 + SIGMA_ATT ** 2 + (s_px / V.CAM_FX) ** 2)
    s_lat = rng_m * s_ang
    s_pos = max(SIGMA_POS_FLOOR, float(eph or 0.0), float(epv or 0.0))
    return math.sqrt(sigma_range ** 2 + s_lat ** 2 + s_pos ** 2), s_lat


class Register:
    def __init__(self, gate_m=2.0, retry_s=40.0, max_clusters=400, max_pending=120):
        self.gate = float(gate_m)
        self.retry_s = float(retry_s)
        self.max_clusters = max_clusters
        self.max_pending = max_pending
        self.clusters = []
        self.pending = []           # unlocalized observations still being retried
        self.next_id = 1
        self.stats = {'raw_detections': 0, 'localized_first_try': 0, 'localized_on_retry': 0,
                      'unlocalized_expired': 0, 'pending_dropped': 0}

    # ------------------------------------------------------------ intake --
    def observe(self, obs, keys, vox, now):
        """obs: dict(family, score, bbox, stamp, wp, o, R_cam, sigma_gimbal, eph, epv).
        Ray origin/rotation are the EKF+gimbal camera pose at the frame time."""
        self.stats['raw_detections'] += 1
        obs['ray'] = obs['R_cam'] @ V.pixel_ray((obs['bbox'][0] + obs['bbox'][2]) / 2,
                                                (obs['bbox'][1] + obs['bbox'][3]) / 2)
        obs['t_in'] = now
        loc = localize(keys, vox, obs['o'], obs['ray'])
        if loc is None:
            if len(self.pending) < self.max_pending:
                self.pending.append(obs)
            else:
                self.stats['pending_dropped'] += 1
            return False
        self.stats['localized_first_try'] += 1
        self._add_localized(obs, loc)
        return True

    def retry(self, keys, vox, now, budget=15):
        """Retry pending rays against the current map, at most `budget` ray-casts per call
        (oldest first) so a burst of unlocalized detections cannot stall the node; expiry is
        checked for all of them."""
        keep, tried = [], 0
        for obs in self.pending:
            if tried < budget:
                tried += 1
                loc = localize(keys, vox, obs['o'], obs['ray'])
                if loc is not None:
                    self.stats['localized_on_retry'] += 1
                    self._add_localized(obs, loc)
                    continue
            if now - obs['t_in'] > self.retry_s:
                self.stats['unlocalized_expired'] += 1
                self._add_unlocalized(obs)
            else:
                keep.append(obs)
        self.pending = keep

    def flush(self, keys, vox, now):
        """Final pass: retry once more, then everything still pending is unlocalized."""
        self.retry(keys, vox, now, budget=len(self.pending))
        for obs in self.pending:
            self.stats['unlocalized_expired'] += 1
            self._add_unlocalized(obs)
        self.pending = []

    # ---------------------------------------------------------- clusters --
    def _add_localized(self, obs, loc):
        p = np.array(loc['point'])
        sig, s_lat = observation_sigma(loc['range_m'], loc['sigma_range_m'], obs['sigma_gimbal'],
                                       obs['bbox'], obs.get('eph'), obs.get('epv'))
        size, size_method = None, 'none'
        if loc['normal'] is not None:
            size = plane_size(obs['o'], obs['R_cam'], obs['bbox'], np.array(loc['normal']), p)
            size_method = 'plane_fit'
        if size is None:
            x0, y0, x1, y1 = obs['bbox']
            size = ((x1 - x0) * loc['range_m'] / V.CAM_FX, (y1 - y0) * loc['range_m'] / V.CAM_FY)
            size_method = 'fronto_parallel'
        o = {'p': p, 'sigma': sig, 'score': obs['score'], 'stamp': obs['stamp'], 'wp': obs['wp'],
             'size': size, 'size_method': size_method, 'range': loc['range_m'], 'method': loc['method'],
             'sigma_lat': s_lat}
        best, bd = None, self.gate
        for c in self.clusters:
            if c['fam'] != obs['family'] or c['localized'] is not True:
                continue
            dd = float(np.linalg.norm(c['pos'] - p))
            if dd < bd:
                best, bd = c, dd
        if best is None:
            if len(self.clusters) >= self.max_clusters:
                return
            best = {'id': f'D{self.next_id:03d}', 'fam': obs['family'], 'localized': True, 'obs': []}
            self.next_id += 1
            self.clusters.append(best)
        best['obs'].append(o)
        self._refresh(best)
        self._consolidate()

    def _add_unlocalized(self, obs):
        for c in self.clusters:
            if c['localized'] is False and c['fam'] == obs['family'] and c['wp_key'] == obs['wp']:
                c['obs'].append({'score': obs['score'], 'stamp': obs['stamp'], 'wp': obs['wp']})
                c['last_ray'] = (obs['o'], obs['ray'])
                self._refresh(c)
                return
        if len(self.clusters) >= self.max_clusters:
            return
        c = {'id': f'U{self.next_id:03d}', 'fam': obs['family'], 'localized': False, 'wp_key': obs['wp'],
             'last_ray': (obs['o'], obs['ray']),
             'obs': [{'score': obs['score'], 'stamp': obs['stamp'], 'wp': obs['wp']}]}
        self.next_id += 1
        self.clusters.append(c)
        self._refresh(c)

    def _refresh(self, c):
        ob = c['obs']
        sc = [o['score'] for o in ob]
        c['n_obs'] = len(ob)
        c['best_conf'] = float(max(sc))
        c['mean_conf'] = float(np.mean(sc))
        c['first_seen_s'] = float(min(o['stamp'] for o in ob))
        c['last_seen_s'] = float(max(o['stamp'] for o in ob))
        c['first_wp'] = min(ob, key=lambda o: o['stamp'])['wp']
        c['last_wp'] = max(ob, key=lambda o: o['stamp'])['wp']
        c['n_viewpoints'] = len({o['wp'] for o in ob})
        if not c['localized']:
            return
        P = np.array([o['p'] for o in ob])
        w = 1.0 / np.array([o['sigma'] for o in ob]) ** 2
        c['pos'] = (P * w[:, None]).sum(0) / w.sum()
        c['pos_sigma_m'] = float(np.median([o['sigma'] for o in ob]))
        c['pos_scatter_m'] = float(np.sqrt(np.mean(np.sum((P - c['pos']) ** 2, axis=1)))) if len(P) > 1 else None
        sz = np.array([o['size'] for o in ob])
        c['size_w_m'], c['size_h_m'] = float(np.median(sz[:, 0])), float(np.median(sz[:, 1]))
        m = [o['size_method'] for o in ob]
        c['size_method'] = 'plane_fit' if m.count('plane_fit') >= len(m) / 2 else 'fronto_parallel'
        meth = {}
        for o in ob:
            meth[o['method']] = meth.get(o['method'], 0) + 1
        c['ranging_methods'] = meth
        c['median_range_m'] = float(np.median([o['range'] for o in ob]))

    def _consolidate(self):
        merged = True
        while merged:
            merged = False
            loc = [c for c in self.clusters if c['localized']]
            for a, b in itertools.combinations(loc, 2):
                if a['fam'] == b['fam'] and np.linalg.norm(a['pos'] - b['pos']) < self.gate:
                    a['obs'].extend(b['obs'])
                    self.clusters.remove(b)
                    self._refresh(a)
                    merged = True
                    break

    # ------------------------------------------------------------ output --
    def localized(self):
        return [c for c in self.clusters if c['localized']]

    def snapshot(self):
        loc = self.localized()
        unl = [c for c in self.clusters if not c['localized']]
        rows = []
        for c in sorted(loc, key=lambda c: c['id']):
            rows.append({
                'id': c['id'], 'class': c['fam'],
                'x_m': round(float(c['pos'][0]), 3), 'y_m': round(float(c['pos'][1]), 3),
                'z_m': round(float(c['pos'][2]), 3),
                'pos_sigma_m': round(c['pos_sigma_m'], 3),
                'pos_scatter_m': None if c['pos_scatter_m'] is None else round(c['pos_scatter_m'], 3),
                'size_w_m': round(c['size_w_m'], 3), 'size_h_m': round(c['size_h_m'], 3),
                'size_method': c['size_method'], 'ranging_methods': c['ranging_methods'],
                'median_range_m': round(c['median_range_m'], 2),
                'n_obs': c['n_obs'], 'n_viewpoints': c['n_viewpoints'],
                'best_conf': round(c['best_conf'], 4), 'mean_conf': round(c['mean_conf'], 4),
                'first_seen_sim_s': round(c['first_seen_s'], 2), 'last_seen_sim_s': round(c['last_seen_s'], 2),
                'first_wp': c['first_wp'], 'last_wp': c['last_wp']})
        urows = [{'id': c['id'], 'class': c['fam'], 'n_obs': c['n_obs'], 'best_conf': round(c['best_conf'], 4),
                  'first_seen_sim_s': round(c['first_seen_s'], 2), 'last_seen_sim_s': round(c['last_seen_s'], 2),
                  'wp': c['wp_key']} for c in unl]
        n_loc_obs = sum(c['n_obs'] for c in loc)
        return {
            'summary': {
                'raw_detections': self.stats['raw_detections'],
                'localized_detections': n_loc_obs,
                'unlocalized_detections': sum(c['n_obs'] for c in unl),
                'still_pending_retry': len(self.pending),
                'distinct_localized_defects': len(loc),
                'distinct_unlocalized_groups': len(unl),
                'localized_on_first_try': self.stats['localized_first_try'],
                'localized_on_retry': self.stats['localized_on_retry'],
                'gate_m': self.gate, 'retry_s': self.retry_s},
            'defects': rows, 'unlocalized': urows,
            'frame': 'map == Gazebo world ENU (x east, y north, z up), metres',
            'method': __doc__.split('Uncertainty')[0].strip(),
            'source': 'drone sensors + PX4 EKF pose + gimbal joint state only; no ground truth'}

    def write(self, out_dir):
        snap = self.snapshot()
        tmp = os.path.join(out_dir, 'defect_register.json.tmp')
        json.dump(snap, open(tmp, 'w'), indent=1)
        os.replace(tmp, os.path.join(out_dir, 'defect_register.json'))
        cols = ['id', 'class', 'x_m', 'y_m', 'z_m', 'pos_sigma_m', 'pos_scatter_m', 'size_w_m', 'size_h_m',
                'size_method', 'median_range_m', 'n_obs', 'n_viewpoints', 'best_conf', 'mean_conf',
                'first_seen_sim_s', 'last_seen_sim_s', 'first_wp', 'last_wp']
        with open(os.path.join(out_dir, 'defect_register.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
            w.writeheader()
            w.writerows(snap['defects'])
        return snap
