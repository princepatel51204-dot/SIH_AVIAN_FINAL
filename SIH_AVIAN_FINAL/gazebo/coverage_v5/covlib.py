"""SIH_AVIAN_FINAL -- coverage v5 library (offline; planning + scoring only).

One geometry model shared by every coverage number in Task 1 v5:
  * solids  = the digital twin's collision JSON (ground/water handled as the
              flat z=0 datum) + the Gazebo-only visual boxes (road vehicles,
              bank vegetation) that the camera and LiDAR also see
  * patches = the bridge surface rebuilt on a uniform grid (each patch
              carries its area), inspectable kinds = coverage_final.py's
  * camera  = the real Gazebo front_camera: 640x480, HFOV 80 deg (VFOV
              64.4 deg), max range 12 m, incidence <= 60 deg, line of sight
              ray-cast against the solids
Nothing here is ever imported by the flight follower: navigation and
avoidance stay sensed-only.
"""
import sys
sys.modules.setdefault("coverage", None)   # numba vs the system 'coverage' 7.4 package: hide it
import json
import math
import os
import re

import numba
import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
COLLISION = os.path.join(ROOT, "scene", "collision", "avian_bridge_collision.json")
VISUAL_ONLY_SDF = [os.path.join(ROOT, "gazebo", "models", m, "model.sdf")
                   for m in ("avian_final_vehicles", "avian_final_vegetation")]
EXCLUDE_KINDS = {"ground", "water", "landing_pad", "train_car"}   # coverage_final.py's own set
GROUND_Z = 0.0

# ---------------------------------------------------------------- camera ---
IMG_W, IMG_H = 640, 480
HFOV = 1.3962634                                   # x500_base/model.sdf front_camera
TAN_H = math.tan(HFOV / 2)
TAN_V = TAN_H * IMG_H / IMG_W
VFOV = 2 * math.atan(TAN_V)
MAX_RANGE_M = 12.0
MAX_INC_DEG = 60.0
LOS_EPS_M = 0.05
FIXED_MOUNT = np.array([0.15, 0.0, -0.02])         # base_link -> camera, current airframe
FIXED_PITCH_DOWN = 0.4                             # rad, SDF pose pitch (nose-down positive)


class Solids:
    def __init__(self):
        rows = [p for p in json.load(open(COLLISION))["primitives"] if p["kind"] not in ("ground", "water")]
        self.n_json = len(rows)
        pat = re.compile(r'<visual name="([^"]+)">\s*<pose>([^<]+)</pose>\s*'
                         r'<geometry>\s*<box>\s*<size>([^<]+)</size>', re.S)
        for f in VISUAL_ONLY_SDF:
            for name, pose, size in pat.findall(open(f).read()):
                p = [float(v) for v in pose.split()]
                s = [float(v) for v in size.split()]
                assert abs(p[3]) < 1e-9 and abs(p[4]) < 1e-9, "visual-only box with roll/pitch"
                rows.append({"name": name, "type": "BOX", "kind": "visual_only", "centre": p[:3],
                             "half_extents": [v / 2 for v in s], "yaw": p[5]})
        self.rows = rows
        self.names = [r["name"] for r in rows]
        self.kinds = [r["kind"] for r in rows]
        n = len(rows)
        self.typ = np.array([0 if r["type"] == "BOX" else 1 for r in rows], np.int64)
        self.c = np.array([r["centre"] for r in rows], float)
        self.h = np.zeros((n, 3))
        for i, r in enumerate(rows):
            self.h[i] = r["half_extents"] if r["type"] == "BOX" else [r["radius"], r["radius"], r["half_height"]]
        yaw = np.array([r.get("yaw", 0.0) for r in rows])
        self.cyaw, self.syaw = np.cos(yaw), np.sin(yaw)
        self.bound = np.where(self.typ == 0, np.linalg.norm(self.h, axis=1),
                              np.hypot(self.h[:, 0], self.h[:, 2]))
        self.index = {nm: i for i, nm in enumerate(self.names)}

    def args(self):
        return self.typ, self.c, self.h, self.cyaw, self.syaw, self.bound


# ------------------------------------------------------------ numba core ---
@numba.njit(cache=True)
def _sdf(px, py, pz, t, cx, cy, cz, hx, hy, hz, ca, sa):
    dx, dy, dz = px - cx, py - cy, pz - cz
    if t == 0:
        lx = ca * dx + sa * dy
        ly = -sa * dx + ca * dy
        qx, qy, qz = abs(lx) - hx, abs(ly) - hy, abs(dz) - hz
        ox, oy, oz = max(qx, 0.0), max(qy, 0.0), max(qz, 0.0)
        return math.sqrt(ox * ox + oy * oy + oz * oz) + min(max(qx, max(qy, qz)), 0.0)
    qr = math.sqrt(dx * dx + dy * dy) - hx
    qz = abs(dz) - hz
    orr, oz = max(qr, 0.0), max(qz, 0.0)
    return math.sqrt(orr * orr + oz * oz) + min(max(qr, qz), 0.0)


@numba.njit(parallel=True, cache=True)
def min_sdf(pts, skip, cutoff, typ, c, h, ca, sa, bound):
    """Signed distance of each point to the nearest solid (skip[i] = a solid
    to ignore, -1 none), capped at `cutoff`."""
    m = pts.shape[0]
    out = np.empty(m)
    for i in numba.prange(m):
        best = cutoff
        px, py, pz = pts[i, 0], pts[i, 1], pts[i, 2]
        for k in range(typ.shape[0]):
            if k == skip[i]:
                continue
            ddx, ddy, ddz = px - c[k, 0], py - c[k, 1], pz - c[k, 2]
            if math.sqrt(ddx * ddx + ddy * ddy + ddz * ddz) - bound[k] >= best:
                continue
            d = _sdf(px, py, pz, typ[k], c[k, 0], c[k, 1], c[k, 2], h[k, 0], h[k, 1], h[k, 2], ca[k], sa[k])
            if d < best:
                best = d
        out[i] = best
    return out


@numba.njit(cache=True)
def _ray(ox, oy, oz, ux, uy, uz, t, cx, cy, cz, hx, hy, hz, ca, sa):
    """(t_in, t_out) of the ray o + s*u through one solid; t_in > t_out = miss."""
    px, py, pz = ox - cx, oy - cy, oz - cz
    if t == 0:
        lo = (ca * px + sa * py, -sa * px + ca * py, pz)
        ld = (ca * ux + sa * uy, -sa * ux + ca * uy, uz)
        hh = (hx, hy, hz)
        t0, t1 = -1e30, 1e30
        for a in range(3):
            if abs(ld[a]) < 1e-12:
                if abs(lo[a]) > hh[a]:
                    return 1.0, -1.0
            else:
                ta = (-hh[a] - lo[a]) / ld[a]
                tb = (hh[a] - lo[a]) / ld[a]
                if ta > tb:
                    ta, tb = tb, ta
                if ta > t0:
                    t0 = ta
                if tb < t1:
                    t1 = tb
                if t0 > t1:
                    return 1.0, -1.0
        return t0, t1
    # vertical cylinder
    t0, t1 = -1e30, 1e30
    a = ux * ux + uy * uy
    if a < 1e-12:
        if px * px + py * py > hx * hx:
            return 1.0, -1.0
    else:
        b = 2.0 * (px * ux + py * uy)
        cc = px * px + py * py - hx * hx
        disc = b * b - 4.0 * a * cc
        if disc < 0.0:
            return 1.0, -1.0
        sq = math.sqrt(disc)
        t0 = (-b - sq) / (2.0 * a)
        t1 = (-b + sq) / (2.0 * a)
    if abs(uz) < 1e-12:
        if abs(pz) > hz:
            return 1.0, -1.0
    else:
        ta = (-hz - pz) / uz
        tb = (hz - pz) / uz
        if ta > tb:
            ta, tb = tb, ta
        if ta > t0:
            t0 = ta
        if tb < t1:
            t1 = tb
    return t0, t1


@numba.njit(parallel=True, cache=True)
def blocked(orig, tgt, eps_end, skip, cand, typ, c, h, ca, sa, bound):
    """Segment orig[i] -> tgt[i] blocked by any solid in `cand` (indices),
    other than skip[i], before (length - eps_end)?"""
    m = orig.shape[0]
    out = np.zeros(m, np.bool_)
    for i in numba.prange(m):
        ox, oy, oz = orig[i, 0], orig[i, 1], orig[i, 2]
        dx, dy, dz = tgt[i, 0] - ox, tgt[i, 1] - oy, tgt[i, 2] - oz
        L = math.sqrt(dx * dx + dy * dy + dz * dz)
        if L < 1e-9:
            continue
        ux, uy, uz = dx / L, dy / L, dz / L
        lim = L - eps_end
        for j in range(cand.shape[0]):
            k = cand[j]
            if k == skip[i]:
                continue
            # bounding-sphere cull: distance from solid centre to the segment
            wx, wy, wz = c[k, 0] - ox, c[k, 1] - oy, c[k, 2] - oz
            s = wx * ux + wy * uy + wz * uz
            if s < 0.0:
                s = 0.0
            elif s > L:
                s = L
            ex, ey, ez = wx - s * ux, wy - s * uy, wz - s * uz
            if ex * ex + ey * ey + ez * ez > bound[k] * bound[k]:
                continue
            t0, t1 = _ray(ox, oy, oz, ux, uy, uz, typ[k], c[k, 0], c[k, 1], c[k, 2],
                          h[k, 0], h[k, 1], h[k, 2], ca[k], sa[k])
            if t0 <= t1 and t1 > 1e-6 and t0 < lim:
                out[i] = True
                break
    return out


# -------------------------------------------------------------- patches ---
def _grid(half, step):
    n = max(1, int(math.ceil(2 * half / step - 1e-9)))
    return (-half + 2 * half * (np.arange(n) + 0.5) / n), 2 * half / n


def build_patches(solids, step=1.0):
    """Uniform surface sampling of every inspectable primitive (cells of at
    most step x step m). Returns dict of arrays."""
    P, N, A, S, F = [], [], [], [], []
    for k, r in enumerate(solids.rows):
        if r["kind"] in EXCLUDE_KINDS or r["kind"] == "visual_only":
            continue
        c = np.array(r["centre"], float)
        if r["type"] == "BOX":
            hx, hy, hz = r["half_extents"]
            yaw = r.get("yaw", 0.0)
            ex = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            ey = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
            ez = np.array([0.0, 0.0, 1.0])
            faces = [("+X", ex, ey, ez, hy, hz, hx), ("-X", -ex, ey, ez, hy, hz, hx),
                     ("+Y", ey, ex, ez, hx, hz, hy), ("-Y", -ey, ex, ez, hx, hz, hy),
                     ("+Z", ez, ex, ey, hx, hy, hz), ("-Z", -ez, ex, ey, hx, hy, hz)]
            for fname, nrm, ua, va, hu, hv, hn in faces:
                us, du = _grid(hu, step)
                vs, dv = _grid(hv, step)
                fc = c + nrm * hn
                for u in us:
                    for v in vs:
                        P.append(fc + ua * u + va * v); N.append(nrm); A.append(du * dv); S.append(k); F.append(fname)
        else:
            rad, hh = r["radius"], r["half_height"]
            nth = max(8, int(math.ceil(2 * math.pi * rad / step)))
            zs, dz = _grid(hh, step)
            for i in range(nth):
                th = 2 * math.pi * (i + 0.5) / nth
                nrm = np.array([math.cos(th), math.sin(th), 0.0])
                for z in zs:
                    P.append(c + nrm * rad + [0, 0, z]); N.append(nrm)
                    A.append(2 * math.pi * rad / nth * dz); S.append(k); F.append("side")
            for cname, sgn in (("top", 1.0), ("bottom", -1.0)):
                g, d = _grid(rad, step)
                cells = [(x, y) for x in g for y in g if x * x + y * y <= rad * rad] or [(0.0, 0.0)]
                a = math.pi * rad * rad / len(cells)
                for x, y in cells:
                    P.append(c + [x, y, sgn * hh]); N.append(np.array([0, 0, sgn])); A.append(a); S.append(k); F.append(cname)
    return {"p": np.array(P), "n": np.array(N, float), "area": np.array(A), "solid": np.array(S, np.int64),
            "face": np.array(F)}


def classify_exclusions(solids, pt):
    """Returns reason codes per patch: 0 visible, 1 buried (centre inside
    another solid / below ground), 2 faces into a solid within 0.5 m."""
    a = solids.args()
    p, n, own = pt["p"], pt["n"], pt["solid"]
    sd = min_sdf(p, own, 1e9, *a)
    below = p[:, 2] < GROUND_Z - 1e-3
    buried = (sd < -1e-3) | below
    o = p + 0.01 * n
    blk = blocked(o, o + 0.5 * n, 0.0, own, np.arange(len(solids.typ)), *a)
    # the ground plane: a downward-facing patch within 0.5 m of z=0
    gnd = (n[:, 2] < -1e-6) & ((p[:, 2] - GROUND_Z) / np.maximum(-n[:, 2], 1e-6) <= 0.5)
    reason = np.zeros(len(p), np.int64)
    reason[(blk | gnd) & ~buried] = 2
    reason[buried] = 1
    return reason, sd


def component_of(kind, name, nz):
    if kind in ("pier_column", "pier_cap", "pier_footing"):
        return "piers"
    if kind == "bearing":
        return "bearings"
    if kind == "abutment" or "WINGWALL" in name:
        return "abutments"
    if kind in ("truss_chord", "truss_vertical", "truss_diagonal", "bracing", "gusset_plate"):
        return "truss"
    if kind in ("girder", "diaphragm", "stringer", "floor_beam") or name.startswith("MB_WEB"):
        return "girders"
    if kind in ("deck_slab", "deck_box", "track_slab") or name.startswith(("BR_WEARING", "BR_MEDIAN")):
        return "deck underside" if nz < -0.7 else ("deck top" if nz > 0.7 else "deck edges")
    if kind == "parapet":
        return "parapets"
    return "deck furniture"


# ---------------------------------------------------------------- camera ---
def cam_axes(yaw, pitch_up):
    """Camera forward/left/up unit vectors (world ENU) for a level body at
    `yaw` with the optical axis `pitch_up` rad above horizontal."""
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch_up), math.sin(pitch_up)
    f = np.array([cy * cp, sy * cp, sp])
    l = np.array([-sy, cy, 0.0])
    u = np.cross(f, l)
    return f, l, u


class Visibility:
    def __init__(self, solids, patches, mask):
        self.s = solids
        self.args = solids.args()
        self.idx = np.flatnonzero(mask)                # patch ids considered
        self.p = patches["p"][self.idx]
        self.n = patches["n"][self.idx]
        self.tree = cKDTree(self.p)
        self.cos_inc = math.cos(math.radians(MAX_INC_DEG))

    def visible(self, cam, f, l, u):
        """Patch ids (global) visible from a camera at `cam` with axes f,l,u."""
        near = self.tree.query_ball_point(cam, MAX_RANGE_M)
        if not near:
            return np.zeros(0, np.int64)
        near = np.asarray(near)
        v = self.p[near] - cam
        fw = v @ f
        ok = fw > 1e-6
        ok &= np.abs(v @ l) <= TAN_H * fw
        ok &= np.abs(v @ u) <= TAN_V * fw
        d = np.linalg.norm(v, axis=1)
        ok &= d <= MAX_RANGE_M
        ok &= (-(v * self.n[near]).sum(1) / np.maximum(d, 1e-9)) >= self.cos_inc
        sel = near[ok]
        if not len(sel):
            return np.zeros(0, np.int64)
        typ, c, h, ca, sa, bound = self.args
        cand = np.flatnonzero(np.linalg.norm(c - cam, axis=1) - bound <= MAX_RANGE_M + 0.1)
        orig = np.repeat(cam[None], len(sel), 0)
        blk = blocked(orig, self.p[sel], LOS_EPS_M, np.full(len(sel), -1, np.int64), cand, *self.args)
        return self.idx[sel[~blk]]


def fast_clearance(solids, pts):
    """Planning clearance = min(distance to any solid, height above ground)."""
    pts = np.atleast_2d(np.asarray(pts, float))
    sd = min_sdf(pts, np.full(len(pts), -1, np.int64), 50.0, *solids.args())
    return np.minimum(sd, pts[:, 2] - GROUND_Z)
