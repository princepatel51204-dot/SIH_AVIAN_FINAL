#!/usr/bin/env python3
"""SIH_AVIAN_FINAL -- Gazebo Task 1: turn coverage_mission.json into a
flyable single-pass mission plan for the Gazebo corridor world.

OFFLINE MISSION PLANNING ONLY. This is the one place that reads the known
scene geometry (the digital twin's own collision JSON, plus the SDF boxes of
the Gazebo-only visual models the drone's LiDAR will also see). It runs
before flight and writes mission/gazebo_mission_plan.json. The runtime
follower (mission_follower_node.py) reads ONLY that plan file plus its own
onboard sensors -- never this script's inputs -- so in-flight navigation
and avoidance stay sensed-only.

What it does, and why:
  1. Keeps coverage_mission.json's 150 inspection waypoints, same IDs, same
     order, each exactly once.
  2. Nudges only the waypoints that sit closer than PLAN_CLEARANCE_M to
     structure (or below MIN_AGL_M). coverage_final.py placed them at an 8 m
     standoff from their OWN target patch but did not check every other
     primitive -- 36/150 sit 0.08-2.6 m from some other member, inside the
     3.0 m sensed-avoidance ring, so the aircraft could never legally
     settle there. Each nudge is the smallest move that restores clearance
     while keeping a clear line of sight to the waypoint's own target_m.
     Every nudge is recorded (original position kept alongside).
  3. Routes every transit leg (home -> WP1 -> ... -> WP150 -> home) either
     straight, when the straight segment keeps PLAN_CLEARANCE_M, or via an
     A* detour on a 1 m occupancy grid, line-of-sight pruned. Via-points are
     pass-through points, not inspection waypoints.

Usage: python3 plan_gazebo_mission.py
"""
from __future__ import annotations

import heapq
import json
import math
import os
import re
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "source"))
from pathfinding_final import _clearance_batch  # noqa: E402  (same geometry as Section 4)

COVERAGE = os.path.join(ROOT, "mission", "coverage_mission.json")
COLLISION = os.path.join(ROOT, "scene", "collision", "avian_bridge_collision.json")
VISUAL_ONLY_SDF = [os.path.join(ROOT, "gazebo", "models", m, "model.sdf")
                   for m in ("avian_final_vehicles", "avian_final_vegetation")]
OUT = os.path.join(ROOT, "mission", "gazebo_mission_plan.json")

SENSED_CLEARANCE_M = 3.0   # the runtime safety layer's hard ring
PLAN_CLEARANCE_M = 3.5     # plan 0.5 m outside it, so position-hold jitter
                           # at a settled waypoint never trips the ring
MIN_AGL_M = 3.5            # same rule against the ground (terrain top z=0)
GROUND_Z = 0.0
HOME_WORLD = (20.0, -30.0)  # AVI_BASE_SCANNER pad centre = spawn x,y
TAKEOFF_ALT_M = 5.0
VOXEL_M = 1.0
GRID_LO = np.array([-25.0, -45.0, 0.0])
GRID_HI = np.array([385.0, 55.0, 40.0])
# The follower's sensed coverage is the LiDAR's +/-14 deg band plus the
# +/-45 deg up/down cones, so it only flies near-horizontal (<=12 deg) or
# vertical motion. Every planned segment obeys the same rule, so what is
# flown is exactly what was clearance-checked here.
MAX_SLOPE_DEG = 12.0


def load_obstacles():
    prims = [p for p in json.load(open(COLLISION))["primitives"]
             if p["kind"] not in ("ground", "water")]
    n_json = len(prims)
    # Gazebo-only visual models (road vehicles, bank vegetation): no
    # collision in the physics world, but a gpu_lidar renders visuals, so
    # the sensed safety layer WILL see them -- plan around them too.
    pat = re.compile(r'<visual name="([^"]+)">\s*<pose>([^<]+)</pose>\s*'
                     r'<geometry>\s*<box>\s*<size>([^<]+)</size>', re.S)
    n_vis = 0
    for f in VISUAL_ONLY_SDF:
        for name, pose, size in pat.findall(open(f).read()):
            p = list(map(float, pose.split()))
            s = list(map(float, size.split()))
            prims.append({"name": name, "type": "BOX", "kind": "visual_only",
                          "centre": p[:3], "half_extents": [v / 2 for v in s],
                          "yaw": p[5]})
            n_vis += 1
    return prims, n_json, n_vis


def clearance(points, prims):
    pts = np.atleast_2d(np.asarray(points, float))
    c = _clearance_batch(pts, prims)
    return np.minimum(c, pts[:, 2] - GROUND_Z + (PLAN_CLEARANCE_M - MIN_AGL_M))


def segment_min_clearance(a, b, prims, step=0.25):
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(math.ceil(np.linalg.norm(b - a) / step)) + 1)
    return float(clearance(np.linspace(a, b, n), prims).min())


def slope_ok(a, b):
    d = np.asarray(b, float) - np.asarray(a, float)
    dxy = math.hypot(d[0], d[1])
    return dxy < 0.05 or math.degrees(math.atan2(abs(d[2]), dxy)) <= MAX_SLOPE_DEG


def seg_ok(a, b, prims):
    # relax to an endpoint's own clearance only down to the tier-3 floor
    # (3.2 m) -- a relaxation meant for real waypoints must never let an
    # intermediate corner sitting inside structure lower its own bar
    need = max(TIER3_CLEARANCE_M, min(PLAN_CLEARANCE_M, _endclear(a, b, prims))) - 1e-6
    return slope_ok(a, b) and segment_min_clearance(a, b, prims) >= need


def line_of_sight(a, target, prims, stop_short=0.3):
    a, t = np.asarray(a, float), np.asarray(target, float)
    d = np.linalg.norm(t - a)
    if d <= stop_short:
        return True
    b = a + (t - a) * (d - stop_short) / d
    n = max(2, int(math.ceil((d - stop_short) / 0.1)) + 1)
    return float(_clearance_batch(np.linspace(a, b, n), prims).min()) > 0.02


def fib_dirs(n=400):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    th = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], 1)


# Nudge tiers, tried in order; the tier used is recorded per waypoint.
TIER3_CLEARANCE_M = 3.2
NUDGE_TIERS = [(PLAN_CLEARANCE_M, 10.0), (PLAN_CLEARANCE_M, 20.0), (TIER3_CLEARANCE_M, 20.0)]


def nudge(pos, target, prims, need=PLAN_CLEARANCE_M, max_r=10.0):
    pos = np.asarray(pos, float)
    standoff = float(np.linalg.norm(np.asarray(target) - pos))
    dirs = fib_dirs()
    for r in np.arange(0.25, max_r + 0.01, 0.25):
        cand = pos + r * dirs
        ok = clearance(cand, prims) >= need
        if not ok.any():
            continue
        good = cand[ok]
        order = np.argsort(np.abs(np.linalg.norm(good - np.asarray(target), axis=1) - standoff))
        for k in order:
            if line_of_sight(good[k], target, prims):
                return good[k], float(r)
    return None, None



# ---------------------------------------------------------------------------
# Tier 4 (fallback): a waypoint no position can serve (tiers 1-3 all fail)
# is re-examined patch by patch. Each of its member's patches (regenerated
# exactly as coverage_final.py lays them out, from the mission's own sensor
# and sampling parameters) is classed as
#   embedded       -- the patch point lies inside another solid: no camera
#                     anywhere can see it (needs a manual/contact inspection)
#   neighbour      -- already in view from the previous/next waypoint
#   needs_view     -- exposed but unseen: search a viewpoint for it
# The waypoint is re-targeted to the first needs_view patch that has a
# viewpoint (level camera, HFOVxVFOV, range <= FALLBACK_MAX_RANGE_M, clear
# line of sight, clearance >= 3.5 m else 3.2 m, AGL >= MIN_AGL_M).
# ---------------------------------------------------------------------------
FALLBACK_MAX_RANGE_M = 12.0
FALLBACK_MAX_INCIDENCE_DEG = 75.0


def prim_patches(prim, cov):
    """(patch_id, point, outward normal) -- coverage_final._build_patches'
    layout (grid of cell centres per face / per cylinder side band)."""
    hf, vf = math.radians(cov["sensor"]["hfov_deg"]), math.radians(cov["sensor"]["vfov_deg"])
    step_u = 2 * cov["standoff_m"] * math.tan(hf / 2) * (1 - cov["face_overlap_frac"])
    step_v = 2 * cov["standoff_m"] * math.tan(vf / 2) * (1 - cov["face_overlap_frac"])
    cap = cov["max_samples_per_axis"]

    def grid(h, step):
        n = min(cap, max(1, math.ceil(2 * h / step)))
        return [0.0] if n == 1 else [-h + 2 * h * (k + 0.5) / n for k in range(n)]

    def ground(pt, nrm):
        return nrm[2] < -0.5 and pt[2] <= GROUND_Z + 0.05
    out, pid, c = [], 0, np.array(prim["centre"], float)
    if prim["type"] == "BOX":
        he, yaw = prim["half_extents"], prim.get("yaw", 0.0)
        ex = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        ey = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        ez = np.array([0.0, 0.0, 1.0])
        faces = {"+X": (ex, ey, ez, he[1], he[2], c + ex * he[0]), "-X": (-ex, ey, ez, he[1], he[2], c - ex * he[0]),
                 "+Y": (ey, ex, ez, he[0], he[2], c + ey * he[1]), "-Y": (-ey, ex, ez, he[0], he[2], c - ey * he[1]),
                 "+Z": (ez, ex, ey, he[0], he[1], c + ez * he[2]), "-Z": (-ez, ex, ey, he[0], he[1], c - ez * he[2])}
        for fname, (nrm, ua, va, hu, hv, fc) in faces.items():
            for u in grid(hu, step_u):
                for v in grid(hv, step_v):
                    pt = fc + ua * u + va * v
                    if not ground(pt, nrm):
                        out.append((f"{prim['name']}_{fname}_{pid}", pt, nrm))
                        pid += 1
    else:
        r, hh = prim["radius"], prim["half_height"]
        n_th = min(cap * 2, max(4, math.ceil(2 * math.pi / (step_u / max(r, 0.05)))))
        for k in range(n_th):
            th = 2 * math.pi * k / n_th
            nrm = np.array([math.cos(th), math.sin(th), 0.0])
            for z in grid(hh, step_v):
                pt = c + nrm * r + np.array([0, 0, z])
                if not ground(pt, nrm):
                    out.append((f"{prim['name']}_side_{pid}", pt, nrm))
                    pid += 1
        for cname, nrm in (("bottom", np.array([0, 0, -1.0])), ("top", np.array([0, 0, 1.0]))):
            pt = c + nrm * hh
            if not ground(pt, nrm):
                out.append((f"{prim['name']}_cap_{cname}", pt, nrm))
    return out


def camera_sees(cam, yaw, pt, nrm, prims, cov):
    """Mission sensor model (level camera, HFOV x VFOV about `yaw`)."""
    v = np.asarray(pt, float) - np.asarray(cam, float)
    rng = float(np.linalg.norm(v))
    if rng > FALLBACK_MAX_RANGE_M or rng < 1e-6:
        return False, None
    inc = math.degrees(math.acos(max(-1.0, min(1.0, float(-v @ nrm) / rng))))
    x = math.cos(yaw) * v[0] + math.sin(yaw) * v[1]
    y = -math.sin(yaw) * v[0] + math.cos(yaw) * v[1]
    az = math.degrees(math.atan2(y, x))
    el = math.degrees(math.atan2(v[2], math.hypot(x, y)))
    ok = (x > 0 and abs(az) <= cov["sensor"]["hfov_deg"] / 2 and abs(el) <= cov["sensor"]["vfov_deg"] / 2
          and inc <= FALLBACK_MAX_INCIDENCE_DEG and line_of_sight(cam, pt, prims, stop_short=0.05))
    return ok, {"range_m": round(rng, 2), "az_deg": round(az, 1), "el_deg": round(el, 1), "incidence_deg": round(inc, 1)}


def fallback_viewpoint(pt, nrm, prims, cov):
    """Legal viewpoint for one patch (camera heading aimed at it): among
    views with incidence <= 30 deg, the one closest to the mission standoff;
    None if no legal view exists."""
    half_v = cov["sensor"]["vfov_deg"] / 2
    for need in (PLAN_CLEARANCE_M, TIER3_CLEARANCE_M):
        cands = []
        base = math.atan2(nrm[1], nrm[0]) if abs(nrm[2]) < 0.9 else 0.0
        for R in np.arange(3.0, FALLBACK_MAX_RANGE_M + 0.01, 0.5):
            for daz in range(-75, 76, 5):
                for dep in range(0, int(half_v) + 1, 2):   # camera level, patch below by <= VFOV/2
                    for sgn in (1, -1):                   # viewpoint above or below the patch
                        a, e = base + math.radians(daz), math.radians(dep) * sgn
                        cam = np.asarray(pt) + R * np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
                        if cam[2] < MIN_AGL_M or float(clearance(cam, prims)[0]) < need:
                            continue
                        yaw = math.atan2(pt[1] - cam[1], pt[0] - cam[0])
                        ok, geo = camera_sees(cam, yaw, pt, nrm, prims, cov)
                        if ok:
                            cands.append((geo["incidence_deg"], R, cam, yaw, geo))
        if cands:
            # prefer the mission's own standoff among near-square-on views
            good = [c_ for c_ in cands if c_[0] <= 30.0] or cands
            inc, R, cam, yaw, geo = min(good, key=lambda t: (abs(t[1] - cov["standoff_m"]), t[0]))
            return {"position_m": [round(float(v), 4) for v in cam], "heading_rad": round(yaw, 4),
                    "clearance_need_m": need, "view": geo, "n_candidates": len(cands)}
    return None


def tier4_fallback(wps, idx, prims, cov):
    rec = wps[idx]
    jprims = {p["name"]: p for p in json.load(open(COLLISION))["primitives"]}
    prim = jprims[rec["prim_name"]]
    solids = [p for n, p in jprims.items() if n != rec["prim_name"] and p["kind"] not in ("ground", "water")]
    covers = set(rec["covers"])
    acct = {"embedded": [], "neighbour": {}, "needs_view": [], "fallback": None}
    patches = [q for q in prim_patches(prim, cov) if q[0] in covers]
    acct["patches_regenerated"] = len(patches)
    acct["patches_in_covers"] = len(covers)
    nbrs = [wps[j] for j in (idx - 1, idx + 1) if 0 <= j < len(wps) and wps[j]["reachable_in_plan"]]
    for pid, pt, nrm in patches:
        cl = float(_clearance_batch(pt[None], solids)[0])
        if cl < 0:
            acct["embedded"].append({"patch": pid, "depth_inside_m": round(-cl, 3)})
            continue
        seen = None
        for nb in nbrs:
            ok, geo = camera_sees(nb["position_m"], nb["heading_rad"], pt, nrm, prims, cov)
            if ok:
                seen = {"by": nb["waypoint_id"], **geo}
                break
        if seen:
            acct["neighbour"][pid] = seen
        else:
            acct["needs_view"].append((pid, pt, nrm))
    for pid, pt, nrm in acct["needs_view"]:
        vp = fallback_viewpoint(pt, nrm, prims, cov)
        if vp:
            acct["fallback"] = {"patch": pid, "target_m": [round(float(v), 4) for v in pt], **vp}
            break
    acct["needs_view"] = [q[0] for q in acct["needs_view"]]
    return acct


class Grid:
    def __init__(self, prims):
        self.dims = np.ceil((GRID_HI - GRID_LO) / VOXEL_M).astype(int) + 1
        t0 = time.time()
        axes = [GRID_LO[i] + np.arange(self.dims[i]) * VOXEL_M for i in range(3)]
        X, Y, Z = np.meshgrid(*axes, indexing="ij")
        pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
        self.free = (clearance(pts, prims) >= PLAN_CLEARANCE_M).reshape(self.dims)
        print(f"  occupancy grid {tuple(self.dims)} built in {time.time()-t0:.1f} s, "
              f"{100*self.free.mean():.1f}% free", flush=True)

    def to_g(self, p):
        return tuple(np.clip(np.round((np.asarray(p) - GRID_LO) / VOXEL_M).astype(int), 0, self.dims - 1))

    def to_w(self, g):
        return GRID_LO + np.asarray(g) * VOXEL_M

    def nearest_free(self, p, prims, max_r=4):
        """Nearest free cell reachable from p by a vertical piece then a
        horizontal piece (both clearance-checked). Returns (cell, [corner])."""
        p = np.asarray(p, float)
        g0 = np.array(self.to_g(p))
        cands = []
        for dx in range(-max_r, max_r + 1):
            for dy in range(-max_r, max_r + 1):
                for dz in range(-max_r, max_r + 1):
                    g = tuple(g0 + (dx, dy, dz))
                    if all(0 <= g[i] < self.dims[i] for i in range(3)) and self.free[g]:
                        cands.append((float(np.linalg.norm(self.to_w(g) - p)), g))
        for _, g in sorted(cands):
            w = self.to_w(g)
            corner = np.array([p[0], p[1], w[2]])
            pieces = [p, corner, w] if abs(corner[2] - p[2]) > 1e-6 else [p, w]
            if all(seg_ok(a, b, prims) for a, b in zip(pieces[:-1], pieces[1:])):
                return g, pieces[1:-1]
        return None, None

    def astar(self, s, e, max_exp=2_000_000):
        nb = [(dx, dy, 0) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)] \
            + [(0, 0, 1), (0, 0, -1)]   # horizontal or vertical steps only
        cost = {d: math.sqrt(d[0]**2 + d[1]**2 + d[2]**2) for d in nb}
        ea = np.array(e)
        h = lambda g: float(np.linalg.norm(np.array(g) - ea))  # noqa: E731
        openh = [(h(s), 0.0, s)]
        g_s = {s: 0.0}
        came = {}
        closed = set()
        n = 0
        while openh and n < max_exp:
            _, gc, cur = heapq.heappop(openh)
            if cur in closed:
                continue
            if cur == e:
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                return path[::-1]
            closed.add(cur)
            n += 1
            for d in nb:
                q = (cur[0] + d[0], cur[1] + d[1], cur[2] + d[2])
                if not (0 <= q[0] < self.dims[0] and 0 <= q[1] < self.dims[1] and 0 <= q[2] < self.dims[2]):
                    continue
                if q in closed or not self.free[q]:
                    continue
                t = gc + cost[d]
                if t < g_s.get(q, math.inf):
                    g_s[q] = t
                    came[q] = cur
                    heapq.heappush(openh, (t + h(q), t, q))
        return None


def prune(points, prims):
    """Greedy line-of-sight shortcutting at the full planning clearance."""
    out = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not seg_ok(points[i], points[j], prims):
            j -= 1
        out.append(points[j])
        i = j
    return out


def _endclear(a, b, prims):
    return float(clearance(np.array([a, b], float), prims).min())


def route(a, b, prims, grid):
    """Returns (via_points_excluding_a_and_b, method, min_clearance)."""
    # endpoints may be tier-3 (3.2 m) waypoints; the leg itself is judged
    # by its interior, the part the aircraft transits at speed
    a, b = np.asarray(a, float), np.asarray(b, float)
    if seg_ok(a, b, prims):
        return [], "straight", segment_min_clearance(a, b, prims)
    for corner, name in ((np.array([a[0], a[1], b[2]]), "L_vertical_first"),
                         (np.array([b[0], b[1], a[2]]), "L_horizontal_first")):
        if seg_ok(a, corner, prims) and seg_ok(corner, b, prims):
            return [corner.tolist()], name, min(segment_min_clearance(a, corner, prims),
                                                segment_min_clearance(corner, b, prims))
    ga, ca = grid.nearest_free(a, prims)
    gb, cb = grid.nearest_free(b, prims)
    if ga is None or gb is None:
        return None, "no_free_cell_near_endpoint", None
    path = grid.astar(ga, gb)
    if path is None:
        return None, "astar_failed", None
    pts = [a] + ca + [grid.to_w(g) for g in path] + cb[::-1] + [b]
    pts = prune(pts, prims)
    mins = [segment_min_clearance(p, q, prims) for p, q in zip(pts[:-1], pts[1:])]
    return [list(map(float, p)) for p in pts[1:-1]], "astar", float(min(mins))


def main():
    t0 = time.time()
    cov = json.load(open(COVERAGE))
    prims, n_json, n_vis = load_obstacles()
    print(f"obstacles: {n_json} collision-JSON primitives (ground/water excluded, "
          f"ground handled as z>={MIN_AGL_M}) + {n_vis} Gazebo visual-only boxes", flush=True)

    wps = []
    n_nudged = 0
    unfixable = []
    for w in cov["waypoints"]:
        pos = np.asarray(w["position_m"], float)
        c0 = float(clearance(pos, prims)[0])
        rec = {k: w[k] for k in ("waypoint_id", "prim_name", "prim_kind", "target_m", "covers")}
        rec["original_position_m"] = w["position_m"]
        rec["original_heading_rad"] = w["heading_rad"]
        rec["original_clearance_m"] = round(c0, 3)
        tier = 0
        new, r = (pos, 0.0) if c0 >= PLAN_CLEARANCE_M else (None, None)
        if new is None:
            for tier, (need, max_r) in enumerate(NUDGE_TIERS, 1):
                new, r = nudge(pos, w["target_m"], prims, need, max_r)
                if new is not None:
                    break
        rec["nudge_tier"] = tier if new is not None else None
        rec["reachable_in_plan"] = new is not None
        if new is None:
            unfixable.append(w["waypoint_id"])
            new, r = pos, None
        rec["nudged"] = bool(r)
        rec["nudge_m"] = None if r is None else round(r, 3)
        n_nudged += bool(r)
        rec["position_m"] = [round(float(v), 4) for v in new]
        tgt = np.asarray(w["target_m"], float)
        dxy = tgt[:2] - new[:2]
        # keep the planner's own heading unless the nudge moved us enough
        # that the target's bearing actually changed
        rec["heading_rad"] = (round(math.atan2(dxy[1], dxy[0]), 4)
                              if r and np.linalg.norm(dxy) >= 1.0 else w["heading_rad"])
        rec["planned_clearance_m"] = round(float(clearance(new, prims)[0]), 3)
        rec["standoff_to_target_m"] = round(float(np.linalg.norm(tgt - new)), 3)
        wps.append(rec)
    print(f"waypoints: {len(wps)}, nudged {n_nudged}, unfixable {len(unfixable)} {unfixable}", flush=True)

    # tier 4: patch-level fallback for waypoints no position can serve
    tier4 = {}
    for i, rec in enumerate(wps):
        if rec["reachable_in_plan"]:
            continue
        acct = tier4_fallback(wps, i, prims, cov)
        rec["patch_accounting"] = acct
        fb = acct["fallback"]
        print(f"  TIER4 {rec['waypoint_id']}: {len(acct['embedded'])} embedded, "
              f"{len(acct['neighbour'])} seen by neighbour, needs_view {acct['needs_view']}, "
              f"fallback {'-> ' + fb['patch'] + ' from ' + str(fb['position_m']) if fb else 'none'}", flush=True)
        if fb:
            rec["original_target_m"] = rec["target_m"]
            rec["target_m"] = fb["target_m"]
            rec["position_m"] = fb["position_m"]
            rec["heading_rad"] = fb["heading_rad"]
            rec["nudge_tier"] = 4
            rec["reachable_in_plan"] = True
            rec["nudged"] = True
            rec["nudge_m"] = round(float(np.linalg.norm(np.subtract(fb["position_m"], rec["original_position_m"]))), 3)
            rec["planned_clearance_m"] = round(float(clearance(fb["position_m"], prims)[0]), 3)
            rec["standoff_to_target_m"] = fb["view"]["range_m"]
            unfixable.remove(rec["waypoint_id"])
            tier4[rec["waypoint_id"]] = fb["patch"]
        else:
            rec["limitation"] = ("no exposed, unseen patch has a legal viewpoint; embedded patches need a "
                                 "manual or contact inspection")

    grid = Grid(prims)
    home_ground = [HOME_WORLD[0], HOME_WORLD[1], 0.0]
    home_air = [HOME_WORLD[0], HOME_WORLD[1], TAKEOFF_ALT_M]
    prev = home_air
    legs = {"straight": 0, "L_vertical_first": 0, "L_horizontal_first": 0, "astar": 0, "failed": 0}
    min_leg_clear = math.inf
    total = TAKEOFF_ALT_M
    for rec in wps:
        if not rec["reachable_in_plan"]:
            rec.update(route_in=[], route_method="skipped_unreachable_in_plan",
                       route_min_clearance_m=None, route_length_m=0.0)
            print(f"  SKIP {rec['waypoint_id']}: no position with >=3.2 m clearance and "
                  f"line of sight to its target within 20 m", flush=True)
            continue
        via, method, mc = route(prev, rec["position_m"], prims, grid)
        if via is None:
            legs["failed"] += 1
            via = []
            print(f"  ROUTE FAIL {rec['waypoint_id']}: {method}", flush=True)
            rec["reachable_in_plan"] = False
        else:
            legs[method] += 1
            min_leg_clear = min(min_leg_clear, mc)
        rec["route_in"] = via
        rec["route_method"] = method
        rec["route_min_clearance_m"] = None if mc is None else round(mc, 3)
        pts = [prev] + via + [rec["position_m"]]
        rec["route_length_m"] = round(sum(float(np.linalg.norm(np.subtract(q, p)))
                                          for p, q in zip(pts[:-1], pts[1:])), 3)
        total += rec["route_length_m"]
        prev = rec["position_m"]
    rth_via, rth_method, rth_mc = route(prev, home_air, prims, grid)
    pts = [prev] + (rth_via or []) + [home_air]
    rth_len = sum(float(np.linalg.norm(np.subtract(q, p))) for p, q in zip(pts[:-1], pts[1:]))
    total += rth_len + TAKEOFF_ALT_M

    # independent re-check of every segment that will be flown
    n_seg = n_bad = 0
    raw_min = math.inf
    prev = home_air
    for rec in wps:
        if not rec["reachable_in_plan"]:
            continue
        pts = [prev] + rec["route_in"] + [rec["position_m"]]
        for p_, q_ in zip(pts[:-1], pts[1:]):
            n_seg += 1
            n_bad += not seg_ok(p_, q_, prims)
            raw_min = min(raw_min, segment_min_clearance(p_, q_, prims))
        prev = rec["position_m"]
    pts = [prev] + (rth_via or []) + [home_air]
    for p_, q_ in zip(pts[:-1], pts[1:]):
        n_seg += 1
        n_bad += not seg_ok(p_, q_, prims)
        raw_min = min(raw_min, segment_min_clearance(p_, q_, prims))
    print(f"segment re-check: {n_seg} segments, {n_bad} violate slope/clearance, "
          f"raw min clearance over every flown segment {raw_min:.3f} m", flush=True)

    plan = {
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "planner": "gazebo/mission_follower/plan_gazebo_mission.py (offline; geometry only, no defect data)",
        "source_mission": "mission/coverage_mission.json",
        "frame": "Gazebo world ENU == Blender/collision-JSON frame (no origin shift; verified primitive-by-primitive)",
        "sensed_clearance_m": SENSED_CLEARANCE_M,
        "plan_clearance_m": PLAN_CLEARANCE_M,
        "min_agl_m": MIN_AGL_M,
        "home_ground_world_m": home_ground,
        "home_air_world_m": home_air,
        "n_obstacles_collision_json": n_json,
        "n_obstacles_visual_only": n_vis,
        "n_waypoints": len(wps),
        "n_nudged": n_nudged,
        "unfixable_waypoints": unfixable,
        "tier4_fallback": tier4,
        "legs": legs,
        "max_slope_deg": MAX_SLOPE_DEG,
        "segment_recheck": {"n_segments": n_seg, "n_violations": n_bad,
                            "raw_min_clearance_m": round(raw_min, 3)},
        "min_route_clearance_m": round(min_leg_clear, 3),
        "rth": {"route": rth_via or [], "method": rth_method,
                "min_clearance_m": None if rth_mc is None else round(rth_mc, 3),
                "length_m": round(rth_len, 3)},
        "planned_total_path_m": round(total, 2),
        "waypoints": wps,
    }
    json.dump(plan, open(OUT, "w"), indent=1)
    print(f"legs {legs}, min route clearance {min_leg_clear:.3f} m, RTH {rth_method} "
          f"{rth_len:.1f} m, total planned path {total:.1f} m", flush=True)
    print(f"wrote {OUT} in {time.time()-t0:.0f} s")


if __name__ == "__main__":
    main()
