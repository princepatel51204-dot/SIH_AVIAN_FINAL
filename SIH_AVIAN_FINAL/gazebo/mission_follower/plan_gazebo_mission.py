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
