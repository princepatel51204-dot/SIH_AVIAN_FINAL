"""SIH_AVIAN_FINAL -- Phase B step B2: A* detours for transit legs the
new, stricter clearance check rejects.

Reads no defect-taxonomy data of any kind: only `avian_bridge_collision.json`
(via `mission_final._load_collision_primitives()`, itself already clean)
and the two waypoint positions it is asked to connect.

Scope, stated plainly: a full corridor-wide 3D occupancy grid (360 m x
~14 m x ~20 m at 1 m voxels is ~100,000+ cells) searched from scratch for
every blocked leg would be slow repeated ~100+ times. Each query instead
builds a LOCAL grid bounded to a padded box around just that leg's own
start/end points -- a detour should stay near the original path, not
route through the far end of the bridge -- which keeps each A* search to
a few thousand cells. A hard expansion cap bounds worst-case runtime per
query; a query that exceeds it gives up and reports no detour found
(handled by the caller, not silently retried forever).
"""
from __future__ import annotations
import heapq
import math

import numpy as np

VOXEL_M = 1.0
PAD_M = 3.0            # extra room around the straight-line bounding box
MAX_EXPANSIONS = 60000
NEIGHBORS_26 = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
               for dz in (-1, 0, 1) if (dx, dy, dz) != (0, 0, 0)]


def _clearance_batch(points, primitives):
    """Vectorised signed clearance for an (N,3) array of points against
    every primitive -- the same geometry `mission_final._signed_clearance`
    computes one point at a time, batched with numpy so the dense sampling
    step B2 asks for (half the thinnest primitive's own dimension) is
    tractable instead of a per-point Python loop."""
    n = points.shape[0]
    best = np.full(n, np.inf)
    for p in primitives:
        c = np.array(p["centre"])
        if p["type"] == "BOX":
            he = np.array(p["half_extents"])
            yaw = p.get("yaw", 0.0)
            d = points - c
            ca, sa = math.cos(-yaw), math.sin(-yaw)
            lx = d[:, 0] * ca - d[:, 1] * sa
            ly = d[:, 0] * sa + d[:, 1] * ca
            lz = d[:, 2]
            ox = np.abs(lx) - he[0]
            oy = np.abs(ly) - he[1]
            oz = np.abs(lz) - he[2]
            inside = (ox < 0) & (oy < 0) & (oz < 0)
            outside_dist = np.sqrt(np.clip(ox, 0, None)**2 +
                                   np.clip(oy, 0, None)**2 +
                                   np.clip(oz, 0, None)**2)
            inside_dist = np.maximum(np.maximum(ox, oy), oz)
            dist = np.where(inside, inside_dist, outside_dist)
        else:
            r, hh = p["radius"], p["half_height"]
            oz = np.abs(points[:, 2] - c[2]) - hh
            oxy = np.hypot(points[:, 0] - c[0], points[:, 1] - c[1]) - r
            inside = (oz < 0) & (oxy < 0)
            outside_dist = np.sqrt(np.clip(oz, 0, None)**2 +
                                   np.clip(oxy, 0, None)**2)
            inside_dist = np.maximum(oz, oxy)
            dist = np.where(inside, inside_dist, outside_dist)
        best = np.minimum(best, dist)
    return best


def find_detour(start, end, primitives, margin, voxel_m=VOXEL_M,
                pad_m=PAD_M, max_expansions=MAX_EXPANSIONS):
    """A* on a local occupancy grid. Returns a list of Vector-compatible
    (x, y, z) tuples from `start` to `end` (inclusive) if a clear path
    exists, else None. `start`/`end` are mathutils.Vector or (x,y,z)."""
    lo = np.minimum(start, end) - pad_m
    hi = np.maximum(start, end) + pad_m
    dims = np.maximum(1, np.ceil((hi - lo) / voxel_m).astype(int))
    if np.prod(dims) > 400000:
        return None   # refuse a pathologically large local grid, not hang

    def to_grid(p):
        return tuple(np.clip(np.round((np.array(p) - lo) / voxel_m).astype(int),
                             0, dims - 1))

    def to_world(g):
        return lo + np.array(g) * voxel_m

    start_g, end_g = to_grid(start), to_grid(end)

    occupied_cache = {}

    def is_occupied(g):
        # The start/end cells are the inspection waypoints themselves --
        # deliberately allowed to sit closer than the transit margin on
        # purpose (the whole reason the endpoint check uses a much smaller
        # STRUCTURE_EMBED_MARGIN_M, not this transit one). Only the PATH
        # between them needs the larger buffer, so the endpoints are
        # exempted from occupancy here rather than failing every query
        # whose waypoints are close-in inspection shots by design.
        if g == start_g or g == end_g:
            return False
        if g in occupied_cache:
            return occupied_cache[g]
        world_pt = to_world(g).reshape(1, 3)
        clearance = _clearance_batch(world_pt, primitives)[0]
        occ = bool(clearance < margin)
        occupied_cache[g] = occ
        return occ

    def h(g):
        return float(np.linalg.norm(np.array(g) - np.array(end_g)))

    open_heap = [(h(start_g), 0.0, start_g)]
    came_from = {}
    g_score = {start_g: 0.0}
    visited = set()
    expansions = 0

    while open_heap and expansions < max_expansions:
        _, g_cost, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        expansions += 1
        if current == end_g:
            path_g = [current]
            while current in came_from:
                current = came_from[current]
                path_g.append(current)
            path_g.reverse()
            return [tuple(float(v) for v in to_world(g)) for g in path_g]
        for dx, dy, dz in NEIGHBORS_26:
            nb = (current[0] + dx, current[1] + dy, current[2] + dz)
            if not all(0 <= nb[i] < dims[i] for i in range(3)):
                continue
            if nb in visited or is_occupied(nb):
                continue
            step_cost = math.sqrt(dx * dx + dy * dy + dz * dz) * voxel_m
            tentative = g_score[current] + step_cost
            if tentative < g_score.get(nb, math.inf):
                g_score[nb] = tentative
                came_from[nb] = current
                heapq.heappush(open_heap, (tentative + h(nb), tentative, nb))
    return None   # exhausted the budget or the space -- no detour found
