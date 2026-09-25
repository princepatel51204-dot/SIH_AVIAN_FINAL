#!/usr/bin/env python3
"""SIH_AVIAN_FINAL -- Task 1 v5: coverage-driven viewpoint planning.

Stages (each cached under cache/):
  geometry   rebuilt surface patches + exclusion reasons (covlib)
  cands      candidate viewpoints on offset layers 4/6/8 m from surfaces,
             camera aimed at a seed patch; legal = >=3.5 m clearance to every
             solid (collision JSON + Gazebo-only boxes), >=3.5 m AGL, inside the
             free space connected to home (same 1 m grid the router uses)
  vis        visible patch set per candidate (real camera model, ray-cast LOS)
  cover      area-weighted greedy set cover to the target
  route      structure-by-structure ordering + flat/vertical routing (plan JSON)

Camera models:
  fixed   the current airframe: mount (0.15, 0, -0.02), pitch 22.9 deg down
  gimbal  proposed pitch gimbal on a front boom: mount (0.35, 0, 0.05),
          pitch -90..+90 deg chosen per viewpoint
Usage: plan_v5.py <stage> <camera> [options]
"""
import argparse
import json
import math
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import covlib as C  # noqa: E402
import numpy as np  # noqa: E402

CACHE = os.path.join(HERE, "cache")
os.makedirs(CACHE, exist_ok=True)
CAMS = {
    "fixed": {"mount": np.array([0.15, 0.0, -0.02]), "pitch": (-C.FIXED_PITCH_DOWN, -C.FIXED_PITCH_DOWN)},
    "gimbal": {"mount": np.array([0.35, 0.0, 0.05]), "pitch": (-math.pi / 2, math.pi / 2)},
}
PLAN_CLEARANCE_M = 3.5
MIN_AGL_M = 3.5
OFFSETS_M = (4.0, 6.0, 8.0)
SEED_CELL_M = 2.0
HOME_AIR = np.array([20.0, -30.0, 5.0])
GRID_LO = np.array([-25.0, -45.0, 0.0])
GRID_HI = np.array([385.0, 55.0, 40.0])


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


# ------------------------------------------------------------ geometry ---
def geometry():
    f = os.path.join(CACHE, "geometry.npz")
    S = C.Solids()
    if os.path.exists(f):
        g = dict(np.load(f, allow_pickle=True))
        return S, g
    pt = C.build_patches(S, 1.0)
    reason, sd = C.classify_exclusions(S, pt)
    comp = np.array([C.component_of(S.kinds[k], S.names[k], nz) for k, nz in zip(pt["solid"], pt["n"][:, 2])])
    fam = np.array([S.names[k].split("_")[0] for k in pt["solid"]])
    g = {**pt, "reason": reason, "comp": comp, "fam": fam}
    np.savez(f, **g)
    return S, g


# ---------------------------------------------------------- free space ---
def free_space(S):
    f = os.path.join(CACHE, "freespace.npz")
    if os.path.exists(f):
        d = np.load(f)
        return d["free"], d["reach"]
    dims = np.ceil((GRID_HI - GRID_LO)).astype(int) + 1
    axes = [GRID_LO[i] + np.arange(dims[i]) for i in range(3)]
    X, Y, Z = np.meshgrid(*axes, indexing="ij")
    pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
    free = (C.fast_clearance(S, pts) >= PLAN_CLEARANCE_M).reshape(dims)
    from scipy import ndimage
    st = np.zeros((3, 3, 3), bool)
    st[:, :, 1] = True          # 8 horizontal neighbours
    st[1, 1, :] = True          # + vertical: the router's own move set
    lab, _ = ndimage.label(free, structure=st)
    h = tuple(np.round(HOME_AIR - GRID_LO).astype(int))
    reach = lab == lab[h]
    np.savez(f, free=free, reach=reach)
    return free, reach


def in_reach(reach, pts):
    g = np.round((pts - GRID_LO)).astype(int)
    ok = np.all((g >= 0) & (g < np.array(reach.shape)), axis=1)
    out = np.zeros(len(pts), bool)
    gi = g[ok]
    out[ok] = reach[gi[:, 0], gi[:, 1], gi[:, 2]]
    return out


# ---------------------------------------------------------- candidates ---
def _dirs(n):
    """Offset directions for one seed normal: the normal, a 35 deg ring of 6,
    and the normal raised 23 / 45 deg (views from above the seed)."""
    a = np.array([0, 0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0, 0])
    t1 = np.cross(n, a)
    t1 /= np.linalg.norm(t1)
    t2 = np.cross(n, t1)
    out = [n]
    for k in range(6):
        ph = 2 * math.pi * k / 6
        out.append(math.cos(math.radians(35)) * n + math.sin(math.radians(35)) * (math.cos(ph) * t1 + math.sin(ph) * t2))
    if abs(n[2]) < 0.7:
        h = n.copy()
        h[2] = 0
        h /= np.linalg.norm(h)
        for el in (23.0, 45.0):
            out.append(math.cos(math.radians(el)) * h + math.sin(math.radians(el)) * np.array([0, 0, 1.0]))
    return out


def candidates(S, g, cam):
    f = os.path.join(CACHE, f"cands_{cam}.npz")
    if os.path.exists(f):
        return dict(np.load(f))
    free, reach = free_space(S)
    vis = np.flatnonzero(g["reason"] == 0)
    key = {}
    for i in vis:
        p, n = g["p"][i], g["n"][i]
        k = (tuple(np.floor(p / SEED_CELL_M).astype(int)), tuple(np.round(n, 1)))
        key.setdefault(k, i)
    seeds = np.array(sorted(key.values()))
    log(f"{cam}: {len(seeds)} seeds from {len(vis)} visible patches")
    mount = CAMS[cam]["mount"]
    pmin, pmax = CAMS[cam]["pitch"]
    rows = []
    for s in seeds:
        p, n = g["p"][s], g["n"][s]
        for u in _dirs(n):
            for d in OFFSETS_M:
                cam_p = p + d * u
                v = p - cam_p
                yaw = math.atan2(v[1], v[0]) if math.hypot(v[0], v[1]) > 1e-6 else math.atan2(-n[1], -n[0]) if math.hypot(n[0], n[1]) > 1e-6 else 0.0
                el = math.atan2(v[2], math.hypot(v[0], v[1]))
                if pmin == pmax:
                    pitch = pmin
                    # seed must lie inside the fixed camera's vertical field
                    if abs(el - pitch) >= math.pi / 2 or abs(math.tan(el - pitch)) > C.TAN_V:
                        continue
                else:
                    pitch = min(max(el, pmin), pmax)
                cy, sy = math.cos(yaw), math.sin(yaw)
                base = cam_p - np.array([cy * mount[0] - sy * mount[1], sy * mount[0] + cy * mount[1], mount[2]])
                rows.append((*base, *cam_p, yaw, pitch, s))
    a = np.array(rows)
    base = a[:, 0:3]
    ok = base[:, 2] >= MIN_AGL_M
    ok[ok] = C.fast_clearance(S, base[ok]) >= PLAN_CLEARANCE_M
    ok[ok] = in_reach(reach, base[ok])
    a = a[ok]
    out = {"base": a[:, 0:3], "cam": a[:, 3:6], "yaw": a[:, 6], "pitch": a[:, 7], "seed": a[:, 8].astype(np.int64)}
    np.savez(f, **out)
    log(f"{cam}: {len(rows)} raw candidates -> {len(a)} legal (clearance/AGL/reachable)")
    return out


# ----------------------------------------------------------- visibility ---
_W = {}


def _winit(cam):
    import numba
    numba.set_num_threads(1)
    S, g = geometry()
    _W["V"] = C.Visibility(S, g, g["reason"] == 0)
    _W["c"] = candidates(S, g, cam)


def _wrun(rng):
    V, c = _W["V"], _W["c"]
    out = []
    for i in range(*rng):
        f, l, u = C.cam_axes(c["yaw"][i], c["pitch"][i])
        out.append(V.visible(c["cam"][i], f, l, u).astype(np.int32))
    return rng[0], out


def visibility(cam, workers=14):
    f = os.path.join(CACHE, f"vis_{cam}.npz")
    if os.path.exists(f):
        d = np.load(f)
        return d["ptr"], d["ids"]
    S, g = geometry()
    c = candidates(S, g, cam)
    n = len(c["yaw"])
    chunks = [(i, min(n, i + 500)) for i in range(0, n, 500)]
    res = [None] * n
    t = time.time()
    with mp.get_context("fork").Pool(workers, initializer=_winit, initargs=(cam,)) as pool:
        for k, (s0, lst) in enumerate(pool.imap_unordered(_wrun, chunks)):
            res[s0:s0 + len(lst)] = lst
            if k % 20 == 0:
                log(f"  vis {cam}: {k + 1}/{len(chunks)} chunks, {time.time() - t:.0f} s")
    ptr = np.zeros(n + 1, np.int64)
    ptr[1:] = np.cumsum([len(r) for r in res])
    ids = np.concatenate(res) if n else np.zeros(0, np.int32)
    np.savez(f, ptr=ptr, ids=ids)
    log(f"vis {cam}: {n} candidates, mean {ptr[-1] / max(n, 1):.0f} patches each, {time.time() - t:.0f} s")
    return ptr, ids


# --------------------------------------------------------------- cover ---
def greedy(area, ptr, ids, target_area, min_gain=0.5):
    import heapq
    covered = np.zeros(len(area), bool)
    gains = np.array([area[ids[ptr[i]:ptr[i + 1]]].sum() for i in range(len(ptr) - 1)])
    heap = [(-gains[i], i) for i in range(len(gains)) if gains[i] > 0]
    heapq.heapify(heap)
    chosen, got = [], 0.0
    while heap and got < target_area:
        ng, i = heapq.heappop(heap)
        s = ids[ptr[i]:ptr[i + 1]]
        real = area[s[~covered[s]]].sum()
        if real < min_gain:
            continue
        if heap and real < -heap[0][0] - 1e-9:
            heapq.heappush(heap, (-real, i))
            continue
        chosen.append(i)
        covered[s] = True
        got += real
    return chosen, covered


def prune_redundant(area, ptr, ids, chosen, min_unique=1.0):
    """Drop viewpoints whose unique contribution is below min_unique m2."""
    cnt = np.zeros(len(area), np.int32)
    for i in chosen:
        cnt[ids[ptr[i]:ptr[i + 1]]] += 1
    keep = list(chosen)
    changed = True
    while changed:
        changed = False
        uniq = [(area[ids[ptr[i]:ptr[i + 1]]][cnt[ids[ptr[i]:ptr[i + 1]]] == 1].sum(), i) for i in keep]
        uniq.sort()
        if uniq and uniq[0][0] < min_unique:
            _, i = uniq[0]
            keep.remove(i)
            cnt[ids[ptr[i]:ptr[i + 1]]] -= 1
            changed = True
    return keep


def cover(cam, target=0.90):
    S, g = geometry()
    ptr, ids = visibility(cam)
    area = g["area"]
    vis = g["reason"] == 0
    A = area[vis].sum()
    ceil_mask = np.zeros(len(area), bool)
    ceil_mask[ids] = True
    ceiling = area[ceil_mask].sum()
    chosen, covered = greedy(area, ptr, ids, min(target * A, ceiling) + 1e-6)
    kept = prune_redundant(area, ptr, ids, chosen)
    cov = np.zeros(len(area), bool)
    for i in kept:
        cov[ids[ptr[i]:ptr[i + 1]]] = True
    res = {"camera": cam, "visible_area_m2": round(float(A), 1), "ceiling_area_m2": round(float(ceiling), 1),
           "ceiling_pct": round(100 * ceiling / A, 2), "target_pct": 100 * target,
           "n_greedy": len(chosen), "n_after_prune": len(kept),
           "covered_area_m2": round(float(area[cov].sum()), 1), "covered_pct": round(100 * area[cov].sum() / A, 2),
           "by_component": {}}
    for c in sorted(set(g["comp"])):
        m = vis & (g["comp"] == c)
        res["by_component"][str(c)] = {"visible_m2": round(float(area[m].sum()), 1),
                                       "ceiling_pct": round(100 * area[m & ceil_mask].sum() / max(area[m].sum(), 1e-9), 1),
                                       "covered_pct": round(100 * area[m & cov].sum() / max(area[m].sum(), 1e-9), 1)}
    np.save(os.path.join(CACHE, f"chosen_{cam}.npy"), np.array(kept))
    json.dump(res, open(os.path.join(CACHE, f"cover_{cam}.json"), "w"), indent=1)
    log(json.dumps(res, indent=1))
    return res


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=["geometry", "cands", "vis", "cover"])
    ap.add_argument("camera", nargs="?", default="gimbal", choices=list(CAMS))
    ap.add_argument("--target", type=float, default=0.90)
    a = ap.parse_args()
    if a.stage == "geometry":
        S, g = geometry()
        log(len(g["p"]), "patches")
    elif a.stage == "cands":
        S, g = geometry()
        candidates(S, g, a.camera)
    elif a.stage == "vis":
        visibility(a.camera)
    elif a.stage == "cover":
        cover(a.camera, a.target)
