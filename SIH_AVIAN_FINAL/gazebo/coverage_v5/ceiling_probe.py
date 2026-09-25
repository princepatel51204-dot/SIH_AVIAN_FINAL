#!/usr/bin/env python3
"""Is the coverage ceiling a sampling artefact or a physical limit?

For every visible patch that NO planning candidate sees, try a much denser
candidate set aimed straight at it (every patch a seed; normal + tilt rings
at 20/40/55 deg x 8 azimuths + raised variants; offsets 4-12 m), under
  legal     >= 3.5 m clearance, >= 3.5 m AGL, reachable free space (the rules)
  physical  >= 0.3 m clearance, >= 0.3 m AGL, air connected to the outside
            (any camera, any size of vehicle: the physics-only bound)
Gimbal camera (pitch -90..+90). Writes cache/ceiling_probe.json.
"""
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
import plan_v5 as PV  # noqa: E402

MOUNT = PV.CAMS["gimbal"]["mount"]


def dirs(n):
    a = np.array([0, 0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0, 0])
    t1 = np.cross(n, a)
    t1 /= np.linalg.norm(t1)
    t2 = np.cross(n, t1)
    out = [n]
    for tilt in (20, 40, 55):
        for k in range(8):
            ph = 2 * math.pi * k / 8
            out.append(math.cos(math.radians(tilt)) * n + math.sin(math.radians(tilt)) * (math.cos(ph) * t1 + math.sin(ph) * t2))
    return out


def free_space(S, clear):
    dims = np.ceil(PV.GRID_HI - PV.GRID_LO).astype(int) + 1
    axes = [PV.GRID_LO[i] + np.arange(dims[i]) for i in range(3)]
    X, Y, Z = np.meshgrid(*axes, indexing="ij")
    pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
    free = (C.fast_clearance(S, pts) >= clear).reshape(dims)
    from scipy import ndimage
    st = np.zeros((3, 3, 3), bool)
    st[:, :, 1] = True
    st[1, 1, :] = True
    lab, _ = ndimage.label(free, structure=st)
    # the component that touches the top of the box is "outside air"
    top = lab[:, :, -1]
    outside = np.bincount(top[top > 0].ravel()).argmax()
    return lab == outside


_W = {}


def _init():
    import numba
    numba.set_num_threads(1)
    S, g = PV.geometry()
    _W["V"] = C.Visibility(S, g, g["reason"] == 0)


def _run(job):
    V = _W["V"]
    out = []
    for cam, yaw, pitch in job:
        f, l, u = C.cam_axes(yaw, pitch)
        out.append(V.visible(cam, f, l, u))
    return np.unique(np.concatenate(out)) if out else np.zeros(0, np.int64)


def probe(S, g, targets, clear, offsets):
    reach = free_space(S, clear) if clear < PV.PLAN_CLEARANCE_M else PV.free_space(S)[1]
    rows = []
    for s in targets:
        p, n = g["p"][s], g["n"][s]
        for u in dirs(n):
            for d in offsets:
                cam = p + d * u
                v = p - cam
                yaw = math.atan2(v[1], v[0]) if math.hypot(v[0], v[1]) > 1e-6 else 0.0
                pitch = math.atan2(v[2], math.hypot(v[0], v[1]))
                cy, sy = math.cos(yaw), math.sin(yaw)
                base = cam - np.array([cy * MOUNT[0] - sy * MOUNT[1], sy * MOUNT[0] + cy * MOUNT[1], MOUNT[2]])
                rows.append((*base, *cam, yaw, pitch))
    a = np.array(rows)
    base = a[:, 0:3]
    ok = base[:, 2] >= clear
    ok[ok] = C.fast_clearance(S, base[ok]) >= clear
    ok[ok] = PV.in_reach(reach, base[ok])
    a = a[ok]
    jobs = [[(a[i, 3:6], a[i, 6], a[i, 7]) for i in range(j, min(len(a), j + 400))] for j in range(0, len(a), 400)]
    seen = set()
    with mp.get_context("fork").Pool(14, initializer=_init) as pool:
        for r in pool.imap_unordered(_run, jobs):
            seen.update(r.tolist())
    return len(rows), len(a), np.array(sorted(seen), np.int64)


def main():
    S, g = PV.geometry()
    area = g["area"]
    vis = g["reason"] == 0
    A = area[vis].sum()
    ptr, ids = PV.visibility("gimbal")
    base_cov = np.zeros(len(area), bool)
    base_cov[ids] = True
    unc = np.flatnonzero(vis & ~base_cov)
    print(f"visible {A:.0f} m2; planning-candidate ceiling {100 * area[base_cov].sum() / A:.2f}%; "
          f"uncovered {len(unc)} patches, {area[unc].sum():.0f} m2", flush=True)
    res = {"visible_m2": round(float(A), 1), "planning_ceiling_pct": round(100 * area[base_cov].sum() / A, 2)}
    for name, clear, offs in (("legal", 3.5, (4, 5, 6, 7, 8, 10, 12)), ("physical", 0.3, (1, 2, 3, 4, 6, 8, 10, 12))):
        t = time.time()
        nraw, nleg, seen = probe(S, g, unc, clear, offs)
        newly = np.zeros(len(area), bool)
        newly[seen] = True
        newly &= ~base_cov
        tot = base_cov | newly
        res[name] = {"clearance_m": clear, "offsets_m": list(offs), "raw_candidates": nraw, "legal_candidates": nleg,
                     "newly_coverable_m2": round(float(area[newly].sum()), 1),
                     "ceiling_pct": round(100 * area[tot].sum() / A, 2),
                     "by_component_uncoverable_m2": {}}
        for c in sorted(set(g["comp"])):
            m = vis & (g["comp"] == c)
            res[name]["by_component_uncoverable_m2"][str(c)] = round(float(area[m & ~tot].sum()), 1)
        np.save(os.path.join(PV.CACHE, f"probe_{name}_seen.npy"), np.flatnonzero(tot))
        print(name, json.dumps(res[name]), f"{time.time() - t:.0f} s", flush=True)
    json.dump(res, open(os.path.join(PV.CACHE, "ceiling_probe.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
