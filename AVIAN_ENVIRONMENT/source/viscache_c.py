"""Cache for visibility.compute().

visibility.compute() casts a 61-direction hemisphere per defect and was
233.4 s of a 384.8 s build -- the single largest cost in the loop, and it
recomputes an identical answer every time nothing relevant has changed.

WHAT THE KEY COVERS, AND WHY IT COVERS MORE THAN THE GROUND TRUTH
-----------------------------------------------------------------
Keying on the defect ground truth alone would be wrong. Visibility is a
measurement of OCCLUSION, so it depends on the geometry doing the occluding
as much as on the defects themselves. Stage 2 adds a metro viaduct beside
the bridge; that changes what a defect can see without changing one byte of
the defect ground truth. A ground-truth-only key would serve a stale answer
into exactly the stage where it is most wrong.

So the key is (defect identity + probe parameters + a scene geometry
fingerprint). The fingerprint is object count, rendered triangle count and a
digest of the structural object names -- cheap to compute and sensitive to
anything that could cast a new shadow.

GATE RUNS NEVER USE THE CACHE
-----------------------------
compute_cached(gate=True) always recomputes. A validation report built on a
cached measurement is a validation report that has not measured anything,
and working rule 3 does not have an exception for slow checks.
"""
from __future__ import annotations

import hashlib
import json
import os
import time

import bpy

CACHE_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scene", "vis_cache")

# The record fields visibility.compute() writes. Cached and restored as a
# unit; if compute() grows a field this list must grow with it, which is
# checked at restore time rather than assumed.
_VIS_FIELDS = (
    "visible_fraction", "occlusion_measured", "recommended_view_direction",
    "recommended_view_obliquity_deg", "best_view_clear_m",
    "feature_size_mm", "min_inspection_range_m", "max_useful_range_m",
    "expected_rgb_visibility", "expected_rgb_max_range_m",
    "rgb_width_measurable", "required_gsd_mm", "rgb_limited_by",
    "expected_depth_visibility", "expected_depth_max_range_m",
    "expected_lidar_visibility", "expected_lidar_max_range_m",
    "illumination_prior", "detection_difficulty",
    "detection_difficulty_score", "recommended_sensor", "visibility_method",
)


def _scene_fingerprint():
    """Cheap digest of the geometry that could occlude a defect."""
    n_obj = len(bpy.data.objects)
    tris = 0
    names = []
    for o in bpy.data.objects:
        if o.type != "MESH":
            continue
        if not o.hide_render:
            tris += sum(max(0, len(p.vertices) - 2) for p in o.data.polygons)
        if o.name.startswith(("BR_", "MB_", "AVI_DET_")):
            names.append(o.name)
    h = hashlib.sha256("\n".join(sorted(names)).encode()).hexdigest()[:16]
    return f"{n_obj}-{tris}-{h}"


def _defect_fingerprint(records):
    """Digest of everything about the defects that visibility depends on."""
    parts = []
    for r in records:
        parts.append("|".join(str(r.get(k)) for k in (
            "defect_id", "type", "severity", "position_m", "surface_normal",
            "host_surface", "host_object", "representation", "depth_mm",
            "width_mm", "area_m2", "length_m", "surface_offset_mm")))
    return hashlib.sha256("\n".join(parts).encode()).hexdigest()[:24]


def cache_key(records, probe_dist, n_dirs):
    return (f"vis-{_defect_fingerprint(records)}-{_scene_fingerprint()}"
            f"-p{probe_dist:g}-d{n_dirs}")


def _path(key):
    return os.path.join(CACHE_DIR, key + ".json")


def compute_cached(compute_fn, records, probe_dist=25.0, n_dirs=61,
                   force=False, gate=False, log=print):
    """visibility.compute() with a cache in front of it.

    gate=True recomputes unconditionally and still refreshes the cache.
    force=True recomputes because the caller asked (--no-vis-cache).
    """
    key = cache_key(records, probe_dist, n_dirs)
    path = _path(key)

    if not (gate or force) and os.path.exists(path):
        try:
            with open(path) as f:
                blob = json.load(f)
            by_id = blob["records"]
            missing = [r["defect_id"] for r in records
                       if r["defect_id"] not in by_id]
            if missing:
                raise KeyError(f"{len(missing)} defects absent from cache")
            for r in records:
                cached = by_id[r["defect_id"]]
                absent = [k for k in _VIS_FIELDS if k not in cached]
                if absent:
                    raise KeyError(
                        f"cache predates fields {absent[:3]}")
                r.update(cached)
            log(f"  visible : restored from cache ({key[:28]}..., "
                f"{len(records)} defects, 0.0 s)")
            return blob["summary"]
        except (KeyError, ValueError, OSError) as e:
            log(f"  visible : cache unusable ({e}); recomputing")

    t0 = time.time()
    summary = compute_fn(records, probe_dist=probe_dist, n_dirs=n_dirs,
                         log=log)
    dt = time.time() - t0

    try:
        os.makedirs(CACHE_DIR, exist_ok=True)
        blob = {
            "key": key,
            "written_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "probe_dist": probe_dist, "n_dirs": n_dirs,
            "compute_wall_s": round(dt, 1),
            "summary": summary,
            "records": {r["defect_id"]: {k: r[k] for k in _VIS_FIELDS
                                         if k in r}
                        for r in records},
        }
        with open(_path(key), "w") as f:
            json.dump(blob, f)
        log(f"  visible : computed in {dt:.1f} s and cached "
            f"({'gate run' if gate else 'key ' + key[:28] + '...'})")
    except OSError as e:
        log(f"  visible : computed in {dt:.1f} s, cache write failed ({e})")
    return summary


def wrap(compute_fn, force=False, gate=False, log=print):
    """Return a drop-in replacement for visibility.compute()."""
    def _compute(records, probe_dist=25.0, n_dirs=61, log=log):
        return compute_cached(compute_fn, records, probe_dist=probe_dist,
                              n_dirs=n_dirs, force=force, gate=gate, log=log)
    return _compute
