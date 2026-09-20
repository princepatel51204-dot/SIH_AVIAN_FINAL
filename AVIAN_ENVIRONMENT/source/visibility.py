"""Per-defect sensor visibility ground truth, measured against real geometry.

THE POINT OF THIS MODULE
------------------------
A defect existing in a scene is not the same as a defect being observable. The
REV-A ground truth carried an `occlusion` figure that was an *authored
estimate* -- a number typed into the site catalogue because a diaphragm bay is
obviously harder to see than a parapet. That is a reasonable prior and a bad
measurement, and any detection score computed against it would inherit the
author's guess rather than the world's geometry.

This module replaces the guess with a measurement. For every defect it fires a
hemisphere of rays from the surface outward and records which directions
actually escape the structure, how far they get, and at what angle they leave.
From that it derives, per sensor modality, whether the defect could be
observed at all and from where.

WHAT IS MEASURED, PER DEFECT
----------------------------
  visible_fraction        of the outward hemisphere that is unobstructed
  occlusion_measured      1 - visible_fraction
  best_view_direction     the unobstructed direction closest to the surface
                          normal, weighted by how far it runs clear
  best_view_clear_m       how far that direction runs before hitting anything
  min_inspection_range_m  from the sensor's own minimum range
  max_useful_range_m      the smaller of the sensor's useful range and the
                          distance at which the defect drops below the
                          detectability threshold for its size
  rgb / depth / lidar     expected visibility per modality
  detection_difficulty    LEVEL_1..LEVEL_4

HONEST LIMITS
-------------
The hemisphere is sampled, not integrated: 61 directions on a Fibonacci
lattice. That resolves an occlusion fraction to roughly +/-2 %, which is far
finer than the difficulty banding needs and far coarser than a real viewpoint
planner would want. Lighting is NOT ray-traced here -- illumination enters
through a per-surface prior, not a light transport calculation, and that is
stated in the record rather than hidden. `expected_*_visibility` is a
capability statement about the sensor and the geometry, not a promise that a
detector will succeed.
"""
from __future__ import annotations
import json
import math

import bpy
from mathutils import Vector

import params as P


# Sensor envelopes, kept consistent with sensors.SENSOR_SPECS.
RGB_MIN_M, RGB_MAX_M = 0.35, 12.0
RGB_GSD_MM_AT_1M = 0.62
DEPTH_MIN_M, DEPTH_MAX_M = 0.30, 8.0
DEPTH_ACCURACY_PCT = 2.0
LIDAR_MIN_M, LIDAR_MAX_M = 0.10, 70.0
LIDAR_RANGE_ACCURACY_M = 0.03

# Detection and measurement are different thresholds and conflating them is
# a real error. A dark line one pixel wide against light concrete is
# DETECTABLE -- that is how crack detection works in practice, because a line
# has extent along its length even when it has none across it. MEASURING that
# crack's width to engineering tolerance needs several pixels across it.
# Using the measurement threshold for both reported 92 of 192 defects as
# invisible, which is wrong in a way that would quietly discard half the
# dataset.
PIXELS_TO_DETECT = 1.0
PIXELS_TO_MEASURE = 3.0

# Room needed in a direction for it to count as a usable viewpoint. Set by
# the largest sensor minimum range plus a small standoff margin, because that
# is the physical thing being asked: can an instrument be placed here.
OPEN_THRESHOLD_M = 1.0

# Illumination prior by host surface. This is a PRIOR, not a measurement:
# the daylight rig is fixed and these values encode which faces it reaches.
# Written into the record as `illumination_prior` so nobody mistakes it for
# a rendered result.
ILLUMINATION_PRIOR = {
    "PARAPET": 0.95,
    "PIER_COLUMN": 0.70,
    "PIER_CAP": 0.45,
    "GIRDER_WEB": 0.22,
    "GIRDER_BOTTOM_FLANGE": 0.18,
    "DECK_UNDERSIDE": 0.12,
    "DIAPHRAGM": 0.08,
    "BEARING_SEAT": 0.06,
}


def _hemisphere(normal, n=61):
    """Fibonacci-lattice directions on the hemisphere about `normal`."""
    n_v = Vector(normal).normalized()
    # build an orthonormal basis around the normal
    a = Vector((0.0, 0.0, 1.0))
    if abs(n_v.dot(a)) > 0.95:
        a = Vector((1.0, 0.0, 0.0))
    u = n_v.cross(a).normalized()
    v = n_v.cross(u).normalized()
    ga = math.pi * (3.0 - math.sqrt(5.0))
    out = []
    for i in range(n):
        # z from 1 (along the normal) down to ~0 (grazing)
        z = 1.0 - (i + 0.5) / n
        r = math.sqrt(max(0.0, 1.0 - z * z))
        th = ga * i
        d = (n_v * z + u * (r * math.cos(th)) + v * (r * math.sin(th)))
        out.append(d.normalized())
    return out


def _ray_clear(dg, scene, origin, direction, max_dist):
    """Distance the ray runs before hitting renderable geometry.

    Defect objects and non-rendering volumes are stepped past: a decal is
    6 mm off its own host and would otherwise block every ray leaving it.
    """
    org = Vector(origin)
    rem = max_dist
    for _ in range(10):
        hit, loc, nrm, idx, ob, mw = scene.ray_cast(dg, org, direction,
                                                    distance=rem)
        if not hit or ob is None:
            return max_dist, None
        if not ob.hide_render and not ob.name.startswith("DEFECT_"):
            return (Vector(loc) - Vector(origin)).length, ob
        step = (Vector(loc) - org).length + 1e-3
        rem -= step
        if rem <= 0.0:
            return max_dist, None
        org = Vector(loc) + direction * 1e-3
    return max_dist, None


def _feature_size_mm(rec):
    """The smallest dimension a sensor has to resolve for this defect."""
    if rec.get("width_mm", 0.0) and rec["width_mm"] > 0.0:
        return float(rec["width_mm"])          # crack: width is the limit
    r = rec.get("radius_m")
    if r:
        return float(r) * 2000.0               # spall: the opening
    a = rec.get("area_m2", 0.0)
    if a:
        return math.sqrt(float(a)) * 1000.0
    return 10.0


def compute(records, probe_dist=25.0, n_dirs=61, log=print):
    """Measure visibility for every defect. Mutates `records` in place."""
    scene = bpy.context.scene
    dg = bpy.context.evaluated_depsgraph_get()

    diff_counts = {}
    n_blind = 0
    for rec in records:
        p = Vector(rec["position_m"])
        n = Vector(rec["surface_normal"]).normalized()
        origin = p + n * 0.02                  # lift clear of the host face

        dirs = _hemisphere(n, n_dirs)
        clear = []
        for d in dirs:
            dist, _ = _ray_clear(dg, scene, origin, d, probe_dist)
            clear.append(dist)

        # A direction counts as OPEN if it leaves room to put a camera at
        # its minimum working distance -- not if it runs unobstructed to the
        # horizon. Requiring the full probe distance made almost every
        # under-deck defect "observable by no modality", which is nonsense:
        # a defect on a girder web is perfectly observable from 1.5 m away,
        # and the fact that the next girder is 4 m beyond that is irrelevant.
        open_dirs = [(d, c) for d, c in zip(dirs, clear)
                     if c >= OPEN_THRESHOLD_M]
        vis_frac = len(open_dirs) / float(n_dirs)

        # Best view: the open direction most nearly along the normal. A view
        # at 70 degrees off-normal foreshortens a crack by a factor of three
        # and is worth much less than the raw "it is visible" would suggest.
        if open_dirs:
            best_d, best_c = max(open_dirs, key=lambda t: t[0].dot(n))
        else:
            # nothing fully open; take the direction that runs furthest
            j = max(range(n_dirs), key=lambda k: clear[k])
            best_d, best_c = dirs[j], clear[j]
        cos_best = max(0.0, min(1.0, best_d.dot(n)))
        obliquity_deg = math.degrees(math.acos(cos_best))

        # ---- per-modality range limits -----------------------------------
        feat_mm = _feature_size_mm(rec)
        # RGB: range at which the feature spans PIXELS_TO_DETECT pixels,
        # scaled by the foreshortening of an oblique view.
        rgb_range = (feat_mm / (RGB_GSD_MM_AT_1M * PIXELS_TO_DETECT)) \
            * max(0.25, cos_best)
        rgb_meas_range = (feat_mm / (RGB_GSD_MM_AT_1M * PIXELS_TO_MEASURE)) \
            * max(0.25, cos_best)
        rgb_max = min(RGB_MAX_M, best_c, rgb_range)
        rgb_ok = bool(rgb_max >= RGB_MIN_M and best_c >= RGB_MIN_M)
        rgb_measurable = bool(min(RGB_MAX_M, best_c, rgb_meas_range)
                              >= RGB_MIN_M and best_c >= RGB_MIN_M)
        # The GSD this defect actually demands, which is the number a sensor
        # selection argument needs.
        required_gsd_mm = feat_mm / PIXELS_TO_DETECT

        # Depth: only geometry-backed defects have relief to measure, and the
        # relief has to exceed the sensor's own accuracy at that range.
        depth_mm = float(rec.get("depth_mm") or 0.0)
        geo = rec.get("representation") == "GEOMETRY"
        if geo and depth_mm > 0.0:
            # accuracy is a percentage of range -> range where relief == error
            depth_limit = (depth_mm / 1000.0) / (DEPTH_ACCURACY_PCT / 100.0)
            depth_max = min(DEPTH_MAX_M, best_c, depth_limit)
            depth_ok = bool(depth_max >= DEPTH_MIN_M
                            and best_c >= DEPTH_MIN_M)
        else:
            depth_max = 0.0
            depth_ok = False

        # LiDAR: relief must exceed the range accuracy outright.
        if geo and depth_mm / 1000.0 > LIDAR_RANGE_ACCURACY_M:
            lidar_max = min(LIDAR_MAX_M, best_c)
            lidar_ok = bool(lidar_max >= LIDAR_MIN_M
                            and best_c >= LIDAR_MIN_M)
        else:
            lidar_max = 0.0
            lidar_ok = False

        illum = ILLUMINATION_PRIOR.get(rec["host_surface"], 0.3)

        # ---- detection difficulty ---------------------------------------
        # Four drivers, each genuinely independent: how much of the feature
        # there is, how much light reaches it, how much of the hemisphere is
        # open, and how obliquely it must be viewed.
        size_score = min(1.0, feat_mm / 60.0)
        score = (0.34 * (1.0 - size_score)
                 + 0.26 * (1.0 - illum)
                 + 0.25 * (1.0 - vis_frac)
                 + 0.15 * min(1.0, obliquity_deg / 75.0))
        if not rgb_ok:
            score = max(score, 0.88)
        sev = rec.get("severity", 1)
        score = max(0.0, min(1.0, score - 0.05 * (sev - 1)))
        if score < 0.30:
            level = "LEVEL_1"
        elif score < 0.52:
            level = "LEVEL_2"
        elif score < 0.72:
            level = "LEVEL_3"
        else:
            level = "LEVEL_4"
        diff_counts[level] = diff_counts.get(level, 0) + 1
        if not (rgb_ok or depth_ok or lidar_ok):
            n_blind += 1

        rec.update({
            "visible_fraction": round(vis_frac, 4),
            "occlusion_measured": round(1.0 - vis_frac, 4),
            "recommended_view_direction": [round(v, 4) for v in best_d],
            "recommended_view_obliquity_deg": round(obliquity_deg, 1),
            "best_view_clear_m": round(min(best_c, probe_dist), 2),
            "feature_size_mm": round(feat_mm, 3),
            "min_inspection_range_m": round(
                max(RGB_MIN_M, DEPTH_MIN_M if depth_ok else 0.0), 2),
            "max_useful_range_m": round(max(rgb_max, depth_max, lidar_max), 2),
            "expected_rgb_visibility": rgb_ok,
            "expected_rgb_max_range_m": round(rgb_max, 2),
            "rgb_width_measurable": rgb_measurable,
            "required_gsd_mm": round(required_gsd_mm, 3),
            "rgb_limited_by": (
                "sensor minimum focus range" if rgb_range < RGB_MIN_M
                else "feature size vs ground sample distance"
                if rgb_range < min(RGB_MAX_M, best_c)
                else "available clear standoff" if best_c < RGB_MAX_M
                else "sensor maximum useful range"),
            "expected_depth_visibility": depth_ok,
            "expected_depth_max_range_m": round(depth_max, 2),
            "expected_lidar_visibility": lidar_ok,
            "expected_lidar_max_range_m": round(lidar_max, 2),
            "illumination_prior": round(illum, 2),
            "detection_difficulty": level,
            "detection_difficulty_score": round(score, 3),
            "recommended_sensor": ("DEPTH+RGB" if depth_ok
                                   else "RGB" if rgb_ok else "NONE"),
            "visibility_method": (
                f"{n_dirs}-direction hemisphere ray-cast to "
                f"{probe_dist:.0f} m; a direction counts as open at "
                f"{OPEN_THRESHOLD_M:.1f} m clear. Illumination is a "
                f"per-surface prior, NOT ray-traced."),
        })

        ob = bpy.data.objects.get(rec["defect_id"])
        if ob is not None:
            ob["avi_occlusion_measured"] = rec["occlusion_measured"]
            ob["avi_visible_fraction"] = rec["visible_fraction"]
            ob["avi_detection_difficulty"] = level
            ob["avi_recommended_sensor"] = rec["recommended_sensor"]
            ob["avi_max_useful_range_m"] = rec["max_useful_range_m"]
            ob["avi_min_inspection_range_m"] = rec["min_inspection_range_m"]
            ob["avi_recommended_view_direction"] = \
                rec["recommended_view_direction"]

    order = ["LEVEL_1", "LEVEL_2", "LEVEL_3", "LEVEL_4"]
    log("  visible : "
        + ", ".join(f"{k}={diff_counts.get(k, 0)}" for k in order)
        + f"; {n_blind} observable by no modality")
    return {"difficulty": {k: diff_counts.get(k, 0) for k in order},
            "unobservable": n_blind}


def correlate_with_prior(records):
    """How far the authored occlusion estimate was from the measurement.

    Reported rather than quietly dropped: the authored value is kept in the
    ground truth as `occlusion`, and this says how much it should be trusted.
    """
    pairs = [(r.get("occlusion", 0.0), r["occlusion_measured"])
             for r in records if "occlusion_measured" in r]
    if len(pairs) < 2:
        return None
    n = len(pairs)
    ma = sum(a for a, _ in pairs) / n
    mb = sum(b for _, b in pairs) / n
    va = sum((a - ma) ** 2 for a, _ in pairs)
    vb = sum((b - mb) ** 2 for _, b in pairs)
    cov = sum((a - ma) * (b - mb) for a, b in pairs)
    r = cov / math.sqrt(va * vb) if va > 0 and vb > 0 else 0.0
    mae = sum(abs(a - b) for a, b in pairs) / n
    return {"n": n, "authored_mean": round(ma, 3),
            "measured_mean": round(mb, 3),
            "pearson_r": round(r, 3), "mean_abs_error": round(mae, 3)}
