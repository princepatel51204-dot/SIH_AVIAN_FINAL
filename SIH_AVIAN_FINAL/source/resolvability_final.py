"""SIH_AVIAN_FINAL -- detection pass: resolvability and escalation (SPEC S5).

This is the deliverable, not the geometry. Two things are computed, on
EVERY defect record -- concrete, metro and steel alike:

  min_detect_range_m   the range at which `feature_size_mm` (already
                        measured by visibility.py's own `_feature_size_mm`,
                        a geometric quantity independent of any sensor)
                        drops to exactly PX_TO_IDENTIFY pixels across, using
                        THIS project's own stated sensor figure
                        (2.1478 mm/px @ 1 m) -- not REV-C's `visibility.py`
                        RGB_GSD_MM_AT_1M (0.62 mm/px), which belongs to a
                        different, more capable sensor and is used there
                        only for that module's OWN internal rgb_range calc.
                        This is the exact "unlinked constant" trap flagged
                        earlier in this project (Phase 3's Gazebo materials
                        fix) -- reusing visibility.py's ALREADY-measured
                        feature_size_mm field sidesteps it entirely, since
                        that field is sensor-independent.

  escalation_reason     one of none / below_contrast / below_resolution /
                        occluded / unreachable_angle -- DERIVED from four
                        already-measured signals (visibility.py's
                        best_view_clear_m / recommended_view_obliquity_deg
                        / visible_fraction, and contrast_c.py's
                        contrast_limited), never authored. Must run AFTER
                        both visibility.compute() and contrast_c.measure(),
                        i.e. in measure_final.py (phase 2), not build_final.py
                        (phase 1) -- contrast is a phase-2-only measurement.

`min_detect_range_m` itself has no contrast dependency and IS added in
phase 1, right after visibility.compute() -- see add_min_detect_range().
"""
from __future__ import annotations

RGB_MIN_M = 0.35   # visibility.py's own RGB_MIN_M -- the sensor's closest
                   # focus distance, imported as a literal (not a live
                   # import of visibility.py) to keep this module usable in
                   # both build_final.py's and measure_final.py's processes
                   # without caring which one has visibility.py loaded.

OBLIQUITY_ESCALATE_DEG = 70.0
VIS_FRAC_ESCALATE = 0.15


def add_min_detect_range(records, params, log=print):
    """Phase 1: feature_size_mm -> min_detect_range_m. Requires
    visibility.compute() to already have run (it writes feature_size_mm)."""
    n = 0
    for r in records:
        feat = r.get("feature_size_mm")
        if feat is None:
            continue
        r["min_detect_range_m"] = round(params.min_detect_range_m(feat), 4)
        n += 1
    if log:
        log(f"  resolve : min_detect_range_m added to {n}/{len(records)} "
            f"records (GSD {params.GSD_MM_PER_PX_AT_1M} mm/px @ 1 m, "
            f"{params.PX_TO_IDENTIFY} px to identify)")
    return n


def resolvability_breakdown(records, params):
    """The three-bucket table SPEC S5 asks for, by defect type and overall."""
    overall = {"identifiable_at_standoff": 0, "requires_closer": 0,
              "unidentifiable": 0}
    by_type = {}
    for r in records:
        m = r.get("min_detect_range_m")
        if m is None:
            continue
        if m >= params.UNDERSIDE_INSPECTION_STANDOFF_M:
            bucket = "identifiable_at_standoff"
        elif m >= params.MIN_FLYABLE_RANGE_M:
            bucket = "requires_closer"
        else:
            bucket = "unidentifiable"
        overall[bucket] += 1
        t = r.get("type", "UNKNOWN")
        by_type.setdefault(t, {"identifiable_at_standoff": 0,
                               "requires_closer": 0, "unidentifiable": 0})
        by_type[t][bucket] += 1
    return {"overall": overall, "by_type": by_type}


def _reason(r, params):
    """Precedence: occluded > below_resolution > unreachable_angle >
    below_contrast > none. Each test consumes a measurement already on the
    record; nothing here is authored."""
    best_c = r.get("best_view_clear_m")
    obliquity = r.get("recommended_view_obliquity_deg")
    vis_frac = r.get("visible_fraction")
    mdr = r.get("min_detect_range_m")
    contrast_limited = r.get("contrast_limited")

    if best_c is None or mdr is None:
        return "none"   # not measured (no visibility pass) -- nothing to escalate on

    reachable_min = max(params.MIN_FLYABLE_RANGE_M, RGB_MIN_M)
    reachable_max = best_c
    required_max = mdr

    valid_window = reachable_min <= min(reachable_max, required_max)
    if not valid_window:
        if reachable_max < reachable_min:
            return "occluded"
        return "below_resolution"

    if (obliquity is not None and obliquity > OBLIQUITY_ESCALATE_DEG
            and vis_frac is not None and vis_frac < VIS_FRAC_ESCALATE):
        return "unreachable_angle"

    if contrast_limited:
        return "below_contrast"

    return "none"


def add_escalation(records, params, log=print, label=""):
    """Phase 2, AFTER contrast_c.measure(). Adds `escalation_reason` and
    returns the count breakdown."""
    counts = {"none": 0, "below_contrast": 0, "below_resolution": 0,
             "occluded": 0, "unreachable_angle": 0}
    for r in records:
        reason = _reason(r, params)
        r["escalation_reason"] = reason
        counts[reason] = counts.get(reason, 0) + 1
    escalated = len(records) - counts["none"]
    if log:
        tag = f" ({label})" if label else ""
        log(f"  escalate{tag}: {escalated}/{len(records)} escalated -> "
            f"{counts}")
    return counts
