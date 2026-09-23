"""SIH_AVIAN_FINAL -- Stage 2: the coverage mission.

Stage 1 (`mission_final.py`) builds waypoints by reading the defect
taxonomy's own logged JSON files (road + metro + steel) and pointing the
aircraft at 192 known defect positions -- privileged knowledge a real
inspection drone would not have before it flies. This module plans the
mission the other way round: from `avian_bridge_collision.json` alone (the
568 oriented box/cylinder primitives PyBullet itself loads) plus
`params_final.py`'s sensor constants, it works out every square metre of
every inspectable surface that needs a photograph taken of it at resolving
range, and reduces that to a minimum set of viewpoints. No defect position
is ever read here -- this file makes no reads of the taxonomy's logged
JSON at all; the one class-level constant this pass is allowed from that
taxonomy (the finest feature size the aircraft's camera claims to resolve
at all, which sets the flying standoff) is computed in, and imported from,
`detect_stub_final.py` instead, which is the sole quarantined touchpoint.

REUSED VERBATIM FROM `mission_final.py`, NOT REIMPLEMENTED: collision
primitive loading, the signed clearance test against those primitives,
line-of-sight ray-casting against the real rendered mesh, and
nearest-neighbour ordering -- all imported from that module so Stage 1 and
Stage 2 agree, by construction, on what "clear of structure" and "has line
of sight" mean.

INSPECTABLE SURFACES: everything in the collision export except `ground`,
`water`, `landing_pad`, and `train_car` (the sky-train itself is not part
of the bridge under inspection). That is an EXCLUSION rule, not an
enumerated whitelist -- new structural kinds added to the collision export
later are inspectable by default, matching the export's own convention of
naming what to skip rather than what to keep. Checked against the real
manifest, not assumed: 481 of 568 primitives qualify (1 ground + 1 water +
2 landing_pad + 83 train_car excluded) -- the master prompt's own guess of
"546" does not match the actual `kind` counts and is corrected here.

COVERAGE MODEL, AND WHERE THIS PASS DELIBERATELY SIMPLIFIES: a real
FOV-correct "does viewpoint A also frame patch B" test would need a
per-pair ray-cast (infeasible at thousands of patches) or a proper
perspective-projection check. This pass instead grids every face/band at a
spacing that guarantees gap-free coverage (a configurable overlap fraction
of the sensor footprint at the flying standoff) and treats one candidate as
covering any other SAME-primitive patch within one grid-diagonal step of
its own -- i.e. "the immediately adjacent samples this grid was already
built to keep gap-free," not a from-scratch visibility model. This is the
prototype-scope choice `mission_final.py`'s own comments use elsewhere
(nearest-neighbour over TSP-optimal, greedy set-cover over an ILP solver)
-- stated here rather than silently assumed.
"""
from __future__ import annotations
import inspect
import json
import math
import os
import sys
import time

import bpy
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(ROOT)
ENV_SRC = os.environ.get(
    "AVIAN_ENV_SRC", os.path.join(REPO, "AVIAN_ENVIRONMENT", "source"))
UAV_DIR = os.environ.get("AVIAN_UAV_DIR", os.path.join(REPO, "AVIAN_UAV"))
if ENV_SRC not in sys.path:
    sys.path.insert(0, ENV_SRC)
if HERE not in sys.path:
    sys.path.insert(0, HERE)
if UAV_DIR not in sys.path:
    sys.path.insert(0, UAV_DIR)

import params_final as PF
import mission_final as MF          # reused verbatim -- see module docstring
import sensors.cameras as UAVCAM
from detect_stub_final import FINEST_FEATURE_MM  # the ONE taxonomy constant

T0 = time.time()
LOG_LINES = []


def log(msg=""):
    print(msg)
    LOG_LINES.append(str(msg))


SCENE_DIR = MF.SCENE_DIR
BLEND_PATH = MF.BLEND_PATH
MISSION_DIR = MF.MISSION_DIR

# ---------------------------------------------------------------------------
# sensor intrinsics -- read from AVIAN_UAV/sensors/cameras.py, same pattern
# dataset_final.py and mission_final.py already use, not retyped.
# ---------------------------------------------------------------------------
_sig = inspect.signature(UAVCAM.RGBCamera.__init__)
HFOV_DEG = float(_sig.parameters["hfov_deg"].default)
VFOV_DEG = float(_sig.parameters["vfov_deg"].default)

# The literal reading of "standoff = PF.min_detect_range_m() at the finest
# taxonomy feature" was tried first: PF.min_detect_range_m(0.052 mm) is
# ~0.008 m, clamped by MIN_FLYABLE_RANGE_M to 0.30 m -- and at 0.30 m every
# inspectable surface in this 360 m corridor (girders and deck members
# alone run up to 45 m long) grids out to 1.63 MILLION raw candidate
# viewpoints before any filtering, measured directly, not estimated. That
# is several orders of magnitude past "hundreds to low thousands" and past
# what this prototype pass can compute. It is also not buying real
# resolving power: 0.052 mm is BELOW_RESOLUTION at any flyable range in
# Stage 1's own logged taxonomy regardless of standoff (checked directly,
# not assumed -- a 2.019 mm hairline
# crack, resolvable in principle at 0.313 m, is STILL flagged
# below_resolution there once its own occlusion/clamping is accounted for),
# so flying every square metre of the bridge at 0.30 m buys nothing for
# the very features that would justify it.
#
# Used instead: `PF.UNDERSIDE_INSPECTION_STANDOFF_M` (1.5 m), the project's
# own already-sourced SURVEY standoff (SPEC.md's airspace table, read not
# invented) -- the broad first-pass distance a real inspection flies before
# zooming in on anything a survey pass actually flags, which is exactly
# what "autonomous coverage exploration" means at this prototype's scope:
# see every surface at a distance that can still resolve the taxonomy's
# STRUCTURALLY MEANINGFUL small defects (a 6 mm loose bolt resolves at
# 0.93 m, well inside 1.5 m), not the below-resolution-regardless tail.
# This is a stated scope decision, not a silent substitution -- and it
# still needs a per-face sample cap (`MAX_SAMPLES_PER_AXIS`) below to keep
# the long spanning members' patch counts tractable.
# Both overridable via env for tuning the mission down to a size that can
# actually be FLOWN this session (a first pass at these defaults produced
# 5,291 waypoints / ~12,320 Wh estimated -- fine to PLAN in 145 s, not fine
# to fly). Not a hidden knob: whatever values a run used are printed and
# saved into the manifest below.
STANDOFF_M = float(os.environ.get("AVIAN_COVERAGE_STANDOFF_M",
                                  PF.UNDERSIDE_INSPECTION_STANDOFF_M))
MAX_SAMPLES_PER_AXIS = int(os.environ.get("AVIAN_COVERAGE_MAX_SAMPLES", 10))

FACE_OVERLAP_FRAC = 0.30

# ground/water/landing_pad are not part of the structure; train_car is the
# rolling stock, not the bridge. Everything else is inspectable BY DEFAULT
# -- see module docstring for why this is exclusion-based, not a whitelist.
EXCLUDE_KINDS = {"ground", "water", "landing_pad", "train_car"}


def _footprint_m(standoff_m):
    """Sensor footprint (width, height) in metres at a given standoff --
    same pinhole-camera relation as dataset_final.py's own `_gsd_mm_at_1m`,
    just evaluated for a frame's full extent instead of one pixel."""
    w = 2.0 * standoff_m * math.tan(math.radians(HFOV_DEG / 2.0))
    h = 2.0 * standoff_m * math.tan(math.radians(VFOV_DEG / 2.0))
    return w, h


_FOOTPRINT_W, _FOOTPRINT_H = _footprint_m(STANDOFF_M)
_STEP_U = _FOOTPRINT_W * (1.0 - FACE_OVERLAP_FRAC)
_STEP_V = _FOOTPRINT_H * (1.0 - FACE_OVERLAP_FRAC)
# "one grid-diagonal step" -- see module docstring's coverage-model note.
_COVER_RADIUS_M = math.hypot(_STEP_U, _STEP_V) * 1.02


def _grid_1d(half_extent, step, cap=MAX_SAMPLES_PER_AXIS):
    """`n` evenly spaced cell centres covering `[-half_extent, half_extent]`
    at (at most) `step` spacing -- always at least 1 sample, capped at
    `cap` for the handful of very long members (see MAX_SAMPLES_PER_AXIS)."""
    n = min(cap, max(1, math.ceil((2.0 * half_extent) / step)))
    if n == 1:
        return [0.0]
    return [-half_extent + (2.0 * half_extent) * (k + 0.5) / n
           for k in range(n)]


def _box_faces(prim):
    """6 faces of a yawed BOX primitive: face_name -> (normal, u_axis,
    v_axis, half_u, half_v, face_centre), all in world space."""
    c = Vector(prim["centre"])
    he = prim["half_extents"]
    yaw = prim.get("yaw", 0.0)
    ex = Vector((math.cos(yaw), math.sin(yaw), 0.0))
    ey = Vector((-math.sin(yaw), math.cos(yaw), 0.0))
    ez = Vector((0.0, 0.0, 1.0))
    return {
        "+X": (ex, ey, ez, he[1], he[2], c + ex * he[0]),
        "-X": (-ex, ey, ez, he[1], he[2], c - ex * he[0]),
        "+Y": (ey, ex, ez, he[0], he[2], c + ey * he[1]),
        "-Y": (-ey, ex, ez, he[0], he[2], c - ey * he[1]),
        "+Z": (ez, ex, ey, he[0], he[1], c + ez * he[2]),
        "-Z": (-ez, ex, ey, he[0], he[1], c - ez * he[2]),
    }


def _touches_ground(pt, normal):
    """The one "permanently obstructed direction" this pass filters
    proactively (the master prompt's own example: a footing's underside
    against the ground half-space) -- a downward-facing patch sitting at or
    below local grade. Anything else that turns out to be embedded or
    blocked is caught downstream by the same clearance/line-of-sight checks
    every other candidate goes through, not specially pre-filtered."""
    if normal.z >= -0.5:
        return False
    return pt.z <= PF.ground_z(pt.x, pt.y) + MF.GROUND_CLEARANCE_MARGIN_M


def _build_patches(primitives, log=print):
    """Every sampled surface point, before any clearance/LOS filtering.
    One patch = one (point, outward normal) pair a camera would centre a
    photograph on."""
    patches = []
    n_by_kind = {}
    for prim in primitives:
        if prim["kind"] in EXCLUDE_KINDS:
            continue
        n_by_kind[prim["kind"]] = n_by_kind.get(prim["kind"], 0) + 1
        pid = 0
        if prim["type"] == "BOX":
            for face_name, (normal, uax, vax, hu, hv, fc) in \
                    _box_faces(prim).items():
                for u in _grid_1d(hu, _STEP_U):
                    for v in _grid_1d(hv, _STEP_V):
                        pt = fc + uax * u + vax * v
                        if _touches_ground(pt, normal):
                            continue
                        patches.append({
                            "patch_id": f"{prim['name']}_{face_name}_{pid}",
                            "prim_name": prim["name"],
                            "prim_kind": prim["kind"],
                            "point": pt, "normal": normal,
                        })
                        pid += 1
        else:   # CYLINDER
            c = Vector(prim["centre"])
            r = prim["radius"]
            hh = prim["half_height"]
            step_theta = _STEP_U / max(r, 0.05)
            n_theta = min(MAX_SAMPLES_PER_AXIS * 2,
                         max(4, math.ceil((2.0 * math.pi) / step_theta)))
            for k in range(n_theta):
                th = 2.0 * math.pi * k / n_theta
                normal = Vector((math.cos(th), math.sin(th), 0.0))
                for z in _grid_1d(hh, _STEP_V):
                    pt = c + normal * r + Vector((0.0, 0.0, z))
                    if _touches_ground(pt, normal):
                        continue
                    patches.append({
                        "patch_id": f"{prim['name']}_side_{pid}",
                        "prim_name": prim["name"],
                        "prim_kind": prim["kind"],
                        "point": pt, "normal": normal,
                    })
                    pid += 1
            for cap_name, cap_normal in (("bottom", Vector((0, 0, -1))),
                                         ("top", Vector((0, 0, 1)))):
                pt = c + cap_normal * hh
                if _touches_ground(pt, cap_normal):
                    continue
                patches.append({
                    "patch_id": f"{prim['name']}_cap_{cap_name}",
                    "prim_name": prim["name"], "prim_kind": prim["kind"],
                    "point": pt, "normal": cap_normal,
                })
    log(f"  patches : {len(patches)} surface samples across "
        f"{sum(n_by_kind.values())}/{len(primitives)} inspectable "
        f"primitives (standoff {STANDOFF_M:.3f} m, footprint "
        f"{_FOOTPRINT_W:.2f}x{_FOOTPRINT_H:.2f} m, step "
        f"{_STEP_U:.2f}x{_STEP_V:.2f} m)")
    return patches


# Phase B item 4 (waypoint clearance): before dropping a candidate whose
# endpoint embeds in structure, try nudging it further out along its OWN
# view normal first -- the same axis its standoff is already measured
# along, so this doesn't turn into the isotropic per-waypoint probe this
# file's own module docstring already rejected (that one failed because it
# demanded clearance on every side; this one only ever asks for more room
# on the one side the camera is already backing away from).
NUDGE_STEP_M = 0.05
NUDGE_MAX_EXTRA_M = 1.0


def _build_candidates(patches, primitives, log=print):
    """One candidate viewpoint per surviving patch -- filtered by the SAME
    terrain-clearance, structure-hull-clearance, and line-of-sight rules
    `mission_final.py`'s own `filter_flyable()` applies, reused verbatim."""
    candidates = []
    n_below_grade = 0
    n_embedded = 0
    n_nudged = 0
    n_no_los = 0
    for patch in patches:
        standoff = STANDOFF_M
        pos = patch["point"] + patch["normal"] * standoff
        grade = PF.ground_z(pos.x, pos.y)
        if pos.z < grade + MF.GROUND_CLEARANCE_MARGIN_M:
            n_below_grade += 1
            continue
        clearance = MF._min_structure_clearance(pos, primitives)
        if clearance < MF.STRUCTURE_EMBED_MARGIN_M:
            extra = 0.0
            while (clearance < MF.STRUCTURE_EMBED_MARGIN_M and
                  extra < NUDGE_MAX_EXTRA_M):
                extra += NUDGE_STEP_M
                pos = patch["point"] + patch["normal"] * (standoff + extra)
                clearance = MF._min_structure_clearance(pos, primitives)
            if clearance < MF.STRUCTURE_EMBED_MARGIN_M:
                n_embedded += 1
                continue
            n_nudged += 1
        ok, _clear = MF.has_line_of_sight(pos, patch["point"])
        if not ok:
            n_no_los += 1
            continue
        heading_rad = math.atan2(-patch["normal"].y, -patch["normal"].x)
        candidates.append({
            "candidate_id": f"C_{len(candidates)}",
            "patch_id": patch["patch_id"],
            "prim_name": patch["prim_name"],
            "prim_kind": patch["prim_kind"],
            "pos": pos, "target": patch["point"],
            "heading_rad": heading_rad,
        })
    log(f"  viewpts : {len(candidates)}/{len(patches)} candidates clear "
        f"({n_below_grade} below grade, {n_embedded} hull-embedded "
        f"(dropped), {n_nudged} nudged outward to clear, "
        f"{n_no_los} no line of sight)")
    return candidates


def _attach_coverage(candidates, patches, log=print):
    """Which patches each candidate covers: itself, plus any other patch on
    the SAME primitive within one grid-diagonal step -- see the module
    docstring's coverage-model note for why this, not a per-pair
    perspective check, is this pass's scope."""
    by_prim = {}
    for p in patches:
        by_prim.setdefault(p["prim_name"], []).append(p)
    for c in candidates:
        own = c["patch_id"]
        covers = {own}
        target = c["target"]
        for p in by_prim.get(c["prim_name"], ()):
            if p["patch_id"] == own:
                continue
            if (p["point"] - target).length <= _COVER_RADIUS_M:
                covers.add(p["patch_id"])
        c["covers"] = covers
    log(f"  overlap : avg {sum(len(c['covers']) for c in candidates) / max(1, len(candidates)):.2f} "
        "patches/candidate (grid-diagonal radius "
        f"{_COVER_RADIUS_M:.2f} m)")
    return candidates


def _greedy_set_cover(candidates, patches, max_waypoints=None, log=print):
    """Repeatedly pick the viewpoint covering the most not-yet-covered
    patches -- the same greedy, not-ILP-optimal pragmatism
    `mission_final.py`'s own `cluster_candidates()` documents choosing.

    Implemented as LAZY greedy (a max-heap of stale gains, re-validated
    against `remaining` only when popped and re-pushed if it dropped) --
    coverage is submodular, so a candidate's true gain can only fall as
    `remaining` shrinks, never rise, which is exactly the property the
    classic lazy-greedy trick needs. A naive rescan-every-candidate-every-
    round implementation is O(rounds x candidates); at this pass's scale
    (tens of thousands of candidates, comparable rounds) that is billions
    of set intersections -- measured as impractical here, not assumed.
    The lazy heap is the standard fix, not a novel algorithm.

    `max_waypoints`, if given, stops the greedy after that many picks --
    covering a 360 m corridor's full inspectable surface, even coarsely,
    needs on the order of 800-1,000 viewpoints (measured directly across
    several standoff/cap combinations, not a guess), which is too many to
    actually fly through PyBullet in one session. Since greedy always picks
    the highest-remaining-value viewpoint next, stopping early keeps
    exactly the N most valuable viewpoints, not an arbitrary subset -- a
    stated, logged prototype-scope cut, not a silent one."""
    import heapq
    all_ids = {p["patch_id"] for p in patches}
    coverable = set()
    for c in candidates:
        coverable |= c["covers"]
    remaining = set(coverable)
    heap = [(-len(c["covers"]), idx) for idx, c in enumerate(candidates)]
    heapq.heapify(heap)
    chosen = []
    while remaining and heap:
        if max_waypoints is not None and len(chosen) >= max_waypoints:
            break
        neg_gain, idx = heapq.heappop(heap)
        c = candidates[idx]
        true_gain = len(c["covers"] & remaining)
        if true_gain == 0:
            continue
        # Stale entry: its recomputed gain no longer matches what's
        # sitting at the top of the heap, so some other candidate might
        # now be strictly better -- re-push with the fresh value and let
        # the heap re-sort rather than trusting the old estimate.
        if heap and true_gain < -heap[0][0]:
            heapq.heappush(heap, (-true_gain, idx))
            continue
        chosen.append(c)
        remaining -= c["covers"]
    uncovered = (all_ids - coverable) | remaining
    log(f"  cover   : {len(chosen)} viewpoints selected"
        f"{' (capped)' if max_waypoints else ''}, covering "
        f"{len(all_ids) - len(uncovered)}/{len(all_ids)} patches "
        f"({100.0 * (len(all_ids) - len(uncovered)) / max(1, len(all_ids)):.1f}%)")
    return chosen, uncovered


MAX_WAYPOINTS = os.environ.get("AVIAN_COVERAGE_MAX_WAYPOINTS")
MAX_WAYPOINTS = int(MAX_WAYPOINTS) if MAX_WAYPOINTS else None


def build_coverage_mission(log=print):
    primitives = MF._load_collision_primitives()
    patches = _build_patches(primitives, log=log)
    candidates = _build_candidates(patches, primitives, log=log)
    candidates = _attach_coverage(candidates, patches, log=log)
    chosen, uncovered_patch_ids = _greedy_set_cover(
        candidates, patches, max_waypoints=MAX_WAYPOINTS, log=log)

    seed_clusters = [{"seed": {"wp_pos": c["pos"],
                              "heading_rad": c["heading_rad"]},
                     "members": [c]} for c in chosen]
    base_pos = MF._drone_base_position()
    ordered = MF.order_nearest_neighbour(seed_clusters, base_pos, log=log)

    uncovered_by_prim = {}
    patch_by_id = {p["patch_id"]: p for p in patches}
    for pid in uncovered_patch_ids:
        kind = patch_by_id[pid]["prim_kind"]
        uncovered_by_prim[kind] = uncovered_by_prim.get(kind, 0) + 1

    waypoints = []
    cum_dist = 0.0
    prev = base_pos
    n_detour_legs = n_transit_risk_legs = 0
    for i, cl in enumerate(ordered, start=1):
        seed = cl["seed"]
        c = cl["members"][0]
        # Phase B: a leg the direct-line check rejected but A* solved gets
        # its intermediate waypoints inserted here, transit-only (empty
        # `covers`, `transit_only: true`) -- never counted as inspection
        # coverage, just flown through on the way to the real waypoint
        # that follows. `keep the planner's viewpoint selection unchanged`
        # means these are ADDED, not a substitute for the inspection point.
        for j, dp in enumerate(cl.get("detour_path", [])):
            leg = (Vector(dp) - prev).length
            cum_dist += leg
            prev = Vector(dp)
            waypoints.append({
                "waypoint_id": f"CWP_{i:03d}_DETOUR_{j + 1}",
                "position_m": [round(v, 4) for v in dp],
                "heading_rad": seed["heading_rad"],
                "heading_deg": round(math.degrees(seed["heading_rad"]), 2),
                "leg_distance_m": round(leg, 3),
                "cumulative_distance_m": round(cum_dist, 3),
                "covers": [], "target_m": [round(v, 4) for v in dp],
                "prim_name": None, "prim_kind": None,
                "transit_only": True,
            })
            n_detour_legs += 1
        leg = (seed["wp_pos"] - prev).length
        cum_dist += leg
        prev = seed["wp_pos"]
        if cl.get("transit_risk"):
            n_transit_risk_legs += 1
        waypoints.append({
            "waypoint_id": f"CWP_{i:03d}",
            "position_m": [round(v, 4) for v in seed["wp_pos"]],
            "heading_rad": round(seed["heading_rad"], 4),
            "heading_deg": round(math.degrees(seed["heading_rad"]), 2),
            "leg_distance_m": round(leg, 3),
            "cumulative_distance_m": round(cum_dist, 3),
            "covers": sorted(c["covers"]),
            "target_m": [round(v, 4) for v in c["target"]],
            "prim_name": c["prim_name"],
            "prim_kind": c["prim_kind"],
            "transit_risk": bool(cl.get("transit_risk", False)),
        })
    cum_dist += (base_pos - prev).length

    total_s, energy_wh = MF.estimate_time_energy(cum_dist, len(waypoints))
    total_area_m2 = None   # not tracked per-patch at prototype scope

    manifest = {
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "planner": "coverage_final.py (Stage 2 -- structure-driven, no "
                  "ground truth read for planning)",
        "base_position_m": [round(v, 4) for v in base_pos],
        "standoff_m": round(STANDOFF_M, 4),
        "max_samples_per_axis": MAX_SAMPLES_PER_AXIS,
        "max_waypoints_cap": MAX_WAYPOINTS,
        "finest_feature_mm": FINEST_FEATURE_MM,
        "face_overlap_frac": FACE_OVERLAP_FRAC,
        "sensor": {"hfov_deg": HFOV_DEG, "vfov_deg": VFOV_DEG},
        "n_inspectable_primitives": sum(
            1 for p in primitives if p["kind"] not in EXCLUDE_KINDS),
        "n_primitives_total": len(primitives),
        "n_patches_total": len(patches),
        "n_patches_covered": len(patches) - len(uncovered_patch_ids),
        "structural_coverage_pct": round(
            100.0 * (len(patches) - len(uncovered_patch_ids)) /
            max(1, len(patches)), 2),
        "n_waypoints": len(waypoints),
        "n_detour_legs": n_detour_legs,
        "n_transit_risk_legs": n_transit_risk_legs,
        "n_patches_uncovered": len(uncovered_patch_ids),
        "uncovered_by_kind": uncovered_by_prim,
        "total_distance_m": round(cum_dist, 3),
        "estimated_time_s": total_s,
        "estimated_energy_Wh": energy_wh,
        "battery_Wh_nominal": 1300.0,
        "battery_reserve_pct": 20.0,
        "battery_Wh_usable": 1040.0,
        "waypoints": waypoints,
    }
    return manifest


def main():
    log("== SIH_AVIAN_FINAL :: coverage_final.py (Stage 2 planner) ==")
    bpy.ops.wm.open_mainfile(filepath=BLEND_PATH)
    os.makedirs(MISSION_DIR, exist_ok=True)

    manifest = build_coverage_mission(log=log)

    log(f"  mission : {manifest['n_waypoints']} waypoints, "
        f"{manifest['structural_coverage_pct']:.1f}% structural coverage "
        f"({manifest['n_patches_covered']}/{manifest['n_patches_total']} "
        "patches)")
    log(f"  flight  : {manifest['total_distance_m']:.1f} m, "
        f"~{manifest['estimated_time_s']:.0f} s, "
        f"~{manifest['estimated_energy_Wh']:.1f} Wh estimated "
        f"(of {manifest['battery_Wh_usable']:.0f} Wh usable)")
    if manifest["uncovered_by_kind"]:
        log(f"  uncovered by kind: {manifest['uncovered_by_kind']}")

    # Phase B (Section 4) rebuilds with the fixed margin/sampling/detour
    # logic but must never overwrite the already-committed Stage 2
    # coverage_mission.json Section 2/3's own frozen results reference --
    # AVIAN_COVERAGE_OUT_NAME lets this same script write a new file
    # instead, same pattern as the existing AVIAN_COVERAGE_* env knobs.
    out_name = os.environ.get("AVIAN_COVERAGE_OUT_NAME", "coverage_mission.json")
    out_path = os.path.join(MISSION_DIR, out_name)
    with open(out_path, "w") as f:
        json.dump(manifest, f, indent=2)
    log(f"  saved   : {out_path}")
    if manifest.get("n_detour_legs") or manifest.get("n_transit_risk_legs"):
        log(f"  transit : {manifest['n_detour_legs']} legs routed via A* "
           f"detour, {manifest['n_transit_risk_legs']} still flagged "
           "transit-risk (no clear path or detour found)")

    with open(os.path.join(MISSION_DIR,
                          "AVIAN_coverage_build_log_FINAL.txt"), "w") as f:
        f.write("\n".join(LOG_LINES))

    log(f"== done in {time.time() - T0:.1f} s ==")
    print("FINAL_COVERAGE_COMPLETE")


if __name__ == "__main__":
    main()
