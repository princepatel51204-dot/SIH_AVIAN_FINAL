"""SIH_AVIAN_FINAL -- two-panel inspection system, Stage A: the mission.

Derives an inspection flight plan straight from the ground truth -- no
hand-authored waypoint list. Every one of the 192 defects contributes a
candidate waypoint:

    waypoint_position = defect_position + recommended_view_direction * standoff
    waypoint_heading  = atan2 of -recommended_view_direction (facing the defect)

`standoff` is `max(min_detect_range_m, MIN_FLYABLE_RANGE_M)` -- the same
physical clamp `dataset_render_final.py` uses: fly as close as the defect's
own resolvability requires, never closer than the aircraft can actually hold
station.

Then: cluster (many bolts share one gusset -- one waypoint sees several),
filter to what is flyable (ray-cast line of sight against the REAL geometry;
"a waypoint that cannot see its own target" is explicitly named in the brief
as the bug class this project has hit six times), order by nearest-neighbour
from the drone base, and export `mission.json`.

NO AIRSPACE VOLUME CHECK. `zones_final.py` was never built for this scene
(documented in the detection pass's own README). "Outside a defined airspace
volume" is covered by the line-of-sight ray-cast instead: a waypoint whose
ray to its own defect is blocked at near-zero range is embedded in
structure, which is what an airspace check would also have caught here.
"""
from __future__ import annotations
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
if ENV_SRC not in sys.path:
    sys.path.insert(0, ENV_SRC)
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import damage as DMG
import dataset_final as DSF   # reuses _load_records()
import params_final as PF

T0 = time.time()
LOG_LINES = []


def log(msg=""):
    print(msg)
    LOG_LINES.append(str(msg))


SCENE_DIR = os.path.join(ROOT, "scene")
BLEND_PATH = os.path.join(SCENE_DIR, "SIH_AVIAN_FINAL.blend")
MISSION_DIR = os.path.join(ROOT, "mission")

CLUSTER_RADIUS_M = 0.5

# Same exclusion set as the dataset task's negative sampling: steel defect
# decals ("SDEFECT_...") don't start with "DEFECT_" (an "S" was chosen for
# the detection pass, not caught by damage.py's own _RAY_SKIP_PREFIXES) and
# would otherwise register as a false obstruction in a line-of-sight ray.
_LOS_SKIP_PREFIXES = ("DEFECT_", "AVI_", "_", "SDEFECT_", "FAST_")


def _los_clear_distance(origin, direction, max_dist):
    """Distance the ray runs before hitting REAL (non-hidden, non-decal,
    non-fastener) geometry -- same stepping pattern as visibility.py's own
    `_ray_clear`, reused conceptually rather than imported, since this one
    also has to step past this scene's own steel-defect/fastener decals."""
    scene = bpy.context.scene
    dg = bpy.context.evaluated_depsgraph_get()
    org = Vector(origin)
    rem = max_dist
    for _ in range(12):
        hit, loc, nrm, idx, ob, mw = scene.ray_cast(dg, org, direction,
                                                    distance=max(rem, 1e-4))
        if not hit or ob is None:
            return max_dist
        if not ob.hide_render and not ob.name.startswith(_LOS_SKIP_PREFIXES):
            return (Vector(loc) - Vector(origin)).length
        step = (Vector(loc) - org).length + 1e-3
        rem -= step
        if rem <= 0.0:
            return max_dist
        org = Vector(loc) + direction * 1e-3
    return max_dist


def has_line_of_sight(from_pos, to_pos, margin_m=0.05):
    """Fires one ray from the waypoint toward the defect. Clear if nothing
    REAL is hit before getting within `margin_m` of the target -- the
    defect's own tiny geometry sitting exactly at the target is not itself
    counted as the obstruction."""
    from_pos = Vector(from_pos)
    to_pos = Vector(to_pos)
    delta = to_pos - from_pos
    dist = delta.length
    if dist < 1e-6:
        return True, 0.0
    direction = delta.normalized()
    target_dist = max(0.01, dist - margin_m)
    clear = _los_clear_distance(from_pos, direction, target_dist)
    return clear >= target_dist - 1e-3, clear


def candidate_for(r):
    pos = Vector(r["position_m"])
    view_dir = Vector(r.get("recommended_view_direction")
                      or r["surface_normal"])
    if view_dir.length < 1e-6:
        view_dir = Vector((0.0, 1.0, 0.0))
    view_dir = view_dir.normalized()
    # Never push the waypoint past the ALREADY-MEASURED open space along
    # this direction (visibility.py's own hemisphere ray-cast, from the
    # detection pass): a confined site (a girder bay, a diaphragm gap) can
    # have a `best_view_clear_m` shorter than min_detect_range_m demands,
    # and placing the waypoint there anyway embeds it in the next member
    # over instead of respecting the room that's actually there.
    standoff = max(r["min_detect_range_m"], PF.MIN_FLYABLE_RANGE_M)
    best_clear = r.get("best_view_clear_m")
    if best_clear:
        standoff = min(standoff, max(best_clear - 0.05, PF.MIN_FLYABLE_RANGE_M))
    wp_pos = pos + view_dir * standoff
    # `best_clear` is measured against the RENDERED mesh (visibility.py's
    # hemisphere ray-cast); PyBullet's own collision hull is a coarser,
    # often LARGER box/cylinder wrap around that same geometry (found by
    # checking a wedged-in-flight waypoint against the collision JSON
    # directly: it cleared the mesh ray-cast by design but sat 0.0 m from
    # BR_GIRDER_003_G1's hull). Back the camera further out along its own
    # view axis -- never sideways, so it keeps looking at the same defect
    # -- until it clears the hull too, capped at whichever is tighter: the
    # resolvability budget (1.5x the required range, so the shot doesn't
    # go soft) or `best_clear` itself (the already-measured ceiling before
    # the NEXT piece of structure starts).
    hard_cap = r["min_detect_range_m"] * 1.5 + 0.3
    if best_clear:
        hard_cap = min(hard_cap, max(best_clear - 0.05, PF.MIN_FLYABLE_RANGE_M))
    primitives = _load_collision_primitives()
    while (_min_structure_clearance(wp_pos, primitives) < STRUCTURE_EMBED_MARGIN_M
          and standoff < hard_cap):
        standoff = min(standoff + 0.03, hard_cap)
        wp_pos = pos + view_dir * standoff
    # heading: yaw that points the aircraft's forward axis FROM the
    # waypoint BACK toward the defect, i.e. along -view_dir
    heading_rad = math.atan2(-view_dir.y, -view_dir.x)
    return {
        "defect_id": r["defect_id"], "type": r["type"],
        "wp_pos": wp_pos, "view_dir": view_dir, "standoff": standoff,
        "defect_pos": pos, "heading_rad": heading_rad,
        "escalation_reason": r.get("escalation_reason"),
        "feature_size_mm": r.get("feature_size_mm"),
        "min_detect_range_m": r.get("min_detect_range_m"),
        "defect_background_contrast": r.get("defect_background_contrast"),
        "contrast_limited": r.get("contrast_limited"),
        "avi_condition": r.get("avi_condition"),
        "host_structure": r.get("_host_structure"),
        "host_object": r.get("host_object"),
        "severity": r.get("severity"),
    }


def cluster_candidates(candidates, radius=CLUSTER_RADIUS_M, log=print):
    """Greedy clustering by DEFECT proximity (not waypoint proximity --
    "many bolts share one gusset" is about the defects being close
    together, which is what makes one camera position able to see several).
    A member only stays in a cluster if the CLUSTER'S OWN canonical
    waypoint pose actually has line of sight to that member's defect --
    verified, not assumed, since two bolts 0.3 m apart can still have a
    gusset stiffener between them."""
    pool = list(candidates)
    clusters = []
    while pool:
        seed = pool.pop(0)
        members = [seed]
        remaining = []
        for c in pool:
            if (c["defect_pos"] - seed["defect_pos"]).length <= radius:
                members.append(c)
            else:
                remaining.append(c)
        pool = remaining

        kept = [seed]
        rejected = []
        for c in members[1:]:
            ok, _clear = has_line_of_sight(seed["wp_pos"], c["defect_pos"])
            if ok:
                kept.append(c)
            else:
                rejected.append(c)
        clusters.append({"seed": seed, "members": kept})
        pool = rejected + pool   # rejected members go back for their own cluster
    log(f"  cluster : {len(candidates)} candidates -> {len(clusters)} "
        f"clusters (radius {radius} m)")
    return clusters


# TERRAIN EMBEDDING, found by actually flying a first attempt, not guessed
# in advance: a waypoint near the south abutment (x=-0.4, z=0.78) passed its
# line-of-sight ray to the defect (the ray to the defect doesn't cross the
# embankment) but the waypoint ITSELF was below the embankment's own rising
# grade there -- PyBullet's penetration response flung the real aircraft
# ~85 m off course on the very next test flight, diagnosed from
# flight_log.json's achieved positions, not assumed. Cheap and exact:
# params_final.ground_z(x, y) is a closed-form terrain height, no ray-cast
# needed.
#
# A second filter -- a 6-axis isotropic clearance probe against nearby
# STRUCTURE, not terrain -- was also tried and rejected: even at 0.4 m
# radius (well under the X8's own ~0.76 m rotor-tip reach) it dropped 102
# of 163 clusters, including many perfectly ordinary close shots of a
# single member's face. Real inspection holds the camera axis closer than
# the rotor span while the airframe's OTHER sides retain room, which an
# isotropic probe cannot distinguish from genuine multi-sided embedding.
# Rather than fabricate a directional model this pass has no time to
# validate, the real test is Stage B's actual PyBullet flight: a waypoint
# that truly cannot be held collides for real and shows up as a large
# settle error there, reported and excluded from Stage C's renders, not
# silently accepted.
#
# That plan met reality: a --limit=10 PyBullet test flight showed the
# aircraft wedging solid into BR_GIRDER_003_G1 at WP_003 and never freeing
# itself for four waypoints after it (settle error climbing 1.7 m -> 11.6 m
# while its ACTUAL position stayed frozen within millimetres) -- burning
# ~34 Wh/waypoint versus ~1-3 Wh for a normal settle, which on the full
# 145-waypoint mission would blow the entire 1,040 Wh usable budget on a
# handful of stuck legs alone. Root cause, found by checking the waypoint
# against the SAME box/cylinder primitives `avian_bridge_collision.json`
# gives PyBullet: the collision hull is a coarser, often LARGER wrap around
# the visual mesh (e.g. a box standing in for a real I-beam profile), so a
# point that clears the rendered geometry `has_line_of_sight()` checks
# against can still sit inside the physics hull. 43/145 waypoints in the
# current mission are literally inside a collision primitive (signed
# distance <= 0), one by 1.93 m. This is not the rejected isotropic probe
# -- it is not a comfort-margin heuristic, it is testing membership in the
# EXACT solid PyBullet will refuse to let the airframe occupy, using its
# own geometry. A small positive margin covers the airframe's own radius.
GROUND_CLEARANCE_MARGIN_M = 0.5
STRUCTURE_EMBED_MARGIN_M = 0.05

COLLISION_JSON = os.path.join(SCENE_DIR, "collision",
                              "avian_bridge_collision.json")


_COLLISION_PRIMITIVES_CACHE = None


def _load_collision_primitives():
    global _COLLISION_PRIMITIVES_CACHE
    if _COLLISION_PRIMITIVES_CACHE is None:
        with open(COLLISION_JSON) as f:
            _COLLISION_PRIMITIVES_CACHE = json.load(f)["primitives"]
    return _COLLISION_PRIMITIVES_CACHE


def _signed_clearance(pos, prim):
    """Signed distance from `pos` to `prim`'s surface: negative means
    `pos` is inside. Box distance is computed in the box's own (yawed)
    local frame, matching how PyBullet actually places these hulls."""
    c = prim["centre"]
    if prim["type"] == "BOX":
        he = prim["half_extents"]
        yaw = prim.get("yaw", 0.0)
        dx0, dy0 = pos.x - c[0], pos.y - c[1]
        ca, sa = math.cos(-yaw), math.sin(-yaw)
        lx = dx0 * ca - dy0 * sa
        ly = dx0 * sa + dy0 * ca
        lz = pos.z - c[2]
        ox, oy, oz = abs(lx) - he[0], abs(ly) - he[1], abs(lz) - he[2]
        if ox < 0 and oy < 0 and oz < 0:
            return max(ox, oy, oz)      # inside: distance to nearest face
        return math.sqrt(max(ox, 0)**2 + max(oy, 0)**2 + max(oz, 0)**2)
    r, hh = prim["radius"], prim["half_height"]
    oz = abs(pos.z - c[2]) - hh
    oxy = math.hypot(pos.x - c[0], pos.y - c[1]) - r
    if oz < 0 and oxy < 0:
        return max(oz, oxy)
    return math.sqrt(max(oz, 0)**2 + max(oxy, 0)**2)


def _min_structure_clearance(pos, primitives):
    return min(_signed_clearance(pos, p) for p in primitives)


def filter_flyable(clusters, log=print):
    """Ground-embedding, structure-hull-embedding, and line-of-sight -- see
    the module comment above for why the structure check tests literal
    collision-hull membership rather than a clearance-probe heuristic."""
    flyable = []
    uncovered = []
    n_below_grade = 0
    n_embedded = 0
    primitives = _load_collision_primitives()
    for cl in clusters:
        seed = cl["seed"]
        wp = seed["wp_pos"]
        grade = PF.ground_z(wp.x, wp.y)
        if wp.z < grade + GROUND_CLEARANCE_MARGIN_M:
            n_below_grade += 1
            for m in cl["members"]:
                uncovered.append({
                    "defect_id": m["defect_id"], "type": m["type"],
                    "reason": "waypoint sits at or below the terrain's own "
                             f"grade there (waypoint z={wp.z:.2f} m, "
                             f"grade={grade:.2f} m, needs +"
                             f"{GROUND_CLEARANCE_MARGIN_M} m clearance)",
                })
            continue
        clearance = _min_structure_clearance(wp, primitives)
        if clearance < STRUCTURE_EMBED_MARGIN_M:
            n_embedded += 1
            for m in cl["members"]:
                uncovered.append({
                    "defect_id": m["defect_id"], "type": m["type"],
                    "reason": "waypoint is inside or touching the PyBullet "
                             f"collision hull (clearance {clearance:.3f} m, "
                             f"needs +{STRUCTURE_EMBED_MARGIN_M} m)",
                })
            continue
        ok, clear = has_line_of_sight(wp, seed["defect_pos"])
        if not ok:
            for m in cl["members"]:
                uncovered.append({
                    "defect_id": m["defect_id"], "type": m["type"],
                    "reason": "waypoint has no clear line of sight to its "
                             "own seed defect (embedded in or blocked by "
                             f"structure, ray reached {clear:.2f} m of "
                             f"{(wp-seed['defect_pos']).length:.2f} m needed)",
                })
            continue
        flyable.append(cl)
    n_covered = sum(len(c["members"]) for c in flyable)
    log(f"  filter  : {len(flyable)}/{len(clusters)} clusters flyable "
        f"({n_below_grade} below grade, {n_embedded} collision-hull "
        f"embedded), {n_covered} defects covered, {len(uncovered)} uncovered")
    return flyable, uncovered


# Transit ideally wants the aircraft's full ~0.76 m rotor-tip reach clear
# on every side (unlike an endpoint, which is allowed to sit closer than
# that on ONE face by design -- the whole reason the isotropic per-waypoint
# clearance probe was rejected above). In practice, inside a dense steel
# truss corridor, a 0.4 m margin leaves 84/104 legs with no candidate among
# their 12 nearest neighbours satisfying it at all -- close inspection
# waypoints are simply packed too tightly among the members for that much
# clearance to exist on every transit. Falling back to the same ~0 m
# "don't be literally inside a primitive" margin as the endpoint check
# still catches the worst case actually observed (a straight line clipping
# solidly through a pier column), even though it can't guarantee a full
# rotor-span graze never happens -- confirmed the controller itself is not
# at fault first: the exact relative move that crashed completes cleanly
# at err 0.13 m in open air with nothing nearby. Real, un-preventable
# grazes are handled at flight time instead: see flight_final.py's
# stuck-detection and recovery-climb logic.
TRANSIT_CLEARANCE_MARGIN_M = STRUCTURE_EMBED_MARGIN_M


def _segment_clear(a, b, primitives, margin=TRANSIT_CLEARANCE_MARGIN_M,
                   sample_step=0.2):
    """Samples the straight-line transit from `a` to `b` against the SAME
    collision hull the endpoint check uses. Endpoint clearance alone isn't
    enough: two waypoints can each be individually clear while the direct
    line between them still clips a pier column or girder standing between
    them (found by flying a real ordered mission -- WP_010->WP_011 climbed
    8.9 m in altitude while passing within ~1 m of BR_PIER_COL_003 and the
    aircraft ended up wedged against it at 0.5 m off the ground for the
    rest of the flight)."""
    length = (b - a).length
    if length < 1e-6:
        return True
    n = max(2, int(length / sample_step) + 1)
    for i in range(n + 1):
        p = a.lerp(b, i / n)
        if _min_structure_clearance(p, primitives) < margin:
            return False
    return True


def order_nearest_neighbour(clusters, start_pos, log=print):
    """Nearest-neighbour heuristic -- not optimal TSP, but the standard,
    cheap first choice, and stated as such rather than dressed up as more
    than it is. Among the nearest candidates at each step, picks the first
    whose direct transit path is actually clear of structure (see
    `_segment_clear`); only falls back to the plain-nearest, flagged as a
    transit risk, if none of them are."""
    primitives = _load_collision_primitives()
    remaining = list(clusters)
    ordered = []
    n_risky = 0
    cur = Vector(start_pos)
    while remaining:
        remaining.sort(key=lambda c: (c["seed"]["wp_pos"] - cur).length)
        chosen = None
        for cand in remaining[:12]:
            if _segment_clear(cur, cand["seed"]["wp_pos"], primitives):
                chosen = cand
                break
        if chosen is None:
            chosen = remaining[0]
            chosen["transit_risk"] = True
            n_risky += 1
        remaining.remove(chosen)
        ordered.append(chosen)
        cur = chosen["seed"]["wp_pos"]
    log(f"  order   : nearest-clear-neighbour, {len(ordered)} waypoints "
        f"({n_risky} could not find a clear transit path)")
    return ordered


def _drone_base_position():
    ob = bpy.data.objects.get("AVI_BASE_SCANNER")
    if ob is not None and "avi_pad_centre_m" in ob.keys():
        c = list(ob["avi_pad_centre_m"])
        return Vector((c[0], c[1], c[2] + 2.0))   # 2 m above the pad -- takeoff altitude
    return Vector((20.0, -30.0, 2.0))


# ---------------------------------------------------------------------------
# energy estimate -- sourced, not invented. AVIAN_UAV/logs/phase4_dynamics.json
# is a REAL logged flight test: mission_time_s=70.0, mission_energy_Wh=51.76
# (average power ~2,662 W across takeoff+hover+5 waypoints+settle), and its
# own settling_time_s=2.54 s and controller.py's own LIMITS["v_xy_max"]=4.0
# m/s. Stage A's number is an ESTIMATE built from these; Stage B's real
# PyBullet flight is what actually measures it.
# ---------------------------------------------------------------------------
_REF_MISSION_TIME_S = 70.0
_REF_MISSION_ENERGY_WH = 51.76
_REF_AVG_POWER_W = _REF_MISSION_ENERGY_WH * 3600.0 / _REF_MISSION_TIME_S
_CRUISE_SPEED_MPS = 4.0          # controller.py LIMITS["v_xy_max"]
_SETTLE_TIME_S = 2.54            # phase4_dynamics.json settling_time_s
_TAKEOFF_TIME_S = 4.4            # phase4_dynamics.json takeoff_time_s


def estimate_time_energy(total_distance_m, n_waypoints):
    cruise_s = total_distance_m / _CRUISE_SPEED_MPS
    settle_s = n_waypoints * _SETTLE_TIME_S
    total_s = _TAKEOFF_TIME_S + cruise_s + settle_s
    energy_wh = total_s * _REF_AVG_POWER_W / 3600.0
    return round(total_s, 1), round(energy_wh, 2)


def build_mission(log=print):
    records = DSF._load_records()
    candidates = [candidate_for(r) for r in records]
    log(f"  defects : {len(candidates)} candidates from ground truth")

    clusters = cluster_candidates(candidates, log=log)
    flyable, uncovered = filter_flyable(clusters, log=log)

    base_pos = _drone_base_position()
    ordered = order_nearest_neighbour(flyable, base_pos, log=log)

    waypoints = []
    cum_dist = 0.0
    prev = base_pos
    for i, cl in enumerate(ordered, start=1):
        seed = cl["seed"]
        leg = (seed["wp_pos"] - prev).length
        cum_dist += leg
        prev = seed["wp_pos"]
        waypoints.append({
            "waypoint_id": f"WP_{i:03d}",
            "position_m": [round(v, 4) for v in seed["wp_pos"]],
            "heading_rad": round(seed["heading_rad"], 4),
            "heading_deg": round(math.degrees(seed["heading_rad"]), 2),
            "leg_distance_m": round(leg, 3),
            "cumulative_distance_m": round(cum_dist, 3),
            "defects_covered": [
                {"defect_id": m["defect_id"], "type": m["type"],
                 "feature_size_mm": m["feature_size_mm"],
                 "min_detect_range_m": m["min_detect_range_m"],
                 "defect_background_contrast": m["defect_background_contrast"],
                 "contrast_limited": m["contrast_limited"],
                 "escalation_reason": m["escalation_reason"],
                 "avi_condition": m["avi_condition"],
                 "host_structure": m["host_structure"],
                 "severity": m["severity"],
                 "position_m": [round(v, 4) for v in m["defect_pos"]],
                 "view_direction": [round(v, 4) for v in m["view_dir"]]}
                for m in cl["members"]],
        })
    cum_dist += (base_pos - prev).length   # return leg

    total_s, energy_wh = estimate_time_energy(cum_dist, len(waypoints))

    manifest = {
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "base_position_m": [round(v, 4) for v in base_pos],
        "cluster_radius_m": CLUSTER_RADIUS_M,
        "ordering_method": "nearest_neighbour",
        "sensor": {"hfov_deg": None, "loop_res": None},  # filled by render stage's own read
        "n_defects_total": len(candidates),
        "n_waypoints": len(waypoints),
        "n_defects_covered": sum(len(w["defects_covered"]) for w in waypoints),
        "n_defects_uncovered": len(uncovered),
        "uncovered_defects": uncovered,
        "total_distance_m": round(cum_dist, 3),
        "estimated_time_s": total_s,
        "estimated_energy_Wh": energy_wh,
        "energy_estimate_method": (
            f"takeoff {_TAKEOFF_TIME_S} s + cruise (distance / "
            f"{_CRUISE_SPEED_MPS} m/s) + {len(waypoints)} x "
            f"{_SETTLE_TIME_S} s settle, at {_REF_AVG_POWER_W:.0f} W "
            f"average power -- sourced from AVIAN_UAV/logs/"
            f"phase4_dynamics.json's own logged 70 s/51.76 Wh test "
            f"mission, not invented. Stage B's real PyBullet flight "
            f"measures the actual number."),
        "battery_Wh_nominal": 1300.0,
        "battery_reserve_pct": 20.0,
        "battery_Wh_usable": 1040.0,
        "waypoints": waypoints,
    }
    return manifest


def main():
    log("== SIH_AVIAN_FINAL :: mission_final.py (two-panel Stage A) ==")
    bpy.ops.wm.open_mainfile(filepath=BLEND_PATH)
    os.makedirs(MISSION_DIR, exist_ok=True)

    manifest = build_mission(log=log)

    log(f"  mission : {manifest['n_waypoints']} waypoints, "
        f"{manifest['n_defects_covered']}/{manifest['n_defects_total']} "
        f"defects covered, {manifest['n_defects_uncovered']} uncovered")
    log(f"  flight  : {manifest['total_distance_m']:.1f} m, "
        f"~{manifest['estimated_time_s']:.0f} s, "
        f"~{manifest['estimated_energy_Wh']:.1f} Wh estimated "
        f"(of {manifest['battery_Wh_usable']:.0f} Wh usable)")

    if manifest["uncovered_defects"]:
        by_reason = {}
        for u in manifest["uncovered_defects"]:
            by_reason[u["type"]] = by_reason.get(u["type"], 0) + 1
        log(f"  uncovered by type: {by_reason}")

    out_path = os.path.join(MISSION_DIR, "mission.json")
    with open(out_path, "w") as f:
        json.dump(manifest, f, indent=2)
    log(f"  saved   : {out_path}")

    with open(os.path.join(MISSION_DIR, "AVIAN_mission_build_log_FINAL.txt"),
             "w") as f:
        f.write("\n".join(LOG_LINES))

    log(f"== done in {time.time()-T0:.1f} s ==")
    print("FINAL_MISSION_COMPLETE")


if __name__ == "__main__":
    main()
