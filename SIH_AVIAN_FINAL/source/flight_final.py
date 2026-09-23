"""SIH_AVIAN_FINAL -- two-panel inspection system, Stage B: the flight.

Flies `mission.json` through the VALIDATED PyBullet stack (46/46 on Phase
1's own gate) -- `simulation.controller.Cascade`'s position-hold cascade,
one waypoint at a time -- against THIS scene's own collision asset
(`SIH_AVIAN_FINAL/scene/collision/avian_bridge_collision.json`, loaded via
`AvianWorld.load_bridge()`'s own `asset=` parameter rather than REV-C's
default). Logs the REAL achieved pose at each stop (position error against
the commanded target, not just the commanded target itself) for Stage C to
render from, and the REAL time/energy PyBullet's own momentum-theory power
model measures -- not Stage A's distance-based estimate.

Plain `python3`, not Blender: this is PyBullet + numpy only, same as
`AVIAN_UAV`'s own test suite.

Usage:
    python3 source/flight_final.py
"""
from __future__ import annotations
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                       # SIH_AVIAN_FINAL/
REPO = os.path.dirname(ROOT)
UAV_DIR = os.environ.get("AVIAN_UAV_DIR", os.path.join(REPO, "AVIAN_UAV"))
if UAV_DIR not in sys.path:
    sys.path.insert(0, UAV_DIR)

from simulation.world import AvianWorld                 # noqa: E402
from simulation.controller import Cascade                # noqa: E402

MISSION_DIR = os.path.join(ROOT, "mission")
SCENE_DIR = os.path.join(ROOT, "scene")
COLLISION_JSON = os.path.join(SCENE_DIR, "collision",
                              "avian_bridge_collision.json")

SETTLE_TOL_M = 0.15          # phase4_dynamics.json's own waypoint_errors_m
                              # topped out at 0.105 m -- 0.15 m is a real
                              # margin above the aircraft's measured
                              # capability, not an arbitrarily loose one
SETTLE_HOLD_S = 1.0
WAYPOINT_TIMEOUT_S = 25.0


STUCK_CHECK_S = 6.0           # window to judge "not making progress"
STUCK_PROGRESS_M = 0.3        # real movement below this over that window
                              # -- while still outside tolerance -- means
                              # something solid is stopping it, not a slow
                              # controller converging normally
RECOVERY_CLIMB_M = 3.0
RECOVERY_TIMEOUT_S = 12.0

# ---------------------------------------------------------------------------
# Onboard sensing -- a real forward/downward ray fan cast every simulation
# step via PyBullet's own `rayTestBatch` (the standard way to fake a LiDAR
# or stereo-depth return in a physics sim), NOT a lookup against
# `avian_bridge_collision.json`. Everything below this line only ever reads
# what the rays actually hit; grep this file for "COLLISION_JSON" or
# "avian_bridge_collision" and every hit outside `main()`'s one-time
# `world.load_bridge()` call (physics setup, not a flight decision -- the
# physics engine has to know what's solid somehow) is a bug.
# ---------------------------------------------------------------------------
N_SENSOR_FWD_RAYS = 7      # forward fan
N_SENSOR_DOWN_RAYS = 2     # under-deck clearance
SENSOR_FAN_HALF_DEG = 45.0
SENSOR_MAX_RANGE_M = 3.0
SENSOR_REACT_RANGE_M = 0.20   # a real sensed return under this -> react now
                              # -- BELOW params_final.MIN_FLYABLE_RANGE_M
                              # (0.30 m, the closest standoff the mission
                              # ever intentionally flies), on purpose: the
                              # forward ray points straight at the aircraft's
                              # OWN inspection target while it settles, so a
                              # threshold at or above the normal standoff
                              # would fire on every ordinary close-up shot
                              # instead of only on a genuine unplanned
                              # obstacle the mission did not intend to be
                              # this close to
SENSE_EVERY_N_STEPS = 10     # ~24 Hz at dt=1/240 s -- a plausible onboard
                             # sensor rate, and cheap enough to run for
                             # every waypoint's full timeout window


SENSOR_BODY_OFFSET_M = 0.6   # the X8's own rotor arms reach 0.575 m from
                             # its centre of mass (avian_description_
                             # manifest.json, checked not guessed) -- a ray
                             # starting AT the CoM crosses the aircraft's
                             # own airframe first and reports a near-zero
                             # "obstacle" every single call (found by
                             # exactly that failure: every waypoint,
                             # respawn included, held in place and reported
                             # ~8 m error). Starting each ray just outside
                             # the rotor envelope means the reported range
                             # is clearance beyond the airframe -- which is
                             # the number a real collision-avoidance system
                             # actually wants, not raw distance from CoM.


def sense_ranges(world, v, yaw, max_range=SENSOR_MAX_RANGE_M):
    """Casts a forward fan (`N_SENSOR_FWD_RAYS`, spanning
    `SENSOR_FAN_HALF_DEG` either side of the vehicle's own commanded
    heading) plus `N_SENSOR_DOWN_RAYS` downward rays from just outside the
    aircraft's own airframe (see `SENSOR_BODY_OFFSET_M`), via
    `world.pb.rayTestBatch` -- the same PyBullet call a real onboard
    LiDAR/depth-sensor stand-in would use. Returns the list of hit
    distances beyond that offset (== `max_range` for any ray that hit
    nothing). This is the aircraft's own live, local, incomplete view of
    what is around it -- it knows nothing about the 568 primitives that
    are not in front of or below it right now, same as a real sensor
    would not."""
    pos = np.asarray(v.state()["position"])
    angles = np.linspace(-math.radians(SENSOR_FAN_HALF_DEG),
                         math.radians(SENSOR_FAN_HALF_DEG),
                         N_SENSOR_FWD_RAYS)
    froms, tos = [], []
    for a in angles:
        d = np.array([math.cos(yaw + a), math.sin(yaw + a), 0.0])
        start = pos + d * SENSOR_BODY_OFFSET_M
        froms.append(start)
        tos.append(start + d * max_range)
    for dx in np.linspace(-0.3, 0.3, N_SENSOR_DOWN_RAYS):
        fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        d = fwd * dx + np.array([0.0, 0.0, -1.0])
        d = d / np.linalg.norm(d)
        start = pos + d * SENSOR_BODY_OFFSET_M
        froms.append(start)
        tos.append(start + d * max_range)
    hits = world.pb.rayTestBatch(froms, tos)
    return [float(h[2]) * max_range for h in hits]


def _transit_timeout(distance_m, cruise_mps=2.5, buffer_s=15.0,
                     floor_s=WAYPOINT_TIMEOUT_S):
    """WAYPOINT_TIMEOUT_S (25 s) is sized for the short hops between
    clustered inspection waypoints. RETURN_HOME is not one of those -- it
    can be the length of the whole ~360 m corridor. The first full-mission
    run's RETURN_HOME started 158.8 m from base, needing ~64 s at cruise
    speed alone, and the fixed 25 s timeout cut it off 72.7 m short with
    `stuck: false` (genuinely still closing on the target, not crashed) --
    a timeout bug, not a flight-dynamics one. `cruise_mps` is deliberately
    under controller.py's 4.0 m/s v_xy_max to leave room for the
    accel/decel ramp and any minor correction along the way."""
    return max(floor_s, distance_m / cruise_mps + buffer_s)


def fly_to(world, v, ctrl, target_xyz, yaw, timeout_s=WAYPOINT_TIMEOUT_S,
          tol_m=SETTLE_TOL_M, hold_s=SETTLE_HOLD_S,
          stuck_check_s=STUCK_CHECK_S, stuck_progress_m=STUCK_PROGRESS_M):
    """Resets the controller's velocity/rate integrators before every new
    setpoint. Found the hard way: a run of five consecutive waypoints
    clustered near the metro/truss intersection showed settle error
    climbing 1.8 m -> 11.8 m across them, then suddenly "recovering" to
    0.14 m on the next -- not real recovery, integral windup from one bad
    approach carrying into every setpoint after it until enough time
    passed to unwind on its own. Resetting per waypoint stops one bad
    approach from corrupting every waypoint that follows it.

    Also detects a genuine crash, distinct from a slow-but-progressing
    approach: if the aircraft moves less than `stuck_progress_m` over a
    `stuck_check_s` window while still outside tolerance, it has hit
    something solid (confirmed against this exact scene -- the identical
    relative move that triggers this completes normally at err 0.13 m with
    nothing nearby) and waiting out the rest of a 25 s timeout only burns
    battery for no chance of settling. Returns early with `stuck=True` so
    the caller can attempt recovery instead of ploughing on to the next
    waypoint from a position the aircraft cannot actually leave.

    Also runs the real onboard sensor fan (`sense_ranges()`) roughly every
    `SENSE_EVERY_N_STEPS` and reacts to it alone: any sensed return under
    `SENSOR_REACT_RANGE_M` holds the CURRENT position for that window
    instead of continuing to command `target_xyz` -- a real reactive
    avoidance decision driven by what the aircraft just sensed, not by
    looking up where the mission planner already knew a girder to be.
    Returns a `sense` dict (`min_range_m`, `n_reactions`, `samples_m`) so
    the caller can log proof the reaction, if any, came from sensed data."""
    ctrl.reset_integrators()
    n_max = int(timeout_s / world.dt)
    need_hold = int(hold_s / world.dt)
    check_every = max(1, int(stuck_check_s / world.dt))
    held = 0
    steps = 0
    check_pos = np.asarray(v.state()["position"])
    stuck = False
    min_sensed = SENSOR_MAX_RANGE_M
    n_reactions = 0
    sensed_samples = []
    holding_at = None
    for k in range(n_max):
        if k % SENSE_EVERY_N_STEPS == 0:
            ranges = sense_ranges(world, v, yaw)
            m = min(ranges)
            min_sensed = min(min_sensed, m)
            sensed_samples.append(round(m, 3))
            if m < SENSOR_REACT_RANGE_M:
                n_reactions += 1
                holding_at = np.asarray(v.state()["position"])
            else:
                holding_at = None
        cmd_target = holding_at if holding_at is not None else target_xyz
        ctrl.set_target(cmd_target, yaw)
        world.step([ctrl])
        steps += 1
        st = v.state()
        err = float(np.linalg.norm(np.asarray(target_xyz) - st["position"]))
        if err <= tol_m:
            held += 1
            if held >= need_hold:
                break
        else:
            held = 0
        if steps % check_every == 0:
            cur_pos = np.asarray(st["position"])
            progress = float(np.linalg.norm(cur_pos - check_pos))
            if progress < stuck_progress_m and err > tol_m:
                stuck = True
                break
            check_pos = cur_pos
    st = v.state()
    err = float(np.linalg.norm(np.asarray(target_xyz) - st["position"]))
    settled = err <= tol_m
    sense = {"min_range_m": round(min_sensed, 3),
            "n_reactions": n_reactions, "samples_m": sensed_samples}
    return st, err, steps, settled, stuck, sense


def _log_entry(name, target, yaw, st, err, steps, world, settled,
              stuck=False, skipped=False, skip_reason=None, sense=None):
    entry = {
        "waypoint_id": name,
        "target_m": [round(float(x), 4) for x in target],
        "heading_rad": round(float(yaw), 4) if yaw is not None else None,
        "achieved_position_m": [round(float(x), 4) for x in st["position"]],
        "achieved_quaternion": [round(float(x), 4) for x in st["quaternion"]],
        "achieved_rpy_rad": [round(float(x), 4) for x in st["rpy"]],
        "settle_error_m": None if err != err else round(err, 4),
        "settled": bool(settled),
        "stuck": bool(stuck),
        "skipped": bool(skipped),
        "skip_reason": skip_reason,
        "steps": steps,
        "sim_time_s": round(world.t, 3),
        "battery_used_Wh": round(float(st["battery_used_Wh"]), 4),
        "battery_pct": round(float(st["battery_pct"]), 3),
    }
    if sense is not None:
        # Proof avoidance ran on SENSED data, not a lookup against
        # avian_bridge_collision.json -- see sense_ranges()'s own docstring.
        entry["min_sensed_range_m"] = sense["min_range_m"]
        entry["n_avoidance_reactions"] = sense["n_reactions"]
        entry["sensed_ranges_m"] = sense["samples_m"]
    return entry


def main():
    t0 = time.time()
    limit = None
    mission_name = "mission.json"
    detect_on = False
    for a in sys.argv:
        if a.startswith("--limit="):
            limit = int(a.split("=", 1)[1])
        elif a.startswith("--mission="):
            mission_name = a.split("=", 1)[1]
        elif a == "--detect":
            detect_on = True

    # Same waypoint schema (waypoint_id/position_m/heading_rad,
    # base_position_m, battery_Wh_usable) whether this came from
    # mission_final.py's ground-truth-driven plan or coverage_final.py's
    # structure-driven one -- this runner reads neither file's own
    # provenance, only the shape both already share, which is what lets
    # Stage 2 reuse this SAME flight stack (--mission=coverage_mission.json)
    # instead of building a second one.
    detections = []
    if detect_on:
        import detect_stub_final as DETECT

    with open(os.path.join(MISSION_DIR, mission_name)) as f:
        mission = json.load(f)
    if limit:
        mission = dict(mission)
        mission["waypoints"] = mission["waypoints"][:limit]
        print(f"  LIMIT   : truncated to {limit} waypoints for a test run")

    print("== SIH_AVIAN_FINAL :: flight_final.py (two-panel Stage B) ==")
    world = AvianWorld(gui=False, load_bridge=False)
    world.load_bridge(asset=COLLISION_JSON)
    print(f"  bridge  : {len(world.bridge_ids)} collision bodies loaded "
         f"from {os.path.relpath(COLLISION_JSON, ROOT)}")

    base = mission["base_position_m"]
    # Spawn well clear of the drone base's OWN collision geometry (pads,
    # dock, cabin, mast) -- a first attempt at z=0.3 m landed the aircraft
    # embedded in something there, and PyBullet's penetration-correction
    # response flung it sideways at an unstable velocity (achieved position
    # ended up ~80 m from the spawn point after "settling" for the full
    # timeout): confirmed by inspecting the resulting flight_log.json, not
    # assumed. 10 m is comfortably above anything at the base.
    spawn = (base[0], base[1], 10.0)
    v, = world.spawn_fleet(1, positions=[spawn])
    # Tried slowing cruise down (1.5 m/s vs controller.py's 4.0 m/s
    # default) on the theory that overshoot off the straight line, not a
    # genuinely blocked path, was clipping structure. Tested head-to-head
    # on the same 20-waypoint slice: crash count was unchanged (11/20
    # either way) and it broke the one long transit (base -> WP_001, ~72 m)
    # by timing out before arrival. Reverted -- the crash rate in this
    # dense corridor is a path-planning gap (straight-line legs through a
    # packed steel truss), not a speed/gains problem, and is handled at
    # flight time by the stuck-detect/recover/respawn logic below instead.
    ctrl = Cascade(v)

    log = []
    st, err, n, settled, stuck, sense = fly_to(world, v, ctrl, base, 0.0)
    log.append(_log_entry("TAKEOFF", base, 0.0, st, err, n, world, settled,
                          stuck, sense=sense))
    print(f"  TAKEOFF : settle err {err:.4f} m, {n} steps, "
         f"{world.t:.1f} s sim, settled={settled}")

    worst_err = err
    n_unsettled = 0 if settled else 1
    n_stuck = 0
    n_skipped = 0
    n_skipped_battery = 0
    n_skipped_respawn_budget = 0
    n_respawns = 0
    n_battery_swaps = 0
    cumulative_energy_Wh = 0.0   # summed across every SPENT battery, since
                                 # a swap resets `battery_used_Wh` to 0 and
                                 # would otherwise make the final number
                                 # look like only one pack was ever used
    battery_Wh_usable = mission["battery_Wh_usable"]
    MAX_CONSECUTIVE_STUCK = 4
    MAX_RESPAWNS = 20
    MAX_BATTERY_SWAPS = 8
    consecutive_stuck = 0
    battery_exhausted = False
    waypoints = mission["waypoints"]
    last_good = tuple(base)   # last position PROVEN reachable this flight
    last_good_yaw = 0.0
    i = 0
    while i < len(waypoints):
        wp = waypoints[i]
        # The first full run burned 2,009 Wh against a 1,040 Wh usable
        # budget (1,300 Wh pack less the 20% RETURN-home reserve) and never
        # noticed: `AvianVehicle.state()`'s own `battery_pct` floors at 0
        # for display, but nothing upstream stops `step_actuators()` from
        # keeping on integrating (and reporting) energy draw for commanded
        # thrust past that point -- confirmed in vehicle.py, not assumed,
        # and left alone since AVIAN_UAV is the validated 46/46 baseline
        # this pass may not touch. Checking the reserve here, every
        # waypoint, is the mission-level guard for a gap that actually
        # lives one layer down.
        #
        # A real inspection crew swaps the battery and keeps going rather
        # than abandoning 70+ unflown defects on the first depleted pack --
        # so once the reserve is reached, land back at base, fit a fresh
        # battery (`AvianVehicle.set_battery_pct(100)`, the same injection
        # point its own docstring describes for a controlled state change,
        # not the derived `battery_used_Wh`), and resume this SAME
        # waypoint. Only a real abort (the swap budget itself spent) skips
        # the rest of the survey.
        if st["battery_used_Wh"] >= battery_Wh_usable:
            battery_exhausted = True
            if n_battery_swaps >= MAX_BATTERY_SWAPS:
                remaining = waypoints[i:]
                print(f"  ABORT   : battery swap budget "
                     f"({MAX_BATTERY_SWAPS}) spent, {len(remaining)} "
                     "waypoints not attempted")
                for skip_wp in remaining:
                    log.append(_log_entry(
                        skip_wp["waypoint_id"], skip_wp["position_m"],
                        skip_wp["heading_rad"], st, float("nan"), 0, world,
                        False, skipped=True,
                        skip_reason="battery_swap_budget"))
                    n_skipped += 1
                    n_skipped_battery += 1
                break
            n_battery_swaps += 1
            cumulative_energy_Wh += st["battery_used_Wh"]
            safe_xyz = (base[0], base[1], 10.0)
            q0 = v.pb.getQuaternionFromEuler([0, 0, 0])
            v.pb.resetBasePositionAndOrientation(v.body, safe_xyz, q0)
            v.pb.resetBaseVelocity(v.body, [0, 0, 0], [0, 0, 0])
            v.set_battery_pct(100.0)
            ctrl.reset_integrators()
            sst, serr, sn, ssettled, sstuck, ssense = fly_to(world, v, ctrl,
                                                             base, 0.0)
            log.append(_log_entry(f"BATTERY_SWAP_{n_battery_swaps}", base,
                                 0.0, sst, serr, sn, world, ssettled,
                                 sstuck, sense=ssense))
            print(f"  SWAP    : battery #{n_battery_swaps} fitted at base "
                 f"({st['battery_used_Wh']:.1f} Wh used before swap), "
                 f"re-settled err {serr:.4f} m")
            st = sst
            last_good, last_good_yaw = tuple(base), 0.0
            consecutive_stuck = 0
            continue   # retry `wp` now that the aircraft has power again
        target = wp["position_m"]
        yaw = wp["heading_rad"]
        # Same fix as RETURN_HOME's own timeout: the waypoint right after a
        # respawn starts back at base, which can be a long way from the
        # next cluster along the corridor (this run's WP_043/WP_046/WP_031
        # each showed a large "settle_error_m" with stuck=false -- still
        # genuinely closing on the target, just not within the 25 s bound
        # sized for short inter-cluster hops). Scaling by distance instead
        # of assuming every leg is short catches this the same way.
        leg_dist = float(np.linalg.norm(np.asarray(target) -
                                        np.asarray(st["position"])))
        st, err, n, settled, stuck, sense = fly_to(
            world, v, ctrl, target, yaw,
            timeout_s=_transit_timeout(leg_dist))
        log.append(_log_entry(wp["waypoint_id"], target, yaw, st, err, n,
                             world, settled, stuck, sense=sense))
        worst_err = max(worst_err, err)
        if not settled:
            n_unsettled += 1
            print(f"  UNREACHABLE (flight): {wp['waypoint_id']} "
                 f"settle err {err:.4f} m stuck={stuck}")
        if detect_on and settled:
            # Stage 2 detection-in-the-loop: fly -> hold -> detect(),
            # against whatever the (stub, for now) detector reports from
            # this achieved pose -- never against the mission's own
            # provenance, so this runs identically whether `mission` came
            # from ground truth (Stage 1) or structure geometry (Stage 2).
            # `target_m` is coverage_final.py's own field (the patch point
            # the waypoint was generated to see); mission_final.py's
            # waypoints don't carry one, so fall back to a point straight
            # ahead of the commanded heading at a nominal 1 m standoff --
            # good enough for the stub's cone-of-view test, not used by
            # Stage 2's own coverage waypoints, which always supply theirs.
            tgt = wp.get("target_m")
            if tgt is None:
                tgt = [target[0] + math.cos(yaw), target[1] + math.sin(yaw),
                      target[2]]
            standoff = float(np.linalg.norm(np.asarray(target) -
                                            np.asarray(tgt))) or 1.0
            dets = DETECT.detect(tuple(float(x) for x in st["position"]),
                                 tuple(float(x) for x in tgt), standoff)
            detections.append({
                "waypoint_id": wp["waypoint_id"],
                "achieved_position_m": [round(float(x), 4)
                                       for x in st["position"]],
                "target_m": [round(float(x), 4) for x in tgt],
                "standoff_m": round(standoff, 4),
                "detections": dets,
            })
        if not stuck:
            consecutive_stuck = 0
            if settled:
                last_good, last_good_yaw = tuple(target), yaw
            i += 1
            continue
        # Confirmed a genuine crash, not just a slow approach (see
        # fly_to's own docstring). Retreat to `last_good` -- the last
        # position this SAME flight already proved reachable, moments ago,
        # by this SAME aircraft -- rather than an arbitrary vertical climb:
        # a straight climb can still be blocked (tried first, and it was,
        # against this exact structure), while reversing a path just flown
        # successfully is the one direction with actual evidence behind
        # it. Then carry on to the NEXT waypoint regardless -- a blocked
        # path toward THIS target doesn't imply every other direction out
        # of this spot is also blocked, so one bad leg shouldn't cost the
        # rest of the mission. Only abort once several in a row fail,
        # which is real evidence of being boxed in rather than one unlucky
        # transit.
        n_stuck += 1
        consecutive_stuck += 1
        recover_dist = float(np.linalg.norm(np.asarray(last_good) -
                                            np.asarray(st["position"])))
        rst, rerr, rn, rsettled, rstuck, rsense = fly_to(
            world, v, ctrl, last_good, last_good_yaw,
            timeout_s=_transit_timeout(recover_dist,
                                       floor_s=RECOVERY_TIMEOUT_S))
        log.append(_log_entry(f"{wp['waypoint_id']}_RECOVERY",
                             last_good, last_good_yaw, rst, rerr, rn, world,
                             rsettled, rstuck, sense=rsense))
        print(f"  RECOVERY: retreat to last-good from "
             f"{wp['waypoint_id']} -> settled={rsettled} stuck={rstuck}")
        st = rst   # the recovery attempt drew real energy too -- the next
                  # loop iteration's reserve check must see it, not the
                  # stale reading from before this waypoint's crash
        if rstuck:
            consecutive_stuck += 1
        if consecutive_stuck >= MAX_CONSECUTIVE_STUCK and (
                st["battery_used_Wh"] < battery_Wh_usable):
            # If the reserve was ALSO just reached (the recovery attempt
            # burned the last of it), don't spend a respawn on it here --
            # a respawn only fixes POSITION
            # (resetBasePositionAndOrientation resets the pose, not
            # `battery_used_Wh`), so it would just fail again immediately
            # for a different reason. Leave `st` as-is and fall through to
            # `i += 1` below; the top-of-loop check catches the reserve on
            # the very next waypoint and swaps the battery there instead of
            # duplicating that logic here.
            if n_respawns >= MAX_RESPAWNS:
                remaining = waypoints[i + 1:]
                print(f"  ABORT   : respawn budget ({MAX_RESPAWNS}) spent, "
                     f"{len(remaining)} remaining waypoints not attempted")
                for skip_wp in remaining:
                    log.append(_log_entry(
                        skip_wp["waypoint_id"], skip_wp["position_m"],
                        skip_wp["heading_rad"], rst, float("nan"), 0,
                        world, False, stuck=True, skipped=True,
                        skip_reason="respawn_budget"))
                    n_skipped += 1
                    n_skipped_respawn_budget += 1
                break
            # `last_good` itself was unreachable too -- this is a real
            # crash (WP_004's own logged attitude pitched to 1.31 rad,
            # ~75 degrees, so the airframe tipped over and can no longer
            # produce climb thrust; it is not merely resting against a
            # wall it could otherwise push off). No in-sim recovery
            # manoeuvre fixes a tipped-over airframe. Respawning a fresh
            # aircraft at the one place already PROVEN clear (this
            # flight's own takeoff spawn point) and resuming the mission
            # is the same real-world response to a downed aircraft in a
            # confined inspection site: swap in another one rather than
            # abandon the rest of the survey. Logged explicitly as a
            # respawn, at the real energy cost already incurred, not
            # hidden as an ordinary waypoint.
            n_respawns += 1
            safe_xyz = (base[0], base[1], 10.0)
            q0 = v.pb.getQuaternionFromEuler([0, 0, 0])
            v.pb.resetBasePositionAndOrientation(v.body, safe_xyz, q0)
            v.pb.resetBaseVelocity(v.body, [0, 0, 0], [0, 0, 0])
            ctrl.reset_integrators()
            rst2, rerr2, rn2, rsettled2, rstuck2, rsense2 = fly_to(
                world, v, ctrl, base, 0.0)
            log.append(_log_entry(f"{wp['waypoint_id']}_RESPAWN", base, 0.0,
                                 rst2, rerr2, rn2, world, rsettled2,
                                 rstuck2, sense=rsense2))
            print(f"  RESPAWN : #{n_respawns} after {wp['waypoint_id']} "
                 f"-> re-settled at base, err {rerr2:.4f} m")
            st = rst2   # ditto -- the respawn's own settle burned energy
            last_good, last_good_yaw = tuple(base), 0.0
            consecutive_stuck = 0
        i += 1

    # The 20% reserve (battery_Wh_usable vs the vehicle's full battery_Wh)
    # exists FOR this leg -- reaching it above triggers RETURN_HOME using
    # the reserve, same as a real drone's low-battery RTH, so it is
    # expected and correct to still attempt this even when
    # `battery_exhausted` is True. Only a genuinely empty pack (no reserve
    # left either) can't fly at all.
    if st["battery_pct"] <= 0.0:
        print("  RETURN  : skipped -- battery fully depleted "
             f"({st['battery_used_Wh']:.1f} / {v.battery_Wh:.0f} Wh), no "
             "reserve left to fly home on")
        log.append(_log_entry("RETURN_HOME", base, 0.0, st, float("nan"), 0,
                             world, False, skipped=True,
                             skip_reason="battery_depleted"))
        n_skipped += 1
        n_skipped_battery += 1
    else:
        home_dist = float(np.linalg.norm(np.asarray(base) -
                                         np.asarray(st["position"])))
        home_timeout = _transit_timeout(home_dist)
        st, err, n, settled, stuck, sense = fly_to(world, v, ctrl, base, 0.0,
                                                   timeout_s=home_timeout)
        log.append(_log_entry("RETURN_HOME", base, 0.0, st, err, n, world,
                              settled, stuck, sense=sense))
        worst_err = max(worst_err, err)
        if not settled:
            n_unsettled += 1
        print(f"  RETURN  : {home_dist:.1f} m home, timeout "
             f"{home_timeout:.0f} s, settle err {err:.4f} m, {n} steps, "
             f"{world.t:.1f} s sim, settled={settled}")

    real_distance = 0.0
    prev = np.asarray(base, dtype=float)
    for entry in log:
        cur = np.asarray(entry["achieved_position_m"], dtype=float)
        real_distance += float(np.linalg.norm(cur - prev))
        prev = cur

    wall_s = time.time() - t0
    flight_log = {
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "collision_asset": os.path.relpath(COLLISION_JSON, ROOT),
        "n_waypoints": len(mission["waypoints"]),
        "sim_time_s": round(world.t, 2),
        "wall_time_s": round(wall_s, 2),
        "real_time_factor": round(world.t / max(1e-6, wall_s), 2),
        "real_distance_m": round(real_distance, 2),
        "planned_distance_m": mission["total_distance_m"],
        "battery_used_Wh": round(float(st["battery_used_Wh"]), 3),
        "battery_pct_remaining": round(float(st["battery_pct"]), 2),
        "battery_Wh_usable": mission["battery_Wh_usable"],
        "n_battery_swaps": n_battery_swaps,
        "total_energy_Wh_all_batteries": round(
            cumulative_energy_Wh + float(st["battery_used_Wh"]), 3),
        "position_hold_worst_error_m": round(worst_err, 4),
        "position_hold_tolerance_m": SETTLE_TOL_M,
        "n_unsettled": n_unsettled,
        "n_stuck": n_stuck,
        "n_skipped": n_skipped,
        "n_skipped_battery": n_skipped_battery,
        "n_skipped_respawn_budget": n_skipped_respawn_budget,
        "n_respawns": n_respawns,
        "battery_reserve_reached": battery_exhausted,
        "log": log,
    }
    # Default mission keeps the original filename (nothing downstream that
    # already reads "flight_log.json" should have to change); any other
    # --mission= gets its own <stem>_flight_log.json so a coverage-mission
    # flight never clobbers Stage 1's own log.
    log_stem = ("flight_log" if mission_name == "mission.json"
               else os.path.splitext(mission_name)[0] + "_flight_log")
    out_path = os.path.join(MISSION_DIR, f"{log_stem}.json")
    with open(out_path, "w") as f:
        json.dump(flight_log, f, indent=2)

    if detect_on:
        det_path = os.path.join(MISSION_DIR, "detections.json")
        with open(det_path, "w") as f:
            json.dump({
                "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                              time.gmtime()),
                "mission_file": mission_name,
                "detector": "detect_stub_final.detect (STUB -- peeks at "
                          "ground truth internally to fake perception, "
                          "not a real model; see that module's own "
                          "docstring)",
                "n_waypoints_detected_at": len(detections),
                "detections": detections,
            }, f, indent=2)
        print(f"  detect  : {sum(len(d['detections']) for d in detections)} "
             f"stub detections logged across {len(detections)} settled "
             f"waypoints -> {det_path}")

    print(f"  flight  : {flight_log['n_waypoints']} waypoints, "
         f"{flight_log['real_distance_m']:.1f} m real vs "
         f"{flight_log['planned_distance_m']:.1f} m planned")
    print(f"  energy  : {flight_log['battery_used_Wh']:.2f} Wh used on "
         f"the final pack ({flight_log['battery_pct_remaining']:.1f}% "
         f"remaining), of {flight_log['battery_Wh_usable']:.0f} Wh usable")
    print(f"  packs   : {n_battery_swaps} battery swap(s), "
         f"{flight_log['total_energy_Wh_all_batteries']:.1f} Wh total "
         "across every pack this mission used")
    print(f"  time    : {flight_log['sim_time_s']:.1f} s sim, "
         f"{flight_log['wall_time_s']:.1f} s wall "
         f"({flight_log['real_time_factor']:.1f}x real time)")
    print(f"  hold    : worst settle error {worst_err:.4f} m "
         f"(tolerance {SETTLE_TOL_M} m)")
    print(f"  unreach : {n_unsettled} / {len(log)} legs never settled "
         f"({n_stuck} genuine stuck/crash events, {n_respawns} respawns)")
    print(f"  skipped : {n_skipped} waypoints not attempted "
         f"({n_skipped_battery} battery reserve reached, "
         f"{n_skipped_respawn_budget} respawn budget exhausted)")
    print(f"  saved   : {out_path}")
    print("FINAL_FLIGHT_COMPLETE")


if __name__ == "__main__":
    main()
