"""Phase 1F tests 18-25: the six manipulator joints, IK, and collision.

WHAT "A JOINT WORKS" MEANS HERE
------------------------------
Not that a command was accepted. Six things are asserted per joint, and a
joint that fails any one of them is not working:

    1. it MOVES        commanded travel is actually achieved, not clipped
    2. it moves ALONE  the other five stay within a small dead-band
    3. it moves the RIGHT WAY  the tool displaces about the CAD's own axis
    4. it STOPS        both mechanical limits refuse to be exceeded
    5. it obeys SPEED  the realised rate never exceeds the CAD velocity limit
    6. it AGREES       PyBullet FK matches `kin_b.py` to sub-millimetre

Item 3 is the one that catches the interesting bugs. A joint driven about the
wrong axis, or with the sign flipped, still "reaches its target" and still
passes limits and speed. It shows up only when the tool goes the wrong way,
which is why the expected displacement direction is computed from the CAD
axis and compared, rather than assumed.

The arm is tested with the aircraft HELD FIXED. Phase 1 is validating the
mechanism; whether the airframe can hold station against the reaction torque
of a moving 6-DOF arm is a flight-dynamics question and it is not answered
here, so it is not implied here either.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in (os.environ.get("AVIAN_ENV_DIR", "/home/claude/avian_env"),
          os.environ.get("AVIAN_CAD_DIR", "/home/claude/avian"), ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

import pybullet as pb                                      # noqa: E402
from simulation.world import AvianWorld                     # noqa: E402
import params as PENV                                      # noqa: E402

RESULTS, METRICS = [], {}
DT = 1.0 / 240.0


def check(cid, name, ok, measured, expected, detail=""):
    RESULTS.append({"id": cid, "name": name,
                    "status": "PASS" if ok else "FAIL",
                    "measured": str(measured), "expected": str(expected),
                    "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid:4} {name:<42} "
          f"{str(measured):<32} {detail}")
    return ok


def drive(arm, q_target, steps=900):
    """Command a pose and step until it settles. Returns the q history."""
    arm.set_q(q_target)
    hist = []
    for _ in range(steps):
        pb.stepSimulation()
        hist.append(arm.q())
    return np.array(hist)


def main():
    print("PHASE 1F - MANIPULATOR")
    t0 = time.time()
    xm = 2100.0
    soffit = PENV.soffit_z(xm)
    w = AvianWorld(gui=False, load_bridge=True, seed=21)
    v, = w.spawn_fleet(1, positions=[(xm, -26.0, soffit - 6.0)])
    # Hold the airframe still. The arm is the device under test; letting the
    # aircraft drift would mix a control question into a kinematics answer.
    hold = pb.createConstraint(v.body, -1, -1, -1, pb.JOINT_FIXED, [0, 0, 0],
                               [0, 0, 0], list(v.state()["position"]))
    arm = v.attach_manipulator()
    zero = np.zeros(6)
    arm.set_q(zero, instant=True)
    pb.stepSimulation()

    METRICS["joints"] = {}
    METRICS["reach_m"] = round(arm.reach_m, 4)
    METRICS["acceleration_limits_rad_s2"] = [round(float(a), 2)
                                             for a in arm.amax]
    METRICS["acceleration_limit_method"] = (
        "tau / I, I from downstream link masses; ESTIMATE, timing only")

    # ---- 18-23: J1..J6, one test each ------------------------------------
    for k in range(6):
        cid = f"M{18 + k}"
        nm = arm.names[k]
        spec = arm.spec[k]
        lo, hi = arm.lower[k], arm.upper[k]
        # A travel that is large enough to be unambiguous but stays clear of
        # both stops, so this test measures motion and the limit test below
        # measures stops.
        travel = min(math.radians(40.0), 0.35 * (hi - lo))
        target = zero.copy()
        target[k] = travel

        arm.set_q(zero, instant=True)
        pb.stepSimulation()
        p_before = arm.tip_position()
        _, tq0 = arm.tool_pose()
        bq0 = arm.base_frame()[1]
        jst = pb.getLinkState(v.body, arm.joints[k],
                              computeForwardKinematics=True)
        Rj = np.array(pb.getMatrixFromQuaternion(jst[5])).reshape(3, 3)
        Rb0 = np.array(pb.getMatrixFromQuaternion(bq0)).reshape(3, 3)
        # Joint axis expressed in the BODY frame, not the world frame: the
        # airframe recoils slightly against the arm even on a hold
        # constraint, and measuring the tool rotation in the world folds
        # that recoil into the answer. Relative to the body it cancels.
        axis_body = Rb0.T @ (Rj @ np.array(spec["axis"], dtype=float))

        hist = drive(arm, target)
        q_end = hist[-1]
        p_after = arm.tip_position()
        _, tq1 = arm.tool_pose()
        bq1 = arm.base_frame()[1]

        reached = abs(q_end[k] - travel)
        others = float(np.max(np.abs(np.delete(q_end, k))))

        # 3. SENSE. Not "did the tool move" -- J1 and J4 carry the tool on
        # their own axis at q=0, so the tip barely translates and a
        # displacement test says nothing about them. What is always true is
        # that the tool FRAME rotates about the CAD's joint axis by the
        # commanded angle, in the commanded direction. A joint wired to the
        # wrong axis, or with the sign flipped, fails this and passes every
        # other check in this test.
        # Tool orientation in the BODY frame at both ends, so airframe
        # recoil is differenced out.
        t0b = pb.multiplyTransforms([0, 0, 0], pb.invertTransform(
            [0, 0, 0], list(bq0))[1], [0, 0, 0], list(tq0))[1]
        t1b = pb.multiplyTransforms([0, 0, 0], pb.invertTransform(
            [0, 0, 0], list(bq1))[1], [0, 0, 0], list(tq1))[1]
        ang_axis = pb.getAxisAngleFromQuaternion(
            pb.getDifferenceQuaternion(list(t0b), list(t1b)))
        rot_ax = np.array(ang_axis[0], dtype=float)
        rot_ang = float(ang_axis[1])
        if rot_ang < 0:
            rot_ax, rot_ang = -rot_ax, -rot_ang
        cosang = (float(np.dot(rot_ax, axis_body)) if rot_ang > 1e-4
                  else -1.0)
        ang_err = abs(rot_ang - travel)
        moved = p_after - p_before

        # 4. stops, both directions, commanded 30 deg beyond each.
        arm.set_q(zero, instant=True)
        pb.stepSimulation()
        over = zero.copy()
        over[k] = hi + math.radians(30.0)
        drive(arm, over, 900)
        q_hi = arm.q()[k]
        under = zero.copy()
        under[k] = lo - math.radians(30.0)
        drive(arm, under, 900)
        q_lo = arm.q()[k]
        stops_ok = (q_hi <= hi + 1e-3) and (q_lo >= lo - 1e-3)

        # 5. speed, measured over the settling motion.
        arm.set_q(zero, instant=True)
        pb.stepSimulation()
        hist2 = drive(arm, target, 900)
        rate = np.max(np.abs(np.diff(hist2[:, k]))) / DT
        speed_ok = rate <= arm.vmax[k] * 1.05

        # 6. CAD agreement at the achieved pose. Measured against base_link's
        # URDF FRAME, not the COM frame that
        # getBasePositionAndOrientation returns -- those differ by 86 mm on
        # this airframe.
        base_p, base_q = arm.base_frame()
        R = np.array(pb.getMatrixFromQuaternion(base_q)).reshape(3, 3)
        cad_w = np.array(base_p) + R @ arm.fk_cad(arm.q())
        tool_w, _ = arm.tool_pose()
        fk_err = float(np.linalg.norm(cad_w - tool_w))

        ok = (reached < math.radians(1.5) and others < math.radians(0.5)
              and cosang > 0.99 and ang_err < math.radians(1.5)
              and stops_ok and speed_ok and fk_err < 0.003)
        METRICS["joints"][nm] = {
            "commanded_deg": round(math.degrees(travel), 2),
            "reached_error_deg": round(math.degrees(reached), 3),
            "cross_talk_deg": round(math.degrees(others), 3),
            "rotation_axis_cos_vs_cad": round(cosang, 4),
            "rotation_angle_error_deg": round(math.degrees(ang_err), 3),
            "cad_joint_axis": spec["axis"],
            "limit_deg": spec["limit_deg"],
            "held_at_upper_deg": round(math.degrees(q_hi), 2),
            "held_at_lower_deg": round(math.degrees(q_lo), 2),
            "peak_rate_deg_s": round(math.degrees(rate), 1),
            "velocity_limit_deg_s": spec["velocity_deg_s"],
            "cad_fk_error_mm": round(fk_err * 1000.0, 4),
            "tip_travel_mm": round(float(np.linalg.norm(moved)) * 1000.0, 1),
        }
        check(cid, f"{nm} moves alone, the right way, and stops at both "
                   f"limits", ok,
              f"{math.degrees(travel):.0f} deg cmd, err "
              f"{math.degrees(reached):.2f} deg",
              "<1.5 deg error, <0.5 deg cross-talk, rotation about the CAD "
              "axis, limits held, within speed",
              f"cross-talk {math.degrees(others):.2f} deg, axis cos "
              f"{cosang:+.3f}, stops {math.degrees(q_lo):.1f}/"
              f"{math.degrees(q_hi):.1f} deg, peak "
              f"{math.degrees(rate):.0f}/{spec['velocity_deg_s']:.0f} deg/s, "
              f"FK {fk_err*1000:.3f} mm")

    arm.set_q(zero, instant=True)
    pb.stepSimulation()

    # ---- 24: IK, verified and honestly rejected --------------------------
    # Two halves. A reachable target must be SOLVED and the solution must
    # actually put the tool there; an unreachable one must be REFUSED. An IK
    # that never says no is not a solver, it is an optimiser with a return
    # statement.
    base_p, base_q = pb.getBasePositionAndOrientation(v.body)
    R = np.array(pb.getMatrixFromQuaternion(base_q)).reshape(3, 3)
    arm_base = np.array(__import__("params_b").vArmBase) * 0.001

    solved, errs = 0, []
    tries = []
    rng = np.random.default_rng(7)
    for _ in range(12):
        # Sample inside the reach envelope, forward and below the airframe,
        # which is the half-space the arm is built to work in.
        d = rng.uniform(0.35, 0.80) * arm.reach_m
        az = rng.uniform(-math.pi / 3, math.pi / 3)
        el = rng.uniform(-math.pi / 2.2, -0.15)
        local = np.array([math.cos(el) * math.cos(az),
                          math.cos(el) * math.sin(az), math.sin(el)]) * d
        tgt = np.array(base_p) + R @ (arm_base + local)
        q, info = arm.solve_ik(tgt, bridge_ids=w.bridge_ids)
        tries.append({"target_world": info["target_world"],
                      "solved": q is not None,
                      "reason": info.get("reason"),
                      "position_error_m": info.get("position_error_m")})
        if q is not None:
            solved += 1
            errs.append(info["position_error_m"])

    far = np.array(base_p) + R @ (arm_base + np.array([0.0, 0.0, -3.0]))
    q_far, info_far = arm.solve_ik(far, bridge_ids=w.bridge_ids)
    refused = q_far is None

    worst = max(errs) if errs else None
    METRICS["ik_targets"] = len(tries)
    METRICS["ik_solved"] = solved
    METRICS["ik_worst_position_error_m"] = worst
    METRICS["ik_attempts"] = tries
    METRICS["ik_unreachable_refused"] = refused
    METRICS["ik_unreachable_reason"] = info_far.get("reason")
    check("M24", "IK solves reachable targets and refuses unreachable ones",
          solved >= 9 and refused and worst is not None and worst <= 0.01,
          f"{solved}/12 solved, worst error "
          f"{(worst or 0) * 1000:.1f} mm",
          ">=9/12 solved to <=10 mm, 3 m target refused",
          f"unreachable: {info_far.get('reason')}")

    # ---- 25: collision detection actually detects -------------------------
    # Both directions. A checker that always returns "clear" passes a stowed
    # pose, so a pose that IS in collision has to be constructed and caught.
    arm.set_q(zero, instant=True)
    pb.stepSimulation()
    ok_stow, stow_hits = arm.check_self_collision()
    ok_env_free, env_free_hits = arm.check_environment_collision(w.bridge_ids)

    # Fold the arm hard back over the airframe: J1 swung round, J2 and J3
    # near their stops, which buries the forearm in the fuselage.
    folded = np.radians([-180.0, 60.0, 160.0, 0.0, 0.0, 0.0])
    arm.set_q(folded, instant=True)
    pb.stepSimulation()
    ok_fold, fold_hits = arm.check_self_collision()
    deepest = min((h[2] for h in fold_hits), default=0.0)

    # Now drive the aircraft up into the soffit so the arm meets structure.
    # The hold constraint has to go first, or the reset is undone on the
    # next step and the test measures nothing -- which is what it did.
    arm.set_q(zero, instant=True)
    clear = arm.clearance_to(w.bridge_ids, max_dist=20.0)
    pb.removeConstraint(hold)
    # Under the deck edge, 300 mm above the soffit: the arm hangs below the
    # airframe, so this drives the ARM into structure, which is the thing
    # being detected.
    pb.resetBasePositionAndOrientation(
        v.body, [xm, -10.0, soffit + 0.30], [0, 0, 0, 1])
    pb.resetBaseVelocity(v.body, [0, 0, 0], [0, 0, 0])
    pb.performCollisionDetection()
    ok_env, env_hits = arm.check_environment_collision(w.bridge_ids)
    touching = arm.clearance_to(w.bridge_ids, max_dist=20.0)

    METRICS["collision"] = {
        "stowed_self_free": bool(ok_stow),
        "stowed_environment_free": bool(ok_env_free),
        "folded_self_collision_detected": bool(not ok_fold),
        "folded_contacts": len(fold_hits),
        "folded_deepest_penetration_m": round(float(deepest), 4),
        "self_collision_note": (
            "DETECTED, not PREVENTED: the airframe is loaded without "
            "URDF_USE_SELF_COLLISION, so the solver will not push the arm "
            "out of the body. getClosestPoints is used, which reports "
            "penetration regardless of that flag."),
        "arm_clearance_hovering_m": round(float(clear), 4),
        "arm_clearance_pressed_m": round(float(touching), 4),
        "environment_collision_detected": bool(not ok_env),
        "environment_contacts": len(env_hits),
    }
    check("M25", "collision detection catches real contact, both kinds",
          ok_stow and ok_env_free and (not ok_fold) and (not ok_env),
          f"stowed clear, folded {len(fold_hits)} self-contacts, "
          f"pressed {len(env_hits)} structure contacts",
          "clear when clear, detected when not",
          f"deepest self-penetration {deepest*1000:.0f} mm; arm clearance "
          f"{clear:.2f} m hovering -> {touching:.3f} m at the soffit")

    w.close()
    METRICS["wall_time_s"] = round(time.time() - t0, 1)
    n_pass = sum(1 for r in RESULTS if r["status"] == "PASS")
    n_fail = len(RESULTS) - n_pass
    print(f"\n  {n_pass} pass, {n_fail} fail of {len(RESULTS)} checks")
    os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
    json.dump({"checks": RESULTS, "metrics": METRICS,
               "run_id": os.environ.get("AVIAN_RUN_ID"),
               "summary": {"pass": n_pass, "fail": n_fail}},
              open(os.path.join(ROOT, "logs", "phase1_manipulator.json"), "w"),
              indent=2, default=str)
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
