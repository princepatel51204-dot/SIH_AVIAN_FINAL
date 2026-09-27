"""Phase 1 demo video -- track generator.

Drives the REAL, already-verified Phase 1 stack (AvianVehicle, Cascade,
Manipulator) through one flight + one repair sequence and records every frame
needed to render it later: base pose, all 17 mesh-bearing link world poses,
the 6 arm joint angles, and 8 illustrative rotor angles.

This script adds NO new physics, control logic or collision checking. It only
calls solve_ik(), plan_trajectory(), within_limits(), check_self_collision()
and check_environment_collision() -- the methods Phase 1 already verifies --
and records what they returned. The synthetic repair target is a plain
PyBullet box, not a modelled defect (see README.md in this directory).

Usage:
    AVIAN_CAD_DIR=$PWD/../cad ../../.venv/bin/python3 generate_track.py
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
os.environ.setdefault("AVIAN_CAD_DIR", os.path.join(ROOT, "cad"))
for p in (ROOT, os.environ["AVIAN_CAD_DIR"]):
    if p not in sys.path:
        sys.path.insert(0, p)

import pybullet as pb                                    # noqa: E402
from simulation.world import AvianWorld                   # noqa: E402
from simulation.controller import Cascade                 # noqa: E402

FPS = 30
ROTOR_SPIN_RATE = 25.0     # rad/s, ILLUSTRATIVE -- see README.md

MESH_LINKS = [
    "base_link", "arm_base_link",
    "rotor_1_upper", "rotor_1_lower", "rotor_2_upper", "rotor_2_lower",
    "rotor_3_upper", "rotor_3_lower", "rotor_4_upper", "rotor_4_lower",
    "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "tool0",
]

PANEL_HALF_EXTENTS = [0.15, 0.15, 0.025]     # 0.3 x 0.3 x 0.05 m
PRECONTACT_STANDOFF_M = 0.15
CONTACT_STANDOFF_M = 0.08

# Offset of the panel centre from the arm base, in the arm base frame.
# CHOSEN, NOT DERIVED: the arm's own zero pose ("stowed", verified
# self-collision-free by tests/test_manipulator.py) happens to hang the tool
# almost fully extended straight down from the arm base. A panel placed on
# that line makes every joint-space interpolation toward it sweep back
# through the panel before the elbow re-bends into its final shape. This
# offset, and the via-point below, were chosen by sampling candidates and
# keeping the first one whose every leg -- home->via, via->pre-contact,
# pre-contact->contact and their reverses -- verified collision-free at every
# sampled keyframe (see media/README.md).
PANEL_OFFSET_FROM_ARM_BASE = (0.05, 0.45, -0.9)
VIA_OFFSET_FROM_ARM_BASE = (0.3, 0.3, -0.6)


def link_world_pose(v, name):
    if name == "base_link":
        p, q = v.arm.base_frame()
        return np.asarray(p), np.asarray(q)
    st = v.pb.getLinkState(v.body, v.link_of[name], computeForwardKinematics=True)
    return np.asarray(st[4]), np.asarray(st[5])


def snapshot_links(v):
    return {name: {"position": [round(float(x), 6) for x in p],
                   "quaternion": [round(float(x), 6) for x in q]}
            for name, (p, q) in ((n, link_world_pose(v, n)) for n in MESH_LINKS)}


class Recorder:
    def __init__(self, v, ctrl, world):
        self.v = v
        self.ctrl = ctrl
        self.world = world
        self.frames = []
        self.rotor_angle = np.zeros(v.n_rotors)
        self.t = 0.0

    def _advance_physics(self, seconds):
        n = max(1, int(round(seconds / self.world.dt)))
        for _ in range(n):
            self.ctrl.update(self.world.dt)
            self.v.step_actuators(self.world.dt)
            pb.stepSimulation()
            self.world.t += self.world.dt

    def record_frame(self, phase, caption="", arm_check=None):
        v = self.v
        dt_frame = 1.0 / FPS
        self.rotor_angle += v.rotor_sign * ROTOR_SPIN_RATE * dt_frame
        st = v.state()
        self.frames.append({
            "frame": len(self.frames),
            "t": round(self.t, 5),
            "phase": phase,
            "caption": caption,
            "links": snapshot_links(v),
            "base_com_position": [round(float(x), 6) for x in
                                  pb.getBasePositionAndOrientation(v.body)[0]],
            "base_com_quaternion": [round(float(x), 6) for x in
                                    pb.getBasePositionAndOrientation(v.body)[1]],
            "arm_joint_rad": [round(float(x), 6) for x in v.arm.q()],
            "rotor_angle_rad": [round(float(x), 6) for x in self.rotor_angle],
            "battery_pct": round(st["battery_pct"], 3),
            "arm_check": arm_check,
        })
        self.t += dt_frame

    def fly_segment(self, phase, seconds, caption=""):
        """Physics-only segment: no arm motion, body flies under Cascade."""
        n_frames = max(1, int(round(seconds * FPS)))
        substeps = max(1, int(round((1.0 / FPS) / self.world.dt)))
        for _ in range(n_frames):
            self._advance_physics(substeps * self.world.dt)
            self.record_frame(phase, caption)

    def arm_segment(self, phase, q_goal, bridge_ids, caption=""):
        """Arm motion, verified at every ACTUALLY REALISED keyframe.

        The commanded quintic profile is sent to the real POSITION_CONTROL
        servo (set_q(..., instant=False)) and the body keeps flying under
        Cascade for the physics substeps in between -- never teleported.
        What is verified and rendered is the joint state the servo actually
        reached each frame, not the idealised target: an instant reset here
        would silently fight the joint's still-live position-control target
        every substep, and the resulting reaction torques visibly knocked the
        aircraft off its hover during testing.
        """
        arm = self.v.arm
        q_start = arm.q()
        plan0 = arm.plan_trajectory(q_goal, q_start=q_start, n=40)
        duration = plan0["duration_s"]
        n_frames = max(2, int(round(duration * FPS)))
        plan = arm.plan_trajectory(q_goal, q_start=q_start, n=n_frames)
        substeps = max(1, int(round((1.0 / FPS) / self.world.dt)))

        results = []
        for q_cmd in plan["q"]:
            arm.set_q(q_cmd, instant=False)
            self._advance_physics(substeps * self.world.dt)
            q_actual = arm.q()
            ok_lim, bad_lim = arm.within_limits(q_actual)
            ok_self, hits_self = arm.check_self_collision(q_actual)
            ok_env, hits_env = arm.check_environment_collision(bridge_ids,
                                                                q_actual)
            chk = {"within_limits": ok_lim,
                  "self_collision_free": ok_self,
                  "environment_collision_free": ok_env,
                  "bad_limits": bad_lim,
                  "self_collision_hits": hits_self,
                  "environment_collision_hits": hits_env}
            results.append(chk)
            self.record_frame(phase, caption, arm_check=chk)

        # Servo the final target a little longer so the segment ends AT the
        # goal rather than wherever tracking lag left it.
        arm.set_q(q_goal, instant=False)
        for _ in range(int(round(0.3 * FPS))):
            self._advance_physics(substeps * self.world.dt)
        q_actual = arm.q()
        ok_lim, bad_lim = arm.within_limits(q_actual)
        ok_self, hits_self = arm.check_self_collision(q_actual)
        ok_env, hits_env = arm.check_environment_collision(bridge_ids,
                                                            q_actual)
        chk = {"within_limits": ok_lim, "self_collision_free": ok_self,
              "environment_collision_free": ok_env, "bad_limits": bad_lim,
              "self_collision_hits": hits_self,
              "environment_collision_hits": hits_env}
        results.append(chk)
        self.record_frame(phase, "", arm_check=chk)

        return {"duration_s": duration,
                "within_velocity_limits": plan["within_velocity_limits"],
                "n_keyframes": len(results),
                "all_within_limits": all(r["within_limits"] for r in results),
                "all_self_collision_free": all(r["self_collision_free"]
                                               for r in results),
                "all_environment_collision_free": all(
                    r["environment_collision_free"] for r in results)}


def build_panel(pb_mod, center, half_extents):
    cs = pb_mod.createCollisionShape(pb_mod.GEOM_BOX, halfExtents=half_extents)
    vs = pb_mod.createVisualShape(pb_mod.GEOM_BOX, halfExtents=half_extents,
                                  rgbaColor=[0.55, 0.56, 0.58, 1.0])
    return pb_mod.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=cs,
                                  baseVisualShapeIndex=vs, basePosition=center)


def main():
    out_path = os.path.join(HERE, "avian_demo_track.json")

    world = AvianWorld(gui=False, load_bridge=False)
    v, = world.spawn_fleet(1, positions=[(0.0, 0.0, 1.0)])
    ctrl = Cascade(v)
    v.attach_manipulator()
    v.armed = True

    rec = Recorder(v, ctrl, world)

    log = {"fps": FPS, "rotor_spin_rate_rad_s": ROTOR_SPIN_RATE,
          "segments": [], "solves": {}}

    # ---- 1. climb to hover ------------------------------------------------
    ctrl.set_target([0.0, 0.0, 3.0], 0.0)
    rec.fly_segment("climb", 3.0, "AVIAN flight demo -- Phase 1 (verified)")
    log["segments"].append({"name": "climb", "seconds": 3.0})

    # ---- 2. cruise to standoff position near the repair target ------------
    ctrl.reset_integrators()
    ctrl.set_target([2.0, 0.0, 3.0], 0.0)
    rec.fly_segment("cruise_out", 8.0)
    log["segments"].append({"name": "cruise_out", "seconds": 8.0})

    # ---- 3. brief hold, then place the synthetic repair target -----------
    rec.fly_segment("hold", 0.5, "repair sequence begins")
    log["segments"].append({"name": "hold", "seconds": 0.5})

    arm_base_p, arm_base_q = v.arm.base_frame()
    arm_base_p = np.asarray(arm_base_p)
    panel_center = tuple(float(x) for x in
                        (arm_base_p + np.array(PANEL_OFFSET_FROM_ARM_BASE)))
    panel_id = build_panel(pb, panel_center, PANEL_HALF_EXTENTS)
    surface_z = panel_center[2] + PANEL_HALF_EXTENTS[2]
    precontact_target = np.array([panel_center[0], panel_center[1],
                                  surface_z + PRECONTACT_STANDOFF_M])
    contact_target = np.array([panel_center[0], panel_center[1],
                               surface_z + CONTACT_STANDOFF_M])
    via_target = arm_base_p + np.array(VIA_OFFSET_FROM_ARM_BASE)
    log["panel"] = {"center_m": list(panel_center),
                    "half_extents_m": PANEL_HALF_EXTENTS,
                    "label": "synthetic repair target -- not a modelled defect"}

    bridge_ids = [panel_id]

    q_via, info_via = v.arm.solve_ik(via_target, bridge_ids=bridge_ids)
    if q_via is None:
        raise RuntimeError(f"via-point IK failed to verify: {info_via}")
    log["solves"]["via"] = info_via

    q_pre, info_pre = v.arm.solve_ik(precontact_target, bridge_ids=bridge_ids,
                                     seed_qs=[q_via])
    if q_pre is None:
        raise RuntimeError(f"pre-contact IK failed to verify: {info_pre}")
    log["solves"]["pre_contact"] = info_pre

    q_contact, info_contact = v.arm.solve_ik(contact_target,
                                             bridge_ids=bridge_ids,
                                             seed_qs=[q_pre])
    if q_contact is None:
        raise RuntimeError(f"contact IK failed to verify: {info_contact}")
    log["solves"]["contact"] = info_contact

    q_home = np.zeros(6)

    # ---- 4. arm deploy: home -> via -> pre-contact -------------------------
    # Straight to pre-contact sweeps the stowed pose's near-full-extension
    # line through the panel (see PANEL_OFFSET_FROM_ARM_BASE note above), so
    # the deploy is staged through a verified clear via-point instead.
    r = rec.arm_segment("arm_deploy_via", q_via, bridge_ids,
                        "6-DOF arm: verified solve_ik + plan_trajectory")
    log["segments"].append({"name": "arm_deploy_via", **r})
    r = rec.arm_segment("arm_deploy_pre", q_pre, bridge_ids)
    log["segments"].append({"name": "arm_deploy_pre", **r})

    # ---- 5. approach: pre-contact -> contact -------------------------------
    r = rec.arm_segment("arm_contact", q_contact, bridge_ids,
                        "tool approaches the target")
    log["segments"].append({"name": "arm_contact", **r})

    # ---- 6. repair-applied cue (hold in contact pose) ----------------------
    rec.fly_segment("repair_cue", 4.0,
                    "ILLUSTRATIVE -- repair-applied cue, not simulated")
    log["segments"].append({"name": "repair_cue", "seconds": 4.0})

    # ---- 7. retract: contact -> pre-contact -> via -> home -----------------
    r = rec.arm_segment("arm_retract_1", q_pre, bridge_ids, "retracting")
    log["segments"].append({"name": "arm_retract_1", **r})
    r = rec.arm_segment("arm_retract_via", q_via, bridge_ids)
    log["segments"].append({"name": "arm_retract_via", **r})
    r = rec.arm_segment("arm_retract_home", q_home, bridge_ids, "arm stowed")
    log["segments"].append({"name": "arm_retract_home", **r})

    # ---- 8. cruise back / closing shot -------------------------------------
    ctrl.reset_integrators()
    ctrl.set_target([0.0, 0.0, 3.5], 0.0)
    rec.fly_segment("cruise_back", 7.0)
    log["segments"].append({"name": "cruise_back", "seconds": 7.0})

    rec.fly_segment("hold_end", 4.0, "Phase 1 -- 46/46 acceptance checks")
    log["segments"].append({"name": "hold_end", "seconds": 4.0})

    total_s = rec.t
    log["total_seconds"] = round(total_s, 3)
    log["n_frames"] = len(rec.frames)
    log["frames"] = rec.frames
    log["reach_m"] = v.arm.reach_m

    all_ok = all(seg.get("all_within_limits", True) and
                seg.get("all_self_collision_free", True) and
                seg.get("all_environment_collision_free", True)
                for seg in log["segments"])
    log["all_arm_keyframes_verified"] = all_ok

    with open(out_path, "w") as f:
        json.dump(log, f)

    print(f"wrote {out_path}")
    print(f"  total duration: {total_s:.2f} s  ({len(rec.frames)} frames @ "
         f"{FPS} fps)")
    print(f"  panel center: {panel_center}")
    print(f"  pre-contact solved: seed {info_pre.get('reason')}, "
         f"pos_err {info_pre.get('position_error_m')} m")
    print(f"  contact solved:     seed {info_contact.get('reason')}, "
         f"pos_err {info_contact.get('position_error_m')} m")
    print(f"  all arm keyframes verified: {all_ok}")

    world.close()
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
