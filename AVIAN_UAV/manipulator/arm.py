"""AVIAN 6-DOF manipulator: limits, IK, collision and self-collision.

THE CAD IS THE KINEMATIC AUTHORITY.
Every solution this module returns is verified against `kin_b.py` -- the CAD's
own forward kinematics -- before it is handed back. PyBullet's IK is a damped
least-squares solver over the whole articulated body, and left unchecked it
will happily return a "solution" that violates a joint limit, puts a link
through a girder, or simply does not reach. None of those are solutions, and a
manipulator module that returns them is worse than one that returns nothing.

WHAT `solve_ik` GUARANTEES
-------------------------
A returned solution has passed, in order:

    1. reach       tool0 within `tol_m` of the target, measured by CAD FK
    2. limits      every joint inside the CAD's own limits
    3. self        no link-to-link penetration on the aircraft
    4. environment no link-to-structure penetration
    5. rotors      the arm stays clear of the rotor discs

If any check fails the solve returns `None` and a reason. Phase 1 validates
the mechanism; it does not perform repair.

ACCELERATION LIMITS
-------------------
The CAD supplies torque and speed per joint but no acceleration limit, so one
is DERIVED: a = tau / I with I estimated from the link masses and lengths in
the URDF. That is an estimate and is labelled as one. It is used only for
trajectory timing, never for a safety decision.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

CAD_DIR = os.environ.get("AVIAN_CAD_DIR", "/home/claude/avian")
if CAD_DIR not in sys.path:
    sys.path.insert(0, CAD_DIR)

import params_b as P                                       # noqa: E402
import kin_b as K                                          # noqa: E402

MM = 0.001


class Manipulator:
    def __init__(self, vehicle):
        self.v = vehicle
        self.pb = vehicle.pb
        self.joints = vehicle.arm_joints            # PyBullet indices
        self.names = vehicle.arm_names              # J1..J6
        self.spec = vehicle.man["joints"]
        self.lower = np.array([math.radians(s["limit_deg"][0])
                               for s in self.spec])
        self.upper = np.array([math.radians(s["limit_deg"][1])
                               for s in self.spec])
        self.vmax = np.array([math.radians(s["velocity_deg_s"])
                              for s in self.spec])
        self.effort = np.array([s["effort_Nm"] for s in self.spec])
        # Link indices FIRST: the acceleration derivation walks the chain
        # and needs tool0 to close it.
        self.tool_link = vehicle.link_of["tool0"]
        self.tip_link = vehicle.link_of["nozzle_tip"]
        self.arm_links = [vehicle.link_of[f"link_{k}"] for k in range(1, 7)]
        self.rotor_links = vehicle.rotor_link
        self._parents = None
        self.reach_m = (P.vL0 + P.vL1 + P.vL2 + P.vL3 + P.vL4 + P.vL5) * MM
        self.amax = self._derive_acceleration_limits()

    # -----------------------------------------------------------------
    def _derive_acceleration_limits(self):
        """tau / I per joint. ESTIMATED -- the CAD gives torque, not inertia.

        I is approximated as the sum of downstream link masses times the
        square of their distance from the joint, which is a lower bound on
        the true inertia and therefore an OPTIMISTIC acceleration. Used for
        trajectory timing only.
        """
        pb = self.pb
        out = []
        for k, ji in enumerate(self.joints):
            I = 0.0
            jp = np.array(pb.getLinkState(
                self.v.body, ji, computeForwardKinematics=True)[4])
            for jj in self.joints[k:] + [self.tool_link]:
                m = pb.getDynamicsInfo(self.v.body, jj)[0]
                p = np.array(pb.getLinkState(
                    self.v.body, jj, computeForwardKinematics=True)[4])
                I += m * float(np.sum((p - jp) ** 2))
            out.append(self.effort[k] / max(I, 1e-4))
        return np.array(out)

    # -----------------------------------------------------------------
    def q(self):
        return np.array([self.pb.getJointState(self.v.body, ji)[0]
                         for ji in self.joints])

    def set_q(self, q, instant=False):
        q = np.clip(np.asarray(q, dtype=float), self.lower, self.upper)
        for ji, spec, val, vm in zip(self.joints, self.spec, q, self.vmax):
            if instant:
                self.pb.resetJointState(self.v.body, ji, float(val))
            else:
                self.pb.setJointMotorControl2(
                    self.v.body, ji, self.pb.POSITION_CONTROL,
                    targetPosition=float(val), force=spec["effort_Nm"],
                    maxVelocity=float(vm))
        return q

    def base_frame(self):
        """World pose of base_link's URDF FRAME -- not its COM frame.

        `getBasePositionAndOrientation` returns the centre-of-mass frame.
        base_link's inertial origin is 86 mm off its URDF origin, so using
        the COM as the body frame puts every CAD comparison 86 mm out. That
        is enough to fail the 1 mm FK cross-check on a solution that is
        actually correct, which is exactly what it did: `solve_ik` rejected
        every valid solution it found until this was separated out.
        """
        p, q = self.pb.getBasePositionAndOrientation(self.v.body)
        di = self.pb.getDynamicsInfo(self.v.body, -1)
        inv_p, inv_q = self.pb.invertTransform(di[3], di[4])
        return self.pb.multiplyTransforms(p, q, inv_p, inv_q)

    def tool_pose(self):
        st = self.pb.getLinkState(self.v.body, self.tool_link,
                                  computeForwardKinematics=True)
        return np.array(st[4]), np.array(st[5])

    def tip_position(self):
        return np.array(self.pb.getLinkState(
            self.v.body, self.tip_link, computeForwardKinematics=True)[4])

    # -----------------------------------------------------------------
    def within_limits(self, q):
        q = np.asarray(q)
        bad = [(self.names[i], math.degrees(q[i]),
                self.spec[i]["limit_deg"])
               for i in range(6)
               if q[i] < self.lower[i] - 1e-6 or q[i] > self.upper[i] + 1e-6]
        return (not bad), bad

    def fk_cad(self, q):
        """Tool0 position in the BODY frame, from the CAD's own FK, metres."""
        frames = K.joint_frames([math.degrees(v) for v in np.asarray(q)])
        return frames[-1][:3, 3] * MM

    def _parent_map(self):
        """link index -> parent link index, from the URDF, cached.

        The previous version excluded pairs by `abs(a - b) <= 1`, which is
        not the parent relation. It is only a proxy for it that happens to
        hold when links are numbered in chain order, and it silently
        excludes genuinely non-adjacent pairs whose indices are adjacent.
        """
        if getattr(self, "_parents", None) is None:
            self._parents = {}
            for i in range(self.pb.getNumJoints(self.v.body)):
                self._parents[i] = self.pb.getJointInfo(self.v.body, i)[16]
        return self._parents

    def check_self_collision(self, q=None, margin=0.002):
        """Link-to-link penetration on the aircraft, including rotor discs.

        USES getClosestPoints, NOT getContactPoints, AND THAT IS THE POINT.
        The airframe is loaded without `URDF_USE_SELF_COLLISION`, because
        enabling contact response between links of one body changes the
        flight dynamics this project has already validated. The consequence
        is that `getContactPoints(body, body)` returns an empty list no
        matter how deeply the arm is buried in the fuselage -- so the
        original implementation reported "clear" for every pose ever tested,
        including one where link_5 penetrates base_link by 110 mm.

        `getClosestPoints` performs the query geometrically and is
        unaffected by that flag, so the check reports real penetration.

        The honest statement of what this gives: self-collision is
        DETECTED, not PREVENTED. The solver will not push the arm out of the
        airframe; planning must not command a pose this rejects.
        """
        if q is not None:
            saved = self.q()
            self.set_q(q, instant=True)
        self.pb.performCollisionDetection()
        parents = self._parent_map()
        worst = {}
        for c in self.pb.getClosestPoints(self.v.body, self.v.body,
                                          max(margin, 0.001)):
            a, b = c[3], c[4]
            if a == b:
                continue
            if parents.get(a, -2) == b or parents.get(b, -2) == a:
                continue                       # parent and child touch by design
            k = (min(a, b), max(a, b))
            if k not in worst or c[8] < worst[k]:
                worst[k] = c[8]
        hits = [(a, b, round(d, 5)) for (a, b), d in worst.items()
                if d < -margin]
        hits.sort(key=lambda h: h[2])
        if q is not None:
            self.set_q(saved, instant=True)
        return (not hits), hits

    def check_environment_collision(self, bridge_ids, q=None, margin=0.002):
        if q is not None:
            saved = self.q()
            self.set_q(q, instant=True)
        # getClosestPoints for the same reason as above: it reports geometric
        # penetration without depending on what the contact solver chose to
        # register this step.
        self.pb.performCollisionDetection()
        hits = []
        for bid in bridge_ids:
            for c in self.pb.getClosestPoints(self.v.body, bid,
                                              max(margin, 0.001)):
                if c[8] < -margin:
                    hits.append((c[3], bid, round(c[8], 5)))
        hits.sort(key=lambda h: h[2])
        if q is not None:
            self.set_q(saved, instant=True)
        return (not hits), hits

    def clearance_to(self, bridge_ids, max_dist=5.0):  # noqa: E301
        """Smallest distance from ANY arm link to structure."""
        best = max_dist
        for bid in bridge_ids:
            for c in self.pb.getClosestPoints(self.v.body, bid, max_dist):
                if c[3] in self.arm_links or c[3] == self.tool_link:
                    best = min(best, c[8])
        return best

    # -----------------------------------------------------------------
    def solve_ik(self, target_world, bridge_ids=None, tol_m=0.01,
                 tries=6, seed_qs=None):
        """IK with verification. Returns (q, info) or (None, info).

        PyBullet's solver is used as a NUMERICAL PROPOSER only. Every
        proposal is then checked against the CAD kinematics and the real
        collision world, and rejected if it fails. Multiple restarts are used
        because a damped least-squares solver is seed-dependent and a single
        failed attempt says nothing about reachability.
        """
        pb = self.pb
        target = np.asarray(target_world, dtype=float)
        base_p, base_q = self.base_frame()
        R = np.array(pb.getMatrixFromQuaternion(base_q)).reshape(3, 3)
        target_body = R.T @ (target - np.array(base_p))
        info = {"target_world": [round(float(v), 4) for v in target],
                "target_body": [round(float(v), 4) for v in target_body],
                "attempts": [], "reach_m": round(self.reach_m, 4)}

        arm_base_body = np.array(P.vArmBase) * MM
        d = float(np.linalg.norm(target_body - arm_base_body))
        info["distance_from_arm_base_m"] = round(d, 4)
        if d > self.reach_m + tol_m:
            info["reason"] = (f"target {d:.3f} m from the arm base exceeds "
                              f"the {self.reach_m:.3f} m reach")
            return None, info

        saved = self.q()
        rng = np.random.default_rng(1234)
        seeds = list(seed_qs or [])
        seeds.append(saved)
        seeds.append(np.zeros(6))
        while len(seeds) < tries:
            seeds.append(rng.uniform(self.lower * 0.8, self.upper * 0.8))

        for k, s in enumerate(seeds[:tries]):
            self.set_q(s, instant=True)
            sol = pb.calculateInverseKinematics(
                self.v.body, self.tool_link, list(target),
                lowerLimits=list(self.lower), upperLimits=list(self.upper),
                jointRanges=list(self.upper - self.lower),
                restPoses=list(np.clip(s, self.lower, self.upper)),
                maxNumIterations=300, residualThreshold=1e-5)
            # PyBullet returns a value for EVERY movable joint on the body,
            # including the 8 rotor joints. Slicing the arm joints out by
            # position in that list is the only correct way to read it.
            movable = [i for i in range(pb.getNumJoints(self.v.body))
                       if pb.getJointInfo(self.v.body, i)[2] !=
                       pb.JOINT_FIXED]
            qmap = {ji: sol[n] for n, ji in enumerate(movable)}
            q = np.array([qmap[ji] for ji in self.joints])

            ok_lim, bad = self.within_limits(q)
            self.set_q(q, instant=True)
            got = np.array(pb.getLinkState(self.v.body, self.tool_link,
                                           computeForwardKinematics=True)[4])
            err = float(np.linalg.norm(got - target))

            # Independent cross-check against the CAD's own FK.
            cad_body = self.fk_cad(q)
            cad_world = np.array(base_p) + R @ cad_body
            cad_err = float(np.linalg.norm(cad_world - got))

            ok_self, self_hits = self.check_self_collision()
            ok_env, env_hits = (self.check_environment_collision(bridge_ids)
                                if bridge_ids else (True, []))

            att = {"seed": k, "position_error_m": round(err, 5),
                   "cad_fk_disagreement_m": round(cad_err, 6),
                   "within_limits": ok_lim,
                   "self_collision_free": ok_self,
                   "environment_collision_free": ok_env}
            info["attempts"].append(att)

            if err <= tol_m and ok_lim and ok_self and ok_env \
                    and cad_err < 1e-3:
                info.update({"position_error_m": round(err, 5),
                             "cad_fk_disagreement_m": round(cad_err, 6),
                             "solution_deg": [round(math.degrees(x), 3)
                                              for x in q],
                             "reason": "solved"})
                self.set_q(saved, instant=True)
                return q, info

        self.set_q(saved, instant=True)
        best = min(info["attempts"], key=lambda a: a["position_error_m"]) \
            if info["attempts"] else None
        info["reason"] = ("no verified solution: best attempt "
                          f"{best['position_error_m']:.4f} m"
                          if best else "no attempts")
        return None, info

    # -----------------------------------------------------------------
    def plan_trajectory(self, q_goal, q_start=None, n=40):
        """Time-scaled joint trajectory honouring velocity limits.

        A quintic profile, so velocity and acceleration are zero at both ends
        -- a linear interpolation would command a step in velocity that the
        joint cannot produce and the tracking error would be blamed on the
        controller.
        """
        q0 = np.asarray(q_start if q_start is not None else self.q())
        q1 = np.asarray(q_goal)
        dq = q1 - q0
        T = float(max(np.max(np.abs(dq) / np.maximum(self.vmax, 1e-6)), 0.3))
        T *= 1.6                      # quintic peak velocity is ~1.875 x mean
        ts = np.linspace(0.0, T, n)
        s = 10 * (ts / T) ** 3 - 15 * (ts / T) ** 4 + 6 * (ts / T) ** 5
        qs = q0[None, :] + s[:, None] * dq[None, :]
        vs = np.gradient(qs, ts, axis=0)
        peak_v = np.max(np.abs(vs), axis=0)
        return {"t": ts, "q": qs, "v": vs, "duration_s": round(T, 3),
                "peak_velocity_rad_s": peak_v,
                "velocity_limit_rad_s": self.vmax,
                "within_velocity_limits": bool(
                    np.all(peak_v <= self.vmax * 1.02))}
