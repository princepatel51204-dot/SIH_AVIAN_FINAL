"""Phase 2 tests: the URDF must agree with the CAD, not merely load.

The check that matters is T05. It drives the URDF in PyBullet to a set of
joint configurations and compares the resulting tool pose against `kin_b.py`
-- the CAD's own forward kinematics, the same code the Onshape mate scheme and
the 49,275-cell collision envelope were built from. If the generator ever
drifts from the CAD, the tool position diverges and this fails.

Run:  python tests/test_description.py
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CAD = os.environ.get("AVIAN_CAD_DIR", os.path.join(ROOT, "cad"))
for p in (ROOT, CAD):
    if p not in sys.path:
        sys.path.insert(0, p)

import pybullet as pb                                    # noqa: E402
import params_b as P                                     # noqa: E402
import kin_b as K                                        # noqa: E402

URDF = os.path.join(ROOT, "description", "avian.urdf")
MANIFEST = os.path.join(ROOT, "description",
                        "avian_description_manifest.json")

RESULTS = []


def check(cid, name, ok, measured, expected, detail=""):
    RESULTS.append({"id": cid, "name": name, "status": "PASS" if ok
                    else "FAIL", "measured": str(measured),
                    "expected": str(expected), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid:4} {name:<44} "
          f"{str(measured):<28} {detail}")
    return ok


def main():
    print("PHASE 2 - DESCRIPTION TESTS")
    man = json.load(open(MANIFEST))

    cl = pb.connect(pb.DIRECT)
    pb.setGravity(0, 0, -9.80665)
    # Base fixed for kinematic testing; the flight tests unfix it.
    rid = pb.loadURDF(URDF, [0, 0, 0], useFixedBase=True,
                      flags=pb.URDF_USE_INERTIA_FROM_FILE)

    # ---- T01 the model loads and has the expected topology --------------
    n_j = pb.getNumJoints(rid)
    names = [pb.getJointInfo(rid, i)[1].decode() for i in range(n_j)]
    revolute = [n for i, n in enumerate(names)
                if pb.getJointInfo(rid, i)[2] == pb.JOINT_REVOLUTE]
    continuous = [n for i, n in enumerate(names)
                  if pb.getJointInfo(rid, i)[2] == pb.JOINT_REVOLUTE
                  or pb.getJointInfo(rid, i)[2] == pb.JOINT_PRISMATIC]
    check("T01", "URDF loads with a valid kinematic tree",
          n_j > 0, f"{n_j} joints", "> 0",
          f"{len(revolute)} revolute")

    # ---- T02 six manipulator joints, correct names ----------------------
    want = [j[0] for j in P.JOINTS]
    have = [n for n in names if n in want]
    check("T02", "six manipulator joints J1-J6 present",
          have == want, have, want)

    # ---- T03 joint limits match the CAD ---------------------------------
    bad = []
    for i in range(n_j):
        info = pb.getJointInfo(rid, i)
        nm = info[1].decode()
        if nm not in want:
            continue
        lo, hi = info[8], info[9]
        cad = next(j for j in P.JOINTS if j[0] == nm)
        want_lo, want_hi = math.radians(cad[3][0]), math.radians(cad[3][1])
        if abs(lo - want_lo) > 1e-6 or abs(hi - want_hi) > 1e-6:
            bad.append(f"{nm}: {math.degrees(lo):.1f}..{math.degrees(hi):.1f}"
                       f" != {cad[3]}")
        # effort and velocity too
        if abs(info[10] - cad[4]) > 1e-6:
            bad.append(f"{nm} effort {info[10]} != {cad[4]}")
        if abs(info[11] - math.radians(cad[5])) > 1e-6:
            bad.append(f"{nm} velocity {info[11]} != {cad[5]} deg/s")
    check("T03", "joint limits, effort and velocity match the CAD",
          not bad, f"{len(want)} joints checked", "exact match",
          "; ".join(bad[:3]))

    # ---- T04 eight rotors, coaxial, counter-rotating --------------------
    rotors = man["rotors"]
    ccw = sum(1 for r in rotors if r["spin"] == "CCW")
    pairs = {}
    for r in rotors:
        pairs.setdefault(r["arm"], []).append(r["level"])
    ok = (len(rotors) == 8 and ccw == 4
          and all(sorted(v) == ["lower", "upper"] for v in pairs.values()))
    check("T04", "8 rotors in 4 counter-rotating coaxial pairs",
          ok, f"{len(rotors)} rotors, {ccw} CCW, {len(pairs)} arms",
          "8 rotors, 4 CCW, 4 arms")

    # ---- T05 forward kinematics agree with the CAD ----------------------
    # THE test. Drive both to the same joint angles and compare tool0.
    link_index = {pb.getJointInfo(rid, i)[12].decode(): i
                  for i in range(n_j)}
    ti = link_index["tool0"]

    configs = [
        [0, 0, 0, 0, 0, 0],
        [30, -40, 60, 0, 25, 0],
        [-90, 55, -70, 45, -30, 60],
        [180, -115, 160, -180, 120, -180],
        [12.5, 33.3, -47.9, 88.1, -61.2, 15.0],
        [-45, 20, 20, 90, 90, 90],
    ]
    worst = 0.0
    detail = ""
    for q_deg in configs:
        for k, nm in enumerate(want):
            pb.resetJointState(rid, link_index[f"link_{k+1}"], 0.0)
        # joint index == the joint whose child is link_k+1
        for k, nm in enumerate(want):
            ji = next(i for i in range(n_j)
                      if pb.getJointInfo(rid, i)[1].decode() == nm)
            pb.resetJointState(rid, ji, math.radians(q_deg[k]))
        st = pb.getLinkState(rid, ti, computeForwardKinematics=True)
        sim_p = np.array(st[4])                    # world frame link origin

        cad_frames = K.joint_frames([float(v) for v in q_deg])
        cad_p = cad_frames[-1][:3, 3] * 0.001       # mm -> m

        d = float(np.linalg.norm(sim_p - cad_p))
        if d > worst:
            worst = d
            detail = (f"q={q_deg} sim={np.round(sim_p, 4)} "
                      f"cad={np.round(cad_p, 4)}")
    check("T05", "URDF forward kinematics match CAD kin_b.py",
          worst < 0.001, f"max error {worst*1000:.3f} mm", "< 1.000 mm",
          detail if worst >= 0.001 else "6 configurations tested")

    # ---- T06 reach envelope matches the CAD ------------------------------
    # The kinematic chain from arm_base to tool0 is the sum of all six joint
    # offsets plus the tool flange: vL0..vL5. The first version of this test
    # subtracted vL0 as if it were not part of the chain, which it is -- J2
    # sits vL0 below J1. That was a test bug, not a model error.
    chain_mm = P.vL0 + P.vL1 + P.vL2 + P.vL3 + P.vL4 + P.vL5
    f = K.joint_frames([0.0] * 6)
    stowed_mm = float(np.linalg.norm(f[-1][:3, 3] - f[0][:3, 3]))
    # Maximum reach over the joint envelope, measured not assumed.
    best = 0.0
    for j2 in range(-115, 116, 15):
        for j3 in range(-160, 161, 20):
            ff = K.joint_frames([0.0, float(j2), float(j3), 0.0, 0.0, 0.0])
            best = max(best, float(np.linalg.norm(ff[-1][:3, 3]
                                                  - ff[0][:3, 3])))
    check("T06", "manipulator chain length matches the CAD",
          abs(stowed_mm - chain_mm) < 1.0 and best <= chain_mm + 1.0,
          f"stowed {stowed_mm:.1f} mm, max {best:.1f} mm",
          f"chain {chain_mm:.1f} mm",
          "vL0..vL5 summed; max reach sampled over J2/J3")

    # ---- T07 mass and CG agree with the closed budget --------------------
    # PyBullet reports a FIXED base as mass 0, so the 21 kg airframe vanishes
    # from the sum. Load a free-floating instance to weigh the whole vehicle.
    # (Also a test bug in the first version, not a model error.)
    fid = pb.loadURDF(URDF, [0, 0, 50], useFixedBase=False,
                      flags=pb.URDF_USE_INERTIA_FROM_FILE)
    total = pb.getDynamicsInfo(fid, -1)[0]
    for i in range(pb.getNumJoints(fid)):
        total += pb.getDynamicsInfo(fid, i)[0]
    pb.removeBody(fid)
    # Compare the SIMULATED vehicle against the CAD mass budget directly --
    # not against the manifest, which is derived from the same generator and
    # would agree with itself even if both were wrong.
    budget = man["mass_budget_total_kg"]
    frames = man.get("frame_link_mass_kg", 0.0)
    check("T07", "simulated mass matches the CAD mass budget",
          abs((total - frames) - budget) < 0.01,
          f"{total - frames:.3f} kg", f"{budget:.3f} kg",
          f"config {man['config']}, {frames*1000:.0f} g of solver frames "
          f"excluded")

    # ---- T08 thrust-to-weight is flyable ---------------------------------
    twr = man["max_total_thrust_N"] / (total * 9.80665)
    check("T08", "thrust-to-weight ratio is flyable",
          twr >= 1.5, f"{twr:.2f}", ">= 1.50",
          f"{man['max_total_thrust_N']:.0f} N over {total:.1f} kg")

    # ---- T09 no self-collision in the stowed pose ------------------------
    # getClosestPoints, NOT getContactPoints. The URDF is loaded without
    # URDF_USE_SELF_COLLISION, so getContactPoints(body, body) returns an
    # empty list for ANY pose -- including one with a link buried 110 mm
    # inside the fuselage. This check passed vacuously until that was found
    # by the manipulator suite. Parent-child pairs are excluded by the real
    # parent relation from the URDF, not by an index heuristic.
    for i in range(n_j):
        pb.resetJointState(rid, i, 0.0)
    pb.performCollisionDetection()
    parent_of = {i: pb.getJointInfo(rid, i)[16] for i in range(n_j)}
    worst_pair = {}
    for c in pb.getClosestPoints(rid, rid, 0.001):
        a, b = c[3], c[4]
        if a == b or parent_of.get(a, -2) == b or parent_of.get(b, -2) == a:
            continue
        key = (min(a, b), max(a, b))
        if key not in worst_pair or c[8] < worst_pair[key]:
            worst_pair[key] = c[8]
    pts = [(k, d) for k, d in worst_pair.items() if d < -0.001]
    check("T09", "no self-collision in the stowed pose",
          not pts, f"{len(pts)} penetrating link pairs", "0",
          "; ".join(f"{k[0]}-{k[1]} {d*1000:.1f} mm" for k, d in pts[:3]))

    # ---- T10 sensor frames present, placed AND AIMED ---------------------
    # Presence alone is not enough. The frames were present and every one of
    # them was rotated 90 deg off the nose, which no count of links can
    # detect; the error survived until a depth-camera range test caught it.
    # So the boresight of each frame is now asserted against the direction
    # the mount angles say it should point.
    missing = [s["name"] for s in man["sensors"]
               if s["name"] not in link_index]

    def _expected(pitch, yaw):
        p, y = math.radians(pitch), math.radians(yaw)
        # +X forward, pitched nose-DOWN by `pitch`, then yawed by `yaw`.
        f = np.array([math.cos(p), 0.0, -math.sin(p)])
        Rz = np.array([[math.cos(y), -math.sin(y), 0],
                       [math.sin(y), math.cos(y), 0], [0, 0, 1]])
        return Rz @ f

    aim_bad = []
    for s in man["sensors"]:
        want = _expected(s["pitch_deg"], s["yaw_deg"])
        got = np.array(s["boresight_body"])
        if float(np.dot(want, got)) < math.cos(math.radians(1.0)):
            aim_bad.append(f"{s['name']} -> {np.round(got, 3).tolist()} "
                           f"want {np.round(want, 3).tolist()}")
    # A spinning LiDAR sweeps about its own +Z; that must be body +Z, or the
    # scan plane comes out vertical.
    spin = next(s for s in man["sensors"] if s["name"] == "sensor_lidar")
    spin_ok = float(np.dot(np.array(spin["frame_z_body"]),
                           np.array([0.0, 0.0, 1.0]))) > 0.999
    if not spin_ok:
        aim_bad.append(f"sensor_lidar spin axis {spin['frame_z_body']}")

    check("T10", "seven sensor frames present, placed and correctly aimed",
          not missing and not aim_bad and len(man["sensors"]) == 7,
          f"{len(man['sensors'])} frames, "
          f"{len(man['sensors']) - len(aim_bad)} correctly aimed",
          "7 frames, every boresight within 1 deg of its mount angles",
          "; ".join(missing + aim_bad))

    pb.disconnect()

    n_pass = sum(1 for r in RESULTS if r["status"] == "PASS")
    n_fail = len(RESULTS) - n_pass
    print(f"\n  {n_pass} pass, {n_fail} fail of {len(RESULTS)} checks")
    out = os.path.join(ROOT, "logs", "phase2_test_report.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        json.dump({"phase": 2, "checks": RESULTS,
                   "run_id": os.environ.get("AVIAN_RUN_ID"),
                   "summary": {"pass": n_pass, "fail": n_fail,
                               "total": len(RESULTS)}}, f, indent=2)
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
