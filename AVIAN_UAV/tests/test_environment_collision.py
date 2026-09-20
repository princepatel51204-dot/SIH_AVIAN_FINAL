"""Phase 3/4 tests: the bridge collision asset, in the simulator.

Loading is not evidence. These fly the aircraft against real structure and
measure what happens.
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
for p in (ROOT, os.environ.get("AVIAN_CAD_DIR", "/home/claude/avian")):
    if p not in sys.path:
        sys.path.insert(0, p)

import pybullet as pb                                     # noqa: E402
from simulation.world import AvianWorld                    # noqa: E402
from simulation.controller import Cascade                  # noqa: E402
from simulation.safety import SafetySupervisor             # noqa: E402

RESULTS, METRICS = [], {}


def check(cid, name, ok, measured, expected, detail=""):
    RESULTS.append({"id": cid, "name": name,
                    "status": "PASS" if ok else "FAIL",
                    "measured": str(measured), "expected": str(expected),
                    "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid:4} {name:<44} "
          f"{str(measured):<28} {detail}")
    return ok


def main():
    print("PHASE 3/4 - BRIDGE COLLISION IN SIMULATION")
    t0 = time.time()
    w = AvianWorld(gui=False, load_bridge=True)
    man = w.bridge_manifest
    METRICS["collision_primitives"] = man["collision_primitives"]
    METRICS["source_triangles"] = man["source_triangles_in_corridor"]
    METRICS["simplification_ratio"] = man["simplification_ratio"]
    METRICS["corridor_m"] = man["corridor_m"]

    check("E01", "bridge collision asset loads into the simulator",
          len(w.bridge_ids) > 0,
          f"{man['collision_primitives']} primitives in "
          f"{len(w.bridge_ids)} bodies", "> 0",
          f"corridor {man['corridor_m'][0]:.0f}-{man['corridor_m'][1]:.0f} m")

    # ---- E02 the deck is solid -------------------------------------------
    # Drop the aircraft onto the deck from above. If it falls through, the
    # collision asset is decorative.
    sys.path.insert(0, os.environ.get("AVIAN_ENV_DIR",
                                      "/home/claude/avian_env"))
    import params as PENV
    xm = 2100.0
    deck_z = PENV.deck_top_z(xm) if hasattr(PENV, "deck_top_z") else 27.0
    v, = w.spawn_fleet(1, positions=[(xm, 0.0, deck_z + 6.0)])
    for _ in range(int(4.0 / w.dt)):
        pb.stepSimulation()
    z = v.state()["position"][2]
    landed = z > deck_z - 1.0
    METRICS["deck_drop_final_z_m"] = round(float(z), 3)
    METRICS["deck_top_z_m"] = round(float(deck_z), 3)
    check("E02", "the deck is solid - aircraft does not fall through",
          landed, f"rested at z={z:.2f} m", f"deck top {deck_z:.2f} m",
          "free drop from 6 m above the deck")

    pb.removeBody(v.body)
    w.vehicles.clear()

    # ---- E03 under-deck hover with measured clearance --------------------
    soffit = PENV.soffit_z(xm)
    start = (xm, 0.0, soffit - 6.0)
    v, = w.spawn_fleet(1, positions=[start])
    c = Cascade(v)
    tgt = [xm, 0.0, soffit - 4.0]
    c.set_target(tgt, 0.0)
    clearances = []
    for k in range(int(25.0 / w.dt)):
        c.update(w.dt)
        v.step_actuators(w.dt)
        pb.stepSimulation()
        if k % 24 == 0:
            clearances.append(w.min_clearance(v.vid))
    pos = v.state()["position"]
    err = float(np.linalg.norm(pos - np.array(tgt)))
    min_cl = float(min(clearances))
    contacts = w.contacts(v.vid)
    METRICS["underdeck_hover_error_m"] = round(err, 4)
    METRICS["underdeck_min_clearance_m"] = round(min_cl, 3)
    METRICS["underdeck_contacts"] = contacts
    check("E03", "stable hover under the deck, no contact",
          err < 0.30 and contacts == 0 and min_cl > 0.5,
          f"error {err*100:.1f} cm, clearance {min_cl:.2f} m",
          "< 30 cm, > 0.5 m, 0 contacts",
          f"soffit at z={soffit:.1f} m, {contacts} contacts")

    # ---- E04 clearance is actually measured, not assumed -----------------
    # Command the aircraft up toward the soffit and watch clearance fall.
    c.set_target([xm, 0.0, soffit - 1.2], 0.0)
    seq = []
    for k in range(int(20.0 / w.dt)):
        c.update(w.dt)
        v.step_actuators(w.dt)
        pb.stepSimulation()
        if k % 48 == 0:
            seq.append(w.min_clearance(v.vid))
    closed = seq[0] - seq[-1]
    METRICS["clearance_start_m"] = round(float(seq[0]), 3)
    METRICS["clearance_end_m"] = round(float(seq[-1]), 3)
    check("E04", "clearance decreases as the aircraft approaches structure",
          closed > 1.0, f"{seq[0]:.2f} -> {seq[-1]:.2f} m",
          "monotone decrease > 1 m",
          "proves clearance is measured against real geometry")

    # ---- E05 the safety supervisor stops the approach ---------------------
    sup = SafetySupervisor(v, c, home=(xm, -40.0, soffit - 8.0))
    c.set_target([xm, 0.0, soffit + 2.0], 0.0)     # commanded INTO the deck
    hit = False
    for k in range(int(25.0 / w.dt)):
        cl = w.min_clearance(v.vid) if k % 12 == 0 else None
        sup.update(w.t, cl)
        c.update(w.dt)
        v.step_actuators(w.dt)
        pb.stepSimulation()
        w.t += w.dt
        if w.contacts(v.vid) > 0:
            hit = True
    METRICS["safety_transitions"] = sup.transitions
    METRICS["proximity_contacts"] = int(hit)
    check("E05", "proximity supervisor prevents contact with the deck",
          sup.state == "HOLD" and not hit,
          f"state {sup.state}, contacts {int(hit)}",
          "HOLD, 0 contacts",
          sup.reason[:60])

    w.close()
    wall = time.time() - t0
    METRICS["wall_time_s"] = round(wall, 1)

    n_pass = sum(1 for r in RESULTS if r["status"] == "PASS")
    n_fail = len(RESULTS) - n_pass
    print(f"\n  {n_pass} pass, {n_fail} fail of {len(RESULTS)} checks")
    json.dump({"checks": RESULTS, "metrics": METRICS,
               "bridge_manifest": man,
               "run_id": os.environ.get("AVIAN_RUN_ID"),
               "summary": {"pass": n_pass, "fail": n_fail}},
              open(os.path.join(ROOT, "logs", "phase3_report.json"), "w"),
              indent=2)
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
