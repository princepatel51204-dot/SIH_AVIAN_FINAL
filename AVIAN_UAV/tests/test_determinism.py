"""Test 6 (determinism) and Test 7 (emergency behaviour).

Determinism is not decoration. Every metric in this project is a single run,
and a single run of a non-deterministic simulator is an anecdote. This
measures the divergence between two identical runs and reports it rather than
asserting bit-exactness.
"""
from __future__ import annotations

import json
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


def run_mission(seed=7, seconds=30.0):
    """One fixed mission: same seed, dt, pose, gains, environment."""
    w = AvianWorld(gui=False, seed=seed, load_bridge=False)
    v, = w.spawn_fleet(1, positions=[(0.0, 0.0, 5.0)])
    c = Cascade(v)
    wps = [(0, 0, 8.0), (10.0, 4.0, 12.0), (-6.0, -5.0, 9.0)]
    traj = []
    per = int(seconds / len(wps) / w.dt)
    for wp in wps:
        c.set_target(list(wp), 0.0)
        for _ in range(per):
            c.update(w.dt)
            v.step_actuators(w.dt)
            pb.stepSimulation()
            traj.append(v.state()["position"].copy())
    e = v.energy_J
    w.close()
    return np.array(traj), e


def main():
    print("PHASE 4 - DETERMINISM AND EMERGENCY BEHAVIOUR")
    t0 = time.time()

    # ---- TEST 6: determinism --------------------------------------------
    a, ea = run_mission(seed=7)
    b, eb = run_mission(seed=7)
    cdif, ec = run_mission(seed=99)
    n = min(len(a), len(b))
    dev = np.linalg.norm(a[:n] - b[:n], axis=1)
    max_dev = float(dev.max())
    final_dev = float(dev[-1])
    METRICS["determinism_max_deviation_m"] = float(f"{max_dev:.3e}")
    METRICS["determinism_final_deviation_m"] = float(f"{final_dev:.3e}")
    METRICS["determinism_energy_delta_J"] = float(f"{abs(ea - eb):.3e}")
    check("T6a", "identical runs reproduce to tolerance",
          max_dev < 1e-6,
          f"max {max_dev:.2e} m, final {final_dev:.2e} m", "< 1e-6 m",
          f"energy delta {abs(ea-eb):.2e} J over {n} steps")

    # A different seed must NOT change a deterministic mission -- there is no
    # stochastic term in the flight path. Confirming that stops "determinism"
    # from silently meaning "the seed is ignored everywhere".
    n2 = min(len(a), len(cdif))
    seed_dev = float(np.linalg.norm(a[:n2] - cdif[:n2], axis=1).max())
    METRICS["seed_sensitivity_m"] = float(f"{seed_dev:.3e}")
    check("T6b", "flight path has no stochastic term",
          seed_dev < 1e-6, f"{seed_dev:.2e} m across seeds", "< 1e-6 m",
          "seed 7 vs seed 99; noise enters later, at the sensor layer")

    # ---- TEST 7: emergency ----------------------------------------------
    # 7a LOW BATTERY -> RETURN
    w = AvianWorld(gui=False, load_bridge=False)
    v, = w.spawn_fleet(1, positions=[(20.0, 15.0, 10.0)])
    c = Cascade(v)
    c.set_target([20.0, 15.0, 10.0], 0.0)
    sup = SafetySupervisor(v, c, home=(0.0, 0.0, 6.0))
    for _ in range(int(6.0 / w.dt)):
        sup.update(w.t)
        c.update(w.dt)
        v.step_actuators(w.dt)
        pb.stepSimulation()
        w.t += w.dt
    v.set_battery_pct(15.0)                       # 15 % remaining
    for _ in range(int(30.0 / w.dt)):
        sup.update(w.t)
        c.update(w.dt)
        v.step_actuators(w.dt)
        pb.stepSimulation()
        w.t += w.dt
    d_home = float(np.linalg.norm(v.state()["position"]
                                  - np.array([0.0, 0.0, 6.0])))
    METRICS["low_battery_state"] = sup.state
    METRICS["low_battery_distance_to_home_m"] = round(d_home, 2)
    check("T7a", "low battery triggers RETURN and flies home",
          sup.state == "RETURN" and d_home < 2.0,
          f"{sup.state}, {d_home:.2f} m from home", "RETURN, < 2 m",
          sup.reason[:52])

    # 7b MOTOR FAILURE -> RETURN (one rotor) / HOLD (two)
    v2, = w.spawn_fleet(1, positions=[(-30.0, 0.0, 12.0)])
    v2.vid = "AVIAN_02"
    c2 = Cascade(v2)
    c2.set_target([-30.0, 0.0, 12.0], 0.0)
    sup2 = SafetySupervisor(v2, c2, home=(-30.0, 0.0, 6.0))
    for _ in range(int(8.0 / w.dt)):
        sup2.update(w.t)
        c2.update(w.dt)
        v2.step_actuators(w.dt)
        pb.stepSimulation()
        w.t += w.dt
    z_before = v2.state()["position"][2]
    v2.fail_motor(0)
    for _ in range(int(20.0 / w.dt)):
        sup2.update(w.t)
        c2.update(w.dt)
        v2.step_actuators(w.dt)
        pb.stepSimulation()
        w.t += w.dt
    st2 = v2.state()
    alive = bool(st2["position"][2] > 3.0
                 and abs(np.degrees(st2["rpy"][0])) < 45.0
                 and abs(np.degrees(st2["rpy"][1])) < 45.0)
    METRICS["motor_fail_state"] = sup2.state
    METRICS["motor_fail_altitude_m"] = round(float(st2["position"][2]), 2)
    METRICS["motor_fail_attitude_deg"] = [
        round(float(np.degrees(st2["rpy"][0])), 2),
        round(float(np.degrees(st2["rpy"][1])), 2)]
    check("T7b", "single rotor failure: stays airborne, enters RETURN",
          sup2.state == "RETURN" and alive,
          f"{sup2.state}, z={st2['position'][2]:.1f} m, "
          f"tilt {abs(np.degrees(st2['rpy'][1])):.1f} deg",
          "RETURN, airborne, tilt < 45 deg",
          f"was z={z_before:.1f} m; X8 redundancy")

    # 7c SENSOR INVALID -> HOLD
    sup2.sensor_valid = False
    for _ in range(int(3.0 / w.dt)):
        sup2.update(w.t)
        c2.update(w.dt)
        v2.step_actuators(w.dt)
        pb.stepSimulation()
        w.t += w.dt
    METRICS["sensor_invalid_state"] = sup2.state
    check("T7c", "invalid state estimate triggers HOLD",
          sup2.state == "HOLD", sup2.state, "HOLD", sup2.reason[:52])

    # 7d LATCHING: the state does not silently clear itself
    sup2.sensor_valid = True
    for _ in range(int(2.0 / w.dt)):
        sup2.update(w.t)
        c2.update(w.dt)
        v2.step_actuators(w.dt)
        pb.stepSimulation()
        w.t += w.dt
    latched = sup2.state
    sup2.clear("operator acknowledged")
    METRICS["latched_state_after_recovery"] = latched
    METRICS["state_after_explicit_clear"] = sup2.state
    check("T7d", "safety state latches until conditions clear with margin",
          latched in ("HOLD", "RETURN") and sup2.state == "NOMINAL",
          f"held {latched}, cleared to {sup2.state}",
          "latched then explicitly cleared",
          f"{len(sup2.transitions)} recorded transitions")

    w.close()
    METRICS["wall_time_s"] = round(time.time() - t0, 1)

    n_pass = sum(1 for r in RESULTS if r["status"] == "PASS")
    n_fail = len(RESULTS) - n_pass
    print(f"\n  {n_pass} pass, {n_fail} fail of {len(RESULTS)} checks")
    json.dump({"checks": RESULTS, "metrics": METRICS,
               "run_id": os.environ.get("AVIAN_RUN_ID"),
               "summary": {"pass": n_pass, "fail": n_fail}},
              open(os.path.join(ROOT, "logs", "phase4_determinism.json"),
                   "w"), indent=2)
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
