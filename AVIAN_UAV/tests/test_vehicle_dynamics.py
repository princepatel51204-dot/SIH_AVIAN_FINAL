"""Phase 4 tests: propulsion model, allocation, and flight.

Every claim here is a number produced by running the simulation, not an
assertion about how it was written.
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

RESULTS = []
METRICS = {}


def check(cid, name, ok, measured, expected, detail=""):
    RESULTS.append({"id": cid, "name": name,
                    "status": "PASS" if ok else "FAIL",
                    "measured": str(measured), "expected": str(expected),
                    "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid:4} {name:<42} "
          f"{str(measured):<30} {detail}")
    return ok


def fly(world, v, ctrl, seconds, target=None, yaw=None, hook=None):
    n = int(seconds / world.dt)
    log = []
    for k in range(n):
        if target is not None:
            ctrl.set_target(target, yaw)
        if hook:
            hook(k, v, ctrl)
        ctrl.update(world.dt)
        v.step_actuators(world.dt)
        pb.stepSimulation()
        world.t += world.dt
        st = v.state()
        log.append((world.t, st["position"].copy(), st["rpy"].copy(),
                    st["velocity"].copy(), float(v.thrust.sum())))
    return log


# ===========================================================================
def main():
    print("PHASE 4 - VEHICLE DYNAMICS AND CONTROL")
    t_wall = time.time()

    w = AvianWorld(gui=False, load_bridge=False)
    v, = w.spawn_fleet(1, positions=[(0.0, 0.0, 3.0)])
    ctrl = Cascade(v)

    # ---- D01 allocation matrix is well posed -----------------------------
    A = v.A
    cond = float(np.linalg.cond(A))
    METRICS["allocation_matrix"] = [[round(float(x), 5) for x in row]
                                    for row in A]
    METRICS["allocation_condition_number"] = round(cond, 3)
    check("D01", "control allocation matrix is well conditioned",
          A.shape == (4, 8) and cond < 200.0,
          f"4x8, cond {cond:.1f}", "4x8, cond < 200",
          f"{v.n_rotors} rotors")

    # ---- D02 collective is symmetric -------------------------------------
    f = v.A_pinv @ np.array([v.weight, 0, 0, 0])
    spread = float(f.max() - f.min())
    METRICS["hover_thrust_per_rotor_N"] = round(float(f.mean()), 3)
    METRICS["hover_thrust_total_N"] = round(v.weight, 2)
    check("D02", "pure collective loads all 8 rotors equally",
          spread < 1e-6, f"spread {spread:.2e} N", "< 1e-6 N",
          f"{f.mean():.2f} N per rotor at hover")

    # ---- D03 roll / pitch / yaw authority --------------------------------
    auth = {}
    for k, nm in ((1, "roll"), (2, "pitch"), (3, "yaw")):
        d = np.zeros(4)
        d[0] = v.weight
        # largest moment achievable without exceeding per-rotor limits
        lo, hi = 0.0, 400.0
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            d[k] = mid
            fk = v.A_pinv @ d
            if fk.max() <= v.f_max[0] and fk.min() >= 0.0:
                lo = mid
            else:
                hi = mid
        auth[nm] = round(lo, 2)
    METRICS["authority_Nm"] = auth
    check("D03", "roll, pitch and yaw authority at hover",
          auth["roll"] > 20 and auth["pitch"] > 20 and auth["yaw"] > 1.0,
          f"roll {auth['roll']}, pitch {auth['pitch']}, "
          f"yaw {auth['yaw']} Nm",
          "roll/pitch > 20, yaw > 1 Nm",
          "measured by bisection against per-rotor limits")

    # ---- D04 yaw does not disturb roll or pitch --------------------------
    d = np.array([v.weight, 0, 0, 5.0])
    fk = v.A_pinv @ d
    got = A @ fk
    cross = float(max(abs(got[1]), abs(got[2])))
    check("D04", "yaw demand produces no roll or pitch coupling",
          cross < 1e-6, f"cross-coupling {cross:.2e} Nm", "< 1e-6 Nm",
          "the X8 property that motivates the layout")

    # ---- TEST 1: free flight - takeoff, hover, land ----------------------
    ctrl.set_target([0, 0, 3.0], 0.0)
    fly(w, v, ctrl, 3.0, [0, 0, 3.0], 0.0)          # settle
    ctrl.reset_integrators()
    t0 = w.t
    log = fly(w, v, ctrl, 12.0, [0, 0, 12.0], 0.0)  # climb to 12 m
    zs = np.array([p[1][2] for p in log])
    ts = np.array([p[0] for p in log])
    reached = np.where(zs > 12.0 - 0.5)[0]
    t_takeoff = float(ts[reached[0]] - t0) if len(reached) else float("nan")
    settle = zs[int(len(zs) * 0.6):]
    alt_err = float(np.sqrt(np.mean((settle - 12.0) ** 2)))
    vz_max = float(max(abs(p[3][2]) for p in log))
    METRICS["takeoff_time_s"] = round(t_takeoff, 2)
    METRICS["takeoff_altitude_rms_error_m"] = round(alt_err, 4)
    METRICS["max_climb_rate_mps"] = round(vz_max, 3)
    check("T1a", "takeoff reaches 12 m and holds",
          t_takeoff == t_takeoff and alt_err < 0.10,
          f"{t_takeoff:.1f} s, RMS {alt_err*100:.1f} cm", "< 10 cm RMS",
          f"peak climb {vz_max:.2f} m/s")

    # land
    log = fly(w, v, ctrl, 12.0, [0, 0, 1.0], 0.0)
    z_end = log[-1][1][2]
    check("T1b", "commanded descent tracks to 1 m",
          abs(z_end - 1.0) < 0.15, f"{z_end:.3f} m", "1.00 +/- 0.15 m")

    # ---- TEST 2: position hold -------------------------------------------
    ctrl.set_target([0, 0, 10.0], 0.0)
    fly(w, v, ctrl, 10.0, [0, 0, 10.0], 0.0)
    ctrl.reset_integrators()
    tgt = np.array([4.0, -3.0, 10.0])
    log = fly(w, v, ctrl, 25.0, tgt, 0.0)
    tail = log[int(len(log) * 0.5):]
    errs = np.array([np.linalg.norm(p[1] - tgt) for p in tail])
    att = np.array([p[2][:2] for p in tail])
    rms_pos = float(np.sqrt(np.mean(errs ** 2)))
    max_pos = float(errs.max())
    rms_att = float(np.degrees(np.sqrt(np.mean(att ** 2))))
    # settling time: first time it stays inside 0.25 m
    all_e = np.array([np.linalg.norm(p[1] - tgt) for p in log])
    inside = all_e < 0.25
    settle_t = float("nan")
    for i in range(len(inside)):
        if inside[i:].all():
            settle_t = log[i][0] - log[0][0]
            break
    METRICS["position_hold_rms_error_m"] = round(rms_pos, 4)
    METRICS["position_hold_max_error_m"] = round(max_pos, 4)
    METRICS["attitude_rms_error_deg"] = round(rms_att, 4)
    METRICS["settling_time_s"] = round(settle_t, 2)
    check("T2", "position hold RMS error",
          rms_pos < 0.15, f"RMS {rms_pos*100:.1f} cm, max {max_pos*100:.1f} cm",
          "< 15 cm RMS",
          f"attitude RMS {rms_att:.2f} deg, settle {settle_t:.1f} s")

    # ---- TEST 3: waypoint mission ----------------------------------------
    wps = [(0, 0, 10.0), (25.0, 0, 10.0), (25.0, 18.0, 14.0),
           (0.0, 18.0, 12.0), (0, 0, 10.0)]
    e0 = v.energy_J
    t_start = w.t
    wp_err = []
    for wp in wps:
        ctrl.set_target(list(wp), 0.0)
        lg = fly(w, v, ctrl, 14.0, list(wp), 0.0)
        wp_err.append(float(np.linalg.norm(lg[-1][1] - np.array(wp))))
    mission_t = w.t - t_start
    energy_Wh = (v.energy_J - e0) / 3600.0
    METRICS["waypoint_errors_m"] = [round(e, 4) for e in wp_err]
    METRICS["waypoint_max_error_m"] = round(max(wp_err), 4)
    METRICS["mission_time_s"] = round(mission_t, 2)
    METRICS["mission_energy_Wh"] = round(energy_Wh, 2)
    check("T3", "waypoint mission arrival accuracy",
          max(wp_err) < 0.30, f"max {max(wp_err)*100:.1f} cm over "
          f"{len(wps)} waypoints", "< 30 cm",
          f"{mission_t:.0f} s, {energy_Wh:.1f} Wh")

    # ---- TEST 5: disturbance rejection -----------------------------------
    ctrl.set_target([0, 0, 10.0], 0.0)
    fly(w, v, ctrl, 10.0, [0, 0, 10.0], 0.0)
    ctrl.reset_integrators()
    base = v.state()["position"].copy()

    def kick(k, veh, c):
        if k == 0:
            pb.applyExternalForce(veh.body, -1, [120.0, 60.0, 0.0],
                                  [0, 0, 0], pb.LINK_FRAME)
        if k == 1:
            pb.applyExternalTorque(veh.body, -1, [0, 0, 12.0],
                                   pb.WORLD_FRAME)

    log = fly(w, v, ctrl, 20.0, [0, 0, 10.0], 0.0, hook=kick)
    dev = max(float(np.linalg.norm(p[1] - base)) for p in log)
    final = float(np.linalg.norm(log[-1][1] - base))
    rec = float("nan")
    for p in log:
        if np.linalg.norm(p[1] - base) < 0.25:
            rec_candidate = p[0] - log[0][0]
            rec = rec_candidate
    # first time after the kick that it is back inside 0.25 m and stays
    errs = np.array([np.linalg.norm(p[1] - base) for p in log])
    rec = float("nan")
    for i in range(len(errs)):
        if (errs[i:] < 0.25).all():
            rec = log[i][0] - log[0][0]
            break
    METRICS["disturbance_max_deviation_m"] = round(dev, 3)
    METRICS["disturbance_recovery_s"] = round(rec, 2)
    METRICS["disturbance_final_error_m"] = round(final, 4)
    check("T5", "recovers from lateral force and yaw torque",
          final < 0.20 and rec == rec,
          f"peak {dev:.2f} m, recovered in {rec:.1f} s",
          "returns inside 0.25 m",
          "120 N lateral impulse + 12 Nm yaw")

    # ---- D05 motor saturation --------------------------------------------
    METRICS["saturation_events"] = v.saturation_events
    METRICS["max_saturation_ratio"] = round(v.max_saturation, 3)
    check("D05", "motor saturation stays bounded",
          v.max_saturation < 1.6,
          f"{v.saturation_events} events, peak "
          f"{v.max_saturation*100:.0f}% of limit", "< 160%",
          "clipped and counted, never silently ignored")

    # ---- D06 real-time factor --------------------------------------------
    wall = time.time() - t_wall
    METRICS["sim_time_s"] = round(w.t, 1)
    METRICS["wall_time_s"] = round(wall, 1)
    METRICS["real_time_factor"] = round(w.t / wall, 3)
    METRICS["timestep_s"] = w.dt
    check("D06", "real-time factor on this host",
          w.t / wall > 0.05, f"{w.t/wall:.2f}x", "> 0.05x (recorded)",
          f"{w.t:.0f} s sim in {wall:.0f} s wall, dt={w.dt*1000:.2f} ms")

    w.close()

    n_pass = sum(1 for r in RESULTS if r["status"] == "PASS")
    n_fail = len(RESULTS) - n_pass
    print(f"\n  {n_pass} pass, {n_fail} fail of {len(RESULTS)} checks")
    out = os.path.join(ROOT, "logs", "phase4_dynamics.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump({"checks": RESULTS, "metrics": METRICS,
               "run_id": os.environ.get("AVIAN_RUN_ID"),
               "summary": {"pass": n_pass, "fail": n_fail}},
              open(out, "w"), indent=2)
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
