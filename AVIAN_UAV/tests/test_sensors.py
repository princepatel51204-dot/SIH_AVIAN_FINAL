"""Phase 1E tests 13-17: RGB, depth, LiDAR, IMU, GNSS.

A sensor that returns an array is not a working sensor. Each test here checks
that the sensor measures something CORRECT about a known scene -- that the
depth camera reports the true distance to a girder, that the LiDAR sees the
deck above and not the aircraft below it, that the IMU reads +1 g at rest, and
that GNSS actually degrades under the bridge.
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
from simulation.controller import Cascade                   # noqa: E402
import params as PENV                                      # noqa: E402

RESULTS, METRICS = [], {}


def check(cid, name, ok, measured, expected, detail=""):
    RESULTS.append({"id": cid, "name": name,
                    "status": "PASS" if ok else "FAIL",
                    "measured": str(measured), "expected": str(expected),
                    "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid:4} {name:<44} "
          f"{str(measured):<30} {detail}")
    return ok


def settle(v, c, steps=600):
    for _ in range(steps):
        c.update(1 / 240.0)
        v.step_actuators(1 / 240.0)
        pb.stepSimulation()


def main():
    print("PHASE 1E - SENSORS")
    t0 = time.time()
    w = AvianWorld(gui=False, load_bridge=True, seed=11)

    xm = 2100.0
    soffit = PENV.soffit_z(xm)
    # Hover in the open, off to the side of the deck, under the soffit.
    v, = w.spawn_fleet(1, positions=[(xm, -22.0, soffit - 3.0)])
    S = v.attach_sensors(seed=11)
    c = Cascade(v)
    c.set_target([xm, -22.0, soffit - 3.0], 0.0)
    settle(v, c, 1400)

    # ---- 13 RGB ----------------------------------------------------------
    t_r = time.time()
    img = S["rgb_front"].read(w.t)
    dt_loop = time.time() - t_r
    a = img["image"]
    t_r = time.time()
    cap = S["rgb_front"].read(w.t, capture=True)
    dt_cap = time.time() - t_r
    METRICS["rgb_loop_res"] = [img["width"], img["height"]]
    METRICS["rgb_capture_res"] = [cap["width"], cap["height"]]
    METRICS["rgb_loop_ms"] = round(dt_loop * 1000, 1)
    METRICS["rgb_capture_ms"] = round(dt_cap * 1000, 1)
    METRICS["rgb_gsd_mm_at_1m"] = img["gsd_mm_at_1m"]
    nonuniform = float(a.std())
    check("S13", "RGB camera renders a non-uniform image at both resolutions",
          img["width"] == 640 and cap["width"] == 1920 and nonuniform > 3.0,
          f"640x480 {dt_loop*1000:.0f} ms, 1920x1080 {dt_cap*1000:.0f} ms",
          "both resolutions, image std > 3",
          f"std {nonuniform:.1f} counts, GSD {img['gsd_mm_at_1m']} mm/px @1m")

    # ---- 14 DEPTH: measure a KNOWN distance ------------------------------
    # Park the aircraft a fixed stand-off in front of a river pier column,
    # nose-on, and compare the range the camera reports at the principal
    # point against a ray cast along the SAME boresight.
    #
    # The earlier version of this test compared `nearest_m` against
    # `w.min_clearance()`. That was not a valid comparison: min_clearance is
    # the distance to the nearest structure ANYWHERE in 3-D, and the nearest
    # structure to an aircraft parked under the deck is the soffit ABOVE it,
    # which a forward-looking camera cannot see. A camera can only be checked
    # against geometry inside its own field of view, so the truth here is a
    # ray along the optical axis.
    pier_x = min((s[0] for s in PENV.pier_stations() if s[1] == "river"),
                 key=lambda x: abs(x - 2055.0))
    col_y = PENV.PIER_COL_SPACING / 2.0
    stand = 4.0
    v2, = w.spawn_fleet(1, positions=[(pier_x - stand, col_y, 10.0)])
    S2 = v2.attach_sensors(seed=12)
    c2 = Cascade(v2)
    c2.set_target([pier_x - stand, col_y, 10.0], 0.0)   # yaw 0 -> +X, at pier
    settle(v2, c2, 1600)

    dep = S2["depth"]
    d = dep.read(w.t)
    cam_p, cam_q = dep.pose()
    axis = np.array(pb.getMatrixFromQuaternion(cam_q)).reshape(3, 3) @ \
        np.array([0.0, 0.0, -1.0])
    # Ray along the boresight, stepping past the aircraft's own structure so
    # the truth is the first ENVIRONMENT surface, exactly what the masked
    # depth image reports.
    truth, start = None, cam_p + axis * 1e-3
    for _ in range(8):
        r = pb.rayTestBatch([list(start)], [list(cam_p + axis * 30.0)])[0]
        if r[0] < 0:
            break
        if r[0] != v2.body:
            truth = float(np.linalg.norm(np.array(r[3]) - cam_p))
            break
        start = np.array(r[3]) + axis * 0.05

    meas = d["centre_range_m"]
    err = (abs(meas - truth) if (meas is not None and truth is not None)
           else float("inf"))
    METRICS["depth_boresight_measured_m"] = meas
    METRICS["depth_boresight_raycast_m"] = (round(truth, 4) if truth
                                            else None)
    METRICS["depth_error_m"] = round(float(err), 4)
    METRICS["depth_valid_fraction"] = d["valid_fraction"]
    METRICS["depth_self_masked_px"] = d["self_masked_pixels"]
    METRICS["depth_optical_axis_world"] = [round(float(a), 4) for a in axis]
    check("S14", "depth camera range agrees with true geometry",
          meas is not None and truth is not None and err < 0.6
          and d["valid_fraction"] > 0.05,
          f"{meas} m vs ray-cast {truth:.3f} m" if truth else "no ray truth",
          "error < 0.6 m on the boresight",
          f"{d['valid_pixels']} valid px "
          f"({d['valid_fraction']*100:.0f} %), {d['self_masked_pixels']} px "
          f"self-masked, gate {d['min_range_m']}-{d['max_range_m']} m")

    # ---- 15 LIDAR --------------------------------------------------------
    t_l = time.time()
    L = S2["lidar"].read(w.t)
    dt_l = time.time() - t_l
    pts = L["points_world"]
    above = int((pts[:, 2] > v2.state()["position"][2] + 1.0).sum()) \
        if len(pts) else 0
    METRICS["lidar_rays"] = L["n_rays"]
    METRICS["lidar_returns"] = L["n_returns"]
    METRICS["lidar_self_occluded"] = L["self_occluded_rays"]
    METRICS["lidar_ms"] = round(dt_l * 1000, 1)
    METRICS["lidar_decimation"] = L["decimation_factor"]
    METRICS["lidar_nearest_m"] = L["nearest_m"]
    check("S15", "LiDAR returns structure, rejects its own airframe",
          L["n_returns"] > 500 and L["self_occluded_rays"] > 0
          and above > 50 and L["nearest_m"] > 0.5,
          f"{L['n_returns']} returns, {L['self_occluded_rays']} self-hits "
          f"discarded", "> 500 returns, self-hits removed",
          f"{above} points above the aircraft (the deck), "
          f"{dt_l*1000:.0f} ms, {L['decimation_factor']}x decimated")

    # ---- 16 IMU: +1 g at rest, gravity in the right place ----------------
    acc, gyr = [], []
    for k in range(240):
        c2.update(1 / 240.0)
        v2.step_actuators(1 / 240.0)
        pb.stepSimulation()
        r = S2["imu"].read(w.t + k / 240.0)
        acc.append(r["linear_acceleration"])
        gyr.append(r["angular_velocity"])
    acc = np.array(acc)
    gyr = np.array(gyr)
    mean_az = float(acc[:, 2].mean())
    mean_g = float(np.linalg.norm(acc.mean(axis=0)))
    gyro_rms = float(np.sqrt(np.mean(gyr ** 2)))
    METRICS["imu_mean_specific_force_z"] = round(mean_az, 4)
    METRICS["imu_mean_magnitude_ms2"] = round(mean_g, 4)
    METRICS["imu_gyro_rms_rad_s"] = round(gyro_rms, 5)
    check("S16", "IMU reads +1 g in hover with bounded gyro noise",
          abs(mean_g - 9.80665) < 0.35 and gyro_rms < 0.15,
          f"|a| = {mean_g:.3f} m/s2, gyro RMS {gyro_rms:.4f} rad/s",
          "9.807 +/- 0.35, gyro < 0.15",
          "specific force, so hover reads +g not 0")

    # ---- 17 GNSS: degrades under the deck --------------------------------
    open_sky = S["gnss"].read(w.t)          # aircraft 1, out to the side
    # Move aircraft 2 directly under the deck between the girders.
    for _ in range(4):
        pb.resetBasePositionAndOrientation(
            v2.body, [xm, 0.0, soffit - 1.0], [0, 0, 0, 1])
        pb.stepSimulation()
    under = S2["gnss"].read(w.t)
    METRICS["gnss_open_sats"] = open_sky["satellites_visible"]
    METRICS["gnss_open_quality"] = open_sky["quality"]
    METRICS["gnss_under_sats"] = under["satellites_visible"]
    METRICS["gnss_under_quality"] = under["quality"]
    METRICS["gnss_datum"] = open_sky.get("datum")
    degraded = under["satellites_visible"] < open_sky["satellites_visible"]
    has_fix = (open_sky["valid"] and open_sky["latitude_deg"] is not None)
    check("S17", "GNSS gives a fix in the open and degrades under structure",
          has_fix and degraded,
          f"open {open_sky['satellites_visible']} sats "
          f"({open_sky['quality']}) -> under "
          f"{under['satellites_visible']} ({under['quality']})",
          "fewer satellites under the deck",
          f"lat {open_sky['latitude_deg']}, lon "
          f"{open_sky['longitude_deg']} (arbitrary datum)")

    # ---- S18 rate limiting is real ---------------------------------------
    # The gate accumulates phase, so rewinding the clock means re-anchoring
    # it -- clearing `last_stamp` alone leaves `_next_due` sitting wherever
    # the 240 reads in S16 left it and the sensor stays mute for a whole
    # simulated second. `reset_timing` is the supported way to do that, and
    # a mission leg or a replay needs exactly the same call.
    imu = S2["imu"]
    imu.reset_timing(0.0)
    n_due = sum(1 for k in range(240) if imu.due(k / 240.0)
                and (imu.envelope(k / 240.0, {}) or True))
    METRICS["imu_samples_per_sim_second"] = n_due
    check("S18", "sensors are rate-limited on simulation time",
          190 <= n_due <= 210, f"{n_due} IMU samples in 1 s",
          "200 +/- 10 at 200 Hz",
          "gating is on sim time, so determinism survives")

    w.close()
    METRICS["wall_time_s"] = round(time.time() - t0, 1)
    n_pass = sum(1 for r in RESULTS if r["status"] == "PASS")
    n_fail = len(RESULTS) - n_pass
    print(f"\n  {n_pass} pass, {n_fail} fail of {len(RESULTS)} checks")
    json.dump({"checks": RESULTS, "metrics": METRICS,
               "run_id": os.environ.get("AVIAN_RUN_ID"),
               "summary": {"pass": n_pass, "fail": n_fail}},
              open(os.path.join(ROOT, "logs", "phase1_sensors.json"), "w"),
              indent=2)
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
