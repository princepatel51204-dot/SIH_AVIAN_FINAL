#!/usr/bin/env python3
"""SIH_AVIAN_FINAL -- Task 1 v5: MEASURED coverage of a flown run.

For every camera frame the recorder saved (camera/frames.csv, ROS header
stamp = Gazebo sim time), the TRUE camera pose is rebuilt from the pose
audit's truth track (pose_audit_track.csv: base_link position + attitude
quaternion + true gimbal joint angle, 20 Hz sim, interpolated to the frame
stamp) and the airframe geometry (boom mount, gimbal pitch joint, sensor
offset). The patches in view are then found with the same camera model /
ray casting as the planner (covlib). Union over frames = flown coverage.

Also reports: coverage from frames taken only while holding at a reached
viewpoint (the last SETTLE_HOLD_S before each 'reached' event), and the
planned coverage of the plan the run flew.

Usage: measure_flown.py <run_dir> [plan.json]
"""
import csv
import json
import math
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import covlib as C  # noqa: E402
import numpy as np  # noqa: E402
import plan_v5 as PV  # noqa: E402

BOOM = np.array([0.35, 0.0, 0.05])      # base_link -> gimbal pivot (x500_base/model.sdf)
SENSOR = np.array([0.03, 0.0, 0.0])     # pivot -> camera sensor, in the gimbal link
SETTLE_HOLD_S = 1.0
MAX_TRACK_GAP_S = 0.25


def qmat(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def ry(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def load_track(path):
    a = np.genfromtxt(path, delimiter=",", skip_header=1)
    a = a[np.argsort(a[:, 0])]
    return a  # t_sim,t_wall,x,y,z,qw,qx,qy,qz,joint


def camera_poses(track, stamps):
    t = track[:, 0]
    k = np.searchsorted(t, stamps)
    ok = (k > 0) & (k < len(t))
    out = []
    for s, kk, o in zip(stamps, k, ok):
        if not o or t[kk] - t[kk - 1] > MAX_TRACK_GAP_S:
            out.append(None)
            continue
        a, b = track[kk - 1], track[kk]
        w = (s - a[0]) / max(b[0] - a[0], 1e-9)
        p = a[2:5] + w * (b[2:5] - a[2:5])
        q = a[5:9] + w * (b[5:9] * np.sign(np.dot(a[5:9], b[5:9])) - a[5:9])
        q /= np.linalg.norm(q)
        jnt = a[9] + w * (b[9] - a[9])
        if not np.isfinite(jnt):
            out.append(None)
            continue
        R = qmat(*q)
        Rc = R @ ry(jnt)
        cam = p + R @ BOOM + Rc @ SENSOR
        out.append((cam, Rc[:, 0], Rc[:, 1], Rc[:, 2]))
    return out


_W = {}


def _init():
    import numba
    numba.set_num_threads(1)
    S, g = PV.geometry()
    _W["V"] = C.Visibility(S, g, g["reason"] == 0)


def _run(job):
    V = _W["V"]
    return [(fi, V.visible(cam, f, l, u)) for fi, cam, f, l, u in job]


def union_visible(jobs):
    seen = {}
    with mp.get_context("fork").Pool(14, initializer=_init) as pool:
        for res in pool.imap_unordered(_run, jobs):
            for fi, v in res:
                seen[fi] = v
    return seen


def main():
    run = sys.argv[1]
    S, g = PV.geometry()
    area, vis = g["area"], g["reason"] == 0
    A = area[vis].sum()
    L = json.load(open(os.path.join(run, "mission_log.json")))
    plan_path = sys.argv[2] if len(sys.argv) > 2 else L["plan"]
    plan = json.load(open(plan_path))
    track = load_track(os.path.join(run, "pose_audit_track.csv"))
    fr = np.genfromtxt(os.path.join(run, "camera", "frames.csv"), delimiter=",", skip_header=1)
    stamps = fr[:, 1]
    t0 = time.time()
    poses = camera_poses(track, stamps)
    jobs, cur = [], []
    for fi, pz in enumerate(poses):
        if pz is None:
            continue
        cur.append((fi, *pz))
        if len(cur) == 300:
            jobs.append(cur)
            cur = []
    if cur:
        jobs.append(cur)
    seen = union_visible(jobs)
    flown = np.zeros(len(area), bool)
    for v in seen.values():
        flown[v] = True
    # frames while holding at a reached viewpoint
    reached_t = [e["t_sim"] for e in L["events"] if " reached: " in e["msg"] and e["msg"].startswith(("V5_", "CWP_"))]
    hold = np.zeros(len(stamps), bool)
    rt = np.array(sorted(reached_t))
    if len(rt):
        k = np.searchsorted(rt, stamps)
        k = np.clip(k, 0, len(rt) - 1)
        hold = (rt[k] - stamps >= 0) & (rt[k] - stamps <= SETTLE_HOLD_S)
    flown_hold = np.zeros(len(area), bool)
    for fi, v in seen.items():
        if hold[fi]:
            flown_hold[v] = True
    # planned coverage of the flown plan (camera pose from the plan)
    V = C.Visibility(S, g, vis)
    planned = np.zeros(len(area), bool)
    for w in plan["waypoints"]:
        if not w.get("reachable_in_plan", True):
            continue
        f, l_, u = C.cam_axes(w["heading_rad"], w.get("gimbal_pitch_rad", -C.FIXED_PITCH_DOWN))
        planned[V.visible(np.array(w.get("camera_world_m", w["position_m"])), f, l_, u)] = True
    comp = g["comp"]

    def pct(m, sub=None):
        base = vis if sub is None else vis & sub
        return round(100 * area[m & base].sum() / max(area[base].sum(), 1e-9), 2)

    res = {"run": os.path.basename(os.path.normpath(run)), "plan": os.path.relpath(plan_path, C.ROOT),
           "visible_area_m2": round(float(A), 1),
           "frames_total": int(len(stamps)), "frames_with_true_pose": int(sum(p is not None for p in poses)),
           "frames_holding_at_reached_viewpoints": int(hold.sum()),
           "coverage_pct": {"planned": pct(planned), "flown_all_frames": pct(flown), "flown_hold_frames_only": pct(flown_hold)},
           "covered_area_m2": {"planned": round(float(area[planned].sum()), 1), "flown_all_frames": round(float(area[flown].sum()), 1),
                               "flown_hold_frames_only": round(float(area[flown_hold].sum()), 1)},
           "planned_but_not_flown_m2": round(float(area[planned & ~flown].sum()), 1),
           "by_component": {str(c): {"visible_m2": round(float(area[vis & (comp == c)].sum()), 1),
                                     "planned_pct": pct(planned, comp == c), "flown_pct": pct(flown, comp == c),
                                     "flown_hold_pct": pct(flown_hold, comp == c)} for c in sorted(set(comp))},
           "method": "true base_link pose + true gimbal angle (pose_audit_track.csv, interpolated to each frame's "
                     "sim stamp) -> camera pose -> covlib camera model (640x480, 80x64.4 deg, 12 m, incidence "
                     "<=60 deg, ray-cast LOS vs collision JSON + Gazebo-only boxes)",
           "elapsed_s": round(time.time() - t0, 1)}
    json.dump(res, open(os.path.join(run, "coverage_flown.json"), "w"), indent=1)
    np.save(os.path.join(run, "coverage_flown_patches.npy"), np.flatnonzero(flown))
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
