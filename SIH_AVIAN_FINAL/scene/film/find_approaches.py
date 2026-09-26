"""Which of the flown poses actually frame a detector-positive defect?

The 8 waypoints where the live detector fired during full_pass_05 are NOT
usable as film beats: those detections were made against Gazebo's flat,
untextured collision geometry, and the poses sit at y ~ 30 m, well outside the
bridge's own y range of about -5.6..5.6 m. Rendering the Blender scene from
one of them shows clean, undamaged pier (verified by eye on media/_filmtest/
onboard.png). Using them would mean drawing a box where this scene has no
defect, which the brief forbids.

So instead this asks the honest question directly: over all 160,052 logged
poses, does the real onboard camera ever actually look at one of the 22
defects the detector fires on? A pose qualifies when the defect is

  * within MAX_RANGE of the camera,
  * inside the camera's real field of view (from the 24 mm lens on a 36 mm
    sensor), with a margin so it is not clipping the frame edge, and
  * facing the camera -- the defect's own surface normal must point back
    within FACING_DEG, or we would be looking at the back of the surface.

Output feeds the shot plan: every defect gets either a real-trajectory window
or a note that it needs a staged approach, and the film report reproduces that
split verbatim.
"""
import json
import math
import os

import numpy as np

ROOT = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL"
TRACK = f"{ROOT}/gazebo/mission_follower/results/full_pass_05/pose_audit_track.csv"
GT = f"{ROOT}/scene/AVIAN_defect_ground_truth_FINAL.json"
CAST = f"{ROOT}/media/_casting/casting_detections.json"
INDEX = f"{ROOT}/media/_casting/casting_index.json"
OUT = f"{ROOT}/scene/film/approaches.json"

BOOM = np.array([0.35, 0.0, 0.05])
SENSOR = np.array([0.03, 0.0, 0.0])
LENS_MM, SENSOR_MM = 24.0, 36.0
MAX_RANGE = 12.0
MIN_RANGE = 0.8
FOV_MARGIN = 0.72          # fraction of the half-FOV we require, keeps it off the edge
FACING_DEG = 75.0


def qmat_batch(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = len(q)
    R = np.empty((n, 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - w * z); R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y); R[:, 2, 1] = 2 * (y * z + w * x); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def ry_batch(t):
    c, s = np.cos(t), np.sin(t)
    n = len(t)
    R = np.zeros((n, 3, 3))
    R[:, 0, 0] = c; R[:, 0, 2] = s; R[:, 1, 1] = 1; R[:, 2, 0] = -s; R[:, 2, 2] = c
    return R


def main():
    a = np.genfromtxt(TRACK, delimiter=",", skip_header=1)
    a = a[np.isfinite(a).all(axis=1)]
    a = a[np.argsort(a[:, 0])]
    t, p, q, jnt = a[:, 0], a[:, 2:5], a[:, 5:9], a[:, 9]

    R = qmat_batch(q)
    Rc = R @ ry_batch(jnt)
    cam = p + np.einsum("nij,j->ni", R, BOOM) + np.einsum("nij,j->ni", Rc, SENSOR)
    fwd = Rc[:, :, 0]

    half_fov = math.atan((SENSOR_MM / 2) / LENS_MM)
    cos_lim = math.cos(half_fov * FOV_MARGIN)
    cos_face = math.cos(math.radians(FACING_DEG))

    gt = {d["defect_id"]: d for d in json.load(open(GT))["defects"]}
    idx = {r["defect_id"]: r for r in json.load(open(INDEX))}
    fired = {}
    for r in json.load(open(CAST)):
        did = os.path.basename(r["image"])[:-4]
        if r["detections"]:
            fired[did] = max(r["detections"], key=lambda d: d["score"])

    out = []
    for did, best in sorted(fired.items(), key=lambda kv: -kv[1]["score"]):
        d = gt[did]
        pos = np.array(d["position_m"])
        nrm = np.array(d.get("surface_normal") or [0, 1, 0], dtype=float)
        if np.linalg.norm(nrm) > 1e-9:
            nrm /= np.linalg.norm(nrm)

        v = pos - cam
        rng = np.linalg.norm(v, axis=1)
        near = (rng > MIN_RANGE) & (rng < MAX_RANGE)
        rec = {"defect_id": did, "type": d["type"],
               "host_surface": d.get("host_surface"),
               "bridge_section": d.get("bridge_section"),
               "position_m": [round(float(x), 3) for x in pos],
               "detector": {"family": best["family"], "score": best["score"],
                            "bbox": best["bbox"]},
               "casting_standoff_m": idx[did]["standoff_m"]}

        if not near.any():
            rec.update({"real_trajectory": False,
                        "reason": f"closest approach {rng.min():.1f} m, never within {MAX_RANGE} m"})
            out.append(rec)
            continue

        u = v / np.maximum(rng, 1e-9)[:, None]
        on_axis = np.einsum("ni,ni->n", u, fwd)
        facing = -np.einsum("i,ni->n", nrm, u)
        ok = near & (on_axis > cos_lim) & (facing > cos_face)

        if not ok.any():
            best_i = int(np.argmax(np.where(near, on_axis, -2)))
            rec.update({"real_trajectory": False,
                        "reason": "came within range but never pointed at it",
                        "closest_range_m": round(float(rng.min()), 2),
                        "best_on_axis_deg": round(float(math.degrees(math.acos(
                            max(-1, min(1, on_axis[best_i]))))), 1),
                        "fov_half_deg": round(math.degrees(half_fov * FOV_MARGIN), 1)})
            out.append(rec)
            continue

        ii = np.where(ok)[0]
        # longest contiguous run of qualifying samples
        splits = np.where(np.diff(ii) > 3)[0]
        runs = np.split(ii, splits + 1)
        run = max(runs, key=len)
        i_best = run[int(np.argmin(rng[run]))]
        rec.update({
            "real_trajectory": True,
            "n_qualifying_samples": int(ok.sum()),
            "window_t_sim": [round(float(t[run[0]]), 2), round(float(t[run[-1]]), 2)],
            "window_duration_s": round(float(t[run[-1]] - t[run[0]]), 2),
            "closest_range_m": round(float(rng[i_best]), 2),
            "t_closest": round(float(t[i_best]), 2),
            "cam_xyz_at_closest": [round(float(x), 3) for x in cam[i_best]],
            "drone_xyz_at_closest": [round(float(x), 3) for x in p[i_best]],
            "gimbal_deg_at_closest": round(float(math.degrees(jnt[i_best])), 1),
            "on_axis_deg": round(float(math.degrees(math.acos(
                max(-1, min(1, on_axis[i_best]))))), 1),
        })
        out.append(rec)

    real = [r for r in out if r.get("real_trajectory")]
    json.dump({"max_range_m": MAX_RANGE, "lens_mm": LENS_MM,
               "fov_half_deg": round(math.degrees(half_fov), 2),
               "fov_margin": FOV_MARGIN, "facing_deg": FACING_DEG,
               "n_detector_positive": len(fired), "n_real_trajectory": len(real),
               "defects": out}, open(OUT, "w"), indent=1)

    print(f"detector-positive defects: {len(fired)}")
    print(f"reachable on the real flown trajectory: {len(real)}\n")
    for r in out:
        if r.get("real_trajectory"):
            print(f"  REAL   {r['defect_id']:<32} {r['type']:<20} "
                  f"{r['detector']['score']:.3f}  t {r['window_t_sim'][0]:8.1f}-{r['window_t_sim'][1]:8.1f}s "
                  f"({r['window_duration_s']:5.1f}s)  {r['closest_range_m']:5.2f} m  {r['on_axis_deg']:4.1f} deg off-axis")
    print()
    for r in out:
        if not r.get("real_trajectory"):
            print(f"  STAGED {r['defect_id']:<32} {r['type']:<20} "
                  f"{r['detector']['score']:.3f}  {r['reason']}")
    print(f"\nwrote {OUT}")


if __name__ == "__main__":
    main()
