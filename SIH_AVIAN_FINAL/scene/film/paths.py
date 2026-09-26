"""Per-frame camera geometry for a beat, without Blender.

render_film.py builds these paths inside Blender; compose_film.py needs the
same numbers afterwards to put a truthful ALT and RANGE on the HUD. Rather
than have the renderer dump them (it is already running, and a meta file
written mid-flight would not cover the beats already finished), the geometry
is reproduced here from the same inputs: pose_audit_track.csv for real beats,
and the declared standoff plus the easing curve for staged ones.

The onboard camera transform is identical to rigs.py / measure_flown.py:

    R   = quat_to_matrix(q)
    Rc  = R @ ry(gimbal)
    cam = p + R @ BOOM + Rc @ SENSOR
"""
from __future__ import annotations

import json
import math

import numpy as np

ROOT = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL"
TRACK = (f"{ROOT}/gazebo/mission_follower/results/full_pass_05/"
         "pose_audit_track.csv")
BOOM = np.array([0.35, 0.0, 0.05])
SENSOR = np.array([0.03, 0.0, 0.0])
MAX_GAP_S = 0.25

_T = None
_GT = None


def track():
    global _T
    if _T is None:
        a = np.genfromtxt(TRACK, delimiter=",", skip_header=1)
        a = a[np.isfinite(a).all(axis=1)]
        _T = a[np.argsort(a[:, 0])]
    return _T


def defects():
    global _GT
    if _GT is None:
        _GT = {d["defect_id"]: d for d in json.load(
            open(f"{ROOT}/scene/AVIAN_defect_ground_truth_FINAL.json"))["defects"]}
    return _GT


def _qmat(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _ry(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _cam_at(t_sim):
    a = track()
    ts = a[:, 0]
    k = int(np.searchsorted(ts, t_sim))
    if k <= 0 or k >= len(ts) or (ts[k] - ts[k - 1]) > MAX_GAP_S:
        return None
    lo, hi = a[k - 1], a[k]
    w = (t_sim - lo[0]) / max(hi[0] - lo[0], 1e-9)
    p = lo[2:5] + w * (hi[2:5] - lo[2:5])
    qa, qb = lo[5:9], hi[5:9]
    q = qa + w * (qb * np.sign(np.dot(qa, qb)) - qa)
    q /= max(np.linalg.norm(q), 1e-12)
    jnt = lo[9] + w * (hi[9] - lo[9])
    R = _qmat(*q)
    Rc = R @ _ry(jnt)
    return p + R @ BOOM + Rc @ SENSOR


def beat_geometry(beat, n_frames, fps, approach, hold, card, peel,
                  min_legal_standoff):
    """-> list of dicts: {alt_m, range_m} per frame, mirroring render_film."""
    d = defects()[beat["defect"]]
    tgt = np.array(d["position_m"], dtype=float)
    out = []

    if beat["kind"] == "real":
        half = (n_frames / 2) / fps
        last = None
        for i in range(n_frames):
            c = _cam_at(beat["t"] - half + i / fps)
            if c is None:
                c = last
            last = c
            if c is None:
                out.append(None)
                continue
            out.append({"alt_m": float(c[2]),
                        "range_m": float(np.linalg.norm(tgt - c))})
        first = next((o for o in out if o), None)
        out = [o if o else first for o in out]
    else:
        vd = np.array(d.get("recommended_view_direction")
                      or d.get("surface_normal") or [0, 1, 0], dtype=float)
        if np.linalg.norm(vd) < 1e-9:
            vd = np.array([0.0, 1.0, 0.0])
        vd /= np.linalg.norm(vd)
        near = float(beat["standoff"])
        far = near * 1.7
        for i in range(n_frames):
            if i < approach:
                u = i / max(1, approach - 1)
                r = far + (near - far) * (1 - (1 - u) ** 3)
            elif i < approach + hold + card:
                r = near
            else:
                u = (i - approach - hold - card) / max(1, peel - 1)
                r = near + (far - near) * 0.35 * (u ** 2)
            r = max(r, min_legal_standoff)
            c = tgt + vd * r
            out.append({"alt_m": float(c[2]), "range_m": float(r)})
    return out
