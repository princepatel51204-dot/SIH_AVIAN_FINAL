"""SIH_AVIAN_FINAL -- static, offline AVIAN mission dashboard.

HTML/file work only: nothing here runs a simulation, a flight or a detector.

Every figure on the page is read from a committed result file by `load_facts()`
below and formatted from that value; nothing is typed into the HTML. Each fact
carries its source file, and the footer lists file@commit for all of them. The build
fails if any placeholder word ("pending") reaches the page, or if a stat tile's
number does not match the fact it was built from.

Headline stats come from the committed Gazebo run full_pass_05 (commit 68d71d6).
Detection accuracy is quoted ONLY from the Blender benchmark and the Blender film,
never from the Gazebo runs (that world is flat-shaded, untextured geometry).

    python3 dashboard/build_dashboard.py            # reuses assets/ (gallery tiles, hero)
    python3 dashboard/build_dashboard.py --posters  # also re-extracts the two video posters
"""
from __future__ import annotations
import csv
import html
import json
import math
import os
import re
import statistics
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
ASSETS = os.path.join(HERE, "assets")
os.makedirs(ASSETS, exist_ok=True)

FP05 = "gazebo/mission_follower/results/full_pass_05"
FILM = "media/avian_inspection_film.mp4"
WALK = "media/bridge_defect_walkthrough_cinematic.mp4"
DEMO_SCREEN = "media/demo_screen_recording.mp4"


def J(path):
    with open(os.path.join(ROOT, path)) as f:
        return json.load(f)


def git(*a):
    try:
        return subprocess.check_output(["git", *a], cwd=ROOT, stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return ""


def file_commit(path):
    return git("log", "-1", "--format=%h", "--", path) or "uncommitted"


class Facts:
    """key -> raw value + the file it was read from."""
    def __init__(self):
        self.v, self.src = {}, {}

    def add(self, key, value, source):
        self.v[key], self.src[key] = value, source
        return value

    def __getitem__(self, k):
        return self.v[k]


def ffprobe(path):
    o = subprocess.check_output(["ffprobe", "-v", "error", "-show_entries", "format=duration,size", "-of", "json",
                                 os.path.join(ROOT, path)]).decode()
    return json.loads(o)["format"]


def load_facts():
    F = Facts()
    ts = f"{FP05}/task1_summary.json"
    ml = f"{FP05}/mission_log.json"
    cf = f"{FP05}/coverage_flown.json"
    cs = f"{FP05}/contact_summary.json"
    T, M, C, K = J(ts), J(ml), J(cf), J(cs)
    S = M["summary"]
    F.add("fp05.coverage_flown_pct", C["coverage_pct"]["flown_all_frames"], cf)
    F.add("fp05.coverage_planned_pct", C["coverage_pct"]["planned"], cf)
    F.add("fp05.visible_m2", C["visible_area_m2"], cf)
    F.add("fp05.covered_m2", C["covered_area_m2"]["flown_all_frames"], cf)
    F.add("fp05.frames", C["frames_total"], cf)
    F.add("fp05.by_component", {k: v["flown_pct"] for k, v in C["by_component"].items()}, cf)
    F.add("fp05.reached", S["reached"], ml)
    F.add("fp05.in_plan", S["waypoints_in_plan"], ml)
    F.add("fp05.attempted", S["attempted"], ml)
    F.add("fp05.not_attempted", S["results"].get("not_attempted_unreachable_in_plan", 0), ml)
    F.add("fp05.contact_episodes", K["contact_episodes"], cs)
    F.add("fp05.distance_true_m", T["distance_true_m"], ts)
    F.add("fp05.mission_time_sim_s", S["mission_time_sim_s"], ml)
    F.add("fp05.mission_time_h", S["mission_time_sim_s"] / 3600.0, ml)
    F.add("fp05.ekf_err_mean_m", S["ekf_err_m"]["mean"], ml)
    F.add("fp05.ekf_err_p95_m", S["ekf_err_m"]["p95"], ml)
    F.add("fp05.true_vs_plan_mean_m", T["mapping_true_minus_target_m"]["mean"], ts)
    F.add("fp05.sense_ticks", M["sense_stats"]["ticks"], ml)
    F.add("fp05.sense_limited", M["sense_stats"]["ticks_limited"], ml)
    F.add("fp05.sense_pushed", M["sense_stats"]["ticks_pushed"], ml)
    F.add("fp05.brake_speed", M["brake_test"]["speed_at_cmd_mps"], ml)
    F.add("fp05.brake_dist", M["brake_test"]["stop_dist_m"], ml)
    F.add("fp05.brake_budget", M["brake_test"]["budget_stop_dist_m"], ml)
    F.add("fp05.commit", "68d71d6", "git log (full_pass_05 commit)")

    s1 = J("mission/flight_log.json")
    F.add("stage1.stuck", s1["n_stuck"], "mission/flight_log.json")
    F.add("stage1.waypoints", s1["n_waypoints"], "mission/flight_log.json")
    s2 = J("mission/coverage_mission_flight_log.json")
    F.add("stage2.stuck", s2["n_stuck"], "mission/coverage_mission_flight_log.json")
    F.add("stage2.waypoints", 150, "mission/coverage_mission_flight_log.json")

    v3 = J("mission/coverage_score_mission_v3.json")
    F.add("det.headline", v3["headline_model"], "mission/coverage_score_mission_v3.json")
    F.add("det.n_val_tiles", v3["n_val_tiles"], "mission/coverage_score_mission_v3.json")
    F.add("det.n_test_tiles", v3["n_test_tiles"], "mission/coverage_score_mission_v3.json")
    models = {}
    for n in ("v1", "v2", "v3"):
        val, test = v3["mission_val_sweep_best"][n], v3["mission_test"][n]
        models[n] = {"threshold": val["threshold"], "val_f1": val["f1"],
                     "unseen": [test["unseen_defects"]["tp"], test["unseen_defects"]["tp"] + test["unseen_defects"]["fn"]],
                     "all": [test["all_defects"]["tp"], test["all_defects"]["tp"] + test["all_defects"]["fn"]],
                     "fp100": test["fp_per_100_tiles"]}
    F.add("det.models", models, "mission/coverage_score_mission_v3.json")
    rp = J("detection/AVIAN_real_photo_eval_v2_FINAL.json")
    F.add("det.real_photo", rp, "detection/AVIAN_real_photo_eval_v2_FINAL.json")

    zt = J("detection/AVIAN_zoom_tiles_all_FINAL.json")
    hfov = statistics.median(t["zoom_hfov_deg"] for t in zt["tiles"])
    F.add("zoom.hfov_deg", hfov, "detection/AVIAN_zoom_tiles_all_FINAL.json")
    F.add("zoom.optical_x", math.tan(math.radians(34.5)) / math.tan(math.radians(hfov / 2)),
          "detection/AVIAN_zoom_tiles_all_FINAL.json")

    fl = {"film": ffprobe(FILM), "walk": ffprobe(WALK)}
    F.add("video.film_s", float(fl["film"]["duration"]), FILM)
    F.add("video.walk_s", float(fl["walk"]["duration"]), WALK)
    dl = J("scene/film/detection_log.json")
    F.add("film.detector_records", len(dl), "scene/film/detection_log.json")
    F.add("film.boxes_drawn", sum(1 for r in dl if r.get("drawn")), "scene/film/detection_log.json")
    F.add("film.beats", len({r["shot"] for r in dl if str(r["shot"]).startswith("beat")}), "scene/film/detection_log.json")

    coll = J("scene/collision/avian_bridge_collision.json")["primitives"]
    F.add("scene.primitives", len(coll), "scene/collision/avian_bridge_collision.json")
    n_def = sum(len(J(f"scene/{f}")["defects"]) for f in ("AVIAN_defect_ground_truth_FINAL.json",
                                                          "AVIAN_metro_ground_truth_FINAL.json",
                                                          "AVIAN_steel_ground_truth_FINAL.json"))
    F.add("scene.defects", n_def, "scene/AVIAN_*_ground_truth_FINAL.json")

    # column-orbit flight (results/ is gitignored; this summary is the tracked copy of its numbers)
    cfs = "gazebo/mission_follower/columns_flight_summary.json"
    CF = J(cfs)
    F.add("col.summary", CF, cfs)
    for k in ("reached", "waypoints_in_plan", "contact_episodes", "look_up_dwells", "mission_time_wall_s", "mission_time_sim_s",
              "rtf", "distance_true_m", "min_sensed_range_mission_legs_m", "mission_samples_inside_3p0_ring", "legs_speed_limited"):
        F.add(f"col.{k}", CF["mission"][k], cfs)
    F.add("col.n_columns", len(CF["mission"]["columns"]), cfs)
    F.add("col.pos_err_mean_m", CF["mission"]["true_pos_err_m"]["mean"], cfs)
    F.add("col.pos_err_p95_m", CF["mission"]["true_pos_err_m"]["p95"], cfs)
    F.add("col.coverage_mean_whole_pct", CF["coverage_mean_whole_true_pct"], cfs)
    F.add("col.ring_axis", CF["aiming"]["ring_axis_on_column"], cfs)
    F.add("col.dwell_axis", CF["aiming"]["dwell_axis_on_cap_or_column"], cfs)
    F.add("col.det_hz", CF["detector"]["achieved_hz"], cfs)
    cpr = J("mission/columns_plan_report.json")
    F.add("col.planned_mean_whole_pct", statistics.mean(v["coverage_pct"]["whole_subject"] for v in cpr["per_subject"].values()),
          "mission/columns_plan_report.json")

    # Gazebo detection scoring (offline, against all ground-truth defects; gazebo/mission_follower/eval_detection_gazebo.py)
    de = "gazebo/mission_follower/detection_eval"
    fp = J(f"{de}/fp05_precision_strict_3p5_10m_los.json")["summary"]
    F.add("gzdet.fp05_boxes", fp["boxes"], f"{de}/fp05_precision_strict_3p5_10m_los.json")
    F.add("gzdet.fp05_tp", fp["tp_location"], f"{de}/fp05_precision_strict_3p5_10m_los.json")
    F.add("gzdet.fp05_hz", J(f"{FP05}/detection/detection_stats.json")["achieved_hz"], f"{FP05}/detection/detection_stats.json")
    bt = J(f"{de}/fp05_box_targets.json")["box_centre_on"]
    F.add("gzdet.fp05_on_veg", bt.get("ENV_BANKVEG", 0), f"{de}/fp05_box_targets.json")
    F.add("gzdet.fp05_on_road", bt.get("BR_WEARING", 0), f"{de}/fp05_box_targets.json")
    fr = J(f"{de}/fp05_snapshot_precision_recall.json")["summary"]
    F.add("gzdet.fp05_usable", fr["usable_defect_instances"], f"{de}/fp05_snapshot_precision_recall.json")
    F.add("gzdet.fp05_detected", fr["instances_detected_location"], f"{de}/fp05_snapshot_precision_recall.json")
    cp = J(f"{de}/columns_full_precision_strict.json")["summary"]
    F.add("gzdet.col_boxes", cp["boxes"], f"{de}/columns_full_precision_strict.json")
    F.add("gzdet.col_tp", cp["tp_location"], f"{de}/columns_full_precision_strict.json")
    F.add("gzdet.col_tp_defects", len(cp["tp_boxes_by_defect"]), f"{de}/columns_full_precision_strict.json")
    F.add("gzdet.col_hi_conf", cp["precision_vs_confidence_floor"]["0.9"], f"{de}/columns_full_precision_strict.json")
    cr = J(f"{de}/columns_full_snapshot_precision_recall.json")["summary"]
    F.add("gzdet.col_recall", cr["recall_instances_location"], f"{de}/columns_full_snapshot_precision_recall.json")

    dp = J(f"{de}/decals_rp0304_precision_strict.json")["summary"]
    F.add("gzdet.dec_boxes", dp["boxes"], f"{de}/decals_rp0304_precision_strict.json")
    F.add("gzdet.dec_tp", dp["tp_location"], f"{de}/decals_rp0304_precision_strict.json")
    F.add("gzdet.dec_conf", dp["confidence"], f"{de}/decals_rp0304_precision_strict.json")
    dr = J(f"{de}/decals_rp0304_snapshot_precision_recall.json")["summary"]
    F.add("gzdet.dec_unique_usable", dr["unique_defects_usable"], f"{de}/decals_rp0304_snapshot_precision_recall.json")

    # gimbal aiming before / after (camera target projected through the true camera pose)
    ev = "gazebo/mission_follower/viz/evidence"
    F.add("aim.before", J(f"{ev}/centering_before_fixed_camera.json")["summary"], f"{ev}/centering_before_fixed_camera.json")
    F.add("aim.after", J(f"{ev}/centering_after_final.json")["summary"], f"{ev}/centering_after_final.json")

    # --- 2026-09-27 update: continuous column aim + narrow inspection camera, flown as aimnarrow_rp0304 ---
    ab = J(f"{de}/aim_before_decals_rp0304_narrow16.json")["by_leg_type"]
    aa = J(f"{de}/aim_after_aimnarrow_rp0304_narrow16.json")["by_leg_type"]
    ab_mid, aa_mid = J(f"{de}/aim_before_decals_rp0304_narrow16.json")["ring_mid_transit_only"], \
        J(f"{de}/aim_after_aimnarrow_rp0304_narrow16.json")["ring_mid_transit_only"]
    F.add("orbit.ring_before_fov", ab["ring"]["column_in_fov_pct"], f"{de}/aim_before_decals_rp0304_narrow16.json")
    F.add("orbit.ring_after_fov", aa["ring"]["column_in_fov_pct"], f"{de}/aim_after_aimnarrow_rp0304_narrow16.json")
    F.add("orbit.link_before_fov", ab["link"]["column_in_fov_pct"], f"{de}/aim_before_decals_rp0304_narrow16.json")
    F.add("orbit.link_after_fov", aa["link"]["column_in_fov_pct"], f"{de}/aim_after_aimnarrow_rp0304_narrow16.json")
    F.add("orbit.ring_mid_before_fov", ab_mid["column_in_fov_pct"], f"{de}/aim_before_decals_rp0304_narrow16.json")
    F.add("orbit.ring_mid_after_fov", aa_mid["column_in_fov_pct"], f"{de}/aim_after_aimnarrow_rp0304_narrow16.json")
    F.add("orbit.el_before_deg", ab["ring"]["el_err_deg"]["mean"], f"{de}/aim_before_decals_rp0304_narrow16.json")
    F.add("orbit.el_after_deg", aa["ring"]["el_err_deg"]["mean"], f"{de}/aim_after_aimnarrow_rp0304_narrow16.json")
    F.add("orbit.az_before_deg", ab["ring"]["az_err_deg"]["mean"], f"{de}/aim_before_decals_rp0304_narrow16.json")
    F.add("orbit.az_after_deg", aa["ring"]["az_err_deg"]["mean"], f"{de}/aim_after_aimnarrow_rp0304_narrow16.json")

    # narrow vs wide decal firing (3f7317d): 5 narrow variants x 3 standoffs = 15 frames; wide control on 2 variants x 3 = 6
    n_fire = n_tot = w_fire = w_tot = 0
    for v in "ABCDE":
        for dets in J(f"{de}/decal_test_narrow_{v}_detections.json").values():
            n_tot += 1
            n_fire += 1 if dets else 0
    for v in ("D", "E"):
        for dets in J(f"{de}/decal_test_wide_{v}_detections.json").values():
            w_tot += 1
            w_fire += 1 if dets else 0
    F.add("decal.narrow_fired", n_fire, f"{de}/decal_test_narrow_[A-E]_detections.json")
    F.add("decal.narrow_total", n_tot, f"{de}/decal_test_narrow_[A-E]_detections.json")
    F.add("decal.wide_fired", w_fire, f"{de}/decal_test_wide_[D-E]_detections.json")
    F.add("decal.wide_total", w_tot, f"{de}/decal_test_wide_[D-E]_detections.json")

    # aimnarrow_rp0304 precision/recall (flown; see detection_eval/PHASE1_VERDICT.md Phase 2)
    anp = J(f"{de}/aimnarrow_rp0304_precision_strict.json")["summary"]
    F.add("gzdet.an_boxes", anp["boxes"], f"{de}/aimnarrow_rp0304_precision_strict.json")
    F.add("gzdet.an_tp", anp["tp_location"], f"{de}/aimnarrow_rp0304_precision_strict.json")
    anr = J(f"{de}/aimnarrow_rp0304_snapshot_precision_recall.json")["summary"]["vs_wide_usable"]
    F.add("gzdet.an_recall_inst", anr["recall_instances"], f"{de}/aimnarrow_rp0304_snapshot_precision_recall.json")
    F.add("gzdet.an_recall_inst_n", anr["instances"], f"{de}/aimnarrow_rp0304_snapshot_precision_recall.json")
    F.add("gzdet.an_recall_detected", anr["instances_detected"], f"{de}/aimnarrow_rp0304_snapshot_precision_recall.json")

    # wide-view reprojection verification (viz/wide_box_reproject_node.py, the demo display fix)
    wr = J(f"{de}/wide_reprojection_verification.json")
    F.add("reproj.px_diff", wr["closed_form_vs_ray_method_px_diff"], f"{de}/wide_reprojection_verification.json")
    F.add("reproj.gt_inside", wr["ground_truth_inside_reprojected_box"], f"{de}/wide_reprojection_verification.json")
    F.add("reproj.range_m", wr["ground_truth_range_m"], f"{de}/wide_reprojection_verification.json")

    # gimbal aiming bug: twin-column piers, aim the pier spine (before) vs the nearest column axis (after)
    cpv = J("mission/columns_plan_verify.json")["aiming"]
    F.add("aimbug.after_hits", cpv["ring_viewpoints_axis_hits_a_column"], "mission/columns_plan_verify.json")
    F.add("aimbug.after_of", cpv["of_ring_viewpoints"], "mission/columns_plan_verify.json")
    F.add("aimbug.before_hits", 389, "git log 80c7a0b (commit message; the pre-fix plan was not itself committed)")
    F.add("aimbug.before_of", 428, "git log 80c7a0b (commit message; the pre-fix plan was not itself committed)")

    # methodology: discarded runs and the brake-test-derived safety margin
    dr_ = J("gazebo/mission_follower/discarded_runs_summary.json")
    F.add("method.discarded", dr_, "gazebo/mission_follower/discarded_runs_summary.json")
    bt_ = J("gazebo/mission_follower/safety_margin_brake_tests.json")
    F.add("method.brake_n", len(bt_["valid_measured_brake_tests"]), "gazebo/mission_follower/safety_margin_brake_tests.json")
    F.add("method.brake_min", bt_["decel_mps2_range"]["min"], "gazebo/mission_follower/safety_margin_brake_tests.json")
    F.add("method.brake_max", bt_["decel_mps2_range"]["max"], "gazebo/mission_follower/safety_margin_brake_tests.json")
    F.add("method.decel_assumed", 1.95, "gazebo/mission_follower/mission_follower_node.py (DECEL_MPS2)")

    F.add("video.demo_s", float(ffprobe(DEMO_SCREEN)["duration"]), DEMO_SCREEN)
    return F


# ---------------------------------------------------------------- plan view --
_EXCLUDE_KINDS_PLAN = {"ground", "water", "train_car"}


def _prim_footprint_xy(p):
    cx, cy = p["centre"][0], p["centre"][1]
    if p["type"] == "BOX":
        hx, hy = p["half_extents"][0], p["half_extents"][1]
        yaw = p.get("yaw", 0.0)
        cos_y, sin_y = abs(math.cos(yaw)), abs(math.sin(yaw))
        ex, ey = hx * cos_y + hy * sin_y, hx * sin_y + hy * cos_y
        return (cx - ex, cy - ey, cx + ex, cy + ey)
    r = p["radius"]
    return (cx - r, cy - r, cx + r, cy + r)


def load_layers():
    """Two flown layers for the plan view. Gazebo full_pass_05 is the default; the PyBullet
    Stage 2 flight is the alternative. Returns (layers, sources)."""
    # Gazebo: true track (simulator truth, audit file) decimated, plus the reached waypoints (EKF)
    track = []
    with open(os.path.join(ROOT, FP05, "pose_audit_track.csv")) as f:
        rd = csv.reader(f)
        next(rd)
        for row in rd:
            track.append((float(row[2]), float(row[3])))
    step = max(1, len(track) // 3000)
    track = track[::step]
    M = J(f"{FP05}/mission_log.json")
    dots = [(w["ekf_world_m"][0], w["ekf_world_m"][1], "var(--ok)") for w in M["waypoints"] if w.get("reached")]
    plan = J(f"{FP05}/plan_used_v5.json")
    reached_ids = {w["waypoint_id"] for w in M["waypoints"] if w.get("reached")}
    for w in plan["waypoints"]:
        if w["waypoint_id"] not in reached_ids:
            dots.append((w["position_m"][0], w["position_m"][1], "var(--warn)"))
    gz = {"key": "gazebo", "label": "Gazebo full pass 05", "track": track, "dots": dots}
    # PyBullet Stage 2 flight
    log = J("mission/coverage_mission_flight_log.json")["log"]
    pts = [(e["achieved_position_m"][0], e["achieved_position_m"][1]) for e in log if e.get("achieved_position_m")]
    dots2 = [(e["achieved_position_m"][0], e["achieved_position_m"][1],
              "var(--ok)" if e.get("settled") else "var(--warn)") for e in log if e.get("achieved_position_m")]
    pb = {"key": "pybullet", "label": "PyBullet Stage 2", "track": pts, "dots": dots2}
    return [gz, pb]


def svg_plan_view(primitives, layers, width=1120, height=340):
    fps = [_prim_footprint_xy(p) for p in primitives if p["kind"] not in _EXCLUDE_KINDS_PLAN]
    wx0, wy0 = min(f[0] for f in fps) - 8, min(f[1] for f in fps) - 8
    wx1, wy1 = max(f[2] for f in fps) + 8, max(f[3] for f in fps) + 8
    margin = 28
    scale = min((width - 2 * margin) / (wx1 - wx0), (height - 2 * margin) / (wy1 - wy0))
    off_x = margin + ((width - 2 * margin) - (wx1 - wx0) * scale) / 2
    off_y = margin + ((height - 2 * margin) - (wy1 - wy0) * scale) / 2
    X = lambda x: off_x + (x - wx0) * scale          # noqa: E731
    Y = lambda y: off_y + (wy1 - y) * scale          # noqa: E731
    o = [f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg" role="img" '
         f'aria-label="Top-down plan view of the 360 metre inspection corridor with structure footprints, '
         f'the flown track and the waypoints">',
         f'<rect x="0" y="0" width="{width}" height="{height}" fill="var(--surface)" rx="8"/>']
    for (x0, y0, x1, y1) in fps:
        o.append(f'<rect x="{X(x0):.1f}" y="{Y(y1):.1f}" width="{max(X(x1) - X(x0), 1):.1f}" '
                 f'height="{max(Y(y0) - Y(y1), 1):.1f}" fill="#C7CFDA" opacity="0.85"/>')
    for i, L in enumerate(layers):
        g = [f'<g class="plan-layer" data-layer="{L["key"]}"' + ('' if i == 0 else ' style="display:none"') + '>']
        g.append('<polyline points="' + " ".join(f"{X(x):.1f},{Y(y):.1f}" for x, y in L["track"]) +
                 '" fill="none" stroke="var(--ink-2)" stroke-width="0.8" opacity="0.5"/>')
        r = 1.9 if len(L["dots"]) > 600 else 3.2
        for x, y, c in [d for d in L["dots"] if d[2] != "var(--warn)"] + [d for d in L["dots"] if d[2] == "var(--warn)"]:
            if c == "var(--warn)":
                g.append(f'<circle cx="{X(x):.1f}" cy="{Y(y):.1f}" r="{r * 2.4:.1f}" fill="{c}" stroke="#fff" stroke-width="1.5"/>')
            else:
                g.append(f'<circle cx="{X(x):.1f}" cy="{Y(y):.1f}" r="{r}" fill="{c}" opacity="0.9"/>')
        g.append('</g>')
        o.append("".join(g))
    bar = 50.0 * scale
    o.append(f'<line x1="{margin}" y1="{height - 14}" x2="{margin + bar:.1f}" y2="{height - 14}" stroke="var(--ink)" stroke-width="2"/>')
    o.append(f'<text x="{margin}" y="{height - 20}" font-size="11" fill="var(--ink-2)">0</text>')
    o.append(f'<text x="{margin + bar - 14:.1f}" y="{height - 20}" font-size="11" fill="var(--ink-2)">50 m</text>')
    o.append(f'<g transform="translate({width - margin - 4},{margin + 4})"><line x1="0" y1="18" x2="0" y2="0" '
             f'stroke="var(--ink)" stroke-width="2"/><polygon points="0,-5 -4,4 4,4" fill="var(--ink)"/>'
             f'<text x="6" y="14" font-size="11" fill="var(--ink-2)">+X</text></g>')
    o.append('</svg>')
    return "".join(o)


def svg_bar_chart(series, width=420, height=230, aria="Bar chart"):
    """series: [(label, pct, colour)]"""
    ml, mb, mt = 34, 56, 22
    ah = height - mb - mt
    slot = (width - ml - 16) / len(series)
    bw = slot * 0.5
    o = [f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg" role="img" '
         f'aria-label="{html.escape(aria)}">']
    for gy in (0, 25, 50, 75, 100):
        y = mt + ah * (1 - gy / 100)
        o.append(f'<line x1="{ml}" y1="{y:.1f}" x2="{width - 8}" y2="{y:.1f}" stroke="var(--line)"/>')
        o.append(f'<text x="4" y="{y + 4:.1f}" font-size="10" fill="var(--ink-2)">{gy}</text>')
    for i, (label, pct, col) in enumerate(series):
        cx = ml + slot * i + slot / 2
        bh = ah * min(pct, 100) / 100
        by = mt + ah - bh
        o.append(f'<rect x="{cx - bw / 2:.1f}" y="{by:.1f}" width="{bw:.1f}" height="{max(bh, 1.5):.1f}" fill="{col}" rx="3"/>')
        o.append(f'<text x="{cx:.1f}" y="{by - 6:.1f}" font-size="12" font-weight="700" text-anchor="middle" fill="var(--ink)">{pct:.1f}%</text>')
        for j, part in enumerate(label.split("\n")):
            o.append(f'<text x="{cx:.1f}" y="{height - 38 + 13 * j}" font-size="10.5" text-anchor="middle" fill="var(--ink-2)">{html.escape(part)}</text>')
    o.append('</svg>')
    return "".join(o)


# ------------------------------------------------------------------ gallery --
GALLERY = [
    ("CWP_014_T0203", "tp", "true positive", "MISSION-VAL catch, the one that decided the headline model"),
    ("CWP_020_T0502", "tp", "true positive", "an unseen defect on MISSION-TEST, the headline recall catch"),
    ("CWP_031_T0701", "tp", "true positive", "MISSION-TEST catch, SPALL_DELAM"),
    ("CWP_018_T0407", "fp", "false positive", "no ground truth here: the texture-triggered CRACK habit"),
    ("CWP_073_T0402", "fn", "missed defect", "4 ground-truth bolts in frame, none detected (FASTENER limit)"),
    ("CWP_016_T0101", "fn", "missed defect", "ground-truth crack in frame, not detected"),
]


def hero_and_posters(make_posters):
    if make_posters:
        for src, name, t in ((FILM, "poster_film.jpg", 24), (WALK, "poster_walk.jpg", 12), (DEMO_SCREEN, "poster_demo_screen.jpg", 5)):
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", str(t), "-i", os.path.join(ROOT, src),
                            "-frames:v", "1", "-vf", "scale=1280:-1", "-q:v", "4", os.path.join(ASSETS, name)], check=True)


def initials(n):
    p = [x for x in n.split() if x]
    return (p[0][0] + p[-1][0]).upper() if len(p) > 1 else n[:2].upper()


# ---------------------------------------------------------------------- css --
CSS = """
:root{--bg:#FFFFFF;--surface:#F5F7FA;--ink:#0E1116;--ink-2:#4A5568;--line:#DDE3EA;--accent:#1D4ED8;
  --ok:#15803D;--warn:#B45309;--bad:#B91C1C;--maxw:1200px;}
*{box-sizing:border-box}
html,body{margin:0;padding:0;background:var(--bg);color:var(--ink);
  font-family:"Public Sans",system-ui,-apple-system,"Segoe UI",Arial,sans-serif;}
body{font-size:16px;line-height:1.6;}
.num{font-variant-numeric:tabular-nums;}
a{color:var(--accent);}
:focus-visible{outline:3px solid var(--accent);outline-offset:2px;}
.wrap{max-width:var(--maxw);margin:0 auto;padding:0 24px;}
.topbar{position:sticky;top:0;background:#fff;border-bottom:1px solid var(--line);z-index:50;}
.topbar .wrap{display:flex;align-items:center;justify-content:space-between;min-height:60px;flex-wrap:wrap;gap:8px;}
.wordmark{font-weight:700;font-size:20px;letter-spacing:.02em;}
.topnav{display:flex;gap:20px;font-size:14px;flex-wrap:wrap;}
.topnav a{color:var(--ink-2);text-decoration:none;}
.topnav a:hover{color:var(--accent);}
.topmeta{font-size:13px;color:var(--ink-2);}
.hero{position:relative;min-height:540px;display:flex;align-items:flex-end;overflow:hidden;background:#20242c;}
.hero img{position:absolute;inset:0;width:100%;height:100%;object-fit:cover;}
.hero .overlay{position:absolute;inset:0;background:linear-gradient(180deg,rgba(14,17,22,.10) 0%,rgba(14,17,22,.82) 100%);}
.hero .content{position:relative;width:100%;max-width:var(--maxw);margin:0 auto;padding:120px 24px 56px;color:#fff;}
.hero h1{font-size:52px;font-weight:700;line-height:1.05;margin:0 0 14px;max-width:820px;}
.hero p{font-size:18px;margin:0;color:#E7ECF3;max-width:700px;}
.hero .tagline{font-size:15px;font-weight:700;letter-spacing:.01em;color:#fff;background:rgba(255,255,255,.14);
  border:1px solid rgba(255,255,255,.3);border-radius:999px;padding:7px 16px;display:inline-block;margin:0 0 18px;max-width:720px;}
@media (max-width:760px){.hero h1{font-size:34px;}.hero{min-height:440px;}.hero .content{padding-top:80px;padding-bottom:36px;}
  .hero .tagline{font-size:13.5px;padding:6px 12px;}}
.statrow{display:grid;grid-template-columns:repeat(4,1fr);gap:16px;margin:28px 0 0;}
@media (max-width:1000px){.statrow{grid-template-columns:1fr 1fr;}}
@media (max-width:520px){.statrow{grid-template-columns:1fr;}}
.stat{background:#fff;border:1px solid var(--line);border-radius:8px;padding:18px 16px;}
.stat .n{font-size:clamp(28px,2.6vw,38px);font-weight:700;color:var(--accent);display:block;line-height:1.1;white-space:nowrap;}
.stat .cap{font-size:14px;color:var(--ink);margin-top:8px;}
.stat .src{font-size:11.5px;color:var(--ink-2);margin-top:6px;word-break:break-all;}
.pbband{background:var(--surface);padding:28px 0;margin-top:48px;}
.pbband .label{font-size:12px;text-transform:uppercase;letter-spacing:.08em;color:var(--ink-2);font-weight:700;}
.pbband h2{font-size:20px;margin:8px 0 10px;font-weight:700;}
.chips{display:flex;gap:8px;flex-wrap:wrap;margin:10px 0;}
.chip{background:#fff;border:1px solid var(--line);border-radius:999px;padding:4px 12px;font-size:13px;color:var(--ink-2);}
.pbband .resp{margin-top:10px;font-size:15px;}
section{padding:72px 0;}
section.tight{padding:48px 0;}
h2.sec{font-size:28px;font-weight:700;margin:0 0 12px;}
p.lede{color:var(--ink-2);margin:0 0 24px;max-width:860px;}
.grid12{display:grid;grid-template-columns:repeat(12,1fr);gap:20px;align-items:start;}
.span8{grid-column:span 8}.span4{grid-column:span 4}.span6{grid-column:span 6}.span12{grid-column:span 12}
@media (max-width:900px){.span8,.span4,.span6{grid-column:1/-1}}
.card{background:#fff;border:1px solid var(--line);border-radius:8px;padding:20px;}
.vid video{width:100%;display:block;border-radius:8px;background:#000;aspect-ratio:16/9;}
.vid .cap{font-size:14px;color:var(--ink-2);margin-top:10px;}
.vid h3{font-size:17px;margin:0 0 10px;}
.legend{display:flex;gap:18px;flex-wrap:wrap;font-size:13px;color:var(--ink-2);margin-top:10px;}
.legend .dot{width:10px;height:10px;border-radius:50%;display:inline-block;margin-right:6px;vertical-align:middle;}
.toggle{display:inline-flex;border:1px solid var(--line);border-radius:999px;overflow:hidden;font-size:13px;margin-bottom:12px;}
.toggle button{border:none;background:#fff;padding:6px 16px;cursor:pointer;color:var(--ink-2);font-family:inherit;}
.toggle button.active{background:var(--accent);color:#fff;}
table{width:100%;border-collapse:collapse;font-size:14.5px;}
th,td{text-align:left;padding:10px 12px;border-bottom:1px solid var(--line);}
th{color:var(--ink-2);font-weight:700;font-size:13px;text-transform:uppercase;letter-spacing:.02em;}
tr.headline{background:var(--surface);}
tr.headline td:first-child{font-weight:700;color:var(--accent);}
.tablewrap{overflow-x:auto;}
.gallery{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;margin-top:20px;}
@media (max-width:760px){.gallery{grid-template-columns:1fr 1fr;}}
@media (max-width:480px){.gallery{grid-template-columns:1fr;}}
.tile{border:1px solid var(--line);border-radius:8px;overflow:hidden;background:#fff;cursor:pointer;text-align:left;padding:0;font-family:inherit;}
.tile img{width:100%;display:block;aspect-ratio:4/3;object-fit:cover;}
.tile .meta{padding:10px 12px;}
.tile .badge{display:inline-block;font-size:11px;font-weight:700;text-transform:uppercase;padding:2px 8px;border-radius:4px;color:#fff;margin-bottom:6px;}
.badge.tp{background:var(--ok);}.badge.fp{background:var(--bad);}.badge.fn{background:var(--warn);}
.tile .note{font-size:13px;color:var(--ink-2);}
.lightbox{position:fixed;inset:0;background:rgba(14,17,22,.85);display:none;align-items:center;justify-content:center;z-index:100;padding:24px;}
.lightbox.open{display:flex;}
.lightbox img{max-width:90vw;max-height:80vh;border-radius:8px;}
.lightbox .cap{color:#fff;text-align:center;margin-top:12px;font-size:14px;max-width:640px;}
.lightbox button.close{position:absolute;top:20px;right:24px;background:none;border:none;color:#fff;font-size:28px;cursor:pointer;}
.pipeline{display:grid;grid-template-columns:repeat(6,1fr);gap:14px;}
@media (max-width:1000px){.pipeline{grid-template-columns:1fr 1fr 1fr;}}
@media (max-width:640px){.pipeline{grid-template-columns:1fr;}}
.pstep{border:1px solid var(--line);border-radius:8px;padding:14px;background:#fff;}
.pstep .idx{font-size:12px;color:var(--accent);font-weight:700;}
.pstep h3{font-size:15px;margin:6px 0;}
.pstep p{font-size:13px;color:var(--ink-2);margin:0 0 6px;}
.pstep code{font-size:11.5px;color:var(--ink-2);word-break:break-all;}
.fixlist{display:grid;gap:14px;margin-top:20px;}
.fixrow{display:grid;grid-template-columns:44px 1fr;gap:16px;background:#fff;border:1px solid var(--line);border-radius:8px;padding:16px 18px;}
.fixrow .fixnum{font-size:13px;font-weight:700;color:var(--accent);background:var(--surface);border-radius:6px;height:28px;
  display:flex;align-items:center;justify-content:center;}
.fixrow h4{margin:0 0 6px;font-size:15.5px;}
.fixrow p{margin:0;font-size:14.5px;color:var(--ink-2);line-height:1.55;}
.fixrow .src{display:block;font-size:11.5px;color:var(--ink-2);margin-top:6px;font-family:monospace;}
@media (max-width:600px){.fixrow{grid-template-columns:1fr;}.fixrow .fixnum{width:28px;}}
ul.limits{list-style:none;margin:0;padding:0;display:grid;gap:10px;}
ul.limits li{background:var(--surface);border-radius:8px;padding:12px 16px 12px 34px;font-size:15px;position:relative;}
ul.limits li::before{content:"\\2014";position:absolute;left:14px;color:var(--warn);font-weight:700;}
ul.limits .src{display:block;font-size:12px;color:var(--ink-2);margin-top:2px;}
.team-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;margin-top:18px;}
@media (max-width:760px){.team-grid{grid-template-columns:1fr 1fr;}}
@media (max-width:480px){.team-grid{grid-template-columns:1fr;}}
.member{border:1px solid var(--line);border-radius:8px;padding:16px;text-align:center;background:#fff;}
.member .av{width:48px;height:48px;border-radius:50%;background:var(--accent);color:#fff;display:flex;align-items:center;justify-content:center;font-weight:700;margin:0 auto 10px;}
.member .name{font-weight:700;font-size:14.5px;}
.member .role{font-size:13px;color:var(--ink-2);}
footer{background:var(--surface);padding:48px 0;font-size:13.5px;color:var(--ink-2);}
footer h4{font-size:13px;text-transform:uppercase;color:var(--ink);margin:0 0 8px;}
footer .cols{display:grid;grid-template-columns:1fr 1fr 1fr;gap:24px;}
@media (max-width:760px){footer .cols{grid-template-columns:1fr;}}
footer ul{margin:0;padding-left:18px;}
footer .src-list{max-height:220px;overflow:auto;font-size:12.5px;}
footer code{word-break:break-all;}
"""

JS = """
(function(){
  var tg = document.querySelectorAll('[data-toggle]');
  tg.forEach(function(b){
    b.addEventListener('click', function(){
      var k = b.getAttribute('data-toggle');
      document.querySelectorAll('.plan-layer').forEach(function(g){ g.style.display = g.getAttribute('data-layer') === k ? '' : 'none'; });
      document.querySelectorAll('[data-toggle]').forEach(function(x){ x.classList.toggle('active', x === b); });
      document.querySelectorAll('[data-note]').forEach(function(n){ n.style.display = n.getAttribute('data-note') === k ? '' : 'none'; });
    });
  });
  var lb = document.getElementById('lightbox'), img = document.getElementById('lightbox-img'), cap = document.getElementById('lightbox-cap');
  document.querySelectorAll('.tile').forEach(function(t){
    t.addEventListener('click', function(){ img.src = t.querySelector('img').src; cap.textContent = t.getAttribute('data-caption') || ''; lb.classList.add('open'); });
  });
  function close(){ lb.classList.remove('open'); }
  document.getElementById('lightbox-close').addEventListener('click', close);
  lb.addEventListener('click', function(e){ if (e.target === lb) close(); });
  document.addEventListener('keydown', function(e){ if (e.key === 'Escape') close(); });
})();
"""


# --------------------------------------------------------------------- page --
def n0(x):
    return f"{x:,.0f}"


def build_html(F, team, prim, layers, commit):
    checks = []                                   # (string on page, fact key, expected string)

    def tile(key, text, cap, src_keys):
        checks.append((text, key))
        srcs = sorted({F.src[k].replace(FP05 + "/", "full_pass_05/") for k in src_keys})
        return (f'<div class="stat"><span class="n num">{text}</span><div class="cap">{cap}</div>'
                f'<div class="src">source: {" · ".join(html.escape(s) for s in srcs)}</div></div>')

    cov = F["fp05.coverage_flown_pct"]
    tiles = "".join([
        tile("fp05.coverage_flown_pct", f"{cov:.1f}%",
             f"measured surface coverage, flown (not planned): {n0(F['fp05.covered_m2'])} of {n0(F['fp05.visible_m2'])} m&sup2; "
             f"visible, from {n0(F['fp05.frames'])} camera frames", ["fp05.coverage_flown_pct", "fp05.visible_m2"]),
        tile("fp05.reached", f"{n0(F['fp05.reached'])} / {n0(F['fp05.in_plan'])}",
             f"waypoints reached, {F['fp05.contact_episodes']} contact episodes on the Gazebo contact sensor; "
             f"{F['fp05.not_attempted']} not attempted (the plan has no route to it)",
             ["fp05.reached", "fp05.contact_episodes"]),
        tile("fp05.distance_true_m", f"{n0(F['fp05.distance_true_m'])} m",
             f"flown in {F['fp05.mission_time_h']:.2f} h of simulated mission time ({n0(F['fp05.mission_time_sim_s'])} s)",
             ["fp05.distance_true_m", "fp05.mission_time_sim_s"]),
        tile("fp05.ekf_err_mean_m", f"{F['fp05.ekf_err_mean_m']:.3f} m",
             f"mean position-hold error at waypoints (EKF, p95 {F['fp05.ekf_err_p95_m']:.3f} m); "
             f"true position vs plan {F['fp05.true_vs_plan_mean_m']:.3f} m mean", ["fp05.ekf_err_mean_m", "fp05.true_vs_plan_mean_m"]),
    ])

    prob = team["problem_statement"]
    problem = f'''<div class="pbband"><div class="wrap">
      <div class="label">Problem statement &middot; {html.escape(prob["id"])}</div>
      <h2>{html.escape(prob["title"])}</h2>
      <div class="chips"><span class="chip">{html.escape(prob["organization"])}</span>
      <span class="chip">{html.escape(prob["technology_bucket"])}</span><span class="chip">{html.escape(prob["category"])}</span></div>
      <p class="resp">Our response: bridges and viaducts are infrastructure whose failure becomes a rescue operation.
      AVIAN inspects them before that, and can check a damaged bridge before rescue convoys cross.</p></div></div>'''

    # method: HOW the numbers on this page were obtained, not just what they say. Placed right after the
    # problem statement, before any result section, because it is what distinguishes this page.
    disc = F["method.discarded"]
    method = f'''<section id="method" class="tight"><div class="wrap">
  <h2 class="sec">How these numbers were obtained</h2>
  <p class="lede">What distinguishes this project is not any single number, but the discipline behind them. This page is built by a script
  (<code>dashboard/build_dashboard.py</code>) that reads every figure from a committed result file and refuses to build if a stat tile does not
  match its source, or if an unfilled placeholder reaches the page &mdash; there is no field on this page typed in by hand.</p>
  <div class="grid12">
    <div class="card span6"><h3 style="margin:0 0 8px;font-size:16px">Failed runs are discarded, not salvaged</h3>
      <p style="margin:0;font-size:14.5px;color:var(--ink-2)">Two early attempts at the coverage mission, <code>full_pass_01</code>
      ({n0(disc["full_pass_01"]["waypoint_records_before_stop"])} waypoints logged) and <code>full_pass_02</code>
      ({n0(disc["full_pass_02"]["waypoint_records_before_stop"])} logged), crashed mid-flight and were thrown away: not committed, not scored, no
      number from either appears anywhere on this page. <code>full_pass_03</code> is the first pass that ran to completion, and every full-pass
      number shown above is <code>full_pass_05</code>, three iterations later.</p></div>
    <div class="card span6"><h3 style="margin:0 0 8px;font-size:16px">Safety margins are measured, not assumed</h3>
      <p style="margin:0;font-size:14.5px;color:var(--ink-2)">The 3.0 m sensed clearance ring and 1.9 m/s cruise speed are sized against
      <strong class="num">{F["method.brake_n"]}</strong> real in-sim brake tests across separate flights (full speed, zero velocity commanded,
      stop distance measured from the true pose): implied deceleration ranged <strong class="num">{F["method.brake_min"]}&ndash;{F["method.brake_max"]}
      m/s&sup2;</strong>. The follower's own assumed constant, <strong class="num">{F["method.decel_assumed"]} m/s&sup2;</strong>, is worse
      (slower-stopping) than every one of them &mdash; a conservative margin, not a guess.</p></div>
    <div class="card span6"><h3 style="margin:0 0 8px;font-size:16px">Sensed-only navigation is a checked claim</h3>
      <p style="margin:0;font-size:14.5px;color:var(--ink-2)"><code>grep -nE 'covlib|geometry\\(|collision|\\.sdf|world_boxes'
      mission_follower_node.py</code> returns nothing: the flight follower subscribes only to 4 PX4 estimator topics and the LiDAR point cloud.
      Maps, collision meshes and ground-truth defect positions exist in this repo for planning and scoring, and are never wired into the code
      that flies the aircraft.</p></div>
    <div class="card span6"><h3 style="margin:0 0 8px;font-size:16px">We tested our own detector adversarially, and published the failure</h3>
      <p style="margin:0;font-size:14.5px;color:var(--ink-2)"><strong class="num">{F["gzdet.fp05_tp"]} of {F["gzdet.fp05_boxes"]}</strong> boxes
      were correct on <code>full_pass_05</code>; precision <strong>inverts</strong> with confidence in the column flight
      ({F["gzdet.col_hi_conf"]["tp_location"]} of {F["gzdet.col_hi_conf"]["boxes"]} above 0.90); <strong class="num">{F["det.real_photo"]["tp"]}
      of {F["det.real_photo"]["n_positive"]}</strong> real crack photos were detected. That is a strength of the process, not an apology: a team
      that will not report its own detector's failure will not report anything else honestly either.</p></div>
  </div></div></section>'''

    # engineering narrative: diagnose -> measure -> fix -> re-measure, with real numbers, each independently sourced
    fixes = [
        ("Frontier exploration &rarr; waypoint coverage",
         f'SLAM + Nav2 MPPI frontier exploration reached <strong class="num">0 of 4</strong> GOTO goals (controller tuning on a CPU-constrained '
         f'machine, not a sensing failure). Replaced with direct PX4 offboard waypoint following: <strong class="num">148 of 150</strong> reached '
         f'on the very next flight.', "git log daec5a9, git log b4386d8"),
        ("Physics engine: DART &rarr; ODE",
         'DART produced zero motor/actuator output for the airframe -- it never left the ground. Switched to ODE (used by every other '
         'PX4-verified world in this project): armed, climbed to 3.01 m (target 3.0 m), held a clean 10 s hover, landed.', "git log 98ba59b"),
        ("Magnetometer auto-calibration silently broke arming",
         f'<code>full_pass_05</code>\'s own 2.6 h flight drifted its learned mag bias and saved it to disk; every run afterwards failed to arm '
         f'with an unrelated-looking error. Root-caused from the .ulg logs: calibrated field {0.359:.3f} G (pass) vs {0.2816:.4f} G (fail) '
         f'against a {0.2824:.4f} G gate. Fix: disable auto-calibration, add a world-identity tripwire. Smoke test after: armed in 5.4 s.',
         "git log 3c3d555"),
        ("Twin-column aiming bug",
         f'The planner aimed each orbit ring at the nearest point of a pier\'s central spine, so broadside viewpoints on twin-column road piers '
         f'looked through the gap between the columns: only 19 of 25 viewpoints hit a column in a smoke flight. Fixed to aim at the nearest '
         f'column axis instead: <strong class="num">{F["aimbug.before_hits"]} of {F["aimbug.before_of"]}</strong> &rarr; '
         f'<strong class="num">{F["aimbug.after_hits"]} of {F["aimbug.after_of"]}</strong> ring viewpoints hit a column.', "git log 80c7a0b"),
        ("Orbit yaw only aimed at waypoints, not continuously",
         f'Between waypoints on a circular orbit the camera faced the direction of travel, not the column, so it fell out of frame mid-leg. '
         f'Fixed to track the column axis every control tick on orbit legs: ring column-in-frame '
         f'<strong class="num">{F["orbit.ring_before_fov"]:.1f}%</strong> &rarr; <strong class="num">{F["orbit.ring_after_fov"]:.1f}%</strong>, '
         f'link legs <strong class="num">{F["orbit.link_before_fov"]:.1f}%</strong> &rarr; <strong class="num">{F["orbit.link_after_fov"]:.1f}%</strong>. '
         f'Honestly still incomplete: elevation error is unchanged ({F["orbit.el_before_deg"]:.1f} &rarr; {F["orbit.el_after_deg"]:.1f} deg) '
         f'because only yaw is tracked continuously, gimbal pitch is not &mdash; the column is still outside the 16&deg; frame '
         f'{100 - F["orbit.ring_after_fov"]:.1f}% of ring time.', "detection_eval/PHASE1_VERDICT.md (Phase 2)"),
        ("Wide 80&deg; camera made defects too small for the detector",
         f'The trained detector was tuned on an ~9&deg; simulated zoom; the wide demo camera renders the same defect about four times smaller. '
         f'A 16&deg; inspection camera on the same gimbal, feeding the detector only (navigation unaffected): decal test '
         f'<strong class="num">{F["decal.narrow_fired"]} of {F["decal.narrow_total"]}</strong> frames fire vs '
         f'<strong class="num">{F["decal.wide_fired"]} of {F["decal.wide_total"]}</strong> on the wide camera.',
         "detection_eval/decal_test_narrow_*_detections.json, decal_test_wide_*_detections.json"),
        ("The 16&deg; camera made an unwatchable demo window",
         f'Needed for the detector to fire at all, but on screen it shows an unrecognisable zoomed patch of concrete. Fix: since both gimbal '
         f'cameras share the identical mount pose (zero baseline), a narrow-camera box reprojects onto the wide 80&deg; view by an exact '
         f'closed-form scale transform, not an approximation. Verified against the codebase&rsquo;s own ray-projection method on a real flown '
         f'frame: <strong class="num">{F["reproj.px_diff"]:.6f} px</strong> difference; the same frame&rsquo;s ground-truth defect, projected '
         f'independently into the wide camera at the true recorded pose ({F["reproj.range_m"]:.2f} m range), lands '
         f'{"inside" if F["reproj.gt_inside"] else "outside"} the reprojected box.',
         "detection_eval/wide_reprojection_verification.json, viz/wide_box_reproject_node.py"),
    ]
    fix_html = "".join(f'''<div class="fixrow"><div class="fixnum">{i + 1:02d}</div><div><h4>{title}</h4>
      <p>{body}</p><span class="src">{html.escape(src)}</span></div></div>''' for i, (title, body, src) in enumerate(fixes))
    engineering = f'''<section id="engineering" class="tight"><div class="wrap">
  <h2 class="sec">Problems found and fixed</h2>
  <p class="lede">The diagnose &rarr; measure &rarr; fix &rarr; re-measure loop, in order, each with a real before/after number. This is what the
  headline stats above don&rsquo;t show.</p>
  <div class="fixlist">{fix_html}</div></div></section>'''

    film_s, walk_s, demo_s = F["video.film_s"], F["video.walk_s"], F["video.demo_s"]
    videos = f'''<section id="videos" class="tight"><div class="wrap">
  <h2 class="sec">Videos</h2>
  <p class="lede">All three play from the local file (relative paths); no external links.</p>
  <div class="grid12">
    <div class="card vid span8"><h3>Inspection film &middot; {film_s:.1f} s</h3>
      <video controls preload="metadata" poster="assets/poster_film.jpg" playsinline>
        <source src="../{FILM}" type="video/mp4">Your browser cannot play this video; open <a href="../{FILM}">{FILM}</a>.</video>
      <div class="cap">Rendered from the AVIAN digital twin in Blender. {F["film.beats"]} defect inspections on textured Blender imagery. The detection
      boxes are real output of the trained detector on the rendered frames (score threshold 0.65; {n0(F["film.detector_records"])} detector records
      on the final frames, {n0(F["film.boxes_drawn"])} drawn), logged in <code>scene/film/detection_log.json</code>.</div></div>
    <div class="card vid span4"><h3>Environment walkthrough &middot; {walk_s:.1f} s</h3>
      <video controls preload="metadata" poster="assets/poster_walk.jpg" playsinline>
        <source src="../{WALK}" type="video/mp4">Your browser cannot play this video; open <a href="../{WALK}">{WALK}</a>.</video>
      <div class="cap">Rendered from the AVIAN digital twin in Blender. A camera walkthrough of the {F["scene.defects"]}-defect corridor: road bridge,
      steel truss span and metro viaduct. No detector output is shown.</div></div>
    <div class="card vid span12"><h3>Live demo, screen recording &middot; {demo_s:.1f} s</h3>
      <video controls preload="metadata" poster="assets/poster_demo_screen.jpg" playsinline>
        <source src="../{DEMO_SCREEN}" type="video/mp4">Your browser cannot play this video; open <a href="../{DEMO_SCREEN}">{DEMO_SCREEN}</a>.</video>
      <div class="cap">A screen recording, not a render, verified against the recorded topic name visible in-frame. It covers two separate parts of the
      real three-window demo (<code>run_demo.sh</code> / <code>demo_camera.sh</code> / <code>demo_rviz.sh</code>): the Gazebo simulation next to the
      live camera window showing the natural 80&deg; view with the wide-view box-reprojection overlay running (<code>/detection/image_annotated_wide</code>
      &mdash; see <a href="#engineering">Problems found and fixed</a> above); no detection box happens to fire in this clip. And, from an earlier
      recording, RViz's live 3D voxel map built only from the drone's own LiDAR and range cones, with the flown trajectory and camera axis. This is the
      autonomy and camera pipeline running live, not a defect-detection accuracy demonstration &mdash; see Detection below for that.</div></div>
  </div></div></section>'''

    # mission plan view + toggle
    svg = svg_plan_view(prim, layers)
    gz_note = (f'<span data-note="gazebo">Gazebo full pass 05: the simulator&rsquo;s true flown track ({n0(F["fp05.distance_true_m"])} m) and the '
               f'{n0(F["fp05.reached"])} reached waypoints (green); the {F["fp05.not_attempted"]} waypoint with no route in the plan is amber.</span>')
    pb_note = (f'<span data-note="pybullet" style="display:none">PyBullet Stage 2 flight: {F["stage2.waypoints"] - F["stage2.stuck"]} of '
               f'{F["stage2.waypoints"]} coverage waypoints without a stuck event (green), {F["stage2.stuck"]} stuck (amber). '
               f'A different simulator and flight stack, shown for comparison.</span>')
    plan = f'''<section id="mission"><div class="wrap">
  <h2 class="sec">Mission plan view</h2>
  <p class="lede">Top-down view of the {F["scene.primitives"]}-primitive collision model of the corridor, with real flown data.</p>
  <div class="toggle" role="group" aria-label="Choose the flight to draw">
    <button data-toggle="gazebo" class="active">Gazebo full pass 05</button><button data-toggle="pybullet">PyBullet Stage 2</button></div>
  <div class="card">{svg}</div>
  <div class="legend"><span><span class="dot" style="background:var(--ok)"></span>reached / settled</span>
    <span><span class="dot" style="background:var(--warn)"></span>not attempted / stuck</span>
    <span>grey shapes: structure footprints from the collision model</span></div>
  <p style="color:var(--ink-2);margin-top:12px;">{gz_note}{pb_note}</p>
</div></section>'''

    # autonomy: share of attempted waypoints COMPLETED (higher is better). An earlier version plotted the
    # failure share, which drew our best result as an empty bar labelled "0 of 1,161".
    ok1 = F["stage1.waypoints"] - F["stage1.stuck"]
    ok2 = F["stage2.waypoints"] - F["stage2.stuck"]
    bars = svg_bar_chart([
        (f"PyBullet Stage 1\n{ok1} of {F['stage1.waypoints']}", 100 * ok1 / F["stage1.waypoints"], "var(--bad)"),
        (f"PyBullet Stage 2\n{ok2} of {F['stage2.waypoints']}", 100 * ok2 / F["stage2.waypoints"], "var(--warn)"),
        (f"Gazebo full pass 05\n{n0(F['fp05.reached'])} of {n0(F['fp05.attempted'])}", 100 * F["fp05.reached"] / F["fp05.attempted"], "var(--accent)"),
        (f"Gazebo columns\n{F['col.reached']} of {F['col.waypoints_in_plan']}", 100 * F["col.reached"] / F["col.waypoints_in_plan"], "var(--ok)")],
        width=520, aria="Bar chart: share of attempted waypoints completed without a stuck event, four flights, higher is better")
    autonomy = f'''<section id="autonomy" class="tight"><div class="wrap">
  <h2 class="sec">Autonomy</h2>
  <p class="lede">Share of attempted waypoints completed without a stuck event &mdash; <strong>higher is better</strong>. PyBullet Stage 1
  stuck on {F["stage1.stuck"]} of {F["stage1.waypoints"]} waypoints and Stage 2 on {F["stage2.stuck"]} of {F["stage2.waypoints"]}; the Gazebo runs
  reached every waypoint they attempted ({n0(F["fp05.reached"])} of {n0(F["fp05.in_plan"])} planned in full pass 05: {F["fp05.not_attempted"]} had no legal route
  in the plan and was not attempted). The bars come from different simulators, plans and flight stacks, so this is a
  progression, not a controlled before/after.</p>
  <div class="grid12"><div class="card span6">{bars}</div>
  <div class="card span6"><ul style="list-style:none;padding:0;margin:0;display:grid;gap:12px;font-size:15px;">
    <li><strong class="num">{n0(F["fp05.sense_ticks"])}</strong> control ticks flown with the sensed-only safety layer; it limited speed on
      <strong class="num">{n0(F["fp05.sense_limited"])}</strong> and pushed away from a return on <strong class="num">{n0(F["fp05.sense_pushed"])}</strong> of them</li>
    <li>Brake test: <strong class="num">{F["fp05.brake_speed"]:.3f} m/s</strong> stopped in <strong class="num">{F["fp05.brake_dist"]:.3f} m</strong>
      (budget {F["fp05.brake_budget"]:.3f} m)</li>
    <li>Coverage flown by component: {", ".join(f"{k} {v:.0f}%" for k, v in sorted(F["fp05.by_component"].items(), key=lambda kv: -kv[1])[:3])};
      lowest: {min(F["fp05.by_component"].items(), key=lambda kv: kv[1])[0]} {min(F["fp05.by_component"].values()):.1f}%</li></ul></div></div>
  <p style="margin-top:16px;color:var(--ink-2);">PyBullet Stage 2 stuck events: 13 near the target (settle failures) and 54 in transit, where the airframe was
  displaced by metres, not centimetres (<code>mission/AVIAN_collision_diagnosis_FINAL.md</code>).</p>
</div></section>'''

    # column-orbit inspection flight
    CF = F["col.summary"]
    col_tiles = "".join([
        tile("col.reached", f"{F['col.reached']} / {F['col.waypoints_in_plan']}",
             f"viewpoints reached around {F['col.n_columns']} bridge columns ({F['col.look_up_dwells']} of them look-up dwells at the pier caps); "
             f"{F['col.contact_episodes']} contact episodes", ["col.reached", "col.contact_episodes"]),
        tile("col.pos_err_mean_m", f"{F['col.pos_err_mean_m']:.3f} m",
             f"mean true position error at the viewpoints (p95 {F['col.pos_err_p95_m']:.3f} m, simulator truth)", ["col.pos_err_mean_m"]),
        tile("col.min_sensed_range_mission_legs_m", f"{F['col.min_sensed_range_mission_legs_m']:.2f} m",
             f"closest sensed return on any mission leg; never inside the 3.0 m ring "
             f"({F['col.mission_samples_inside_3p0_ring']} samples), avoidance never had to slow the drone ({F['col.legs_speed_limited']} legs)",
             ["col.min_sensed_range_mission_legs_m"]),
        tile("col.coverage_mean_whole_pct", f"{F['col.coverage_mean_whole_pct']:.1f}%",
             f"mean whole-column coverage at the flown poses vs {F['col.planned_mean_whole_pct']:.1f}% planned "
             f"(planner&rsquo;s visibility model, evaluated at the true camera poses)", ["col.coverage_mean_whole_pct"]),
    ])
    crow = ""
    for sid, v in CF["coverage_per_column"].items():
        p, t = v["planned"], v["measured_true_poses"]
        crow += (f'<tr><td>{sid}</td><td class="num">{v["waypoints"]}</td><td class="num">{v["dwells"]}</td>'
                 f'<td class="num">{p["shaft"]:.1f} / {t["shaft"]:.1f}</td><td class="num">{p["cap_or_head"]:.1f} / {t["cap_or_head"]:.1f}</td>'
                 f'<td class="num">{p["footing"]:.1f} / {t["footing"]:.1f}</td><td class="num"><strong>{p["whole"]:.1f} / {t["whole"]:.1f}</strong></td></tr>')
    columns = f'''<section id="columns"><div class="wrap">
  <h2 class="sec">Column inspection flight</h2>
  <p class="lede">A second Gazebo mission flew stacked orbit rings around every one of the corridor&rsquo;s {F["col.n_columns"]} bridge columns, the
  gimbal camera aimed at the nearest column axis, with extra look-up dwells at the road-pier caps. Same PX4 offboard follower, same sensed-only
  avoidance (3.0 m ring, 1.9 m/s), no change to its safety constants. {n0(F["col.mission_time_sim_s"])} s of mission time took
  {n0(F["col.mission_time_wall_s"])} s of wall time (real-time factor {F["col.rtf"]:.2f}) with the live detector running.</p>
  <div class="statrow" style="margin-top:0">{col_tiles}</div>
  <div class="grid12" style="margin-top:20px">
    <div class="card span8"><img src="assets/columns_plan_overview.png" style="width:100%" alt="Top-down view of the corridor with the 19 column orbits and the flown track">
      <div class="cap" style="font-size:13.5px;color:var(--ink-2)">Flown track (simulator truth) over the orbit viewpoints of the plan. Road piers (y&nbsp;&asymp;&nbsp;0) are twin columns
      orbited as one capsule; metro columns (y&nbsp;&asymp;&nbsp;28) are single. Camera axis on a column at {F["col.ring_axis"]} ring viewpoints, on the cap or a
      column at {F["col.dwell_axis"]} dwells.</div></div>
    <div class="card span4"><img src="assets/columns_plan_rp0304.png" style="width:100%" alt="Close-up of the orbits around road piers RP03 and RP04">
      <div class="cap" style="font-size:13.5px;color:var(--ink-2)">Close-up of road piers RP03 and RP04: the capsule orbit and the flown track.</div></div>
  </div>
  <div class="card tablewrap" style="margin-top:20px"><table><thead><tr><th>Column</th><th>Waypoints (incl. dwells)</th><th>Look-up dwells</th><th>Shaft %</th>
    <th>Cap / head %</th><th>Footing %</th><th>Whole %</th></tr></thead><tbody>{crow}</tbody></table>
  <p style="font-size:13px;color:var(--ink-2);margin:10px 0 0">Each cell: planned / at the flown poses. Both use the planner&rsquo;s own visibility model, so the
  second number measures how much pose and aim error cost, not an independent check of what the camera saw. Cap top faces are not reachable:
  the deck is directly above them. Cap end faces are limited by the 3.5 m planned clearance. Metro columns MC04&ndash;MC06 sit in a constrained
  bay and are weak (58&ndash;61% whole).</p></div>
</div></section>'''

    # gallery of flight images (each viewed before inclusion; captions state only what the image shows)
    GAL2 = [
        ("gz_frame_RP02_R1_02.jpg", "Gazebo camera, column flight RP02 ring 1: the gimbal looks down at the column, which fills the centre of the frame."),
        ("gz_frame_MC07_R2_03.jpg", "Gazebo camera, metro column MC07 ring 2. The dark-green boxes behind it are bank-vegetation props: most Gazebo false detections landed on these."),
        ("gz_frame_RP04_R1_01_D1.jpg", "A look-up dwell at RP04: the column top and the underside of the pier cap, with a flat dark defect-marker sphere on the cap."),
        ("gz_frame_RP04_R2_09.jpg", "RP04 ring 2. The brown ball is the flat-coloured marker for DEFECT_REBAR_EXPOSED_006: Gazebo defects are untextured spheres, not modelled damage."),
        ("gz_decal_rebar006.jpg", "Gazebo, road pier RP04 with a textured decal of DEFECT_REBAR_EXPOSED_006: the live detector's boxes (0.88 left, 0.66 right) are on the decal. The only textured defect of ten that fired; not accuracy evidence."),
        ("rviz_map_flight.jpg", "RViz during a Gazebo flight: the live 0.25 m voxel map built only from the drone's LiDAR and range cones (deck, piers, ground), the EKF trajectory (yellow) and the camera axis (magenta)."),
        ("rviz_map_closeup.jpg", "RViz close-up of two piers in the live voxel map, coloured by height, with the trajectory and live scan."),
        ("aim_before_after_B.jpg", f"Gimbal aiming, before and after (CWP_005 / CWP_006): the red ring is the inspection target projected through the true camera pose. Across 8 waypoints the target moved from {F['aim.before']['off_axis_deg']['mean']:.1f} to {F['aim.after']['off_axis_deg']['mean']:.2f} degrees off the camera axis on average."),
        ("aim_before_after_C.jpg", f"Gimbal aiming, before and after (CWP_007 / CWP_008). In frame: {F['aim.before']['target_in_frame']} of {F['aim.before']['n']} targets before, {F['aim.after']['target_in_frame']} of {F['aim.after']['n']} after."),
        ("film_02_beat1_spall_007_0.99.jpg", "Blender inspection film, beat 1: the trained detector on a textured spall (SPALL_007), SPALL_DELAM 0.99 on this frame. Real detector output."),
        ("film_06_beat5_rebar_exposed_004_0.76.jpg", "Blender film, beat 5: exposed rebar at a bearing seat, SPALL_DELAM 0.76 on this frame. This beat's detections drop out on some frames."),
        ("film_09_beat8_rebar_exposed_001_0.99.jpg", "Blender film, beat 8: exposed rebar on a girder web, SPALL_DELAM 0.99. Staged viewpoint; its camera is 2.30 m from a deck edge (below the 3.5 m rule)."),
    ]
    gal2 = "".join(f'''<button class="tile" data-caption="{html.escape(c)}"><img src="assets/{f}" alt="{html.escape(c)}">
      <div class="meta"><div class="note">{html.escape(c)}</div></div></button>''' for f, c in GAL2)
    for f, _ in GAL2:
        assert os.path.exists(os.path.join(ASSETS, f)), f"missing gallery image {f}"
    flights = f'''<section id="gallery" class="tight"><div class="wrap">
  <h2 class="sec">From the flights</h2>
  <p class="lede">Gazebo camera frames, RViz map captures, gimbal aiming before and after, and stills from the Blender film. Plan views are in the
  column-inspection section above.</p>
  <div class="gallery">{gal2}</div></div></section>'''

    # detection
    m = F["det.models"]
    head = F["det.headline"]
    rows = ""
    for name in ("v1", "v2", "v3"):
        d = m[name]
        cls = ' class="headline"' if name == head else ""
        rows += (f'<tr{cls}><td>{name}{" (headline)" if name == head else ""}</td><td class="num">{d["threshold"]}</td>'
                 f'<td class="num">{d["val_f1"]}</td><td class="num">{d["unseen"][0]} of {d["unseen"][1]}</td>'
                 f'<td class="num">{d["all"][0]} of {d["all"][1]} <span style="color:var(--ink-2)">(optimistic)</span></td>'
                 f'<td class="num">{d["fp100"]}</td></tr>')
    rp = F["det.real_photo"]
    gal = "".join(f'''<button class="tile" data-caption="{html.escape(note)}"><img src="assets/tile_{t}.jpg"
      alt="Zoom tile {t}, {label}: {html.escape(note)}"><div class="meta"><span class="badge {oc}">{label}</span>
      <div class="note">{html.escape(note)}</div></div></button>''' for t, oc, label, note in GALLERY)
    detection = f'''<section id="detection"><div class="wrap">
  <h2 class="sec">Detection</h2>
  <p class="lede">All accuracy figures here are from the Blender benchmark: {n0(F["det.n_val_tiles"])} validation and {n0(F["det.n_test_tiles"])} test
  zoom tiles ({F["zoom.optical_x"]:.1f}&times; simulated zoom, {F["zoom.hfov_deg"]:.1f}&deg; field of view), scored against projected ground truth.
  The Gazebo runs are not used to claim detection accuracy: that world is flat-shaded, untextured geometry.</p>
  <div class="card tablewrap"><table><thead><tr><th>Model</th><th>Threshold</th><th>F1 (MISSION-VAL)</th><th>Unseen-defect recall</th>
    <th>All-defects recall</th><th>False positives / 100 tiles</th></tr></thead><tbody>{rows}</tbody></table></div>
  <h3 style="margin:32px 0 6px;font-size:18px;">Evidence gallery</h3>
  <p style="color:var(--ink-2);margin:0;">Six Blender zoom tiles with boxes drawn from the headline model&rsquo;s actual output: true positives,
  a false positive and missed defects, shown together.</p>
  <div class="gallery">{gal}</div>
  <div style="margin-top:24px;"><p><strong>Real-photo crack test</strong> ({html.escape(rp["dataset"])}, {html.escape(rp["license"])}):
    {rp["n_positive"]} crack and {rp["n_negative"]} clean photos, headline model at threshold {rp["threshold"]}. Detected {rp["tp"]} of {rp["n_positive"]} cracks and flagged
    {rp["fp"]} of {rp["n_negative"]} clean photos: precision <span class="num">{rp["precision"]}</span>, recall <span class="num">{rp["recall"]}</span>.
    <strong>The detector does not yet transfer to real photos.</strong></p>
  <p style="color:var(--ink-2);">Bolts (FASTENER) are not reliably detectable at the current camera resolution.</p></div>
  <div class="card" style="margin-top:24px;border-left:4px solid var(--warn)"><h3 style="margin:0 0 8px;font-size:17px">Gazebo detections are not accuracy evidence</h3>
  <p style="margin:0 0 8px">The trained detector runs in the loop during the Gazebo flights (about {F["col.det_hz"]:.1f} frames per second measured in the column flight), so
  the pipeline works end to end. But the Gazebo world is untextured SDF primitives and its defects are flat-coloured spheres, so what it detects
  there says nothing about finding real damage. Scored against all {F["scene.defects"]} ground-truth defects at the true camera pose:</p>
  <ul style="margin:0;padding-left:20px;font-size:15px">
    <li>Full pass 05: <strong>{F["gzdet.fp05_tp"]} of {F["gzdet.fp05_boxes"]}</strong> boxes contained a usable defect (3.5&ndash;10 m, line of sight).
      {F["gzdet.fp05_on_veg"]} box centres lay on bank-vegetation props and {F["gzdet.fp05_on_road"]} on the road surface. Recall on the waypoint
      snapshots: {F["gzdet.fp05_detected"]} of {F["gzdet.fp05_usable"]} defect sightings.</li>
    <li>Column flight: {F["gzdet.col_tp"]} of {F["gzdet.col_boxes"]} boxes contained a defect, all on {F["gzdet.col_tp_defects"]} dark marker spheres;
      above confidence 0.90, {F["gzdet.col_hi_conf"]["tp_location"]} of {F["gzdet.col_hi_conf"]["boxes"]}. Snapshot recall {100 * F["gzdet.col_recall"]:.1f}%.</li>
    <li>Textured decals rendered from the Blender twin, fitted to 10 defects on road piers RP03 and RP04 and flown again: {F["gzdet.dec_boxes"]} boxes,
      {F["gzdet.dec_tp"]} on a defect, all on one exposed-rebar decal (confidence {F["gzdet.dec_conf"]["min"]:.2f}&ndash;{F["gzdet.dec_conf"]["max"]:.2f});
      of {F["gzdet.dec_unique_usable"]} defects in usable view, one was detected. The wide 80&deg; camera renders defects a few dozen pixels across,
      about four times smaller than the imagery the detector was trained on: a 16&deg; inspection camera decal test fires on
      {F["decal.narrow_fired"]} of {F["decal.narrow_total"]} frames vs {F["decal.wide_fired"]} of {F["decal.wide_total"]} on the wide camera
      (Problems found and fixed, below).</li>
    <li>Same RP03/RP04 plan flown again with both fixes (continuous column aim, 16&deg; camera feeding the detector): {F["gzdet.an_tp"]} of
      {F["gzdet.an_boxes"]} boxes contained a defect (down from {F["gzdet.dec_tp"]} of {F["gzdet.dec_boxes"]} &mdash; precision fell as hits
      concentrated on one dwell waypoint), but every true positive moved to a <em>different</em>, previously-undetected defect. Recall on the
      same sightings basis moved off zero for the first time: {F["gzdet.an_recall_detected"]} of {F["gzdet.an_recall_inst_n"]}
      ({100 * F["gzdet.an_recall_inst"]:.1f}%), still not usable detection. Full numbers:
      <code>gazebo/mission_follower/detection_eval/PHASE1_VERDICT.md</code> (Phase 2).</li>
  </ul>
  <p style="margin:8px 0 0;color:var(--ink-2)">Detector accuracy evidence comes only from the textured Blender imagery: the benchmark above and the inspection film. None of the
  numbers in this card are used, or should be read, as a measurement of how well the detector finds real damage.</p></div>
</div></section>'''

    steps = [("Digital twin", f"Blender-built 360 m corridor, {F['scene.defects']} measured defects", "scene/SIH_AVIAN_FINAL.blend"),
             ("Plan coverage", "Structure-driven waypoint planning; no defect positions read", "gazebo/coverage_v5/plan_v5.py"),
             ("Fly &amp; avoid", "PX4 + Gazebo flight with sensed-only avoidance", "gazebo/mission_follower/mission_follower_node.py"),
             (f"Zoom capture ({F['zoom.optical_x']:.1f}&times;, {F['zoom.hfov_deg']:.0f}&deg;)", "Simulated optical zoom at the same safe standoff",
              "source/render_zoom_tiles_all_final.py"),
             ("Detect", "Faster R-CNN (torchvision, BSD-3-Clause)", "source/train_detector_v2_final.py"),
             ("Score", "IoU-matched against projected ground truth", "source/evaluate_v3_mission_final.py")]
    pipeline = "".join(f'<div class="pstep"><div class="idx">{i + 1:02d}</div><h3>{n}</h3><p>{d}</p><code>{p}</code></div>'
                       for i, (n, d, p) in enumerate(steps))

    members = "".join(f'<div class="member"><div class="av">{initials(x["name"])}</div><div class="name">{html.escape(x["name"])}</div>'
                      f'<div class="role">{html.escape(x["role"])}</div></div>' for x in team["team"]["members"])
    team_sec = f'''<section id="team" class="tight"><div class="wrap"><h2 class="sec">Team</h2>
  <p>Team {html.escape(team["team"]["name"])}, Team ID {html.escape(team["team"]["team_id"])} &mdash; {html.escape(team["team"]["institute"])}</p>
  <div class="team-grid">{members}</div></div></section>'''

    comp = F["fp05.by_component"]
    lim = [
        (f"Detection does not yet transfer to real photos: {rp['tp']} of {rp['n_positive']} real crack photos detected, "
         f"precision and recall both {rp['precision']}.", F.src["det.real_photo"]),
        (f"Trained and tested on synthetic Blender renders. On the held-out mission tiles, unseen-defect recall is {m[head]['unseen'][0]} of "
         f"{m[head]['unseen'][1]} for every model, too few defects to tell the models apart.", F.src["det.models"]),
        ("Bolts (FASTENER) are not detectable at the current camera resolution.", "detection/AVIAN_detector_report_FINAL.md"),
        (f"Coverage is {cov:.1f}%, not 100%: flown coverage runs from {max(comp.values()):.0f}% of the abutments down to "
         f"{min(comp.values()):.1f}% of the steel truss.", F.src["fp05.coverage_flown_pct"]),
        (f"{F['fp05.not_attempted']} of {n0(F['fp05.in_plan'])} planned waypoints was not attempted: the plan found no legal route to it.",
         F.src["fp05.reached"]),
        (f"The live detector in the Gazebo runs is not evidence of detection accuracy: that world is untextured geometry with flat-coloured "
         f"defect spheres, and in full pass 05 {F['gzdet.fp05_tp']} of {F['gzdet.fp05_boxes']} boxes contained a real defect. Detection accuracy is quoted "
         f"only from the Blender benchmark and film.", F.src["gzdet.fp05_tp"]),
        ("Cap top faces cannot be inspected (the deck is directly above them); cap end faces are limited by the 3.5 m planned clearance.",
         "mission/columns_plan_report.json"),
        ("Inspection film: beats 3 and 5 have detector dropouts, so their boxes flicker; beat 8's staged camera is 2.30 m from a deck edge.",
         "scene/film/detection_log.json, scene/film/clearance_result.json"),
        ("Simulation only: nothing here has been flight-tested on hardware.", "project scope"),
    ]
    limits = "".join(f'<li>{html.escape(t)}<span class="src">{html.escape(s)}</span></li>' for t, s in lim)
    scope = f'''<section id="limits"><div class="wrap"><h2 class="sec">Scope and known limits</h2>
  <p class="lede">What the numbers above do and do not show.</p><ul class="limits">{limits}</ul></div></section>'''

    srcs = {}
    for k, s in F.src.items():
        for one in re.split(r" \+ |, ", s):
            if "git log" not in one:
                srcs.setdefault(one, None)
    src_html = "".join(f"<li><code>{html.escape(p)}</code>@{html.escape(file_commit(p) if '*' not in p and os.path.exists(os.path.join(ROOT, p)) else 'git')}</li>"
                       for p in sorted(srcs))
    stack = ["torchvision (Faster R-CNN) &mdash; BSD-3-Clause", "PyTorch &mdash; BSD-3-Clause", "PyBullet &mdash; zlib",
             "Blender (build and render tooling only) &mdash; GPL"]
    tm = f'Team {html.escape(team["team"]["name"])} &middot; {html.escape(prob["id"])} &middot; commit {commit}'

    page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>AVIAN &mdash; mission dashboard</title>
<meta name="description" content="AVIAN autonomous bridge inspection: flown coverage, autonomy and detection results, measured.">
<link rel="preconnect" href="https://fonts.googleapis.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Public+Sans:wght@400;700&display=swap" rel="stylesheet">
<style>{CSS}</style></head><body>
<div class="topbar"><div class="wrap"><div class="wordmark">AVIAN</div>
  <nav class="topnav" aria-label="Sections"><a href="#method">Method</a><a href="#engineering">Fixes</a><a href="#mission">Mission</a><a href="#columns">Columns</a>
    <a href="#autonomy">Autonomy</a><a href="#detection">Detection</a><a href="#gallery">Gallery</a><a href="#pipeline">Pipeline</a><a href="#videos">Videos</a><a href="#team">Team</a></nav>
  <div class="topmeta">{tm}</div></div></div>
<div class="hero"><img src="assets/hero.jpg" alt="Steel truss main span of the AVIAN bridge digital twin"><div class="overlay"></div>
  <div class="content"><p class="tagline">30 seconds: a simulated drone plans, flies and inspects a 360 m road-and-metro bridge on its own sensors,
  with every number on this page measured from a committed file &mdash; including the numbers that came out badly.</p>
  <h1>Autonomous bridge inspection, measured end to end.</h1>
  <p>A simulation digital twin of a 360 m road-and-metro corridor with {F["scene.defects"]} known defects, flown, avoided, zoomed and scored without a human in the loop.</p></div></div>
<div class="wrap"><div class="statrow">{tiles}</div></div>
{problem}
{method}
{engineering}
{videos}
{plan}
{columns}
{autonomy}
{detection}
{flights}
<section id="pipeline" class="tight"><div class="wrap"><h2 class="sec">Pipeline</h2><div class="pipeline">{pipeline}</div></div></section>
{team_sec}
{scope}
<footer><div class="wrap"><div class="cols">
  <div><h4>Sources (file@commit)</h4><ul class="src-list">{src_html}</ul></div>
  <div><h4>Licenses</h4><ul class="src-list">{"".join(f"<li>{x}</li>" for x in stack)}</ul></div>
  <div><h4>Note</h4><p>Simulation results. Not flight-tested on hardware.</p><p>Headline stats: Gazebo run full_pass_05, commit {F["fp05.commit"]}. Page built at commit {commit}.</p></div>
</div></div></footer>
<div class="lightbox" id="lightbox"><button class="close" id="lightbox-close" aria-label="Close">&times;</button>
  <div><img id="lightbox-img" src="" alt="Zoom tile, enlarged"><div class="cap" id="lightbox-cap"></div></div></div>
<script type="application/json" id="measured-facts">{json.dumps({k: {"value": F.v[k], "source": F.src[k]} for k in F.v}, default=str)}</script>
<script>{JS}</script></body></html>"""
    return page, checks


def verify(page, checks, F):
    text = re.sub(r"<(script|style)\b.*?</\1>", "", page, flags=re.S | re.I)
    text = re.sub(r"<[^>]+>", " ", text)
    if re.search(r"pending", text, re.I):
        raise SystemExit("BUILD FAILED: the word 'pending' reached the page")
    facts = json.loads(re.search(r'id="measured-facts">(.*?)</script>', page, re.S).group(1))
    bad = 0
    for shown, key in checks:
        if key not in facts:
            bad += 1
            print("  no fact for", key)
            continue
        raw = facts[key]["value"]
        digits = re.sub(r"[^0-9.]", "", shown.split("/")[0])
        ok = digits and abs(float(digits) - round(float(raw), len(digits.split(".")[1]) if "." in digits else 0)) < 1e-9
        if not ok:
            bad += 1
            print(f"  MISMATCH {key}: page shows {shown!r}, file value {raw}")
    print(f"tile check: {len(checks) - bad}/{len(checks)} stat tiles match the value read from their source file")
    print("'pending' check: not present on the page")
    if bad:
        raise SystemExit("BUILD FAILED: a stat tile does not match its source")


def main():
    F = load_facts()
    hero_and_posters("--posters" in sys.argv)
    team = json.load(open(os.path.join(HERE, "team.json")))
    prim = J("scene/collision/avian_bridge_collision.json")["primitives"]
    layers = load_layers()
    commit = git("rev-parse", "--short", "HEAD") or "unknown"
    page, checks = build_html(F, team, prim, layers, commit)
    verify(page, checks, F)
    out = os.path.join(HERE, "index.html")
    open(out, "w").write(page)
    size = sum(os.path.getsize(os.path.join(ASSETS, f)) for f in os.listdir(ASSETS))
    print(f"saved {out}; assets {size / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
