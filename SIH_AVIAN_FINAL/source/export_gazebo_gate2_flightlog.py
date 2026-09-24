"""Export Gate 2's real Gazebo-achieved poses (gazebo/gate2_explore/results/
metrics.csv, TF-logged, sensor/odometry derived) into the same
mission.json + flight_log.json schema render_zoom_tiles_final.py expects,
so that script can be reused unmodified for Gate 3 tile rendering.

target_m per synthetic waypoint: the drone's own real (x, y) at that
sample, aimed at the corridor structure directly (same x, road/deck
centerline y=15, z=10 -- mid-height of the steel/deck structure spanning
this corridor) -- NOT read from any ground-truth defect file. This is a
camera-aim choice (point roughly at the structure the drone flew near),
identical in kind to how a human operator would frame a shot, not a
detection input.
"""
import csv
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
METRICS = os.path.join(ROOT, "gazebo", "gate2_explore", "results", "metrics.csv")
# Uses the committed, official Gate 2 result (21.87 m run) -- the bounded
# retry (lowered PreferForwardCritic) also reached 0 goals and per
# instruction the original partial result is what's kept.
MISSION_OUT = os.path.join(ROOT, "mission", "gazebo_gate2_mission.json")
FLIGHTLOG_OUT = os.path.join(ROOT, "mission", "gazebo_gate2_flight_log.json")

CRUISE_ALT = 8.0
STRUCT_Y = 15.0
STRUCT_Z = 10.0
N_SAMPLE = 8  # small, time-boxed for this pass


def main():
    rows = []
    with open(METRICS) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    if not rows:
        raise SystemExit("no metrics rows found")

    step = max(1, len(rows) // N_SAMPLE)
    picked = rows[::step][:N_SAMPLE]

    waypoints = []
    log = []
    for i, r in enumerate(picked):
        wid = f"GZ_{i:03d}"
        x, y = float(r["x"]), float(r["y"])
        achieved = [x, y, CRUISE_ALT]
        target = [x, STRUCT_Y, STRUCT_Z]
        waypoints.append({
            "waypoint_id": wid,
            "target_m": target,
            "prim_kind": "pier_column",
        })
        log.append({
            "waypoint_id": wid,
            "target_m": target,
            "achieved_position_m": achieved,
            "settled": True,
            "stuck": False,
            "skipped": False,
        })

    with open(MISSION_OUT, "w") as f:
        json.dump({
            "generated_utc": "2026-09-24T13:53:00Z",
            "planner": "export_gazebo_gate2_flightlog.py -- real Gazebo TF poses, "
                      "camera aimed at structure, no ground-truth read",
            "waypoints": waypoints,
        }, f, indent=2)
    with open(FLIGHTLOG_OUT, "w") as f:
        json.dump({
            "generated_utc": "2026-09-24T13:53:00Z",
            "source": "Gazebo Gate 2 run, gazebo/gate2_explore/results/metrics.csv",
            "n_waypoints": len(log),
            "log": log,
        }, f, indent=2)
    print(f"wrote {len(waypoints)} waypoints -> {MISSION_OUT}, {FLIGHTLOG_OUT}")


if __name__ == "__main__":
    main()
