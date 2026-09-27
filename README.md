# AVIAN — Autonomous Bridge-Inspection Drone

**Smart India Hackathon 2026 · Problem Statement SIH26201**
Team TRINETRA, Charotar University of Science & Technology, Anand

## Overview

AVIAN is a simulated autonomous drone system for inspecting a 360 m road-and-metro
bridge. It plans full-surface coverage, flies the plan with sensed-only obstacle
avoidance (no maps, no SLAM, no pre-built collision geometry feeding navigation),
and runs a trained crack detector on its camera feed.

Two simulators are used, each proving a different part of the system:

| Simulator | Proves |
|---|---|
| **Gazebo** (PX4 SITL + ROS 2 Jazzy) | Flight, autonomy, obstacle avoidance, coverage, 3D mapping |
| **Blender** | Photoreal digital twin, ~190 modelled defects, cinematic film |

## Architecture

- **`mission_follower_node.py`** — flies plans via direct PX4 OFFBOARD velocity
  setpoints. Not Nav2, not SLAM. Avoidance is sensed-only, from LiDAR and up/down
  range cones.
- **Safety constants:** 1.9 m/s cruise, 3.0 m sensed avoidance ring, 3.5 m planned
  clearance, flat (≤12°) or vertical route legs only. Cruise speed is derived from
  6 measured brake tests (2.07× margin).
- **Sensors:** 360°×15-channel LiDAR (±14°), up/down range cones (9×9 rays, ±45°),
  gimbal camera. A 14–45° elevation blind band is why routes are flat or vertical,
  never diagonal.
- **Cameras:** 80° wide camera (demo view, both feeds recorded) and 16° narrow
  inspection camera (feeds the detector).
- **`pose_audit_node.py`** — reads simulator ground truth for logging only; never
  publishes to ROS or reaches navigation.

## Measured Results

All figures below are measured and traceable to result files in this repo — no
figure is estimated.

**Full-coverage mission** (`full_pass_05`, commit `68d71d6`):
- 1161/1162 waypoints reached, 0 contacts
- 81.10% measured surface coverage
- 9050 m flown, 2.60 h
- EKF hold error: 0.099 m mean

**Column orbital inspection** (460 waypoints, 19 columns):
- 460/460 reached, 0 contacts
- Position error: 0.122 m mean
- Measured coverage 80.2% vs. 80.1% planned
- Column-in-frame: 427/428 ring viewpoints

**Orbit aiming fix** (continuous yaw tracking, replacing waypoint-only yaw):
- Column-in-frame, ring legs: 62.1% → 75.3%
- Column-in-frame, link legs: 35.9% → 96.7%

## Detection — Honest Status

The crack detector is the weak subsystem and is reported as such, per project
policy of never overstating a measured result.

- `full_pass_05`: 0/87 detection boxes were real (77 on bank vegetation, 10 on
  road surface). Recall: 0/444 defect sightings.
- Column flight (wide camera): 61/233 boxes landed on a defect, but on only 2
  distinct defects. Recall: 1.9%.
- Precision **inverts** with confidence: 0/56 boxes above 0.90 confidence were
  real defects.
- Synthetic benchmark: 5/20 defects detected. On 500 real crack photographs: 0.
- Root cause identified: the 80° wide camera made defects ~4× smaller than the
  detector's training imagery. A 16° narrow inspection camera fixed the effect
  in isolated testing (14/15 frames fire at 0.94–0.997 confidence vs. 1/15 on
  wide), but in-flight the improvement was partial — recall rose 0% → 3.6%, and
  the false-positive rate fell 97% → 82% under confidence filtering.

**What Gazebo demonstrates:** autonomy, obstacle avoidance, coverage planning,
and camera aiming. Detection accuracy is not yet solved and is an open,
documented gap — not a claimed capability.

## Known Limitations

- Bridge cap top faces (16–28% of cap area) are physically unreachable from
  below the deck.
- Cap end faces (~8%) require clearance below the 3.5 m safety minimum;
  deliberately not chased.
- Metro columns MC04–06: 72–77% shaft coverage.
- Detections shown in Gazebo footage are not accuracy evidence — see Detection
  section above.
- The entire system is simulation-only. No hardware or field validation.

## Running the Demo

From `SIH_AVIAN_FINAL/`, three terminals (terminals 2 and 3 start ~35 s after
terminal 1 prints `phase WAIT -> ARM`):

```bash
gazebo/mission_follower/run_demo.sh --minutes 7 --detect-hz 2 --map-hz 0.5
gazebo/mission_follower/demo_camera.sh
gazebo/mission_follower/demo_rviz.sh
```

Runs at ~0.31× real time (GPU-bound). A 7-minute mission takes ~14 minutes
wall-clock. Camera feeds display in `rqt_image_view` — RViz Image displays
crash RViz on the reference machine.

## Media

- `SIH_AVIAN_FINAL/media/avian_inspection_film.mp4` — 82.6 s cinematic film,
  8 defect beats, real detector boxes composited on textured Blender frames.
- `SIH_AVIAN_FINAL/media/bridge_defect_walkthrough_cinematic.mp4` — 47.9 s
  environment walkthrough.
- Full demo recording (Google Drive): https://drive.google.com/file/d/10TSNNnv58uuF3M4rD5IqXag2SsFtw8x6/view?usp=drive_link
- `dashboard/index.html` — results dashboard, all figures sourced from result
  files (`build_dashboard.py`).

### Gallery

See `docs/media/gallery/` for extracted frames from both videos, and
`docs/media/live_demo/` for live Gazebo/RViz screenshots.

## Key Documents

- `PROTOTYPE_REPORT.md` — consolidated project status, including an explicit
  "what we can and cannot claim" section.
- `detection_eval/PHASE1_VERDICT.md` — full detection evaluation and numbers.

## What's Left

1. Continuous gimbal pitch tracking (companion fix to the yaw-tracking change
   above) — diagnosed, not yet implemented.
2. Demo camera display: wide-angle view with detector boxes re-projected from
   the narrow camera.
3. Detector accuracy on a real, held-out crack dataset — highest priority,
   since the problem statement is defect detection.
4. Truss coverage improvement (64% → 91.7% planned) — costed, not yet flown.
