# AVIAN Phase 1 demo video

`avian_demo.mp4` -- 1920x1080, 30 fps, H.264, ~33 s. Renders what Phase 1
already verifies (46/46 acceptance checks, see `../logs/phase1_report.json`).
It adds no new physics, control logic or sensor model.

## What is REAL (i.e. an actual Phase 1 output, not decoration)

- **Geometry**: every mesh is the exported CAD B-rep from
  `../description/meshes/*.stl` (17 links), imported at the URDF's own
  `scale="0.001 0.001 0.001"`, never a primitive stand-in.
- **Flight**: `simulation.vehicle.AvianVehicle` + `simulation.controller.Cascade`,
  the same X8 allocation and cascaded controller the Phase 1 dynamics tests
  exercise. The aircraft climbs, cruises to a standoff position, and cruises
  back under real closed-loop control -- no scripted/keyframed flight path.
- **Arm motion**: every joint angle in the deploy/contact/retract segments is
  a sample of `Manipulator.plan_trajectory()`'s quintic profile, tracked by
  the arm's real `POSITION_CONTROL` servo (not teleported -- see "known
  quirks" below for why that distinction mattered). `solve_ik()` verified the
  pre-contact, contact and via-point targets against reach, joint limits,
  self-collision and environment collision before any of them were used.
- **Camera feed (picture-in-picture, bottom-right)**: `sensors.cameras.RGBCamera`
  and `DepthCamera`'s own `read()` -- PyBullet's own renderer, its own
  `computeViewMatrix`/`computeProjectionMatrixFOV`, at the exact recorded
  pose. Not a Blender re-creation. See D05 in the acceptance table.
- **Rotor count, arm reach, joint limits, mass**: all read from
  `avian_description_manifest.json` / the CAD, same as every other Phase 1
  script.

## What is ILLUSTRATIVE (stated here and captioned on screen)

- **Rotor spin rate** (25 rad/s, constant): Phase 1 has no measured RPM or
  blade-element model, and the rotor joints are free-hinged with no motor
  drive (`force=0`) because thrust is applied as an external force, not a
  joint torque. The visible spin is a kinematic overlay for readability, not
  a simulated quantity. Labelled in the watermark caption.
- **Repair target** (the flat panel): a plain PyBullet box
  (0.3 x 0.3 x 0.05 m), not a modelled defect. Phase 1 carries no
  bridge/defect ground truth -- that lives in the separate `AVIAN_ENVIRONMENT`
  package, under a different coordinate system for a different scene, and is
  deliberately not imported here (mixing them would misrepresent both
  packages). Labelled in `avian_demo_track.json["panel"]["label"]` and on
  screen; never called a "crack" or "spall".
- **Repair-applied cue** (the panel's green glow during the hold): there is
  no fluid/spray-repair model in Phase 1. The glow is a Blender emission
  overlay for the ~4 s hold, captioned "ILLUSTRATIVE -- repair-applied cue,
  not simulated" on screen.
- **Backdrop**: a plain grey studio sweep (floor + wall + 3-point lighting),
  not the bridge environment, per the brief's default.
- **Cinematic camera path**: an orbiting third-person camera, independent of
  the aircraft's own sensors, for watchability.

## Panel placement and the via-point (why the arm path isn't a straight line)

The arm's own verified "stowed" pose (`q = 0`, the same pose
`tests/test_manipulator.py` calls stowed and checks self-collision-free)
happens to leave `tool0` almost fully extended straight down from the arm
base (0.045, 0, -1.045 m in the arm-base frame) -- a property of this
specific joint-zero convention, not something this task changed. A panel
placed near that line makes the straight joint-space interpolation toward it
sweep back through the panel before the elbow re-bends into its final shape:
confirmed by running `within_limits`/`check_self_collision`/
`check_environment_collision` at every sampled keyframe of the direct path
and finding intrusions of 1-6 cm on links 12-15 partway through.

The fix was to stage the deploy/retract through an explicit via-point
(`VIA_OFFSET_FROM_ARM_BASE` in `generate_track.py`), chosen by sampling
candidates and keeping the first whose home->via, via->pre-contact,
pre-contact->contact legs (and their reverses) all verified collision-free
at *every* sampled keyframe, not just their endpoints. This is exactly what
the D02/D03 checks below assert.

## Known quirk carried over from `manipulator/arm.py` (not fixed here)

`plan_trajectory()`'s own `within_velocity_limits` self-check evaluates
`False` for several of this demo's arm legs: the duration formula uses a
1.6x stretch factor while the module's own docstring cites a quintic
peak/mean velocity ratio of ~1.875x, so the timing-dominant joint's peak
velocity runs ~17% over its limit regardless of how many samples are taken
(verified at n=40, 59, 200 and 2000 -- the ratio converges to ~1.172, so
this is not a sampling artifact of this renderer). This is a property of
`manipulator/arm.py`, which this task does not modify (frozen, Phase 1
guarantees). It does not affect D02/D03 (joint-limit and collision
verification), which check `within_limits()` / `check_self_collision()` /
`check_environment_collision()` directly, not `plan_trajectory`'s own
velocity self-check.

## Pipeline

```
media/generate_track.py        PyBullet: flight + verified repair sequence
                                -> avian_demo_track.json
media/render_camera_feed.py    PyBullet sensors, replayed from the track
                                -> frames_cam/frame_?????.png
media/render_captions.py       PIL caption/label overlay
                                -> frames_caption/frame_?????.png
media/render_blender.py        Blender (headless): meshes + lighting + camera
                                -> frames_main/frame_?????.png
media/compose_video.py         ffmpeg: composite the three layers
                                -> avian_demo.mp4
media/verify_demo.py           D01-D07 acceptance checks
```

## Commands (all run from `AVIAN_UAV/`, all actually run while building this)

```bash
# 1. track (PyBullet, project venv) -- ~15 s
AVIAN_CAD_DIR=$PWD/cad ../.venv/bin/python3 media/generate_track.py

# 2. camera-feed PIP (PyBullet, project venv) -- ~6 min at LOOP_RES
AVIAN_CAD_DIR=$PWD/cad ../.venv/bin/python3 media/render_camera_feed.py

# 3. captions (PIL, project venv) -- a few seconds
../.venv/bin/python3 media/render_captions.py

# 4. aircraft render (Blender's own Python, NOT the venv) -- ~25-30 min
blender --background --python media/render_blender.py -- \
    --track media/avian_demo_track.json --outdir media/frames_main

# 5. composite (ffmpeg)
../.venv/bin/python3 media/compose_video.py

# 6. acceptance checks
AVIAN_CAD_DIR=$PWD/cad ../.venv/bin/python3 media/verify_demo.py
```

## Acceptance (D01-D07)

Result of the actual run made while building this video (2026-09-27). Re-run
`verify_demo.py` for the authoritative, current answer -- do not trust this
table over a fresh run, and note it was run against the intermediate PNG
sequences, which are deleted after a successful build to keep the repo
small (regenerate them with the commands above if you need to re-verify
without re-encoding).

| ID | Check | Result |
|---|---|---|
| D01 | every sampled frame differs from its neighbour | PASS -- stddevs [4.991, 2.051, 0.45] |
| D02 | arm trajectory never exceeds a joint limit | PASS -- 194 keyframes checked, 0 bad |
| D03 | arm trajectory collision-free at every keyframe | PASS -- 0 self-collision, 0 environment-collision |
| D04 | rotors rotate continuously (strictly monotonic) | PASS -- all 8 rotors monotonic |
| D05 | camera inset matches a standalone sensor read() | PASS -- frames [2, 4, 6], max pixel diff 0 |
| D06 | video file is valid H.264 1920x1080 ~30fps | PASS -- 1920x1080 h264 30.00fps 33.0s |
| D07 | illustrative vs simulated is labelled | PASS -- panel/rotor/repair-cue/README all labelled |

**7/7 checks passed.**

Frozen Phase 1 gate re-run after building this (`tests/run_all.py`, with
`AVIAN_ENV_DIR`/`AVIAN_CAD_DIR` set): still **46/46**, confirming this task
did not touch `simulation/vehicle.py`, `simulation/controller.py`,
`sensors/*.py` or `manipulator/arm.py`'s guarantees.
