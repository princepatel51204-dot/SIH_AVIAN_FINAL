# Overnight autonomous run — running report

(Started 2026-09-27 ~00:30. Numbers below each trace to a file named next to them. Updated as phases finish.)

## Decisions made on the user's behalf (and why)
1. **Dashboard ownership.** Another Claude session ("environment build") was already editing `dashboard/build_dashboard.py` /
   `index.html` for its own user (uncommitted edits timestamped 00:02-00:04) and had applied the two fixes I found (Stage 1 =
   65.4 %; relabelled Gazebo autonomy bar). Two sessions editing one file would clobber each other, so I agreed that the other session
   keeps the file and I supply column-flight numbers + image paths and review the built page.
2. **Low-priority prep work during the flight.** Blender introspection, decal rendering and script writing ran at `nice 19` while the
   column flight was still airborne (the follower runs on simulation time; nothing wall-clock-dependent). Gazebo test runs and detector
   inference waited for the flight to finish.
3. **Ground truth = all 192 defects.** The first baseline used only the 96 road defects; the Gazebo defects model has 192 spheres
   (road 96 + metro 20 + steel 76). I re-scored against all 192 and replaced the earlier "3 of 87" figure.

## Commits (all local, none pushed)
- `83fed75` docs: Stage 1 65.4 % (68/104) replaces unsourced 74 / 71.2 % in two planning documents (not requested; user's overnight plan asked to reconcile the numbers)
- `1a90605` Gazebo detection baseline tooling + results (`eval_detection_gazebo.py`, `detection_box_targets.py`, `detection_eval/`)
