# SIH_AVIAN_FINAL — Stage 2: autonomous coverage exploration

**Stage 1 (`mission_final.py` + `flight_final.py`) proves the airframe,
controller and battery model can fly a *given* path without crashing or
running out of power. It is not autonomy — every waypoint comes straight
out of `scene/collision/AVIAN_defect_ground_truth_FINAL.json`, privileged
knowledge a real inspection drone would not have. Stage 2 removes that
privilege: the mission is planned from structure geometry alone, and
defects are *found*, not looked up.**

This file is the message pasted into Claude Code to start this pass, kept
here per the established convention (see `SIH_AVIAN_FINAL_MASTER_PROMPT.md`,
`SIH_AVIAN_DETECTION_MASTER_PROMPT.md`, `SIH_AVIAN_DATASET_MASTER_PROMPT.md`,
`SIH_AVIAN_TWOPANEL_MASTER_PROMPT.md`, and `SIH_AVIAN_FLIGHTFIX_MASTER_PROMPT.md`
for the builds this one sits on top of).

## Can this start before Stage 1's gate is clean?

**Yes, and it should run in parallel.** `mission_final.py` builds waypoints
by reading `AVIAN_defect_ground_truth_FINAL.json` and pointing at known
defect positions; the coverage planner below reads only
`scene/collision/avian_bridge_collision.json` — the 568 oriented
box/cylinder primitives PyBullet itself loads (same file, same
`_load_collision_primitives()` / `_signed_clearance()` helpers already
proven correct in `mission_final.py`). It has no dependency on the
WP_013–015 collision-cluster bug currently being chased in Stage 1, and it
still flies through the *same* validated `flight_final.py` stack once
Stage 1's gate does pass — so build the plan now, and simply re-point it
at Stage 1's fixed flight runner when that lands.

## §1 What "autonomous" means here, precisely

Not full online SLAM-driven exploration in one pass — that's a much larger
build than this prototype window supports, and the brief's own gate
(`SIH_AVIAN_FINAL_MASTER_PROMPT.md` — stop at Stage D for the demo) doesn't
ask for it. What it does mean, and what actually distinguishes this from
Stage 1:

1. **The planner never reads a defect ground-truth file.** Input is
   *only* `avian_bridge_collision.json` (structure geometry) +
   `params_final.py`'s sensor constants (`GSD_MM_PER_PX_AT_1M`,
   `PX_TO_IDENTIFY`, `MIN_FLYABLE_RANGE_M`). If a function signature in
   this pass takes a ground-truth path as an argument, that's a smell —
   stop and re-check the boundary.
2. **Coverage is the objective, not defect count.** The planner's job is
   "see every square metre of every inspectable surface at resolving
   range," computed geometrically from the collision primitives'
   `kind`/`centre`/`half_extents`/`yaw` — not "visit these N known
   trouble spots."
3. **Detection happens in the loop, at flight time**, against whatever the
   (stub, for now) detector reports from the rendered/simulated view —
   not read from the ground-truth JSON. Ground truth is held out and used
   **only** to score the run afterward (recall/precision), never to plan
   or to decide what counts as "found" during the flight.

## §2 Coverage planner (`coverage_final.py`, new file, mirrors `mission_final.py`'s structure)

**Input:** `scene/collision/avian_bridge_collision.json` only.

**Which primitives are inspectable surfaces vs. not-inspectable:**
exclude by `kind` the same way the collision export itself already
excludes non-obstacles — `ground`, `water`, `landing_pad`, `train_car` are
not inspection targets. Everything else (`girder`, `pier_column`,
`pier_cap`, `pier_footing`, `deck_box`, `deck_slab`, `bearing`, `bracing`,
`diaphragm`, `gusset_plate`, `truss_chord`, `truss_diagonal`,
`truss_vertical`, `stringer`, `floor_beam`, `parapet`, `rail`,
`track_slab`, `catenary_mast`, `abutment`, `expansion_joint`, `joint_gap`,
`access_hatch`, `drain`, `cable_trough`, `service_duct`) is a real
structural surface and belongs in the coverage set — 546 of the 568
primitives by `primitives_by_kind` in the manifest.

**Per-primitive viewpoint generation:**
- For a BOX primitive, sample candidate view targets on each of its 6
  faces (skip faces that face permanently-obstructed directions, e.g. the
  underside of a footing against the ground half-space) — face centre
  plus a grid across the face sized so adjacent samples overlap by a
  configurable fraction (start at 30%) once projected through the sensor
  FOV, not a fixed metre spacing, since a `pier_column` face and a
  `gusset_plate` face need very different sample density.
- For a CYLINDER primitive (`pier_column`, `catenary_mast`), sample
  around the circumference at an angular step derived the same way (FOV
  at standoff range), plus top/bottom caps where exposed.
- **Standoff distance** at each sample: reuse `min_detect_range_m()` from
  `params_final.py` — but since there's no known defect's
  `feature_size_mm` here (no ground truth), use the *smallest
  feature size this prototype claims to be able to resolve at all*
  (check `params_final.py` / the detection master prompt for the finest
  defect class already in the ground-truth population, e.g. hairline
  cracks) as the standoff-defining feature size. This is the one place
  Stage 2 is allowed to read a *class-level* constant from the defect
  taxonomy (a design decision: "we inspect for defects down to X mm"),
  never a specific defect's logged position.
- **Clearance/embedding check:** identical logic to `mission_final.py`'s
  `_min_structure_clearance()` / `STRUCTURE_EMBED_MARGIN_M` (0.05 m) —
  don't reinvent it, import or port it verbatim so Stage 1 and Stage 2
  agree on what "clear of structure" means.
- **Line-of-sight check:** identical logic to `mission_final.py`'s
  `has_line_of_sight()` — a viewpoint whose ray to its own target face is
  blocked by another primitive is discarded, same as Stage 1's rule for
  defect visibility.

**Coverage reduction (this is the actual planning problem):** the raw
per-face/per-primitive sample set will be large (hundreds to low
thousands across 546 primitives) — reduce it to a minimum-viewpoint set
that still covers every sampled surface patch at least once. A greedy
set-cover (repeatedly pick the viewpoint that covers the most
not-yet-covered patches, matching `mission_final.py`'s own
`cluster_candidates()` pragmatism rather than reaching for an ILP solver
this prototype doesn't need) is the right scope here — document the
choice the same way `mission_final.py`'s comments document why
nearest-neighbour ordering was chosen over TSP-optimal.

**Ordering:** reuse `order_nearest_neighbour()` from `mission_final.py`
directly (it already takes a collision-checked transit path between
arbitrary points; it has no dependency on defects) rather than
duplicating it.

**Output:** `mission/coverage_mission.json`, same schema shape as
`mission/mission.json` (`waypoints: [{waypoint_id, position_m,
heading_rad, ...}]`, `base_position_m`, `total_distance_m`,
`battery_Wh_usable`) so `flight_final.py` can fly it with zero changes —
plus a `covers` field per waypoint naming which primitive(s)/face patch it
was generated to see, for later coverage scoring. Explicitly **no**
`defects` field — that's the tell that this file was built without
ground truth.

## §3 Detection-in-the-loop (stub first, real model later)

At each settled waypoint, Stage 2 needs *something* to call that returns
"defect present / not present, confidence" from the view — even before the
real detector (still 0% per project status) exists.

- **Stub for this pass:** a function with the same call signature the real
  detector will eventually have — `detect(image_or_viewpoint_pose) ->
  list[{class, confidence, bbox}]` — that, for now, can legitimately peek
  at ground truth internally (it's standing in for perception, and its
  whole job is to be swapped out later) but must be clearly marked as a
  stub in its own docstring and isolated to one file/function so replacing
  it with the real detector is a one-point change.
- **The mission loop itself still must not read ground truth directly** —
  only the stub detector may, and only to fake what a real detector would
  see. This boundary is the difference between an autonomy demo and a
  relabeled Stage 1.
- Loop shape per settled waypoint: fly → hold → call `detect()` → log
  result against `waypoint_id` in a new `detections.json` (parallel to
  `flight_log.json`, not merged into it — keep "what the sim's physics
  did" and "what the perception system claims it saw" as separate,
  independently-inspectable artifacts).

## §4 Scoring (after the flight only)

Build `score_coverage.py`: compares `detections.json` against
`AVIAN_defect_ground_truth_FINAL.json` (only file allowed to touch both)
to compute:
- **Structural coverage %**: fraction of the 546 inspectable primitives'
  surface area that had at least one settled, non-stuck waypoint within
  its standoff range and clear line of sight.
- **Defect recall**: of the defects in ground truth *not* already flagged
  `escalation_reason` (below_contrast/below_resolution/occluded/
  unreachable_angle — those are provably undetectable even in principle,
  don't penalize the planner for missing them), what fraction fell inside
  a flown, covering waypoint's standoff+FOV.
- **Detector precision/recall** once the stub is replaced with a real
  model — until then, report the stub's numbers as a sanity check on the
  scoring code itself, not as a real detector result, and say so plainly
  in the output.

## §5 Gate before calling Stage 2 done

1. `coverage_mission.json` built with zero reads of any
   `*ground_truth*.json` file — grep the new source file for
   `ground_truth` and confirm every hit is inside the isolated detector
   stub, nowhere else.
2. Structural coverage ≥ some stated target (pick one — e.g. 95% of
   inspectable surface area at resolving range) — report the real number,
   don't just assert it.
3. Flown through the *same* `flight_final.py` Stage 1 already validates —
   no second flight stack.
4. `flight_log.json` for this run passes the same gate criteria as Stage 1
   (worst position-hold error, battery reserve, no silent skips) — a
   coverage mission that crashes the aircraft just as much as a bad
   defect-driven one hasn't actually improved anything.
5. Recall against non-escalated ground-truth defects reported honestly,
   including the stub-detector caveat from §4.

Only once this holds does Stage 2 have real content for Stage C's renders
(now rendering *discovered* defects, not known ones) and Stage D's
dashboard.
