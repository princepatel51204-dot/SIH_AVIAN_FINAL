# SIH_AVIAN_FINAL — Two-panel inspection system

**Gazebo shows where the drone is. Blender shows what its camera sees.**
**Build the offline path first — it cannot fail. Then add the live bridge.**

This file is the message pasted into Claude Code to start this pass, kept
here per the established convention (see `SIH_AVIAN_FINAL_MASTER_PROMPT.md`,
`SIH_AVIAN_DETECTION_MASTER_PROMPT.md`, and `SIH_AVIAN_DATASET_MASTER_PROMPT.md`
for the builds this one sits on top of).

## Locked pre-flight decisions (2026-09-23)

1. **Waypoint scope: the full set.** All 192 defects, clustered — not the
   capped 22-plus-sample option. Confirmed by the user; render cost per
   waypoint is far lower than the per-defect dataset render (clustering
   means far fewer camera positions than 192), so the full set stays well
   inside budget.
2. **Position view for Stages A-D: a 2-D plan view**, not pre-rendered
   Gazebo screenshots. The prompt's own §5 table and §7 stage list already
   put Gazebo at Stage E ("Same mission in Gazebo... Stage E makes it
   live") — a plan view for D isn't a shortcut, it's what the architecture
   already specifies.
3. **Dashboard stack: a single static HTML page** reading `mission.json`
   and the rendered PNGs directly, no server. Stated outright in the
   prompt's own §9.3 ("least that can break... only earns its place at
   Stage E").

## Corrections made against source before writing code

- **Energy budget: 1,300 Wh, not 1,335.** `AVIAN_UAV/simulation/vehicle.py`'s
  `battery_Wh = 1300.0` ("from the CAD battery group") is the only battery
  capacity figure in the codebase; "1,335 Wh" does not appear anywhere in
  `AVIAN_UAV/`. `safety.py`'s `BATTERY_RESERVE_PCT = 20.0` gives 1,040 Wh
  usable before the RETURN-to-home safety mode triggers. Reported against
  the verified number, not retyped from the prompt.
- **No airspace volumes exist for SIH_AVIAN_FINAL.** `zones_final.py` (REV-
  C's five-class airspace system) was never built for this scene — already
  documented as a gap in the detection pass's own README section ("What did
  NOT get built"). "Outside a defined airspace volume" is enforced instead
  by structure-collision + ray-cast line-of-sight (§2.2's own required
  check folds naturally into this: a waypoint whose ray to its own defect
  is blocked at near-zero range is embedded in structure), not a volume
  that would have to be fabricated to exist.
- **The detection dataset (previous task) has 4/8 checks failing on the
  full run** (VD01/VD03/VD04/VD06) — traced to FLOOR_STRINGER bolt joints
  never receiving the same face-offset fix GUSSET joints got after the
  CAM_12 bug. Flagged, not fixed here (out of this pass's scope, and this
  work reads the ground-truth JSON directly, not the dataset's own
  rendered images).

## What follows is the original prompt, verbatim

> **Build the two-panel inspection system. Read
> `SIH_AVIAN_TWOPANEL_MASTER_PROMPT.md` in full. §3 of
> `AVIAN_REV_C_MASTER_PROMPT.md` still binds, including §3.10.**
>
> [... see the pasted prompt in the session transcript for the full text:
> §1 architecture, §2 mission-from-ground-truth, §3 camera intrinsics,
> §4 PyBullet-first flight, §5 dashboard, §6 checks VB01-08, §7 stages
> A-E (stop at D), §8 gate, §9 blocking questions ...]
