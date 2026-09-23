# SIH_AVIAN_FINAL — Final sprint to the deadline (24 Sep 2026, 17:00 IST)

**This one file replaces all section-by-section prompts from here to the
deadline.** Run it top to bottom without waiting for a human between phases.
The user is asleep until about 07:00 IST.

This overrides earlier master prompts wherever they conflict. Earlier
prompts still apply for rules they set that aren't restated here: the
leakage rules, the "no ground truth in planning" rule, and the headline
rules.

## Operating rules (apply to every phase)

1. **Hard stops beat completeness.** At each phase's stop time, stop,
   commit what's real, and move on. Unfinished work goes in the handoff as
   "not run". It is never estimated.
2. **Never overwrite a result that's already committed.** New runs write
   new files (e.g. `flight_log_s4.json`). Earlier numbers stay as "before".
3. **Failures:** retry once. If it fails again, skip that step, record the
   error in one line, and continue. Don't wait for the user.
4. **Save usage.** The weekly limit is close. Run long jobs in the
   background. Check in about every 45–60 minutes, not more often, and
   don't re-read large files you've already read.
5. **Commit at the end of every phase**, with the headline numbers in the
   message. Never commit raw renders, tiles, downloaded datasets or weights
   larger than 100 MB (use `.gitignore`).

---

## PHASE A — Finish Section 3 (now → 07:00 IST)

Follow `SIH_AVIAN_SECTION3_DETECTOR_V3_PROMPT.md`, with these changes:

1. Let the all-waypoint zoom-tile render finish. Then train v3 (synthetic +
   MISSION-NEG hard negatives). Evaluate v1, v2 and v3 on MISSION-VAL and
   MISSION-TEST using the headline rule already fixed: best F1 on
   MISSION-VAL, with thresholds chosen on MISSION-VAL.
2. **Real photos, test only, no retraining.** In the background, download
   the smallest of CODEBRIM, SDNET2018 or the Mendeley "Concrete Crack
   Images for Classification" that can finish by about 06:00 at the
   measured speed. Check its license. Run the headline model on it as a
   **crack / no-crack test** and report precision and recall with counts.
   If nothing finishes in time, write "not run" plus the measured speed and
   size.
3. **Hard stop 07:00.** Commit, then post **HANDOFF — SECTION 3** in the
   format from the Section 3 prompt, containing only numbers that were
   actually measured.

---

## PHASE B — Section 4: cut the collision rate (07:00 → 11:30 IST)

Current state, frozen as "before": Stage 2, 150 waypoints,
**67/150 = 44.7% stuck/collision**, 56.66% coverage, 39/73 recall,
worst position error 201 m. Stage 1 was 71%.

### B1 — Diagnose before changing anything (about 45 min)

Using the committed Stage 2 `flight_log.json` (positions and
`sensed_ranges_m`), classify every one of the 67 stuck events:
- **(a) Approach:** stuck within about 2 m of its target waypoint.
- **(b) Transit:** stuck mid-leg, where the path clipped structure.
- **(c) Recovery:** stuck during a retreat or recovery leg.

Also measure the controller's **cross-track error during transit legs**
(the median and 95th percentile of the distance from the straight line
between waypoints). Write both into `mission/AVIAN_collision_diagnosis_FINAL.md`.

This decides the fix. Don't guess.

### B2 — Fix what the diagnosis points at (about 2 h)

Apply only the fixes the diagnosis supports, most likely:
- **Transit margin from data.** Replace `TRANSIT_CLEARANCE_MARGIN_M = 0.05`
  with airframe radius + the measured 95th-percentile cross-track error.
  Write the derivation in a code comment.
- **Denser segment check.** Sample transit segments at a step no larger than
  half the thinnest structural primitive's smallest dimension, taken from
  the collision JSON.
- **Route around blocked legs.** For any leg that fails the new check, plan
  a detour with A* on a coarse 3-D occupancy grid (about 0.5–1 m voxels,
  inflated by the new margin), built from the collision primitives. This
  also covers the "8 waypoints could not find a clear transit path" from
  the coverage build log.
- **Waypoint clearance.** Any waypoint closer to structure than the new
  margin is nudged outward along the surface normal, or dropped. Report
  counts for both.

Keep the planner's viewpoint selection unchanged, so before and after fly
the same inspection plan. The "no ground truth in planning" rule still
applies; re-run its grep.

### B3 — Re-fly and re-score (about 1 h)

- Re-fly Stage 2 with the fixes. Write `mission/flight_log_s4.json`, and
  run `score_coverage.py` on it.
- Report **before → after**: stuck/collision rate, coverage, defect recall,
  respawns, worst position error, battery packs used, and waypoints dropped
  or nudged.
- **Report the "after" numbers whatever they are.** If they're worse, say
  so and keep the "before" numbers as the result. Don't tune until they
  look good.
- Detection results (Phase A) were measured on the *earlier* flight's
  poses. Don't re-render tiles. Label them that way in the handoff.

### B4 — Hard stop 11:30

Commit, then post:

```
HANDOFF — SECTION 4
Diagnosis (67 events):        approach a / transit b / recovery c; cross-track median, p95
Fixes applied:                margin old → new (derivation), sampling step, A* detours n legs, waypoints nudged/dropped
Stage 2 before → after:       stuck % ; coverage % ; recall (n/73) ; respawns ; worst error m ; packs
Grep (no ground truth in planning):  clean / hits
Still weak, plainly:          …
Commit:                       …
```

---

## PHASE C — Freeze the prototype (11:30 → 12:30 IST)

1. Write `FINAL_RESULTS.json` at the repo root: every number the deck will
   quote, each with its source file and commit. It covers autonomy
   (before/after), coverage, recall, avoidance evidence, the detector
   (synthetic test, MISSION-TEST, real-photo test), dataset and test-set
   sizes, zoom (≈ 8.7× optical, 9° HFOV, about 1.97 mm/px at 8 m), models
   and licenses.
2. Add a **"Results at a glance"** section at the top of `README.md`, taken
   from `FINAL_RESULTS.json`, with no new claims. Include a short **"What
   this prototype does NOT do yet"** list: real hardware, live Gazebo
   flight, bolt detection, and anything else that is true.
3. Commit and tag: `git tag sih-idea-submission`.
4. Post the final block:

```
FINAL HANDOFF — PROTOTYPE FROZEN
Autonomy:        stuck before → after; coverage; recall; sensed-avoidance evidence
Detection:       headline model; synthetic test mAP (n); MISSION-TEST P/R (unseen n | all n); real-photo P/R (n) or "not run"
Zoom:            8.7× optical, 9° HFOV, ~1.97 mm/px at 8 m
Stack/licenses:  …
Not done yet:    …
Tag / commit:    sih-idea-submission / …
```

### Stretch — only if Phase C is committed before 12:00

Build a single static `dashboard/index.html`: no server, reading
`FINAL_RESULTS.json` plus a few flight and detection files. Include:
- a plan-view map of the waypoints, coloured by settled or stuck
- the before/after collision numbers
- 6 example zoom tiles with their detections, including one false positive
  and one miss, not only successes

Take one screenshot (`dashboard/screenshot.png`). Stop at 12:30 whatever
state it's in.

**Then stop entirely.** The deck is built separately; don't touch `deck/`.
