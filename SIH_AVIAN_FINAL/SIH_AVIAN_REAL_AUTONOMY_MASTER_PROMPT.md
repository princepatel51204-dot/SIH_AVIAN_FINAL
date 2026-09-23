# SIH_AVIAN_FINAL — Real obstacle avoidance, then a real detector

**Order matters and is fixed: finish sensed obstacle avoidance and Stage 2's
flight numbers FIRST. Do not start on the detector until that gate passes.**
A trained detector bolted onto a mission planner that still crashes 71% of
the time, using obstacle avoidance that just reads a file instead of
sensing anything, improves the wrong layer — judges will ask about
autonomy before they ask about detection accuracy.

This file is the message pasted into Claude Code to start this pass, kept
here per the established convention (see `SIH_AVIAN_FINAL_MASTER_PROMPT.md`,
`SIH_AVIAN_TWOPANEL_MASTER_PROMPT.md`, `SIH_AVIAN_FLIGHTFIX_MASTER_PROMPT.md`,
and `SIH_AVIAN_AUTONOMOUS_MASTER_PROMPT.md` for the builds this sits on top
of).

---

## PHASE 1 — Obstacle avoidance stops being scripted, starts being sensed

### The gap, precisely

Today, `flight_final.py`'s obstacle handling reads
`scene/collision/avian_bridge_collision.json` — the full, exact set of 568
structural primitives — directly, before the flight even starts. The
aircraft "avoids" a girder because the mission planner already knew
exactly where every girder was. That is not obstacle avoidance; it's path
planning against known geometry. A real inspection drone doesn't have that
file — it has to build up what's around it from what it can actually
sense, in real time, and react to that.

### What to build

1. **A synthetic onboard sensor**, not a file read. In PyBullet, add a
   raycast-based virtual range sensor on the simulated aircraft (a small
   forward/downward fan of rays — this is exactly how PyBullet's own
   `rayTestBatch` is meant to be used, and it's the standard way to fake a
   LiDAR/depth sensor in a physics sim without needing a real sensor
   model). At each simulation step, cast the rays and get back distances
   to whatever they actually hit — the vehicle now has a live, local,
   incomplete picture of its surroundings, the same limitation a real
   drone's LiDAR or stereo camera has under a bridge deck.
2. **A reactive avoidance layer** that consumes ONLY the sensor's returns
   — never `avian_bridge_collision.json` directly during flight. If a ray
   comes back under some threshold distance, the controller must react
   (slow, divert, or hold) based on that sensed reading alone. The
   collision JSON is allowed to still exist as the *ground truth PyBullet
   itself uses to know what's physically solid* (that's unavoidable — it's
   the physics engine's own world model) — the constraint is specifically
   that the aircraft's *decision-making* code path may not read that file.
   Grep for it in the flight/avoidance code the same way the coverage
   planner was checked for ground-truth reads in the previous pass: any
   hit outside the physics-loading code is a bug.
3. **Log what the sensor actually saw**, not just the resulting position —
   `flight_log.json` entries should gain a `sensed_ranges_m` (or similar)
   field per step or per waypoint, so a reviewer can later check that
   avoidance decisions were actually driven by sensed data and not quietly
   falling back to the old file-based check.
4. **Re-run the Stage 2 coverage-mission flight (150 waypoints, standoff
   8.0 m) to completion** with this real sensing in place. The result
   that never finished last session — real crash rate, real recall — has
   to exist as an actual number before anything else proceeds. If it
   still doesn't finish, that itself is the finding to report, not
   something to route around.

### Gate before Phase 2

- `sensed_ranges_m`-style data is present in the flight log, proving
  avoidance ran on sensor data, not the collision file.
- The Stage 2 flight has a completed `flight_log.json` with real
  `n_stuck`, `n_respawns`, `position_hold_worst_error_m`, and
  `battery_pct_remaining` — not "pending."
- `score_coverage.py` has run against that completed flight and produced
  a real coverage % and defect recall number.

---

## PHASE 2 — Replace the fake detector with a real one

### Don't train on the dataset as-is — it has known, documented bugs

`dataset/AVIAN_dataset_build_log_FINAL.txt` (798 frames: 364 positive, 434
negative) already shows **4 of 8 validation checks failing**:

- **VD01** — 78 of 364 positive frames' masks don't contain their own
  labeled instance (mask/label mismatch — training on these teaches the
  model wrong pixels for the defect it's supposed to find).
- **VD03** — 205 of 798 frames' achieved camera standoff drifted more
  than 5% from what was commanded (the geometry/scale implied by the
  image doesn't match what was intended — matters for any
  size-of-defect-dependent training signal).
- **VD04** — 121 of 434 "negative" (no-defect) frames actually contain a
  defect instance in their mask (contaminated negatives — this is the
  worst one: it directly teaches the model that real defects look like
  nothing).
- **VD06** — 9 manifest rows missing `view_obliquity_deg`.

**Fix or filter before training, don't train through it:**
1. For VD01 and VD04, the safest move is exclusion, not repair —
   drop any frame whose mask doesn't match its manifest label (VD01) and
   any "negative" whose mask isn't actually empty (VD04) from the training
   set entirely, and report exactly how many frames were dropped and why.
   Training on a known-contaminated negative to save frame count is worse
   than training on fewer, clean ones.
2. For VD03, decide per-model whether standoff drift matters (it does if
   the model or any post-processing estimates physical defect size from
   pixel size — `min_detect_range_m` math depends on this). If it does,
   drop or flag those frames too; if the model is a pure classifier/
   detector with no size-estimation step, document the decision either
   way.
3. For VD06, fill or drop the 9 rows — don't leave nulls silently flowing
   into a training pipeline.
4. Re-run `validate_final.py`'s dataset checks (or `dataset_render_final.py`'s
   own VD01-08) against the filtered set and confirm it now passes clean,
   or explicitly document which checks still fail and why that's
   acceptable for training purposes.

### Build the real detector

1. **Task**: start with defect *presence/class* detection (a bounding-box
   or classification model over the 19 defect types already in the
   ground truth), not full segmentation — matching scope to what a
   hackathon timeline can actually validate. A lightweight YOLO-family
   model (small enough to train quickly, matches the class of models your
   own research found peer-reviewed accuracy numbers for — RCO-YOLOv5 at
   91.0% mAP@0.5) is the right scope; don't reach for a heavier
   architecture than the dataset size supports.
2. **Split honestly**: train/val/test split from the *filtered* frame set,
   stratified by defect type so rare classes (check `AVIAN_dataset_stats_FINAL.json`'s
   class balance) appear in all three splits where possible. Record the
   exact split sizes — your own research flagged that judges specifically
   credit "accuracy WITH test-set size," not a bare percentage.
3. **Train, then report failure modes, not just accuracy.** At minimum:
   overall mAP/accuracy on the held-out test set, per-class breakdown
   (some of the 19 defect types have very few instances — say so), and at
   least one characterized failure case (e.g., "confuses hairline cracks
   with formwork lines at long standoff," or whatever the confusion matrix
   actually shows). This is exactly the "about 87% on 400 samples, drops
   on night images"-style honesty your research says wins over an
   unqualified 99% claim.
4. **Replace `detect_stub_final.py`** with a thin wrapper that loads the
   trained model and runs real inference on a rendered image — keep the
   same call signature (`detect(image_or_viewpoint_pose) -> list[{class,
   confidence, bbox}]`) so nothing else in the pipeline (the flight-in-
   the-loop detection call, `score_coverage.py`) has to change. Delete or
   clearly quarantine the old ground-truth-peeking stub logic — don't
   leave it reachable as a silent fallback.
5. **Re-run `score_coverage.py`** with the real detector wired in and
   report real recall/precision against held-out, non-escalated ground
   truth — this number now means something it didn't when the "detector"
   was reading the answer key.

### Gate before calling this done

- Training frame counts are reported *after* filtering, with the filter
  reasons stated (not just a total frame count).
- Test-set accuracy is reported with test-set size and at least one named
  failure mode — no bare percentage.
- `detect_stub_final.py`'s ground-truth-reading code path is gone or
  clearly dead, not just superseded.
- `score_coverage.py`'s recall/precision numbers come from the real model,
  and the report says so explicitly (distinguish "this run used the real
  detector" from any earlier stub-based sanity check still in the repo).
