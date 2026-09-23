# SIH_AVIAN_FINAL — Stage B fix: battery-exhaustion abort

**The 104-waypoint flight completed but the result is not usable. Fix this
before Stage C (renders) or Stage D (dashboard) touch `flight_log.json` —
both would be built on data from a drone that was already dead for a third
of the mission.**

This file is the message pasted into Claude Code to start this pass, kept
here per the established convention (see `SIH_AVIAN_FINAL_MASTER_PROMPT.md`,
`SIH_AVIAN_DETECTION_MASTER_PROMPT.md`, `SIH_AVIAN_DATASET_MASTER_PROMPT.md`,
and `SIH_AVIAN_TWOPANEL_MASTER_PROMPT.md` for the builds this one sits on top
of).

## What the full run actually produced (2026-09-23, `mission/flight_log.json`)

| metric | value | planned/expected |
|---|---|---|
| real distance flown | 6,776.4 m | 2,141.5 m planned (3.16×) |
| battery used | 2,009.1 Wh | 1,040 Wh usable (1,300 Wh full pack) |
| battery remaining at end | 0.0 % | — |
| worst position-hold error | 272.97 m | 0.15 m tolerance |
| waypoints never settled | 84 | of 174 log entries |
| genuine stuck/crash events | 78 | |
| respawns used | 12 | of 20 budget |

This is not "a handful of respawns, recovery worked" — it's a controller
running an aircraft that is already out of power for over a third of the
flight.

## Root cause, confirmed against the log directly

`battery_pct` first reaches **0.0 % at `WP_069`** — log entry 113 of 174,
sim time 1,574 s, **35 waypoints before the end of the 104-waypoint
mission**. `flight_final.py`'s main loop (source/flight_final.py:192-272)
has no check for this. It keeps calling `fly_to()` for every remaining
waypoint, every stuck-recovery retreat, and every respawn exactly as if the
battery were full.

The worst single entry in the whole log, `WP_094_RECOVERY`
(`settle_error_m: 272.97`, `stuck: true`, `battery_pct: 0.0`, rpy roll
`-3.14 rad` — the airframe upside down), happens 25 waypoints *after*
the battery was already at zero. An unpowered aircraft cannot climb, hold
attitude, or retreat to `last_good`; what the log calls "stuck" and
"recovery" for that whole back third of the flight is PyBullet's gravity
and the collision response acting on a airframe that has no thrust left,
not the structure-collision behaviour the stuck-detect logic
(`fly_to`'s own docstring, source/flight_final.py:71-79) was built to
catch. That's also why `real_distance_m` (6,776 m) is over 3× the planned
distance and `battery_used_Wh` (2,009 Wh) is nearly double the 1,040 Wh
usable budget — a tumbling/falling airframe keeps accumulating position
delta and (apparently) keeps being charged for commanded thrust the motors
can't actually deliver.

The recovery/respawn logic itself (retreat-to-last-good,
`MAX_CONSECUTIVE_STUCK`, `MAX_RESPAWNS`) is correct and was already
validated on the first ~34 waypoints, where every stuck/respawn event
resolved normally (e.g. `WP_005_RESPAWN` settled at 0.094 m,
`WP_021_RESPAWN` at 0.094 m, both with battery well above zero). **The bug
is that nothing in the loop asks "is there still power to do this" before
attempting the next waypoint, retreat, or respawn.**

## The fix

In `source/flight_final.py`'s main loop, add a battery-exhaustion check
that behaves like the existing `MAX_RESPAWNS` abort path
(source/flight_final.py:234-244) — reuse that same skip-and-log pattern
rather than inventing a second one:

1. **Before each waypoint attempt** (top of the `for i, wp in
   enumerate(waypoints)` loop), read `v.state()["battery_pct"]`. If it is
   at or below a small floor (0.0%, or whatever the vehicle model treats
   as "no usable thrust" — check `AVIAN_UAV/simulation/vehicle.py`'s own
   battery/power model for the real cutoff rather than assuming 0.0 is
   exact), stop attempting flight immediately: log the remaining waypoints
   as `skipped=True` (same shape as the existing `MAX_RESPAWNS` skip
   block) and break out of the loop. Do **not** attempt `RETURN_HOME`
   with a dead battery either — log why it was skipped too.
2. **Before a respawn** (source/flight_final.py:258-271): a respawn resets
   *position*, not the battery — confirm this against `vehicle.py`
   (`resetBasePositionAndOrientation`/`resetBaseVelocity` reset pose only,
   `battery_used_Wh` is presumably still whatever the vehicle object was
   tracking). If the battery is already exhausted, respawning a fresh
   airframe at base doesn't help — it will immediately fail again. Fold
   this into the same check as (1): don't spend a respawn budget slot on
   an aircraft that has nothing left to fly with.
3. **Decide, explicitly, whether battery should have a real cutoff
   physically enforced** — i.e. should `Cascade`/`AvianWorld` itself refuse
   to command thrust once `battery_pct <= 0`, rather than relying on
   `flight_final.py` to notice and stop asking? That would make the
   root-cause fix live in the validated `AVIAN_UAV` stack instead of only
   in this mission runner, and would prevent the exact "commanded thrust
   with no power" state that produced the 272 m error and the 3.16×
   distance blowup. If `AVIAN_UAV` is out of scope for this pass (it's
   REV-C's validated 46/46 stack — treat any change there as a bigger,
   separate decision, not a quick edit), at minimum add a code comment in
   `flight_final.py` at the new check explaining that the *cause* of the
   thrashing is upstream and this is a mission-level guard, not a physics
   fix.
4. **Re-derive `n_skipped` honestly.** The last run reported
   `n_skipped: 0` because nothing was ever marked skipped — the loop just
   kept running to a physically meaningless conclusion. After the fix,
   `n_skipped` should reflect the real count of waypoints abandoned for
   battery reasons, distinct from `n_respawns` (budget exhausted) and
   `n_stuck` (genuine collision/crash) — three different reasons a
   waypoint doesn't get flown, and the summary should keep them visibly
   separate rather than folding battery-exhaustion into one of the
   existing counters.

## What this means for the mission, independent of the code fix

Even with the fix, **the underlying mission probably still can't be flown
on one battery.** `mission/AVIAN_mission_build_log_FINAL.txt` already
estimated 2,141.5 m / ~804 s / ~594.5 Wh for the planned route — well
inside the 1,040 Wh usable budget on paper — but the *real* PyBullet
power model burned through 1,040 Wh by WP_069, roughly a third of the way
through. That's a large enough gap between Stage A's distance-based
estimate and Stage B's real momentum-theory measurement that it needs a
number, not a guess:

- After the fix, re-run the full 104-waypoint flight and read
  `battery_used_Wh` at the point the *original* run reached WP_069 (or
  just note the sim time / battery burn rate on a clean first third of the
  flight, since the fixed run won't crash there).
- Compare real Wh/m burn rate to Stage A's estimate. If the real rate is
  roughly double the planned one (matches the 2,009 vs ~595 Wh gap in the
  broken run, even discounting the tumbling-airframe waste), Stage A's
  mission needs either a shorter/split mission (return-to-base battery
  swap, matching the `n_respawns` pattern already built) or the
  1,040 Wh usable budget needs revisiting against real flight data rather
  than the distance-based estimate.

Flag this at the gate below — don't quietly re-run and move on if the
fixed flight still can't finish the mission on one charge.

## Gate before Stage C/D

Re-run `python3 source/flight_final.py` after the fix and confirm, before
touching renders or the dashboard:

1. `n_skipped` and `n_stuck` and `n_respawns` are each individually
   sane (no single number absorbing what should be a different failure
   mode).
2. `battery_pct_remaining` at the end is either > 0 (mission completed
   with power to spare) or the log clearly shows *when and why* it hit
   zero and that everything after that point is honestly marked skipped
   — not attempted-and-failed.
3. `position_hold_worst_error_m` is back in the range the validated
   Phase 1 gate actually supports (≤ ~0.15-0.2 m outside genuine
   stuck/crash events) — a few outliers from real structure-collision
   events are expected and fine; a 272 m outlier is not.
4. `real_distance_m` is close to `planned_distance_m` (some overhead from
   transit + recovery legs is expected; 3× is not).
5. If the mission can't finish on one battery even after the fix, that's
   a Stage A decision (split mission / battery swap / revisit the energy
   budget), not something to paper over in Stage B.

Only once these hold does `flight_log.json` become real ground truth for
Stage C's renders and Stage D's dashboard.
