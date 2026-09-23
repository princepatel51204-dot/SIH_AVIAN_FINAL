# Stage 2 collision diagnosis — Phase B, step B1

Source: `mission/coverage_mission_flight_log.json` (the committed Stage 2
flight, 150 waypoints, `n_stuck=67`). Classification uses `stuck`,
`settle_error_m`, and `waypoint_id` naming (a `_RECOVERY`/`_RESPAWN`
suffix or a `BATTERY_SWAP_*` id marks a sub-leg, not one of the 67
primary waypoint attempts).

## Classification of the 67 stuck events

The 67 are **all primary waypoint attempts** (confirmed: filtering log
entries to `stuck=True` AND a plain `CWP_NNN` id gives exactly 67,
matching the flight log's own `n_stuck` field). None of the 67 are
themselves a recovery/retreat leg — those are separate, subsequent log
entries — so category (c) as literally "stuck during a retreat leg"
matches zero of the 67 by construction. Reported honestly rather than
forced to a non-zero count:

| category | count | settle_error_m range |
|---|---:|---|
| (a) Approach — stuck within ~2 m of target | **13** | 0.150 – 0.263 |
| (b) Transit — stuck mid-leg, far from target | **54** | 6.556 – 201.090 |
| (c) Recovery — stuck ITSELF being a recovery/retreat leg | **0** (see below) | n/a |

**Related but distinct fact, not double-counted into the 67**: of the 67,
**30 also failed their own subsequent recovery/retreat attempt** (the
retreat-to-last-good-position leg that follows every primary stuck event
also came back `stuck=True`). This is the closest real match to the
prompt's category (c) — not a stuck event in its own right among the 67,
but a second failure riding on top of 30 of them.

**Reading the split**: 81% (54/67) of stuck events are genuine
mid-transit failures, not near-target convergence trouble. Only 13/67
(19%) are the "basically arrived, just outside tolerance" case category
(a) describes — settle errors of 15-26 cm, a few cm past the 15 cm
tolerance, most plausibly resolved by loosening the stuck-detector's own
patience rather than a structural fix. **The transit category is where
the real problem is**, both by count and by the size of the miss (up to
201 m off target).

## Cross-track error

**Method, stated plainly (a real data limitation, not hidden)**: the
flight log records achieved position at the START and END of each leg,
not a continuous trajectory. "Cross-track error" here is therefore the
perpendicular distance from the leg's own final achieved/frozen position
to the straight line connecting the previous leg's achieved position and
the current leg's commanded target — a single endpoint sample per leg,
not an integral along the real flown path. This underestimates the true
maximum deviation during a leg that later returned toward the line, and
is exact for a leg that got stuck and never moved again (54 of the 67
transit cases fall in this second bucket).

| population | n | median | p95 | max |
|---|---:|---:|---:|---:|
| All 150 main-leg attempts (settled + stuck) | 150 | 0.133 m | 14.87 m | 34.44 m |
| Settled legs only | 82 | 0.129 m | 0.136 m | — |
| **Transit-stuck legs only (category b, 54 events)** | **54** | **6.31 m** | **20.19 m** | **34.44 m** |

Settled legs cluster tightly around the 0.15 m settle tolerance itself
(expected — a settled leg's final position IS close to the target,
which sits on the line by construction, so this number mostly measures
settle precision, not path deviation, and is reported for completeness
rather than as the diagnostic number). **The transit-stuck population is
the one that matters**: a median 6.3 m and a 95th-percentile 20.2 m
deviation from the straight line is far larger than a simple "clipped a
girder by a few cm" story — this is consistent with what was already
found earlier this project (a real PyBullet contact/penetration-response
event that displaces the airframe substantially, not a narrow miss), not
a small clearance-margin problem alone.

## What this means for B2

A margin fix sized from this p95 (airframe radius + 20.19 m) would be
absurd in this densely-packed corridor — it would reject nearly every
transit leg, not fix a narrow-miss problem. **The diagnosis does not
support "the margin was just slightly too small."** It supports two
different, real problems that need two different fixes:

1. **The 13 approach cases** (near-target, not converging) are a settle-
   tolerance/patience issue, not a clearance issue — cheapest fix is
   allowing more time or a slightly larger stuck-progress window right at
   the end of a leg, not touched by the margin/routing changes below.
2. **The 54 transit cases** are real structural encounters severe enough
   to physically displace the airframe by meters, not centimeters. A
   larger `TRANSIT_CLEARANCE_MARGIN_M` (still applied, using the p95
   figure as instructed) will reject far more candidate paths than
   before — which is the intended effect: those paths were never safe,
   the pre-fix segment check just wasn't catching them. The A* detour
   step is what has to recover the coverage that stricter rejection
   removes, not the margin number alone.
