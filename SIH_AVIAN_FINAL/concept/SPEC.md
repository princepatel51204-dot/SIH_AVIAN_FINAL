# SIH_AVIAN_FINAL — dimensions

The drawings are the idea. These are the numbers to build them from.

## Corridor

| | |
|---|---|
| Length | **360 m** (x = 0 → 360) |
| River | centre x = 180, width **90 m**, water z = **−2.0** |
| Research zone | x = **90 → 270** (180 m) — high-detail shaders only here |
| Seed | new baseline, `20260921` |

## Road bridge — y = 0

| | |
|---|---|
| Deck width | **14.0 m** (2 + 2 lanes @ 3.5) |
| Deck top | z = **14.0** · soffit z = **11.5** |
| Spans | 45, 45, 45, **90**, 45, 45, 45 = 360 m |
| Piers | x = 45, 90, 135, 225, 270, 315 — twin circular columns Ø1.7 m |
| Abutments | x = 0, 360 |
| Girders | 5 × I-girder, 1.6 m deep (2.5 m over the main span) |
| Air draft | **12.1 m** over water — clears the 12 m floor |
| Vehicles | ~16 cars, trucks, a bus |

**No pier in the river.** The 90 m main span crosses it clear.

## Metro viaduct — y = +28

| | |
|---|---|
| Deck width | **9.0 m** |
| Deck top | z = **19.0** · soffit z = **16.8** |
| Girder | single-cell precast box, 2.2 m deep |
| Spans | 12 × 30 m; **none in the river** — one 90 m span matching the road bridge |
| Piers | single circular Ø2.0 m with flared head |
| Track | ballastless slab, two rails, catenary masts |
| Train | **3 cars × 22 m = 66 m**, parked mid-span for the demo |

## The gap

**16.5 m clear** between road deck edge (y = 7) and metro deck edge (y = 23.5).
This is the `INTER_STRUCTURE_CORRIDOR` — confined, GNSS-degraded, and the one
flight condition that exists in no single-bridge scene.

## Defects — 96

| Type | n | Type | n |
|---|---|---|---|
| hairline crack | 24 | delamination | 8 |
| longitudinal | 12 | rebar exposed | 6 |
| transverse | 10 | corrosion stain | 10 |
| network | 8 | joint deterioration | 6 |
| spall | 12 | | |

Same discipline as before: measured visibility over the 61-direction
hemisphere, measured albedo contrast, all 46 ground-truth fields. Metro carries
its own `MDEFECT_` population on top.

## Airspace

| Class | Speed | Standoff |
|---|---|---|
| TRANSIT | 8.0 m/s | — |
| DECK_INSPECTION | 2.5 m/s | 6.0 m |
| UNDERSIDE_INSPECTION | 1.2 m/s | 2.5 m |
| PIER_INSPECTION | 1.5 m/s | 3.0 m |
| INTER_STRUCTURE_CORRIDOR | 1.0 m/s | 2.0 m |

## Fleet

| | SCANNER | REPAIRER |
|---|---|---|
| Arm | none | 6-DOF, 950 mm, nozzle |
| Sensors | 3× RGB, depth, LiDAR, thermal, IMU, GNSS | same |

## Scale, against REV-C

| | REV-C | SIH_AVIAN_FINAL |
|---|---|---|
| Length | 4 500 m | **360 m** |
| Spans | 137 | **7** |
| Objects | 14 656 | **~900** |
| Buildings | 680 | **0** |
| Defects | 192 + 60 | **96 + metro** |
| Build time | ~15 min | **~2 min est.** |

## What carries over unchanged

`meshlib`, `materials` + `materials_c`, `visibility.py`, `contrast_c.py`,
`viscache_c.py`, `avian_common/decompose.py`, `export_gazebo.py`,
`sabotage_c.py`, the phase-split orchestrator, and every working rule.

**Only the scene changes. None of the machinery does.**
