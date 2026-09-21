# SIH_AVIAN_FINAL

A compact 360 m inspection corridor — one road bridge, one parallel metro
viaduct, one river, no city — built in Blender and exported to Gazebo, with
96 measured, ground-truthed defects and a hand-placed hero shot. Replaces
the 4.5 km REV-C scene for the demo/pitch use case where the whole structure
needs to fit in one frame. See `SIH_AVIAN_FINAL_MASTER_PROMPT.md` for the
brief and `concept/SPEC.md` for every source dimension.

REV-C (`../AVIAN_ENVIRONMENT/`, `../AVIAN_UAV/`) is untouched by this work.

## Build it

```bash
export AVIAN_ENV_SRC=/home/prince/avian_rev_c/AVIAN_ENVIRONMENT/source
export AVIAN_COMMON_DIR=/home/prince/avian_rev_c/avian_common

cd /home/prince/avian_rev_c/SIH_AVIAN_FINAL

# Phase 1: geometry, materials, defects, measured visibility (~3 s)
blender --background --python run_blender.py -- source/build_final.py

# Phase 2: measured contrast, cameras, validation, sabotage, ground truth,
# baseline freeze (~1 min; add --render for the 8 named camera PNGs, ~8 min)
blender --background --python run_blender.py -- source/measure_final.py --render

# Collision asset (reuses AVIAN_UAV/simulation/export_bridge_collision.py
# unchanged) and the Gazebo SDF world
blender --background --python run_blender.py -- source/collision_final.py
blender --background --python run_blender.py -- source/export_gazebo_final.py
```

Phases are split into two separate Blender processes on purpose: damage
placement and visibility both ray-cast heavily, and doing that in the same
process as `contrast_c.py`'s EEVEE renders is a documented crash risk (see
`build_scene_c.py`'s own docstring in REV-C). Both phases are cheap enough
here (this scene is two orders of magnitude smaller than REV-C) that they
run as a single pass each rather than REV-C's further OS-subprocess split.

## See it in Gazebo

```bash
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=/home/prince/avian_rev_c/SIH_AVIAN_FINAL/gazebo/models
export SDF_PATH=$GZ_SIM_RESOURCE_PATH   # gz sdf -k needs this one specifically

gz sdf -k gazebo/worlds/sih_avian_final.sdf      # validate, exit
gz sim -v 4 gazebo/worlds/sih_avian_final.sdf    # GUI -- loads paused, press Play
gz sim -s -r --iterations 100 gazebo/worlds/sih_avian_final.sdf   # headless
cd gazebo && ./shoot_final.sh                    # headless screenshot
```

No world-origin shift: unlike REV-C (research zone at Blender x=2100,
shifted to Gazebo x=0), this corridor is already only 360 m and close to
the origin, so Gazebo coordinates equal Blender coordinates directly.

## What's here

| | |
|---|---|
| Road bridge | 360 m, 7 spans (45/45/45/**90**/45/45/45), 8 piers (twin Ø1.7 m columns), y=0, deck top 14.0–14.6 m (slight vertical curve, crest over the river) |
| Metro viaduct | y=28, deck top z=19.0 (flat), single-cell box girder (2.2 m approach / 3.2 m over the main span), single Ø2.0 m piers with flared heads, 11 piers / 10 spans, 5 catenary masts, no station (locked decision) |
| River | centre x=180, wetted channel 60 m + graded banks reaching grade exactly at x=135/225 (the piers flanking the main span) — bank-to-bank ≈90 m, matching SPEC.md |
| Inter-structure gap | 14.7 m clear between the road deck edge and the metro deck edge (VF15) |
| Traffic | 16 vehicles (13 cars, 2 trucks, 1 bus) on the road deck |
| Train | 3 cars × 22 m = 67 m, parked mid-span, static |
| Defects | **96 road** (95 from the SPEC.md population + 1 hand-placed hero) + **20 metro** (`MDEFECT_*`, own namespace) |
| Scene | 606 objects, 31,466 triangles, 112 materials |
| Cameras | 8 named views (`CAM_01_OVERVIEW` … `CAM_08_DECK`), rendered to `renders/` |
| Validation | **21/21 PASS** (`validate_final.py`) |
| Sabotage | **16/16 proven** capable of failing (`sabotage_final.py`) |
| Gazebo | 401 collision primitives, SDF world, `gz sdf -k` clean, loads and steps |

## The hero defect

`DEFECT_REBAR_EXPOSED_006` — a 1.40 m² severity-4 spall with 3 exposed,
corroded rebars, hand-placed on the outboard face of `BR_PIER_COL_004_1`
(the road pier at x=135, immediately flanking the main span) at z=6.0 m,
tagged `avi_hero=True`. It occupies one of REBAR_EXPOSED's 6 slots in the
96-defect population (the random target was set to 5, not 6, specifically
to leave this one for the hand-placed hero) — see `build_final.py`'s
"hand-placed hero defect" section and `params_final.py`'s DAMAGE comment.
Framed by `CAM_06_HERO_DEFECT`, pulled back to ~3 m so the cavity's rim and
its raked-light shadow are actually in frame (a dead-on close-up at 1.6 m
showed only the cavity floor).

## Assumptions made (working rule 5) — everything not fixed by SPEC.md

- **Road deck vertical curve**: SPEC.md's z=14.0 is the value AT the
  abutments; a gentle parabola crests at +0.6 m over the river (locked
  pre-flight decision 1). Reported, not asserted: crest 14.6, midspan
  soffit 11.775, air draft 13.775 m (≥ the 12.0 m floor).
- **River geometry**: SPEC.md's "90 m river width" is read as bank-to-bank
  (60 m wetted channel + 15 m bank run each side), so the banks reach grade
  exactly at the piers flanking the main span — "no pier in the river" by
  construction, not by post-hoc measurement.
- **Approach embankments**: a 30 m earthen fill ramp outside the modelled
  corridor (x<0, x>360), rising to 6.0 m at the abutment face; the abutment
  wall itself retains the rest of the height to deck level.
- **Metro main-span depth**: 3.2 m (deeper than the 2.2 m approach depth,
  for a 90 m clear span). Originally tried 4.5 m; that put the metro's
  main-span soffit (14.5) BELOW the road deck's own crest (14.6) — no
  actual collision (16.5 m apart in Y) but needlessly confusing, so it was
  reduced until the soffit cleared the crest with >1 m margin.
- **Metro defect count**: 20, scaled down from REV-C's 60 for a viaduct
  with roughly 1/4 the piers/spans — SPEC.md gives no metro defect table.
- **No metro station** (locked decision 2): `metro.py`'s `build_station()`
  is disabled via monkeypatch rather than never called, since the reused
  module always calls it internally.
- **2–3 distant building silhouettes** for scale on the horizon (locked
  decision 3) — simple `CITY_`-prefixed boxes, excluded from collision by
  prefix like REV-C's city detail.
- **Ground-object height tolerance (VF04)**: 1.2 m, not REV-C's 0.60 m.
  REV-C's limit was tuned to ITS rock prototype's embedding depth; this
  scene's randomly-rotated riprap/debris props measure up to ~0.97 m at the
  bbox-bottom, so 1.2 m is set from that measurement with headroom, not
  copied.

## What did NOT get built

Scoped out under the time available, and worth flagging rather than
silently omitting:

- **Micro-detail (Checkpoint C)**: formwork lines, construction-joint
  efflorescence banding, honeycombing, and >3 mm crack GEOMETRY (vs. decal)
  were not added on top of what `materials.py`/`materials_c.py` already do
  (weathering, grime-in-crevices via Pointiness, the crack decal shader
  itself). The concrete/asphalt/water weathering from Checkpoint B **is**
  applied (`materials_c.build_all`).
- **Airspace volumes and mission sectors** (`zones_final.py` — SPEC.md's
  five airspace classes, `MSECTOR_*`/`INTER_STRUCTURE_CORRIDOR` markers):
  not built. `validate_final.py`'s VF15 checks the inter-structure gap
  directly off BR_/MB_ bounding boxes instead of a proper airspace volume.
  A consequence: `export_bridge_collision.py`'s own internal checks C06
  ("6 road sectors") and C08 (metro pier diameter hardcoded to REV-C's
  2.8 m) fail against this scene — **not defects in this build**, but
  REV-C-specific assumptions baked into that shared, reused file, which
  was not edited (it is protected machinery both packages depend on).
- **Vehicle/train collision**: `VEH_` and `MB_TRAIN_CAR_` objects are not
  BR_/MB_-prefixed, so `export_bridge_collision.py` does not turn them into
  obstacles. Decorative only, matching how REV-C excludes vegetation/city
  detail from collision.

## Files

```
source/
  params_final.py       every dimension; bridge.py/damage.py-compatible
                         contract (they run unmodified via monkeypatch)
  terrain_final.py       river, banks, embankments, riprap, debris, trees
  build_final.py          phase 1: geometry + materials + defects + hero
  cameras_final.py        the 8 named cameras
  measure_final.py        phase 2: contrast + validation + sabotage + export
  validate_final.py       VF01-VF18, this scene's own limits
  sabotage_final.py       proves every VF check can fail
  collision_final.py      calls AVIAN_UAV's export_bridge_collision.py
  export_gazebo_final.py  SDF world writer
scene/                    build outputs (gitignored: *.blend, handoff JSON)
  AVIAN_defect_ground_truth_FINAL.{json,csv}
  AVIAN_metro_ground_truth_FINAL.{json,csv}
  BASELINE_FINAL_ground_truth.json   frozen at first successful build
  AVIAN_scene_stats_FINAL.json
  collision/avian_bridge_collision.json
renders/                  the 8 named camera views
gazebo/
  worlds/sih_avian_final.sdf
  models/avian_final_{road,metro,terrain,defects}/
  shoot_final.sh           headless screenshot
  screenshots/
concept/                  the six source drawings + SPEC.md
```
