# SIH_AVIAN_FINAL

A compact 360 m inspection corridor — one road bridge, one parallel metro
viaduct, one river, no city — built in Blender and exported to Gazebo, with
96 measured, ground-truthed defects, a hand-placed hero shot, real vehicle/
train geometry, and a two-pad drone base. Replaces the 4.5 km REV-C scene for
the demo/pitch use case where the whole structure needs to fit in one frame.
See `SIH_AVIAN_FINAL_MASTER_PROMPT.md` for the brief and `concept/SPEC.md`
for every source dimension.

REV-C (`../AVIAN_ENVIRONMENT/`) is untouched by this work. `AVIAN_UAV/` has
exactly one additive line changed — see "The drone base" below.

## Build it

```bash
export AVIAN_ENV_SRC=/home/prince/avian_rev_c/AVIAN_ENVIRONMENT/source
export AVIAN_COMMON_DIR=/home/prince/avian_rev_c/avian_common

cd /home/prince/avian_rev_c/SIH_AVIAN_FINAL

# Phase 1: geometry, materials, defects, measured visibility (~3 s)
blender --background --python run_blender.py -- source/build_final.py

# Collision asset (reuses AVIAN_UAV/simulation/export_bridge_collision.py
# unchanged) -- run BEFORE phase 2 so VF22 (landing pads reach collision)
# has a fresh file to check against
blender --background --python run_blender.py -- source/collision_final.py

# Phase 2: measured contrast, cameras, validation, sabotage, ground truth,
# baseline freeze (~1 min; add --render for the 9 named camera PNGs, ~10 min)
blender --background --python run_blender.py -- source/measure_final.py --render

# Gazebo SDF world -- needs the ground truth JSON phase 2 just wrote (for
# defect markers), so it runs AFTER measure_final.py, not before
blender --background --python run_blender.py -- source/export_gazebo_final.py

# Re-run phase 2 once more so VF26/VF27 (SDF material coverage / colour
# variety) check against a fresh export instead of SKIPping -- there is a
# real circular dependency here (the SDF needs ground truth, which
# measure_final.py produces, so the very first measure_final.py pass can
# only SKIP those two) and this is the acyclic way through it, not a bug
# to fix later.
blender --background --python run_blender.py -- source/measure_final.py
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
| Waterline staining | dark algal band + pale efflorescence bloom, on the 4 road + 2 metro pier columns nearest the river |
| Inter-structure gap | 14.7 m clear between the road deck edge and the metro deck edge (VF15) |
| Traffic | 18 vehicles: 13 real-silhouette cars (bonnet/cabin/boot, raked windscreen, 4 wheels, glass), 2 trucks (cab + box body, 6 wheels), 1 bus (window band, 6 wheels), 2 auto-rickshaws |
| Train | Real 3-car EMU, 22 m/car, raked cab noses on cars 1/3, window band + 4 door pairs/side, 2 bogies/car, roof AC units + a diamond pantograph, cream body with a Mumbai-Metro-blue stripe |
| Drone base | Two 6×6 m landing pads (SCANNER, REPAIRER), 12 m apart, near the x=0 abutment — painted H + circle, kerb, corner markers, equipment cabin, mast + windsock, charging docks |
| Micro-detail | Formwork lines + tie-hole patches + honeycombing on the 4 HIGH-band piers (both structures), chamfered pier-cap arrises, 204 parapet posts, crack-relief geometry cut into 7 cracks over 3 mm |
| Defects | **96 road** (95 from the SPEC.md population + 1 hand-placed hero) + **20 metro** (`MDEFECT_*`, own namespace) — untouched by this pass, baseline drift 0.000 mm |
| Scene | 1,514 objects, 60,206 triangles (31× under the 2,000,000 budget), 121 materials |
| Cameras | 9 named views (`CAM_01_OVERVIEW` … `CAM_08_DECK`, `CAM_09_BASE`), rendered to `renders/` |
| Validation | **27/27 PASS** (`validate_final.py`) |
| Sabotage | **22/22 proven** capable of failing (`sabotage_final.py`) |
| Gazebo | 483 collision primitives (incl. both landing pads) + 173 visual-only (vehicles, vegetation), 26 distinct flat colours resolved from real Blender materials, SDF world, `gz sdf -k` clean, loads and steps |

## The drone base — and the collision trap

Two 6×6 m pads, `AVI_BASE_SCANNER` and `AVI_BASE_REPAIRER`, on the flat bank
south of the road bridge (x=20, y=−30/−18 — clear of the abutment/wing walls,
clear of the river, 12 m apart). Each pad deck carries `avi_kind=
"landing_pad"`, `avi_base_role="SCANNER"|"REPAIRER"`, and `avi_pad_centre_m`.

**The trap the brief called out is real and was caught, not assumed away.**
`export_bridge_collision.py`'s `EXCLUDE_PREFIXES` contains `"AVI_HOME"` — a
pad named `AVI_HOME_*` would render, the SDF would parse, the world would
load, and a UAV would fall straight through it on first spawn, because
nothing in a visual check would ever catch a missing *collision* primitive.
Two things make these pads land on solid ground instead of merely avoiding
that one string by luck:

1. `"landing_pad": "BOX"` was added to `STRUCTURAL_KINDS` in
   `AVIAN_UAV/simulation/export_bridge_collision.py` (the one line changed
   outside this project — purely additive, inert for REV-C, which has no
   `landing_pad` objects). Classification now happens on `avi_kind`, not on
   name prefix, which is the robust path.
2. `export_gazebo_final.py`'s model groups originally only matched `BR_`/
   `MB_`/`ENV` prefixes — a first pass had the pads correctly reaching the
   *collision JSON* (VF22 passed) while silently **missing from the actual
   Gazebo SDF world**, because none of those three groups matched
   `AVI_BASE_`. Caught by eye (the base didn't appear in a screenshot),
   fixed by adding a fourth `avian_final_base` group. This is exactly why
   VF22 checks the collision JSON's primitive *count*, not a render: a
   missing SDF group is a rendering gap; a missing collision primitive is
   a physics gap, and the two failed independently of each other here.

VF19–22 check, respectively: both pads exist, both carry correct tags,
both sit exactly on `terrain_final.height()` (0.0000 m drift — placed
directly from it, not just checked against it), and both reach the
*exported* collision JSON by primitive count. All four are sabotage-tested.

## Gazebo materials — read from the live node graph, not a second table

The first Gazebo export had `<material>` blocks (every visual had one), but
their colour came from a per-`avi_kind` guess table, not from the actual
Blender material — and vehicles/train/vegetation weren't in the collision
export at all, so they were completely absent from the world. The result
read as "everything is grey" even though materials existed technically.

`export_gazebo_final.py` now resolves each visual's flat colour from the
**real** Blender material at export time:

1. Base Color unlinked → use its constant directly (`water()` sets one).
2. Base Color linked → walk the node graph backward for the first authored
   constant: a `ShaderNodeRGB`, a `ShaderNodeValToRGB` (stops averaged), or
   an **unlinked colour socket on any node along the way** — `spall_face()`'s
   rust tint, for instance, lives in a Mix node's unwired "B" input, not a
   separate node, so the walk checks each node's own sockets before
   recursing deeper, or it would find the wrong (aggregate/grit) colour
   instead of the rust one.
3. Nothing resolvable → mid-grey, logged by material name, once.

This reads the graph `materials.py` actually built, every time — it cannot
drift from a hand-typed second copy of the same numbers the way a parallel
table would. Two real bugs surfaced while wiring this up, both now fixed:

- The hero defect's marker was supposed to turn gold (`HERO_COLOUR`), but
  the check read `avi_hero` off the **ground-truth JSON record**, which
  never had that field — only the Blender *object*'s custom property does
  (`ML.set_custom(hero_ob, {"avi_hero": True, ...})` in `build_final.py`).
  Fixed to check the object; the hero now keeps its real rust colour and a
  bigger marker radius (0.45 m vs 0.25 m) instead of a distinct paint colour
  — "read as damage from across the river" means visible at range, not
  differently coloured.
- The collision exporter *synthesizes* simplified `ENV_GROUND_PLANE`/
  `ENV_WATER_SURFACE` box primitives for the terrain and river (there is no
  Blender object by those names — the real ones are `ENV_TERRAIN`/
  `ENV_RIVER_WATER`), so name-based water detection silently never matched
  and both fell to default grey. Fixed to also match by primitive `kind`.

Water's colour (blue-green) and its `<transparency>0.35</transparency>` are
the one deliberate override, not derived: `water()`'s real shader is
realistically murky silt (see its own docstring), which is correct for a
photoreal render and reads as mud in a flat-colour schematic. `<transparency>`
also has no Blender-shader analogue to derive from. Both were also caught
being placed as a **child of `<material>`** on the first pass — `gz sdf -k`
tolerates it with a schema warning rather than a failure ("not defined in
SDF... copying as children"), which only shows up if you read `gz`'s output
rather than trusting "it loaded"; `<transparency>` is schema-correct as a
sibling of `<material>`, a direct child of `<visual>`.

Vehicles and vegetation reach the world through two new **visual-only**
SDF groups (`avian_final_vehicles`, `avian_final_vegetation`) built straight
from the live scene via `avian_common.decompose.obb()` — the same shared
box-fitting the collision exporter uses, so there is still only one
implementation of it. They carry no `<collision>`, matching the README's own
"decorative, not obstacles" stance for traffic and trees.

**VF26** checks every SDF visual carries a `<material>` block; **VF27**
checks the world has ≥8 distinct diffuse colours. VF26 alone is the
check-that-cannot-fail shape this project keeps finding — it would pass
100% even if every material resolved to the same fallback grey, which is
exactly what VF27 exists to catch. Both are sabotage-tested: forcing every
material to fallback grey (VF27's sabotage) and stripping all `<material>`
blocks from one exported file (VF26's) each flip PASS→FAIL and are restored
byte-for-byte afterward.

## The hero defect

`DEFECT_REBAR_EXPOSED_006` — a 1.40 m² severity-4 spall with 3 exposed,
corroded rebars, hand-placed on the outboard face of `BR_PIER_COL_004_1`
(the road pier at x=135, immediately flanking the main span) at z=6.0 m,
tagged `avi_hero=True`. It occupies one of REBAR_EXPOSED's 6 slots in the
96-defect population (the random target was set to 5, not 6, specifically
to leave this one for the hand-placed hero). Framed by `CAM_06_HERO_DEFECT`,
pulled back to ~3 m so the cavity's rim and its raked-light shadow are
actually in frame (a dead-on close-up at 1.6 m showed only the cavity floor).

## A bug this pass caught in its own first draft

The metro train's body/window/door/bogie geometry is built in a local frame
with Y=0 as the track centreline, meant to be shifted onto the real
centreline (y=28) once, in `train_final.build()`. The first version of that
function only applied the Z shift (onto the rail top) and forgot Y entirely
— the whole train sat at y≈0, **on top of the road bridge**, not the metro
deck 28 m away. `VF15` (inter-structure corridor clear of structure) caught
it immediately: a re-run reported the metro's own bounding box reaching
y=−1.49, deep in the road bridge's territory. Fixed by adding an explicit
`centre_y` parameter; documented in `train_final.py` itself as a warning
against forgetting it again.

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
  decision 3) — dim, hazed via `_aerial()`, flat roof + parapet, excluded
  from collision by the `CITY_` prefix like REV-C's own city detail.
- **Ground-object height tolerance (VF04)**: 1.2 m, not REV-C's 0.60 m.
  REV-C's limit was tuned to ITS rock prototype's embedding depth; this
  scene's randomly-rotated riprap/debris props measure up to ~0.97 m at the
  bbox-bottom, so 1.2 m is set from that measurement with headroom, not
  copied.
- **Drone base position**: x=20 (near the x=0 abutment but clear of its
  wing walls), y=−30/−18 (south of the road bridge, clear of its edge by
  ≥11 m). Not specified further by the brief.
- **Train livery**: cream body, Mumbai-Metro-blue stripe — a specific,
  named choice (materials_final.py's `TRAIN_BODY`/`TRAIN_STRIPE`), picked
  to match the South Mumbai / coastal-metro city character `materials_c.py`
  already establishes, and modelled on Mumbai Metro Line 1's real livery.
- **Vehicle body colours**: 8 named colours (silver, white, red, dark blue,
  black, taxi yellow, grey-green, maroon) via one material driven by
  `Object Info → Random`, not 8 separate material datablocks.
- **Micro-detail is additive, not a groove**: formwork lines are raised
  rings (a proud line reads the same under a raking sun as a grooved one,
  without a boolean), and every micro-detail object is prefixed `_MD_`
  specifically so `damage.py`'s ray-cast host resolution skips it — without
  that, a formwork ring or tie-hole patch sitting right next to a pier
  column could get ray-cast-selected as a defect's host instead of the
  column itself.

## What did NOT get built

- **Airspace volumes and mission sectors** (`zones_final.py` — SPEC.md's
  five airspace classes, `MSECTOR_*`/`INTER_STRUCTURE_CORRIDOR` markers):
  not built. `validate_final.py`'s VF15 checks the inter-structure gap
  directly off BR_/MB_ bounding boxes instead of a proper airspace volume.
  A consequence: `export_bridge_collision.py`'s own internal checks C06
  ("6 road sectors") and C08 (metro pier diameter hardcoded to REV-C's
  2.8 m) fail against this scene — **not defects in this build**, but
  REV-C-specific assumptions baked into that shared, reused file, which
  was not (and should not be) edited for this beyond the one additive
  `STRUCTURAL_KINDS` entry the drone base needed.
- **Vehicle/train collision**: `VEH_`/`MB_TRAIN_*` objects are not
  BR_/MB_-prefixed and carry no `avi_kind` in `STRUCTURAL_KINDS`, so
  `export_bridge_collision.py` does not turn them into obstacles.
  Decorative only, matching how REV-C excludes vegetation/city detail.
- **Construction-joint efflorescence bleed as its own geometry pass**: the
  waterline staining and `materials_c.py`'s existing efflorescence-at-joints
  shader layer cover this; no separate construction-joint object was added.

## Files

```
source/
  params_final.py         every dimension; bridge.py/damage.py-compatible
                           contract (they run unmodified via monkeypatch)
  terrain_final.py         river, banks, embankments, riprap, debris, trees,
                           distant silhouettes
  materials_final.py       vehicle body (random colour)/tyre/trim, rickshaw,
                           train livery, waterline staining, silhouette
  vehicles_final.py        real car/truck/bus/auto-rickshaw geometry
  train_final.py           real 3-car EMU geometry
  base_final.py            the two landing pads + base furniture
  microdetail_final.py     formwork/tie-holes/honeycombing/chamfers/
                           parapet posts/crack-relief geometry
  build_final.py           phase 1: geometry + materials + defects + hero
  cameras_final.py         the 9 named cameras
  measure_final.py         phase 2: contrast + validation + sabotage + export
  validate_final.py        VF01-VF27, this scene's own limits
  sabotage_final.py        proves every VF check can fail
  collision_final.py       calls AVIAN_UAV's export_bridge_collision.py
  export_gazebo_final.py   SDF world writer; resolves flat colours from the
                           live Blender material graph, not a lookup table
scene/                     build outputs (gitignored: *.blend, handoff JSON)
  AVIAN_defect_ground_truth_FINAL.{json,csv}
  AVIAN_metro_ground_truth_FINAL.{json,csv}
  BASELINE_FINAL_ground_truth.json   frozen at first successful build
  AVIAN_scene_stats_FINAL.json
  collision/avian_bridge_collision.json
renders/                   the 9 named camera views
gazebo/
  worlds/sih_avian_final.sdf
  models/avian_final_{road,metro,terrain,base,defects}/
  shoot_final.sh            headless screenshot
  screenshots/
concept/                   the six source drawings + SPEC.md
```
