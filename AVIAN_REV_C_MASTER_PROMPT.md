# AVIAN REV-C — Master Brief

**Environment realism, metro viaduct, and advanced city**
**Version 2** — supersedes v1. Stage 1 decisions locked; repo now assembled and green.

> **How to use this file.** It lives at the root of `/home/prince/avian_rev_c`.
> Open Claude Code there and paste the opening message in §0.

---

## 0. Opening message to paste into Claude Code

> I'm starting AVIAN REV-C Stage 1. Read `AVIAN_REV_C_MASTER_PROMPT.md` at the
> repo root — v2, with the Stage 1 decisions already locked in §5 and the scene
> rename in §2.7. Adopt the working rules in §3 verbatim.
>
> Before writing any code: confirm both gates still pass on this machine
> (`AVIAN_UAV/tests/run_all.py` → 46/46, and the environment's 32/32), then give
> me **A–F** — what exists, what works, what is broken, what is missing, what
> must change, what you recommend — for **Stage 1 only**, and ask me only the
> blocking questions.
>
> Do not start Stage 2 until I approve Stage 1 at its gate.

---

## 1. Where the project actually stands

Updated after the repo was assembled. This section is fact, not plan.

| | |
|---|---|
| Repo | `/home/prince/avian_rev_c` |
| Environment | `AVIAN_ENVIRONMENT/source/` ← `AVIAN_REV_B.zip` |
| Aircraft | `AVIAN_UAV/` ← `AVIAN_2_UAV_PHASE1.zip` (the signed-off variant — the only one carrying `cad/`) |
| UAV gate | **46/46**, genuinely reproduced on this machine |
| Environment gate | **32/32**, rebuilt from source on Blender 4.0.2 |
| Ground-truth drift | **0.00 mm** — V22 holds after a local rebuild |
| Triangles | **384,200** — matches the reference exactly |
| Objects | **13,835**, all uniquely named |

**Layout is flat, and that is canonical.** `AVIAN_UAV/simulation/`,
`AVIAN_UAV/description/`, `AVIAN_UAV/cad/`, `AVIAN_UAV/sensors/`,
`AVIAN_UAV/manipulator/`. The nested `avian/sim/` spelling in the other zips is
dead — do not reintroduce it.

### 1.1 Bugs already fixed during assembly (packaging drift, not logic)

1. `AVIAN_CAD_DIR` hardcoded to `/home/claude/avian` in `build_urdf.py` and
   `test_description.py`.
2. `test_description.py` and `make_phase1_report.py` hardcoded
   `avian/description/...` against a layout that is flat.
3. Nine files imported `from avian.sim…` / `avian.sensors…` /
   `avian.manipulator…` — a namespace package that does not exist here.
4. `test_sensors.py` and `test_manipulator.py` put `AVIAN_ENV_DIR` ahead of the
   UAV root on `sys.path`, so the environment's Blender `sensors.py` shadowed
   the UAV's PyBullet `sensors/` and crashed on `import bpy`.

None of these touched physics, control or ground truth — which is why the
numbers came back identical.

### 1.2 One bug still open, and it is the important one

`scripts/make_phase1_report.py` builds its report by reading `logs/*.json` off
disk and **silently falls back to a stale log when a suite never wrote one**:

```python
if not os.path.exists(p):
    return None          # ← no error, no warning
```

`tests/run_all.py` then calls it unconditionally, *after* printing the true
tally, so the last line on screen is the stale PASS:

```python
print(f"\nGATE: {tp} pass, {tf} fail")
r = subprocess.run([sys.executable, "scripts/make_phase1_report.py"], ...)
print(r.stdout.strip() or r.stderr.strip())      # ← prints last
sys.exit(0 if tf==0 else 1)
```

This was observed live: 5 of 6 suites failing while the report printed
`PASS 46/46`.

**Fix it as part of Stage 1**, and record it in the report's own
"defects found" section — that section exists for exactly this. The report must
stamp each log with a run id and **refuse** to report a log it did not just see
written. Working rule §3.3 is unenforceable until it does.

### 1.3 Recorded assumption: Blender 4.0.2, not 4.5 LTS

No pip `bpy` wheel exists for this machine's Python 3.12 (cp311 only), so builds
route through system Blender 4.0.2 via the `run_blender.py` launcher. A local
rebuild reproduced the ground truth at 0.00 mm drift, so 4.0.2 is *proven*
adequate for REV-B. It is **not** proven for REV-C, which adds new mesh and
material operations. Label this as an assumption in code per §3.5, and if any
REV-C feature needs a 4.5-only API, stop and say so rather than working around it.

---

## 2. The invariants — break any of these and the work is wrong

### 2.1 The 192 defects are frozen

`V22` requires **0 missing, 0 added, drift ≤ 1 mm** against
`BASELINE_REV_A_ground_truth.json`. It currently measures **0.00 mm**.

- Do not regenerate the road-bridge defect population.
- Do not change `SEED = 20260827`.
- Do not perturb any object a defect is anchored to — `damage.py` finds hosts by
  ray-cast, so moving a pier by a centimetre moves its defects.
- **Metro defects go in a separate namespace and a separate record list.** V22
  must continue to see exactly 192. Name them `MDEFECT_{kind}_{idx:03d}`.

### 2.2 Name prefixes are a cross-package API

`AVIAN_UAV/simulation/export_bridge_collision.py` selects collision primitives
**by name prefix**:

```python
STRUCTURAL_PREFIXES = ("BR_",)
SERVICE_PREFIXES    = ("AVI_DET_",)
EXCLUDE_PREFIXES    = ("DEFECT_", "AVI_AIRSPACE", …, "CITY_", "ENV_ROCK", …)
```

and `validate.py` **V10** tests airspace against structure matched on `"BR_"`.

**The trap:** a metro viaduct named outside those prefixes is invisible to both.
The UAV would fly straight through it in physics and no check would fire.

- Metro structural members are prefixed **`MB_`**.
- `MB_` **must be added to `STRUCTURAL_PREFIXES`** — a cross-package edit. Report it.
- `MB_` **must be added to V10's structure selector.**
- City detail keeps the `CITY_` prefix so it stays excluded from collision.

### 2.3 V03 is one object away from failing

```
V03  no floating or sunken ground objects
     measured: max |dz| 0.593 m over 7957 objects      limit: 0.60 m
```

**98.8% of tolerance.** Every new ground-placed object — street furniture,
kiosks, metro footings, parked vehicles — is a chance to break it. Place
everything on `terrain.height(x, y)` and re-run V03 after each stage.

### 2.4 Other checks new geometry can break

| Check | Guards | REV-C risk |
|---|---|---|
| `V07` | air draft ≥ 12.0 m over the channel (now 24.58 m) | metro crossing |
| `V10` | SAFE + RETURN airspace clear of structure | metro piers and deck |
| `V11` | under-bridge headroom ≥ 3.0 m (now 13.6–19.6 m) | metro substructure |
| `V13` | all 192 defects inside `x ∈ [1650, 2550]` | must stay true |
| `V16` | polygon budget | **raised to 15,000,000 — see §2.5** |
| `V19` | unique names across 13,835 objects | facade instancing |
| `V29` | mission graph connected, all sectors reachable | metro sectors |

### 2.5 The polygon budget rises to 15 M — with a recorded reason

Current **384,200** of a 6,000,000 ceiling. Raise V16 to **15,000,000** and
record in the check's `detail` string why it moved and who authorised it.
**Do not remove the guard.** Report the triangle count after every stage.

### 2.6 Render cost is measured, not assumed

Blender material complexity costs **dataset generation** (`dataset.py`, 4 passes)
and the validation renders. It does **not** cost the PyBullet flight loop, which
uses the exported collision JSON.

But 640×480 already costs ~300 ms/frame, and that is the binding constraint on
Phase 2's 30-minute mission budget. **Report frame cost before and after every
stage.** If Stage 1 more than doubles it, stop and report rather than continuing.

### 2.7 Scene rename — `AVIAN_SIC_REV_C.blend`

The REV-C scene is **`AVIAN_SIC_REV_C.blend`** (AVIAN Smart Infrastructure City).

- `build_scene_c.py` writes **`AVIAN_SIC_REV_C.blend`**. It must **not** overwrite
  `AVIAN_Smart_Infrastructure_City_REV_B.blend` — REV-B stays on disk as the
  fallback and the thing V22 is measured against.
- Write it to **`AVIAN_ENVIRONMENT/scene/`**, not next to the script. REV-B saving
  the `.blend` into `source/` is a wart; fix it for REV-C rather than inherit it.
  Create `scene/` if it does not exist.
- Find every reference before renaming anything:
  `grep -rn "Smart_Infrastructure_City" .` — expect hits in `build_scene_b.py`,
  `run_blender.py`, `SETUP.md`, and the environment's `docs/`.
- Leave REV-B's own references alone. Only REV-C code points at the new name.

---

## 3. Working rules — adopt these verbatim

1. **Inspect before building.** For any substantial task, first report
   *A. what exists · B. what works · C. what is broken · D. what is missing ·
   E. what must change · F. what you recommend.* Then ask only blocking questions.
2. **When a check fails:** stop → root cause → fix → rerun → report.
3. **Never hide failures.** Never claim something works unless it has been run.
   §1.2 is what happens when the reporting layer can break this rule by itself.
4. **Do not change acceptance criteria to make a failing check pass.** If a check
   is *wrong*, say so and record it as a check defect. The one sanctioned
   threshold change in REV-C is V16, and §2.5 says how to record it.
5. **Do not make unstated assumptions.** Every assumed constant is labelled as an
   assumption in code — including the Blender 4.0.2 note in §1.3.
6. **Upgrade in place; do not rebuild from scratch.** Do not regenerate the defect
   ground truth. Do not break the coordinate system. 1 BU = 1 m.
7. **Only give commands that actually exist and work.** No imaginary CLI.
8. **Do not mix stages.** Finish one, stop at the gate, wait for approval.
9. **Commit before each stage and at each gate.** A stage's work must be a
   reviewable diff. Tag the pre-REV-C state `rev-b-green`.

---

## 4. Architecture — REV-C is additive

`build_scene_b.py` states the pattern in its own docstring: REV-A modules are
imported and run **unchanged**; REV-B adds modules alongside them. REV-C does it
again. **Do not restructurally edit** `bridge.py`, `city.py`, `damage.py`,
`terrain.py`, `roads.py` or `params.py`. Add:

```
AVIAN_ENVIRONMENT/source/
  build_scene_c.py     master build — REV-A, then REV-B, then REV-C
  params_c.py          REV-C constants; imports params as P
  materials_c.py       facade, glass, signage, weathering, palette constants
  facade_c.py          window/balcony/AC geometry, LOD-banded
  metro.py             MB_ viaduct: box girder, piers, track, station
  metro_damage.py      MDEFECT_* population + its own ground-truth export
  streetscape_c.py     kiosks, hoardings, wires, bins, stalls
  zones_c.py           metro airspace + MSECTOR_* inspection sectors
  validate_c.py        V33+
```

`params.py` may gain **appended constants only**, under a `# ---- REV-C ----`
banner. Anything REV-C-specific goes in `params_c.py`.

Reuse `meshlib.py` (`box`, `cylinder`, `prism`, `grid`, `ribbon`, `link_dup`,
`join_objects`, `set_custom`, `tri_count`). **`link_dup` is how you instance
without paying for unique mesh data** — use it for every repeated facade element.

---

## 5. Stage 1 — materials, colour and micro-detail

### 5.1 Decisions — locked, do not re-ask

| | |
|---|---|
| **City character** | **South Mumbai / coastal metro** |
| **Lighting scope** | **Daylight only.** Night and monsoon must keep working, not look good. No emissive window grids this stage. |
| **Realism target** | **Photoreal, within Cycles CPU limits** — subject to the frame-cost obligation in §2.6 |

### 5.2 What is wrong today

`materials.py` builds 35 procedural materials from a good node toolkit
(`_noise`, `_ramp`, `_mix`, `_bump`, `_bsdf`, `_aerial`). The concrete, spall,
rebar and water shaders are genuinely good. **The city is not.** All 680
buildings share four flat Principled BSDFs:

```python
"bldg_a": simple("MAT_BUILDING_A", (0.315, 0.288, 0.252), 0.78),
"bldg_b": simple("MAT_BUILDING_B", (0.245, 0.238, 0.228), 0.80),
"bldg_c": simple("MAT_BUILDING_C", (0.352, 0.315, 0.272), 0.75),
"bldg_d": simple("MAT_BUILDING_D", (0.198, 0.205, 0.212), 0.72),
```

Base colour plus roughness, nothing else. `MAT_BUILDING_B` is **7% saturated**.
That is why the city reads as grey, and it is the single highest-value fix in
the whole stage.

### 5.3 The South Mumbai palette

This is the anchor for every colour decision. No generic choices.

- **Art Deco frontage** (Marine Drive, Oval Maidan): buttermilk, cream, pale
  ochre render; horizontal banding; curved corners; vertical fins.
- **Mid-century RCC**: exposed concrete gone grey-black, with heavy monsoon
  staining on every north and seaward face. **Black mould streaking below sills
  and slab edges is the most recognisable Bombay surface** — get it right and
  the scene reads immediately.
- **Chawls and walk-ups** behind the frontage: oxide red, ochre, deep green
  painted woodwork, wooden balcony railings, external stairs.
- **Stone**: Malad yellow basalt, Kota grey on older stock.
- **Rooftops**: blue tarpaulin, black Sintex water tanks, dish antennas, AC
  units — this is what makes an aerial view read as Indian rather than generic.
- **Sea haze**: `_aerial()` already implements aerial perspective. Every new
  material routes through it, and coastal haze is slightly stronger than a
  dry-inland default.

**What this choice buys the project:** a coastal city makes chloride-driven
rebar corrosion the obvious failure mode — exactly what `MAT_REBAR_CORRODED`
and `MAT_SPALL_FACE_RUST` already model. The defect population becomes
physically motivated by its own setting rather than arbitrary. Say so in the
build log.

### 5.4 Work

- **Facade shaders** in `materials_c.py`: window grid, floor banding, vertical
  staining below sills, plaster patchiness, and a **per-instance hue and
  weathering offset driven by `Object Info → Random`** so 680 buildings stop
  being four colours.
- **Concrete weathering driven by geometry**: runoff streaks below drains and
  joints, efflorescence at construction joints, dirt in ambient-occluded
  crevices via Geometry → Pointiness.
- **Asphalt**: wheel-path polishing, patch repairs, albedo change near joints.
- **Water**: keep `MAT_RIVER`'s structure; add bank turbidity and flow direction.
- **Do not touch** `crack_decal`, `spall_face`, `delamination_face`,
  `repair_patch`, `rebar` — ground-truth-bearing. Weather the concrete, asphalt
  and water shaders further; do not replace them.
- **Palette as named constants** in `materials_c.py`, each with a comment naming
  its real-world reference. Scattered RGB tuples are how a palette stops being
  a palette.

### 5.5 Stage 1 gate

- REV-B **32/32** unchanged, V22 still 0.00 mm.
- Triangle count reported and **materially unchanged** — this stage is shaders.
- **Frame cost at 640×480 reported before and after.**
- `make_phase1_report.py` stale-log bug fixed (§1.2) and recorded as a defect.
- `VIEW_01`, `VIEW_04`, `VIEW_05`, `VIEW_07` regenerated and shown.
- Committed. **Stop. Do not start Stage 2.**

---

## 6. Stage 2 — the metro viaduct

### 6.1 Layout — decided

**Parallel viaduct, separate pier line.** Not shared bents (that would perturb
the road bridge's piers and risk the frozen defect positions), not
double-decker (that would break V07/V11).

- Centreline offset in **Y**. The clear band is bounded by
  `CITY_SETBACK_Y = 70.0`, so roughly **y = +45 m** sits in clear ground.
  **Measure before committing** against `zones_b` airspace volumes and the
  SAFE/RETURN corridors, and report what you measured.
- Runs the research zone at minimum (`x ∈ [1650, 2550]`) and crosses the river.
- **Over the channel it must clear V07's 12.0 m air draft.** Give it its own
  main span over `x ∈ (2055, 2145)` — no pier in the channel.

### 6.2 Structure — model it like a real Indian elevated metro

- **Single-cell precast box girder**, not I-girders. This is the visual
  difference from a road bridge, and it gives a genuinely different inspection
  problem: a hollow box has an interior.
- **Single circular pier** per bent with a flared pier head. Typical span 25–31 m.
- Segmental joints, bearings, parapet, cable trough.
- **Track**: ballastless slab track, two rails, third rail or catenary masts.
- **At least one elevated station** in the research zone — station box,
  platforms, roof, staircases, concourse. Also a large GNSS shadow, which is a
  useful new flight condition.
- Prefix everything `MB_`, and tag each member with `avi_kind` matching the
  `STRUCTURAL_KINDS` vocabulary (`deck_box`, `girder`, `pier_column`,
  `pier_cap`, `bearing`, `parapet`, …) so the exporter classifies it with no
  new code path.

### 6.3 The cross-package edit

In `AVIAN_UAV/simulation/export_bridge_collision.py`:

```python
STRUCTURAL_PREFIXES = ("BR_", "MB_")
```

Re-export the collision asset and report the new primitive count against the
current **677**.

### 6.4 Airspace and sectors

In `zones_c.py`: `MSECTOR_A..F`; new classes `METRO_DECK_INSPECTION`,
`METRO_UNDERSIDE_INSPECTION`, `METRO_PIER_INSPECTION`, and — the interesting one
— `INTER_STRUCTURE_CORRIDOR` for the volume *between* the two bridges, a
confined, GNSS-degraded, multi-UAV-separation problem that does not exist in
REV-B. Mission markers and graph edges so V29 stays satisfied.

### 6.5 Metro defects

Separate population, separate export, separate baseline, prefix `MDEFECT_`.
**Every record carries all 44 fields the road-bridge ground truth carries** —
including the measured-visibility fields (`visible_fraction`,
`occlusion_measured`, `expected_rgb_visibility`, `detection_difficulty`,
`recommended_sensor`, …). Run them through `visibility.py`'s 61-direction
hemisphere exactly as the originals were. A metro defect without measured
visibility is a defect the planner cannot be honestly scored on.

Box-girder-specific types are what make this worth doing: segmental joint
leakage, efflorescence at joints, bearing distress, internal soffit cracking.

### 6.6 Stage 2 gate

REV-B 32/32 including V22 at 0.00 mm on the original 192; V33+ pass; collision
asset re-exported with primitive count reported; metro visible in two renders;
committed. **Stop.**

---

## 7. Stage 3 — the advanced city

### 7.1 Facade detail — hybrid by LOD band

`params.py` already has the machinery: `detail_at(x)` returns `HIGH`/`MED`/`LOW`
from `RESEARCH_X0/X1` and `LOD_MED_MARGIN = 500.0`. **Use it. Do not invent a
second LOD system.**

| Band | Treatment |
|---|---|
| `HIGH` — research zone ±500 m | Real geometry: recessed windows, balconies, AC units, parapet railings, signage |
| `MED` | Window recesses only, simplified balconies |
| `LOW` | Material-only facades, existing box massing |

Rationale: `UNDERSIDE_INSPECTION` and `PIER_INSPECTION` fly at **1.2–3.0 m
standoff**. A texture-only facade falls apart at that range. Beyond the research
zone nothing flies close, so geometry there is wasted.

Use `ML.link_dup` for every repeated element. Watch `V19` — instanced objects
still need unique *object* names; number them deterministically from the seed.

### 7.2 Building variety

Today: three styles (`SLAB`, `SETBACK`, `LOWRISE`), joined boxes, one material
each. Add, in `facade_c.py`: podium-and-tower; chawl/walk-up with external
stairs; informal rooftop additions; under-construction shells with scaffolding
and exposed rebar columns. **Ground-floor retail that differs from the floors
above** does more for street-level realism than anything else.

### 7.3 Streetscape

In `streetscape_c.py` — this is what makes it read as Bombay, not generic:
overhead wire bundles, transformers, meter boxes; hoardings and painted wall
advertising; roadside kiosks, market stalls, tarpaulins, handcarts;
autorickshaws and two-wheelers in the existing dynamic vehicle collection;
compound walls, gates, water tanks, dish antennas; median planting, dividers,
bus shelters.

Everything ground-placed goes on `terrain.height(x, y)` — **V03 has 7 mm of
headroom.**

### 7.4 Stage 3 gate

All checks pass, V16 reported against the 15 M ceiling, triangle count broken
down by band, frame cost reported, renders regenerated, committed.

---

## 8. New validation checks — part of the deliverable

Add to `validate_c.py`. A stage is not done until its checks exist and pass.

| ID | Check | Limit |
|---|---|---|
| V33 | metro deck clears the navigation channel | air draft ≥ 12.0 m, no pier in `x ∈ (2055, 2145)` |
| V34 | metro and road bridge do not intersect | 0 overlapping members; report min clearance |
| V35 | inter-structure corridor is flyable | ≥ 3.0 m every dimension |
| V36 | every `MB_` member carries a valid `avi_kind` | 100% classified |
| V37 | `MB_` reaches the collision exporter | primitive count > 0, reported |
| V38 | metro ground truth complete | all 44 fields × N records |
| V39 | metro defects have measured visibility | 100%, ≥ 3 difficulty levels |
| V40 | original 192 untouched | 0 added, 0 removed, 0.00 mm |
| V41 | facade LOD matches `detail_at()` | no HIGH outside the band, no LOW inside |
| V42 | triangle budget by band | reported per band; total < 15,000,000 |
| V43 | metro sectors reachable | all `MSECTOR_*`, no dangling edges |

---

## 9. Order of work, and what to report

```
Stage 1  materials + micro-detail   → gate → commit → STOP, await approval
Stage 2  metro viaduct + defects    → gate → commit → STOP, await approval
Stage 3  advanced city + street     → gate → commit → STOP
```

After every stage, report:

1. **Every** check, measured against limit — not a summary
2. Triangle count, delta from the previous stage, and where it went
3. Build wall-time, and **640×480 frame cost**
4. **Anything found by a check that failed** — the section that tells me whether
   the checks are any good
5. Renders

---

## 10. Commands

```bash
# Environment — from AVIAN_ENVIRONMENT/source/
blender --background --python-expr \
"import sys; sys.path.insert(0,'/home/prince/avian_rev_c/AVIAN_ENVIRONMENT/source'); \
import build_scene_b; build_scene_b.main()"

blender ../scene/AVIAN_SIC_REV_C.blend          # once Stage 1 has built it

# UAV — from AVIAN_UAV/
export AVIAN_CAD_DIR=/home/prince/avian_rev_c/AVIAN_UAV/cad
python3 tests/run_all.py                         # 46-check gate, ~4 min
python3 simulation/export_bridge_collision.py    # re-export after the metro
```

Environment: Blender **4.0.2** system install (see §1.3).
UAV: `pybullet` 3.2.7, `numpy`, `cadquery`/OCCT.

---

## 11. Reference — the numbers as they stand

| | REV-B |
|---|---|
| Corridor | 4,500 m, 137 spans, 138 piers, 2,055 bridge objects |
| Research zone | `x = 1650…2550` (900 m), 6 sectors |
| River | centre `x = 2100`, width 620 m, water `z = −2.0` |
| Navigation channel | `x ∈ (2055, 2145)`, 90 m clear, air draft 24.58 m |
| City | 680 buildings, 158 high-rise, 2,048 trees, 318 vehicles |
| Bridge services | 1,562 objects |
| Defects | **192** across 11 types, 36 non-repairable (18.8%) |
| Airspace | 43 volumes, 8 classes |
| Mission graph | 124 nodes, 136 edges |
| Scenarios | 16 (REV-B) + 9 (REV-A) = 25 |
| Objects | 13,835, uniquely named |
| **Triangles** | **384,200** of 6,000,000 → raised to 15 M for REV-C |
| Frame cost | 640×480 ≈ 300 ms · 1920×1080 ≈ 1.8 s (Cycles CPU) |
| Seed | `20260827` |
| Collision asset | 677 primitives from 22,850 triangles (33.8:1) |
| Validation | **32/32** · GT drift **0.00 mm** |
| UAV gate | **46/46** · mass 23.693 kg · T/W 2.31 · hold 3.0 cm RMS |
