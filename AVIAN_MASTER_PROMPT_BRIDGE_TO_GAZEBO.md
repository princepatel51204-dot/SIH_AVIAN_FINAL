# AVIAN — Master Prompt: Bridge + Metro → Gazebo

**Focus: the bridge and the metro only.** The city is not being advanced.
**Goal: the environment finished and loadable in Gazebo, with commands to see it.**

```
Stage 2   Metro viaduct                    → gate → commit → STOP
Stage 3   CUT — colour tuning only, folded into Stage 2
Stage 4   Gazebo export + VIEW IT          → gate → commit → STOP
Stage 5   ROS 2 + aircraft in Gazebo       → gate → commit → STOP
```

This supersedes §6 and §7 of `AVIAN_REV_C_MASTER_PROMPT.md`. §1–§4 of that
brief — invariants, working rules, additive architecture — still bind unchanged.

---

## Paste this into Claude Code

> **AVIAN: finish the bridge and metro environment, then put it in Gazebo. Read
> `AVIAN_MASTER_PROMPT_BRIDGE_TO_GAZEBO.md` in full. §1–§4 of
> `AVIAN_REV_C_MASTER_PROMPT.md` still bind. §6 and §7 of that brief are
> replaced by this file.**
>
> **Scope: bridge and metro only. The city stays as it is — Stage 3 is cut.**
>
> The deliverable I most want is **Stage 4**: the environment loaded in Gazebo,
> with a command I can run to look at it. Everything before that serves it.
>
> Start with Stage 2. Gate at each stage, commit, and stop for approval before
> the next.

---

## Decisions already made — do not re-ask

| Question | Answer |
|---|---|
| Advance the city? | **No.** Stage 3 cut. |
| Metro position | Parallel viaduct, own pier line, centreline **y = +45 m** |
| Metro deck height | **z ≈ 30–34 m**, hard ceiling **z < 55 m** (§2.2) |
| Metro an inspection target? | Yes — own defects, sectors, airspace, ground truth |
| Shared decomposer | `avian_common/` at repo root, `AVIAN_COMMON_DIR` (§4.1) |
| Physics engine | **PyBullet stays the validated reference.** Gazebo is added, not substituted |
| Gazebo version | **Harmonic (gz-sim 8)** — verify before installing. Classic is EOL |
| Fleet for Stage 5 | **2 heterogeneous** — one scanner, one repairer (§5.4) |
| Polygon budget | 15,000,000 (V16 raised, reason recorded) |

---

# STAGE 2 — The metro viaduct

## 2.1 Already confirmed by measurement — do not re-measure

- `BR_` structure reaches **y = +12.00 m** max → 33 m lateral clearance at y=+45
- REV-B sector, deck-inspection and separation volumes stop at **y = +24** → 21 m clear
- Ground clutter (`RD_BARRIER`, `CITY_VEH`) ends at **y = +25.7** → footings land clear
- `STRUCTURAL_KINDS` already holds `deck_box → BOX` and `pier_column → CYL`, and
  the exporter tests `kind in STRUCTURAL_KINDS` **before** prefix — so a tagged
  box girder and circular pier classify with no new code path
- **`MB_` is still required**: rails, catenary masts and station parts have no
  matching `avi_kind`, so without the prefix they fail `is_struct` and vanish
  from collision silently

## 2.2 The ceiling constraint

`AVI_TRANSIT_AIRSPACE_MAIN` spans **y ∈ [−280, +280] at z ∈ [55, 100]** and
passes directly over y = +45.

**The metro's entire vertical envelope stays below z = 55 m** — deck, parapet,
catenary masts, station roof. Deck at **z ≈ 30–34 m**: above the road soffit,
clear of the deck-inspection volumes that top out at z = 41 only within y ≤ 24.
Assert the ceiling with a check, do not rely on intent.

## 2.3 Structure

- **Single-cell precast box girder**, not I-girders. The hollow interior is a
  confined inspection space that does not exist anywhere in REV-B.
- **Single circular pier** per bent, flared pier head. Typical span 25–31 m.
- Segmental joints, bearings, parapet, cable trough.
- **Track**: ballastless slab track, two rails, catenary masts.
- **One elevated station** in the research zone — box, platforms, roof, stairs,
  concourse. A large GNSS shadow, which is a useful new flight condition.
  **Roof below z = 55 m.**
- Own main span over `x ∈ (2055, 2145)` clearing V07's 12 m air draft.
  **No pier in the navigation channel.**
- Prefix `MB_`; tag every member with `avi_kind` from `STRUCTURAL_KINDS`.

## 2.4 Cross-package edit

```python
# AVIAN_UAV/simulation/export_bridge_collision.py
STRUCTURAL_PREFIXES = ("BR_", "MB_")
```

Add `MB_` to V10's structure selector in `validate.py`. Re-export and report the
new primitive count against the current **677**.

## 2.5 Metro defects

Separate population, separate export, separate baseline, prefix `MDEFECT_`.
**V22 must still see exactly 192.**

Every record carries **all 46 fields** — including the Stage 1b contrast fields
(`defect_background_contrast`, `contrast_limited`) and the measured-visibility
fields. Run them through `visibility.py`'s 61-direction hemisphere and the EEVEE
contrast pass exactly as the originals were.

Box-girder types are what make this worth doing: segmental joint leakage,
efflorescence at joints, bearing distress, internal soffit cracking.

## 2.6 Airspace and sectors

In `zones_c.py`: `MSECTOR_A..F`, plus `METRO_DECK_INSPECTION`,
`METRO_UNDERSIDE_INSPECTION`, `METRO_PIER_INSPECTION`, `METRO_INTERIOR` (box
girder interior), and **`INTER_STRUCTURE_CORRIDOR`** — the volume between the two
bridges, a confined, GNSS-degraded, multi-UAV-separation problem that does not
exist in REV-B and is the most interesting new flight condition in REV-C.
Mission markers and graph edges so V29 stays satisfied.

## 2.7 Folded in from the cut Stage 3 — both time-boxed to one pass

**Facade colour balance.** Facades read cool blue-grey because the glass tint
dominates at this window density. Reduce the tint, warm the render tones, show a
before/after of `VIEW_01`. One pass; if it doesn't land, record and move on.

**The Pointiness knob.** `CONCRETE_WEATHER_STRENGTH` has no authority — grime is
Pointiness-driven, Pointiness is ~0.5 on the flat girder webs where defects live,
so the sweep moves mean contrast only 0.0798 → 0.0822. The metro box girder needs
weathering anyway, so re-drive the grime off a **large-scale noise mask** with
authority on flat faces, keeping Pointiness for edges and crevices only. Re-run
the sweep and report whether the knob now moves contrast. **One attempt** — if it
still has no authority, record it as inert. The substantive result is already in
hand: ~30% of defects are contrast-limited even on clean concrete.

## 2.8 Stage 2 gate

| Check | Requirement |
|---|---|
| V22 | 0.00 mm on the original 192, 0 added, 0 removed |
| V33 | metro clears the channel, air draft ≥ 12.0 m, no pier in `x ∈ (2055, 2145)` |
| V34 | metro and road bridge do not intersect; **metro max z < 55.0 m** |
| V35 | inter-structure corridor flyable, ≥ 3.0 m every dimension |
| V36 | every `MB_` member carries a valid `avi_kind` |
| V37 | `MB_` reaches the collision exporter; count reported vs 677 |
| V38 | metro ground truth complete — all 46 fields × N records |
| V39 | metro defects have measured visibility and contrast, ≥ 3 difficulty levels |
| V40 | original 192 untouched |
| V43 | `MSECTOR_*` reachable, no dangling edges |
| V03 | still ≤ 0.60 m — **7 mm headroom, metro footings are the risk** |
| V16 | triangles reported against the 15 M ceiling |
| — | Cycles cost per view before/after; metro visible in two renders |

Commit. **Stop.**

---

# STAGE 4 — Gazebo export, and seeing it

**This is the deliverable that matters most. Treat it as the priority, not a
formality.** The stage is not done until I can run one command and look at the
bridge in Gazebo.

## 4.1 The shared decomposer — decision made

`AVIAN_ENVIRONMENT` and `AVIAN_UAV` have no common import path, and `sys.path`
insertion from inside each caller is the wrong answer.

**Create `avian_common/` at the repo root**, holding the collision decomposition
lifted out of `export_bridge_collision.py`. Both packages import it via an
`AVIAN_COMMON_DIR` environment variable, following the `AVIAN_CAD_DIR` precedent.

Why an env var and not `pip install -e`: the decomposer must import from **two
different interpreters** — system `python3` for PyBullet, Blender's bundled
Python for the environment. A pip install would have to be done twice, into two
environments, and would drift. One directory, one variable, documented in
`SETUP.md`.

**Do not write a second decomposer.** Two decomposers are two sources of the same
numbers and they drift the first time a member changes — the same rule that makes
the URDF generated rather than hand-written.

## 4.2 Output layout — standard Gazebo, not improvised

```
AVIAN_ENVIRONMENT/gazebo/
  worlds/
    avian_sic.sdf              the world
  models/
    avian_bridge/
      model.config
      model.sdf                BR_* collision + visual
      meshes/*.dae
    avian_metro/
      model.config
      model.sdf                MB_* collision + visual
      meshes/*.dae
    avian_terrain/
    avian_defects/
      model.config
      model.sdf                one <model> per defect, named as in Blender
```

Meshes are referenced by `model://avian_bridge/meshes/...`, resolved through
`GZ_SIM_RESOURCE_PATH`. Never absolute paths — the world must be portable.

## 4.3 The export

`export_gazebo.py` in `AVIAN_ENVIRONMENT/source/`.

- **Collision**: the same primitive set PyBullet gets, as SDF `<collision>`.
  Assert the count equals what `export_bridge_collision.py` produces from the
  same scene.
- **Visual**: per-collection COLLADA or glTF, decimated by LOD band.
- **Scope**: export the physics corridor `x ∈ [1550, 2650]`, **not the full
  4.5 km**. Stage 1b already hit an OOM kill with two Blender processes; Gazebo
  with a 4.5 km world will not fit. Make the corridor a parameter, record the
  default, and report the resulting file sizes.
- **Origin**: shift the world so the research-zone centre (`x = 2100`) sits near
  the Gazebo origin. A model at x = 2100 m in float32 loses precision and the
  GUI camera starts a kilometre from anything. **Record the offset in the world
  file as a comment and in the manifest** — every later pose depends on it.
- **Frames**: Blender and Gazebo are both Z-up, metres, right-handed. No
  conversion should be needed — **assert it, do not assume it**, and fail loudly
  if any exported pose disagrees with the Blender world matrix.
- **THE ORIGIN TRAP — added after Stage 2 found it.** `meshlib.box()` bakes the
  centre into the vertex coordinates and leaves the object origin at (0,0,0), so
  `matrix_world.translation` reads **zero for every object meshlib built** —
  which is most of the scene: `BR_*`, `MB_*`, terrain, city, all of it. Every
  SDF `<pose>` must therefore come from the **world-space bounding-box centre**,
  the way `export_bridge_collision.py`'s `_obb()` already computes it
  (`matrix_world @ ob.bound_box`). Reaching for the object origin anywhere in
  `export_gazebo.py` produces a world that parses cleanly, loads without error,
  and stacks ~1,400 models on top of each other at the world origin. The
  PyBullet path was never affected — `_obb()` was always right — so the 46/46
  gate is unharmed.
- **Defects**: each exported as its own named SDF model at its ground-truth pose,
  named identically to the Blender object, so a Gazebo-side detector can be
  scored against the same answer key.
- **Rendering in this process**: if any render is needed, use **EEVEE**. Cycles
  segfaults in Embree in any process that has run `scene.ray_cast`, and
  `open_mainfile` does not tear down the Cycles device.

## 4.4 Make it viewable — required, not optional

Write `AVIAN_ENVIRONMENT/gazebo/view.sh`, executable, that sets
`GZ_SIM_RESOURCE_PATH` and launches the world. **Run it yourself and confirm the
bridge appears before reporting the stage done.** Take a screenshot from the
Gazebo GUI and show it to me.

Also add the four camera poses to the world as `<gui><camera_pose>` or document
them in `gazebo/README.md`, so I can jump straight to:

1. Whole corridor, from above
2. River crossing, from the water
3. Under the deck — the soffit and girders
4. The inter-structure corridor, between road bridge and metro

## 4.5 Stage 4 gate

| ID | Check | Limit |
|---|---|---|
| V44 | SDF world parses | `gz sdf -k` exits 0 |
| V45 | SDF collision count equals PyBullet's | exact match |
| V46 | exported poses match Blender world **bounding-box centres** | ≤ 1 mm, ≤ 0.01°, **and a non-degenerate pose spread** |
| V47 | every defect exported and addressable by name | 192 + metro count |
| V48 | world loads in Gazebo | `gz sim -s -r --iterations 100` exits 0 |
| V49 | memory on load | peak RSS reported |
| V50 | no absolute paths in any SDF | 0 matches for `/home` |

**V46 must be written so it can fail.** Comparing object origins would compare
zero against zero and pass vacuously for all 677+ members — the same shape as
V33, which would have found no pier in the channel no matter where the piers
were. So V46 asserts the **spread** of exported poses is non-degenerate before
comparing, and states the measured envelope in the check detail, so a stacked
export is visible in the report instead of hidden behind a green tick.

**A named pattern, now that it has happened three times** (S14, E05, V33):
*a check that reads a quantity which can be uniformly zero needs an assertion
that it isn't.* Apply it to V45 and V47 as well — a count of zero exported
primitives or zero defects must not read as agreement.

**Naming collision to keep out of the report:** the road bridge exports **677
collision primitives** and the metro has **677 members**. Same number, different
quantities, pure coincidence. Label them distinctly and report the new combined
primitive count against the old 677 explicitly — never as "677 → 677".

Plus: `view.sh` works, a screenshot of the loaded bridge, and the world origin
offset recorded. Commit. **Stop, and give me A–F for Stage 5.**

---

# STAGE 5 — ROS 2 and the aircraft in Gazebo

## 5.0 The architecture decision — binding

**All 46 checks of Phase 1 evidence live in PyBullet.** The controller, the
allocation matrix, the safety supervisor, the 3.0 cm hold, the bit-exact
determinism — every one measured against `simulation/vehicle.py`. A different
integrator and contact solver produce different numbers, and "we validated it in
PyBullet" is not a statement about the thing you are then flying.

**PyBullet stays the validated dynamics reference. Gazebo is added as the ROS 2
integration and demonstration layer.** Then run the same mission in both and
report the divergence — §5.5. **Port the interface, not the controller.**

## 5.1 Before any code

Report actual versions — do not assume. `lsb_release -a`, `python3 --version`,
`gz sim --versions`, and whether ROS 2 is installed. Python 3.12 implies Ubuntu
24.04, which pairs with **ROS 2 Jazzy** and **Gazebo Harmonic (gz-sim 8)** via
`ros_gz`. If that is what you find, use it. If not, say what is there before
installing anything. **Gazebo Classic is EOL — do not install it.**

Then give me A–F.

## 5.2 The aircraft

`avian.urdf` is **generated from CAD** by `build_urdf.py`. Rule §3.6 and the
"one definition per number" discipline mean you must not hand-edit it — the next
regeneration silently wipes the changes.

Add a **`--gazebo` flag to `build_urdf.py`** that emits the `<gazebo>` extensions
alongside the existing output, from the same `params_b.py` constants.

Sensors map to gz-sim plugins: three RGB cameras, depth, GPU lidar, IMU, NavSat.
**Use the sensor frames from the URDF** — T10 already asserts all seven
boresights within 1° of their mount angles. That check exists because these
frames were once 90° off the nose; do not re-derive them in a plugin config.

## 5.3 Control and safety

`simulation/controller.py` is the validated cascade: position → velocity (P) →
acceleration (PID) → thrust vector → tilt and collective → rate (P) → torque
(PID) → allocation via the pseudo-inverse of the 4×8 mixer, condition number
50.0. It passes 11 checks. **Wrap it in a ROS 2 node; do not rewrite it.**

`simulation/safety.py` the same. It is latching, hysteretic, and ranks
**proximity above battery** — a deliberate safety decision that must survive
the port intact.

The eight `continuous` rotor joints need either `MulticopterMotorModel` per rotor
or direct wrench application. Pick one, say which, and say why.

## 5.4 The fleet — two aircraft, heterogeneous

**Spawn one first** and confirm it hovers. Then **two**:

- **SCANNER** — no manipulator, lighter, longer endurance. Sensors only.
- **REPAIRER** — the full 23.693 kg airframe with the 6-DOF arm.

This is a change from the N=6 homogeneous fleet the paper models, and it matters:
Eq. (1) budgets `E_max` and `T_max` across identical aircraft, so the
**certification frontier and the 18.0% figure do not apply to a mixed pair**.
Keep the six-sector structure in the environment. Record in the log that the
frontier is derived for the homogeneous case and extending it to a heterogeneous
fleet is open work. **Do not quote 18.0% while flying two aircraft.**

Six identical UAVs remain a supported configuration — make fleet composition a
parameter, not a hard-coded two.

Report peak memory with both airborne. If it does not fit, reduce the exported
corridor, not the fleet.

## 5.5 The cross-validation

New suite, `AVIAN_UAV/tests/test_crossvalidate.py`. Run the identical 5-waypoint
mission `T3` uses in PyBullet and in Gazebo from the same initial state. PyBullet
measured 10.5 cm max arrival error, 70 s, 51.8 Wh. Report:

- Trajectory divergence, RMS and maximum
- Arrival error at each of the 5 waypoints, both engines
- Energy consumed, both engines
- Where and when divergence grows fastest

**Do not set a pass threshold.** Measure first, report the number, and we decide
together. A threshold invented before the measurement is a threshold fitted to
the result — §3.4 exists to stop exactly that.

## 5.6 Stage 5 gate

- Versions reported; nothing installed that wasn't named first
- `ros2_ws` builds clean with `colcon build`
- Gazebo world loads; V44–V50 still pass
- One UAV hovers; scanner and repairer both spawn; peak memory reported
- All seven sensors publish on ROS 2 topics, boresights matching T10
- One 5-waypoint mission completes in Gazebo
- Cross-validation numbers reported, **no threshold invented**
- **PyBullet 46/46 still green**
- Committed

## 5.7 What not to do

- Do not delete or bypass the PyBullet path — it holds all the validated evidence
- Do not hand-edit `avian.urdf`
- Do not rewrite the controller or the safety supervisor
- Do not claim Gazebo results carry Phase 1's validation
- Do not quote the 18.0% frontier for a two-aircraft fleet
- Do not install Gazebo Classic

---

## Commands to see the world in Gazebo

These must all work by the end of Stage 4. Put them in
`AVIAN_ENVIRONMENT/gazebo/README.md`.

```bash
# --- check what's installed -------------------------------------------
gz sim --versions            # expect 8.x (Harmonic)
lsb_release -a
python3 --version

# --- install, only if absent and only after reporting versions --------
sudo apt update
sudo apt install lsb-release gnupg curl
curl https://packages.osrfoundation.org/gazebo.gpg \
  | sudo tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg >/dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
sudo apt update && sudo apt install gz-harmonic

# --- resource path (needed every session) -----------------------------
export GZ_SIM_RESOURCE_PATH=/home/prince/avian_rev_c/AVIAN_ENVIRONMENT/gazebo/models

# --- validate the SDF before launching --------------------------------
gz sdf -k /home/prince/avian_rev_c/AVIAN_ENVIRONMENT/gazebo/worlds/avian_sic.sdf

# --- SEE IT -----------------------------------------------------------
gz sim -v 4 /home/prince/avian_rev_c/AVIAN_ENVIRONMENT/gazebo/worlds/avian_sic.sdf

# or, with everything set up for you
/home/prince/avian_rev_c/AVIAN_ENVIRONMENT/gazebo/view.sh

# --- headless load test, for the gate ---------------------------------
gz sim -s -r --iterations 100 \
  /home/prince/avian_rev_c/AVIAN_ENVIRONMENT/gazebo/worlds/avian_sic.sdf

# --- inspect a running world ------------------------------------------
gz topic -l                  # list topics
gz model --list              # list spawned models
gz model -m BR_DECK_SLAB_001 -p    # pose of one member
```

**In the GUI:** press `Play` (bottom left) to start physics — the world loads
paused. Scroll to zoom, middle-drag to orbit, shift-middle-drag to pan. If you
see nothing, the camera is probably a kilometre from the geometry — that is what
§4.3's origin shift prevents, and what the documented camera poses are for.

---

## Reporting, after every stage

1. **Every** check, measured against limit — not a summary
2. Triangle count, delta, and where it went
3. Build wall-time, and Cycles cost per validation view
4. **Anything found by a check that failed**
5. Renders — and for Stage 4, a Gazebo screenshot

---

## Scale note

Stage 2 is one to three sessions. **Stage 4 is the one you care about** and is
roughly one session, because it reuses the existing decomposition — the new work
is SDF emission, mesh export and getting the resource paths right. Stage 5 is
large: `ros2_ws/src/` holds ten package directories containing zero files, and
the first session will mostly be version and build friction.

If time is short, **Stage 2 → Stage 4 is the protected path.** A bridge and metro
you can fly a camera through in Gazebo is a demo on its own.

And keep this in view: all of the above is environment and integration.
`avian/perception/`, `avian/fleet/`, `avian/repair/`, `avian/metrics/` and
`avian/twin/` remain at **zero lines**.
