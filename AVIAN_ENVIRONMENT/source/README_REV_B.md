# AVIAN Smart Infrastructure Inspection City — REV-B

**PHASE: SIMULATION-READY ENVIRONMENT**

A research-grade digital-twin foundation for autonomous six-UAV bridge
inspection: a 4.5 km highway bridge and viaduct crossing a 620 m river in an
Indian-style metropolis, with a 900 m high-detail research zone carrying 192
frozen, individually-measured concrete defects.

REV-B does **not** contain a UAV, a flight controller, ROS, PX4, SLAM, a path
planner, a detector or a manipulator. It contains the world those systems will
be evaluated in, and the measured ground truth they will be scored against.

---

## 1. What REV-B is

REV-A was an *environment*. REV-B is an environment plus the machine-readable
scaffolding a robotics testbed needs:

```
REV-A                          REV-B  =  REV-A, unchanged, plus
─────────────────────          ──────────────────────────────────────
engineered bridge              + 7 sensor anchor frames
192 defects                    + measured per-defect visibility ground truth
authored occlusion estimate    + 4-level detection difficulty
33 airspace volumes            + 43 more, in an 8-class operating taxonomy
9 scenario markers             + 16 fully-specified inspection scenarios
—                              + 124 mission markers and a costed mission graph
—                              + 9 scenario presets, 7 lighting, 6 weather
—                              + switchable dynamic content and obstacles
—                              + instance IDs, render passes, dataset manifest
21 validation checks           + 11 more, including baseline-drift enforcement
```

### The upgrade was performed in place

The REV-A geometry generators — `params`, `materials`, `meshlib`, `bridge`,
`terrain`, `roads`, `city`, `lighting`, `damage`, `zones` — are imported and run
**unchanged**. REV-B adds modules alongside them. Nothing was rebuilt.

---

## 2. Ground-truth integrity: the headline result

The revision required upgrading the environment without disturbing 192 frozen
defects. Asserting that in a document is worth nothing, so it is enforced:

> **V22 — ground truth matches the REV-A baseline**
> 192 baseline, 192 now, **maximum drift 0.00 mm**

`BASELINE_REV_A_index.json` was written from the REV-A build before a single
line of REV-B was added. V22 loads it and compares every defect ID, coordinate,
type, severity and sector against the rebuilt scene. Any drift over 1 mm fails
the build and names the defect.

This is also why every REV-B object is named `AVI_*`. Defect positions are
produced by ray-casting at the structure and snapping the defect to what is hit
— so a cable tray added 200 mm under a girder soffit would have intercepted
that ray and silently relocated 30-odd defects. `damage._RAY_SKIP_PREFIXES`
makes that impossible by construction.

---

## 3. Preserved from REV-A

| | |
|---|---|
| Coordinate system | 1 BU = 1 m, Z up, right-handed; origin at deck start, +X along the corridor, corridor symmetric about y = 0 |
| Bridge | 4 500 m, 7 sections, 138 piers, 137 spans, 24 m deck |
| Girder depth | 1.60 – 4.09 m, span/depth ratio 22 |
| Main span | 90 m clear, 24.6 m air draft, crest over the navigation channel |
| River | 620 m wide, water at z = −2.0 m, 3 000 m modelled |
| Research zone | 900 m, x = 1 650 – 2 550 |
| Sectors | 6 × 150 m, SECTOR_A … SECTOR_F |
| Defects | **192, identical IDs, identical positions** |
| Repair envelope | severity ≤ 3, area ≤ 0.45 m², width ≤ 3.0 mm, length ≤ 2.2 m |
| Escalation cases | 36 of 192 (18.8 %) |
| Validation | all 21 REV-A checks, still passing |

---

## 4. Sensor anchor frames

`AVI_SENSOR_RIG` holds seven reference frames in a standard body frame
(+X forward, +Y left, +Z up). Each frame's −Z is its optical axis, matching
Blender's camera convention.

| Frame | Class | FOV / spec | Purpose |
|---|---|---|---|
| `AVI_SENSOR_RGB_FRONT` | RGB | 69° × 42°, 1920×1080, 0.62 mm/px @ 1 m | primary inspection imagery |
| `AVI_SENSOR_RGB_DOWN` | RGB | 82° × 52°, global shutter | optical flow, deck surface |
| `AVI_SENSOR_RGB_SIDE` | RGB | 69° × 42° | lateral surface work |
| `AVI_SENSOR_DEPTH` | Depth | 87° × 58°, 0.3–8 m, ±2 % | obstacle avoidance, spall relief |
| `AVI_SENSOR_LIDAR` | LiDAR | 360° × 59°, 32 ch, 0.1–70 m, ±30 mm | mapping, SLAM, clearance |
| `AVI_SENSOR_THERMAL` | Thermal | 57° × 44°, 640×512, 50 mK | delamination by thermal contrast |
| `AVI_SENSOR_IMU` | IMU | 400 Hz | body origin; all offsets measured from here |

**No airframe is modelled.** Building a placeholder quadrotor would fix an
airframe that has not been designed yet, and every downstream system would
silently inherit its dimensions.

---

## 5. Measured sensor visibility ground truth

This is the most substantive addition, and it produced a finding.

REV-A's `occlusion` was an *authored estimate* — a number typed in because a
diaphragm bay is obviously harder to see than a parapet. Reasonable prior, bad
measurement. REV-B replaces it with a measurement: for every defect, 61
directions on a Fibonacci hemisphere are ray-cast against the real geometry to
25 m, and from that the record gains:

```
visible_fraction              measured, not authored
occlusion_measured            1 − visible_fraction
recommended_view_direction    the open direction closest to the surface normal
recommended_view_obliquity_deg
best_view_clear_m             how far that direction actually runs
feature_size_mm               what a sensor has to resolve
required_gsd_mm               the ground sample distance this defect demands
min_inspection_range_m / max_useful_range_m
expected_rgb_visibility       + max range + what limits it
rgb_width_measurable          detection and measurement are separate thresholds
expected_depth_visibility     + max range
expected_lidar_visibility     + max range
illumination_prior            flagged as a PRIOR, not ray-traced
detection_difficulty          LEVEL_1 … LEVEL_4
recommended_sensor
```

### Finding 1 — the authored occlusion estimate was worthless

Measured against the authored REV-A prior across all 192 defects:

```
authored mean occlusion   0.475
measured mean occlusion   0.096
Pearson r                -0.057
mean absolute error       0.41
```

**r = −0.06 is no correlation at all.** The authored numbers were not a rough
approximation of the truth — they were unrelated to it, and off by 0.41 on
average on a 0–1 scale. Both values are kept in the ground truth (`occlusion`
authored, `occlusion_measured` measured) so the comparison stays auditable, but
nothing should ever be scored against the authored one again.

This is the single strongest argument for the module existing. Any detection
result computed against the REV-A occlusion figure would have been measuring
the author's intuition rather than the world.

### Finding 2 — a third of the defects are invisible to the modelled sensor

> **64 of 192 defects (33 %) are undetectable by any modelled sensor.**

Not a modelling failure — a *quantified sensor-selection result*. The RGB spec
gives 0.62 mm/px at 1 m and cannot focus closer than 0.35 m, so the finest
feature it can resolve at one pixel is ≈ 0.22 mm. Every `CRACK_HAIRLINE`
(0.05–0.20 mm) is below that line. The ground truth says so per defect, with
`rgb_limited_by` naming the binding constraint.

**A detector evaluated on this environment must not be scored against those 64.**
That is exactly the sort of error this field makes, and the environment now
makes it impossible to make by accident.

Detection difficulty is a composite of feature size, illumination prior,
measured openness and viewing obliquity, offset by severity — deliberately four
independent drivers rather than a relabelled occlusion number.

---

## 6. UAV airspace — 8-class REV-B taxonomy

The 33 REV-A volumes are preserved. 43 more are added, each carrying an
operating envelope rather than just a bounding box.

| Class | N | Speed | Standoff | Clearance | Risk | GPS |
|---|---|---|---|---|---|---|
| `TRANSIT_AIRSPACE` | 1 | 8.0 m/s | — | 15 m | LOW | HIGH |
| `DECK_INSPECTION_AIRSPACE` | 6 | 2.5 | 6.0 m | 4 m | MEDIUM | HIGH |
| `UNDERSIDE_INSPECTION_AIRSPACE` | 6 | 1.2 | 2.5 m | 1.5 m | HIGH | LOW |
| `PIER_INSPECTION_AIRSPACE` | 16 | 1.5 | 3.0 m | 2 m | MEDIUM | MEDIUM |
| `RIVER_INSPECTION_AIRSPACE` | 1 | 2.0 | 3.5 m | 2.5 m | **CRITICAL** | MEDIUM |
| `CONFINED_INSPECTION_AIRSPACE` | 6 | 0.6 | 1.2 m | 0.8 m | **CRITICAL** | DENIED |
| `EMERGENCY_RETURN_AIRSPACE` | 2 | 6.0 | — | 8 m | LOW | HIGH |
| `MULTI_UAV_SEPARATION_ZONE` | 5 | 1.0 | — | 10 m | HIGH | HIGH |

Volumes are checked against real geometry (V10, V27): free-flight airspace that
intersected structure would fail the build. A volume in the scene can be tested;
a number in a config file cannot.

---

## 7. Sixteen inspection scenarios

`SCENARIO_B01` … `SCENARIO_B16`, each anchored to geometry that exists — a
named pier, a specific bay — not to a convenient coordinate. The nine REV-A
markers are preserved under their original names.

| | Scenario | Diff | GPS | Water |
|---|---|---|---|---|
| B01 | OPEN_DAYLIGHT_INSPECTION | 1 | HIGH | — |
| B02 | UNDERBRIDGE_INSPECTION | 3 | LOW | — |
| B03 | DEEP_UNDERSIDE_INSPECTION | 5 | DENIED | — |
| B04 | PIER_INSPECTION | 2 | MEDIUM | — |
| B05 | RIVER_PIER_INSPECTION | 4 | MEDIUM | ✓ |
| B06 | STRONG_OCCLUSION | 5 | DENIED | — |
| B07 | NARROW_STRUCTURAL_CORRIDOR | 4 | DENIED | — |
| B08 | LOW_LIGHT_INSPECTION | 4 | MEDIUM | — |
| B09 | PARTIAL_SHADOW | 3 | HIGH | — |
| B10 | REFLECTIVE_WATER_ENVIRONMENT | 4 | MEDIUM | ✓ |
| B11 | LONG_RANGE_DETECTION | 4 | HIGH | — |
| B12 | CLOSE_RANGE_INSPECTION | 3 | LOW | — |
| B13 | HIGH_WIND_CONCEPTUAL | 4 | HIGH | — |
| B14 | GPS_DENIED_CONCEPTUAL | 4 | DENIED | ✓ |
| B15 | MULTI_UAV_SHARED_WORKSPACE | 3 | HIGH | — |
| B16 | DYNAMIC_OBSTACLE | 4 | LOW | — |

Each carries `avi_recommended_sensor`, `avi_minimum_clearance_m`,
`avi_lighting_condition`, `avi_occlusion_level`, `avi_water_present`,
`avi_gps_quality`, `avi_dynamic_obstacles`, `avi_recommended_uav_speed_mps`,
plus the challenge, the sensing problem and a suggested mitigation.

**B13 and B14 are explicitly labelled CONCEPTUAL.** No wind field and no GNSS
physics are simulated. `avi_gps_quality` is metadata a navigation stack
consumes, not a simulated signal — though the sky occlusion under the deck at
B14 is geometrically real.

---

## 8. Mission markers and the mission graph

124 markers across 11 types, and a graph whose edges are **measured against real
geometry**: an edge whose straight line intersects structure is not emitted, so
the graph is traversable by construction.

| Marker type | N |
|---|---|
| UAV_SPAWN / TAKEOFF / LANDING | 2 / 2 / 2 |
| RETURN_TO_HOME / SERVICE_STATION | 2 / 2 |
| SECTOR_ENTRY / SECTOR_EXIT | 6 / 6 |
| INSPECTION_START / INSPECTION_END | 6 / 6 |
| EMERGENCY_LANDING | 6 |
| WAYPOINT_CANDIDATE | 84 |

Each edge carries `distance_m`, `min_clearance_m`, `risk` (0–1 composite of
corridor tightness and over-water exposure), `over_water`,
`assumed_speed_mps` and `expected_time_s`.

The six sector tours run **one station at a time** — deck run out, web run back,
transfer down outboard of the deck edge, under-deck run out. Interleaving the
three stations produced a graph where almost every edge was rejected as
blocked, which is what a naive lawnmower pattern would do in reality too.

The three EMERGENCY_LANDING markers over the navigation channel carry
`avi_landable: false` and `avi_ditch_only: true`. There is no landing surface
for 620 m and the graph says so rather than pretending otherwise.

**This is not a path planner.** It is the data a planner will be given.

---

## 9. Scenario controller

```python
import scenarios
scenarios.apply("NIGHT")          # or any preset below
```

**9 presets:** BASELINE · DAY_INSPECTION · UNDERBRIDGE · RIVER_INSPECTION ·
NIGHT · LOW_VISIBILITY · MULTI_UAV · GPS_DENIED · DYNAMIC_OBSTACLE

**7 lighting:** DAY_CLEAR · DAY_OVERCAST · MORNING · EVENING · NIGHT ·
UNDERBRIDGE · DEEP_SHADOW

**6 weather:** CLEAR · CLOUDY · LIGHT_RAIN · WET_SURFACE · HAZY · LOW_LIGHT

Everything here changes illumination, atmosphere and switchable-collection
visibility. **Nothing moves a pier, edits a defect or touches a dimension.**
That separation is the design: a dataset generated under six lighting
conditions is only useful if the geometry underneath is provably identical
across all six.

Note that `DAY_OVERCAST` is the *hardest* condition for crack detection despite
being the brightest underside — a shallow crack is read from shading, and flat
light removes it.

**Weather is honest about its limits.** There is no volumetric atmosphere and
no rain simulation. "Light rain" means wet-look roughness, darker albedo,
overcast sky and stronger haze — what actually changes a camera's view of
concrete, without a participating medium the CPU cannot afford.

---

## 10. Dynamic content and obstacles

| Collection | Contents | Default |
|---|---|---|
| `AVI_DYNAMIC_VEHICLES` | 318 road vehicles | ON |
| `AVI_DYNAMIC_BOATS` | 3 boats, 14 floats | ON |
| `AVI_DYNAMIC_PEDESTRIANS` | ~107 person proxies | ON |
| `AVI_DYNAMIC_OBSTACLES` | scaffold tower, suspended platform, safety net, mobile crane, 10 barriers | **OFF** |

The obstacles sit inside the research zone and deliberately intrude on
inspection airspace — a scaffold tower against a pier in SECTOR_B, a platform
hanging *inside* the under-deck volume in SECTOR_E, a crane boom reaching into
the transit band. All carry `avi_in_digital_twin: false`.

That is the point: a planner working from the digital twin alone will fly into
them. Only live depth sensing prevents it.

---

## 11. Dataset generation support

Prepared, not rendered — sweeping conditions is a research decision and hours
of CPU Cycles.

**Instance identity.** Every defect gets a unique `pass_index` (1–192);
structure occupies 1000+, dynamic content 9000+. The Object Index render pass
therefore *is* the instance mask — no colour matching, no post-hoc association,
and no silent merging when two defects of the same class share a girder.

**Passes enabled:** Combined · Depth (Z) · Normal · Object Index · Position ·
Mist (range set to 1–400 m, not Blender's 25 m default).

**Derivable outputs:** per-defect binary masks, per-class masks, 2-D bounding
boxes, depth maps, normal maps.

**7 capture configurations:** normal · close · oblique · underside · pier ·
difficult-occlusion · long-range screening.

> **Split by SECTOR, never at random.** Defects in one sector share host
> members, lighting and surface finish, so a random split leaks the test set
> into training through the background.

---

## 12. Validation — 32 checks

| | REV-A checks V01–V21 | |
|---|---|---|
| V01–V09 | scale, corridor, floating objects, river, footings, channel, air draft, grade, span/depth | PASS |
| V10–V15 | airspace clearance, headroom, defect positions, research zone, ground-truth completeness, escalation ratio | PASS |
| V16–V21 | polycount, camera clip, sector coverage, unique names, collection roles, river not occluded | PASS |

| | REV-B checks V22–V32 |
|---|---|
| **V22** | **ground truth matches the REV-A baseline — 0.00 mm drift** |
| V23 | mission markers complete and clear of structure |
| V24 | every sector has its own mission markers |
| V25 | sensor anchors defined with optics |
| V26 | sensor visibility ground truth complete, all 4 difficulty levels used |
| V27 | 8 REV-B airspace classes, operating envelope on every volume |
| V28 | 16 scenarios, 9 required fields each |
| V29 | mission graph connected and costed, all sectors reachable |
| V30 | dynamic content separated from structure |
| V31 | scenario controller configured |
| V32 | dataset generation ready — passes, indices, 12 cameras |

Full detail with measured values in `AVIAN_validation_report_REV_B.json`.

V20 checks collection **roles**, not literal strings, so the REV-B
digital-twin naming passes without weakening the check.

---

## 13. Cameras

REV-A's seven are preserved with their original names and poses. Five are added,
three of them **aimed from the ground truth at build time** rather than from
fixed coordinates — a camera pointed at a fixed point is pointed at whatever
happens to be there after the next change.

```
01 GLOBAL_CITY          07 FULL_INFRASTRUCTURE
02 BRIDGE_FULL_LENGTH   08 PIER_INSPECTION       (river pier, from the water)
03 RIVER_CROSSING       09 GIRDER_INSPECTION     (down a confined bay)
04 RESEARCH_ZONE        10 DIFFICULT_OCCLUSION   (← most-occluded defect)
05 UNDERBRIDGE          11 UAV_APPROACH          (transit-band approach)
06 DEFECT_CLOSEUP       12 UAV_INSPECTION        (← from the sensor rig pose)
```

---

## 14. Collection hierarchy

```
AVIAN_WORLD
├── AVIAN_BRIDGE       DECK · PIERS · BEAMS · JOINTS · BARRIERS · DETAILS
├── AVIAN_RIVER
├── AVIAN_ROADS
├── AVIAN_CITY         BUILDINGS · VEHICLES · STREET_LIGHTS · VEGETATION
├── AVIAN_DEFECTS      CRACKS · SPALLING · REBAR · CORROSION · JOINT_DAMAGE
├── AVIAN_AIRSPACE     UAV_AIRSPACE · UAV_AIRSPACE_REV_B · INSPECTION_SECTORS
├── AVIAN_MISSION      MISSION_MARKERS
├── AVIAN_SENSORS
├── AVIAN_SCENARIOS    INSPECTION_SCENARIOS
├── AVIAN_DYNAMIC      VEHICLES · BOATS · PEDESTRIANS · OBSTACLES
├── CAMERAS
└── LIGHTING_ENVIRONMENT
```

The digital-twin groupings are **aliases onto the same collections**, not a
second parallel hierarchy — an object in two hierarchies is an object that gets
exported twice and counted twice.

---

## 15. Files

```
AVIAN_Smart_Infrastructure_City_REV_B.blend
build_scene_b.py                     master build (REV-B)
README_REV_B.md · OPENING_IN_BLENDER.md

PRESERVED REV-A MODULES (run unchanged)
  params · materials · meshlib · bridge · terrain · roads · city
  lighting · damage · zones · validate · renders · build_scene

REV-B MODULES
  sensors.py      7 sensor anchor frames
  mission.py      markers, base-site preparation, costed mission graph
  visibility.py   measured per-defect sensor visibility
  scenarios.py    lighting / weather / preset controller
  dynamics.py     boats, pedestrians, temporary obstacles
  detail_b.py     bridge services, city variety
  zones_b.py      8-class airspace, 16 scenarios
  cameras_b.py    cameras 08–12
  dataset.py      instance IDs, passes, dataset manifest
  validate_b.py   checks V22–V32

MANIFESTS
  AVIAN_defect_ground_truth_REV_B.json / .csv
  AVIAN_zone_manifest_REV_B.json
  AVIAN_mission_manifest_REV_B.json
  AVIAN_sensor_manifest_REV_B.json
  AVIAN_scenario_manifest_REV_B.json
  AVIAN_dataset_manifest_REV_B.json
  AVIAN_scene_stats_REV_B.json
  AVIAN_validation_report_REV_B.json
  AVIAN_build_log_REV_B.txt
  BASELINE_REV_A_index.json          the frozen baseline V22 enforces
```

### Rebuilding

```bash
pip install bpy==4.5.*
python build_scene_b.py                      # build + 32 checks + save
python build_scene_b.py --scenario NIGHT
python build_scene_b.py --render --samples 48
```

`params.SEED = 20260827` drives every random placement.

---

## 16. Known limitations

Stated plainly rather than papered over.

1. **33 % of defects are undetectable by the modelled RGB sensor.** Real,
   quantified, and per-defect. Do not score a detector against them.
2. **Cracks are shader decals.** No depth sensor will register them. Correct
   physics; `representation` says which is which.
3. **Delamination is visually almost nothing.** Honest — it is found by
   sounding, not by looking. Every case escalates.
4. **No thermal emission model.** The thermal frame is a mounting definition;
   it would be the modality that finds delamination, and it is not simulated.
5. **Illumination in the visibility ground truth is a per-surface prior, not a
   ray-traced result.** Flagged as `illumination_prior` in every record.
6. **Wind and GNSS are metadata, not physics.** B13 and B14 are labelled
   CONCEPTUAL. Nothing computes a wind field or a satellite geometry.
7. **Visibility is sampled, not integrated** — 61 directions, roughly ±2 % on
   the occlusion fraction.
8. **Weather is a surface-and-atmosphere approximation.** No volumetrics, no
   particles, no puddle accumulation.
9. **Pedestrians are three primitives.** No articulation, no gait, no motion.
10. **Nothing is animated.** Vehicles, boats and people are static placements;
    dynamics here means *switchable*, not *moving*.
11. **The mission graph is straight-line.** Edges are geometry-checked but not
    curvature- or dynamics-feasible; a real planner must re-check them against
    the aircraft's actual flight envelope.
12. **The riverbank is a straight line** and the distant ground is a ring with
    a hard inner boundary (it must be, or it hides the river — see V21).
13. **Pier columns are 16-sided prisms.** Defects are snapped to the facet, so
    the error is in the cylinder, not the defect.
14. **CPU rendering only** in the environment this was built in.

---

## 16b. Final numbers

| | REV-A | REV-B |
|---|---|---|
| Objects | 11 911 | **13 835** |
| Unique meshes | 3 190 | **3 400** |
| Rendered triangles | 354 856 | **384 200** (+8.3 %) |
| Collections | 26 | **37** |
| Cameras | 7 | **12** |
| Defects | 192 | **192 (0.00 mm drift)** |
| Ground-truth fields per defect | 20 | **38** |
| Airspace volumes | 33 | **76** (33 REV-A + 43 REV-B) |
| Inspection scenarios | 9 | **25** (9 REV-A + 16 REV-B) |
| Mission markers | 0 | **124** |
| Mission graph edges | 0 | **136** |
| Sensor anchor frames | 0 | **7** |
| Scenario presets | 0 | **9** (7 lighting × 6 weather) |
| Instance IDs assigned | 0 | **2 385** |
| Validation checks | 21 | **32, all passing** |
| .blend size | 51.8 MB | **59.2 MB** |
| Build time | ~35 s | ~7 min (visibility ray-casting dominates) |

Detection difficulty distribution: LEVEL_1 51 · LEVEL_2 54 · LEVEL_3 23 ·
LEVEL_4 64.

The polygon budget grew 8.3 % while adding 1 562 bridge service objects, 28
city structures and 138 dynamic objects — because all of it is instanced.

---

## 17. Recommended next phase

The architecture this environment is built to serve:

```
Onshape        →  AVIAN UAV CAD (done — separate package)
Blender        →  bridge + city + river + defects + ground truth  ← YOU ARE HERE
Simulation     →  UAV rigid-body physics + sensor models
Autonomy       →  localisation, SLAM, planning, obstacle avoidance
AI             →  defect detection, classification, 3-D localisation
Digital twin   →  structural condition map + inspection report
```

**Next phase: the simulation layer.** Specifically:

1. **Import the airspace and mission manifests into a physics simulator**
   (Gazebo/Isaac). Both are plain JSON and neither needs Blender at runtime.
2. **Instantiate sensors from `AVIAN_sensor_manifest_REV_B.json`** at the
   frames' body offsets. The optical parameters are already there.
3. **Render the first dataset sweep** — the seven capture configs across three
   lighting conditions, split by sector.
4. **Establish the detection baseline against the 128 defects that are
   actually observable**, reporting the other 64 separately as a sensor
   limitation rather than a detector failure.

Only after a detector has a measured baseline does a planner have anything
worth optimising, and only then does putting an aircraft in the loop test
something real.

**Do not describe the system as autonomous** until the flight controller,
perception, localisation and planning stack exist and have been measured. This
package is the instrument that will measure them.
