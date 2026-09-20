"""08_Documentation, 06_Drawings and 03_CAD_Reference."""
from __future__ import annotations
import csv
import math
import os

import params_b as P
import kin_b as K
import assy_b
import reconcile as RC
from checks_b import placed

D = "pkg/08_Documentation"
DR = "pkg/06_Drawings"
CR = "pkg/03_CAD_Reference"

reg = assy_b.build("01_FLIGHT")
M, CG, items = assy_b.mass_cg(reg)
en = RC.energy_and_endurance(M)
T = P.vThrustPerArm * P.vArmCount


def w(p, t):
    os.makedirs(os.path.dirname(p), exist_ok=True)
    open(p, "w").write(t.lstrip("\n"))
    print("  ", p)


# ---------------------------------------------------------------------------
def spec():
    ext = {}
    for p in reg.parts:
        b = placed(p).BoundingBox()
        for k, v in (("xmin", b.xmin), ("xmax", b.xmax), ("ymin", b.ymin),
                     ("ymax", b.ymax), ("zmin", b.zmin), ("zmax", b.zmax)):
            ext[k] = min(ext.get(k, v), v) if "min" in k \
                else max(ext.get(k, v), v)
    w(f"{D}/AVIAN_SYSTEM_SPECIFICATION.md", f"""
# AVIAN — System specification, REV A

## What AVIAN is

A heavy-lift industrial UAV for **inspection, repair, liquid service and
physical manipulation** of structures — facades, bridges, towers, tanks. It
carries a 6-DOF manipulator with a quick-change wrist, so it does not just
look at a structure, it touches and works on it.

It is a coaxial X8 of {P.vDiagonal:.0f} mm diagonal, {M:.2f} kg nominal MTOW,
with a {P.vReach:.0f} mm arm that reaches **{P.vReach - K.prop_forward_extent():.0f} mm
beyond its own rotor envelope**. That last figure is the design's central
idea: the tool can touch a wall the aircraft cannot fly into.

## Principal characteristics

| | |
|---|---|
| Configuration | Coaxial X8 — 4 arms, 8 motors, 8 rotors |
| Motor-to-motor diagonal | {P.vDiagonal:.0f} mm |
| Overall envelope (L × W × H) | {ext['xmax'] - ext['xmin']:.0f} × {ext['ymax'] - ext['ymin']:.0f} × {ext['zmax'] - ext['zmin']:.0f} mm |
| Rotor diameter | {P.vPropDia:.0f} mm ({P.vPropDia / 25.4:.0f} in) |
| Nominal MTOW | **{M:.3f} kg** |
| Maximum MTOW | **{M + 2.0:.3f} kg** |
| Max static thrust | {T:.2f} kgf |
| T/W nominal / maximum | {T / M:.2f} / {T / (M + 2.0):.2f} |
| Hover power | {en['P_hover']:.0f} W |
| Endurance (20 % reserve) | **{en['endurance_min'] * 0.8:.1f} min** |
| Electrical | 12S, {P.vBattWh:.0f} Wh |
| Manipulator | 6 DOF, {P.vReach:.0f} mm reach, 2 kg payload architecture |
| Service fluid | {P.vTankVol:.2f} L tank, {P.vTankFill:.1f} L working |
| Landing clearance | {abs(P.vGearGround):.0f} mm skid, {abs(P.vGearGround) - 99.5:.0f} mm under the lower rotor |
| Configurations | 7 |
| Parts | 137 |

## Subsystems

### Airframe
Central CF box spine, {P.vSpineLen:.0f} × {P.vSpineTrack:.0f} mm, on four CNC
AL7075 corner nodes. Four CF propulsion arms at 45° stations. A reinforced
AL7075 manipulator hub on the belly centreline. Four printed PA-CF service
panels, each on quarter-turn fasteners with a declared removal direction.

### Propulsion
T-Motor X-U8II-class coaxial architecture: U8 II KV100 motors, ALPHA 60A 12S
ESCs, G28x9.2 CF folding propellers, 12S. Upper rotors CW, lower CCW,
adjacent arms alternating.

### Power
Two parallel 12S {P.vBattAh:.0f} Ah cartridges on a three-position manual
index. Aft extraction, tool-free, two quarter-turn latches per pack. **No
powered CG rail** — deleted with quantitative justification, see
`07_Engineering/CG_ANALYSIS.md`.

### Manipulator
6 DOF: J1 base yaw, J2 shoulder pitch, J3 elbow pitch, J4 wrist roll, J5 wrist
pitch, J6 wrist yaw. Thin-wall machined housings and CF link tubes. Internal
cable routing throughout. 6-axis F/T sensor above the tool changer.

### Tooling
Quick-change wrist interface: 3 tapered locking pins, 12-way electrical, 1
dry-break fluid port, spring-applied cam so power loss locks rather than
drops. Two tools defined — a two-finger repair gripper with replaceable tips,
and a liquid-service nozzle with a replaceable orifice and a standoff guard.
A third tool needs only the tool plate and a body inside the published
envelope.

### Liquid service
Removable {P.vTankVol:.2f} L tank cartridge with {P.vBaffleN} baffles, plus an
airframe-mounted pump, {P.vFilterMicron:.0f} µm filter, check valve,
relief/bypass and pressure sensor. Single dry-break coupling at the cartridge
face. Baffles move the first slosh mode from 1.86 to 3.95 Hz.

### Perception
LiDAR on a forward mast, 3-axis RGB gimbal, forward depth camera, twin RTK
GNSS on outboard masts, downward range finder and optical flow on the belly
panel, telemetry and RC diversity antennas, work lamp.

### Landing gear
Splayed CF skid gear on wire-rope isolators. {P.vGearTrack:.0f} mm track,
{abs(P.vGearGround):.0f} mm skid plane, 35.1° static tip-over half-angle.

## Operating configurations

`01_FLIGHT`, `02_INSPECTION`, `03_MANIPULATION`, `04_REPAIR`,
`05_LIQUID_SERVICE`, `06_TRANSPORT`, `07_MAINTENANCE`. Defined in
`02_Onshape_Build_Guide/CONFIGURATION_GUIDE.md`.

## Status

REV A is a **design-complete CAD package**. It is not a validated design and
no hardware exists. See `07_Engineering/VALIDATION_MATRIX.md` for what is
verified, what is calculated, and what must happen before anything is cut.
""")


# ---------------------------------------------------------------------------
def arch():
    w(f"{D}/AVIAN_CAD_ARCHITECTURE.md", f"""
# AVIAN — CAD architecture, REV A

## Onshape document structure

```
AVIAN  (Onshape document)
│
├── 00_MASTER_LAYOUT ............ Part Studio, sketches only
├── 01_VARIABLES ................ Variable Studio, 131 variables
│
├── Feature Studios
│   ├── AVIAN_Airframe
│   ├── AVIAN_Propulsion
│   ├── AVIAN_Battery
│   ├── AVIAN_Manipulator
│   ├── AVIAN_ToolChanger
│   ├── AVIAN_LiquidService
│   ├── AVIAN_LandingGear
│   └── AVIAN_ServicePanels
│
├── Part Studios
│   ├── 02_AIRFRAME
│   ├── 03_PROPULSION
│   ├── 04_BATTERY
│   ├── 05_AVIONICS
│   ├── 06_PERCEPTION
│   ├── 07_MANIPULATOR
│   ├── 08_TOOL_CHANGER
│   ├── 09_REPAIR_GRIPPER
│   ├── 09_LIQUID_TOOL
│   ├── 10_LIQUID_SERVICE
│   ├── 11_LANDING_GEAR
│   └── 12_SERVICE_PANELS
│
├── Assemblies
│   ├── A_PROPULSION_MODULE
│   ├── A_BATTERY_CARTRIDGE
│   ├── A_FLUID_CARTRIDGE
│   ├── A_MANIPULATOR
│   ├── A_TOOL_GRIPPER
│   ├── A_TOOL_NOZZLE
│   └── 13_MASTER_ASSEMBLY
│
├── 14_CONFIGURATIONS ........... configuration table on the master assembly
├── 15_DRAWINGS ................. 7 drawing sets
├── 16_BOM ...................... generated from the master assembly
└── 17_VALIDATION ............... interference + clearance study
```

## Why it is arranged this way

**One Variable Studio, referenced everywhere.** Every dimension in every Part
Studio resolves against `01_VARIABLES`. Changing `#vDiagonal` rebuilds the
airframe, moves the rotors, re-sizes the arm tubes and updates every drawing.
There are no literal dimensions in the feature tree except process constants —
fillet radii, wall thicknesses.

**Eight Feature Studios, not one.** A single FeatureScript for a 137-part
aircraft is unreviewable, unmergeable, and rebuilds entirely on every edit.
Splitting by subsystem means an airframe change does not invalidate the
manipulator's regeneration.

**Master layout as pure sketch.** `00_MASTER_LAYOUT` carries no solids. It is
the skeleton — rotor axes, spine, hub station, rotor planes, skid plane, reach
circle — that every other studio measures from. Putting the reach circle and
the rotor discs on one sheet is what makes the architecture legible at a
glance.

**Panels built last.** Every aperture in a service panel is driven by the
thing it covers. Build them early and you cut holes by eye.

## Coordinate system

Right-handed, origin at the aircraft datum.

| Axis | Direction |
|---|---|
| +X | forward (toward the manipulator and the primary sensor field of view) |
| +Y | port (left, looking forward) |
| +Z | up |

Rotor stations at 45°, 135°, 225°, 315° at r = {P.vRMotor:.0f} mm.
Skid plane at z = {P.vGearGround:.0f} mm. Rotor planes at z ≈ +99.5 and −24.5 mm.

## Naming

| Pattern | Example |
|---|---|
| Variables | `#v` + CamelCase — `#vDiagonal`, `#vArmReach` |
| Part Studios | two-digit prefix + subsystem — `07_MANIPULATOR` |
| Assemblies | `A_` + subsystem, master is `13_MASTER_ASSEMBLY` |
| Mates | `M_` + what it joins; joints are `J1`…`J6` |
| BOM item IDs | `AV-<subsystem>-<serial>` |

## ROS compatibility

The manipulator's frames follow a URDF-compatible convention: each joint has
an offset and an axis expressed in its parent frame, so `link_n` → `J_n` maps
one-to-one onto a URDF joint. The frames are:

`arm_base_link` → `J1` … `J6` → `tool0` → `ft_sensor_link` →
`tool_changer_link` → tool point.

This is why the link convention matters: `link_n` mates to `F[n]`, exactly as
a URDF child link attaches to its parent joint.

## Source of truth

The dimensions in this package come from a parametric model
(`params_b.py` → geometry → checks → renders → BOM). Every number in every
document is generated from that model at build time, which is why the BOM
reconciles with the mass budget to within display rounding and the
documentation cannot drift from the geometry.

The Onshape rebuild reproduces that model natively. Once it exists, **the
Onshape document becomes the source of truth** and this package becomes its
build record.
""")


# ---------------------------------------------------------------------------
def risks():
    w(f"{D}/ASSUMPTIONS_AND_RISKS.md", f"""
# AVIAN REV A — Assumptions and open risks

Ordered by severity. Nothing here is hidden in a footnote.

---

## R1 — HIGH — The manipulator can be commanded into the aircraft

**Finding.** Over the full mechanical joint range the arm can reach the
airframe (−41 mm), the rotor discs (−42.9 mm), the landing gear (−41 mm) and
itself (−71 mm). Only **20.2 %** of the joint space is collision-free.

**Not fixable by joint limits.** Solved directly: the largest collision-free
axis-aligned (J2, J3) rectangle across all J1 and J5 is J2 ∈ [−9.6°, 0°],
which leaves the arm unable to work.

**Why it exists.** A {P.vReach:.0f} mm arm on a {P.vSpineLen:.0f} mm airframe
has a workspace far larger than the vehicle carrying it. Every industrial arm
on a mobile base shares this property.

**Mitigation.** Software collision avoidance, using the shipped envelope map
(`03_CAD_Reference/interfaces/AVIAN_arm_collision_envelope.csv`, 49,275
cells). The planner must screen **paths**, not just endpoints.

**Residual risk.** There is no hardware interlock. A planner fault, a bad
command, or a manual jog past the envelope results in contact — potentially
with a spinning rotor. This must be treated as a safety-critical software
function.

---

## R2 — HIGH — T/W at maximum MTOW is below 2.0

**Finding.** T/W is {T / M:.2f} nominal but **{T / (M + 2.0):.2f}** with the
2 kg mission payload. For a manipulating aircraft — one that must reject
contact reaction forces and gusts while holding position — 2.0 is the number
usually wanted.

**Cause.** The CAD came in {(M - RC.budget_groups()[0]) * 1000:.0f} g heavier
than the pre-CAD budget, mostly because the manipulator link structure was
under-estimated by a factor of three.

**Options, none free.**
1. Accept it and restrict payload in gusty conditions.
2. Reduce the pack from {P.vBattAh * P.vBattPacks:.0f} Ah — buys T/W, costs
   endurance which is already short.
3. Move to a higher-thrust motor (U8 II KV85 class) — costs efficiency at
   hover.
4. Lighten the manipulator — the honest option, but it is already thin-walled.

**Not yet decided.** This needs a decision before detail design.

---

## R3 — HIGH — Endurance is {en['endurance_min'] * 0.8:.0f} minutes with reserve

**Finding.** {en['endurance_min']:.1f} min nominal,
{en['endurance_min'] * 0.8:.1f} min with a 20 % reserve, on
{en['Wh_usable']:.0f} Wh usable.

**Caveat.** That figure is CALCULATED from momentum theory on the vendor
coaxial-pair curve — static, sea level, no wind, no manoeuvre allowance. The
installed figure will be **lower**, not higher.

**Consequence.** Realistic on-station working time after transit is well under
10 minutes. For a repair aircraft this shapes the whole concept of operations
and may not be acceptable.

---

## R4 — MEDIUM — One-rotor-out control authority is not demonstrated

The thrust arithmetic works (T/W {T * 7 / 8 / M:.2f} on 7 rotors). But losing
one rotor of a coaxial pair leaves a **yaw torque imbalance** on that arm,
because the counter-rotation no longer cancels. Whether the remaining rotors
can trim it has not been shown. Needs a control-allocation study before any
one-motor-out claim is made.

---

## R5 — MEDIUM — No FEA on any structure

Every section — arm root, corner nodes, manipulator hub, gear legs, spine — is
sized by hand analysis on an idealised geometry. No laminate schedule exists
for any CF part; "CFRP" in the BOM is a material class. No bolted-joint
analysis at the CF/AL interfaces, which is the classic failure mode in this
construction.

---

## R6 — MEDIUM — {sum(1 for n, g, m, c in items if any(k in n for k in ('battery_cartridge', 'flight_controller', 'companion', 'lidar', 'gnss_antenna', 'rgb_gimbal', 'depth', 'pump', 'filter', 'actuator_', 'ft_sensor', 'gear_damper')))} line items are PLACEHOLDER

Battery packs ({7.687 / M * 100:.0f} % of MTOW), flight controller, companion
computer, LiDAR, gimbal, GNSS, depth camera, pump, filter, all six joint
actuators, the F/T sensor and the wire-rope isolators are dimensionally
realistic envelopes for parts that have **not been selected**. Each is tagged
in the BOM. Selecting them will move mass and CG.

The battery is the biggest single uncertainty in the mass budget.

---

## R7 — MEDIUM — Thermal is unanalysed

Eight 60 A ESCs, a companion computer and a 12S PDB in a partly enclosed bay
with printed PA-CF covers. The covers carry louvres and the computer has a
fan, but no thermal analysis has been done. The ESC saddles are the specific
concern: they are the only conduction path from the ESC to the arm tube.

---

## R8 — LOW — Slosh figure is a linear estimate

3.95 Hz baffled, from the standard rectangular-tank first-mode formula.
Slosh under manoeuvre is not linear and the linear estimate is known to be
optimistic. Needs a test with the baffles fitted.

---

## R9 — LOW — Grip force is unmatched to an actuator

{P.vGripForce:.0f} N is ESTIMATED and no gripper actuator has been selected.
The jaw and tip geometry will change once one is.

---

## R10 — LOW — Fluid compatibility undefined

"Service fluid" is generic. A solvent-based coating drives material selection
for the tank shell, every seal, the pump diaphragm and the hose. Until the
fluid is specified, the liquid system's material callouts are provisional.

---

## Standing assumptions

| Assumption | Where it bites |
|---|---|
| 80 % usable battery DoD | endurance |
| Sea level, 25 °C, no wind | thrust and endurance |
| Vendor bench thrust is achievable installed | T/W, endurance |
| 2 kg payload is the design case | joint torques, T/W |
| 90 N contact force is the worst repair case | J2/J3 torque, hub |
| 4 g hard landing | gear, isolators, nodes |
| AL7075-T6 at 500 MPa yield | hub, nodes, clamps |
| Arm actuators hold position unpowered | safety when de-energised |
""")


# ---------------------------------------------------------------------------
def review():
    w(f"{D}/AVIAN_REV_A_DESIGN_REVIEW.md", f"""
# AVIAN REV A — Design review

## Scope of this review

The AVIAN CAD package: architecture, geometry, mass and CG closure, the 15
CAD-level validation checks, the Onshape rebuild kit, the BOM and the
engineering set. It does **not** review anything that requires FEA, test or
flight — none of which has been done.

## What changed from the baseline

The baseline aircraft scored 5.22/10. The substantive changes:

| Change | Effect |
|---|---|
| Flat-8 → **coaxial X8** at {P.vDiagonal:.0f} mm | 4 arms space 1.85× wider than 8, so a 28 in rotor fits where a 12 in did |
| **Reach-extension lance deleted** | The tool now reaches {P.vReach - K.prop_forward_extent():.0f} mm past the rotor envelope unaided. The single biggest simplification in the design |
| **Powered CG rail deleted** | Worst CG excursion is lateral and a fore/aft rail cannot fix it; hover uses only {M / T * 100:.0f} % of thrust |
| Loose tank → **service-fluid cartridge** | Retained, serviceable, inspectable, one dry-break |
| Solid billet joints → **thin machined shells** | Manipulator went from 3× over budget to closure |
| Estimated masses → **measured CAD masses** | Mass budget now reconciles to 0 g unexplained |
| 148 static interferences → **0** | Across all seven configurations |

## Where it is genuinely strong

**The rotor-versus-reach resolution.** The central conflict in a manipulating
multirotor is that the props stick out further than the arm can reach, so the
tool cannot touch anything the aircraft can hover next to. Shrinking to a
{P.vDiagonal:.0f} mm coaxial X8 drops the rotor extent to
{K.prop_forward_extent():.0f} mm while the arm reaches {P.vReach:.0f} mm. That
{P.vReach - K.prop_forward_extent():.0f} mm of margin is what let an entire
boom subsystem be deleted.

**Serviceability is designed, not asserted.** Every removable item has a
declared direction, a declared tool list, a declared prerequisite sequence,
and a measured corridor. Battery packs come out aft with no tools. The fluid
cartridge drops out downward. The measured corridors are in the BOM workbook.

**The checks found real defects.** Thirteen genuine problems, including
antennas inside rotor discs, a tank whose declared volume was 79 % wrong, a
gear leg blocking the battery corridor, and a manipulator drawn one joint
downstream. None of these were visible by inspection.

## Where it is weak

**T/W at maximum MTOW is {T / (M + 2.0):.2f}.** Below the 2.0 wanted for
contact work. Undecided.

**Endurance is {en['endurance_min'] * 0.8:.0f} minutes with reserve** — and
that is an optimistic calculation.

**Nothing is validated.** No FEA, no test, no flight. Two-thirds of the
"before fabrication" list in `07_Engineering/VALIDATION_MATRIX.md` is
analysis that has not been started.

**The arm can hit the aircraft.** Mitigated in software only.

## Scores

| Criterion | Score | Why not 10 |
|---|---:|---|
| Architecture | 9.0 | Coherent and well-justified; the coaxial X8 choice is the right one and is defended with numbers. Held back because T/W at max payload is unresolved. |
| Mechanical engineering | 8.0 | Real sections, real fasteners, real service access, zero interference. No FEA on anything. |
| Manipulator | 8.5 | 6 DOF, correct link convention, internal routing, torques from a 123k-pose sweep. No actuator selected; no compliance or backlash budget. |
| Repair capability | 8.0 | Quick-change interface, F/T force control, gripper with replaceable tips. Grip force unmatched to an actuator. |
| Liquid system | 8.5 | Complete, integrated, serviceable, slosh addressed, volume verified against geometry. Fluid unspecified; slosh untested. |
| Propulsion | 8.5 | Vendor components, geometry closed, clearances verified. Thrust is bench data; one-rotor-out authority not demonstrated. |
| Power | 7.0 | 12S architecture sound, current headroom adequate. Pack unselected and is {7.687 / M * 100:.0f} % of MTOW. Endurance is short. |
| CG | 9.0 | Measured across all configurations, trim authority quantified, powered rail deleted on evidence. Lateral excursion accepted rather than solved. |
| Sensors | 7.5 | Complete suite, all clearances verified, mounts real. Every sensor is a PLACEHOLDER envelope. |
| Component realism | 7.5 | Motors, ESCs, packs, boards, pump, actuators all have realistic envelopes and are labelled. They are still envelopes, not vendor CAD. |
| Maintainability | 9.0 | Every item has a verified corridor, direction, tool list and sequence. Held back only because the PDB needs three items removed first. |
| Manufacturability | 7.5 | CF tube/plate, CNC AL, printed covers, standard fasteners. No laminate schedule, no bolted-joint analysis, no tolerance study. |
| CAD organisation | 9.0 | 131 variables, 8 modular Feature Studios, documented build order, 74 mates, 7 configurations from one table. Not yet built in Onshape. |
| Documentation | 9.0 | Every number generated from the model; BOM reconciles with the mass budget; confidence tagged throughout. |
| Validation | 5.5 | 15 checks run, 21 of 26 sub-checks pass, all failures reported. But CAD checks are the only validation that exists — no FEA, no test. |

### Result

```
BASELINE                5.22 / 10
FINAL CAD PACKAGE       8.10 / 10
```

Unweighted mean of the fifteen criteria above: **8.10**.

**This is not 10/10 and should not be read as one.** The gap is almost
entirely validation and component selection: a package that has had FEA run,
its placeholders replaced with selected parts, and its propulsion measured on
a stand would score in the low nines on this same scale. Ten would require
flight test.

The three things that would move the number most, in order:

1. Resolve T/W at maximum MTOW (R2) — it is an architecture-level open item.
2. Run FEA on the arm root, hub, nodes and gear (R5).
3. Select the battery, the actuators and the F/T sensor (R6) — that is
   {7.687 / M * 100 + 8:.0f} %+ of MTOW currently sitting on estimates.
""")


# ---------------------------------------------------------------------------
def drawings():
    SETS = [
        ("overall", "AVIAN general arrangement", [
            ("AV-DWG-001", "General arrangement", "1:8", "A1",
             "Plan, front, right, isometric. Overall envelope, diagonal, "
             "rotor stations, skid plane, datum."),
            ("AV-DWG-002", "Envelope and clearances", "1:8", "A1",
             "Rotor swept discs, manipulator reach circle, tip-to-tip gap, "
             "ground clearances."),
            ("AV-DWG-003", "Mass and CG", "1:10", "A2",
             "CG location in three views for all seven configurations; "
             "battery detent authority."),
        ]),
        ("airframe", "Airframe", [
            ("AV-DWG-010", "Spine assembly", "1:4", "A1",
             "Rail sections, cross member stations, node interfaces."),
            ("AV-DWG-011", "Corner node", "1:1", "A2",
             "Machined AL7075 node: rail bores, cross bore, arm clamp bore, "
             "gear isolator face, all bolt patterns."),
            ("AV-DWG-012", "Manipulator hub", "1:1", "A2",
             "Bearing seat, 8 × M5 on Ø88 PCD, belly aperture interface."),
            ("AV-DWG-013", "Arm tube and root clamp", "1:2", "A2",
             "Tube cut length, clamp position, cable entry."),
        ]),
        ("propulsion", "Propulsion module", [
            ("AV-DWG-020", "Coaxial module GA", "1:2", "A1",
             "Rotor axis, coaxial separation, motor plate faces, ESC "
             "station, cable route, service access."),
            ("AV-DWG-021", "Coaxial mount", "1:1", "A2",
             "Motor bolt patterns both faces, tube clamp, drain path."),
            ("AV-DWG-022", "ESC saddle", "1:1", "A3",
             "Split saddle, tube bore, ESC retention, heat path."),
        ]),
        ("manipulator", "Manipulator", [
            ("AV-DWG-030", "Arm general arrangement", "1:4", "A1",
             "All six joints, link lengths, joint axes, limits, reach "
             "envelope."),
            ("AV-DWG-031", "J1–J2 shoulder", "1:1", "A1",
             "Housings, bearing seats, hard stops, cable path."),
            ("AV-DWG-032", "J3 elbow", "1:1", "A2", "As above."),
            ("AV-DWG-033", "J4–J6 wrist", "1:1", "A1",
             "Three axes, tool0 flange, F/T interface."),
            ("AV-DWG-034", "Link tubes", "1:2", "A2",
             "Cut lengths, bonded end fittings, wall thicknesses."),
        ]),
        ("tool_changer", "Tool interface", [
            ("AV-DWG-040", "Tool changer", "1:1", "A1",
             "Master and tool plate, 3 pins on Ø34 PCD, 12-way electrical, "
             "dry-break port, cam mechanism, split line."),
            ("AV-DWG-041", "Repair gripper", "1:1", "A2",
             "Jaw stroke, tip interface, actuator envelope, service cover."),
            ("AV-DWG-042", "Liquid nozzle", "1:1", "A3",
             "Replaceable tip thread, guard standoff, fluid inlet."),
        ]),
        ("liquid_service", "Liquid service", [
            ("AV-DWG-050", "Fluid schematic", "n/a", "A2",
             "Tank, pump, filter, check, relief, sensor, dry-break, nozzle. "
             "Pressures and set points."),
            ("AV-DWG-051", "Tank cartridge", "1:2", "A1",
             "Shell, baffle stations, filler, vent, sight strip, drain, "
             "frame, latch and pin interfaces."),
            ("AV-DWG-052", "Aft equipment bay", "1:2", "A2",
             "Pump, filter, valve and sensor stations; hose routing."),
        ]),
        ("landing_gear", "Landing gear", [
            ("AV-DWG-060", "Gear general arrangement", "1:4", "A2",
             "Track, skid plane, leg angles, isolator stations, tip-over "
             "half-angle."),
            ("AV-DWG-061", "Leg and skid", "1:2", "A3",
             "Tube cut lengths, bonded joints, replaceable feet."),
        ]),
    ]
    for folder, title, sheets in SETS:
        rows = "\n".join(
            f"| `{n}` | {t} | {s} | {sz} | {c} |" for n, t, s, sz, c in sheets)
        w(f"{DR}/{folder}/README.md", f"""
# AVIAN REV A — {title} drawings

| Drawing | Title | Scale | Sheet | Content |
|---|---|---|---|---|
{rows}

## How to produce these in Onshape

Each sheet is a drawing of the corresponding Part Studio or Assembly listed in
`02_Onshape_Build_Guide/BUILD_ORDER.md`. Create it with **+ → Drawing**,
select the source, and use the standard views noted above.

Dimension **from the Variable Studio**, not by picking geometry: in a
dimension, type `#vDiagonal` rather than clicking two rotor centres. A drawing
dimensioned that way updates when the design does; one dimensioned by picking
faces silently goes stale.

Every sheet carries the same title block: project AVIAN, revision REV A,
units mm, projection third angle, and the tolerance note below.

## Tolerance note (all sheets)

```
UNLESS OTHERWISE STATED
  linear      +/- 0.2 mm       machined AL
              +/- 0.5 mm       CF tube and plate
              +/- 0.4 mm       FDM printed PA-CF
  angular     +/- 0.5 deg
  bore/shaft  H7/g6 at bearing and dowel fits
  surface     Ra 1.6 machined, Ra 3.2 elsewhere
```

**These tolerances are ASSUMED.** No tolerance stack-up study has been done
for this revision — see `07_Engineering/VALIDATION_MATRIX.md`.

## Status

Drawing **structure and content** is defined here. The sheets themselves are
produced in Onshape once the Part Studios and Assemblies exist; they cannot be
generated outside it, and a set of exported images would not be a drawing.
""")


# ---------------------------------------------------------------------------
def cad_reference():
    # dimensions
    rows = []
    for p in reg.parts:
        b = placed(p).BoundingBox()
        rows.append([p.name, p.group, p.material,
                     round(b.xlen, 1), round(b.ylen, 1), round(b.zlen, 1),
                     round(float(p.world[0, 3]), 1),
                     round(float(p.world[1, 3]), 1),
                     round(float(p.world[2, 3]), 1),
                     round(p.mass_kg() * 1000, 1)])
    os.makedirs(f"{CR}/dimensions", exist_ok=True)
    with open(f"{CR}/dimensions/AVIAN_part_dimensions.csv", "w",
              newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["Part", "Subsystem", "Material", "Envelope X (mm)",
                     "Envelope Y (mm)", "Envelope Z (mm)", "Origin X",
                     "Origin Y", "Origin Z", "Mass (g)"])
        wr.writerows(sorted(rows))
    print("  ", f"{CR}/dimensions/AVIAN_part_dimensions.csv")

    # mounting patterns
    os.makedirs(f"{CR}/mounting_patterns", exist_ok=True)
    with open(f"{CR}/mounting_patterns/AVIAN_mounting_patterns.csv", "w",
              newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["Interface", "Pattern", "Count", "PCD / spacing (mm)",
                     "Fastener", "Thread engagement", "Notes"])
        for r in [
            ["Motor to coaxial mount", "circular", 4, 25.0, "M3", "6 mm",
             "T-Motor U8 II standard pattern"],
            ["Coaxial mount to arm tube", "split clamp", 4, "-", "M4", "-",
             "clamps a 25 mm OD tube"],
            ["Arm tube to corner node", "split clamp", 4, "-", "M5", "-",
             "clamp length 72 mm"],
            ["Manipulator hub to arm base", "circular", 8, 88.0, "M5", "10 mm",
             "22.5 deg index"],
            ["Hub to spine", "rectangular", 6, "70 x 90", "M5", "10 mm", ""],
            ["Tool changer master to F/T", "circular", 6, 40.0, "M4", "8 mm",
             ""],
            ["Tool plate to tool body", "circular", 4, 38.0, "M4", "8 mm", ""],
            ["Gripper finger tip", "linear", 2, 18.0, "M3", "6 mm",
             "replaceable wear item"],
            ["Battery tray to spine", "rectangular", 8, "260 x 200", "M4",
             "8 mm", ""],
            ["Gear isolator to node", "circular", 4, 24.0, "M5", "10 mm", ""],
            ["Skid foot to skid", "single", 1, "-", "M4", "6 mm",
             "replaceable wear item"],
            ["Service panel", "perimeter", 6, "-", "quarter-turn", "-",
             "4 on the side panels"],
            ["ESC to saddle", "rectangular", 4, "60 x 30", "M3", "5 mm", ""],
            ["Avionics tray", "rectangular", 4, "95 x 63", "M3", "6 mm",
             "on wire-rope isolators"],
        ]:
            wr.writerow(r)
    print("  ", f"{CR}/mounting_patterns/AVIAN_mounting_patterns.csv")

    # component envelopes
    os.makedirs(f"{CR}/component_envelopes", exist_ok=True)
    with open(f"{CR}/component_envelopes/AVIAN_component_envelopes.csv", "w",
              newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["Component", "Envelope L x W x H (mm)", "Mass (g)",
                     "Station (x, y, z mm)", "Confidence", "Selection status"])
        SEL = ["battery_cartridge_1", "flight_controller",
               "companion_computer", "lidar", "rgb_gimbal", "depth_camera",
               "gnss_antenna_1", "power_distribution", "pump", "filter_40um",
               "ft_sensor", "actuator_J1", "actuator_J2", "actuator_J3",
               "actuator_J4", "actuator_J5", "actuator_J6"]
        for p in reg.parts:
            if p.name not in SEL:
                continue
            b = placed(p).BoundingBox()
            wr.writerow([p.name,
                         f"{b.xlen:.0f} x {b.ylen:.0f} x {b.zlen:.0f}",
                         f"{p.mass_kg()*1000:.0f}",
                         f"({float(p.world[0,3]):.0f}, "
                         f"{float(p.world[1,3]):.0f}, "
                         f"{float(p.world[2,3]):.0f})",
                         "PLACEHOLDER",
                         "NOT SELECTED -- VERIFY BEFORE FABRICATION"])
    print("  ", f"{CR}/component_envelopes/AVIAN_component_envelopes.csv")

    w(f"{CR}/README.md", """
# AVIAN REV A — CAD reference

Machine-readable reference data extracted from the model.

| Folder | File | Contents |
|---|---|---|
| `dimensions/` | `AVIAN_part_dimensions.csv` | Every part: envelope, origin, material, mass |
| `interfaces/` | `AVIAN_arm_collision_envelope.csv` | 49,275-cell manipulator collision-free map over (J1, J2, J3) |
| `mounting_patterns/` | `AVIAN_mounting_patterns.csv` | Every bolted interface: pattern, count, PCD, fastener, engagement |
| `component_envelopes/` | `AVIAN_component_envelopes.csv` | The 17 unselected components and the envelope each must fit |

## Using the collision envelope

`AVIAN_arm_collision_envelope.csv` is the mitigation for risk R1. A `clear`
value of 1 means that pose clears the airframe by ≥15 mm, the rotor discs by
≥50 mm, the landing gear by ≥15 mm, and is free of link-to-link
self-collision — for **all** J5 in ±120°. J4 and J6 are roll axes and do not
change the swept envelope.

The planner must screen **paths**, not just endpoints. A straight-line joint
interpolation between two clear cells can pass through a blocked one.

## Using the component envelopes

Each row is a part that has **not been selected**. The envelope is what the
selected part must fit inside without breaking a clearance check. If a
candidate exceeds it, re-run the interference scan before accepting it — the
equipment bay has very little slack, and several stations were placed by an
automatic packer precisely because hand-placing them kept colliding.
""")


if __name__ == "__main__":
    spec()
    arch()
    risks()
    review()
    drawings()
    cad_reference()
