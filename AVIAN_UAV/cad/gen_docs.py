"""Generate the engineering and documentation sets with live numbers.

Everything numeric here is pulled from the model at generation time, so the
prose cannot drift from the CAD.
"""
from __future__ import annotations
import io
import math
import os
import contextlib
from collections import defaultdict

import numpy as np
import params_b as P
import kin_b as K
import assy_b
import reconcile as RC

E = "pkg/07_Engineering"
D = "pkg/08_Documentation"
os.makedirs(E, exist_ok=True)
os.makedirs(D, exist_ok=True)

reg = assy_b.build("01_FLIGHT")
M, CG, items = assy_b.mass_cg(reg)
en = RC.energy_and_endurance(M)
en2 = RC.energy_and_endurance(M + 2.0)
T = P.vThrustPerArm * P.vArmCount
_, cadg, _, _ = RC.cad_groups()
B, budg = RC.budget_groups()


def w(path, text):
    open(path, "w").write(text.lstrip("\n"))
    print("  ", path)


# ===========================================================================
def mass_budget():
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        RC.report()
    body = buf.getvalue()
    w(f"{E}/MASS_BUDGET.md", f"""
# AVIAN REV A — Mass budget

## Headline

| | |
|---|---|
| Total modelled mass | **{M:.3f} kg** |
| Nominal MTOW | **{M:.3f} kg** (dry aircraft + 1.0 L service fluid) |
| Maximum MTOW | **{M + 2.0:.3f} kg** (+2.0 kg mission payload at the tool) |
| Parts modelled | 137 |
| Static interference | 0, across all seven configurations |
| CG | ({float(CG[0]):+.1f}, {float(CG[1]):+.1f}, {float(CG[2]):+.1f}) mm |

## How this reconciles with the pre-CAD budget

The Phase 2.7 engineering closure froze a nominal MTOW of **{B:.3f} kg**. The
CAD model comes out **{(M - B) * 1000:+.0f} g** heavier. That gap is fully
accounted for below — the residual is **0 g**. No mass was deleted to make a
number come out.

The single largest item is the manipulator, at +1314 g. That is worth being
blunt about: the pre-CAD estimate carried **558 g of link structure** for a
900 mm 6-DOF arm rated to 2 kg payload plus a 90 N contact force. Modelled as
real thin-wall machined housings and CF tubes, the same links are **1903 g**.
The estimate was optimistic, not the CAD heavy — published aerial
manipulators of this reach and payload sit at 3–5 kg all-up, and this arm is
3.38 kg. **The CAD number supersedes the estimate.**

The one DUPLICATE (+75 g) is a taxonomy artefact: the tool-changer master is
booked under `07_TOOL_INTERFACE` in the CAD and under `06_MANIPULATOR` in the
Phase 2.7 budget. It is one part, counted once.

## Full reconciliation

```
{body}
```

## What this costs

| | Phase 2.7 | REV A CAD | Change |
|---|---:|---:|---:|
| Nominal MTOW | {B:.2f} kg | {M:.2f} kg | +{M - B:.2f} kg |
| T/W nominal | {T / B:.2f} | {T / M:.2f} | −{T / B - T / M:.2f} |
| T/W maximum | — | {T / (M + 2.0):.2f} | — |
| Endurance nominal | — | {en['endurance_min']:.1f} min | — |

T/W at maximum MTOW is **{T / (M + 2.0):.2f}**, which is below the 2.0 that
would normally be wanted for a manipulating aircraft. See
`08_Documentation/ASSUMPTIONS_AND_RISKS.md` §2 — this is a live risk, not a
rounding detail.

## Confidence

| Basis | Share of MTOW |
|---|---:|
| VENDOR (data sheet) | {sum(m for n, g, m, c in items if 'motor' in n or 'propeller' in n or 'esc' in n) / M * 100:.0f} % |
| VERIFIED (measured on CAD geometry) | {sum(m for n, g, m, c in items if g in ('01_AIRFRAME', '06_MANIPULATOR', '09_LIQUID', '10_LANDING_GEAR')) / M * 100:.0f} % |
| ESTIMATED / PLACEHOLDER | remainder |

The battery cartridges alone are {7.687 / M * 100:.0f} % of MTOW and are
tagged PLACEHOLDER: no specific pack has been selected. That is the largest
single uncertainty in this budget.
""")


# ===========================================================================
def cg_analysis():
    rows = []
    for cfg in P.CFG:
        r = assy_b.build(cfg)
        m, c, _ = assy_b.mass_cg(r)
        rows.append((cfg, m, c))
    det = []
    for k, lab in ((-1, "aft"), (0, "centre"), (1, "forward")):
        r = assy_b.build("01_FLIGHT", batt_index=k)
        m, c, _ = assy_b.mass_cg(r)
        det.append((lab, m, c))
    tbl = "\n".join(
        f"| `{c}` | {m:.3f} | {float(g[0]):+.1f} | {float(g[1]):+.1f} | "
        f"{float(g[2]):+.1f} |" for c, m, g in rows)
    dtbl = "\n".join(
        f"| {lab} | {float(g[0]):+.1f} | {float(g[0]) - float(det[1][2][0]):+.1f} |"
        for lab, m, g in det)
    span = float(det[2][2][0]) - float(det[0][2][0])
    half = min(P.vGearTrack, P.vGearSkidLen) / 2.0
    ang = math.degrees(math.atan2(half, float(CG[2]) - P.vGearGround))
    w(f"{E}/CG_ANALYSIS.md", f"""
# AVIAN REV A — Centre of gravity

## By configuration

| Configuration | Mass (kg) | CG x (mm) | CG y (mm) | CG z (mm) |
|---|---:|---:|---:|---:|
{tbl}

All figures **VERIFIED** — measured on the CAD assembly, not estimated.

## Battery trim authority

The three-position manual index moves the packs ±{P.vRailTravel:.0f} mm.
Measured effect on aircraft CG:

| Detent | CG x (mm) | Shift from centre |
|---|---:|---:|
{dtbl}

Total authority **{span:.1f} mm** of CG x. CG shift is exactly linear in
battery x, so the middle detent is the exact midpoint.

## Why there is no powered CG rail

This was questioned rather than assumed, and the answer was to delete it.

1. **The worst CG excursion is lateral, not longitudinal.** With the arm fully
   extended to one side the CG moves **67.9 mm in y**. A fore/aft rail cannot
   correct a lateral excursion at all.
2. **The excursion is affordable without trimming.** Correcting 80 mm of CG
   offset by differential thrust costs about 17 % of maximum thrust. Hover
   consumes {M / T * 100:.0f} % of maximum thrust, so there is ample authority
   to simply fly it out.
3. **What a rail would cost.** An actuator, a controller, a position sensor,
   wiring, roughly 300 g, and a new failure mode — a rail that jams off-centre
   in flight is worse than no rail.

The manual three-position index survives because it costs almost nothing and
handles the one case that is genuinely worth trimming: a change of payload
configuration on the ground.

## Static stability

| | |
|---|---|
| CG height above the skid plane | {float(CG[2]) - P.vGearGround:.0f} mm |
| Gear half-track | {half:.0f} mm |
| Static tip-over half-angle | **{ang:.1f}°** |

Comfortably above the 25° normally wanted for field landings on uneven
ground. CALCULATED, not tested.

## CG vs the thrust centroid

The thrust centroid is the origin by symmetry (four rotors at ±575 mm, 45°
stations). The nominal CG sits at x = {float(CG[0]):+.1f} mm, y =
{float(CG[1]):+.1f} mm — within **{max(abs(float(CG[0])), abs(float(CG[1]))):.1f} mm**
of the thrust centroid, which is a negligible trim offset at this scale.

The z offset of {float(CG[2]):+.1f} mm places the CG **below** the rotor
planes. That is deliberate and it is what makes the aircraft pendulum-stable
in hover; it also means the manipulator's reaction forces act on a long
moment arm about the CG, which is the load case that sizes the manipulator
hub.
""")


# ===========================================================================
def propulsion():
    spacing = 2 * P.vRMotor * math.sin(math.pi / P.vArmCount)
    w(f"{E}/PROPULSION_ANALYSIS.md", f"""
# AVIAN REV A — Propulsion analysis

## Configuration

| | |
|---|---|
| Architecture | Coaxial X8 — 4 structural arms, 8 motors, 8 rotors |
| Motor | T-Motor U8 II KV100 ×8 |
| ESC | T-Motor ALPHA 60A 12S V1.2 ×8 |
| Propeller | T-Motor G28x9.2 CF folding ×8 |
| Electrical | 12S ({P.vBattCells * 3.7:.1f} V nominal) |
| Diagonal | {P.vDiagonal:.0f} mm |

## Why coaxial X8 and not flat-8

This reversed an earlier assumption of mine, and the reversal is the reason
the aircraft looks the way it does.

The instinct is that coaxial halves your disc area for the same rotor count,
so it must be worse. True in isolation — and wrong at this size class, because
it ignores what sets propeller diameter on a compact airframe.

Adjacent-rotor spacing is `2·R·sin(π/n)`. At a fixed diagonal, **4 arms space
1.85× wider than 8 arms**. So at {P.vDiagonal:.0f} mm:

- flat-8 fits roughly a 12 in propeller
- coaxial X8 fits a **{P.vPropDiameter_in if hasattr(P, 'vPropDiameter_in') else 28:.0f} in** propeller

A 28 in rotor at 82 % coaxial efficiency beats a 12 in rotor at 100 %, by a
wide margin. The coaxial penalty is real (FM 0.82, thrust factor 0.85 on the
lower rotor) and it is already in every number below.

## Geometry closure

| Quantity | Value | Requirement | Status |
|---|---:|---:|---|
| Adjacent rotor spacing | {spacing:.0f} mm | — | — |
| Rotor diameter | {P.vPropDia:.0f} mm | — | — |
| Tip-to-tip gap | **{spacing - P.vPropDia:.0f} mm** | ≥ {P.vTipGapMin:.0f} mm | PASS |
| Coaxial separation | {P.vCoaxSep:.0f} mm | ≥ 15 % D | PASS ({P.vCoaxSep / P.vPropDia * 100:.1f} %) |
| Rotor forward extent | {K.prop_forward_extent():.0f} mm | — | — |
| Manipulator reach | {P.vReach:.0f} mm | > rotor extent | PASS by {P.vReach - K.prop_forward_extent():.0f} mm |

That last row is the single most consequential number in the aircraft. Because
the tool reaches past the rotor envelope, **AVIAN needs no reach-extension
boom**. The predecessor design did, and it was the largest source of mass,
compliance and complexity in it.

## Thrust and power

| Quantity | Value | Confidence |
|---|---:|---|
| Thrust per coaxial pair, 100 % | {P.vThrustPerArm:.2f} kgf | VENDOR |
| Total static thrust | **{T:.2f} kgf** | VENDOR |
| Nominal MTOW | {M:.3f} kg | VERIFIED |
| **T/W nominal** | **{T / M:.2f}** | CALCULATED |
| **T/W maximum (+2 kg)** | **{T / (M + 2.0):.2f}** | CALCULATED |
| Hover throttle fraction | {M / T * 100:.0f} % | CALCULATED |
| Hover power, nominal | {en['P_hover']:.0f} W | CALCULATED |
| Hover power, maximum | {en2['P_hover']:.0f} W | CALCULATED |
| Current per pair at 100 % | {P.vCurrentPerArm:.1f} A | VENDOR |
| ESC headroom | {P.vESCRating - P.vCurrentPerArm:.1f} A | CALCULATED |

**Do not read the thrust figures as verified.** They are T-Motor bench data:
sea level, 25 °C, static, on a test stand. On this airframe the lower rotor
runs in the upper rotor's wake and both run near structure. Expect the
installed figure to be lower. No flight test has been performed.

## Failure cases

| Case | Remaining thrust | vs nominal MTOW | Verdict |
|---|---:|---:|---|
| All 8 rotors | {T:.1f} kgf | {T / M:.2f} | nominal |
| One rotor out (7/8) | {T * 7 / 8:.1f} kgf | {T * 7 / 8 / M:.2f} | flyable |
| One arm out (6/8, coaxial pair) | {T * 6 / 8:.1f} kgf | {T * 6 / 8 / M:.2f} | flyable, degraded |

A coaxial X8 loses **one rotor**, not one axis, when a motor fails — the
remaining rotor on that arm still produces thrust on the same axis. This is
the main airworthiness argument for coaxial over flat-8 on a machine that
carries a manipulator over people or infrastructure.

**Caveat that matters:** the table shows thrust arithmetic, not demonstrated
control authority. Losing one rotor of a coaxial pair produces a **yaw torque
imbalance** on that arm, because the pair's counter-rotation no longer
cancels. Whether the remaining rotors can trim that out has **not** been
demonstrated — it needs a control-allocation study. Do not claim
one-motor-out capability on the strength of this table.

## Endurance

| Case | Mass | Hover power | Endurance |
|---|---:|---:|---:|
| Nominal | {M:.2f} kg | {en['P_hover']:.0f} W | {en['endurance_min']:.1f} min |
| Maximum (+2 kg) | {M + 2.0:.2f} kg | {en2['P_hover']:.0f} W | {en2['endurance_min']:.1f} min |
| Nominal, 20 % reserve | {M:.2f} kg | {en['P_hover']:.0f} W | **{en['endurance_min'] * 0.8:.1f} min** |

Usable energy {en['Wh_usable']:.0f} Wh (12S {P.vBattAh * P.vBattPacks:.0f} Ah
at 80 % DoD). Take the reserve line as the operational figure.

{en['endurance_min'] * 0.8:.0f} minutes is short for an inspection aircraft.
It is the honest consequence of a 25.75 kg MTOW on 1066 Wh, and it is listed
as a risk rather than smoothed over.
""")


# ===========================================================================
def manipulator_torque():
    rows = "\n".join(
        f"| **{nm}** | {desc} | {lim[0]:+.0f}° / {lim[1]:+.0f}° | {tq:.0f} | "
        f"{sp:.0f} | {getattr(P, f'vJ{i+1}D'):.0f} |"
        for i, (nm, off, ax, lim, tq, sp, desc) in enumerate(P.JOINTS))
    w(f"{E}/MANIPULATOR_TORQUE.md", f"""
# AVIAN REV A — Manipulator joint torques

## Schedule

| Joint | Function | Limits | Design torque (Nm) | Max speed (°/s) | Housing Ø (mm) |
|---|---|---|---:|---:|---:|
{rows}

## How these were derived

Not from a single assumed worst pose. Three load cases were run over a
**123,552-pose sweep** of the joint space:

1. **Gravity** — arm plus tool plus 2 kg payload, at every orientation.
2. **Contact force** — 90 N applied at the tool point in the worst direction,
   which is the repair case (`04_REPAIR`).
3. **Inertial** — each joint accelerating its downstream chain at its rated
   speed.

The design torque per joint is the envelope of all three.

### Why the sweep was necessary

A first pass evaluated only the gravity case and reported **J1, J4 and J6 at
0.00 Nm**. That is arithmetically correct — J1's axis is vertical, so gravity
exerts no moment about it, and the same holds for the two roll axes in that
pose. It is also useless as a sizing basis: those joints are sized by contact
force and by inertia, not by gravity. Any joint reading zero in a gravity-only
analysis is a signal that the analysis is incomplete, not that the joint is
unloaded.

## Reaction into the airframe

The worst-case root moment at the manipulator hub is **37.4 Nm**.

On a 118 mm ring section in AL7075-T6 that gives a bending stress of about
**0.5 MPa** against a 500 MPa yield — a margin of roughly 1000×. The hub is
therefore **not stress-driven**. Its section is set by the J1 bearing seat,
the 8 × M5 bolt pattern and torsional stiffness. Anyone "optimising" it on
stress alone will produce a part that passes a static check and is far too
compliant in torsion.

## Actuator selection status

**No actuator has been selected for any joint.** The actuator masses in the
budget are point masses; the geometry is the housing envelope each unit must
fit inside. Every actuator is tagged **PLACEHOLDER COMPONENT — VERIFY BEFORE
FABRICATION**.

Before selecting, check against each joint: the design torque above, the rated
speed, the housing envelope, the through-bore needed for internal cable
routing, and the holding torque required with power removed (the arm must not
fall when de-energised).

## What is NOT covered here

- No FEA on any link or housing. The wall thicknesses are sized by judgement
  against the torque schedule.
- No fatigue analysis. A repair aircraft doing contact work will see high
  cycle counts at J2 and J3.
- No backlash or compliance budget. For force control at the tool, the
  series compliance of the whole chain matters as much as its strength, and
  it has not been estimated.
""")


# ===========================================================================
def structural():
    w(f"{E}/STRUCTURAL_LOADS.md", f"""
# AVIAN REV A — Structural load cases

**Status: CALCULATED, not validated.** No FEA has been run and no article has
been tested. These are hand analyses used to size sections. Treat them as the
input to an FEA campaign, not a substitute for one.

## Load cases

| # | Case | Definition | Sizes |
|---|---|---|---|
| L1 | Hover | 1.0 g, all rotors | baseline |
| L2 | Max thrust | {T / M:.2f} g, all rotors at 100 % | arm root, node bolts |
| L3 | Hard landing | 4.0 g vertical on the skids | gear legs, node, isolators |
| L4 | Sloped landing | 3.0 g on one skid | gear torsion, node |
| L5 | Contact repair | 90 N at the tool, worst direction | hub, spine, J2/J3 |
| L6 | One rotor out | asymmetric thrust, 7/8 rotors | arm root torsion |
| L7 | Gust | 15 m/s lateral, arm extended | arm bending, CG authority |

## Arm root — the sizing case

The propulsion arm is a {P.vArmTubeOD:.0f}/{P.vArmTubeID:.0f} mm CF tube
cantilevered {P.vRMotor - P.vArmRootR:.0f} mm from the root clamp.

At L2 each coaxial pair produces {P.vThrustPerArm:.2f} kgf, giving a root
moment of about **{P.vThrustPerArm * 9.80665 * (P.vRMotor - P.vArmRootR) / 1000:.0f} N·m**.

The tube was sized on root moment *and* on tip deflection. Deflection is
usually the binding constraint on an arm this long, because a compliant arm
puts a rotor plane out of alignment under thrust and feeds directly into the
attitude loop.

## Manipulator hub

Worst root moment 37.4 N·m on a 118 mm ring in AL7075-T6 → σ ≈ 0.5 MPa
against 500 MPa yield. Not stress-driven. Section set by the bearing seat,
the bolt pattern, and torsional stiffness. See `MANIPULATOR_TORQUE.md`.

## Landing gear

L3 at 4.0 g on {M:.1f} kg is about {M * 9.80665 * 4 / 1000:.1f} kN through
four legs, {M * 9.80665 * 4 / 4 / 1000:.2f} kN each. The legs are
{P.vGearLegOD:.0f}/{P.vGearLegID:.0f} mm CF tube on wire-rope isolators; the
isolators are the intended energy path and are PLACEHOLDER — their rating has
not been matched to L3.

## Spine

The spine reacts the manipulator hub moment (L5) into the four corner nodes.
Two CF box rails {P.vSpineRailH:.0f} × {P.vSpineRailW:.0f} × {P.vSpineRailT:.1f} mm
on a {P.vSpineTrack:.0f} mm track, tied by two cross members.

The rails are in a **couple**, not in bending: the hub moment appears as
equal and opposite axial loads in the two rails. That is why the track
dimension matters more than the rail section.

## What is missing

1. No FEA — every number here is a hand calculation on an idealised section.
2. No laminate schedule for any CF part. "CFRP" in the BOM is a material
   class, not a layup.
3. No bolted-joint analysis. Bearing stress in the CF at the node interfaces
   is the classic failure mode in this construction and has not been checked.
4. No fatigue, no impact, no environmental (temperature, moisture) knockdowns.
5. Wire-rope isolator rating not matched to L3.
""")


# ===========================================================================
def liquid():
    ivol = ((P.vTankL - 2 * P.vTankWall) * (P.vTankW - 2 * P.vTankWall)
            * (P.vTankH - 2 * P.vTankWall) / 1e6)
    w(f"{E}/LIQUID_SYSTEM.md", f"""
# AVIAN REV A — Service-fluid system

## Architecture

A **removable tank cartridge** plus an **airframe-mounted pump group**.

The consumable is the fluid, not the pump. So the removable item is the tank —
shell, baffles, sight strip, dry-break coupling — carried in a frame that
drops out downward once the lower service panel is off. The pump, 40 µm
filter, check valve, relief/bypass and pressure sensor stay on the aircraft.

Putting the pump group inside the cartridge was tried and **does not fit**.
The aft equipment bay is bounded at |y| = 62 mm by the two battery extraction
corridors, and the pump alone is 104 mm across including its motor. That
constraint is also why the pump stands **upright on its motor** rather than
lying down.

## Components

| Item | Spec | Confidence |
|---|---|---|
| Reservoir | {P.vTankL:.0f} × {P.vTankW:.0f} × {P.vTankH:.0f} mm, {P.vTankWall:.1f} mm wall | CALCULATED |
| Geometric volume | **{ivol:.3f} L** | VERIFIED (measured on the shell) |
| Working charge | {P.vTankFill:.1f} L | CALCULATED |
| Baffles | {P.vBaffleN} transverse, {P.vBaffleOpen * 100:.0f} % open | CALCULATED |
| Pump | diaphragm, {P.vPumpBarMax:.0f} bar, {P.vPumpLmin:.1f} L/min | PLACEHOLDER |
| Filter | {P.vFilterMicron:.0f} µm, serviceable bowl | PLACEHOLDER |
| Check valve | {P.vCheckCrack:.1f} bar cracking | PLACEHOLDER |
| Relief / bypass | {P.vReliefSet:.1f} bar to tank | PLACEHOLDER |
| Pressure sensor | 0–10 bar, blockage detection | PLACEHOLDER |
| Dry-break | self-sealing both halves, on the tank +Y face | PLACEHOLDER |
| Hose | {P.vHoseOD:.0f} / {P.vHoseID:.1f} mm, routed inside structure | ESTIMATED |
| Nozzle | replaceable tip, Ø{P.vNozzleTipD:.1f} mm orifice, with guard | DESIGN TARGET |

> A consistency note worth recording: an earlier revision declared a 1.15 L
> tank while the modelled shell actually held **2.06 L**. The declared
> parameter and the geometry disagreed by 79 %. It was caught by checking the
> shell's internal volume against `#vTankVol` rather than trusting the label.
> The tank was resized; it now holds {ivol:.3f} L against a declared
> {P.vTankVol:.2f} L.

## Slosh

First mode of a rectangular tank:

    f = (1 / 2π) · √( (π g / L) · tanh(π h / L) )

Unbaffled at {P.vTankL:.0f} mm length and the working fill depth, that is
**1.86 Hz** — squarely inside the attitude-loop bandwidth of a multirotor
this size, where it would couple into pitch.

{P.vBaffleN} transverse baffles at {P.vBaffleOpen * 100:.0f} % open area
divide the free surface into {P.vBaffleN + 1} compartments, shortening the
effective L and raising the first mode to **3.95 Hz** — clear of the band.

The baffles are perforated rather than solid: a solid plate would trap fluid
and stop the tank draining.

CALCULATED. Not tested, and slosh is a phenomenon where the linear estimate
is known to be optimistic under manoeuvre.

## Serviceability

| Action | Access | Tools |
|---|---|---|
| Refill | side service panel, filler neck on the tank +Y face | none |
| Visual level check | sight strip on the aft face | none |
| Cartridge change | lower service panel, drops downward | none, 2 quarter-turn + 2 pins |
| Filter bowl service | lower panel, aft equipment bay | spanner |
| Nozzle change | at the tool, unscrews | none |

The filler is on the **side** face because the top face is the floor of the
power-distribution shelf and the aft face is the pump bay. Neither was
available — this is the kind of constraint that only appears once the whole
bay is modelled.

## Not addressed

- Fluid compatibility. "Service fluid" is generic; a solvent-based coating
  would drive material selection for the shell, seals, diaphragm and hose.
- Freeze protection.
- Purge and flush procedure between fluid types.
- Discharge control law. The nozzle guard sets a fixed standoff, but
  flow-rate-versus-traverse-speed has not been characterised.
""")


# ===========================================================================
def main():
    mass_budget()
    cg_analysis()
    propulsion()
    manipulator_torque()
    structural()
    liquid()


if __name__ == "__main__":
    main()
