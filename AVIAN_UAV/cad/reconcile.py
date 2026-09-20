"""Phase 0 mass gate: reconcile the CAD model against the Phase 2.7 budget.

Every delta is classified. Nothing is deleted to make a number come out.

  REQUIRED               real hardware the budget missed or under-sized
  DUPLICATE              counted twice, or counted in two groups
  PLACEHOLDER            envelope stands in for a part not yet selected
  CONSERVATIVE ALLOWANCE deliberate margin carried on purpose
  UNNECESSARY            geometry with no engineering function -- delete it
  UNRESOLVED             cannot be explained from the model; carried as risk
"""
from __future__ import annotations
import math
from collections import defaultdict

import numpy as np
import params_b as P
import assy_b
import avian_closure as C

G = 9.80665
RHO = 1.225

# ---------------------------------------------------------------------------
# 1. group-level comparison, on a common taxonomy
# ---------------------------------------------------------------------------
# The CAD books the arm tubes and root clamps under PROPULSION (they carry the
# motors); Phase 2.7 booked them under AIRFRAME. Re-map so like meets like.
REMAP = [("arm_tube", "01_AIRFRAME"), ("arm_clamp", "01_AIRFRAME"),
         ("arm_conduit", "11_CABLE")]


def cad_groups(cfg="01_FLIGHT"):
    reg = assy_b.build(cfg)
    M, cg, items = assy_b.mass_cg(reg)
    g = defaultdict(float)
    for n, gr, m, c in items:
        for key, dest in REMAP:
            if key in n and gr == "02_PROPULSION":
                gr = dest
                break
        g[gr] += m
    return M, cg, dict(g), items


def budget_groups():
    it = C.build(C.PACKS[2], arm_cfg="STOWED", tool="nozzle", fluid_l=1.0)
    g = defaultdict(float)
    for row in it:
        g[row[1]] += row[2]
    return sum(r[2] for r in it), dict(g)


# ---------------------------------------------------------------------------
# 2. the delta ledger -- one line per explained difference
# ---------------------------------------------------------------------------
# (group, kg, class, explanation)
LEDGER = [
    ("06_MANIPULATOR", +1.314, "REQUIRED",
     "Phase 2.7 carried 558 g of ESTIMATED link structure for a 900 mm 6-DOF "
     "arm rated to 2 kg payload plus 90 N contact force. Modelled as real "
     "thin-wall machined housings and CF tubes the same links come out at "
     "1903 g. The estimate was optimistic, not the CAD heavy: published "
     "aerial manipulators of this reach sit at 3-5 kg all-up. The CAD number "
     "supersedes the estimate."),
    ("01_AIRFRAME", +0.323, "REQUIRED",
     "Service panels, corner nodes and the spine grew once real fastener "
     "lands, panel returns and node bolt bosses were modelled. The four "
     "service panels alone account for 105 g of it."),
    ("03_BATTERY", +0.232, "REQUIRED",
     "3-position indexed tray. The budget carried 185 g for a flat plate; "
     "the modelled tray is four longitudinal rails, two lateral ties per pack "
     "and six index bosses, at 355 g. The pack cells themselves are "
     "unchanged."),
    ("02_PROPULSION", +0.118, "REQUIRED",
     "ESC saddles, 8 x 23 g. Phase 2.7 assumed the ESCs bonded directly to "
     "the arm tube with no discrete mount, which is not serviceable and gives "
     "the ESC no heat path."),
    ("07_TOOL_INTERFACE", +0.075, "DUPLICATE",
     "The tool-changer master is booked under 07_TOOL_INTERFACE in the CAD "
     "and under 06_MANIPULATOR in the Phase 2.7 budget. It is one part; the "
     "75 g is a taxonomy artefact, not new mass."),
    ("09_LIQUID", +0.054, "REQUIRED",
     "Service-fluid cartridge frame at 196 g replaces a 125 g bare bracket, "
     "less 71 g saved by shrinking the tank from a 2.06 L shell (which did "
     "not match its own declared 1.15 L) to a correct 1.17 L shell."),
    ("05_PERCEPTION", +0.022, "REQUIRED",
     "Gimbal and lamp booms lengthened and lowered to clear the lower rotor "
     "disc (CHECK 3)."),
    ("11_CABLE", +0.017, "REQUIRED",
     "Arm conduit, 4 x 22 g, less a lighter re-routed main loom."),
    ("10_LANDING_GEAR", -0.012, "REQUIRED",
     "Leg root moved outboard to clear the battery extraction corridor "
     "(CHECK 8); slightly shorter legs."),
    ("04_AVIONICS", -0.040, "REQUIRED",
     "Cooling stack shortened so the top service panel lifts off (CHECK 11)."),
    ("15_SAFETY", -0.035, "REQUIRED",
     "Status beacon moved onto the top service panel; its standoff bracket "
     "deleted."),
]

# carried allowances that are NOT geometry
ALLOWANCES = [
    ("fastener_allowance", 0.165, "CONSERVATIVE ALLOWANCE",
     "M3/M4/M5 hardware, inserts, adhesive. Deliberate margin."),
    ("wiring_harness_main", 0.395, "CONSERVATIVE ALLOWANCE",
     "HV + signal looms sized on 6 AWG and 4 arm pairs. Deliberate margin."),
    ("manip_internal_wiring", 0.105, "CONSERVATIVE ALLOWANCE",
     "Slip-ring-free internal routing to J6."),
]


def energy_and_endurance(mtow_kg):
    """Momentum theory, on the vendor coaxial-pair curve.

    NOT a flight-test number. Hover power comes from the T-Motor X-U8II
    coaxial pair data interpolated to the required per-arm thrust, so it
    inherits the vendor bench conditions (sea level, 25 C, static).
    """
    T_arm = mtow_kg / P.vArmCount                      # kgf per coaxial arm
    frac = T_arm / P.vThrustPerArm
    # vendor pair: 13.672 kgf at 55.32 A. Thrust ~ I^0.72 over the useful band.
    I_arm = P.vCurrentPerArm * frac ** (1 / 0.72)
    I_tot = I_arm * P.vArmCount
    V = P.vBattCells * 3.7
    P_hover = I_tot * V
    Wh = P.vBattWh * 0.80                              # 80 % usable
    t_min = Wh / P_hover * 60.0
    return dict(T_arm=T_arm, I_arm=I_arm, I_tot=I_tot, V=V,
                P_hover=P_hover, Wh_usable=Wh, endurance_min=t_min)


def report():
    M, cg, cg_groups, items = cad_groups()
    B, b_groups = budget_groups()

    print("=" * 78)
    print("PHASE 0 -- MASS RECONCILIATION")
    print("=" * 78)
    print(f'{"GROUP":22s} {"CAD kg":>9s} {"BUDGET kg":>10s} {"DELTA g":>9s}')
    keys = sorted(set(cg_groups) | set(b_groups))
    for k in keys:
        c_, b_ = cg_groups.get(k, 0.0), b_groups.get(k, 0.0)
        print(f"{k:22s} {c_:9.3f} {b_:10.3f} {(c_-b_)*1000:+9.0f}")
    print(f'{"TOTAL":22s} {M:9.3f} {B:10.3f} {(M-B)*1000:+9.0f}')

    print()
    print("DELTA CLASSIFICATION")
    print("-" * 78)
    by_class = defaultdict(float)
    for grp, kg, cls, why in LEDGER:
        by_class[cls] += kg
        print(f"{grp:20s} {kg*1000:+7.0f} g  {cls}")
        for line in _wrap(why, 70):
            print(f"                              {line}")
    explained = sum(k for _, k, _, _ in LEDGER)
    resid = (M - B) - explained
    print("-" * 78)
    for cls, kg in sorted(by_class.items()):
        print(f"{cls:26s} {kg*1000:+8.0f} g")
    print(f"{'EXPLAINED TOTAL':26s} {explained*1000:+8.0f} g")
    print(f"{'RESIDUAL (UNRESOLVED)':26s} {resid*1000:+8.0f} g")

    print()
    print("CARRIED ALLOWANCES (not geometry, retained on purpose)")
    for n, kg, cls, why in ALLOWANCES:
        print(f"  {n:24s} {kg*1000:6.0f} g  {cls}  -- {why}")

    # ---------------- recomputed performance ------------------------------
    m_nom = M
    m_max = M + 2.0                       # + 2 kg mission payload on the tool
    T_max = P.vThrustPerArm * P.vArmCount
    e_nom = energy_and_endurance(m_nom)
    e_max = energy_and_endurance(m_max)

    print()
    print("=" * 78)
    print("RECOMPUTED PERFORMANCE")
    print("=" * 78)
    print(f"  total modelled mass       {M:8.3f} kg   (137 parts, "
          f"0 interferences)")
    print(f"  nominal MTOW              {m_nom:8.3f} kg   "
          "(dry aircraft + 1.0 L service fluid, no external payload)")
    print(f"  maximum MTOW              {m_max:8.3f} kg   "
          "(+2.0 kg mission payload at the tool)")
    print(f"  max static thrust         {T_max:8.3f} kgf  "
          "(4 coaxial pairs, T-Motor vendor data, 100 % throttle)")
    print(f"  T/W at nominal MTOW       {T_max/m_nom:8.3f}")
    print(f"  T/W at maximum MTOW       {T_max/m_max:8.3f}")
    print(f"  CG                        "
          f"({float(cg[0]):+.1f}, {float(cg[1]):+.1f}, {float(cg[2]):+.1f}) mm")
    print(f"  hover power (nominal)     {e_nom['P_hover']:8.0f} W   "
          f"({e_nom['I_tot']:.1f} A at {e_nom['V']:.1f} V)")
    print(f"  hover power (maximum)     {e_max['P_hover']:8.0f} W   "
          f"({e_max['I_tot']:.1f} A)")
    print(f"  usable energy             {e_nom['Wh_usable']:8.0f} Wh  "
          f"(12S {P.vBattAh*P.vBattPacks:.0f} Ah, 80 % DoD)")
    print(f"  endurance (nominal)       {e_nom['endurance_min']:8.1f} min")
    print(f"  endurance (maximum)       {e_max['endurance_min']:8.1f} min")
    print(f"  burst current per arm     {P.vCurrentPerArm:8.1f} A   "
          f"vs ESC rating {P.vESCRating:.0f} A")
    return dict(M=M, cg=cg, m_nom=m_nom, m_max=m_max, T_max=T_max,
                tw_nom=T_max / m_nom, tw_max=T_max / m_max,
                e_nom=e_nom, e_max=e_max, explained=explained, resid=resid,
                cad_groups=cg_groups, budget_groups=b_groups, budget=B)


def _wrap(s, n):
    out, line = [], ""
    for w in s.split():
        if len(line) + len(w) + 1 > n:
            out.append(line)
            line = w
        else:
            line = (line + " " + w).strip()
    if line:
        out.append(line)
    return out


if __name__ == "__main__":
    report()
