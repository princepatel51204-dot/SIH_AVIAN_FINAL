"""AVIAN REV A -- BOM, mass budget and the engineering tables.

The BOM is generated from the same assembly the checks and renders use, so
line-item masses and the mass budget cannot disagree. Vendor identity is
attached only where a real part has been selected; everything else is tagged
PLACEHOLDER and carries the fabrication warning.
"""
from __future__ import annotations
import csv
import math
import os
from collections import defaultdict

import numpy as np
import params_b as P
import kin_b as K
import assy_b
import reconcile as RC

OUT = "pkg/05_BOM"

# name-fragment -> (manufacturer, model, part number, source, confidence,
#                   manufacturing method)
VENDOR = [
    ("motor_",      ("T-Motor", "U8 II KV100", "TM-U8II-100", "VENDOR",
                     "VENDOR", "purchased")),
    ("propeller_",  ("T-Motor", "G28x9.2 CF folding", "TM-G28X92",
                     "VENDOR", "VENDOR", "purchased")),
    ("esc_",        ("T-Motor", "ALPHA 60A 12S V1.2", "TM-ALPHA-60A-12S",
                     "VENDOR", "VENDOR", "purchased")),
    ("battery_cartridge", ("TBD", "12S 12 Ah Li-ion cartridge", "TBD",
                           "TBD", "PLACEHOLDER", "purchased")),
    ("flight_controller", ("TBD", "Pixhawk-class autopilot", "TBD", "TBD",
                           "PLACEHOLDER", "purchased")),
    ("companion_computer", ("TBD", "Jetson Orin NX class", "TBD", "TBD",
                            "PLACEHOLDER", "purchased")),
    ("lidar",       ("TBD", "3D scanning LiDAR", "TBD", "TBD", "PLACEHOLDER",
                     "purchased")),
    ("gnss_antenna", ("TBD", "RTK GNSS antenna", "TBD", "TBD", "PLACEHOLDER",
                      "purchased")),
    ("rgb_gimbal",  ("TBD", "3-axis RGB gimbal", "TBD", "TBD", "PLACEHOLDER",
                     "purchased")),
    ("depth_camera", ("TBD", "stereo depth camera", "TBD", "TBD",
                      "PLACEHOLDER", "purchased")),
    ("pump",        ("TBD", "diaphragm pump 6 bar 1.2 L/min", "TBD", "TBD",
                     "PLACEHOLDER", "purchased")),
    ("filter_40um", ("TBD", "40 um inline filter", "TBD", "TBD",
                     "PLACEHOLDER", "purchased")),
    ("actuator_",   ("TBD", "integrated joint actuator", "TBD", "TBD",
                     "PLACEHOLDER", "purchased")),
    ("ft_sensor",   ("TBD", "6-axis force/torque sensor", "TBD", "TBD",
                     "PLACEHOLDER", "purchased")),
    ("gear_damper", ("TBD", "wire-rope isolator", "TBD", "TBD", "PLACEHOLDER",
                     "purchased")),
]

MFG = [
    ("CFRP",   "CF tube / plate, cut and bonded"),
    ("AL7075", "CNC machined AL7075-T6"),
    ("AL6061", "CNC machined AL6061-T6"),
    ("POLYMER", "FDM printed PA-CF"),
    ("TPU",    "extruded / routed"),
    ("GENERIC", "purchased"),
    ("FLUID",  "consumable"),
]


def vendor_for(name, material):
    for frag, row in VENDOR:
        if frag in name:
            return row
    for m, method in MFG:
        if material == m:
            conf = "CALCULATED" if m in ("CFRP", "AL7075", "AL6061") \
                else "ESTIMATED"
            return ("in-house", "-", "-", "AVIAN CAD", conf, method)
    return ("in-house", "-", "-", "AVIAN CAD", "ESTIMATED", "TBD")


def build_bom():
    reg = assy_b.build("01_FLIGHT")
    M, cg, items = assy_b.mass_cg(reg)
    meta = {p.name: p for p in reg.parts}

    # collapse identical parts into quantity rows
    groups = defaultdict(lambda: dict(qty=0, mass=0.0))
    for n, gr, m, c in items:
        base = "".join(ch for ch in n if not ch.isdigit()).strip("_")
        base = base.replace("__", "_")
        key = (gr, base)
        groups[key]["qty"] += 1
        groups[key]["mass"] += m
        groups[key].setdefault("sample", n)
        groups[key].setdefault("note", "")
        p = meta.get(n)
        if p is not None:
            groups[key]["material"] = p.material
            groups[key]["note"] = p.note or ""

    rows = []
    for i, ((gr, base), d) in enumerate(sorted(groups.items()), start=1):
        mat = d.get("material", "GENERIC")
        mfr, model, pn, src, conf, method = vendor_for(base, mat)
        note = d.get("note", "")
        if conf == "PLACEHOLDER":
            note = ("PLACEHOLDER COMPONENT -- VERIFY BEFORE FABRICATION. "
                    + note).strip()
        rows.append(dict(
            item_id=f"AV-{gr[:2]}-{i:03d}",
            subsystem=gr,
            part_name=base,
            qty=d["qty"],
            material=mat,
            mass_kg=round(d["mass"], 4),
            unit_mass_kg=round(d["mass"] / d["qty"], 4),
            manufacturer=mfr,
            model=model,
            part_number=pn,
            source=src,
            confidence=conf,
            manufacturing=method,
            notes=note,
        ))
    return rows, M, cg, reg


COLS = ["item_id", "subsystem", "part_name", "qty", "material",
        "unit_mass_kg", "mass_kg", "manufacturer", "model", "part_number",
        "source", "confidence", "manufacturing", "notes"]
HDR = ["Item ID", "Subsystem", "Part name", "Quantity", "Material",
       "Unit mass (kg)", "Total mass (kg)", "Manufacturer", "Model",
       "Part Number", "Source", "Confidence", "Manufacturing method", "Notes"]


def write_bom(rows):
    os.makedirs(OUT, exist_ok=True)
    with open(f"{OUT}/AVIAN_BOM.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(HDR)
        for r in rows:
            w.writerow([r[c] for c in COLS])
    return sum(r["mass_kg"] for r in rows)


def write_mass_budget(rows, M):
    g = defaultdict(float)
    for r in rows:
        g[r["subsystem"]] += r["mass_kg"]
    B, bg = RC.budget_groups()
    with open(f"{OUT}/AVIAN_MASS_BUDGET.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Subsystem", "CAD mass (kg)", "Phase 2.7 budget (kg)",
                    "Delta (g)", "Share of MTOW (%)"])
        for k in sorted(set(g) | set(bg)):
            c_, b_ = g.get(k, 0.0), bg.get(k, 0.0)
            w.writerow([k, round(c_, 4), round(b_, 4),
                        round((c_ - b_) * 1000, 1), round(c_ / M * 100, 2)])
        w.writerow(["TOTAL", round(M, 4), round(B, 4),
                    round((M - B) * 1000, 1), 100.0])
    return g


def write_tables(rows, M, cg, reg):
    d = "pkg/07_Engineering"
    os.makedirs(d, exist_ok=True)
    e = RC.energy_and_endurance(M)
    e2 = RC.energy_and_endurance(M + 2.0)
    T = P.vThrustPerArm * P.vArmCount

    # ---- C propulsion table ------------------------------------------------
    with open(f"{OUT}/TABLE_C_propulsion.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Quantity", "Value", "Unit", "Confidence", "Basis"])
        for r in [
            ("Architecture", "coaxial X8", "-", "CALCULATED",
             "4 arms x 2 rotors"),
            ("Motor", "T-Motor U8 II KV100", "-", "VENDOR", "data sheet"),
            ("ESC", "T-Motor ALPHA 60A 12S", "-", "VENDOR", "data sheet"),
            ("Propeller", "T-Motor G28x9.2 CF", "-", "VENDOR", "data sheet"),
            ("Rotor diameter", P.vPropDia, "mm", "VENDOR", "28 in"),
            ("Diagonal", P.vDiagonal, "mm", "CALCULATED", "geometry sweep"),
            ("Tip-to-tip gap", round(2 * P.vRMotor * math.sin(math.pi / 4)
                                     - P.vPropDia, 1), "mm", "VERIFIED",
             "measured on CAD"),
            ("Thrust per coaxial pair", P.vThrustPerArm, "kgf", "VENDOR",
             "100 % throttle, bench"),
            ("Total static thrust", round(T, 2), "kgf", "VENDOR",
             "4 pairs, bench"),
            ("T/W at nominal MTOW", round(T / M, 3), "-", "CALCULATED",
             "vendor thrust / CAD mass"),
            ("T/W at maximum MTOW", round(T / (M + 2.0), 3), "-",
             "CALCULATED", "with 2 kg payload"),
            ("Hover throttle fraction", round(M / T * 100, 1), "%",
             "CALCULATED", "static, sea level"),
            ("Current per pair at max", P.vCurrentPerArm, "A", "VENDOR",
             "44.4 V"),
            ("ESC headroom at max", round(P.vESCRating - P.vCurrentPerArm, 1),
             "A", "CALCULATED", "60 A rating"),
            ("Hover power, nominal", round(e["P_hover"]), "W", "CALCULATED",
             "vendor curve interpolation"),
            ("Hover power, maximum", round(e2["P_hover"]), "W", "CALCULATED",
             "with 2 kg payload"),
        ]:
            w.writerow(list(r))

    # ---- D joint torque ----------------------------------------------------
    with open(f"{OUT}/TABLE_D_joint_torque.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Joint", "Function", "Axis", "Lower limit (deg)",
                    "Upper limit (deg)", "Design torque (Nm)",
                    "Max speed (deg/s)", "Housing dia (mm)", "Confidence"])
        for i, (nm, off, ax, lim, tq, sp, desc) in enumerate(P.JOINTS):
            w.writerow([nm, desc, str(ax), lim[0], lim[1], tq, sp,
                        getattr(P, f"vJ{i+1}D"), "CALCULATED"])

    # ---- E battery energy --------------------------------------------------
    with open(f"{OUT}/TABLE_E_battery.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Quantity", "Value", "Unit", "Confidence"])
        for r in [("Configuration", "12S2P", "-", "CALCULATED"),
                  ("Nominal voltage", P.vBattCells * 3.7, "V", "CALCULATED"),
                  ("Capacity", P.vBattAh * P.vBattPacks, "Ah", "CALCULATED"),
                  ("Nameplate energy", round(P.vBattWh, 1), "Wh",
                   "CALCULATED"),
                  ("Usable energy at 80 % DoD", round(e["Wh_usable"], 1),
                   "Wh", "ASSUMED"),
                  ("Pack mass, each", P.vBattMassEach, "kg", "ESTIMATED"),
                  ("Hover current, nominal", round(e["I_tot"], 1), "A",
                   "CALCULATED"),
                  ("Burst current, all arms", round(P.vCurrentPerArm * 4, 1),
                   "A", "VENDOR")]:
            w.writerow(list(r))

    # ---- F endurance -------------------------------------------------------
    with open(f"{OUT}/TABLE_F_endurance.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Case", "Mass (kg)", "Hover power (W)",
                    "Endurance (min)", "Confidence", "Note"])
        w.writerow(["Nominal (1.0 L fluid, no payload)", round(M, 3),
                    round(e["P_hover"]), round(e["endurance_min"], 1),
                    "CALCULATED",
                    "momentum theory on the vendor coaxial pair curve; "
                    "static, sea level, no wind, no reserve"])
        w.writerow(["Maximum (+2.0 kg payload)", round(M + 2.0, 3),
                    round(e2["P_hover"]), round(e2["endurance_min"], 1),
                    "CALCULATED", "as above"])
        w.writerow(["Nominal with 20 % reserve", round(M, 3),
                    round(e["P_hover"]), round(e["endurance_min"] * 0.8, 1),
                    "CALCULATED", "operationally usable figure"])

    # ---- B CG table --------------------------------------------------------
    with open(f"{OUT}/TABLE_B_cg.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Case", "Config", "Mass (kg)", "CG x (mm)", "CG y (mm)",
                    "CG z (mm)", "Confidence"])
        for cfg in P.CFG:
            r = assy_b.build(cfg)
            m, c, _ = assy_b.mass_cg(r)
            w.writerow([cfg, cfg, round(m, 3), round(float(c[0]), 1),
                        round(float(c[1]), 1), round(float(c[2]), 1),
                        "VERIFIED"])
        for det, lab in ((-1, "battery aft detent"), (1, "battery fwd detent")):
            r = assy_b.build("01_FLIGHT", batt_index=det)
            m, c, _ = assy_b.mass_cg(r)
            w.writerow([lab, "01_FLIGHT", round(m, 3), round(float(c[0]), 1),
                        round(float(c[1]), 1), round(float(c[2]), 1),
                        "VERIFIED"])

    # ---- J configuration matrix -------------------------------------------
    with open(f"{OUT}/TABLE_J_configurations.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Configuration", "Arm state (J1..J6 deg)", "Tool",
                    "Battery", "Service module", "Sensor state",
                    "Access panels", "Mass (kg)"])
        DESC = {
            "01_FLIGHT": ("stowed under the belly", "2 packs, centre detent",
                          "1.0 L charged", "LiDAR + GNSS + depth active, "
                          "gimbal stowed", "all closed"),
            "02_INSPECTION": ("extended forward, camera standoff",
                              "2 packs, centre detent", "1.0 L charged",
                              "all sensors active", "all closed"),
            "03_MANIPULATION": ("working envelope", "2 packs, centre detent",
                                "1.0 L charged",
                                "depth + gimbal + F/T active", "all closed"),
            "04_REPAIR": ("contact pose, gripper fitted",
                          "2 packs, centre detent", "1.0 L charged",
                          "F/T force control active", "all closed"),
            "05_LIQUID": ("nozzle standoff pose", "2 packs, centre detent",
                          "1.0 L, pump armed",
                          "pressure sensor + depth active", "all closed"),
            "06_TRANSPORT": ("folded, minimum envelope", "packs removed",
                             "drained", "all off", "all closed"),
            "07_MAINTENANCE": ("forward and clear of the belly",
                               "packs removable", "cartridge removable",
                               "all off", "all four open"),
        }
        for cfg, q in P.CFG.items():
            r = assy_b.build(cfg)
            m, c, _ = assy_b.mass_cg(r)
            arm, batt, svc, sens, panels = DESC[cfg]
            w.writerow([cfg,
                        ", ".join(f"{v:.0f}" for v in q),
                        P.CFG_TOOL[cfg], batt, svc, sens, panels,
                        round(m, 3)])

    # ---- I serviceability matrix ------------------------------------------
    with open(f"{OUT}/TABLE_I_serviceability.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Item", "Access", "Direction", "Tools", "Prerequisites",
                    "Verified clearance"])
        for r in [
            ("Battery cartridge x2", "no panel", "aft (-X)",
             "none, 2 quarter-turn", "none", "corridor clear (VERIFIED)"),
            ("Service-fluid cartridge", "lower panel", "down (-Z)",
             "none, 2 quarter-turn + 2 pins", "lower panel off",
             "corridor clear (VERIFIED)"),
            ("Flight controller", "top panel", "up (+Z)", "4 x M3",
             "top panel off", "corridor clear (VERIFIED)"),
            ("Companion computer", "top panel", "up (+Z)", "4 x M3",
             "top panel off", "corridor clear (VERIFIED)"),
            ("Power distribution board", "top panel", "up (+Z)", "6 x M3",
             "top panel, FC tray, companion computer",
             "corridor clear (VERIFIED)"),
            ("Pump / filter / valves", "lower + side panel", "aft bay",
             "M3/M4", "lower panel off", "aft bay accessible"),
            ("Propulsion arm", "external", "outboard (+r)", "4 x M5 clamp",
             "propeller removed", "no interference (VERIFIED)"),
            ("Propeller", "external", "up/down", "1 x centre bolt", "none",
             "folding, no removal needed for transport"),
            ("Tool (gripper/nozzle)", "external", "along tool axis",
             "none, quick-change", "arm at tool-change pose",
             "880 mm presentation radius (VERIFIED)"),
            ("Nozzle tip", "external", "unscrew", "none", "none",
             "field-replaceable"),
            ("Landing gear foot", "external", "outboard", "1 x M4", "none",
             "replaceable wear item"),
        ]:
            w.writerow(list(r))
    return e, e2


def main():
    rows, M, cg, reg = build_bom()
    bom_total = write_bom(rows)
    g = write_mass_budget(rows, M)
    e, e2 = write_tables(rows, M, cg, reg)
    print(f"BOM rows          {len(rows)}")
    print(f"BOM mass total    {bom_total:.4f} kg")
    print(f"Assembly MTOW     {M:.4f} kg")
    # BOM masses are published to 4 dp (0.1 g), so the column total can differ
    # from the assembly by at most half a display unit per row. Anything
    # larger means a part is missing from the BOM or double-counted.
    tol = 0.5e-4 * len(rows)
    d = bom_total - M
    print(f"Difference        {d * 1000:+.2f} g  "
          f"(rounding bound {tol * 1000:.2f} g)")
    assert abs(d) < tol, (f"BOM total {bom_total:.6f} does not reconcile with "
                          f"the assembly {M:.6f}")
    print("BOM reconciles with the mass budget within display rounding.")
    return rows, M


if __name__ == "__main__":
    main()
