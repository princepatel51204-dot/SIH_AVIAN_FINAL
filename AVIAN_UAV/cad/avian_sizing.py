"""
AVIAN -- Phase 1/2 sizing, propulsion trade study and battery architecture study.

NOTHING HERE IS VERIFIED PERFORMANCE.
Every output is tagged:
    CALCULATED : follows deterministically from the stated equations + inputs
    ESTIMATE   : an input drawn from engineering judgement / class data
    ASSUMED    : a modelling coefficient with a stated literature range
Verified values require bench thrust-stand and flight test data.

Physics
-------
Momentum theory, hover:
    P_ideal = T^1.5 / sqrt(2 * rho * A)
    P_shaft = P_ideal / FM
    P_elec  = P_shaft / eta_drive
Static thrust available at a given electrical power:
    T = ( P_elec * eta_drive * FM * sqrt(2 * rho * A) ) ^ (2/3)

Coaxial pairs are modelled as ONE disc of area pi*R^2 per arm, with an
interference penalty applied to the figure of merit and to installed thrust.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict

RHO = 1.225            # kg/m3, ISA sea level                        ASSUMED
G = 9.80665

# --- aerodynamic / drive coefficients ------------------------------------
FM_ISOLATED = 0.72     # figure of merit, 14-22" CF prop              ASSUMED (0.65-0.78)
K_COAX_FM = 0.82       # coaxial FM penalty                           ASSUMED (0.78-0.88)
K_COAX_T = 0.85        # coaxial installed-thrust penalty             ASSUMED (0.80-0.90)
ETA_DRIVE = 0.82       # motor + ESC electrical->shaft                ASSUMED (0.78-0.86)
DOD = 0.80             # usable depth of discharge, LiPo              ASSUMED (conservative)
TIP_GAP_FRAC = 0.12    # min tip-to-tip gap as a fraction of D        ASSUMED

# --- fixed equipment mass (kg) -------------------------------------------
# All ESTIMATE / PLACEHOLDER -- REQUIRES VALIDATION against vendor data.
M_AVIONICS = {
    "flight_controller": 0.100,
    "companion_computer": 0.300,
    "power_distribution": 0.150,
    "telemetry_and_rc": 0.080,
    "wiring_harness": 0.450,
}
M_SENSORS = {
    "lidar": 0.300,
    "rgb_camera_gimbal": 0.250,
    "depth_camera": 0.075,
    "gnss_rtk_dual": 0.120,
    "range_finder_flow": 0.045,
}
M_MANIPULATOR = {
    # sized from joint torque, see joint_torque_budget()
    "actuators_j1_j6": 1.180,
    "links_and_housings": 0.650,
    "internal_wiring": 0.120,
}
M_TOOL_INTERFACE = 0.180        # changer master + compact 6-axis F/T
M_FLUID_DRY = {
    "reservoir_empty": 0.220,
    "pump": 0.300,
    "filter_and_check_valve": 0.090,
    "tubing_and_fittings": 0.100,
    "mounting_bracket": 0.150,
}
MARGIN = 0.08                   # design margin on dry mass            ASSUMED


# --- battery options ------------------------------------------------------
@dataclass
class Battery:
    key: str
    label: str
    cells_s: int
    ah: float
    packs: int
    wh_per_kg: float            # pack level, incl. case/leads   ESTIMATE
    note: str = ""

    @property
    def v_nom(self):
        return self.cells_s * 3.7

    @property
    def v_max(self):
        return self.cells_s * 4.2

    @property
    def wh(self):
        return self.v_nom * self.ah * self.packs

    @property
    def mass(self):
        return self.wh / self.wh_per_kg

    @property
    def usable_wh(self):
        return self.wh * DOD


BATTERIES = [
    Battery("A", "1 x 6S 16.0 Ah LiPo (as specified)", 6, 16.0, 1, 145.0,
            "the brief's baseline pack"),
    Battery("B", "1 x 6S 22.0 Ah LiPo", 6, 22.0, 1, 150.0,
            "largest single 6S LiPo that still hand-fits a cartridge"),
    Battery("C", "2 x 6S 16.0 Ah LiPo in PARALLEL (6S 32 Ah)", 6, 16.0, 2, 145.0,
            "keeps 6S; doubles energy; halves per-pack C-rate; dual-pack redundancy"),
    Battery("D", "2 x 6S 16.0 Ah LiPo in SERIES (12S 16 Ah)", 12, 16.0, 1, 145.0,
            "same energy and mass as C, but HALF the current"),
    Battery("E", "1 x 12S 16.0 Ah Li-ion (21700, 12S6P class)", 12, 16.0, 1, 200.0,
            "higher energy density, lower C-rate capability"),
]
# note: option D energy = 12S x 16 Ah = same Wh as C (two 6S 16 Ah packs)
BATTERIES[3].packs = 1
BATTERIES[3].ah = 32.0 / 2.0 * 1.0   # 12S 16 Ah  == 2 x 6S 16 Ah in series


# --- geometry -------------------------------------------------------------
def max_prop_diameter(arch: str, diagonal_mm: float) -> float:
    """Largest propeller that fits with a TIP_GAP_FRAC tip-to-tip gap.

    diagonal = motor-to-motor across the aircraft = 2 * R_motor.
    flat-8 : 8 arms at 45 deg, adjacent spacing = 2*R*sin(22.5)
    coax X8: 4 arms at 90 deg, adjacent spacing = 2*R*sin(45)
    """
    R = diagonal_mm / 2.0
    n_arms = 8 if arch == "flat8" else 4
    spacing = 2 * R * math.sin(math.pi / n_arms)
    return spacing / (1.0 + TIP_GAP_FRAC)


def disc_area(arch: str, prop_d_mm: float) -> float:
    """Total effective rotor disc area (m2). Coaxial pairs share one disc."""
    n_disc = 8 if arch == "flat8" else 4
    r = prop_d_mm / 2000.0
    return n_disc * math.pi * r * r


def forward_rotor_extent(arch: str, diagonal_mm: float, prop_d_mm: float) -> float:
    """Largest +X coordinate reached by any rotor disc, arms in X layout
    (no arm on the forward centreline)."""
    R = diagonal_mm / 2.0
    n_arms = 8 if arch == "flat8" else 4
    half = math.pi / n_arms
    ang0 = half                              # first arm offset from +X
    best = -1e9
    for i in range(n_arms):
        a = ang0 + i * 2 * half
        best = max(best, R * math.cos(a) + prop_d_mm / 2.0)
    return best


def fm(arch):
    return FM_ISOLATED * (K_COAX_FM if arch == "coax_x8" else 1.0)


# --- mass model -----------------------------------------------------------
def airframe_mass(arch: str, diagonal_mm: float, mtow_guess: float) -> dict:
    """Bottom-up structural estimate. ESTIMATE -- REQUIRES VALIDATION.

    Arm tube sized on root bending: M = T_arm * L_arm, CF tube section modulus.
    """
    R = diagonal_mm / 2.0
    n_arms = 8 if arch == "flat8" else 4
    hub_r = 0.16 * diagonal_mm            # spine/hub outer radius
    L_arm = (R - hub_r) / 1000.0          # m, exposed tube length

    # per-arm thrust the tube must carry at max (T/W 2.2 assumed for structure)
    t_arm_N = 2.2 * mtow_guess * G / n_arms
    M_root = t_arm_N * L_arm              # Nm
    # choose a CF tube: sigma_allow 250 MPa (with SF 2.4 on 600 MPa UD-CF)
    sigma = 250e6
    Z_req = M_root / sigma                # m3
    # solve OD for a tube with wall = 0.08*OD
    od = max(0.018, (Z_req * 32 / (math.pi * (1 - 0.84 ** 4))) ** (1 / 3))
    od = math.ceil(od * 1000 / 2) * 2 / 1000.0        # round to 2 mm
    idm = 0.84 * od
    area = math.pi / 4 * (od ** 2 - idm ** 2)
    m_tube = area * L_arm * 1600.0        # CFRP 1600 kg/m3

    m_clamp = 0.055 + 0.00018 * (od * 1000) ** 2 / 10.0     # AL root clamp
    m_mount = (0.075 if arch == "flat8" else 0.135)          # dual mount for coax
    m_arm_wiring = 0.020 + 0.010 * L_arm * 10

    per_arm = m_tube + m_clamp + m_mount + m_arm_wiring
    arms = per_arm * n_arms

    # central structure: spine + decks + hub + panels, scales with span & MTOW
    spine = 0.55 * (diagonal_mm / 900.0) ** 1.15 * (mtow_guess / 14.0) ** 0.45
    hub = 0.42 * (mtow_guess / 14.0) ** 0.55        # reinforced manipulator hub
    panels = 0.38 * (diagonal_mm / 900.0) ** 1.2
    cg_rail = 0.18
    gear = 0.52 * (mtow_guess / 14.0) ** 0.5 * (1.0 + 0.15)
    fasteners = 0.14

    return {
        "arm_tube_od_mm": round(od * 1000, 1),
        "arm_tube_id_mm": round(idm * 1000, 1),
        "arm_root_moment_Nm": round(M_root, 1),
        "per_arm_kg": round(per_arm, 3),
        "arms_total": arms,
        "central_spine": spine,
        "manipulator_hub": hub,
        "panels_covers": panels,
        "cg_rail": cg_rail,
        "landing_gear": gear,
        "fasteners": fasteners,
        "total": arms + spine + hub + panels + cg_rail + gear + fasteners,
    }


def choose_prop(arch, diagonal_mm, mtow_kg, dl_target=120.0):
    """Pick the propeller diameter.

    Sized to a target hover disc loading (endurance driven), then clamped to the
    largest diameter the airframe geometry actually allows.
    dl_target = 120 N/m2 is a normal industrial-multirotor value.   ASSUMED
    """
    n_disc = 8 if arch == "flat8" else 4
    A_want = mtow_kg * G / dl_target
    r_want = math.sqrt(A_want / (n_disc * math.pi))
    d_want = 2 * r_want * 1000.0
    d_max = max_prop_diameter(arch, diagonal_mm)
    d = min(d_want, d_max)
    # round to a real 1-inch propeller size
    d_in = math.floor(d / 25.4)
    return max(8.0, d_in) * 25.4, (d_want > d_max)


def propulsion_mass(arch, prop_d_mm, mtow_kg, twr_target=2.2):
    """Motor / ESC / propeller mass driven by REQUIRED THRUST, not by diameter.

    m_motor = 0.040 * T_max[kgf] + 0.10      fits MN5008 (0.20 kg / 2.8 kgf)
                                             and U11 II  (0.41 kg / 8.8 kgf)
    m_prop  = 2.88e-5 * d_in^2.58            fits 18" = 45 g, 22" = 75 g,
                                             27" = 128 g CF propellers
    All ESTIMATE -- REQUIRES VALIDATION against vendor data.
    """
    n_mot = 8
    k_coax = K_COAX_T if arch == "coax_x8" else 1.0
    t_per_motor_kgf = twr_target * mtow_kg / (n_mot * k_coax)
    d_in = prop_d_mm / 25.4
    m_motor = 0.040 * t_per_motor_kgf + 0.10
    m_prop = 2.88e-5 * d_in ** 2.58
    return {
        "prop_dia_in": round(d_in, 1),
        "thrust_per_motor_kgf": round(t_per_motor_kgf, 2),
        "motor_each": round(m_motor, 3),
        "prop_each": round(m_prop, 3),
        "total_motors_props": n_mot * (m_motor + m_prop),
    }


def esc_mass(a_rating):
    """m = 0.0012 * A + 0.020 -> 40 A = 68 g, 80 A = 116 g.   ESTIMATE"""
    return 0.0012 * a_rating + 0.020


def joint_torque_budget(reach_m=0.80, tool_payload_kg=1.0, arm_mass_kg=1.95,
                        dyn=1.5):
    """Static + dynamic joint torque at full horizontal extension."""
    m_dist = arm_mass_kg * 0.55           # mass outboard of the shoulder
    r_com = reach_m * 0.42
    j2 = (m_dist * G * r_com + tool_payload_kg * G * reach_m) * dyn
    j3 = (m_dist * 0.6 * G * r_com * 0.7 + tool_payload_kg * G * reach_m * 0.6) * dyn
    j1 = j2 * 0.45
    j4 = tool_payload_kg * G * 0.09 * dyn
    j5 = tool_payload_kg * G * 0.13 * dyn
    j6 = tool_payload_kg * G * 0.06 * dyn
    return {"J1": j1, "J2": j2, "J3": j3, "J4": j4, "J5": j5, "J6": j6}


# ---------------------------------------------------------------------------
# Battery current-capability limits -- the constraint that actually binds
# ---------------------------------------------------------------------------
C_BURST_LIPO = 15.0     # sustainable burst C-rate, high-quality LiPo   ASSUMED
C_BURST_LION = 8.0      # 21700 Li-ion high-power cell                  ASSUMED


def c_limit_for(batt: Battery) -> float:
    return C_BURST_LION if "Li-ion" in batt.label else C_BURST_LIPO


def max_mtow_for_battery(batt, arch, diagonal, twr=2.2, dl_target=120.0):
    """Heaviest aircraft the PACK can actually fly at the T/W target,
    limited by burst current, not by energy.   CALCULATED from the model."""
    c = c_limit_for(batt)
    i_max = c * batt.ah * batt.packs
    p_max = batt.v_nom * i_max * 0.92           # 8 % wiring + sag loss
    k = K_COAX_T if arch == "coax_x8" else 1.0
    lo, hi = 1.0, 60.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        d, _cl = choose_prop(arch, diagonal, mid, dl_target)
        A = disc_area(arch, d)
        T = (p_max * ETA_DRIVE * fm(arch) * math.sqrt(2 * RHO * A)) ** (2 / 3) * k
        if T / (mid * G) > twr:
            lo = mid
        else:
            hi = mid
    return lo


def required_ah(mtow, arch, diagonal, cells_s, twr=2.2, c_limit=15.0,
                dl_target=120.0):
    """Pack capacity needed purely to supply the burst current at the T/W
    target.  CALCULATED."""
    d, _ = choose_prop(arch, diagonal, mtow, dl_target)
    A = disc_area(arch, d)
    k = K_COAX_T if arch == "coax_x8" else 1.0
    T = twr * mtow * G / k
    p = T ** 1.5 / math.sqrt(2 * RHO * A) / fm(arch) / ETA_DRIVE
    i = p / (cells_s * 3.7) / 0.92
    return i / c_limit, i, p


@dataclass
class Case:
    name: str
    arm_deployed: bool
    fluid_l: float
    tool: str
    carried_kg: float


# MISSION PAYLOAD DEFINITION (2.0 kg budget, as briefed):
#     fluid 1.0 L (1.00 kg) + gripper tool (0.38 kg) + carried object (0.62 kg)
# The manipulator itself and the fluid-system hardware are aircraft EQUIPMENT,
# not payload.  Stated explicitly because the brief is ambiguous on this.
CASES = [
    Case("MIN     arm stowed, tank empty, no tool", False, 0.0, "none", 0.0),
    Case("NOMINAL arm stowed, 1.0 L, nozzle fitted", False, 1.0, "nozzle", 0.0),
    Case("MAX     arm deployed, 1.0 L, gripper + 0.62 kg", True, 1.0, "gripper", 0.62),
]

TOOL_MASS = {"none": 0.0, "nozzle": 0.240, "gripper": 0.380}
FLUID_DENSITY = 1.0             # kg/L, water-like service fluid


def build_mass(arch, diagonal, batt: Battery, case: Case, iters=60,
               twr_target=2.2, dl_target=120.0):
    """Converge MTOW: airframe, propulsion and ESC mass all depend on MTOW."""
    mtow = 12.0
    prop_d, clamped = choose_prop(arch, diagonal, mtow, dl_target)
    esc_a = 40.0
    for _ in range(iters):
        prop_d, clamped = choose_prop(arch, diagonal, mtow, dl_target)
        af = airframe_mass(arch, diagonal, mtow)
        pr = propulsion_mass(arch, prop_d, mtow, twr_target)
        # ESC sized from the current at the T/W target
        A_disc = disc_area(arch, prop_d)
        k = K_COAX_T if arch == "coax_x8" else 1.0
        T22 = twr_target * mtow * G / k
        p22 = T22 ** 1.5 / math.sqrt(2 * RHO * A_disc) / fm(arch) / ETA_DRIVE
        esc_a = max(30.0, 1.25 * p22 / batt.v_nom / 8)
        m_esc_tot = 8 * esc_mass(esc_a)
        dry = (af["total"] + pr["total_motors_props"] + m_esc_tot
               + sum(M_AVIONICS.values()) + sum(M_SENSORS.values())
               + sum(M_MANIPULATOR.values()) + M_TOOL_INTERFACE
               + sum(M_FLUID_DRY.values()))
        dry *= (1 + MARGIN)
        new = dry + batt.mass + case.fluid_l * FLUID_DENSITY \
            + TOOL_MASS[case.tool] + case.carried_kg
        if abs(new - mtow) < 1e-4:
            mtow = new
            break
        mtow = 0.6 * mtow + 0.4 * new
    pr["esc_each"] = round(esc_mass(esc_a), 3)
    pr["esc_rating_A"] = round(esc_a, 0)
    pr["total"] = pr["total_motors_props"] + 8 * esc_mass(esc_a)
    return mtow, af, pr, dry, prop_d, clamped


def hover(arch, prop_d, mtow):
    A = disc_area(arch, prop_d)
    T = mtow * G
    p_ideal = T ** 1.5 / math.sqrt(2 * RHO * A)
    p_shaft = p_ideal / fm(arch)
    p_elec = p_shaft / ETA_DRIVE
    return {"disc_area_m2": A, "disc_loading_Npm2": T / A,
            "p_ideal_W": p_ideal, "p_hover_W": p_elec,
            "power_loading_g_per_W": mtow * 1000.0 / p_elec}


def thrust_available(arch, prop_d, p_elec_max):
    """Static thrust for a given available electrical power."""
    A = disc_area(arch, prop_d)
    k = K_COAX_T if arch == "coax_x8" else 1.0
    T = (p_elec_max * ETA_DRIVE * fm(arch) * math.sqrt(2 * RHO * A)) ** (2 / 3)
    return T * k


def evaluate(arch, diagonal, batt: Battery, case: Case,
             twr_target=2.2, dl_target=120.0):
    mtow, af, pr, dry, prop_d, clamped = build_mass(
        arch, diagonal, batt, case, twr_target=twr_target, dl_target=dl_target)
    h = hover(arch, prop_d, mtow)
    endurance = batt.usable_wh / h["p_hover_W"] * 60.0

    A_disc = disc_area(arch, prop_d)
    k = K_COAX_T if arch == "coax_x8" else 1.0

    def p_for_twr(t):
        T = t * mtow * G / k
        return T ** 1.5 / math.sqrt(2 * RHO * A_disc) / fm(arch) / ETA_DRIVE

    p22 = p_for_twr(twr_target)
    return {
        "arch": arch, "diagonal_mm": diagonal, "prop_mm": round(prop_d, 1),
        "prop_in": round(prop_d / 25.4, 1),
        "prop_geometry_limited": clamped,
        "battery": batt.key, "case": case.name,
        "mtow_kg": round(mtow, 2), "dry_kg": round(dry, 2),
        "airframe_kg": round(af["total"], 2),
        "propulsion_kg": round(pr["total"], 2),
        "battery_kg": round(batt.mass, 2),
        "motor_each_kg": pr["motor_each"],
        "thrust_per_motor_kgf": pr["thrust_per_motor_kgf"],
        "esc_rating_A": pr["esc_rating_A"],
        "arm_tube_od_mm": af["arm_tube_od_mm"],
        "arm_root_moment_Nm": af["arm_root_moment_Nm"],
        "disc_area_m2": round(A_disc, 3),
        "disc_loading_Npm2": round(h["disc_loading_Npm2"], 1),
        "p_hover_W": round(h["p_hover_W"], 0),
        "power_loading_g_per_W": round(h["power_loading_g_per_W"], 2),
        "hover_current_A": round(h["p_hover_W"] / batt.v_nom, 1),
        "hover_C_rate": round(h["p_hover_W"] / batt.v_nom / (batt.ah * batt.packs), 2),
        "endurance_min": round(endurance, 1),
        "p_for_twr_W": round(p22, 0),
        "current_twr_A": round(p22 / batt.v_nom, 0),
        "C_rate_twr": round(p22 / batt.v_nom / (batt.ah * batt.packs), 1),
        "A_per_motor_twr": round(p22 / batt.v_nom / 8, 1),
        "fwd_rotor_extent_mm": round(forward_rotor_extent(arch, diagonal, prop_d), 0),
    }


def main():
    out = {"coefficients": {
        "rho": RHO, "FM_isolated": FM_ISOLATED, "K_coax_FM": K_COAX_FM,
        "K_coax_thrust": K_COAX_T, "eta_drive": ETA_DRIVE, "DoD": DOD,
        "tip_gap_fraction": TIP_GAP_FRAC, "design_margin": MARGIN}}

    # ---- 1. geometry: what prop fits at each diagonal --------------------
    geo = []
    for dia in (700, 800, 900, 1000, 1100, 1200):
        row = {"diagonal_mm": dia}
        for arch in ("coax_x8", "flat8"):
            d = max_prop_diameter(arch, dia)
            row[f"{arch}_prop_mm"] = round(d, 0)
            row[f"{arch}_prop_in"] = round(d / 25.4, 1)
            row[f"{arch}_disc_m2"] = round(disc_area(arch, d), 3)
            row[f"{arch}_fwd_extent_mm"] = round(
                forward_rotor_extent(arch, dia, d), 0)
        geo.append(row)
    out["geometry_sweep"] = geo

    # ---- 2. architecture trade at the preferred 900 mm ------------------
    nominal = CASES[1]
    battA = BATTERIES[0]
    arch_trade = [evaluate(a, 900, battA, nominal) for a in ("coax_x8", "flat8")]
    out["architecture_trade_900mm"] = arch_trade

    # ---- 3. diagonal sweep, both architectures --------------------------
    out["diagonal_sweep"] = [
        evaluate(a, d, battA, nominal)
        for a in ("coax_x8", "flat8") for d in (700, 800, 900, 1000, 1100, 1200)]

    # ---- 4. battery study, coaxial X8 @ 950 mm --------------------------
    out["battery_study"] = [evaluate("coax_x8", 950, b, nominal) for b in BATTERIES]

    # ---- 5. mission sensitivity for the recommended config --------------
    out["mission_sensitivity"] = [
        evaluate("coax_x8", 950, b, c)
        for b in (BATTERIES[0], BATTERIES[2], BATTERIES[3]) for c in CASES]

    # ---- 6. joint torque budget -----------------------------------------
    out["joint_torque_Nm"] = {k: round(v, 2)
                              for k, v in joint_torque_budget().items()}

    # ---- 7. mass breakdown for the recommendation ------------------------
    b = BATTERIES[3]
    mtow, af, pr, dry, pd, cl = build_mass("coax_x8", 950, b, CASES[1])
    out["recommended_mass_breakdown"] = {
        "airframe": {k: (round(v, 3) if isinstance(v, float) else v)
                     for k, v in af.items()},
        "propulsion": pr,
        "avionics": M_AVIONICS, "sensors": M_SENSORS,
        "manipulator": M_MANIPULATOR,
        "tool_interface": M_TOOL_INTERFACE,
        "fluid_dry": M_FLUID_DRY,
        "battery_kg": round(b.mass, 3),
        "margin_pct": MARGIN * 100,
        "mtow_kg": round(mtow, 2),
    }
    # ---- 8. battery current-capability limit ----------------------------
    lim = []
    for b in BATTERIES:
        for arch in ("coax_x8",):
            for twr in (2.0, 2.2):
                lim.append({
                    "battery": b.key, "label": b.label, "arch": arch,
                    "twr_target": twr,
                    "c_limit": c_limit_for(b),
                    "burst_current_A": round(c_limit_for(b) * b.ah * b.packs, 0),
                    "max_mtow_kg": round(
                        max_mtow_for_battery(b, arch, 950, twr), 2),
                })
    out["battery_current_limit"] = lim

    # ---- 9. capacity required for the briefed mission --------------------
    req = []
    for mt in (14.0, 16.0, 18.0, 20.0):
        for s_cells in (6, 12):
            ah, i, p = required_ah(mt, "coax_x8", 950, s_cells, 2.2, 15.0)
            req.append({"mtow_kg": mt, "cells_s": s_cells,
                        "p_at_twr22_W": round(p, 0),
                        "current_A": round(i, 0),
                        "ah_needed_at_15C": round(ah, 1),
                        "wh_needed": round(ah * s_cells * 3.7, 0)})
    out["required_capacity"] = req

    with open("avian_sizing.json", "w") as f:
        json.dump(out, f, indent=2)
    return out


if __name__ == "__main__":
    r = main()

    print("=" * 78)
    print("1. GEOMETRY  --  largest propeller that fits at each diagonal")
    print("=" * 78)
    print(f"{'diag':>6} | {'COAX X8 prop':>22} {'disc m2':>8} {'fwd ext':>8} |"
          f" {'FLAT-8 prop':>20} {'disc m2':>8} {'fwd ext':>8}")
    for g in r["geometry_sweep"]:
        print(f"{g['diagonal_mm']:>6} | {g['coax_x8_prop_mm']:>10.0f} mm "
              f"({g['coax_x8_prop_in']:>4.1f}\") {g['coax_x8_disc_m2']:>8.3f} "
              f"{g['coax_x8_fwd_extent_mm']:>8.0f} |"
              f" {g['flat8_prop_mm']:>8.0f} mm ({g['flat8_prop_in']:>4.1f}\")"
              f" {g['flat8_disc_m2']:>8.3f} {g['flat8_fwd_extent_mm']:>8.0f}")

    print()
    print("=" * 78)
    print("2. ARCHITECTURE TRADE @ 900 mm diagonal, 6S 16 Ah, nominal mission")
    print("=" * 78)
    k = ["arch", "prop_in", "disc_area_m2", "disc_loading_Npm2", "mtow_kg",
         "propulsion_kg", "p_hover_W", "power_loading_g_per_W", "endurance_min",
         "hover_current_A", "A_per_motor_twr", "fwd_rotor_extent_mm"]
    for key in k:
        a, b = r["architecture_trade_900mm"]
        print(f"  {key:<24} {str(a[key]):>16}   {str(b[key]):>16}")

    print()
    print("=" * 78)
    print("3. BATTERY STUDY  --  coaxial X8 @ 950 mm, nominal mission")
    print("=" * 78)
    print(f"{'opt':>3} {'Wh':>6} {'kg':>6} {'MTOW':>6} {'P_hov':>7} {'A_hov':>7}"
          f" {'C_hov':>6} {'end':>6} {'A/mot@2.2':>10} {'C@2.2':>7}")
    for e, bt in zip(r["battery_study"], BATTERIES):
        print(f"{e['battery']:>3} {bt.wh:>6.0f} {e['battery_kg']:>6.2f} "
              f"{e['mtow_kg']:>6.2f} {e['p_hover_W']:>7.0f} "
              f"{e['hover_current_A']:>7.1f} {e['hover_C_rate']:>6.2f} "
              f"{e['endurance_min']:>6.1f} {e['A_per_motor_twr']:>10.1f} "
              f"{e['C_rate_twr']:>7.1f}")

    print()
    print("=" * 78)
    print("4. MISSION SENSITIVITY  --  coaxial X8 @ 950 mm")
    print("=" * 78)
    for e in r["mission_sensitivity"]:
        print(f"  batt {e['battery']}  {e['case'][:40]:<42} "
              f"MTOW {e['mtow_kg']:>5.2f} kg   "
              f"P {e['p_hover_W']:>5.0f} W   end {e['endurance_min']:>4.1f} min")

    print()
    print("=" * 78)
    print("5. BATTERY CURRENT LIMIT  --  heaviest aircraft each pack can fly")
    print("=" * 78)
    print(f"{'opt':>3} {'burst A':>9} {'T/W':>5} {'max MTOW kg':>12}   pack")
    for e in r["battery_current_limit"]:
        print(f"{e['battery']:>3} {e['burst_current_A']:>9.0f} {e['twr_target']:>5.1f}"
              f" {e['max_mtow_kg']:>12.2f}   {e['label'][:44]}")

    print()
    print("=" * 78)
    print("6. CAPACITY REQUIRED FOR THE BRIEFED MISSION (T/W 2.2, 15C limit)")
    print("=" * 78)
    print(f"{'MTOW':>6} {'S':>4} {'P@2.2 W':>9} {'A':>7} {'Ah needed':>11} {'Wh':>7}")
    for e in r["required_capacity"]:
        print(f"{e['mtow_kg']:>6.1f} {e['cells_s']:>4} {e['p_at_twr22_W']:>9.0f}"
              f" {e['current_A']:>7.0f} {e['ah_needed_at_15C']:>11.1f}"
              f" {e['wh_needed']:>7.0f}")

    print()
    print("7. JOINT TORQUE BUDGET (0.80 m reach, 1.0 kg tool payload, 1.5x dynamic)")
    for j, t in r["joint_torque_Nm"].items():
        print(f"   {j}  {t:>6.2f} Nm")
    print()
    print("MTOW (recommended config):",
          r["recommended_mass_breakdown"]["mtow_kg"], "kg")
