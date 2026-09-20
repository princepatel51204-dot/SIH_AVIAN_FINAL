"""
AVIAN -- PHASE 2.7 ENGINEERING CLOSURE
======================================
Bottom-up mass + CG budget, propulsion component closure on real vendor
hardware, thrust margin including failure cases, CG study across all
configurations, structural load cases, liquid system, manipulator torque
verification, and the final architecture decision.

CONFIDENCE TAGS
  VENDOR      published by the component manufacturer; not independently tested
  CALCULATED  follows deterministically from stated equations and inputs
  ESTIMATED   engineering judgement / class data, no specific source
  ASSUMED     a modelling coefficient, stated range given
  VERIFIED    measured by us  --  NOTHING in this file is VERIFIED

COORDINATE SYSTEM (base_link)
  origin  centre of the four arm-root axes, in the arm/rotor mid-plane
  X       forward (manipulator and inspection sensors face +X)
  Y       left
  Z       up
"""
from __future__ import annotations
import json, math
from dataclasses import dataclass, field

G = 9.80665
RHO = 1.225

# ===========================================================================
# 1. FROZEN GEOMETRY
# ===========================================================================
DIAGONAL = 1150.0                 # mm, motor-to-motor
R_MOT = DIAGONAL / 2.0            # 575
ARM_ANG = [45.0, 135.0, 225.0, 315.0]
PROP_D = 711.2                    # G28x9.2 -> 28 in
PROP_R = PROP_D / 2.0
COAX_SEP = 124.0                  # upper-to-lower rotor plane
Z_UP, Z_LO = +COAX_SEP / 2, -COAX_SEP / 2

SPACING = 2 * R_MOT * math.sin(math.radians(45.0))
TIP_GAP = SPACING - PROP_D
INNER_SWEEP = R_MOT - PROP_R
FWD_ROTOR = R_MOT * math.cos(math.radians(45.0)) + PROP_R

ARM_BASE = (45.0, 0.0, -95.0)     # arm_base_link on the manipulator hub
A_L0, A_L1, A_L2, A_L3, A_L4, A_L5 = 95.0, 330.0, 300.0, 85.0, 80.0, 60.0
A_TOOL = 45.0                     # tool0 flange -> tool point
REACH = A_L1 + A_L2 + A_L3 + A_L4 + A_L5 + A_TOOL       # 900 mm from J2

# ===========================================================================
# 2. PROPULSION COMPONENT CLOSURE  --  real hardware
# ===========================================================================
# Source: T-Motor "X-U8II Standard Integrated Propulsion Kit" product page,
# store.tmotor.com, retrieved 2026-08-26.  VENDOR-PUBLISHED, NOT VERIFIED BY US.
PROP_KIT = {
    "kit": "T-Motor X-U8II coaxial integrated propulsion arm set",
    "motor": "T-Motor U8 II KV100",
    "kv": 100,
    "cells": "12S LiPo (44.4 V nominal / 50.4 V full)",
    "esc": "T-Motor ALPHA 60A 12S V1.2",
    "esc_a": 60.0,
    "propeller": "T-Motor G28x9.2 CF, folding",
    "prop_d_in": 28.0,
    "prop_pitch_in": 9.2,
    "pair_thrust_g": 13672.0,      # VENDOR, 100 % throttle, coaxial pair
    "pair_current_a": 55.32,       # VENDOR, at that point
    "pair_eff_pct": 77.35,         # VENDOR (ESC efficiency)
    "arm_set_g": 955.0,            # VENDOR range 945-965 g incl. cable
    "note": "arm-set mass covers 2 motors + 2 ESCs + cabling + coaxial mount; "
            "propellers quoted separately",
}
T_PAIR_MAX = PROP_KIT["pair_thrust_g"] / 1000.0          # kgf per arm
T_TOTAL_MAX = 4 * T_PAIR_MAX                              # kgf
I_MAX_TOTAL = 4 * PROP_KIT["pair_current_a"]              # A at 100 % throttle
V_NOM = 44.4
P_MAX_TOTAL = I_MAX_TOTAL * V_NOM

M_MOTOR = 0.240      # VENDOR, U8 II
M_ESC = 0.082        # VENDOR, ALPHA 60A 12S
M_PROP = 0.130       # VENDOR, G28x9.2 CF
M_COAX_MOUNT = (PROP_KIT["arm_set_g"] / 1000.0) - 2 * M_MOTOR - 2 * M_ESC  # 0.311

# aerodynamic coefficients for HOVER (momentum theory; conservative)
FM_COAX = 0.72 * 0.82            # ASSUMED, 0.78-0.88 range on the coax factor
ETA_DRIVE = 0.82                 # ASSUMED
DOD = 0.80                       # ASSUMED
A_DISC = 4 * math.pi * (PROP_R / 1000.0) ** 2

# ===========================================================================
# 3. BATTERY CANDIDATES
# ===========================================================================
@dataclass
class Pack:
    label: str
    s: int
    ah: float
    n: int
    wh_kg: float
    c_burst: float = 15.0
    @property
    def v(self): return self.s * 3.7
    @property
    def wh(self): return self.v * self.ah * self.n
    @property
    def kg(self): return self.wh / self.wh_kg
    @property
    def usable(self): return self.wh * DOD
    @property
    def burst_a(self): return self.c_burst * self.ah * self.n

PACKS = [
    Pack("12S 16 Ah LiPo (1 pack)", 12, 16.0, 1, 148.0),
    Pack("12S 20 Ah LiPo (2 x 10 Ah parallel)", 12, 10.0, 2, 148.0),
    Pack("12S 24 Ah LiPo (2 x 12 Ah parallel)", 12, 12.0, 2, 148.0),
    Pack("12S 30 Ah LiPo (2 x 15 Ah parallel)", 12, 15.0, 2, 148.0),
]

# ===========================================================================
# 4. MANIPULATOR  --  kinematics, link masses, torque
# ===========================================================================
def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]
def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]
def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]
def mv(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]
def add(a, b): return [a[i] + b[i] for i in range(3)]

# joint: (name, offset in parent, axis in parent, limits, function)
JOINTS = [
    ("J1", (0, 0, 0),       (0, 0, 1),  (-180, 180), "base yaw"),
    ("J2", (0, 0, -A_L0),   (0, -1, 0), (-115, 115), "shoulder pitch"),
    ("J3", (0, 0, -A_L1),   (0, -1, 0), (-160, 160), "elbow pitch"),
    ("J4", (0, 0, -A_L2),   (0, 0, -1), (-180, 180), "wrist roll"),
    ("J5", (0, 0, -A_L3),   (0, -1, 0), (-120, 120), "wrist pitch"),
    ("J6", (0, 0, -A_L4),   (1, 0, 0),  (-180, 180), "wrist yaw"),
]
# link mass and local CG (fraction along the link from its joint origin)
LINK = [   # (actuator kg, structure kg, cg fraction, next-link length)
    (0.190, 0.135, 0.45, A_L0),   # base + J1
    (0.285, 0.165, 0.42, A_L1),   # link 1, upper arm
    (0.230, 0.130, 0.44, A_L2),   # link 2, forearm
    (0.095, 0.048, 0.45, A_L3),   # link 3
    (0.085, 0.042, 0.45, A_L4),   # link 4
    (0.070, 0.038, 0.45, A_L5),   # link 5 -> tool0
]
ARM_WIRING = 0.105
ARM_MASS = sum(a + s for a, s, _, _ in LINK) + ARM_WIRING

M_TOOLCHANGER = 0.130      # master + tool plate, mechanical + 12-way + 1 fluid
M_FTS = 0.080              # compact 6-axis force/torque
TOOLS = {"none": 0.0, "nozzle": 0.240, "gripper": 0.380}

def arm_frames(q):
    """Return (origin, rotation) of each joint frame and tool0, in base_link."""
    p = list(ARM_BASE); R = [[1,0,0],[0,1,0],[0,0,1]]
    out = [(list(p), [r[:] for r in R])]
    for i, (nm, off, ax, lim, fn) in enumerate(JOINTS):
        p = add(p, mv(R, list(off)))
        a = math.radians(q[i])
        if ax == (0, 0, 1):    Rj = Rz(a)
        elif ax == (0, 0, -1): Rj = Rz(-a)
        elif ax == (0, -1, 0): Rj = Ry(-a)
        elif ax == (0, 1, 0):  Rj = Ry(a)
        else:                  Rj = Rx(a)
        R = mul(R, Rj)
        out.append((list(p), [r[:] for r in R]))
    p_tool0 = add(out[-1][0], mv(out[-1][1], [0, 0, -A_L5]))
    out.append((p_tool0, [r[:] for r in out[-1][1]]))
    return out

def arm_items(q, tool="none", carried=0.0):
    """Mass items for the manipulator in configuration q. Returns list of
    (name, kg, (x,y,z))."""
    f = arm_frames(q)
    items = []
    for i, (act, struct, frac, L) in enumerate(LINK):
        origin, R = f[i + 1]
        cg = add(origin, mv(R, [0, 0, -frac * L]))
        items.append((f"manip_{JOINTS[i][0]}_actuator", act, tuple(cg)))
        items.append((f"manip_link_{i+1}_structure", struct, tuple(cg)))
    mid = f[3][0]
    items.append(("manip_internal_wiring", ARM_WIRING, tuple(mid)))
    t0, R0 = f[-1]
    fts = add(t0, mv(R0, [0, 0, -12]))
    tc = add(t0, mv(R0, [0, 0, -30]))
    tp = add(t0, mv(R0, [0, 0, -A_TOOL]))
    items.append(("tool_ft_sensor", M_FTS, tuple(fts)))
    items.append(("tool_changer", M_TOOLCHANGER, tuple(tc)))
    if TOOLS[tool] > 0:
        items.append((f"tool_{tool}", TOOLS[tool], tuple(tp)))
    if carried > 0:
        items.append(("carried_object", carried, tuple(tp)))
    return items, tp

# arm configurations (deg)
CFG = {
    "STOWED":      [0, 100, -168, 0, 68, 0],
    "DEPLOYED":    [0, 55, 25, 0, 10, 0],
    "MAX_REACH":   [0, 90, 0, 0, 0, 0],
    "DOWN_REACH":  [0, 0, 0, 0, 0, 0],
    "SIDE_REACH":  [90, 90, 0, 0, 0, 0],
    "TOOL_SERVICE":[180, 75, -120, 0, 45, 0],
    "TRANSPORT":   [0, 104, -172, 0, 68, 0],
}

def joint_torques(q, tool="gripper", carried=0.62, dyn=1.5):
    """Gravity torque at every joint for configuration q, x dynamic factor.
    CALCULATED: sum of (mass x g x horizontal lever) outboard of each joint,
    projected onto that joint's axis."""
    f = arm_frames(q)
    items, tp = arm_items(q, tool, carried)
    order = {f"manip_{JOINTS[i][0]}_actuator": i for i in range(6)}
    order.update({f"manip_link_{i+1}_structure": i for i in range(6)})
    res = {}
    for j in range(6):
        origin, R = f[j + 1]
        axis = mv(R, [0, 0, 1]) if JOINTS[j][2] == (0, 0, 1) else \
               mv(R, [0, 0, -1]) if JOINTS[j][2] == (0, 0, -1) else \
               mv(R, [0, -1, 0]) if JOINTS[j][2] == (0, -1, 0) else \
               mv(R, [1, 0, 0])
        tq = 0.0
        for nm, m, pos in items:
            idx = order.get(nm, 6)
            if nm in ("manip_internal_wiring",): idx = 3
            if nm.startswith(("tool_", "carried")): idx = 6
            if idx < j:
                continue
            r = [pos[k] / 1000.0 - origin[k] / 1000.0 for k in range(3)]
            Fg = [0, 0, -m * G]
            M = [r[1]*Fg[2]-r[2]*Fg[1], r[2]*Fg[0]-r[0]*Fg[2], r[0]*Fg[1]-r[1]*Fg[0]]
            tq += sum(M[k] * axis[k] for k in range(3))
        res[JOINTS[j][0]] = abs(tq) * dyn
    return res

# ===========================================================================
# 5. FULL MASS + CG BUDGET
# ===========================================================================
# (name, group, kg, (x,y,z) mm, source, confidence)
def base_items(batt: Pack, batt_x=0.0):
    it = []
    A = it.append
    for i, ang in enumerate(ARM_ANG):
        a = math.radians(ang)
        mx, my = R_MOT * math.cos(a), R_MOT * math.sin(a)
        n = i + 1
        A((f"motor_{n}_upper", "02_PROPULSION", M_MOTOR, (mx, my, Z_UP - 32),
           "T-Motor U8 II KV100", "VENDOR"))
        A((f"motor_{n}_lower", "02_PROPULSION", M_MOTOR, (mx, my, Z_LO + 32),
           "T-Motor U8 II KV100", "VENDOR"))
        A((f"propeller_{n}_upper", "02_PROPULSION", M_PROP, (mx, my, Z_UP),
           "T-Motor G28x9.2 CF", "VENDOR"))
        A((f"propeller_{n}_lower", "02_PROPULSION", M_PROP, (mx, my, Z_LO),
           "T-Motor G28x9.2 CF", "VENDOR"))
        A((f"coax_mount_{n}", "02_PROPULSION", M_COAX_MOUNT, (mx, my, 0),
           "X-U8II arm set less motors/ESCs", "VENDOR"))
        ex, ey = 320 * math.cos(a), 320 * math.sin(a)
        A((f"esc_{n}_upper", "02_PROPULSION", M_ESC, (ex, ey, 22),
           "T-Motor ALPHA 60A 12S", "VENDOR"))
        A((f"esc_{n}_lower", "02_PROPULSION", M_ESC, (ex, ey, -22),
           "T-Motor ALPHA 60A 12S", "VENDOR"))
        rc = (210 + R_MOT) / 2.0
        A((f"arm_tube_{n}", "01_AIRFRAME", 0.084,
           (rc * math.cos(a), rc * math.sin(a), 0),
           "CF 25/21 tube, root-moment sized", "CALCULATED"))
        A((f"arm_root_clamp_{n}", "01_AIRFRAME", 0.115,
           (215 * math.cos(a), 215 * math.sin(a), 0),
           "CNC AL7075 split clamp", "ESTIMATED"))
        A((f"arm_harness_{n}", "11_CABLE", 0.048,
           (rc * math.cos(a), rc * math.sin(a), -14), "6 AWG + signal", "ESTIMATED"))

    # ---- airframe ---------------------------------------------------------
    for sy, tag in ((+1, "L"), (-1, "R")):
        A((f"spine_rail_{tag}", "01_AIRFRAME", 0.420, (0, sy * 105, 0),
           "CF box 55x26x2.5, 510 long", "CALCULATED"))
    for sx, tag in ((+1, "F"), (-1, "A")):
        A((f"spine_cross_{tag}", "01_AIRFRAME", 0.105, (sx * 175, 0, 0),
           "CF box cross member", "CALCULATED"))
    for sx in (+1, -1):
        for sy in (+1, -1):
            A((f"corner_casting_{sx}{sy}", "01_AIRFRAME", 0.086,
               (sx * 175, sy * 105, 0), "CNC AL7075 node", "ESTIMATED"))
    A(("manipulator_hub", "01_AIRFRAME", 0.385, (45, 0, -48),
       "CNC AL7075, sized on 34 Nm", "CALCULATED"))
    A(("panel_top_service", "01_AIRFRAME", 0.155, (-40, 0, 116),
       "PA-CF printed, 6 x quarter-turn", "ESTIMATED"))
    A(("panel_lower_service", "01_AIRFRAME", 0.140, (-20, 0, -152),
       "CF sandwich, 6 x quarter-turn", "ESTIMATED"))
    for sy, tag in ((+1, "L"), (-1, "R")):
        A((f"panel_side_{tag}", "01_AIRFRAME", 0.088, (0, sy * 132, -60),
           "PA-CF printed", "ESTIMATED"))
    A(("fastener_allowance", "01_AIRFRAME", 0.165, (0, 0, -20),
       "A2 stainless, distributed", "ESTIMATED"))

    # ---- landing gear -----------------------------------------------------
    for sx in (+1, -1):
        for sy in (+1, -1):
            A((f"lg_leg_{sx}{sy}", "10_LANDING_GEAR", 0.132,
               (sx * 150, sy * 220, -300), "CF 22/18 tube, splayed", "CALCULATED"))
    for sy, tag in ((+1, "L"), (-1, "R")):
        A((f"lg_skid_{tag}", "10_LANDING_GEAR", 0.148, (0, sy * 340, -520),
           "CF 26/22 tube, 620 long", "CALCULATED"))
        A((f"lg_damper_{tag}", "10_LANDING_GEAR", 0.062, (0, sy * 300, -430),
           "elastomeric shock leg", "ESTIMATED"))
    for sx in (+1, -1):
        for sy in (+1, -1):
            A((f"lg_foot_{sx}{sy}", "10_LANDING_GEAR", 0.030,
               (sx * 250, sy * 340, -530), "replaceable TPU foot", "ESTIMATED"))

    # ---- avionics ---------------------------------------------------------
    A(("flight_controller", "04_AVIONICS", 0.100, (-40, -62, 78),
       "Holybro Pixhawk 6X class", "VENDOR"))
    A(("fc_isolator_tray", "04_AVIONICS", 0.045, (-40, -62, 62),
       "4 x wire-rope isolator", "ESTIMATED"))
    A(("companion_computer", "04_AVIONICS", 0.300, (-40, 62, 76),
       "Jetson Orin NX + carrier", "VENDOR"))
    A(("compute_cooling", "04_AVIONICS", 0.085, (-40, 62, 104),
       "heatsink + 50 mm fan", "ESTIMATED"))
    A(("power_distribution", "04_AVIONICS", 0.150, (-125, 0, -28),
       "12S PDB, 4 ESC pairs", "ESTIMATED"))
    A(("power_switch_relay", "04_AVIONICS", 0.090, (-165, 0, -28),
       "anti-spark + isolation relay", "ESTIMATED"))
    A(("dcdc_converters", "04_AVIONICS", 0.075, (-125, -70, 60),
       "12S -> 19 V / 5 V", "ESTIMATED"))
    A(("telemetry_radio", "04_AVIONICS", 0.035, (-160, 55, 62), "900 MHz", "ESTIMATED"))
    A(("rc_receiver", "04_AVIONICS", 0.020, (-160, -30, 62), "dual-band", "ESTIMATED"))
    for k, (ax, ay) in enumerate(((-215, 90), (-215, -90), (170, 132), (170, -132))):
        A((f"antenna_{k+1}", "04_AVIONICS", 0.020, (ax, ay, 20),
           "whip / patch", "ESTIMATED"))
    A(("wiring_harness_main", "11_CABLE", 0.395, (-45, 0, -10),
       "HV + signal looms", "ESTIMATED"))
    A(("connectors_bulkheads", "11_CABLE", 0.115, (-60, 0, -20),
       "module-boundary connectors", "ESTIMATED"))

    # ---- perception -------------------------------------------------------
    A(("lidar", "05_PERCEPTION", 0.300, (30, 0, 212),
       "360 x 45 deg 3D LiDAR", "VENDOR"))
    A(("lidar_mast", "05_PERCEPTION", 0.078, (30, 0, 150),
       "CF mast + isolator", "ESTIMATED"))
    A(("rgb_gimbal", "05_PERCEPTION", 0.250, (198, 96, -88),
       "3-axis 4K, front-left boom", "VENDOR"))
    A(("gimbal_boom", "05_PERCEPTION", 0.058, (150, 96, -40),
       "CF boom", "CALCULATED"))
    A(("depth_camera", "05_PERCEPTION", 0.075, (216, 0, 16),
       "stereo IR + RGB", "VENDOR"))
    A(("depth_bracket", "05_PERCEPTION", 0.036, (200, 0, 10), "AL bracket", "ESTIMATED"))
    for k, sy in enumerate((+1, -1)):
        A((f"gnss_rtk_{k+1}", "05_PERCEPTION", 0.110, (-150, sy * 125, 182),
           "RTK antenna, 250 mm baseline", "VENDOR"))
        A((f"gnss_mast_{k+1}", "05_PERCEPTION", 0.028, (-150, sy * 125, 120),
           "CF mast", "ESTIMATED"))
    A(("range_finder", "05_PERCEPTION", 0.030, (-140, -72, -156),
       "downward laser altimeter", "VENDOR"))
    A(("optical_flow", "05_PERCEPTION", 0.018, (-140, 72, -156),
       "flow + ToF", "VENDOR"))
    A(("inspection_lamp", "05_PERCEPTION", 0.095, (198, -96, -88),
       "high-CRI LED cluster", "ESTIMATED"))

    # ---- liquid service ---------------------------------------------------
    A(("reservoir_shell", "09_LIQUID", 0.215, (-72, 0, -100),
       "blow-moulded HDPE, 1.15 L, baffled", "CALCULATED"))
    A(("reservoir_baffles", "09_LIQUID", 0.048, (-72, 0, -100),
       "3 x transverse, 65 % open", "CALCULATED"))
    A(("pump", "09_LIQUID", 0.285, (-148, 0, -92),
       "diaphragm, 0-6 bar, 1.2 L/min", "VENDOR"))
    A(("filter_40um", "09_LIQUID", 0.052, (-118, 58, -92),
       "inline 40 um, serviceable", "VENDOR"))
    A(("check_valve", "09_LIQUID", 0.024, (-100, 58, -92),
       "0.2 bar cracking", "VENDOR"))
    A(("relief_bypass_valve", "09_LIQUID", 0.038, (-118, -58, -92),
       "6.5 bar bypass to tank", "VENDOR"))
    A(("pressure_sensor", "09_LIQUID", 0.026, (-90, -58, -92),
       "0-10 bar, 4-20 mA", "VENDOR"))
    A(("quick_connect", "09_LIQUID", 0.032, (-40, 0, -128),
       "dry-break, tank side", "VENDOR"))
    A(("hose_body", "09_LIQUID", 0.045, (0, 0, -110), "PTFE-lined", "ESTIMATED"))
    A(("hose_arm_routed", "09_LIQUID", 0.060, (150, 0, -280),
       "along links, service loops", "ESTIMATED"))
    A(("fluid_mount_bracket", "09_LIQUID", 0.125, (-95, 0, -128),
       "CF tray + quarter-turn latches", "ESTIMATED"))

    # ---- battery ----------------------------------------------------------
    for k, sy in enumerate((+1, -1)):
        A((f"battery_cartridge_{k+1}", "03_BATTERY", batt.kg / 2.0,
           (batt_x, sy * 92, -102), batt.label, "ESTIMATED"))
    A(("battery_tray_rail", "03_BATTERY", 0.185, (0, 0, -132),
       "indexed rail, 3 detents", "CALCULATED"))
    A(("battery_latches", "03_BATTERY", 0.070, (-100, 0, -102),
       "2 x quarter-turn + XT150", "ESTIMATED"))

    # ---- safety / thermal -------------------------------------------------
    A(("emergency_stop", "15_SAFETY", 0.045, (-205, 0, -30),
       "HV isolation, external", "ESTIMATED"))
    A(("status_beacon_leds", "15_SAFETY", 0.055, (0, 0, 128),
       "nav + status", "ESTIMATED"))
    A(("arm_brake_release", "15_SAFETY", 0.035, (45, 0, -70),
       "manual brake release", "ESTIMATED"))
    A(("cooling_ducting", "04_AVIONICS", 0.040, (-40, 0, 96),
       "intake + exhaust path", "ESTIMATED"))
    return it


def cg(items):
    m = sum(i[2] for i in items)
    return (sum(i[2] * i[3][0] for i in items) / m,
            sum(i[2] * i[3][1] for i in items) / m,
            sum(i[2] * i[3][2] for i in items) / m, m)


def build(batt, arm_cfg="STOWED", tool="none", carried=0.0, fluid_l=0.0,
          batt_x=0.0, with_battery=True, with_arm=True, with_fluid_hw=True):
    it = [x for x in base_items(batt, batt_x)]
    if not with_battery:
        it = [x for x in it if x[1] != "03_BATTERY"]
    if not with_fluid_hw:
        it = [x for x in it if x[1] != "09_LIQUID"]
    if fluid_l > 0:
        it.append(("fluid_charge", "09_LIQUID", fluid_l * 1.0, (-72, 0, -100 - 18),
                   f"{fluid_l:.1f} L service fluid", "CALCULATED"))
    if with_arm:
        ai, tp = arm_items(CFG[arm_cfg], tool, carried)
        for nm, m, pos in ai:
            it.append((nm, "06_MANIPULATOR", m, pos, "sized on joint torque",
                       "CALCULATED"))
    return it


def hover_power(mtow):
    T = mtow * G
    p_ideal = T ** 1.5 / math.sqrt(2 * RHO * A_DISC)
    return p_ideal / FM_COAX / ETA_DRIVE, T / A_DISC
