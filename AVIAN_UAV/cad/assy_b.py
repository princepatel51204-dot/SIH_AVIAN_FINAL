"""AVIAN master assembly + the six Phase 3 clearance checks."""
from __future__ import annotations
import math
import numpy as np
import cadquery as cq

import params_b as P
import helpers as H
from helpers import Part, T, RX, RY, RZ, chain
import kin_b as K
import parts_b as B
import parts_b2 as B2
import render as R

I4 = np.eye(4)

# actuator point masses inside each joint housing (VENDOR-class, ESTIMATED)
ACT = {"J1": 0.190, "J2": 0.285, "J3": 0.230, "J4": 0.095, "J5": 0.085, "J6": 0.070}


def build(cfg="01_FLIGHT", batt_index=0):
    """batt_index: -1 / 0 / +1 detent on the 3-position mount."""
    q = P.CFG[cfg]
    tool = P.CFG_TOOL[cfg]
    fluid = P.CFG_FLUID[cfg]
    bx = batt_index * P.vRailTravel
    reg = H.Registry()
    A = reg.add

    # ---------------- 01 AIRFRAME ----------------------------------------
    for sy, tg in ((+1, "L"), (-1, "R")):
        A(Part(f"spine_rail_{tg}", "01_AIRFRAME", B.spine_rail(sy), I4, "CFRP",
               "CF_MATTE", explode=(0, sy, 0), note="CF box 55x26x2.5"))
        A(Part(f"panel_side_{tg}", "01_AIRFRAME", B.panel_side(sy), I4, "POLYMER",
               "GRAPHITE", explode=(0, sy * 1.6, 0), note="PA-CF, 4 x quarter-turn"))
    for sx, tg in ((+1, "F"), (-1, "A")):
        A(Part(f"spine_cross_{tg}", "01_AIRFRAME", B.spine_cross(sx), I4, "CFRP",
               "CF_MATTE", explode=(sx, 0, 0)))
    for sx in (+1, -1):
        for sy in (+1, -1):
            tg = f"{'F' if sx>0 else 'A'}{'L' if sy>0 else 'R'}"
            A(Part(f"corner_node_{tg}", "01_AIRFRAME", B.corner_node(sx, sy), I4,
                   "AL7075", "AL_ANOD", explode=(sx * .6, sy * .6, 0),
                   note="CNC AL7075: 2 rail ends + cross + arm clamp"))
    A(Part("manipulator_hub", "01_AIRFRAME", B.manipulator_hub(), I4, "AL7075",
           "AL_ANOD", explode=(0, 0, -1),
           note="ring + ties; interface-driven, sigma 0.5 MPa vs 500 MPa yield"))
    A(Part("panel_top", "01_AIRFRAME", B.panel_top(), I4, "POLYMER", "GRAPHITE",
           explode=(0, 0, 1.5), note="avionics access, 4 x quarter-turn"))
    A(Part("panel_bottom", "01_AIRFRAME", B.panel_bottom(), I4, "CFRP", "CF_MATTE",
           explode=(0, 0, -1.5), note="payload/fluid access, 4 x quarter-turn"))

    # ---------------- 02 PROPULSION --------------------------------------
    for i, ang in enumerate(P.ARM_ANGLES):
        n = i + 1
        W = chain(RZ(ang))
        ex = (math.cos(math.radians(ang)), math.sin(math.radians(ang)), 0)
        A(Part(f"arm_tube_{n}", "02_PROPULSION", B.arm_tube(), W, "CFRP", "CF_TUBE",
               frame=f"arm_{n}_link", explode=ex, note="CF 25/21"))
        A(Part(f"arm_clamp_{n}", "02_PROPULSION", B.arm_clamp(), W, "AL7075",
               "AL_ANOD", explode=ex))
        A(Part(f"coax_mount_{n}", "02_PROPULSION", B.coax_mount(), W, "AL7075",
               "AL_ANOD", explode=ex, note="carries both motors on one arm end"))
        A(Part(f"arm_conduit_{n}", "02_PROPULSION", B.arm_conduit(), W, "TPU",
               "TPU_BLACK", explode=(0, 0, -0.6), note="phase + signal"))
        for sz, tg in ((+1, "upper"), (-1, "lower")):
            zc = sz * P.vCoaxSep / 2
            Wm = chain(RZ(ang), T(P.vRMotor, 0, zc))
            cw = (i % 2 == 0) if sz > 0 else (i % 2 == 1)
            A(Part(f"motor_{n}_{tg}", "02_PROPULSION", B2 and B.motor(sz > 0), Wm,
                   "AL6061", "MOTOR", mass=P.vMotorMass,
                   frame=f"motor_{n}_{tg}_link", explode=(0, 0, sz),
                   note=f"U8 II KV{P.vMotorKV} PLACEHOLDER ENVELOPE, "
                        f"{'CW' if cw else 'CCW'}"))
            zp = zc + sz * (P.vMotorBaseH + P.vMotorBellH - 1)
            A(Part(f"propeller_{n}_{tg}", "02_PROPULSION",
                   B.propeller(cw, sz > 0), chain(RZ(ang), T(P.vRMotor, 0, zp)),
                   "CFRP", "PROP", mass=P.vPropMass,
                   frame=f"prop_{n}_{tg}_link", explode=(0, 0, sz * 1.8),
                   note=f"G28x9.2 {'CW' if cw else 'CCW'}"))
            # the lower ESC is inverted so its heatsink faces away from the arm
            # tube instead of into it (CHECK 7)
            We = chain(RZ(ang), T(P.vESCr, 0, sz * 26),
                       I4 if sz > 0 else RX(180))
            A(Part(f"esc_{n}_{tg}", "02_PROPULSION", B.esc_module(), We, "GENERIC",
                   "DARKGREY", mass=P.vESCMass, frame=f"esc_{n}_{tg}_link",
                   explode=(0, 0, sz), note=f"ALPHA {P.vESCRating:.0f}A 12S"))
            A(Part(f"esc_saddle_{n}_{tg}", "02_PROPULSION", B.esc_saddle(),
                   chain(RZ(ang), T(P.vESCr, 0, 0)), "AL6061", "AL_BRIGHT",
                   explode=ex))

    # ---------------- 03 BATTERY -----------------------------------------
    for k, sy in enumerate((+1, -1)):
        A(Part(f"battery_cartridge_{k+1}", "03_BATTERY", B2.battery_cartridge(),
               chain(T(bx, sy * P.vBattY, P.vBattZ)), "GENERIC", "BATT",
               mass=P.vBattMassEach, frame=f"battery_{k+1}_link",
               explode=(0, sy * 1.4, -0.4),
               note=f"12S {P.vBattAh:.0f} Ah, {12*3.7*P.vBattAh:.0f} Wh, hot-swap"))
        A(Part(f"battery_latch_{k+1}", "03_BATTERY", B2.battery_latch(sy),
               chain(T(bx, 0, 0)), "AL6061", "AL_BRIGHT", explode=(-1.4, 0, 0)))
    A(Part("battery_rail", "03_BATTERY", B2.battery_rail(), I4, "CFRP", "CF_MATTE",
           explode=(0, 0, -1),
           note=f"3-position indexed, +/-{P.vRailTravel:.0f} mm; NOT powered"))

    # ---------------- 04 AVIONICS ----------------------------------------
    A(Part("flight_controller", "04_AVIONICS", B2.flight_controller(),
           chain(T(*P.vFCPos)), "GENERIC", "PCB", mass=0.100, frame="imu_link",
           explode=(0, 0, 1), note="Pixhawk 6X class PLACEHOLDER"))
    A(Part("fc_isolator_tray", "04_AVIONICS", B2.fc_tray(), chain(T(*P.vFCPos)),
           "AL6061", "AL_BRIGHT", mass=0.045, explode=(0, 0, 0.6),
           note="4 x wire-rope isolator"))
    A(Part("companion_computer", "04_AVIONICS", B2.companion_computer(),
           chain(T(*P.vCCPos)), "GENERIC", "DARKGREY", mass=0.300,
           frame="companion_link", explode=(0, 0, 1),
           note="Jetson Orin NX class PLACEHOLDER"))
    A(Part("compute_cooling", "04_AVIONICS", B2.compute_cooling(),
           chain(T(*P.vCCPos)), "AL6061", "AL_BRIGHT", mass=0.085,
           explode=(0, 0, 1.4)))
    A(Part("power_distribution", "04_AVIONICS", B2.pdb(), chain(T(*P.vPDBPos), RZ(90)),
           "GENERIC", "PCB", mass=0.150, explode=(0, 0, -0.8),
           note="12S PDB, 4 arm pairs"))
    A(Part("power_switch_relay", "04_AVIONICS", B2.small_box(58, 42, 26, 14),
           chain(T(-215, -97, 49)), "GENERIC", "DARKGREY", mass=0.090,
           explode=(-1, 0, 0), note="anti-spark + HV isolation"))
    A(Part("dcdc_converters", "04_AVIONICS", B2.small_box(62, 42, 20),
           chain(T(-150, -70, 46)), "GENERIC", "DARKGREY", mass=0.075,
           explode=(0, 0, 0.8)))
    A(Part("telemetry_radio", "04_AVIONICS", B2.small_box(52, 30, 14, 12),
           chain(T(-160, 55, 62)), "GENERIC", "DARKGREY", mass=0.035,
           explode=(0, 0, 0.8)))
    A(Part("rc_receiver", "04_AVIONICS", B2.small_box(44, 26, 12),
           chain(T(-160, -30, 62)), "GENERIC", "DARKGREY", mass=0.020,
           explode=(0, 0, 0.8)))
    # CHECK 1c: aft diversity pair moved inboard out of the aft rotor discs
    for k, (ax, ay) in enumerate(((-195, 62), (-195, -62))):
        A(Part(f"antenna_{k+1}", "04_AVIONICS", B2.antenna(), chain(T(ax, ay, 20)),
               "POLYMER", "GRAPHITE", mass=0.020, explode=(0, 0, 0.8)))
    # CHECK 1c: the forward diversity pair originally sat at (170, +/-132, 10)
    # with a 38 deg outboard bend, which put the whip INSIDE the forward rotor
    # discs. Moved inboard and hung downward so the radius to each rotor centre
    # (393 mm) exceeds vPropR + 25 mm.
    for k, (ax, ay) in enumerate(((100, 160), (100, -160))):
        A(Part(f"antenna_{k+3}", "04_AVIONICS", B2.antenna(180),
               chain(T(ax, ay, -140)), "POLYMER", "GRAPHITE", mass=0.020,
               explode=(0, 0, -0.6)))

    # ---------------- 05 PERCEPTION --------------------------------------
    A(Part("lidar", "05_PERCEPTION", B2.lidar(), chain(T(*P.vLidarPos)), "GENERIC",
           "AL_BRIGHT", mass=0.300, frame="lidar_link", explode=(0, 0, 1.6),
           note="360 x 45 deg PLACEHOLDER ENVELOPE"))
    A(Part("lidar_mast", "05_PERCEPTION", B2.lidar_mast(), chain(T(*P.vLidarPos)),
           "CFRP", "CF_TUBE", mass=0.078, explode=(0, 0, 1.1)))
    A(Part("rgb_gimbal", "05_PERCEPTION", B2.gimbal(), chain(T(*P.vGimbalPos)),
           "GENERIC", "SENSOR", mass=0.250, frame="camera_link",
           explode=(0.8, 0.5, -1), note="3-axis, front-left; keeps the manipulator "
                                        "out of its sight line to the work point"))
    A(Part("gimbal_boom", "05_PERCEPTION", B2.gimbal_boom(1), I4, "CFRP", "CF_TUBE",
           mass=0.058, explode=(1, 0.5, 0)))
    Wd = chain(T(*P.vDepthPos), RY(-P.vDepthTilt))
    A(Part("depth_camera", "05_PERCEPTION", B2.depth_camera(), Wd, "GENERIC",
           "SENSOR", mass=0.075, frame="depth_camera_link", explode=(1, 0, 0),
           note=f"tilted {abs(P.vDepthTilt):.0f} deg down"))
    for k, (gx, gy, gz) in enumerate(P.vGnssPos):
        A(Part(f"gnss_antenna_{k+1}", "05_PERCEPTION", B2.gnss_antenna(),
               chain(T(gx, gy, gz)), "GENERIC", "AL_BRIGHT", mass=0.110,
               frame="gps_link" if k == 0 else "gps_secondary_link",
               explode=(0, 0, 1.8), note="RTK, 250 mm heading baseline"))
        A(Part(f"gnss_mast_{k+1}", "05_PERCEPTION", B2.gnss_mast(),
               chain(T(gx, gy, gz)), "CFRP", "CF_TUBE", mass=0.028,
               explode=(0, 0, 1.2)))
    A(Part("range_finder", "05_PERCEPTION", B2.small_box(34, 32, 22),
           chain(T(*P.vRangePos)), "GENERIC", "SENSOR", mass=0.030,
           frame="range_link", explode=(0, 0, -1)))
    A(Part("optical_flow", "05_PERCEPTION", B2.small_box(28, 28, 14),
           chain(T(*P.vFlowPos)), "GENERIC", "SENSOR", mass=0.018,
           frame="flow_link", explode=(0, 0, -1)))
    A(Part("inspection_lamp", "05_PERCEPTION", B2.lamp_cluster(),
           chain(T(*P.vLampPos)), "AL6061", "AL_BRIGHT", mass=0.095,
           explode=(0.8, -0.5, -1)))
    A(Part("lamp_boom", "05_PERCEPTION", B2.gimbal_boom(-1), I4, "CFRP", "CF_TUBE",
           mass=0.058, explode=(1, -0.5, 0)))

    # ---------------- 10 LANDING GEAR ------------------------------------
    for sy, tg in ((+1, "L"), (-1, "R")):
        A(Part(f"gear_skid_{tg}", "10_LANDING_GEAR", B2.gear_skid(sy), I4, "CFRP",
               "CF_TUBE", explode=(0, sy, -0.5)))
        for sx, t2 in ((+1, "F"), (-1, "A")):
            A(Part(f"gear_leg_{tg}{t2}", "10_LANDING_GEAR", B2.gear_leg(sx, sy), I4,
                   "CFRP", "CF_TUBE", explode=(0.2 * sx, 0.7 * sy, -0.7)))
            A(Part(f"gear_foot_{tg}{t2}", "10_LANDING_GEAR", B2.gear_foot(sx, sy),
                   I4, "TPU", "TPU_BLACK", explode=(0, 0, -1.3),
                   note="replaceable"))
            A(Part(f"gear_damper_{tg}{t2}", "10_LANDING_GEAR",
                   B2.gear_damper(sx, sy), I4, "TPU", "TPU_BLACK", mass=0.062,
                   explode=(0.2 * sx, 0.6 * sy, -0.6)))

    # ---------------- 09 LIQUID SERVICE ----------------------------------
    Wt = chain(T(*P.vTankPos))
    A(Part("tank_shell", "09_LIQUID", B2.tank_shell(), Wt, "POLYMER", "GRAPHITE",
           frame="tank_link", explode=(0, 0, -1.2),
           note=f"{P.vTankVol:.2f} L geometric, HDPE, dry-break outlet"))
    A(Part("tank_baffles", "09_LIQUID", B2.tank_baffles(), Wt, "POLYMER",
           "DARKGREY", explode=(0, 0, -1.2),
           note=f"{P.vBaffleN} transverse, {P.vBaffleOpen*100:.0f} % open; moves the "
                f"slosh mode 1.86 -> 3.95 Hz"))
    if fluid > 0:
        A(Part("fluid_charge", "09_LIQUID", B2.tank_fluid(fluid), Wt, "FLUID",
               "FLUID", mass=fluid * 1.0, explode=(0, 0, -1.2),
               note=f"{fluid:.1f} L service fluid"))
    A(Part("cartridge_frame", "09_LIQUID", B2.cartridge_frame(),
           chain(T(*P.vCartPos)), "AL6061", "AL_ANOD", mass=0.196,
           frame="fluid_cartridge_link", explode=(0, 0, -1.6),
           note="removable service-fluid TANK cartridge; one dry-break fluid "
                "port. Pump/filter/valves stay on the aircraft -- the "
                "consumable is the fluid, not the pump"))
    A(Part("pump", "09_LIQUID", B2.pump(), chain(T(*P.vPumpPos)), "GENERIC",
           "DARKGREY", mass=0.285, frame="pump_link", explode=(0, 0, -1.4),
           note=f"diaphragm 0-{P.vPumpBarMax:.0f} bar, {P.vPumpLmin:.1f} L/min "
                "-- PLACEHOLDER COMPONENT, VERIFY BEFORE FABRICATION"))
    A(Part("filter_40um", "09_LIQUID", B2.filter_unit(),
           chain(T(-250, 1, 6), RY(90)), "GENERIC", "AL_BRIGHT", mass=0.052,
           explode=(0, 0, -1.4),
           note=f"{P.vFilterMicron:.0f} um, serviceable bowl, in the cartridge "
                "end-bay"))
    A(Part("check_valve", "09_LIQUID", B2.valve(24, 38),
           chain(T(-242, 42, 7)), "GENERIC", "AL_BRIGHT", mass=0.024,
           explode=(0, 0, -1.4), note=f"{P.vCheckCrack:.1f} bar cracking"))
    A(Part("relief_valve", "09_LIQUID", B2.valve(28, 44),
           chain(T(-229, -40, 9)), "GENERIC", "AL_BRIGHT", mass=0.038,
           explode=(0, 0, -1.4), note=f"{P.vReliefSet:.1f} bar bypass to tank"))
    A(Part("pressure_sensor", "09_LIQUID", B2.small_box(30, 24, 22),
           chain(T(-205, 40, -30)), "GENERIC", "BLUE_ID", mass=0.026,
           explode=(0, 0, -1.4), note="0-10 bar, blockage detection"))
    A(Part("quick_connect", "09_LIQUID", B2.valve(30, 34),
           chain(T(-45, 44, -60), RX(90)), "GENERIC", "BLUE_ID", mass=0.032,
           explode=(1.2, 0, 0),
           note="dry-break coupling on the cartridge forward face -- the ONLY "
                "fluid joint broken during a cartridge change"))
    A(Part("hose_tank_pump", "09_LIQUID", B2.hose_run(
        [(-170, 16, -108), (-196, 22, -96), (-224, 20, -64), (-238, 10, -40)]),
        I4, "TPU", "TPU_BLACK", mass=0.020, explode=(0, 0, -1.4),
        note="inside the cartridge"))
    A(Part("hose_pump_arm", "09_LIQUID", B2.hose_run(
        [(-45, 52, -60), (-24, 60, -70), (0, 44, -80), (38, 14, -88)]),
        I4, "TPU", "TPU_BLACK", mass=0.028, explode=(0.6, 0, -0.5),
        note="cartridge dry-break to the manipulator hub feed-through"))

    # ---------------- 11 CABLE -------------------------------------------
    A(Part("harness_main", "11_CABLE", B2.small_box(150, 56, 18),
           chain(T(-70, 0, 14)), "TPU", "TPU_BLACK", mass=0.395,
           explode=(0, 0, -0.6), note="HV + signal looms, routed in the spine"))
    A(Part("connector_bulkhead", "11_CABLE", B2.small_box(70, 40, 22),
           chain(T(-14, 0, 30)), "POLYMER", "BLUE_ID", mass=0.115,
           explode=(0, 0, -0.9), note="module-boundary connectors"))

    # ---------------- 15 SAFETY ------------------------------------------
    A(Part("emergency_stop", "15_SAFETY", B2.small_box(44, 40, 26),
           chain(T(-210, 0, 60)), "POLYMER", "WARN", mass=0.045,
           explode=(-1.2, 0, 0), note="external HV isolation"))
    A(Part("status_beacon", "15_SAFETY", B2.small_box(56, 40, 16),
           chain(T(60, 0, 128)), "POLYMER", "BLUE_ID", mass=0.055,
           explode=(0, 0, 1.2)))

    # ---------------- 06 MANIPULATOR -------------------------------------
    F = K.joint_frames(q)
    A(Part("arm_base_flange", "06_MANIPULATOR", B2.arm_base_flange(), F[0],
           "AL7075", "AL_ANOD", frame="arm_base_link", explode=(0, 0, -0.8),
           note=f"{P.vHubBoltN} x M5 to the hub; fluid + electrical bulkhead"))
    LFN = [B2.link1, B2.link2, B2.link3, B2.link4, B2.link5, B2.link6]
    MATS = ["AL7075", "CFRP", "CFRP", "AL7075", "AL7075", "AL7075"]
    COLS = ["AL_ANOD", "CF_TUBE", "CF_TUBE", "AL_ANOD", "AL_ANOD", "AL_ANOD"]
    for i, fn in enumerate(LFN):
        n = i + 1
        jn, off, ax, lim, tq, sp, desc = P.JOINTS[i]
        # link_n is authored from J_n at its origin extending to J_{n+1},
        # so it mates to F[n] -- F[n+1] drew every link one joint downstream.
        A(Part(f"arm_link_{n}", "06_MANIPULATOR", fn(), F[n], MATS[i], COLS[i],
               frame=f"arm_link_{n}", moving=True, explode=(0, 0, -0.4 * n),
               note=f"{jn} {desc}: axis {ax}, {lim[0]:+.0f}..{lim[1]:+.0f} deg, "
                    f"{tq:.0f} Nm, {sp:.0f} deg/s, hard stops"))
        A(Part(f"actuator_{jn}", "06_MANIPULATOR",
               cq.Workplane("XY").box(1, 1, 1), F[n], "GENERIC", "GRAPHITE",
               mass=ACT[jn], moving=True, explode=(0, 0, -0.4 * n),
               note=f"{tq:.0f} Nm module (mass only; envelope is the housing)"))

    t0, fts, tc, tip = K.tool_frames(q)
    A(Part("ft_sensor", "06_MANIPULATOR", B2.ft_sensor(), t0, "AL7075", "BLUE_ID",
           mass=0.080, frame="ft_sensor_link", moving=True, explode=(0, 0, -1.4),
           note="6-axis, contact-force control"))
    A(Part("tool_changer_master", "07_TOOL_INTERFACE", B2.tc_master(), fts,
           "AL7075", "AL_BRIGHT", mass=0.075, frame="tool_changer_link",
           moving=True, explode=(0, 0, -1.8),
           note=f"{P.vTCPinN} locking pins + {P.vTCElecWays}-way + "
                f"{P.vTCFluidPorts} fluid"))
    if tool != "none":
        Wt2 = chain(fts, T(0, 0, -P.vTCMasterH))
        A(Part("tool_plate", "07_TOOL_INTERFACE", B2.tc_tool_plate(), Wt2,
               "AL7075", "AL_BRIGHT", mass=0.055, frame="tool0", moving=True,
               explode=(0, 0, -2.1)))
        if tool == "gripper":
            A(Part("repair_gripper", "08_REPAIR_TOOLS", B2.gripper_tool(), Wt2,
                   "AL7075", "AL_ANOD", mass=0.380, frame="gripper_link",
                   moving=True, explode=(0, 0, -2.5),
                   note=f"parallel 2-jaw, {P.vGripStroke:.0f} mm stroke, "
                        f"{P.vGripForce:.0f} N ESTIMATED"))
        else:
            A(Part("service_nozzle", "08_REPAIR_TOOLS", B2.nozzle_tool(), Wt2,
                   "AL7075", "AL_BRIGHT", mass=0.240, frame="nozzle_link",
                   moving=True, explode=(0, 0, -2.5),
                   note=f"{P.vNozzleTipD:.1f} mm replaceable orifice"))
    return reg


NON_GEO = [("fastener_allowance", "01_AIRFRAME", 0.165, (0, 0, -20)),
           ("wiring_extra", "11_CABLE", 0.120, (-40, 0, 0))]


def mass_cg(reg):
    items = [(p.name, p.group, p.mass_kg(), p.world_com()) for p in reg.parts]
    items += [(n, g, m, np.array(c)) for n, g, m, c in NON_GEO]
    M = sum(i[2] for i in items)
    c = sum(i[2] * i[3] for i in items) / M
    return M, c, items
