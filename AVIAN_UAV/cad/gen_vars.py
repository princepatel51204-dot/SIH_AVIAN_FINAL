"""Emit the Onshape Variable Studio source and the master parameter table
directly from params_b, so the package can never drift from the model."""
from __future__ import annotations
import params_b as P

# (name, value, unit, category, confidence, description)
def V(n, v, u, c, cf, d):
    return dict(name=n, value=v, unit=u, cat=c, conf=cf, desc=d)


ROWS = [
    # ---- 1 AIRCRAFT --------------------------------------------------------
    V("vDiagonal", P.vDiagonal, "mm", "Aircraft", "CALCULATED",
      "Motor-to-motor diagonal. Set by the Phase 2 geometry sweep: the "
      "largest diagonal at which a 28 in prop still clears the manipulator "
      "reach envelope."),
    V("vArmCount", P.vArmCount, "", "Aircraft", "CALCULATED",
      "Structural propulsion arms. 4 arms x 2 rotors = coaxial X8."),
    V("vRMotor", P.vRMotor, "mm", "Aircraft", "CALCULATED",
      "Rotor axis radius = vDiagonal / 2."),
    V("vNominalMTOW", 25.753, "kg", "Aircraft", "CALCULATED",
      "Nominal MTOW: dry aircraft + 1.0 L service fluid, no external "
      "payload. Measured on the CAD model, 137 parts."),
    V("vMaxMTOW", 27.753, "kg", "Aircraft", "CALCULATED",
      "Maximum MTOW: nominal + 2.0 kg mission payload at the tool."),
    V("vLandingClearance", abs(P.vGearGround), "mm", "Aircraft", "CALCULATED",
      "Skid plane below the airframe datum."),

    # ---- 2 PROPULSION ------------------------------------------------------
    V("vPropDiameter", P.vPropDia, "mm", "Propulsion", "VENDOR",
      "T-Motor G28x9.2 CF folding propeller, 28 in."),
    V("vPropPitch", P.vPropPitch, "mm", "Propulsion", "VENDOR",
      "9.2 in geometric pitch."),
    V("vPropR", P.vPropR, "mm", "Propulsion", "CALCULATED",
      "Propeller radius = vPropDiameter / 2."),
    V("vCoaxSep", P.vCoaxSep, "mm", "Propulsion", "CALCULATED",
      "Upper-to-lower rotor plane separation, 17.4 % of diameter."),
    V("vTipGapMin", P.vTipGapMin, "mm", "Propulsion", "ASSUMED",
      "Minimum acceptable tip-to-tip gap between adjacent discs. Design rule."),
    V("vMotorKV", P.vMotorKV, "rpm/V", "Propulsion", "VENDOR",
      "T-Motor U8 II KV100."),
    V("vMotorMass", P.vMotorMass, "kg", "Propulsion", "VENDOR", "Per motor."),
    V("vPropMass", P.vPropMass, "kg", "Propulsion", "VENDOR",
      "Per propeller."),
    V("vESCRating", P.vESCRating, "A", "Propulsion", "VENDOR",
      "T-Motor ALPHA 60A 12S V1.2."),
    V("vThrustPerArm", P.vThrustPerArm, "kgf", "Propulsion", "VENDOR",
      "Coaxial pair at 100 % throttle, T-Motor bench data. NOT flight-test "
      "verified on this airframe."),
    V("vCurrentPerArm", P.vCurrentPerArm, "A", "Propulsion", "VENDOR",
      "Coaxial pair at 100 % throttle, 44.4 V."),
    V("vESCr", P.vESCr, "mm", "Propulsion", "CALCULATED",
      "ESC station along the arm."),

    # ---- 3 BATTERY ---------------------------------------------------------
    V("vBatteryVoltage", P.vBattCells * 3.7, "V", "Battery", "CALCULATED",
      "12S nominal. 12 x 3.7 V."),
    V("vBatteryCapacity", P.vBattAh * P.vBattPacks, "Ah", "Battery",
      "CALCULATED", "Two parallel 12 Ah cartridges."),
    V("vBattCells", P.vBattCells, "", "Battery", "CALCULATED", "Cells in "
      "series."),
    V("vBattPacks", P.vBattPacks, "", "Battery", "CALCULATED",
      "Parallel cartridges."),
    V("vBattWh", P.vBattWh, "Wh", "Battery", "CALCULATED",
      "Nameplate energy. 80 % usable is assumed for endurance."),
    V("vBattL", P.vBattL, "mm", "Battery", "ESTIMATED",
      "Cartridge length. Proportioned long-and-narrow so the packs clear the "
      "118 mm manipulator hub (CHECK 7)."),
    V("vBattW", P.vBattW, "mm", "Battery", "ESTIMATED", "Cartridge width."),
    V("vBattH", P.vBattH, "mm", "Battery", "ESTIMATED", "Cartridge height."),
    V("vBattY", P.vBattY, "mm", "Battery", "CALCULATED",
      "Cartridge centreline offset from the aircraft centreline."),
    V("vBattZ", P.vBattZ, "mm", "Battery", "CALCULATED",
      "Cartridge centre height. Slung below the spine rails."),
    V("vBatteryRailOffset", P.vRailTravel, "mm", "Battery", "CALCULATED",
      "3-position manual index travel, +/- this value. There is NO powered "
      "CG rail: Phase 2.7 showed the worst lateral CG excursion (67.9 mm) is "
      "not correctable by a fore/aft rail, and trimming 80 mm costs 17 % of "
      "max thrust while hover only uses 44 %."),
    V("vRailPositions", P.vRailPositions, "", "Battery", "CALCULATED",
      "Detents: aft, centre, forward."),

    # ---- 4 MANIPULATOR -----------------------------------------------------
    V("vManipulatorDOF", 6, "", "Manipulator", "CALCULATED",
      "J1 yaw, J2 shoulder pitch, J3 elbow pitch, J4 wrist roll, J5 wrist "
      "pitch, J6 wrist yaw."),
    V("vArmReach", P.vReach, "mm", "Manipulator", "CALCULATED",
      "Arm base to tool point, fully extended = vL1+vL2+vL3+vL4+vL5+"
      "vToolOffset."),
    V("vArmPayload", 2.0, "kg", "Manipulator", "DESIGN TARGET",
      "Mission payload at the tool. Joint torques in params_b are sized on "
      "this plus a 90 N contact force case."),
    V("vArmBaseX", P.vArmBase[0], "mm", "Manipulator", "CALCULATED",
      "Arm base station."),
    V("vArmBaseZ", P.vArmBase[2], "mm", "Manipulator", "CALCULATED",
      "Arm base height."),
    V("vL0", P.vL0, "mm", "Manipulator", "CALCULATED", "J1 to J2 offset."),
    V("vL1", P.vL1, "mm", "Manipulator", "CALCULATED", "Upper arm, J2 to J3."),
    V("vL2", P.vL2, "mm", "Manipulator", "CALCULATED", "Forearm, J3 to J4."),
    V("vL3", P.vL3, "mm", "Manipulator", "CALCULATED", "J4 to J5."),
    V("vL4", P.vL4, "mm", "Manipulator", "CALCULATED", "J5 to J6."),
    V("vL5", P.vL5, "mm", "Manipulator", "CALCULATED", "J6 to tool0 flange."),
    V("vToolOffset", P.vToolOffset, "mm", "Manipulator", "CALCULATED",
      "tool0 flange to the tool point."),

    # ---- 5 TOOLING ---------------------------------------------------------
    V("vTCMasterD", P.vTCMasterD, "mm", "Tooling", "ESTIMATED",
      "Tool-changer master plate diameter."),
    V("vTCMasterH", P.vTCMasterH, "mm", "Tooling", "ESTIMATED",
      "Master plate height."),
    V("vTCToolD", P.vTCToolD, "mm", "Tooling", "ESTIMATED",
      "Tool plate diameter."),
    V("vTCToolH", P.vTCToolH, "mm", "Tooling", "ESTIMATED",
      "Tool plate height."),
    V("vTCPinN", P.vTCPinN, "", "Tooling", "CALCULATED",
      "Locking / alignment pins."),
    V("vTCElecWays", P.vTCElecWays, "", "Tooling", "CALCULATED",
      "Electrical ways across the interface."),
    V("vTCFluidPorts", P.vTCFluidPorts, "", "Tooling", "CALCULATED",
      "Fluid ports across the interface."),
    V("vFTSD", P.vFTSD, "mm", "Tooling", "ESTIMATED",
      "6-axis force/torque sensor diameter. PLACEHOLDER ENVELOPE."),
    V("vGripStroke", P.vGripStroke, "mm", "Tooling", "DESIGN TARGET",
      "Gripper jaw opening."),
    V("vGripForce", P.vGripForce, "N", "Tooling", "ESTIMATED",
      "Grip force. Not yet matched to a selected actuator."),

    # ---- 6 LIQUID ----------------------------------------------------------
    V("vFluidWorkingVolume", P.vTankFill, "L", "Liquid", "CALCULATED",
      "Working service-fluid charge."),
    V("vTankVol", P.vTankVol, "L", "Liquid", "CALCULATED",
      "Geometric tank volume. Verified against the modelled shell: "
      "(vTankL-2t)(vTankW-2t)(vTankH-2t) = 1.17 L."),
    V("vTankL", P.vTankL, "mm", "Liquid", "CALCULATED", "Tank length."),
    V("vTankW", P.vTankW, "mm", "Liquid", "CALCULATED", "Tank width."),
    V("vTankH", P.vTankH, "mm", "Liquid", "CALCULATED", "Tank height."),
    V("vTankWall", P.vTankWall, "mm", "Liquid", "ASSUMED",
      "Blow-moulded HDPE wall."),
    V("vBaffleN", P.vBaffleN, "", "Liquid", "CALCULATED",
      "Transverse baffles. Move the first slosh mode 1.86 -> 3.95 Hz, out of "
      "the attitude-loop band."),
    V("vBaffleOpen", P.vBaffleOpen, "", "Liquid", "ASSUMED",
      "Baffle open area fraction."),
    V("vPumpBarMax", P.vPumpBarMax, "bar", "Liquid", "VENDOR",
      "Diaphragm pump maximum pressure."),
    V("vPumpLmin", P.vPumpLmin, "L/min", "Liquid", "VENDOR", "Free flow."),
    V("vFilterMicron", P.vFilterMicron, "um", "Liquid", "VENDOR",
      "Inline filter rating."),
    V("vReliefSet", P.vReliefSet, "bar", "Liquid", "CALCULATED",
      "Relief / bypass setting."),
    V("vCheckCrack", P.vCheckCrack, "bar", "Liquid", "VENDOR",
      "Check-valve cracking pressure."),
    V("vHoseOD", P.vHoseOD, "mm", "Liquid", "VENDOR", "Hose outside diameter."),

    # ---- 7 GEAR / STRUCTURE ------------------------------------------------
    V("vGearGround", P.vGearGround, "mm", "Structure", "CALCULATED",
      "Skid underside relative to the airframe datum."),
    V("vGearTrack", P.vGearTrack, "mm", "Structure", "CALCULATED",
      "Skid centre-to-centre. Sets the 35.1 deg static tip-over half-angle."),
    V("vGearLegRootY", P.vGearLegRootY, "mm", "Structure", "CALCULATED",
      "Leg root offset. Moved outboard from 118 mm to clear the aft battery "
      "extraction corridor (CHECK 8)."),
    V("vSpineLen", P.vSpineLen, "mm", "Structure", "CALCULATED",
      "Spine rail length."),
    V("vSpineTrack", P.vSpineTrack, "mm", "Structure", "CALCULATED",
      "Rail centre-to-centre."),
    V("vHubD", P.vHubD, "mm", "Structure", "CALCULATED",
      "Manipulator hub diameter. Set by the J1 bearing seat and bolt "
      "pattern, NOT by stress (0.5 MPa against 500 MPa yield)."),
    V("vArmTubeOD", P.vArmTubeOD, "mm", "Structure", "CALCULATED",
      "Propulsion arm tube, root-moment and deflection sized."),
    V("vArmTubeID", P.vArmTubeID, "mm", "Structure", "CALCULATED",
      "Arm tube bore. Carries the motor phase leads."),
    V("vPanelT", P.vPanelT, "mm", "Structure", "ASSUMED",
      "Printed PA-CF service panel wall."),
    # ---- 8 DETAIL DIMENSIONS (referenced by the FeatureScript modules) -----
    V("vSpineRailH", P.vSpineRailH, "mm", "Detail", "CALCULATED",
      "Spine rail box height."),
    V("vSpineRailW", P.vSpineRailW, "mm", "Detail", "CALCULATED",
      "Spine rail box width."),
    V("vSpineRailT", P.vSpineRailT, "mm", "Detail", "CALCULATED",
      "CF box wall thickness."),
    V("vCrossStation", P.vCrossStation, "mm", "Detail", "CALCULATED",
      "Fore and aft cross members at x = +/- this."),
    V("vCrossH", P.vCrossH, "mm", "Detail", "CALCULATED", "Cross member height."),
    V("vCrossW", P.vCrossW, "mm", "Detail", "CALCULATED", "Cross member width."),
    V("vNodeSize", P.vNodeSize, "mm", "Detail", "CALCULATED",
      "CNC AL corner node cube dimension."),
    V("vHubX", P.vHubX, "mm", "Detail", "CALCULATED",
      "Manipulator hub station."),
    V("vHubZ", P.vHubZ, "mm", "Detail", "CALCULATED", "Manipulator hub height."),
    V("vHubH", P.vHubH, "mm", "Detail", "CALCULATED", "Hub section height."),
    V("vHubBoltPCD", P.vHubBoltPCD, "mm", "Detail", "CALCULATED",
      "Hub to arm-base bolt circle."),
    V("vHubBoltN", P.vHubBoltN, "", "Detail", "CALCULATED", "Hub bolts, M5."),
    V("vArmRootR", P.vArmRootR, "mm", "Detail", "CALCULATED",
      "Arm tube starts at this radius, inside the node clamp."),
    V("vArmClampL", P.vArmClampL, "mm", "Detail", "CALCULATED",
      "Split root clamp length."),
    V("vArmClampOD", P.vArmClampOD, "mm", "Detail", "CALCULATED",
      "Split root clamp outside diameter."),
    V("vGearLegX", P.vGearLegX, "mm", "Detail", "CALCULATED",
      "Landing gear leg station."),
    V("vGearLegOD", P.vGearLegOD, "mm", "Detail", "CALCULATED", "Leg tube OD."),
    V("vGearLegID", P.vGearLegID, "mm", "Detail", "CALCULATED", "Leg tube ID."),
    V("vGearSkidOD", P.vGearSkidOD, "mm", "Detail", "CALCULATED", "Skid OD."),
    V("vGearSkidLen", P.vGearSkidLen, "mm", "Detail", "CALCULATED",
      "Skid tube length."),
    V("vGearFootD", P.vGearFootD, "mm", "Detail", "CALCULATED",
      "Replaceable end foot diameter."),
    V("vGearDamperD", P.vGearDamperD, "mm", "Detail", "VENDOR",
      "Wire-rope isolator diameter. PLACEHOLDER ENVELOPE."),
    V("vGearDamperH", P.vGearDamperH, "mm", "Detail", "VENDOR",
      "Wire-rope isolator height. PLACEHOLDER ENVELOPE."),
    V("vJ1D", P.vJ1D, "mm", "Detail", "CALCULATED", "J1 housing diameter."),
    V("vJ1H", P.vJ1H, "mm", "Detail", "CALCULATED", "J1 housing height."),
    V("vJ2D", P.vJ2D, "mm", "Detail", "CALCULATED", "J2 housing diameter."),
    V("vJ2W", P.vJ2W, "mm", "Detail", "CALCULATED", "J2 housing width."),
    V("vJ3D", P.vJ3D, "mm", "Detail", "CALCULATED", "J3 housing diameter."),
    V("vJ3W", P.vJ3W, "mm", "Detail", "CALCULATED", "J3 housing width."),
    V("vJ4D", P.vJ4D, "mm", "Detail", "CALCULATED", "J4 housing diameter."),
    V("vJ4W", P.vJ4W, "mm", "Detail", "CALCULATED", "J4 housing width."),
    V("vJ5D", P.vJ5D, "mm", "Detail", "CALCULATED", "J5 housing diameter."),
    V("vJ5W", P.vJ5W, "mm", "Detail", "CALCULATED", "J5 housing width."),
    V("vJ6D", P.vJ6D, "mm", "Detail", "CALCULATED", "J6 housing diameter."),
    V("vJ6W", P.vJ6W, "mm", "Detail", "CALCULATED", "J6 housing width."),
    V("vHardStopA", P.vHardStopA, "deg", "Detail", "CALCULATED",
      "Angular width of a mechanical hard-stop lug."),
    V("vGripBodyD", P.vGripBodyD, "mm", "Detail", "ESTIMATED",
      "Gripper actuator housing diameter. PLACEHOLDER ENVELOPE."),
    V("vGripBodyL", P.vGripBodyL, "mm", "Detail", "ESTIMATED",
      "Gripper actuator housing length. PLACEHOLDER ENVELOPE."),
    V("vGripJawL", P.vGripJawL, "mm", "Detail", "DESIGN TARGET", "Jaw length."),
    V("vNozzleBodyD", P.vNozzleBodyD, "mm", "Detail", "ESTIMATED",
      "Liquid tool body diameter."),
    V("vNozzleBodyL", P.vNozzleBodyL, "mm", "Detail", "ESTIMATED",
      "Liquid tool body length."),
    V("vNozzleTipD", P.vNozzleTipD, "mm", "Detail", "DESIGN TARGET",
      "Replaceable nozzle orifice."),
    V("vNozzleTipL", P.vNozzleTipL, "mm", "Detail", "ESTIMATED",
      "Replaceable nozzle length."),
    V("vPumpW", P.vPumpW, "mm", "Detail", "ESTIMATED",
      "Pump body cross-section. PLACEHOLDER ENVELOPE."),
    V("vPumpL", P.vPumpL, "mm", "Detail", "ESTIMATED",
      "Pump body length. Bounded by the aft bay width between the two "
      "battery extraction corridors (CHECK 8)."),
    V("vFilterD", P.vFilterD, "mm", "Detail", "VENDOR", "Filter bowl diameter."),
    V("vFilterL", P.vFilterL, "mm", "Detail", "VENDOR", "Filter bowl length."),
    V("vPanelTopL", P.vPanelTopL, "mm", "Detail", "CALCULATED",
      "Top service panel length."),
    V("vPanelTopW", P.vPanelTopW, "mm", "Detail", "CALCULATED",
      "Top service panel width."),
    V("vPanelTopZ", P.vPanelTopZ, "mm", "Detail", "CALCULATED",
      "Top service panel plane."),
    V("vPanelBotL", P.vPanelBotL, "mm", "Detail", "CALCULATED",
      "Lower service panel length."),
    V("vPanelBotW", P.vPanelBotW, "mm", "Detail", "CALCULATED",
      "Lower service panel width."),
    V("vPanelBotZ", P.vPanelBotZ, "mm", "Detail", "CALCULATED",
      "Lower service panel plane."),
    V("vPanelSideL", P.vPanelSideL, "mm", "Detail", "CALCULATED",
      "Side panel length. Shortened to 220 mm so it sits between the corner "
      "nodes and clears the 45 deg arm tube (CHECK 11)."),
    V("vPanelSideH", P.vPanelSideH, "mm", "Detail", "CALCULATED",
      "Side panel height."),
    V("vFastenerQTurn", P.vFastenerQTurn, "", "Detail", "CALCULATED",
      "Quarter-turn fasteners per panel."),
]



def emit_fs(path):
    lines = [
        "FeatureScript 2278;",
        'import(path : "onshape/std/geometry.fs", version : "2278.0");',
        "",
        "/**",
        " * AVIAN REV A -- 01_VARIABLES",
        " *",
        " * Paste into a VARIABLE STUDIO named 01_VARIABLES.",
        " * Every Part Studio in the document must reference this studio",
        " * (Insert > Variable Studio) before its features will resolve.",
        " *",
        " * Confidence tags: VENDOR / CALCULATED / ESTIMATED / ASSUMED /",
        " * DESIGN TARGET. Nothing here is tagged VERIFIED -- no physical",
        " * test or FEA has been run on this revision.",
        " */",
        "",
    ]
    cat = None
    for r in ROWS:
        if r["cat"] != cat:
            cat = r["cat"]
            lines.append("")
            lines.append("// " + "=" * 70)
            lines.append(f"// {cat.upper()}")
            lines.append("// " + "=" * 70)
        u = r["unit"]
        if u == "mm":
            val = f'{r["value"]} * millimeter'
        elif u == "deg":
            val = f'{r["value"]} * degree'
        elif u == "kg":
            val = f'{r["value"]} * kilogram'
        elif u == "":
            val = f'{r["value"]}'
        else:
            val = f'{r["value"]}   // {u}'
        lines.append(f'// [{r["conf"]}] {r["desc"]}')
        lines.append(f'export const {r["name"]} = {val};')
    open(path, "w").write("\n".join(lines) + "\n")
    return len(ROWS)


def emit_table(path):
    out = ["# AVIAN REV A -- Master Parameter Table", "",
           "Generated from `params_b.py`, the single source of truth for the "
           "model. Every dimension in the FeatureScript modules resolves "
           "against these.", "",
           "Confidence is one of **VENDOR** (manufacturer data sheet), "
           "**CALCULATED** (derived from a documented analysis), "
           "**ESTIMATED** (engineering judgement, no analysis), "
           "**ASSUMED** (process or convention), **DESIGN TARGET** (a "
           "requirement, not yet demonstrated). Nothing is **VERIFIED**: no "
           "physical test and no FEA has been run on REV A.", ""]
    cat = None
    for r in ROWS:
        if r["cat"] != cat:
            cat = r["cat"]
            out += ["", f"## {cat}", "",
                    "| Variable | Value | Unit | Confidence | Basis |",
                    "|---|---:|---|---|---|"]
        v = r["value"]
        vs = f"{v:g}" if isinstance(v, float) else str(v)
        out.append(f'| `#{r["name"]}` | {vs} | {r["unit"] or "-"} | '
                   f'{r["conf"]} | {r["desc"]} |')
    open(path, "w").write("\n".join(out) + "\n")


if __name__ == "__main__":
    n = emit_fs("pkg/01_Onshape_FeatureScript/AVIAN_Variables.txt")
    emit_table("pkg/02_Onshape_Build_Guide/VARIABLE_TABLE.md")
    print(f"{n} variables emitted")
