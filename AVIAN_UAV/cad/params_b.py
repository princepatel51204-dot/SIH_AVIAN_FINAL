"""
AVIAN  --  REV B FROZEN PARAMETER SET
=====================================
This file is the single source of truth and maps 1:1 onto the Onshape
Variable Studio.  Every name here becomes an Onshape variable of the same
name (see the build kit).  Nothing downstream hard-codes a dimension.

base_link : X forward, Y left, Z up, origin at the centre of the four arm-root
            axes in the rotor mid-plane.  Units mm / deg / kg.
"""
PROJECT, VARIANT, REV = "AVIAN", "R1", "B"

# ---------------------------------------------------------------------------
# GROUP 1  AIRCRAFT ENVELOPE                          (Onshape: #vAircraft)
# ---------------------------------------------------------------------------
vDiagonal        = 1150.0      # motor-to-motor
vArmCount        = 4
vArmAngle0       = 45.0        # X layout, no arm on the forward centreline
vRMotor          = vDiagonal / 2.0
ARM_ANGLES       = [vArmAngle0 + 90.0 * i for i in range(vArmCount)]

vPropDia         = 711.2       # T-Motor G28x9.2  (28 in)
vPropPitch       = 233.7       # 9.2 in
vPropR           = vPropDia / 2.0
vCoaxSep         = 124.0       # upper-to-lower rotor plane
vPropBlades      = 2
vTipGapMin       = 80.0        # design rule, tip-to-tip

# ---------------------------------------------------------------------------
# GROUP 2  CENTRAL SPINE                              (Onshape: #vSpine)
# ---------------------------------------------------------------------------
vSpineLen        = 510.0       # rail length, x -255 .. +255
vSpineTrack      = 210.0       # rail centre-to-centre in Y
vSpineRailH      = 55.0
vSpineRailW      = 26.0
vSpineRailT      = 2.5         # CF box wall
vCrossStation    = 175.0       # cross members at x = +/- this
vCrossH          = 40.0
vCrossW          = 24.0
vNodeSize        = 58.0        # CNC AL corner node cube-ish
vHubX            = 45.0        # manipulator hub station
vHubZ            = -48.0
vHubD            = 118.0
vHubH            = 62.0
vHubBoltPCD      = 88.0
vHubBoltN        = 8

# ---------------------------------------------------------------------------
# GROUP 3  PROPULSION ARM                             (Onshape: #vArm)
# ---------------------------------------------------------------------------
vArmTubeOD       = 25.0        # root-moment + deflection sized
vArmTubeID       = 21.0
vArmRootR        = 210.0       # tube starts here (inside the node clamp)
vArmClampL       = 72.0
vArmClampOD      = 44.0
vESCr            = 320.0       # ESC station along the arm
vConduitD        = 11.0

# ---------------------------------------------------------------------------
# GROUP 4  MOTOR + ESC  (T-Motor X-U8II kit)          (Onshape: #vMotor)
# ---------------------------------------------------------------------------
# PLACEHOLDER ENVELOPE -- vendor CAD not available. Overall dimensions are
# taken from published outline data; internal detail is representative only.
vMotorBellD      = 96.6
vMotorBellH      = 26.0
vMotorBaseD      = 96.6
vMotorBaseH      = 12.5
vMotorShaftD     = 8.0
vMotorBoltPCD    = 25.0
vMotorBoltN      = 4
vMotorMass       = 0.240
vMotorKV         = 100
vMotorCells      = 12
vPropMass        = 0.130
vESCL, vESCW, vESCH = 85.0, 46.0, 22.0
vESCMass         = 0.082
vESCRating       = 60.0
vCoaxMountH      = 96.0        # spans upper and lower motor faces
vThrustPerArm    = 13.672      # kgf, VENDOR, 100 % throttle, coaxial pair
vThrustPerRotor  = 6.80        # kgf, VENDOR, isolated
vCurrentPerArm   = 55.32       # A at 44.4 V

# ---------------------------------------------------------------------------
# GROUP 5  BATTERY                                    (Onshape: #vBattery)
# ---------------------------------------------------------------------------
vBattCells       = 12
vBattAh          = 12.0        # per cartridge
vBattPacks       = 2
vBattWh          = 12 * 3.7 * vBattAh * vBattPacks
# Cartridge proportions set by CHECK 7 (static interference): at 86 mm wide
# on a 92 mm centreline the packs overlapped the 118 mm manipulator hub.
# Same cell count and volume, re-proportioned long-and-narrow.
vBattL, vBattW, vBattH = 240.0, 70.0, 74.0
vBattY           = 100.0       # +/- cartridge centreline
vBattZ           = -102.0
vBattMassEach    = 3.60
vRailTravel      = 40.0        # 3-position indexed mount, +/- this
vRailPositions   = 3
vRailT           = 4.0

# ---------------------------------------------------------------------------
# GROUP 6  AVIONICS BAY                               (Onshape: #vAvionics)
# ---------------------------------------------------------------------------
vBayL, vBayW, vBayH = 230.0, 190.0, 74.0
vBayX, vBayZ     = -40.0, 78.0
vFCL, vFCW, vFCH = 85.0, 55.0, 26.0
vFCPos           = (-40.0, -62.0, 78.0)
vCCL, vCCW, vCCH = 110.0, 82.0, 32.0     # companion computer
vCCPos           = (-40.0, 62.0, 64.0)
vPDBL, vPDBW, vPDBH = 130.0, 92.0, 16.0
# Turned 90 deg about Z: the clear run between the aft cross member
# (x = -163) and the manipulator base flange (x = -33) is 130 mm, and
# the PDB is 130 mm long. Across the spine it fits with margin.
vPDBPos          = (-100.0, 0.0, -6.0)

# ---------------------------------------------------------------------------
# GROUP 7  SENSORS                                    (Onshape: #vSensor)
# ---------------------------------------------------------------------------
vLidarD, vLidarH = 85.0, 76.0
vLidarPos        = (140.0, 0.0, 212.0)     # CHECK 5a: moved forward of the
                                           # top panel footprint (x <= 85)
vLidarMastD      = 34.0
vGimbalPos       = (230.0, 96.0, -175.0)   # CHECK 3: below the lower
                                           # of the lower rotor disc
vGimbalBoomL     = 60.0
vGimbalBoomOD    = 22.0
vDepthL, vDepthW, vDepthH = 92.0, 25.0, 25.0
vDepthPos        = (240.0, 0.0, 16.0)
vDepthTilt       = -14.0
vGnssD, vGnssH   = 72.0, 18.0
# CHECK 5a: outboard of the top service panel footprint (|y| <= 105)
vGnssPos         = [(-150.0, 150.0, 200.0), (-150.0, -150.0, 200.0)]
vGnssMastD       = 20.0
vRangePos        = (-140.0, -72.0, -156.0)
vFlowPos         = (-140.0, 72.0, -156.0)
vLampPos         = (230.0, -96.0, -175.0)  # rotor disc at z = -99.5

# ---------------------------------------------------------------------------
# GROUP 8  LANDING GEAR                               (Onshape: #vGear)
# ---------------------------------------------------------------------------
vGearGround      = -530.0      # skid underside
vGearTrack       = 680.0       # skid centre-to-centre
vGearLegX        = 150.0
vGearLegRootY    = 168.0     # CHECK 4a: root moved outboard to
                             # clear the aft battery corridor
vGearLegOD       = 22.0
vGearLegID       = 18.0
vGearSkidOD      = 26.0
vGearSkidID      = 22.0
vGearSkidLen     = 620.0
vGearFootD       = 44.0
vGearFootH       = 20.0
vGearDamperD     = 30.0
vGearDamperH     = 46.0

# ---------------------------------------------------------------------------
# GROUP 9  MANIPULATOR                                (Onshape: #vArm6)
# ---------------------------------------------------------------------------
vArmBase         = (45.0, 0.0, -95.0)
vL0, vL1, vL2, vL3, vL4, vL5 = 95.0, 330.0, 300.0, 85.0, 80.0, 60.0
vToolOffset      = 45.0        # tool0 flange -> tool point
vReach           = vL1 + vL2 + vL3 + vL4 + vL5 + vToolOffset   # 900

# joint: name, offset in parent, axis in parent, limits, torque Nm, speed deg/s
JOINTS = [
    ("J1", (0, 0, 0),     (0, 0, 1),  (-180.0, 180.0), 24.0, 90.0,  "base yaw"),
    ("J2", (0, 0, -vL0),  (0, -1, 0), (-115.0, 115.0), 40.0, 60.0,  "shoulder pitch"),
    ("J3", (0, 0, -vL1),  (0, -1, 0), (-160.0, 160.0), 25.0, 70.0,  "elbow pitch"),
    ("J4", (0, 0, -vL2),  (0, 0, -1), (-180.0, 180.0),  8.0, 150.0, "wrist roll"),
    ("J5", (0, 0, -vL3),  (0, -1, 0), (-120.0, 120.0), 10.0, 120.0, "wrist pitch"),
    ("J6", (0, 0, -vL4),  (1, 0, 0),  (-180.0, 180.0),  8.0, 150.0, "wrist yaw"),
]
vJ1D, vJ1H       = 96.0, 78.0
vJ2D, vJ2W       = 82.0, 96.0
vJ3D, vJ3W       = 70.0, 82.0
vJ4D, vJ4W       = 56.0, 64.0
vJ5D, vJ5W       = 50.0, 56.0
vJ6D, vJ6W       = 46.0, 48.0
vLinkTube1OD, vLinkTube1ID = 44.0, 38.0
vLinkTube2OD, vLinkTube2ID = 38.0, 32.5
vFlangeD, vFlangeT = 46.0, 7.0
vHardStopA       = 8.0         # angular width of a hard-stop lug

# ---------------------------------------------------------------------------
# GROUP 10  TOOL INTERFACE + TOOLS                    (Onshape: #vTool)
# ---------------------------------------------------------------------------
vTCMasterD, vTCMasterH = 52.0, 22.0
vTCToolD, vTCToolH     = 52.0, 16.0
vTCPinN, vTCPinD       = 3, 9.0
vTCElecWays            = 12
vTCFluidPorts          = 1
vFTSD, vFTSH           = 50.0, 12.0
vNozzleBodyD, vNozzleBodyL = 22.0, 38.0
vNozzleTipD, vNozzleTipL   = 2.4, 26.0
vGripStroke      = 62.0        # jaw opening
vGripJawL        = 74.0
vGripBodyD       = 54.0
vGripBodyL       = 68.0
vGripForce       = 90.0        # N, ESTIMATED

# ---------------------------------------------------------------------------
# GROUP 11  LIQUID SERVICE                            (Onshape: #vFluid)
# ---------------------------------------------------------------------------
vTankL, vTankW, vTankH = 130.0, 88.0, 118.0
vTankWall        = 2.4
vTankPos         = (-101.0, 0.0, -88.0)
# The tank is not a loose bottle: it sits inside a removable SERVICE-FLUID
# CARTRIDGE that also carries the pump, filter, valves and sensor, and
# presents ONE dry-break fluid port + ONE electrical connector to the
# aircraft. Envelope below; see CHECK 4a/5b/5e.
vCartL, vCartW, vCartH = 150.0, 96.0, 118.0
vCartPos         = (-111.0, 0.0, -88.0)
vCartBayL        = 26.0        # end frame depth
vTankVol         = 1.15        # L geometric
vTankFill        = 1.00        # L working
vBaffleN         = 3
vBaffleOpen      = 0.65
# Pump body length is bounded by the aft equipment bay width: the
# battery extraction corridors close in at |y| = 62 mm, and the
# pump + its motor must fit between them (CHECK 8).
vPumpL, vPumpW, vPumpH = 80.0, 62.0, 58.0
vPumpPos         = (-244.0, 0.0, -78.0)  # airframe-mounted aft equipment
                                          # bay; only the TANK is removable.
                                          # y offset puts the pump motor
                                          # inboard, out of the rotor disc
vPumpBarMax      = 6.0
vPumpLmin        = 1.2
vFilterD, vFilterL = 34.0, 62.0
vFilterMicron    = 40.0
vCheckCrack      = 0.2         # bar
vReliefSet       = 6.5         # bar
vHoseOD, vHoseID = 8.0, 4.5

# ---------------------------------------------------------------------------
# GROUP 12  PANELS / SERVICE                          (Onshape: #vPanel)
# ---------------------------------------------------------------------------
vPanelT          = 2.4
vPanelTopL, vPanelTopW = 250.0, 210.0
vPanelTopZ       = 116.0
vPanelBotL, vPanelBotW = 300.0, 240.0
vPanelBotZ       = -152.0
# CHECK 5c: panel shortened to sit BETWEEN the corner nodes so it
# lifts straight outboard (nodes span |x| 146..223)
vPanelSideL, vPanelSideH = 220.0, 120.0
vFastenerQTurn   = 6

# ---------------------------------------------------------------------------
# GROUP 13  CONFIGURATIONS  (joint values J1..J6, deg)
# ---------------------------------------------------------------------------
CFG = {
    "01_FLIGHT":       [0.0, 55.0, -143.0, 0.0, -30.0, 0.0],
    "02_INSPECTION":   [0.0, 78.0, -108.0, 0.0, 42.0, 0.0],
    "03_MANIPULATION": [0.0, 55.0, 25.0, 0.0, 10.0, 0.0],
    "04_REPAIR":       [0.0, 62.0, 8.0, 0.0, 20.0, 0.0],
    "05_LIQUID":       [0.0, 48.0, 34.0, 0.0, 8.0, 0.0],
    "06_TRANSPORT":    [0.0, 55.0, -152.0, 0.0, 30.0, 0.0],
    "07_MAINTENANCE":  [0.0, 100.0, -35.0, 0.0, 0.0, 0.0],
}
CFG_TOOL = {
    "01_FLIGHT": "none", "02_INSPECTION": "none", "03_MANIPULATION": "gripper",
    "04_REPAIR": "gripper", "05_LIQUID": "nozzle", "06_TRANSPORT": "none",
    "07_MAINTENANCE": "none",
}
CFG_FLUID = {"01_FLIGHT": 1.0, "02_INSPECTION": 1.0, "03_MANIPULATION": 1.0,
             "04_REPAIR": 1.0, "05_LIQUID": 1.0, "06_TRANSPORT": 0.0,
             "07_MAINTENANCE": 0.0}

# ---------------------------------------------------------------------------
# MATERIALS + APPEARANCE  (industrial palette, blue as identification only)
# ---------------------------------------------------------------------------
DENSITY = {"CFRP": 1600.0, "AL7075": 2810.0, "AL6061": 2700.0, "POLYMER": 1200.0,
           "STEEL": 7850.0, "TPU": 1200.0, "FLUID": 1000.0, "GENERIC": 1500.0}
# Industrial palette. Values are raised from a first pass that rendered as one
# undifferentiated black mass: matte black stays the primary, but the tonal
# steps between carbon, graphite and dark grey are widened so the structure
# actually reads in a render. Blue is identification and service interfaces
# only -- it is never used as a general part colour.
COLOR = {
    "CF_MATTE":   (0.150, 0.158, 0.172),   # matte carbon, structural plate
    "CF_TUBE":    (0.115, 0.122, 0.134),   # carbon tube, one step darker
    "GRAPHITE":   (0.255, 0.268, 0.288),   # printed PA-CF covers
    "DARKGREY":   (0.360, 0.375, 0.398),   # electronics housings
    "AL_ANOD":    (0.505, 0.525, 0.548),   # dark anodised aluminium
    "AL_BRIGHT":  (0.720, 0.738, 0.760),   # satin machined aluminium
    "BLUE_ID":    (0.110, 0.400, 0.640),   # identification / service interface
    "BLUE_DEEP":  (0.070, 0.270, 0.450),
    "MOTOR":      (0.200, 0.208, 0.222),   # motor bell
    "PROP":       (0.088, 0.092, 0.100),   # CF propeller
    "PCB":        (0.070, 0.180, 0.140),   # board green
    "BATT":       (0.175, 0.188, 0.212),   # pack shell
    "SENSOR":     (0.205, 0.215, 0.232),
    "FLUID":      (0.330, 0.480, 0.580),   # fluid volume representation
    "TPU_BLACK":  (0.125, 0.130, 0.140),   # hose
    "WARN":       (0.660, 0.450, 0.095),   # e-stop / warning only
}
TESS_TOL, TESS_ANG = 0.30, 0.28
