"""AVIAN Smart Infrastructure Inspection City -- central parameter set.

EVERY dimension in the environment resolves against this file. Nothing
downstream hard-codes a number that belongs here.

COORDINATE SYSTEM
-----------------
  Units      1 Blender unit = 1 metre, Z-up, right-handed.
  Origin     (0, 0, 0) sits on the ground plane at the START of the bridge
             deck centreline -- the first expansion joint of the south
             approach. Chosen so every chainage below reads directly as a
             station in metres.
  +X         direction of travel along the corridor (south -> north).
             The bridge runs from x = 0 to x = BRIDGE_LENGTH.
  +Y         left of travel (west). The corridor is symmetric about y = 0.
  +Z         up. Ground datum is z = 0.

  The river flows along Y, crossing the corridor near mid-span.

  Precision note: 4.5 km fits comfortably inside float32 mesh precision
  (which degrades past roughly 10 km from origin), so no scene offset or
  floating-origin scheme is needed.
"""
from __future__ import annotations
import math

# ===========================================================================
# 1. CORRIDOR
# ===========================================================================
BRIDGE_LENGTH = 4500.0          # m, total corridor from x=0 to x=4500
DECK_WIDTH = 24.0               # m, out-to-out incl. parapets
CARRIAGEWAY_WIDTH = 10.5        # m, each direction (3 lanes @ 3.5 m)
MEDIAN_WIDTH = 1.2              # m, central barrier
SHOULDER_WIDTH = 0.9            # m
PARAPET_HEIGHT = 1.10           # m above deck top
PARAPET_THICK = 0.40            # m

DECK_SLAB_T = 0.25              # m, structural deck slab
WEARING_COURSE_T = 0.075        # m, asphalt wearing course
GIRDER_COUNT = 6                # longitudinal I-girders
GIRDER_SPACING = 4.0            # m, centre to centre
GIRDER_DEPTH_MIN = 1.60         # m, shortest approach spans
GIRDER_DEPTH_MAX = 4.20         # m, main navigation span
GIRDER_SPAN_DEPTH_RATIO = 22.0  # span / structural depth


def girder_depth_for_span(span: float) -> float:
    """Structural depth from span, clamped to buildable sections.

    A constant depth over a 4.5 km corridor whose spans run 25-90 m is not
    credible: at 90 m a 2.2 m girder is a span/depth of 41, roughly double
    anything that gets built. Depth follows span at a ratio of 22, which is
    conventional for post-tensioned concrete.
    """
    return max(GIRDER_DEPTH_MIN,
               min(GIRDER_DEPTH_MAX, span / GIRDER_SPAN_DEPTH_RATIO))


GIRDER_DEPTH = 2.00             # m, nominal -- used only where span is unknown
GIRDER_TOP_FLANGE_W = 0.80      # m
GIRDER_BOT_FLANGE_W = 0.70      # m
GIRDER_FLANGE_T = 0.22          # m
GIRDER_WEB_T = 0.20             # m
DIAPHRAGM_T = 0.30              # m, transverse cross beams
DIAPHRAGM_DEPTH = 1.50          # m

# ---------------------------------------------------------------------------
# 2. SECTIONS  (chainage bands along +X)
# ---------------------------------------------------------------------------
# (name, x_start, x_end, span_m, deck_top_z, kind)
#   span_m     typical pier spacing in that band
#   deck_top_z road surface level at the START of the band; the longitudinal
#              profile interpolates smoothly between band ends
SECTIONS = [
    ("SECTION_01_APPROACH_SOUTH", 0.0, 350.0, 25.0, "approach"),
    ("SECTION_02_URBAN_VIADUCT", 350.0, 1400.0, 30.0, "viaduct"),
    ("SECTION_03_TRANSITION", 1400.0, 1700.0, 35.0, "transition"),
    ("SECTION_04_RIVER_CROSSING", 1700.0, 2500.0, 60.0, "river"),
    ("SECTION_05_RIVERSIDE_VIADUCT", 2500.0, 3200.0, 35.0, "viaduct"),
    ("SECTION_06_URBAN_CONTINUATION", 3200.0, 4150.0, 30.0, "viaduct"),
    ("SECTION_07_APPROACH_NORTH", 4150.0, 4500.0, 25.0, "approach"),
]

# Vertical alignment control points (chainage, road surface level).
# Kept separate from SECTIONS because the crest must sit over the navigation
# channel, not at a section boundary. Grades stay under 3 %.
PROFILE = [
    (0.0, 4.0), (350.0, 13.5), (1400.0, 17.0), (1700.0, 23.0),
    (2100.0, 27.0), (2500.0, 21.5), (3200.0, 15.0), (4150.0, 5.0),
    (4500.0, 2.5),
]

# Navigation channel: no pier may stand inside it.
NAV_CHANNEL_HALF = 45.0         # m either side of the river centreline
MAIN_SPAN = 2 * NAV_CHANNEL_HALF    # 90 m clear main span

# ---------------------------------------------------------------------------
# 3. PIERS
# ---------------------------------------------------------------------------
PIER_COL_D = 2.40               # m, circular column diameter (land)
PIER_COL_D_RIVER = 3.20         # m, larger in the river
PIER_COLS_PER_BENT = 2          # twin-column bents
PIER_COL_SPACING = 9.0          # m between the two columns of a bent
PIER_CAP_L = 15.0               # m, transverse pier cap length
PIER_CAP_W = 2.60               # m
PIER_CAP_H = 1.80               # m
PIER_CAP_TAPER = 0.55           # end taper as a fraction of cap height
PIER_FOOTING_L = 8.0            # m
PIER_FOOTING_W = 6.0
PIER_FOOTING_H = 1.6
BEARING_H = 0.35                # m, elastomeric bearing block
BEARING_L = 0.70
BEARING_W = 0.60

# ---------------------------------------------------------------------------
# 4. EXPANSION JOINTS AND DRAINAGE
# ---------------------------------------------------------------------------
JOINT_EVERY_N_SPANS = 4         # continuous over 4 spans, then a joint
JOINT_GAP = 0.08                # m, visible gap at the joint
DRAIN_SPACING = 20.0            # m, deck drainage outlets
DRAIN_PIPE_D = 0.20             # m
SERVICE_DUCT_D = 0.30           # m, utility duct under the cantilever

# ---------------------------------------------------------------------------
# 5. RESEARCH ZONE  (the high-detail inspection region)
# ---------------------------------------------------------------------------
# Placed to straddle the transition AND the river crossing, so it contains
# land piers, tall river piers, deep water, shallow water and both open and
# confined underside geometry.
RESEARCH_X0 = 1650.0
RESEARCH_X1 = 2550.0
RESEARCH_LENGTH = RESEARCH_X1 - RESEARCH_X0     # 900 m

# LOD bands, measured as distance in X outside the research zone
LOD_MED_MARGIN = 500.0          # m either side of the research zone
DETAIL_HIGH = "HIGH"
DETAIL_MED = "MED"
DETAIL_LOW = "LOW"


def detail_at(x: float) -> str:
    """LOD band for a chainage. Sensors only ever see HIGH in this phase."""
    if RESEARCH_X0 <= x <= RESEARCH_X1:
        return DETAIL_HIGH
    if (RESEARCH_X0 - LOD_MED_MARGIN) <= x <= (RESEARCH_X1 + LOD_MED_MARGIN):
        return DETAIL_MED
    return DETAIL_LOW


# ---------------------------------------------------------------------------
# 6. INSPECTION SECTORS  (six, one notional working area per UAV)
# ---------------------------------------------------------------------------
# Six sectors across the 900 m research zone: 150 m each. Deliberately sized
# so six aircraft can work simultaneously without sharing a corridor.
SECTOR_COUNT = 6
SECTOR_NAMES = ["SECTOR_A", "SECTOR_B", "SECTOR_C",
                "SECTOR_D", "SECTOR_E", "SECTOR_F"]
SECTOR_LENGTH = RESEARCH_LENGTH / SECTOR_COUNT   # 150 m


def sector_at(x: float):
    """Sector name for a chainage, or None outside the research zone."""
    if not (RESEARCH_X0 <= x <= RESEARCH_X1):
        return None
    i = int((x - RESEARCH_X0) / SECTOR_LENGTH)
    return SECTOR_NAMES[min(i, SECTOR_COUNT - 1)]


def section_at(x: float):
    for nm, x0, x1, span, kind in SECTIONS:
        if x0 <= x <= x1:
            return nm
    return SECTIONS[-1][0]


def section_kind_at(x: float):
    for nm, x0, x1, span, kind in SECTIONS:
        if x0 <= x <= x1:
            return kind
    return SECTIONS[-1][4]


# ---------------------------------------------------------------------------
# 7. RIVER
# ---------------------------------------------------------------------------
RIVER_CENTRE_X = 2100.0         # chainage of the river centreline
RIVER_WIDTH = 620.0             # m, bank to bank
RIVER_WATER_Z = -2.0            # m, water surface below ground datum
RIVER_DEPTH = 9.0               # m, max depth below the water surface
RIVER_BANK_SLOPE = 14.0         # m of run per m of rise on the banks
RIVER_LENGTH = 3000.0           # m modelled along Y
RIVER_ROCK_COUNT = 260
RIVER_VEG_COUNT = 900

# ---------------------------------------------------------------------------
# 8. TERRAIN
# ---------------------------------------------------------------------------
GROUND_X0, GROUND_X1 = -600.0, 5100.0
GROUND_Y0, GROUND_Y1 = -1500.0, 1500.0
GROUND_RES_X = 190              # terrain grid resolution
GROUND_RES_Y = 100
FAR_GROUND_Z = -1.2             # m, level of the distant ground plane
FAR_GROUND_PAD = 90000.0        # m of low-detail ground beyond the detailed
                                # terrain, so an aerial camera sees a horizon
                                # rather than the void below the sky model

# ---------------------------------------------------------------------------
# 9. CITY
# ---------------------------------------------------------------------------
CITY_DENSITY = 0.78             # 0..1, fraction of candidate plots built on
CITY_BLOCK = 92.0              # m, urban block pitch
CITY_ROAD_W = 14.0              # m, urban street width
CITY_SETBACK_Y = 70.0           # m, keep clear either side of the corridor
CITY_BAND_Y = 1150.0            # m, city extends to this |y|
CITY_MIDRISE_H = (12.0, 34.0)   # m, height range
CITY_HIGHRISE_H = (45.0, 110.0)
CITY_HIGHRISE_FRACTION = 0.13   # of built plots, biased to the far field
CITY_TREE_COUNT = 2400
CITY_STREETLIGHT_SPACING = 32.0
VEHICLE_COUNT_BRIDGE = 90
VEHICLE_COUNT_CITY = 260

# ---------------------------------------------------------------------------
# 10. DAMAGE
# ---------------------------------------------------------------------------
DAMAGE_DENSITY = 1.0            # global multiplier on all defect counts

# Target defect counts inside the research zone, by type.
# Deliberately weighted: most of the structure is healthy, severe damage is
# rare, and a small number of cases sit outside any plausible repair envelope.
DEFECT_TARGETS = {
    "CRACK_HAIRLINE": 46,
    "CRACK_LONGITUDINAL": 22,
    "CRACK_TRANSVERSE": 20,
    "CRACK_DIAGONAL": 16,
    "CRACK_NETWORK": 12,
    "SPALL": 18,
    "DELAMINATION": 10,
    "REBAR_EXPOSED": 8,
    "CORROSION_STAIN": 20,
    "JOINT_DETERIORATION": 9,
    "REPAIR_PATCH": 11,
}

# Severity distribution. LEVEL 0 is healthy surface and is not an object.
SEVERITY_WEIGHTS = {1: 0.44, 2: 0.30, 3: 0.18, 4: 0.08}

# A defect is outside the autonomous repair envelope if any of these hold.
# These become the escalation / rejection cases in later research.
REPAIR_ENVELOPE = {
    "max_severity": 3,          # severity 4 is never auto-repairable
    "max_area_m2": 0.45,        # larger than this needs scaffolding
    "max_crack_width_mm": 3.0,  # wider than this is structural, not surface
    "max_crack_length_m": 2.2,  # longer than this is not a spot repair
    "rebar_exposed_blocks": True,   # exposed reinforcement always escalates
}

# Geometry vs shader split (the hybrid model).
GEOMETRIC_DEFECTS = {"SPALL", "DELAMINATION", "REBAR_EXPOSED", "REPAIR_PATCH"}
SHADER_DEFECTS = {"CRACK_HAIRLINE", "CRACK_LONGITUDINAL", "CRACK_TRANSVERSE",
                  "CRACK_DIAGONAL", "CRACK_NETWORK", "CORROSION_STAIN",
                  "JOINT_DETERIORATION"}

# Crack width bands in mm, by severity -- used for the repairable test and
# written into the ground truth.
CRACK_WIDTH_MM = {1: (0.05, 0.20), 2: (0.20, 0.80),
                  3: (0.80, 2.50), 4: (2.50, 8.00)}

# Host surfaces defects may be placed on, with a relative weight. Weighted
# toward the surfaces a UAV would actually have to work to see.
DEFECT_HOSTS = {
    "DECK_UNDERSIDE": 0.26,
    "GIRDER_WEB": 0.22,
    "GIRDER_BOTTOM_FLANGE": 0.10,
    "DIAPHRAGM": 0.08,
    "PIER_COLUMN": 0.16,
    "PIER_CAP": 0.10,
    "PARAPET": 0.05,
    "BEARING_SEAT": 0.03,
}

# ---------------------------------------------------------------------------
# 11. UAV AIRSPACE
# ---------------------------------------------------------------------------
AIRSPACE_SAFE_Z = (55.0, 95.0)          # transit, well clear of the deck
AIRSPACE_INSPECTION_OFFSET = 12.0       # m standoff around the superstructure
AIRSPACE_UNDER_CLEAR = 3.0              # m clearance below the girders
AIRSPACE_RESTRICTED_PAD = 4.0           # m no-fly shell around structure
AIRSPACE_RIVER_Z = (2.0, 22.0)          # over-water working band

# ---------------------------------------------------------------------------
# 12. LIGHTING
# ---------------------------------------------------------------------------
# The sun disc is disabled in the sky texture and supplied by an explicit SUN
# lamp instead. Leaving both on double-counts the direct component and blows
# the exposure out -- which is exactly what happened on the first pass here.
# The lamp also gives direct control of the shadow softness, which matters for
# under-deck contrast.
SUN_ELEVATION_DEG = 52.0        # mid-morning
SUN_ROTATION_DEG = 145.0
SUN_STRENGTH = 6.0              # W/m^2 in Blender's units for the lamp
SUN_ANGLE_DEG = 0.526           # true solar disc -> correct penumbra
# Sky is left at its physical radiance (1.0) rather than dimmed. Dimming it
# to control exposure also darkens the SKY ITSELF, which at altitude rendered
# the horizon black. Exposure is handled where a real camera handles it --
# at the film -- via RENDER_EXPOSURE below.
SKY_BACKGROUND_STRENGTH = 1.0
RENDER_EXPOSURE = -1.1          # EV, applied in the view transform
SKY_DUST = 0.9                  # slight haze; Indian metro air
SKY_OZONE = 1.6
HAZE_COLOR = (0.55, 0.56, 0.58)   # ground haze below the horizon
HAZE_BLEND_DEG = 3.0              # angular softness of the horizon blend

# ---------------------------------------------------------------------------
# 13. RENDER
# ---------------------------------------------------------------------------
RENDER_W, RENDER_H = 1920, 1080
RENDER_SAMPLES = 64
RENDER_SAMPLES_CLOSEUP = 96
SEED = 20260827                 # one seed drives every random placement


# ---------------------------------------------------------------------------
# 14. DERIVED PROFILE
# ---------------------------------------------------------------------------
def deck_top_z(x: float) -> float:
    """Longitudinal road profile.

    Smoothstep between section start levels rather than a linear ramp, so the
    vertical curve has continuous slope at each section boundary the way a real
    alignment does.
    """
    pts = PROFILE
    x = max(pts[0][0], min(pts[-1][0], x))
    for i in range(len(pts) - 1):
        x0, z0 = pts[i]
        x1, z1 = pts[i + 1]
        if x0 <= x <= x1:
            t = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
            t = t * t * (3.0 - 2.0 * t)
            return z0 + (z1 - z0) * t
    return pts[-1][1]


def span_at(x: float) -> float:
    """Length of the span containing this chainage."""
    ps = pier_stations()
    for i in range(len(ps) - 1):
        if ps[i][0] <= x <= ps[i + 1][0]:
            return ps[i + 1][0] - ps[i][0]
    return SECTIONS[0][3]


def soffit_z(x: float) -> float:
    """Underside of the girders -- the primary inspection surface."""
    return (deck_top_z(x) - WEARING_COURSE_T - DECK_SLAB_T
            - girder_depth_for_span(span_at(x)))


def ground_z(x: float, y: float) -> float:
    """Terrain height, including the river channel."""
    d = abs(x - RIVER_CENTRE_X)
    half = RIVER_WIDTH / 2.0
    if d < half:
        t = d / half
        return RIVER_WATER_Z - RIVER_DEPTH * (1.0 - t * t) * 0.92
    run = RIVER_BANK_SLOPE * abs(RIVER_WATER_Z)
    if d < half + run:
        t = (d - half) / run
        return RIVER_WATER_Z + (0.0 - RIVER_WATER_Z) * (t * t * (3 - 2 * t))
    return 0.0


_PIER_CACHE = None


def pier_stations():
    """Every pier chainage on the corridor.

    Piers are laid out at the section spacing everywhere except across the
    navigation channel, where the two main-span piers are placed exactly at
    +/-NAV_CHANNEL_HALF and the approach spans on each side are redistributed
    to close the gap evenly. Without that redistribution the section spacing
    leaves a ragged part-span against the channel, which no bridge is built
    with.
    """
    global _PIER_CACHE
    if _PIER_CACHE is not None:
        return _PIER_CACHE

    out = []
    for nm, x0, x1, span, kind in SECTIONS:
        if kind == "river":
            a = RIVER_CENTRE_X - NAV_CHANNEL_HALF
            b = RIVER_CENTRE_X + NAV_CHANNEL_HALF
            # south approach spans, evenly divided up to the channel
            n = max(1, int(round((a - x0) / span)))
            for i in range(n + 1):
                out.append((x0 + i * (a - x0) / n, kind))
            out.append((b, kind))          # north main-span pier
            n = max(1, int(round((x1 - b) / span)))
            for i in range(1, n + 1):
                out.append((b + i * (x1 - b) / n, kind))
        else:
            n = max(1, int(round((x1 - x0) / span)))
            for i in range(n + 1):
                out.append((x0 + i * (x1 - x0) / n, kind))

    ded = []
    for x, k in out:
        if ded and abs(x - ded[-1][0]) < 1.0:
            continue
        ded.append((round(x, 3), k))
    _PIER_CACHE = ded
    return ded


def span_count() -> int:
    return max(0, len(pier_stations()) - 1)


if __name__ == "__main__":
    ps = pier_stations()
    print(f"corridor            {BRIDGE_LENGTH:.0f} m")
    print(f"deck width          {DECK_WIDTH:.1f} m")
    print(f"sections            {len(SECTIONS)}")
    print(f"piers               {len(ps)}")
    print(f"spans               {span_count()}")
    print(f"research zone       {RESEARCH_X0:.0f} - {RESEARCH_X1:.0f} "
          f"({RESEARCH_LENGTH:.0f} m)")
    print(f"sectors             {SECTOR_COUNT} x {SECTOR_LENGTH:.0f} m")
    print(f"river width         {RIVER_WIDTH:.0f} m")
    print(f"deck z at river     {deck_top_z(RIVER_CENTRE_X):.1f} m")
    print(f"soffit z at river   {soffit_z(RIVER_CENTRE_X):.1f} m")
    print(f"air draft at river  "
          f"{soffit_z(RIVER_CENTRE_X) - RIVER_WATER_Z:.1f} m")
    print(f"defect targets      {sum(DEFECT_TARGETS.values())}")
    spans = [ps[i + 1][0] - ps[i][0] for i in range(len(ps) - 1)]
    print(f"span range          {min(spans):.0f} - {max(spans):.0f} m")
    print(f"girder depth range  {girder_depth_for_span(min(spans)):.2f} - "
          f"{girder_depth_for_span(max(spans)):.2f} m")
    print(f"max grade           "
          f"{max(abs(PROFILE[i+1][1]-PROFILE[i][1])/(PROFILE[i+1][0]-PROFILE[i][0])*100 for i in range(len(PROFILE)-1)):.2f} %")
    tall = max((deck_top_z(x) - ground_z(x, 0.0)) for x, k in ps)
    print(f"tallest pier        {tall:.1f} m")
