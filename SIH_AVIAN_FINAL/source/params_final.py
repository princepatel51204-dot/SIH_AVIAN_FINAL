"""SIH_AVIAN_FINAL -- central parameter set.

Deliberately shaped to satisfy the SAME function/constant contract as
AVIAN_ENVIRONMENT/source/params.py, so `bridge.py` and `damage.py` -- both
pure functions of a `P` module reference -- can be reused byte-for-byte by
assigning `bridge.P = params_final` / `damage.P = params_final` before calling
their `build()` entry points. See SIH_AVIAN_FINAL_MASTER_PROMPT.md and
concept/SPEC.md for the source numbers.

COORDINATE SYSTEM -- same convention as REV-C: 1 BU = 1 m, Z-up, +X along the
corridor, origin at the south abutment centreline. Road bridge at y = 0,
metro viaduct at y = +28.

ASSUMPTIONS (working rule 5 -- every one labelled)
---------------------------------------------------
* Road deck carries a SLIGHT LONGITUDINAL VERTICAL CURVE (locked pre-flight
  decision, see the master prompt): a single parabola, high point over the
  river main span (helps air draft), low point at the two abutments. SPEC.md's
  "deck top z = 14.0" is the value AT THE ABUTMENTS; the crest is measured and
  reported, not assumed to equal 14.0 everywhere.
* River: wetted channel width 60 m (not 90) -- the STATED 90 m in SPEC.md is
  read as bank-to-bank (channel + both bank slopes), so the banks reach grade
  exactly at x=135 and x=225, the pier locations either side of the main
  span. This guarantees "no pier in the river" by construction rather than by
  checking a measured clearance after the fact.
* Girder depth is a direct two-value map (1.6 m / 2.5 m), not a span/depth
  ratio -- SPEC.md gives both numbers explicitly.
* Pier column spacing, cap and footing dimensions are scaled down from
  REV-C's (24 m deck, 6 girders) to this 14 m / 5-girder deck, proportionally.
* Approach embankments: a 30 m earthen fill ramp outside the modelled
  corridor (x<0, x>360), rising from grade to 6.0 m at the abutment face --
  the abutment wall itself retains the remaining height to deck level. This
  is what stops the bridge "starting in mid-air".
* Metro span/pier counts fall out of the SAME approach-from-both-ends-toward-
  the-channel algorithm metro.py already uses (monkeypatched, not rewritten),
  so SPEC.md's "12 x 30 m" is a nominal count -- the true count is measured
  and reported once built, not asserted here.
"""
from __future__ import annotations

# ===========================================================================
# 1. CORRIDOR
# ===========================================================================
BRIDGE_LENGTH = 360.0            # m, x = 0 .. 360
DECK_WIDTH = 14.0                # m, 2+2 lanes @ 3.5 m, out-to-out incl. parapets
MEDIAN_WIDTH = 0.6               # m, flush central divider (2+2, not 3+3)
SHOULDER_WIDTH = 0.5             # m
PARAPET_HEIGHT = 1.10            # m above deck top
PARAPET_THICK = 0.40             # m

DECK_SLAB_T = 0.25               # m
WEARING_COURSE_T = 0.075         # m
GIRDER_COUNT = 5                 # longitudinal I-girders
GIRDER_SPACING = 2.75            # m, centre to centre (4 x 2.75 = 11.0 m,
                                  # 1.5 m overhang either side under 14 m deck)
GIRDER_DEPTH_APPROACH = 1.60     # m
GIRDER_DEPTH_MAIN = 2.50         # m, over the 90 m river span


def girder_depth_for_span(span: float) -> float:
    """Direct two-value map -- SPEC.md gives both numbers explicitly."""
    return GIRDER_DEPTH_MAIN if span > 60.0 else GIRDER_DEPTH_APPROACH


GIRDER_DEPTH = GIRDER_DEPTH_APPROACH   # nominal, used only where span unknown
GIRDER_TOP_FLANGE_W = 0.80
GIRDER_BOT_FLANGE_W = 0.70
GIRDER_FLANGE_T = 0.22
GIRDER_WEB_T = 0.20
DIAPHRAGM_T = 0.30
DIAPHRAGM_DEPTH = 1.50

# ---------------------------------------------------------------------------
# 2. SECTIONS (for section_at/section_kind_at -- ground truth labelling only)
# ---------------------------------------------------------------------------
SECTIONS = [
    ("SECTION_01_SOUTH_APPROACH", 0.0, 90.0, 45.0, "approach"),
    ("SECTION_02_RIVER_CROSSING", 90.0, 270.0, 45.0, "river"),
    ("SECTION_03_NORTH_APPROACH", 270.0, 360.0, 45.0, "approach"),
]

NAV_CHANNEL_HALF = 45.0          # m either side of the river centreline
MAIN_SPAN = 2 * NAV_CHANNEL_HALF  # 90 m clear main span

# ---------------------------------------------------------------------------
# 3. PIERS -- fixed stations, not derived. SPEC.md gives exact x-values.
# ---------------------------------------------------------------------------
PIER_COL_D = 1.70                # m, twin circular columns
PIER_COL_D_RIVER = 1.70          # m, unused (no pier is ever in the river),
                                  # kept equal for bridge.py's contract
PIER_COLS_PER_BENT = 2
PIER_COL_SPACING = 5.5           # m between the two columns of a bent
PIER_CAP_L = 12.0                # m, transverse pier cap length
PIER_CAP_W = 1.80
PIER_CAP_H = 1.40
PIER_FOOTING_L = 5.0
PIER_FOOTING_W = 4.0
PIER_FOOTING_H = 1.2
BEARING_H = 0.30
BEARING_L = 0.50
BEARING_W = 0.45

_PIER_STATIONS = [0.0, 45.0, 90.0, 135.0, 225.0, 270.0, 315.0, 360.0]


def pier_stations():
    """Fixed per SPEC.md. `kind` is always 'land' -- the main span is sized
    exactly to the river's bank-to-bank width, so no pier ever sits in it."""
    return [(x, "land") for x in _PIER_STATIONS]


def span_count() -> int:
    return len(_PIER_STATIONS) - 1


# ---------------------------------------------------------------------------
# 4. EXPANSION JOINTS AND DRAINAGE
# ---------------------------------------------------------------------------
JOINT_EVERY_N_SPANS = 3          # -> joints at pier index 3 and 6 (2 joints)
JOINT_GAP = 0.08
DRAIN_SPACING = 20.0
DRAIN_PIPE_D = 0.20
SERVICE_DUCT_D = 0.30

# ---------------------------------------------------------------------------
# 5. RESEARCH ZONE / LOD BANDS
# ---------------------------------------------------------------------------
RESEARCH_X0 = 90.0
RESEARCH_X1 = 270.0
RESEARCH_LENGTH = RESEARCH_X1 - RESEARCH_X0      # 180 m

# The corridor ends exactly at the abutments, so "MED to the abutments, LOW
# beyond" (SPEC.md S4.1) means LOW never actually occurs: setting the margin
# to exactly reach x=0/360 gives HIGH inside the research zone and MED
# everywhere else in the modelled corridor -- no LOW band, by construction.
LOD_MED_MARGIN = RESEARCH_X0     # = 90.0, reaches exactly to both abutments
DETAIL_HIGH = "HIGH"
DETAIL_MED = "MED"
DETAIL_LOW = "LOW"


def detail_at(x: float) -> str:
    if RESEARCH_X0 <= x <= RESEARCH_X1:
        return DETAIL_HIGH
    if (RESEARCH_X0 - LOD_MED_MARGIN) <= x <= (RESEARCH_X1 + LOD_MED_MARGIN):
        return DETAIL_MED
    return DETAIL_LOW


# ---------------------------------------------------------------------------
# 6. INSPECTION SECTORS
# ---------------------------------------------------------------------------
SECTOR_COUNT = 3
SECTOR_NAMES = ["SECTOR_A", "SECTOR_B", "SECTOR_C"]
SECTOR_LENGTH = RESEARCH_LENGTH / SECTOR_COUNT   # 60 m


def sector_at(x: float):
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
# 7. RIVER  (see module docstring -- wetted width vs bank-to-bank)
# ---------------------------------------------------------------------------
RIVER_CENTRE_X = 180.0
RIVER_WIDTH = 60.0               # m, WETTED channel (bank-to-bank is 90 m:
                                  # 60 m channel + 15 m bank run either side)
RIVER_WATER_Z = -2.0
RIVER_DEPTH = 4.0
RIVER_BANK_SLOPE = 7.5           # run = 7.5 * 2.0 = 15 m each side
RIVER_LENGTH = 130.0             # m modelled along Y (covers both structures)
RIVER_ROCK_COUNT = 15
RIVER_VEG_COUNT = 20             # SPEC.md: "~20 trees"

# ---------------------------------------------------------------------------
# 8. TERRAIN / APPROACH EMBANKMENTS
# ---------------------------------------------------------------------------
EMBANKMENT_RUN = 30.0            # m, outside the corridor (x<0, x>360)
EMBANKMENT_H = 6.0               # m, fill height at the abutment face --
                                  # the abutment wall retains the rest

GROUND_X0, GROUND_X1 = -60.0, 420.0
GROUND_Y0, GROUND_Y1 = -50.0, 100.0
GROUND_RES_X = 120
GROUND_RES_Y = 60
FAR_GROUND_Z = -1.2
FAR_GROUND_PAD = 8000.0

# ---------------------------------------------------------------------------
# 9. CITY -- none. A handful of distant silhouettes only (locked decision 3).
# ---------------------------------------------------------------------------
DISTANT_SKYLINE_COUNT = 3
VEHICLE_COUNT_BRIDGE = 16        # SPEC.md: ~16 cars, trucks, a bus

# ---------------------------------------------------------------------------
# 10. DAMAGE -- 96 total, per SPEC.md's table exactly (9 types, no diagonal
# crack, no repair patch this time).
#
# REBAR_EXPOSED is targeted at 5, not 6: the 6th is the hand-placed HERO
# defect (SPEC S5.2 -- "place, deliberately and documented as such"), added
# by build_final.py after the random population, on a river-adjacent pier at
# z ~ 6 m. It occupies one of REBAR_EXPOSED's 6 slots, not an extra one, so
# the total stays exactly 96 -- see build_final.py's "tag the hero defect".
# ---------------------------------------------------------------------------
DAMAGE_DENSITY = 1.0

DEFECT_TARGETS = {
    "CRACK_HAIRLINE": 24,
    "CRACK_LONGITUDINAL": 12,
    "CRACK_TRANSVERSE": 10,
    "CRACK_NETWORK": 8,
    "SPALL": 12,
    "DELAMINATION": 8,
    "REBAR_EXPOSED": 5,             # + 1 hand-placed hero = 6
    "CORROSION_STAIN": 10,
    "JOINT_DETERIORATION": 6,
}
assert sum(DEFECT_TARGETS.values()) == 95, "95 random + 1 hand-placed hero = 96"

SEVERITY_WEIGHTS = {1: 0.44, 2: 0.30, 3: 0.18, 4: 0.08}

REPAIR_ENVELOPE = {
    "max_severity": 3,
    "max_area_m2": 0.45,
    "max_crack_width_mm": 3.0,
    "max_crack_length_m": 2.2,
    "rebar_exposed_blocks": True,
}

GEOMETRIC_DEFECTS = {"SPALL", "DELAMINATION", "REBAR_EXPOSED"}
SHADER_DEFECTS = {"CRACK_HAIRLINE", "CRACK_LONGITUDINAL", "CRACK_TRANSVERSE",
                  "CRACK_NETWORK", "CORROSION_STAIN", "JOINT_DETERIORATION"}

CRACK_WIDTH_MM = {1: (0.05, 0.20), 2: (0.20, 0.80),
                  3: (0.80, 2.50), 4: (2.50, 8.00)}

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
# 11. LIGHTING -- low morning sun, raking across the corridor (SPEC S4.4).
# Azimuth ~90 deg from the corridor's +X axis so it grazes the Y-facing
# girder web / pier faces and the deck soffit rather than lighting them flat.
# ---------------------------------------------------------------------------
SUN_ELEVATION_DEG = 11.0         # low morning sun, was 52 deg (REV-C midday)
SUN_ROTATION_DEG = 100.0         # near-perpendicular to the corridor axis
SUN_STRENGTH = 5.0
SUN_ANGLE_DEG = 0.526
SKY_BACKGROUND_STRENGTH = 1.0
RENDER_EXPOSURE = -0.6           # low sun needs less exposure pull-down
SKY_DUST = 0.9
SKY_OZONE = 1.6
HAZE_COLOR = (0.55, 0.56, 0.58)
HAZE_BLEND_DEG = 3.0

# ---------------------------------------------------------------------------
# 12. RENDER
# ---------------------------------------------------------------------------
RENDER_W, RENDER_H = 1920, 1080
RENDER_SAMPLES = 64
RENDER_SAMPLES_CLOSEUP = 96
SEED = 20260921                  # SPEC.md: new baseline seed

# ---------------------------------------------------------------------------
# 13. DERIVED PROFILE -- the vertical curve
# ---------------------------------------------------------------------------
DECK_Z_ABUTMENT = 14.0           # m, SPEC.md value, held AT the abutments
CROWN_RISE = 0.6                 # m, measured/reported, not asserted


def deck_top_z(x: float) -> float:
    """Single parabolic vertical curve, crest over the river main span."""
    t = (x - RIVER_CENTRE_X) / RIVER_CENTRE_X
    t = max(-1.0, min(1.0, t))
    return DECK_Z_ABUTMENT + CROWN_RISE * (1.0 - t * t)


def span_at(x: float) -> float:
    ps = pier_stations()
    for i in range(len(ps) - 1):
        if ps[i][0] <= x <= ps[i + 1][0]:
            return ps[i + 1][0] - ps[i][0]
    return 45.0


def soffit_z(x: float) -> float:
    return (deck_top_z(x) - WEARING_COURSE_T - DECK_SLAB_T
            - girder_depth_for_span(span_at(x)))


def ground_z(x: float, y: float) -> float:
    """Terrain height: river channel + banks inside [0,360], approach
    embankment ramps outside it. y is unused (river runs uniformly along Y),
    matching AVIAN_ENVIRONMENT/source/params.py's own convention."""
    if x < 0.0:
        d = -x
        if d >= EMBANKMENT_RUN:
            return 0.0
        t = 1.0 - d / EMBANKMENT_RUN
        return EMBANKMENT_H * (t * t * (3.0 - 2.0 * t))
    if x > BRIDGE_LENGTH:
        d = x - BRIDGE_LENGTH
        if d >= EMBANKMENT_RUN:
            return 0.0
        t = 1.0 - d / EMBANKMENT_RUN
        return EMBANKMENT_H * (t * t * (3.0 - 2.0 * t))

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


if __name__ == "__main__":
    ps = pier_stations()
    print(f"corridor            {BRIDGE_LENGTH:.0f} m")
    print(f"deck width          {DECK_WIDTH:.1f} m")
    print(f"piers               {len(ps)}  spans {span_count()}")
    print(f"research zone       {RESEARCH_X0:.0f} - {RESEARCH_X1:.0f} "
          f"({RESEARCH_LENGTH:.0f} m)")
    print(f"deck z: abutment {deck_top_z(0.0):.3f}  crest {deck_top_z(180.0):.3f}")
    print(f"soffit at midspan   {soffit_z(180.0):.3f}")
    print(f"air draft at river  {soffit_z(180.0) - RIVER_WATER_Z:.3f} m")
    print(f"river bank reaches grade at x = "
          f"{RIVER_CENTRE_X - RIVER_WIDTH/2 - RIVER_BANK_SLOPE*abs(RIVER_WATER_Z):.1f}"
          f" / {RIVER_CENTRE_X + RIVER_WIDTH/2 + RIVER_BANK_SLOPE*abs(RIVER_WATER_Z):.1f}")
    print(f"ground_z at piers   " + ", ".join(f"{x:.0f}:{ground_z(x,0):.2f}" for x, k in ps))
    print(f"ground_z at abutments (outside) -10:{ground_z(-10,0):.2f} "
          f"0:{ground_z(0,0):.2f} 360:{ground_z(360,0):.2f} 370:{ground_z(370,0):.2f}")
    print(f"defect targets      {sum(DEFECT_TARGETS.values())}")
