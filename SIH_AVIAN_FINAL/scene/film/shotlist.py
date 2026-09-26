"""The film's shot list, and why each shot is allowed to exist.

Two kinds of beat:

  real      The camera sits at a position the drone actually occupied during
            full_pass_05, taken from pose_audit_track.csv. Aim and zoom are
            synthesized -- the recorded gimbal does not point at these defects
            -- so the label is "flown position, re-aimed camera", never
            "flown shot".

  staged    A purpose-built viewpoint that was NOT visited in full_pass_05, but
            is operationally valid: standoff >= MIN_LEGAL_STANDOFF and the
            camera's own clearance to any geometry >= MIN_LEGAL_CLEARANCE.

MIN_LEGAL_STANDOFF is 3.0 m because that is the sensed clearance the mission's
safety case rests on. Staging a shot closer would put a number on screen that
was measured somewhere we would never fly, so those framings are refused even
when the detector fires on them -- DEFECT_JOINT_DETERIORATION_004 is exactly
that case: 0.921 at 1.5 m, nothing at 3.0/3.5/4.5 m, and a viewpoint whose own
clearance was 0.1-1.2 m. It is not in this film.

Every `conf_at_range` below was measured by the real detector on a frame
rendered at that shot's own standoff. They are recorded here as the expectation;
the film draws only what the detector returns on the final frames.
"""

MIN_LEGAL_STANDOFF = 3.0
MIN_LEGAL_CLEARANCE = 3.0
FPS = 24

# beat timing, in frames at 24 fps -- 192 frames = 8.0 s
APPROACH = 48          # 2.0 s  moving in
HOLD = 60              # 2.5 s  settled, detection fires and is held
CARD = 60              # 2.5 s  annotation card up, box still shown
PEEL = 24              # 1.0 s  pulling away
BEAT_FRAMES = APPROACH + HOLD + CARD + PEEL

BEATS = [
    dict(n=1, defect="DEFECT_SPALL_007", kind="real", t=906.4, range_m=8.34,
         conf_at_range=0.994, surface="GIRDER SOFFIT", section="SECTION 01"),
    dict(n=2, defect="DEFECT_SPALL_011", kind="real", t=1068.7, range_m=6.75,
         conf_at_range=0.958, surface="PIER COLUMN", section="SECTION 02"),
    dict(n=3, defect="DEFECT_SPALL_009", kind="real", t=815.5, range_m=6.68,
         conf_at_range=0.769, surface="PIER CAP", section="SECTION 01"),
    dict(n=4, defect="DEFECT_SPALL_008", kind="real", t=1967.8, range_m=6.09,
         conf_at_range=0.987, surface="GIRDER SOFFIT", section="SECTION 03"),
    dict(n=5, defect="DEFECT_REBAR_EXPOSED_004", kind="real", t=4587.6, range_m=7.40,
         conf_at_range=0.662, surface="BEARING SEAT", section="SECTION 02"),
    dict(n=6, defect="DEFECT_SPALL_002", kind="real", t=1505.7, range_m=5.76,
         conf_at_range=0.974, surface="GIRDER SOFFIT", section="SECTION 02"),
    dict(n=7, defect="DEFECT_DELAMINATION_001", kind="staged", standoff=4.5,
         conf_at_range=0.959, cam_clearance=6.4,
         surface="GIRDER SOFFIT", section="SECTION 03"),
    dict(n=8, defect="DEFECT_REBAR_EXPOSED_001", kind="staged", standoff=3.5,
         conf_at_range=0.989, cam_clearance=3.5,
         surface="GIRDER WEB", section="SECTION 01"),
]

# Spares, all confirmed to fire from a real flown position at a legal range.
# Used only if a chosen beat stops firing on its final rendered frames.
SPARES = [
    dict(defect="DEFECT_SPALL_001", t=1402.3, range_m=7.49, conf_at_range=0.981,
         surface="GIRDER SOFFIT", section="SECTION 02"),
    dict(defect="DEFECT_SPALL_003", t=939.8, range_m=6.39, conf_at_range=0.985,
         surface="GIRDER SOFFIT", section="SECTION 01"),
    dict(defect="DEFECT_SPALL_004", t=1917.0, range_m=5.16, conf_at_range=0.975,
         surface="GIRDER SOFFIT", section="SECTION 03"),
    dict(defect="DEFECT_SPALL_005", t=877.4, range_m=7.45, conf_at_range=0.859,
         surface="GIRDER SOFFIT", section="SECTION 01"),
    dict(defect="DEFECT_SPALL_006", t=1210.8, range_m=5.31, conf_at_range=0.974,
         surface="GIRDER SOFFIT", section="SECTION 02"),
]

# Wides and connective tissue. All chase shots ride the real trajectory with the
# drone visible; the chase camera's own path is a smoothed body-frame offset,
# which the report states plainly.
WIDES = [
    dict(name="open", t=820.0, frames=144, chase_offset=(-6.5, 1.5, 2.2),
         lens=28.0, label="establishing"),
    dict(name="mid_a", t=1205.0, frames=48, chase_offset=(-4.0, -1.2, 1.4),
         lens=35.0, label="transit"),
    dict(name="mid_b", t=1960.0, frames=48, chase_offset=(-3.4, 1.0, 0.9),
         lens=35.0, label="transit"),
    dict(name="close", t=4590.0, frames=144, chase_offset=(-7.5, -2.0, 3.0),
         lens=24.0, label="pull-out"),
]

# Only numbers that exist in committed results. 9050.3 m and 2.61 h are measured
# from pose_audit_track.csv directly; the rest are from full_pass_05's own
# committed summary.
TITLE_FACTS = [
    "1161 / 1162 waypoints reached",
    "0 collisions",
    "81.10 % measured coverage",
    "9050.3 m flown  ·  2.61 h",
]


def total_frames():
    return len(BEATS) * BEAT_FRAMES + sum(w["frames"] for w in WIDES)


if __name__ == "__main__":
    n = total_frames()
    print(f"beats       {len(BEATS)} x {BEAT_FRAMES} = {len(BEATS)*BEAT_FRAMES} frames")
    print(f"wides       {sum(w['frames'] for w in WIDES)} frames")
    print(f"TOTAL       {n} frames = {n/FPS:.1f} s at {FPS} fps")
    for b in BEATS:
        r = b.get("range_m") or b.get("standoff")
        print(f"  beat {b['n']}  {b['defect']:<30} {b['kind']:<7} "
              f"{r:4.1f} m  conf {b['conf_at_range']:.3f}  {b['surface']}")
