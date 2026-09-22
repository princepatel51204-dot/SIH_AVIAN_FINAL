"""SIH_AVIAN_FINAL -- sabotage harness for validate_final.py's checks.

Same before/mutate/re-check/undo/assert shape as
AVIAN_ENVIRONMENT/source/sabotage_c.py: every sabotage function mutates one
thing and returns its own inverse, so `run()` can always put the scene back
regardless of whether the check actually caught the mutation.
"""
from __future__ import annotations

import bpy

import params_final as PF
import validate_final as VF


def _find(prefix, contains=None):
    for o in bpy.data.objects:
        if o.name.startswith(prefix) and (contains is None or contains in o.name):
            return o
    return None


def _touch():
    """meshlib.box()/cylinder() bake position into VERTICES and leave
    .location at world (0,0,0) -- so a .location/.scale change is a DELTA
    on top of whatever the baked geometry already encodes, not an absolute
    new position, and matrix_world does not reliably reflect a Python-side
    transform edit until the view layer updates. Every transform-based
    sabotage below calls this right after mutating."""
    bpy.context.view_layer.update()


def _move_to(ob, target_x=None, target_y=None):
    """Shift ob by the DELTA needed to land its bbox centre at target_x/y,
    computed from its CURRENT real-world position (via bounding box, not
    .matrix_world.translation) -- robust regardless of what the object's
    baked vertex offset already is. Returns an undo callable."""
    b = VF._bb(ob)
    cur_x, cur_y = (b[0] + b[1]) / 2.0, (b[2] + b[3]) / 2.0
    old = ob.location.copy()
    dx = 0.0 if target_x is None else target_x - cur_x
    dy = 0.0 if target_y is None else target_y - cur_y
    ob.location = (old.x + dx, old.y + dy, old.z)
    _touch()
    def undo():
        ob.location = old
        _touch()
    return undo


def _sab_VF01():
    ob = _find("BR_DECK_SLAB_")
    old = ob.scale.copy()
    ob.scale = (old.x, old.y * 0.3, old.z)
    _touch()
    def undo():
        ob.scale = old
        _touch()
    return undo


def _sab_VF02():
    # Matches VF02's own measurement (deck-slab extent): shift the last
    # span's deck slab off the end of the corridor.
    ob = _find("BR_DECK_SLAB_007") or _find("BR_DECK_SLAB_")
    old = ob.location.copy()
    ob.location = (old.x + 500.0, old.y, old.z)
    _touch()
    def undo():
        ob.location = old
        _touch()
    return undo


def _sab_VF03():
    old = PF.GIRDER_DEPTH_MAIN
    PF.GIRDER_DEPTH_MAIN = 0.05
    def undo():
        PF.GIRDER_DEPTH_MAIN = old
    return undo


def _sab_VF04():
    # link_dup() objects DO carry a real .location (unlike box/cylinder),
    # so this one is a plain absolute-value bump.
    ob = _find("ENV_RIPRAP_001") or _find("ENV_BANKVEG_001")
    old = ob.location.copy()
    ob.location = (old.x, old.y, old.z + 5.0)
    _touch()
    def undo():
        ob.location = old
        _touch()
    return undo


def _sab_VF05():
    ob = _find("BR_PIER_COL_004")
    return _move_to(ob, target_x=180.0)


def _sab_VF06_07():
    old = PF.RIVER_WATER_Z
    PF.RIVER_WATER_Z = 10.0
    def undo():
        PF.RIVER_WATER_Z = old
    return undo


# VF08 (unique object names) has no sabotage here: bpy.data.objects enforces
# name uniqueness itself (a rename to an existing name is auto-suffixed
# ".001"), so the condition VF08 checks for cannot actually occur in this
# engine. Documented rather than faked -- see the gate report.


def _sab_VF09():
    import meshlib as ML
    real = ML.scene_tris
    ML.scene_tris = lambda: 999_999_999
    def undo():
        ML.scene_tris = real
    return undo


def _sab_VF10():
    cams = [o for o in bpy.data.objects if o.name == "CAM_01_OVERVIEW"]
    if not cams:
        return lambda: None
    cam = cams[0]
    old = cam.data.clip_end
    cam.data.clip_end = 10.0
    return lambda: setattr(cam.data, "clip_end", old)


def _sab_VF11(records):
    olds = [rec.get("surface_offset_mm") for rec in records[:10]]
    for rec in records[:10]:
        rec["surface_offset_mm"] = -1.0
    def undo():
        for rec, old in zip(records[:10], olds):
            rec["surface_offset_mm"] = old
    return undo


def _sab_VF12(records):
    r = records[0]
    key = "area_m2" if "area_m2" in r else list(r.keys())[-1]
    old = r.pop(key)
    def undo():
        r[key] = old
    return undo


def _sab_VF13(records):
    removed = records.pop()
    def undo():
        records.append(removed)
    return undo


def _sab_VF14():
    # Land a metro pier's bbox centre exactly on a road pier's (135, 0).
    ob = _find("MB_PIER_COL_")
    return _move_to(ob, target_x=135.0, target_y=0.0)


def _sab_VF15():
    # Pull a metro pier down to y=10 -- inside the 16.5 m inter-structure
    # gap (road edge at y=7, metro edge at y~23.7).
    ob = _find("MB_PIER_COL_")
    return _move_to(ob, target_y=10.0)


def _sab_VF16():
    ob = _find("MB_PIER_COL_")
    had = "avi_kind" in ob.keys()
    old = ob.get("avi_kind")
    if had:
        del ob["avi_kind"]
    def undo():
        if had:
            ob["avi_kind"] = old
    return undo


def _sab_VF17(records):
    hero = next(r for r in records if r.get("type") == "REBAR_EXPOSED"
               and bpy.data.objects.get(r["defect_id"]) is not None
               and bpy.data.objects[r["defect_id"]].get("avi_hero"))
    ob = bpy.data.objects[hero["defect_id"]]
    del ob["avi_hero"]
    def undo():
        ob["avi_hero"] = True
    return undo


def _sab_VF18(records):
    r = records[5]
    old = list(r["position_m"])
    r["position_m"] = [old[0] + 5.0, old[1], old[2]]
    def undo():
        r["position_m"] = old
    return undo


def _sab_VF19():
    # bpy.data.objects still lists an unlinked object (it's collection
    # membership, not existence), so VF19's bpy.data.objects scan would not
    # even notice an unlink -- renaming it out of the expected name is what
    # actually makes it disappear from that check.
    ob = _find("AVI_BASE_SCANNER")
    old_name = ob.name
    ob.name = "AVI_BASE_SCANNER_SABOTAGED"
    def undo():
        ob.name = old_name
    return undo


def _sab_VF20():
    ob = _find("AVI_BASE_REPAIRER")
    old = ob.get("avi_base_role")
    ob["avi_base_role"] = "NEITHER"
    def undo():
        ob["avi_base_role"] = old
    return undo


def _sab_VF21():
    ob = _find("AVI_BASE_SCANNER")
    old = ob.location.copy()
    ob.location = (old.x, old.y, old.z + 2.0)
    _touch()
    def undo():
        ob.location = old
        _touch()
    return undo


def _sab_VF22(collision_path):
    import json
    with open(collision_path) as f:
        original = f.read()
    data = json.loads(original)
    data["primitives"] = [p for p in data["primitives"]
                          if p.get("kind") != "landing_pad"
                          and not str(p.get("name", "")).startswith("AVI_BASE_")]
    with open(collision_path, "w") as f:
        json.dump(data, f)
    def undo():
        with open(collision_path, "w") as f:
            f.write(original)
    return undo


def _gazebo_models_dir():
    import os
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "gazebo", "models")


def _sdf_files():
    import os
    d = _gazebo_models_dir()
    if not os.path.isdir(d):
        return []
    out = []
    for name in sorted(os.listdir(d)):
        p = os.path.join(d, name, "model.sdf")
        if os.path.exists(p):
            out.append(p)
    return out


def _sab_VF26(sdf_files):
    """Strip every <material>...</material> block from ONE model.sdf --
    plain text, not an XML-library round trip, so what gets restored on
    undo is byte-identical to what was there, not a reserialized copy."""
    import re
    path = sdf_files[0]
    with open(path) as f:
        original = f.read()
    stripped = re.sub(r"<material>.*?</material>\s*", "",
                      original, flags=re.DOTALL)
    with open(path, "w") as f:
        f.write(stripped)
    def undo():
        with open(path, "w") as f:
            f.write(original)
    return undo


def _sab_VF27(sdf_files):
    """Force every <diffuse> value in every exported model to the SAME
    colour -- the check-that-cannot-fail shape VF26 alone has: it would
    still report every visual has A material block, just all the same
    one."""
    import re
    originals = {}
    for path in sdf_files:
        with open(path) as f:
            originals[path] = f.read()
        forced = re.sub(r"<diffuse>[^<]*</diffuse>",
                        "<diffuse>0.500 0.500 0.500 1</diffuse>",
                        originals[path])
        with open(path, "w") as f:
            f.write(forced)
    def undo():
        for path, content in originals.items():
            with open(path, "w") as f:
                f.write(content)
    return undo


def _sab_VF29():
    ob = _find("ST_")
    for o in bpy.data.objects:
        if o.name.startswith("ST_") and o.get("avi_kind") in (
            "truss_chord", "truss_diagonal", "truss_vertical",
            "gusset_plate", "floor_beam", "stringer", "bracing"):
            ob = o
            break
    had = "avi_kind" in ob.keys()
    old = ob.get("avi_kind")
    if had:
        del ob["avi_kind"]
    def undo():
        if had:
            ob["avi_kind"] = old
    return undo


def _sab_VF30():
    """Rename enough bolt objects out of the FAST_ namespace to drop the
    scene-wide count below VF30's 1,000 floor -- undo restores every name."""
    bolts = [o for o in bpy.data.objects
            if o.name.startswith("FAST_")
            and ((o.name.endswith("_NUT")
                 and not o.name.endswith("_MARK_NUT"))
                or o.name.endswith("_HOLE"))]
    victims = bolts[:250]
    renamed = []
    for o in victims:
        old = o.name
        o.name = "SABOTAGED_" + old
        renamed.append((o, old))
    def undo():
        for o, old in renamed:
            o.name = old
    return undo


def _sab_VF31():
    plate = _find("FAST_", contains="_MARK_PLATE")
    old_name = plate.name
    plate.name = old_name + "_SABOTAGED"
    def undo():
        plate.name = old_name
    return undo


def _sab_VF32(srecords):
    loose = next((r for r in srecords
                 if r["type"] in ("BOLT_LOOSE", "JOINT_ANCHOR_LOOSE")), None)
    if loose is None:
        return lambda: None
    did = loose["defect_id"]
    bolt_id = did[:-len("_MARK_NUT")] if did.endswith("_MARK_NUT") else did
    nut_mark = bpy.data.objects.get(f"{bolt_id}_MARK_NUT")
    plate_mark = bpy.data.objects.get(f"{bolt_id}_MARK_PLATE")
    if nut_mark is None or plate_mark is None:
        return lambda: None
    old = nut_mark.rotation_euler.copy()
    nut_mark.rotation_euler = plate_mark.rotation_euler.copy()
    _touch()
    def undo():
        nut_mark.rotation_euler = old
        _touch()
    return undo


def _sab_VF33(srecords):
    r = srecords[0]
    key = "host_surface" if "host_surface" in r else list(r.keys())[-1]
    old = r.pop(key)
    def undo():
        r[key] = old
    return undo


def _sab_VF34(srecords):
    r = srecords[0]
    old = r.pop("feature_size_mm", None)
    def undo():
        if old is not None:
            r["feature_size_mm"] = old
    return undo


def _sab_VF35(srecords):
    olds = [(r, r.get("min_detect_range_m")) for r in srecords]
    for r in srecords:
        r["min_detect_range_m"] = 1.0
    def undo():
        for r, old in olds:
            r["min_detect_range_m"] = old
    return undo


def _sab_VF36():
    ob = _find("ST_")
    had = "avi_condition" in ob.keys()
    old = ob.get("avi_condition")
    if had:
        del ob["avi_condition"]
    def undo():
        if had:
            ob["avi_condition"] = old
    return undo


def _sab_VF37():
    old = tuple(PF.TRUSS_VERTICAL_SIZE)
    PF.TRUSS_VERTICAL_SIZE = (12.0, 12.0)
    def undo():
        PF.TRUSS_VERTICAL_SIZE = old
    return undo


def _sab_VF38(srecords):
    r = srecords[0]
    old = list(r["position_m"])
    r["position_m"] = [old[0] + 5.0, old[1], old[2]]
    def undo():
        r["position_m"] = old
    return undo


def run(records, mrecords, baseline_path, log=print, collision_path=None,
       srecords=None, steel_baseline_path=None):
    srecords = srecords or []
    import os
    rows = []
    if collision_path is None:
        collision_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "scene", "collision", "avian_bridge_collision.json")

    def check():
        R, summary = VF.run(records, mrecords, log=lambda *a: None,
                            baseline_path=baseline_path, srecords=srecords,
                            steel_baseline_path=steel_baseline_path)
        return {r.id: r.status for r in R}

    plan = [
        ("VF01", "shrink a deck slab's width", lambda: _sab_VF01()),
        ("VF02", "move the north abutment 500 m away", lambda: _sab_VF02()),
        ("VF03", "collapse the main-span girder depth to 5 cm",
         lambda: _sab_VF03()),
        ("VF04", "lift a riprap block 5 m off the ground",
         lambda: _sab_VF04()),
        ("VF05", "move a pier into the navigation channel",
         lambda: _sab_VF05()),
        ("VF06_07", "raise the river water 12 m above the deck",
         lambda: _sab_VF06_07()),
        ("VF09", "report an impossible triangle count",
         lambda: _sab_VF09()),
        ("VF10", "shrink a camera's far clip to 10 m", lambda: _sab_VF10()),
        ("VF11", "blank 10 defects' surface_offset_mm",
         lambda: _sab_VF11(records)),
        ("VF12", "remove a required field from a road defect record",
         lambda: _sab_VF12(records)),
        ("VF13", "delete a road defect record", lambda: _sab_VF13(records)),
        ("VF14", "move a metro pier onto a road pier",
         lambda: _sab_VF14()),
        ("VF15", "push a metro pier into the inter-structure gap",
         lambda: _sab_VF15()),
        ("VF16", "strip avi_kind off a metro pier column",
         lambda: _sab_VF16()),
        ("VF17", "strip the avi_hero tag off the hero defect",
         lambda: _sab_VF17(records)),
    ]
    if baseline_path:
        plan.append(("VF18", "shift a defect 5 m off its baseline position",
                    lambda: _sab_VF18(records)))
    plan += [
        ("VF19", "rename a landing pad out from under its expected name",
         lambda: _sab_VF19()),
        ("VF20", "corrupt a landing pad's avi_base_role tag",
         lambda: _sab_VF20()),
        ("VF21", "lift a landing pad 2 m off the terrain",
         lambda: _sab_VF21()),
    ]
    if os.path.exists(collision_path):
        plan.append(("VF22", "strip landing_pad primitives from the "
                    "exported collision JSON",
                    lambda: _sab_VF22(collision_path)))

    sdf_files = _sdf_files()
    if sdf_files:
        plan.append(("VF26", "strip all <material> blocks from one "
                    "exported model.sdf", lambda: _sab_VF26(sdf_files)))
        plan.append(("VF27", "force every exported <diffuse> to the same "
                    "grey", lambda: _sab_VF27(sdf_files)))

    plan.append(("VF28", "raise the river water 12 m above the deck "
                "(same cause as VF06/07 -- the truss's own draft check)",
                lambda: _sab_VF06_07()))
    if srecords:
        plan += [
            ("VF29", "strip avi_kind off a steel truss member",
             lambda: _sab_VF29()),
            ("VF30", "rename 250 bolts out of the FAST_ namespace",
             lambda: _sab_VF30()),
            ("VF31", "rename a bolt's plate-side match mark away",
             lambda: _sab_VF31()),
            ("VF32", "un-rotate a loose bolt's nut-side match mark",
             lambda: _sab_VF32(srecords)),
            ("VF33", "remove a required field from a steel defect record",
             lambda: _sab_VF33(srecords)),
            ("VF34", "strip feature_size_mm off a steel defect record",
             lambda: _sab_VF34(srecords)),
            ("VF35", "force every steel min_detect_range_m to the same "
             "value", lambda: _sab_VF35(srecords)),
            ("VF36", "strip avi_condition off a steel truss member",
             lambda: _sab_VF36()),
            ("VF37", "blow up the truss vertical member size",
             lambda: _sab_VF37()),
        ]
        if steel_baseline_path:
            plan.append(("VF38", "shift a steel defect 5 m off its "
                        "baseline position", lambda: _sab_VF38(srecords)))

    for check_id, desc, setup in plan:
        before = check()
        b = before.get(check_id, before.get(check_id.split("_")[0]))
        undo = setup()
        try:
            after = check()
            a = after.get(check_id, after.get(check_id.split("_")[0]))
            proven = (b == "PASS" and a == "FAIL")
            rows.append({"check": check_id, "sabotage": desc,
                        "before": b, "after": a, "proven": proven})
            log(f"  [{'OK ' if proven else 'BAD'}] {check_id:8} {desc:<48} "
                f"{b} -> {a}")
        finally:
            undo()

    proven_n = sum(1 for r in rows if r["proven"])
    log(f"  SABOTAGE: {proven_n}/{len(rows)} proven capable of failing")
    return {"total": len(rows), "proven": proven_n,
           "unproven": [r["check"] for r in rows if not r["proven"]],
           "rows": rows}
