"""REV-C Stage 2 -- the metro viaduct's own defect population.

SEPARATE FROM THE ROAD BRIDGE, DELIBERATELY
-------------------------------------------
`MDEFECT_*`, its own record list, its own export, its own baseline. V22
compares the road-bridge ground truth against the frozen REV-A baseline and
must keep seeing exactly 192 -- so a metro defect must not be able to enter
that list by any route. Separate namespace and separate list is what
guarantees it; nothing here appends to `records`.

REUSING damage.py's MAKERS, NOT REIMPLEMENTING THEM
---------------------------------------------------
The crack and spall geometry/shader builders in damage.py are validated and
ground-truth-bearing. Writing a second set for the metro would be two
sources of the same visuals, drifting apart the first time one is tuned --
the same argument that makes the URDF generated rather than hand-written.

So the makers are called as-is and the resulting object is RENAMED to
MDEFECT_*. damage.py is not edited: it is in the protected set in S4 of the
REV-C brief, and it is the single most ground-truth-critical file in the
environment. A rename afterwards costs nothing and touches nothing.

The makers key their crack pattern off a fixed vocabulary, so each
metro-specific defect type maps onto a base type for the shader while the
record keeps the metro name. Both are recorded: `type` is what it is,
`base_type` is what drew it.
"""
from __future__ import annotations
import math
import random

import bpy
from mathutils import Vector

import damage as DM
import meshlib as ML
import metro as MB
import params as P


# metro type -> (base type the maker understands, where it lives)
TYPES = {
    "SEGMENTAL_JOINT_LEAKAGE": ("JOINT_DETERIORATION", "JOINT"),
    "EFFLORESCENCE_JOINT": ("CORROSION_STAIN", "JOINT"),
    "BEARING_DISTRESS": ("SPALL", "BEARING"),
    "INTERNAL_SOFFIT_CRACK": ("CRACK_LONGITUDINAL", "BOX_INTERIOR"),
    "WEB_SHEAR_CRACK": ("CRACK_DIAGONAL", "WEB"),
    "SOFFIT_TRANSVERSE_CRACK": ("CRACK_TRANSVERSE", "SOFFIT"),
    "PIER_SPALL": ("SPALL", "PIER"),
    "PIER_CORROSION_STAIN": ("CORROSION_STAIN", "PIER"),
}

# How many of each. Deliberately a smaller population than the road
# bridge's 192 -- the metro is a second target, not a second bridge.
COUNTS = {
    "SEGMENTAL_JOINT_LEAKAGE": 9,
    "EFFLORESCENCE_JOINT": 7,
    "BEARING_DISTRESS": 6,
    "INTERNAL_SOFFIT_CRACK": 8,
    "WEB_SHEAR_CRACK": 10,
    "SOFFIT_TRANSVERSE_CRACK": 8,
    "PIER_SPALL": 6,
    "PIER_CORROSION_STAIN": 6,
}

_APOTHEM = math.cos(math.pi / 16.0)     # 16-gon: flat face sits inside R

# Metro defect instance ids occupy 501..999 -- inside dataset.py's defect
# band (1..999), clear of the road bridge's 1..192, and below
# STRUCTURE_INDEX_BASE at 1000.
METRO_INDEX_BASE = 500


def _sector_of(x):
    """Reuse the road bridge's sector banding so both structures share it."""
    return P.sector_of(x) if hasattr(P, "sector_of") else None


def _sites(rnd, log=print):
    """Deterministic sites on real MB_ surfaces.

    Computed from metro.py's own constants rather than ray-cast, because
    every metro member is an axis-aligned box or a 16-gon column whose face
    positions are known exactly. The one curved case is the pier column,
    where the flat of a 16-gon sits at R*cos(pi/16), not R -- placing at R
    would float the defect about 2 cm off the surface.
    """
    xs = MB.pier_stations()
    spans = [(xs[i], xs[i + 1], i + 1) for i in range(len(xs) - 1)]
    # keep defects inside the research zone, as the road bridge's are
    spans = [s for s in spans
             if P.RESEARCH_X0 <= (s[0] + s[1]) / 2.0 <= P.RESEARCH_X1]
    piers = [(x, i + 1) for i, x in enumerate(xs)
             if P.RESEARCH_X0 <= x <= P.RESEARCH_X1]

    out = {k: [] for k in TYPES}
    for kind, (_base, where) in TYPES.items():
        n = COUNTS[kind]
        for j in range(n):
            if where in ("WEB", "SOFFIT", "BOX_INTERIOR", "JOINT"):
                x0, x1, si = spans[(j * 7 + 3) % len(spans)]
                d = MB.span_depth(x0, x1)
                soffit = MB.DECK_TOP_Z - d
                u = rnd.uniform(0.2, 0.8)
                x = x0 + (x1 - x0) * u
                s = rnd.choice([1, -1])
                if where == "WEB":
                    pos = Vector((x, MB.Y + s * (MB.WEB_GAUGE / 2.0
                                                 + MB.WEB_T / 2.0),
                                  soffit + d * 0.45))
                    nrm = Vector((0, s, 0))
                    tan = Vector((1, 0, 0))
                    host = "METRO_BOX_WEB"
                    obj = f"MB_WEB_{si:03d}_{'W1' if s > 0 else 'W2'}"
                elif where == "SOFFIT":
                    pos = Vector((x, MB.Y + rnd.uniform(-2.5, 2.5), soffit))
                    nrm = Vector((0, 0, -1))
                    tan = Vector((1, 0, 0))
                    host = "METRO_BOX_SOFFIT"
                    obj = f"MB_DECK_SOFFIT_{si:03d}"
                elif where == "BOX_INTERIOR":
                    # inside the box, on the underside of the top slab
                    pos = Vector((x, MB.Y + rnd.uniform(-2.0, 2.0),
                                  MB.DECK_TOP_Z - MB.TOP_SLAB_T))
                    nrm = Vector((0, 0, -1))
                    tan = Vector((1, 0, 0))
                    host = "METRO_BOX_INTERIOR"
                    obj = f"MB_DECK_TOP_{si:03d}"
                else:                      # JOINT
                    pos = Vector((x0 + 0.06, MB.Y + rnd.uniform(-3.0, 3.0),
                                  MB.DECK_TOP_Z - 0.25))
                    nrm = Vector((0, 0, 1))
                    tan = Vector((0, 1, 0))
                    host = "METRO_SEGMENTAL_JOINT"
                    obj = f"MB_JOINT_{si:03d}"
                occ = 0.55 if where == "BOX_INTERIOR" else \
                    0.30 if where == "SOFFIT" else 0.15
            elif where == "BEARING":
                x, pi_ = piers[(j * 5 + 2) % len(piers)]
                s = rnd.choice([1, -1])
                dd = MB.span_depth(x, x + MB.SPAN)
                zb = MB.DECK_TOP_Z - dd - MB.BEARING[2] / 2.0
                pos = Vector((x, MB.Y + s * MB.WEB_GAUGE / 2.0
                              + MB.BEARING[1] / 2.0, zb))
                nrm = Vector((0, s, 0))
                tan = Vector((1, 0, 0))
                host = "METRO_BEARING"
                obj = f"MB_BEARING_{pi_:03d}_{'B1' if s > 0 else 'B2'}"
                occ = 0.45
            else:                          # PIER
                x, pi_ = piers[(j * 3 + 1) % len(piers)]
                ang = rnd.uniform(0, 2 * math.pi)
                r = MB.PIER_D / 2.0 * _APOTHEM
                gz = 0.0
                zc = rnd.uniform(gz + 4.0, MB.DECK_TOP_Z - 8.0)
                pos = Vector((x + r * math.cos(ang),
                              MB.Y + r * math.sin(ang), zc))
                nrm = Vector((math.cos(ang), math.sin(ang), 0)).normalized()
                tan = Vector((0, 0, 1))
                host = "METRO_PIER_COLUMN"
                obj = f"MB_PIER_COL_{pi_:03d}"
                occ = 0.20
            out[kind].append(DM.Site(pos, nrm, tan, host, obj,
                                     f"{where.lower()} of the metro viaduct",
                                     occlusion=occ, u_max=1.6, v_max=1.2))
    return out


def _rename(ob, new):
    """Rename a defect object and the material the maker named after it."""
    old = ob.name
    for slot in ob.data.materials:
        if slot is not None and slot.name == f"MAT_{old}":
            slot.name = f"MAT_{new}"
    ob.name = new
    for ch in list(bpy.data.objects):
        if ch.name.startswith(old + "_"):
            ch.name = new + ch.name[len(old):]
    return ob


def build(colls, mats, log=print):
    """Populate the metro with its own defects. Returns (records, counts)."""
    rnd = random.Random(P.SEED + 137)
    coll = colls.get("AVIAN_METRO_DEFECTS") or colls["AVIAN_DEFECTS"]
    sites = _sites(rnd, log)

    records = []
    counts = {}
    n_extra = [0]
    idx = 0
    for kind, (base, where) in TYPES.items():
        made = 0
        for s in sites[kind]:
            idx += 1
            sev = rnd.choice([1, 2, 2, 3, 3, 4])
            if base in ("SPALL", "DELAMINATION", "REBAR_EXPOSED"):
                # make_spall returns (ob, info, extra, cut). `cut` is the
                # cutter mesh damage.py later booleans into the host to
                # carve the cavity.
                #
                # LIMITATION, recorded rather than hidden: the metro's
                # spalls are NOT carved into their host. The spall face
                # itself is real geometry with real relief -- which is what
                # a depth sensor measures -- but the host member keeps its
                # unbroken surface behind it. damage.py carves because its
                # sites come from a ray-snap onto a 16-sided prism, where
                # the cavity is what makes the defect sit flush; the metro's
                # sites are computed analytically on exact box faces, so
                # nothing here needs the boolean. Skipping it also keeps 38
                # box-girder spans out of a boolean solver that already has
                # a failure path in damage.py's own carve step.
                ob, info, extra, _cut = DM.make_spall(
                    idx, base, s, sev, rnd, coll, mats)
                n_extra[0] += len(extra)
            else:
                ob, info = DM.make_crack(idx, base, s, sev, rnd, coll, mats)
            _rename(ob, f"MDEFECT_{kind}_{idx:03d}")

            rec = {
                "defect_id": ob.name,
                "type": kind,
                "base_type": base,
                "severity": sev,
                "position_m": [round(v, 3) for v in s.pos],
                "surface_normal": [round(v, 3) for v in s.normal],
                "host_surface": s.host,
                "host_object": s.obj_name,
                "bridge_section": "METRO_VIADUCT",
                "inspection_sector": msector_of(s.pos.x),
                "occlusion": round(s.occlusion, 2),
                "placement_rationale": s.reason,
                "structure": "METRO",
            }
            rec.update(info)
            records.append(rec)
            ML.set_custom(ob, {
                "avi_defect_id": ob.name,
                "avi_type": kind,
                "avi_base_type": base,
                "avi_severity": sev,
                "avi_structure": "METRO",
                "avi_repairable": bool(info["repairable"]),
                "avi_reason": info["reason"],
                "avi_host_surface": s.host,
                "avi_host_object": s.obj_name,
                "avi_sector": rec["inspection_sector"],
                "avi_occlusion": s.occlusion,
                "avi_representation": info["representation"],
            })
            made += 1
        counts[kind] = made

    # ---- instance ids, class ids and surface offset ---------------------
    # Found by comparing field sets: the metro records had 45 fields where
    # the road bridge's have 46, missing instance_id, class_index and
    # surface_offset_mm. The first two come from dataset.assign_indices,
    # which only walks the road-bridge records; without them the metro is
    # absent from instance segmentation entirely, so a detector trained on
    # the dataset could never be scored on it.
    #
    # Metro instances occupy 501..999, inside dataset.py's defect band
    # (1..999) and clear of the road bridge's 1..192, so neither can
    # collide with the other or with STRUCTURE_INDEX_BASE at 1000.
    import dataset as DS
    for i, rec in enumerate(records, start=1):
        idx = METRO_INDEX_BASE + i
        rec["instance_id"] = idx
        rec["class_index"] = DS.CLASS_INDEX.get(rec["base_type"], 0)
        # The makers stand a shader decal 6 mm off the host face; geometry
        # defects sit on it. Recorded rather than left absent, because the
        # road bridge measures this during its ray-snap and the metro's
        # sites are placed analytically, so there is nothing to measure.
        rec["surface_offset_mm"] = (
            6.0 if rec["representation"] == "SHADER_DECAL" else 0.0)
        ob = bpy.data.objects.get(rec["defect_id"])
        if ob is not None:
            ob.pass_index = idx
            ob["avi_instance_id"] = idx
            ob["avi_class_index"] = rec["class_index"]
            for ch in bpy.data.objects:
                if ch.name.startswith(rec["defect_id"] + "_"):
                    ch.pass_index = idx
                    ch["avi_instance_id"] = idx

    bpy.context.view_layer.update()
    log(f"  mdamage : {len(records)} metro defects (+{n_extra[0]} child "
        f"objects) across {len(counts)} "
        f"types ({', '.join(f'{k.split(chr(95))[0]}={v}' for k, v in counts.items())})")
    return records, counts


def msector_of(x):
    """MSECTOR_A..F over the research zone, mirroring the road sectors."""
    if not (P.RESEARCH_X0 <= x <= P.RESEARCH_X1):
        return "OUTSIDE_RESEARCH_ZONE"
    i = int((x - P.RESEARCH_X0) / P.SECTOR_LENGTH)
    i = max(0, min(5, i))
    return f"MSECTOR_{'ABCDEF'[i]}"


def export_ground_truth(records, path_json, path_csv):
    """Metro ground truth, exported through damage.py's own writer.

    Same writer, so the metro file has the same shape and the same field
    discipline as the road bridge's -- and a field added to one cannot
    silently be missing from the other.
    """
    return DM.export_ground_truth(records, path_json, path_csv)
