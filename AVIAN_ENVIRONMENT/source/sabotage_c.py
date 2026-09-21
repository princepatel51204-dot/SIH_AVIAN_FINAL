"""Prove every REV-C check can fail. Working rule 3.10.

WHY THIS EXISTS
---------------
Five checks in this project have now been caught passing for the wrong
reason: S14 compared the depth camera against geometry outside its own field
of view; E05 was vacuous; V33 would have found no pier in the navigation
channel no matter where the piers were, because it read
matrix_world.translation, which meshlib leaves at (0,0,0); the collision
phase reported OK in 4.6 s while dying on a KeyError; and the primitive
count read zero and printed it as informational text.

A green check is evidence of nothing until it has been observed going red.
So for each check, this breaks exactly what that check guards, re-runs it,
and records whether it actually failed. A check that stays green under
sabotage is a broken check and is reported as one.

The mutations are applied in memory and reverted immediately. Nothing here
writes to the scene file or to any ground truth.
"""
from __future__ import annotations

import copy
import json
import os
import tempfile

import bpy

import metro as MB
import validate_c as VDC


def _find(prefix, contains=None):
    for o in bpy.data.objects:
        if o.name.startswith(prefix) and (contains is None
                                          or contains in o.name):
            return o
    return None


def _status(results, cid):
    for r in results:
        if r.id == cid:
            return r.status
    return None


# ---------------------------------------------------------------------------
# Each entry: (check id, what is broken, mutate() -> undo())
# ---------------------------------------------------------------------------
def _sab_v33():
    """Put a pier in the navigation channel."""
    ob = _find("MB_PIER_COL_")
    old = tuple(ob.location)
    b = VDC._bb(ob)
    cx = 0.5 * (b[0] + b[1])
    ob.location = (old[0] + (2100.0 - cx), old[1], old[2])
    bpy.context.view_layer.update()
    return lambda: (setattr(ob, "location", old),
                    bpy.context.view_layer.update())


def _sab_v34():
    """Push a metro member through the 55 m ceiling."""
    ob = _find("MB_STN_ROOF") or _find("MB_MAST_")
    old = tuple(ob.location)
    ob.location = (old[0], old[1], old[2] + 20.0)
    bpy.context.view_layer.update()
    return lambda: (setattr(ob, "location", old),
                    bpy.context.view_layer.update())


def _sab_v35():
    """Squash the inter-structure corridor below 3 m."""
    ob = _find("AVI_INTER_STRUCTURE_")
    old = tuple(ob.scale)
    ob.scale = (old[0], 0.02, old[2])
    bpy.context.view_layer.update()
    return lambda: (setattr(ob, "scale", old),
                    bpy.context.view_layer.update())


def _sab_v36():
    """Strip avi_kind off a metro member."""
    ob = _find("MB_WEB_")
    old = ob.get("avi_kind")
    del ob["avi_kind"]
    return lambda: ob.__setitem__("avi_kind", old)


def _sab_v37(state):
    """Make the collision manifest report zero primitives."""
    path = state["manifest"]
    with open(path) as f:
        real = json.load(f)
    tmp = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
    fake = dict(real)
    fake["collision_primitives"] = 0
    json.dump(fake, tmp)
    tmp.close()
    state["manifest_override"] = tmp.name

    def undo():
        state.pop("manifest_override", None)
        os.unlink(tmp.name)
    return undo


def _sab_v38(state):
    """Remove a required field from a metro ground-truth record."""
    rec = state["mrecords"][0]
    old = rec.pop("defect_background_contrast", None)
    return lambda: rec.__setitem__("defect_background_contrast", old)


def _sab_v39(state):
    """Blank a metro defect's measured visibility."""
    rec = state["mrecords"][1]
    old = rec.get("visible_fraction")
    rec["visible_fraction"] = None
    return lambda: rec.__setitem__("visible_fraction", old)


def _sab_v40(state):
    """Add a 193rd road-bridge defect."""
    recs = state["records"]
    ghost = copy.deepcopy(recs[0])
    ghost["defect_id"] = "DEFECT_GHOST_999"
    recs.append(ghost)
    return lambda: recs.pop()


def _sab_v43(state):
    """Orphan a metro sector by deleting its mission markers."""
    victims = [o for o in bpy.data.objects
               if o.get("avi_object_type") == "MISSION_MARKER"
               and o.get("avi_structure") == "METRO"
               and o.get("avi_sector") == "MSECTOR_C"]
    saved = [(o.name, dict(o.items()), tuple(o.location)) for o in victims]
    for o in victims:
        o["avi_structure"] = "DISABLED_BY_SABOTAGE"

    def undo():
        for name, props, _loc in saved:
            ob = bpy.data.objects.get(name)
            if ob is not None:
                ob["avi_structure"] = props.get("avi_structure", "METRO")
    return undo


SABOTAGES = [
    ("V33", "move a pier into the navigation channel", _sab_v33, False),
    ("V34", "raise a metro member through the 55 m ceiling", _sab_v34, False),
    ("V35", "squash the inter-structure corridor below 3 m", _sab_v35, False),
    ("V36", "strip avi_kind off a metro member", _sab_v36, False),
    ("V37", "make the collision manifest report zero primitives",
     _sab_v37, True),
    ("V38", "remove a required metro ground-truth field", _sab_v38, True),
    ("V39", "blank a metro defect's measured visibility", _sab_v39, True),
    ("V40", "add a 193rd road-bridge defect", _sab_v40, True),
    ("V43", "orphan a metro sector's mission markers", _sab_v43, True),
]


def run(records, mrecords, manifest, log=print):
    """Break each check's subject in turn and confirm it goes red."""
    log("")
    log("  SABOTAGE  (working rule 3.10 -- every check observed failing)")
    state = {"records": records, "mrecords": mrecords, "manifest": manifest}

    base, _ = VDC.run(records, mrecords, log=lambda *a: None,
                      collision_manifest=manifest)
    rows = []
    for cid, what, fn, needs_state in SABOTAGES:
        before = _status(base, cid)
        undo = fn(state) if needs_state else fn()
        try:
            res, _ = VDC.run(state["records"], state["mrecords"],
                             log=lambda *a: None,
                             collision_manifest=state.get(
                                 "manifest_override", manifest))
            after = _status(res, cid)
        finally:
            undo()
        good = (before == "PASS" and after == "FAIL")
        rows.append((cid, what, before, after, good))
        log(f"  [{'OK  ' if good else 'BAD '}] {cid}  {before} -> {after}"
            f"   sabotage: {what}")

    bad = [r for r in rows if not r[4]]
    log(f"  {len(rows) - len(bad)}/{len(rows)} checks observed failing "
        f"under sabotage")
    if bad:
        log(f"  UNPROVEN: {', '.join(r[0] for r in bad)} did not go red -- "
            f"treat as broken checks, not as passing ones")
    return {"total": len(rows), "proven": len(rows) - len(bad),
            "unproven": [r[0] for r in bad],
            "rows": [{"check": c, "sabotage": w, "before": b, "after": a,
                      "proven": g} for c, w, b, a, g in rows]}
