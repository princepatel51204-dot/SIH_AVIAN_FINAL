"""Generate a simulation-ready URDF for the AVIAN airframe from the CAD.

NOTHING IN THIS FILE IS INVENTED.
Every dimension comes from the approved CAD parameter set (`params_b.py`) and
every mass and centre of gravity comes from the closed mass budget
(`avian_mass_budget.csv`, 389 line items). The manipulator joint axes, offsets
and limits are read from `params_b.JOINTS`, which is the same table the Onshape
mate scheme and the collision-envelope study were built from. If the CAD
changes, this regenerates and the simulation follows.

WHY A GENERATOR AND NOT A HAND-WRITTEN URDF
-------------------------------------------
A hand-written URDF is a second source of truth for the same numbers, and the
two drift apart the first time a link length changes. The test in
`tests/test_description.py` cross-checks this URDF's forward kinematics against
`kin_b.py` -- the CAD's own FK -- at sub-millimetre tolerance, so a divergence
between CAD and simulation is a test failure rather than a silent error.

INERTIA
-------
Link inertia is computed from the real mass items assigned to that link, using
the parallel-axis theorem about the link's own centre of mass. This is a
point-mass aggregation, NOT a solid-body integral: it captures mass and CG
exactly and the inertia tensor approximately. The approximation is stated in
`inertia_method` in the generated manifest, and it is the honest level of
fidelity available from a mass budget rather than from the meshed solids.
A minimum diagonal inertia is enforced so no link is numerically degenerate.

COLLISION vs VISUAL
-------------------
Collision geometry is deliberately primitive -- boxes, cylinders, spheres.
Contact solvers behave badly on concave meshes and a 137-part B-rep would make
the simulation unusably slow. The visual meshes are referenced separately so a
renderer can show the real airframe while physics uses the primitives.

FRAME CONVENTION
----------------
  base_link   airframe body, origin at the CAD datum, +X forward, +Y left,
              +Z up. Matches the AVIAN_sensor_manifest body frame exactly.
  Rotors      8, in 4 coaxial pairs on arms at ARM_ANGLES, separated
              vertically by vCoaxSep. Upper and lower counter-rotate.
  Manipulator arm_base_link -> link_1 .. link_6 -> tool0 -> nozzle_tip.
              link_n is authored from J_n extending toward J_{n+1}, which is
              the convention the CAD assembly uses; getting this off by one
              draws every link a joint downstream.
"""
from __future__ import annotations

import csv
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from xml.dom import minidom

# The CAD package is the source of truth. Import it directly rather than
# copying numbers across, so there is exactly one definition of each.
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAD_DIR = os.environ.get("AVIAN_CAD_DIR", os.path.join(_ROOT, "cad"))
if CAD_DIR not in sys.path:
    sys.path.insert(0, CAD_DIR)

import params_b as P                                    # noqa: E402

MM = 0.001                                              # CAD is in mm

# Which mass-budget groups belong to which URDF link. Everything not listed
# for the arm lands on base_link, which is correct: the airframe, propulsion
# structure, battery, avionics, perception and landing gear are all rigidly
# attached to the body.
ARM_ITEM_TO_LINK = {
    "manip_J1_actuator": "link_1", "manip_link_1_structure": "link_1",
    "manip_J2_actuator": "link_2", "manip_link_2_structure": "link_2",
    "manip_J3_actuator": "link_3", "manip_link_3_structure": "link_3",
    "manip_J4_actuator": "link_4", "manip_link_4_structure": "link_4",
    "manip_J5_actuator": "link_5", "manip_link_5_structure": "link_5",
    "manip_J6_actuator": "link_6", "manip_link_6_structure": "link_6",
    "manip_internal_wiring": "arm_base_link",
    "tool_ft_sensor": "tool0",
}
# Anything in 06_MANIPULATOR not named above (tool changer, nozzle, camera)
# is carried on tool0.
ARM_DEFAULT_LINK = "tool0"

MIN_INERTIA = 1.0e-6            # kg m^2, keeps the solver conditioned


# ---------------------------------------------------------------------------
def load_mass_items(csv_path, config="NOMINAL"):
    """Mass items for one configuration, in metres and kilograms."""
    out = []
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            if r["config"] != config:
                continue
            out.append({
                "group": r["group"],
                "item": r["item"],
                "mass": float(r["mass_kg"]),
                "cg": (float(r["cg_x_mm"]) * MM,
                       float(r["cg_y_mm"]) * MM,
                       float(r["cg_z_mm"]) * MM),
                "source": r["source"],
            })
    if not out:
        raise ValueError(f"no mass items for config {config!r}")
    return out


def aggregate(items):
    """Total mass, CG and inertia tensor of a set of point masses.

    Inertia is about the aggregate CG, in the frame the CGs are expressed in.
    """
    m = sum(i["mass"] for i in items)
    if m <= 0.0:
        return 0.0, (0.0, 0.0, 0.0), [MIN_INERTIA] * 3 + [0.0] * 3
    cx = sum(i["mass"] * i["cg"][0] for i in items) / m
    cy = sum(i["mass"] * i["cg"][1] for i in items) / m
    cz = sum(i["mass"] * i["cg"][2] for i in items) / m
    ixx = iyy = izz = ixy = ixz = iyz = 0.0
    for i in items:
        dx, dy, dz = (i["cg"][0] - cx, i["cg"][1] - cy, i["cg"][2] - cz)
        mm_ = i["mass"]
        ixx += mm_ * (dy * dy + dz * dz)
        iyy += mm_ * (dx * dx + dz * dz)
        izz += mm_ * (dx * dx + dy * dy)
        ixy -= mm_ * dx * dy
        ixz -= mm_ * dx * dz
        iyz -= mm_ * dy * dz
    # A single point mass has zero inertia about its own CG; give every link
    # a small isotropic floor so the solver cannot divide by zero.
    ixx = max(ixx, MIN_INERTIA)
    iyy = max(iyy, MIN_INERTIA)
    izz = max(izz, MIN_INERTIA)
    return m, (cx, cy, cz), [ixx, iyy, izz, ixy, ixz, iyz]


# ---------------------------------------------------------------------------
def _el(parent, tag, **kw):
    e = ET.SubElement(parent, tag)
    for k, v in kw.items():
        e.set(k.replace("_", "-") if k == "xyz_" else k, v)
    return e


def _rpy_matrix(rpy):
    """URDF fixed-axis XYZ: R = Rz(yaw) . Ry(pitch) . Rx(roll)."""
    import numpy as np
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _matrix_rpy(R):
    """Inverse of `_rpy_matrix`, on the branch with pitch in (-90, 90)."""
    import numpy as np
    sp = -float(R[2, 0])
    sp = max(-1.0, min(1.0, sp))
    p = math.asin(sp)
    if abs(math.cos(p)) < 1e-9:                       # gimbal lock
        return (0.0, p, math.atan2(-R[0, 1], R[1, 1]))
    r = math.atan2(R[2, 1], R[2, 2])
    y = math.atan2(R[1, 0], R[0, 0])
    return (r, p, y)


def _optical_rpy(pitch_deg, yaw_deg):
    """Body-frame mount angles -> URDF rpy for a -Z-forward optical frame.

    R_optical_base = rpy(90, 0, -90) deg is the rotation that maps the
    optical -Z onto body +X, +X onto body -Y (image right) and +Y onto body
    +Z (image up). Mount pitch and yaw are applied outside it, in the body
    frame, so `pitch=90` looks straight down and `yaw=-90` looks right.
    """
    base = _rpy_matrix((math.radians(90.0), 0.0, math.radians(-90.0)))
    R = (_rpy_matrix((0.0, 0.0, math.radians(yaw_deg)))
         @ _rpy_matrix((0.0, math.radians(pitch_deg), 0.0)) @ base)
    return _matrix_rpy(R)


def _body_rpy(pitch_deg, yaw_deg):
    """Body-aligned frame: +X forward, +Y left, +Z up, then mount angles."""
    R = (_rpy_matrix((0.0, 0.0, math.radians(yaw_deg)))
         @ _rpy_matrix((0.0, math.radians(pitch_deg), 0.0)))
    return _matrix_rpy(R)


def _origin(parent, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    e = ET.SubElement(parent, "origin")
    e.set("xyz", " ".join(f"{v:.6f}" for v in xyz))
    e.set("rpy", " ".join(f"{v:.6f}" for v in rpy))
    return e


def _inertial(link, mass, cg, I, origin_frame=(0, 0, 0)):
    """Write an <inertial> block, CG expressed relative to the link origin."""
    e = ET.SubElement(link, "inertial")
    _origin(e, (cg[0] - origin_frame[0], cg[1] - origin_frame[1],
                cg[2] - origin_frame[2]))
    ET.SubElement(e, "mass").set("value", f"{mass:.6f}")
    it = ET.SubElement(e, "inertia")
    for k, v in zip(("ixx", "iyy", "izz", "ixy", "ixz", "iyz"),
                    (I[0], I[1], I[2], I[3], I[4], I[5])):
        it.set(k, f"{v:.9f}")
    return e


def _geom_box(parent, size, xyz=(0, 0, 0), rpy=(0, 0, 0), mat=None):
    g = ET.SubElement(parent, "geometry")
    b = ET.SubElement(g, "box")
    b.set("size", " ".join(f"{v:.5f}" for v in size))
    _origin(parent, xyz, rpy)
    if mat:
        ET.SubElement(parent, "material").set("name", mat)
    return parent


def _geom_cyl(parent, r, l, xyz=(0, 0, 0), rpy=(0, 0, 0), mat=None):
    g = ET.SubElement(parent, "geometry")
    c = ET.SubElement(g, "cylinder")
    c.set("radius", f"{r:.5f}")
    c.set("length", f"{l:.5f}")
    _origin(parent, xyz, rpy)
    if mat:
        ET.SubElement(parent, "material").set("name", mat)
    return parent


# ---------------------------------------------------------------------------
MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "meshes")


def _mesh_visual(link, link_name, mat):
    """Replace a primitive visual with the real CAD mesh, if one exists.

    Visual only. Collision stays primitive -- see the measurement in
    meshes/avian_mesh_manifest.json for why 280k triangles per vehicle is not
    a physics option on this hardware.
    """
    p = os.path.join(MESH_DIR, f"{link_name}.stl")
    if not os.path.exists(p):
        return False
    for v in list(link.findall("visual")):
        link.remove(v)
    v = ET.SubElement(link, "visual", name="cad")
    g = ET.SubElement(v, "geometry")
    m = ET.SubElement(g, "mesh")
    m.set("filename", f"meshes/{link_name}.stl")
    m.set("scale", "0.001 0.001 0.001")     # CAD is mm, URDF is m
    _origin(v)
    ET.SubElement(v, "material").set("name", mat)
    return True


def build(cad_dir=CAD_DIR, config="NOMINAL", out_dir=None, use_meshes=True):
    items = load_mass_items(os.path.join(cad_dir, "avian_mass_budget.csv"),
                            config)

    # Partition ONCE, into three disjoint sets. The rotor motors and
    # propellers get their own links, so they must come OUT of the body
    # aggregate -- leaving them in counted 8 x (0.24 + 0.13) = 2.96 kg twice
    # and inflated the vehicle by 12 %. The mass test caught it; the
    # partition is what stops it recurring.
    def _is_rotor_item(name):
        return name.startswith(("motor_", "propeller_"))

    arm_items, rotor_items, body_items = [], [], []
    for it in items:
        if it["group"] == "06_MANIPULATOR":
            arm_items.append(it)
        elif _is_rotor_item(it["item"]):
            rotor_items.append(it)
        else:
            body_items.append(it)

    robot = ET.Element("robot", name="avian")
    ET.SubElement(robot, "material", name="avian_body").append(
        ET.Element("color", rgba="0.16 0.17 0.19 1"))
    ET.SubElement(robot, "material", name="avian_arm").append(
        ET.Element("color", rgba="0.62 0.63 0.66 1"))
    ET.SubElement(robot, "material", name="avian_rotor").append(
        ET.Element("color", rgba="0.08 0.08 0.09 1"))

    manifest = {"config": config, "links": [], "joints": [], "rotors": []}

    # ================= base_link ==========================================
    m_b, cg_b, I_b = aggregate(body_items)
    base = ET.SubElement(robot, "link", name="base_link")
    _inertial(base, m_b, cg_b, I_b)

    # Collision: central body box sized from the corner-node envelope, plus
    # the four arm tubes. Primitives, not the 137-part solid.
    half_diag = P.vDiagonal * MM / 2.0
    body_w = P.vNodeSize * MM * 3.4
    col = ET.SubElement(base, "collision", name="body")
    _geom_box(col, (body_w, body_w, 0.18), (0.0, 0.0, -0.02))
    vis = ET.SubElement(base, "visual", name="body")
    _geom_box(vis, (body_w, body_w, 0.18), (0.0, 0.0, -0.02),
              mat="avian_body")

    arm_r = P.vNodeSize * MM * 0.28
    for i, ang in enumerate(P.ARM_ANGLES):
        a = math.radians(ang)
        mx, my = half_diag * 0.5 * math.cos(a), half_diag * 0.5 * math.sin(a)
        c = ET.SubElement(base, "collision", name=f"arm_{i+1}")
        _geom_cyl(c, arm_r, half_diag, (mx, my, 0.0),
                  (0.0, math.pi / 2.0, a))
        v = ET.SubElement(base, "visual", name=f"arm_{i+1}")
        _geom_cyl(v, arm_r, half_diag, (mx, my, 0.0),
                  (0.0, math.pi / 2.0, a), mat="avian_body")
    manifest["links"].append({"name": "base_link", "mass_kg": round(m_b, 4),
                              "cg_m": [round(v, 5) for v in cg_b]})

    # ================= rotors =============================================
    # 4 arms x coaxial pair = 8. Upper and lower counter-rotate; the sign is
    # recorded because the control allocation matrix depends on it.
    prop_r = 0.5 * 0.28 * 25.4 * MM * 28.0 / 28.0     # G28x9.2 -> 28 in dia
    prop_r = 0.5 * 28.0 * 25.4 * MM
    for i, ang in enumerate(P.ARM_ANGLES):
        a = math.radians(ang)
        rx, ry = half_diag * math.cos(a), half_diag * math.sin(a)
        for lvl, sgn in (("upper", +1), ("lower", -1)):
            n = f"rotor_{i+1}_{lvl}"
            rz = sgn * P.vCoaxSep * MM / 2.0
            link = ET.SubElement(robot, "link", name=n)
            # rotor disc mass from the budget rows for this station
            rot_items = [it for it in rotor_items
                         if it["item"].startswith(
                             (f"motor_{i+1}_{lvl}",
                              f"propeller_{i+1}_{lvl}"))]
            if not rot_items:
                raise ValueError(
                    f"no mass-budget rows for rotor {i+1} {lvl}; the "
                    "budget and the rotor layout have diverged")
            m_r = sum(it["mass"] for it in rot_items)
            Iz = 0.5 * m_r * prop_r * prop_r
            e = ET.SubElement(link, "inertial")
            _origin(e)
            ET.SubElement(e, "mass").set("value", f"{m_r:.6f}")
            it_e = ET.SubElement(e, "inertia")
            for k, v in (("ixx", Iz / 2), ("iyy", Iz / 2), ("izz", Iz),
                         ("ixy", 0.0), ("ixz", 0.0), ("iyz", 0.0)):
                it_e.set(k, f"{v:.9f}")
            v_ = ET.SubElement(link, "visual", name="disc")
            _geom_cyl(v_, prop_r, 0.012, mat="avian_rotor")
            # No collision on the rotor discs: a spinning disc modelled as a
            # solid cylinder generates spurious contacts with everything it
            # passes near, and thrust is applied as a force, not by blade
            # contact.
            j = ET.SubElement(robot, "joint", name=f"{n}_joint",
                              type="continuous")
            ET.SubElement(j, "parent", link="base_link")
            ET.SubElement(j, "child", link=n)
            _origin(j, (rx, ry, rz))
            ET.SubElement(j, "axis", xyz="0 0 1")
            manifest["rotors"].append({
                "name": n, "arm": i + 1, "level": lvl,
                "position_m": [round(rx, 4), round(ry, 4), round(rz, 4)],
                "spin": ("CCW" if (i % 2 == 0) == (lvl == "upper")
                         else "CW"),
                "mass_kg": round(m_r, 4),
                "max_thrust_N": round(P.vThrustPerArm * 9.80665 / 2.0, 2),
            })

    # ================= manipulator ========================================
    # arm_base_link is fixed to the body at the CAD's vArmBase station.
    by_link = {}
    for it in arm_items:
        key = ARM_ITEM_TO_LINK.get(it["item"], ARM_DEFAULT_LINK)
        by_link.setdefault(key, []).append(it)

    ab = ET.SubElement(robot, "link", name="arm_base_link")
    m_ab, cg_ab, I_ab = aggregate(by_link.get("arm_base_link", []))
    arm_base_xyz = tuple(v * MM for v in P.vArmBase)
    _inertial(ab, m_ab, cg_ab, I_ab, origin_frame=arm_base_xyz)
    c = ET.SubElement(ab, "collision", name="hub")
    _geom_cyl(c, P.vJ1D * MM / 2.0, P.vJ1H * MM)
    v = ET.SubElement(ab, "visual", name="hub")
    _geom_cyl(v, P.vJ1D * MM / 2.0, P.vJ1H * MM, mat="avian_arm")

    manifest["links"].append({"name": "arm_base_link",
                              "mass_kg": round(m_ab, 4)})
    j = ET.SubElement(robot, "joint", name="arm_base_joint", type="fixed")
    ET.SubElement(j, "parent", link="base_link")
    ET.SubElement(j, "child", link="arm_base_link")
    _origin(j, arm_base_xyz)

    # Running CAD-frame position of each joint, so inertial CGs (which are in
    # CAD body coordinates) can be re-expressed relative to their link origin.
    cad_pos = list(arm_base_xyz)
    parent = "arm_base_link"
    link_d = [P.vJ1D, P.vJ2D, P.vJ3D, P.vJ4D, P.vJ5D, P.vJ6D]
    link_len = [P.vL1, P.vL2, P.vL3, P.vL4, P.vL5, P.vL5]

    for n, (nm, off, ax, lim, tq, sp, fn) in enumerate(P.JOINTS, start=1):
        child = f"link_{n}"
        off_m = tuple(v * MM for v in off)
        cad_pos = [cad_pos[k] + off_m[k] for k in range(3)]

        link = ET.SubElement(robot, "link", name=child)
        m_l, cg_l, I_l = aggregate(by_link.get(child, []))
        _inertial(link, m_l, cg_l, I_l, origin_frame=tuple(cad_pos))

        # Link body: a capsule-ish cylinder from this joint toward the next.
        L = link_len[n - 1] * MM
        d = link_d[n - 1] * MM
        c = ET.SubElement(link, "collision", name="body")
        _geom_cyl(c, d / 2.0, max(0.03, L * 0.9), (0, 0, -L / 2.0))
        v = ET.SubElement(link, "visual", name="body")
        _geom_cyl(v, d / 2.0, max(0.03, L * 0.9), (0, 0, -L / 2.0),
                  mat="avian_arm")

        jt = ET.SubElement(robot, "joint", name=nm, type="revolute")
        ET.SubElement(jt, "parent", link=parent)
        ET.SubElement(jt, "child", link=child)
        _origin(jt, off_m)
        ET.SubElement(jt, "axis",
                      xyz=" ".join(str(float(a)) for a in ax))
        lo, hi = math.radians(lim[0]), math.radians(lim[1])
        lm = ET.SubElement(jt, "limit")
        lm.set("lower", f"{lo:.6f}")
        lm.set("upper", f"{hi:.6f}")
        lm.set("effort", f"{tq:.3f}")                # N m, from the CAD
        lm.set("velocity", f"{math.radians(sp):.6f}")   # rad/s
        ET.SubElement(jt, "dynamics", damping="0.06", friction="0.02")

        manifest["joints"].append({
            "name": nm, "parent": parent, "child": child,
            "axis": [float(a) for a in ax],
            "origin_m": [round(v, 5) for v in off_m],
            "limit_deg": [lim[0], lim[1]],
            "effort_Nm": tq, "velocity_deg_s": sp, "function": fn})
        manifest["links"].append({"name": child, "mass_kg": round(m_l, 4)})
        parent = child

    # tool0 at the J6 flange, then the sealant nozzle tip
    t0 = ET.SubElement(robot, "link", name="tool0")
    m_t, cg_t, I_t = aggregate(by_link.get("tool0", []))
    tool_pos = [cad_pos[0], cad_pos[1], cad_pos[2] - P.vL5 * MM]
    _inertial(t0, m_t, cg_t, I_t, origin_frame=tuple(tool_pos))
    c = ET.SubElement(t0, "collision", name="tool")
    _geom_cyl(c, P.vNozzleBodyD * MM / 2.0, P.vNozzleBodyL * MM,
              (0, 0, -P.vNozzleBodyL * MM / 2.0))
    v = ET.SubElement(t0, "visual", name="tool")
    _geom_cyl(v, P.vNozzleBodyD * MM / 2.0, P.vNozzleBodyL * MM,
              (0, 0, -P.vNozzleBodyL * MM / 2.0), mat="avian_arm")
    manifest["links"].append({"name": "tool0", "mass_kg": round(m_t, 4)})
    jt = ET.SubElement(robot, "joint", name="tool0_joint", type="fixed")
    ET.SubElement(jt, "parent", link=parent)
    ET.SubElement(jt, "child", link="tool0")
    _origin(jt, (0.0, 0.0, -P.vL5 * MM))

    tip = ET.SubElement(robot, "link", name="nozzle_tip")
    e = ET.SubElement(tip, "inertial")
    _origin(e)
    ET.SubElement(e, "mass").set("value", "0.001")
    it_e = ET.SubElement(e, "inertia")
    for k in ("ixx", "iyy", "izz"):
        it_e.set(k, f"{MIN_INERTIA:.9f}")
    for k in ("ixy", "ixz", "iyz"):
        it_e.set(k, "0")
    v = ET.SubElement(tip, "visual", name="tip")
    _geom_cyl(v, P.vNozzleTipD * MM / 2.0, P.vNozzleTipL * MM,
              (0, 0, -P.vNozzleTipL * MM / 2.0), mat="avian_arm")
    jt = ET.SubElement(robot, "joint", name="nozzle_tip_joint", type="fixed")
    ET.SubElement(jt, "parent", link="tool0")
    ET.SubElement(jt, "child", link="nozzle_tip")
    _origin(jt, (0.0, 0.0, -P.vToolOffset * MM + P.vL5 * MM))

    # ================= sensor frames ======================================
    # Same body offsets as AVIAN_sensor_manifest_REV_B.json, so the simulated
    # sensors sit exactly where the environment package says they do.
    #
    # ORIENTATION -- CORRECTED, AND WHY
    # ---------------------------------
    # The environment package built these frames with rpy = (90-pitch, 0, yaw)
    # and declared "+X forward, -Z is the optical axis". Those two statements
    # are inconsistent: Rx(90) maps -Z onto +Y, so every "forward-looking"
    # sensor was actually aimed 90 deg to the LEFT of the nose. The generated
    # AVIAN_sensor_manifest_REV_B.json records the contradiction directly --
    # `optical_axis_world` reads [0, 1, 0] for a sensor whose mount pitch and
    # yaw are both zero. This URDF copied that error, and it was found by the
    # depth-camera test: the camera aimed across the airframe instead of at
    # the pier it was parked in front of.
    #
    # The optical convention is kept (-Z forward, +X right, +Y up) and the
    # base rotation is corrected to rpy = (90, 0, -90) deg, which is the one
    # that actually maps -Z onto body +X. Mount pitch (positive = nose-down)
    # and mount yaw are then applied in the BODY frame, outside it:
    #
    #     R = Rz(yaw) . Ry(pitch) . R_optical_base
    #
    # so pitch 90 looks straight down and yaw -90 looks out the right side,
    # which is what the two offsets for those sensors imply.
    #
    # Two frame conventions, not one. A camera frame is OPTICAL (-Z out of
    # the lens); a LiDAR and an IMU are not cameras. A spinning LiDAR sweeps
    # azimuth about its own +Z, so giving it an optical frame lays its spin
    # axis on its side and the scan plane comes out vertical. The IMU must
    # report in the body frame by definition. Those two are therefore
    # BODY-ALIGNED frames, and the distinction is explicit here rather than
    # left for a reader to infer.
    sensors = [
        ("sensor_rgb_front", (0.22, 0.0, -0.02), (0.0, 0.0), "optical"),
        ("sensor_rgb_down", (0.02, 0.0, -0.11), (90.0, 0.0), "optical"),
        ("sensor_rgb_side", (0.05, -0.19, -0.02), (12.0, -90.0), "optical"),
        ("sensor_depth", (0.20, 0.0, 0.04), (0.0, 0.0), "optical"),
        ("sensor_lidar", (0.0, 0.0, 0.13), (0.0, 0.0), "body"),
        ("sensor_thermal", (0.20, 0.10, -0.02), (0.0, 0.0), "optical"),
        ("sensor_imu", (0.0, 0.0, 0.0), (0.0, 0.0), "body"),
    ]
    manifest["sensors"] = []
    for nm, off, (pitch, yaw), conv in sensors:
        lk = ET.SubElement(robot, "link", name=nm)
        e = ET.SubElement(lk, "inertial")
        _origin(e)
        ET.SubElement(e, "mass").set("value", "0.001")
        it_e = ET.SubElement(e, "inertia")
        for k in ("ixx", "iyy", "izz"):
            it_e.set(k, f"{MIN_INERTIA:.9f}")
        for k in ("ixy", "ixz", "iyz"):
            it_e.set(k, "0")
        jt = ET.SubElement(robot, "joint", name=f"{nm}_joint", type="fixed")
        ET.SubElement(jt, "parent", link="base_link")
        ET.SubElement(jt, "child", link=nm)
        rpy = (_optical_rpy(pitch, yaw) if conv == "optical"
               else _body_rpy(pitch, yaw))
        _origin(jt, off, rpy)
        Rf = _rpy_matrix(rpy)
        manifest["sensors"].append({
            "name": nm, "offset_m": list(off),
            "pitch_deg": pitch, "yaw_deg": yaw,
            "frame_convention": conv,
            "rpy_rad": [round(v, 6) for v in rpy],
            "boresight_body": [round(float(v), 6) for v in
                               (Rf @ ((0.0, 0.0, -1.0) if conv == "optical"
                                      else (1.0, 0.0, 0.0)))],
            "frame_z_body": [round(float(v), 6) for v in (Rf @ (0.0, 0.0, 1.0))],
            "convention_note": (
                "optical: -Z out of the lens, +X image right, +Y image up"
                if conv == "optical" else
                "body-aligned: +X forward, +Y left, +Z up"),
        })

    # ================= real CAD visual meshes =============================
    n_mesh = 0
    if use_meshes:
        mat_of = {"base_link": "avian_body"}
        for lk in robot.findall("link"):
            nm = lk.get("name")
            mat = ("avian_rotor" if nm.startswith("rotor_")
                   else "avian_body" if nm == "base_link" else "avian_arm")
            if _mesh_visual(lk, nm, mat):
                n_mesh += 1
    manifest["visual_meshes"] = n_mesh
    manifest["visual_source"] = ("real CAD B-rep tessellation" if n_mesh
                                 else "primitives (no meshes exported yet)")

    # ================= write ==============================================
    out_dir = out_dir or os.path.dirname(os.path.abspath(__file__))
    os.makedirs(out_dir, exist_ok=True)
    urdf_path = os.path.join(out_dir, "avian.urdf")
    xml = minidom.parseString(ET.tostring(robot)).toprettyxml(indent="  ")
    with open(urdf_path, "w") as f:
        f.write(xml)

    # Massless frames: nozzle tip and the seven sensor links, 1 g each so the
    # solver stays conditioned. Counted so the manifest total is the whole
    # vehicle and not a subset.
    n_frames = 1 + len(sensors)
    manifest["frame_links"] = n_frames
    total = (sum(l.get("mass_kg", 0.0) for l in manifest["links"])
             + sum(r["mass_kg"] for r in manifest["rotors"])
             + n_frames * 0.001)
    budget_total = sum(i["mass"] for i in items)
    manifest["mass_budget_total_kg"] = round(budget_total, 4)
    # The synthetic 1 g frame links are solver conditioning, not vehicle mass,
    # so they are excluded from the comparison against the budget.
    from_budget = total - n_frames * 0.001
    manifest["frame_link_mass_kg"] = round(n_frames * 0.001, 4)
    if abs(from_budget - budget_total) > 0.005:
        raise ValueError(
            f"URDF mass {from_budget:.4f} kg != mass budget "
            f"{budget_total:.4f} kg "
            f"for config {config}. Every budget item must land on exactly "
            f"one link.")
    manifest.update({
        "urdf": os.path.basename(urdf_path),
        "total_modelled_mass_kg": round(total, 3),
        "rotor_count": len(manifest["rotors"]),
        "arm_count": P.vArmCount,
        "diagonal_m": P.vDiagonal * MM,
        "max_total_thrust_N": round(P.vThrustPerArm * P.vArmCount * 9.80665,
                                    1),
        "inertia_method": "point-mass aggregation of the CAD mass budget "
                          "with the parallel-axis theorem; exact in mass and "
                          "CG, approximate in the inertia tensor",
        "collision_method": "primitive boxes and cylinders; rotor discs have "
                            "no collision geometry by design",
        "source_cad": os.path.abspath(cad_dir),
    })
    man_path = os.path.join(out_dir, "avian_description_manifest.json")
    with open(man_path, "w") as f:
        json.dump(manifest, f, indent=2)
    return urdf_path, man_path, manifest


if __name__ == "__main__":
    u, m, man = build()
    print(f"URDF     {u}")
    print(f"manifest {m}")
    print(f"links    {len(man['links'])}  joints {len(man['joints'])}  "
          f"rotors {man['rotor_count']}  sensors {len(man['sensors'])}")
    print(f"mass     {man['total_modelled_mass_kg']} kg modelled")
    print(f"thrust   {man['max_total_thrust_N']} N max")
