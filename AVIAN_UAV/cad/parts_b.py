"""AVIAN geometry. One function per Onshape Part Studio feature group."""
from __future__ import annotations
import math
import numpy as np
import cadquery as cq

import params_b as P
from helpers import (Part, T, RX, RY, RZ, chain, tube, rbox, fins, cone_tip,
                     swept_hose, helical_spring)
import kin_b as K

I4 = np.eye(4)


def _boxtube(l, h, w, t, axis="X"):
    """Hollow rectangular CF box section, centred, running along `axis`."""
    o = cq.Workplane("XY").box(l, w, h)
    if t > 0:
        o = o.cut(cq.Workplane("XY").box(l + 2, w - 2 * t, h - 2 * t))
    return o


# ===========================================================================
# 01_AIRFRAME  --  spine, nodes, hub, panels
# ===========================================================================
def spine_rail(sy):
    r = _boxtube(P.vSpineLen, P.vSpineRailH, P.vSpineRailW, P.vSpineRailT)
    for k in range(7):
        x = -P.vSpineLen / 2 + 55 + k * (P.vSpineLen - 110) / 6
        r = r.cut(cq.Workplane("XZ", origin=(x, 0, 0)).circle(11).extrude(200, both=True))
    return r.translate((0, sy * P.vSpineTrack / 2, 0))


def spine_cross(sx):
    c = _boxtube(P.vSpineTrack - P.vSpineRailW + 6, P.vCrossH, P.vCrossW, 2.0)
    c = c.rotate((0, 0, 0), (0, 0, 1), 90)
    for k in (-1, 1):
        c = c.cut(cq.Workplane("YZ", origin=(0, k * 55, 0)).circle(9).extrude(100, both=True))
    return c.translate((sx * P.vCrossStation, 0, 0))


def corner_node(sx, sy):
    """CNC AL node: joins two rail ends, a cross member and one arm clamp."""
    s = P.vNodeSize
    n = cq.Workplane("XY").box(s, s, P.vSpineRailH + 6)
    n = n.cut(cq.Workplane("XY").box(s - 14, s - 14, P.vSpineRailH - 14))
    a = math.radians(P.vArmAngle0 if (sx > 0 and sy > 0) else
                     180 - P.vArmAngle0 if (sx < 0 and sy > 0) else
                     180 + P.vArmAngle0 if (sx < 0 and sy < 0) else -P.vArmAngle0)
    # arm boss
    boss = (tube(P.vArmClampOD, P.vArmTubeOD + 0.2, 46)
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .rotate((0, 0, 0), (0, 0, 1), math.degrees(a)))
    n = n.union(boss)
    n = n.cut(tube(P.vArmTubeOD + 0.2, 0, 120)
              .rotate((0, 0, 0), (0, 1, 0), 90)
              .rotate((0, 0, 0), (0, 0, 1), math.degrees(a))
              .translate((-60 * math.cos(a), -60 * math.sin(a), 0)))
    # rail + cross pockets
    n = n.cut(cq.Workplane("XY").box(P.vSpineRailW + 0.3, s + 2, P.vSpineRailH + 0.3)
              .rotate((0, 0, 0), (0, 0, 1), 90))
    n = n.cut(cq.Workplane("XY").box(s + 2, P.vCrossW + 0.3, P.vCrossH + 0.3))
    # lightening
    for sz in (-1, 1):
        n = n.cut(cq.Workplane("XY", origin=(0, 0, sz * (P.vSpineRailH / 2 + 3)))
                  .circle(17).extrude(-9 * sz))
    for sy2 in (-1, 1):
        n = n.cut(cq.Workplane("XZ", origin=(0, sy2 * s / 2, 0)).rect(30, 26)
                  .extrude(-12 * sy2))
    return n.translate((sx * P.vCrossStation, sy * P.vSpineTrack / 2, 0))


def manipulator_hub():
    """Closes the manipulator reaction into both spine rails.

    NOT stress-driven: 37.4 Nm on a 118 mm ring gives sigma = 0.5 MPa against
    500 MPa yield.  The section is set by the bearing seat, the bolt pattern and
    torsional stiffness, so the ring is thin and the mass goes into the ties.
    """
    Hh = 48.0
    h = tube(P.vHubD, P.vHubD - 10, Hh, -Hh)                 # 5 mm wall ring
    h = h.union(tube(P.vHubD + 16, P.vHubD - 22, 6, -6))     # mounting flange
    h = h.union(tube(P.vHubD - 4, P.vHubD - 26, 5, -Hh))     # bearing land
    for i in range(P.vHubBoltN):
        a = math.radians(22.5 + 360.0 / P.vHubBoltN * i)
        h = h.cut(tube(5.4, 0, 30, -30).translate(
            (P.vHubBoltPCD / 2 * math.cos(a), P.vHubBoltPCD / 2 * math.sin(a), 0)))
    for sy in (+1, -1):
        y = sy * P.vSpineTrack / 2
        tie = cq.Workplane("XY", origin=(0, y / 2, -4)).box(70, abs(y), 4.5)
        for k in (-1, 0, 1):
            tie = tie.cut(tube(30, 0, 20, -15).translate((k * 23, y * 0.58, 0)))
        h = h.union(tie)
    # harness + fluid pass-through
    h = h.cut(cq.Workplane("XY", origin=(0, 0, -Hh / 2)).box(30, 16, Hh - 14)
              .translate((-P.vHubD / 2 + 5, 0, 0)))
    return h.translate((P.vHubX, 0, P.vHubZ))


def panel_top():
    p = cq.Workplane("XY", origin=(P.vBayX, 0, P.vPanelTopZ)).box(
        P.vPanelTopL, P.vPanelTopW, P.vPanelT)
    for k in range(4):
        for sy in (+1, -1):
            p = p.cut(cq.Workplane("XY", origin=(
                P.vBayX - 90 + k * 60, sy * 74, P.vPanelTopZ)).rect(34, 8).extrude(9, both=True))
    for sx in (+1, -1):
        for sy in (+1, -1):
            p = p.cut(tube(5.2, 0, 20, P.vPanelTopZ - 10).translate(
                (P.vBayX + sx * (P.vPanelTopL / 2 - 16), sy * (P.vPanelTopW / 2 - 16), 0)))
    p = p.union(cq.Workplane("XY", origin=(P.vBayX, 0, P.vPanelTopZ + P.vPanelT)).box(66, 14, 7))
    return p


def panel_bottom():
    p = cq.Workplane("XY", origin=(-20, 0, P.vPanelBotZ)).box(
        P.vPanelBotL, P.vPanelBotW, P.vPanelT)
        # aperture for the manipulator: sized on the J1 housing swing, not the hub
    p = p.cut(tube(P.vHubD + 92, 0, 20, P.vPanelBotZ - 10).translate((P.vHubX, 0, 0)))
    for sx in (+1, -1):
        for sy in (+1, -1):
            p = p.cut(tube(5.2, 0, 20, P.vPanelBotZ - 10).translate(
                (-20 + sx * 128, sy * 100, 0)))
    for k in range(3):
        p = p.cut(cq.Workplane("XY", origin=(-110 + k * 34, 0, P.vPanelBotZ))
                  .rect(16, 150).extrude(9, both=True))
    return p


def panel_side(sy):
    # CHECK 5c: the panel plane is set by the BATTERY envelope, not the spine
    # rail -- the cartridges are the widest item in the bay (|y| = 135).
    y = sy * (P.vBattY + P.vBattW / 2 + 13 + P.vPanelT / 2)
    p = cq.Workplane("XY", origin=(-10, y, -60)).box(
        P.vPanelSideL, P.vPanelT, P.vPanelSideH)
    for k in range(5):
        p = p.cut(cq.Workplane("XZ", origin=(-160 + k * 76, y, -60)).rect(36, 9)
                  .extrude(12, both=True))
    return p


# ===========================================================================
# 02_PROPULSION  --  arm, coaxial mount, motor, propeller, ESC
# ===========================================================================
def arm_tube():
    ln = P.vRMotor - P.vArmRootR
    return (tube(P.vArmTubeOD, P.vArmTubeID, ln)
            .rotate((0, 0, 0), (0, 1, 0), 90).translate((P.vArmRootR, 0, 0)))


def arm_clamp():
    c = (tube(P.vArmTubeOD + 11, P.vArmTubeOD + 0.15, P.vArmClampL)
         .rotate((0, 0, 0), (0, 1, 0), 90).translate((P.vArmRootR - 6, 0, 0)))
    c = c.cut(cq.Workplane("XY", origin=(P.vArmRootR + 28, 0, (P.vArmTubeOD + 11) / 2))
              .box(P.vArmClampL + 4, P.vArmClampOD + 4, 1.6))
    for x in (P.vArmRootR + 6, P.vArmRootR + 30, P.vArmRootR + 54):
        c = c.cut(cq.Workplane("XZ", origin=(x, 0, 0)).rect(14, 12)
                  .extrude(60, both=True))
    for x in (P.vArmRootR + 12, P.vArmRootR + 48):
        for sy in (+1, -1):
            c = c.cut(tube(3.4, 0, 60, -30).translate((x, sy * 16, 0)))
    return c


def coax_mount():
    """Aluminium yoke carrying the upper and lower motors on one arm end."""
    x = P.vRMotor
    m = cq.Workplane("XY", origin=(x, 0, 0)).box(42, 36, P.vCoaxMountH)
    m = m.cut(cq.Workplane("XY", origin=(x, 0, 0)).box(44, 24, P.vCoaxMountH - 26))
    m = m.cut(tube(P.vArmTubeOD + 0.2, 0, 120).rotate((0, 0, 0), (0, 1, 0), 90)
              .translate((x - 60, 0, 0)))
    for sz in (+1, -1):
        z = sz * P.vCoaxSep / 2
        pl = cq.Workplane("XY", origin=(x, 0, z - sz * 2.0)).box(100, 88, 4)
        pl = pl.edges("|Z").fillet(9)
        pl = pl.cut(tube(P.vMotorShaftD + 14, 0, 20, z - 10).translate((x, 0, 0)))
        for i in range(P.vMotorBoltN):
            a = math.radians(45 + 90 * i)
            pl = pl.cut(tube(4.4, 0, 20, z - 10).translate(
                (x + P.vMotorBoltPCD / 2 * math.cos(a),
                 P.vMotorBoltPCD / 2 * math.sin(a), 0)))
        for sy in (+1, -1):
            pl = pl.cut(tube(26, 0, 20, z - 10).translate((x + sy * 34, 0, 0)))
            pl = pl.cut(tube(22, 0, 20, z - 10).translate((x, sy * 33, 0)))
        m = m.union(pl)
    for sz in (+1, -1):
        m = m.cut(cq.Workplane("XY", origin=(x, 0, sz * 26)).box(30, 60, 26))
    m = m.union(cq.Workplane("XY", origin=(x + 30, 0, 0)).box(8, 26, 14))   # LED pod
    return m


def motor(up=True):
    """PLACEHOLDER ENVELOPE -- T-Motor U8 II KV100 outline, no vendor CAD."""
    s = 1 if up else -1
    base = tube(P.vMotorBaseD, 0, P.vMotorBaseH, 0 if up else -P.vMotorBaseH)
    for i in range(12):
        a = math.radians(30 * i)
        base = base.cut(cq.Workplane("XY").box(9, 26, 7)
                        .rotate((0, 0, 0), (0, 0, 1), math.degrees(a))
                        .translate((36 * math.cos(a), 36 * math.sin(a),
                                    s * P.vMotorBaseH / 2)))
    z0 = P.vMotorBaseH if up else -P.vMotorBaseH - P.vMotorBellH
    bell = tube(P.vMotorBellD, 0, P.vMotorBellH, z0)
    bell = bell.faces(">Z" if up else "<Z").fillet(3.5)
    for i in range(14):
        a = math.radians(360 / 14 * i + 12)
        bell = bell.cut(cq.Workplane("XY").box(7, 24, 17)
                        .rotate((0, 0, 0), (0, 0, 1), math.degrees(a))
                        .translate((40 * math.cos(a), 40 * math.sin(a),
                                    z0 + P.vMotorBellH / 2)))
    zs = z0 + P.vMotorBellH - 3 if up else z0 - 19
    shaft = tube(P.vMotorShaftD, 0, 22, zs)
    for i in range(P.vMotorBoltN):
        a = math.radians(45 + 90 * i)
        base = base.cut(tube(3.6, 0, 30, -15).translate(
            (P.vMotorBoltPCD / 2 * math.cos(a), P.vMotorBoltPCD / 2 * math.sin(a), 0)))
    return base.union(bell).union(shaft)


def propeller(cw=True, up=True):
    R = P.vPropR
    hub = tube(48, P.vMotorShaftD + 0.3, 18, -9)
    for i in range(2):
        a = math.radians(90 * i)
        hub = hub.cut(tube(4.4, 0, 24, -12).translate(
            (19 * math.cos(a), 19 * math.sin(a), 0)))
    sec = [(28, 44, 12.0, 27.0), (0.14*R, 58, 10.0, 24.0), (0.28*R, 70, 8.4, 19.5),
           (0.45*R, 74, 7.0, 14.0), (0.62*R, 68, 5.4, 10.4), (0.78*R, 56, 4.0, 8.3),
           (0.92*R, 40, 2.8, 7.0), (0.999*R, 20, 1.8, 6.4)]
    sgn = 1.0 if cw else -1.0
    wires = []
    for r, c, t, tw in sec:
        tr = math.radians(sgn * tw)
        pl = cq.Plane(origin=cq.Vector(r, -0.030 * r * sgn, 0.010 * r),
                      xDir=cq.Vector(0, math.cos(tr), math.sin(tr)),
                      normal=cq.Vector(1, 0, 0))
        wires.append(cq.Workplane(pl).ellipse(c / 2, max(t / 2, 0.35)).wires().val())
    blade = cq.Workplane("XY").newObject([cq.Solid.makeLoft(wires, ruled=False)])
    p = hub.union(blade).union(blade.rotate((0, 0, 0), (0, 0, 1), 180))
    if not up:
        p = p.rotate((0, 0, 0), (1, 0, 0), 180)
    return p


def esc_module():
    b = rbox(P.vESCL, P.vESCW, P.vESCH, 3.0)
    b = b.union(fins(9, P.vESCL - 10, 1.8, 9.0, (P.vESCW - 8) / 8)
                .translate((0, 0, P.vESCH / 2 + 4.5)))
    b = b.union(cq.Workplane("XY", origin=(-P.vESCL / 2 - 4, 0, 0)).box(8, 30, 12))
    b = b.union(cq.Workplane("XY", origin=(P.vESCL / 2 + 3, 0, 0)).box(6, 22, 10))
    return b


def esc_saddle():
    s = (cq.Workplane("XY").box(P.vESCL - 18, P.vArmTubeOD + 14, 9)
         .cut(tube(P.vArmTubeOD + 0.2, 0, 200).rotate((0, 0, 0), (0, 1, 0), 90)
              .translate((-100, 0, 0))))
    return s


def arm_conduit():
    ln = P.vRMotor - P.vArmRootR - 40
    c = (tube(P.vConduitD, P.vConduitD - 2.6, ln)
         .rotate((0, 0, 0), (0, 1, 0), 90)
         .translate((P.vArmRootR + 20, 0, -P.vArmTubeOD / 2 - P.vConduitD / 2)))
    for k in range(4):
        x = P.vArmRootR + 55 + k * (ln - 70) / 3
        c = c.union(tube(P.vArmTubeOD + 6, P.vArmTubeOD + 0.4, 6)
                    .rotate((0, 0, 0), (0, 1, 0), 90).translate((x, 0, 0)))
    return c
