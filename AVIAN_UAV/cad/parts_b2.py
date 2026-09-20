"""AVIAN equipment, manipulator, tools and liquid-service geometry."""
from __future__ import annotations
import math
import numpy as np
import cadquery as cq

import params_b as P
from helpers import (T, RX, RY, RZ, chain, tube, rbox, fins, cone_tip,
                     swept_hose, helical_spring)
import kin_b as K


# ===========================================================================
# 03_BATTERY  --  cartridge + 3-position indexed mount
# ===========================================================================
def battery_cartridge():
    b = rbox(P.vBattL, P.vBattW, P.vBattH, 6.0)
    b = b.cut(cq.Workplane("XY").box(P.vBattL + 2, P.vBattW + 2, 1.2))
    b = b.union(cq.Workplane("XY", origin=(-P.vBattL / 2 - 8, 0, -6)).box(16, 44, 30))
    b = b.union(cq.Workplane("XY", origin=(-P.vBattL / 2 - 6, 0, P.vBattH / 2 - 14))
                .box(12, 24, 12))
    b = b.union(cq.Workplane("XY", origin=(P.vBattL / 2 - 30, 0, P.vBattH / 2 + 4))
                .box(52, 30, 8))
    for sy in (+1, -1):
        b = b.union(cq.Workplane("XY", origin=(0, sy * (P.vBattW / 2 + 3), -P.vBattH / 2 + 10))
                    .box(P.vBattL - 34, 6, 13))
    for k in range(5):
        for sy in (+1, -1):
            b = b.cut(cq.Workplane("XZ", origin=(-64 + k * 32, sy * P.vBattW / 2, 14))
                      .rect(18, 5).extrude(-8 * sy))
    return b


def battery_rail():
    """3-position indexed mount. Replaces the deleted powered CG rail."""
    # Two longitudinal rails per pack + two lateral ties. NOT a full plate --
    # the cartridge shell is the stiff member; the tray only reacts vertical
    # load and carries the 3-position index holes.
    z = P.vBattZ - P.vBattH / 2 - P.vRailT / 2
    RW = 24.0
    r = None
    for sy in (+1, -1):
        for si in (+1, -1):
            y = sy * (P.vBattY + si * (P.vBattW / 2 - 6))
            s = cq.Workplane("XY", origin=(0, y, z)).box(
                P.vBattL + 70, RW, P.vRailT)
            r = s if r is None else r.union(s)
    # Ties span each pack only. A full-width tie would cross the service-fluid
    # cartridge below the spine (CHECK 7).
    for sx in (+1, -1):
        for sy in (+1, -1):
            r = r.union(cq.Workplane(
                "XY", origin=(sx * (P.vBattL / 2 + 23), sy * P.vBattY, z)).box(
                    RW, P.vBattW + 12, P.vRailT))
    for sy in (+1, -1):
        for k in (-1, 0, 1):
            r = r.cut(tube(9, 0, 20, z - 10).translate(
                (k * P.vRailTravel, sy * (P.vBattY + P.vBattW / 2 + 4), 0)))
        for kk in (0, 1):
            y = sy * (P.vBattY + (P.vBattW / 2 + 5) * (1 if kk else -1))
            r = r.union(cq.Workplane("XY", origin=(0, y, z + 12)).box(
                P.vBattL + 14, 5, 22))
    return r


def battery_latch(sy):
    y = sy * P.vBattY
    return (cq.Workplane("XY", origin=(-P.vBattL / 2 - 22, y, P.vBattZ)).box(12, 42, 34)
            .union(cq.Workplane("XY", origin=(-P.vBattL / 2 - 32, y, P.vBattZ + 11))
                   .box(18, 16, 10)))


# ===========================================================================
# 04_AVIONICS
# ===========================================================================
def flight_controller():
    b = rbox(P.vFCL, P.vFCW, P.vFCH, 3.0)
    b = b.union(tube(32, 0, 6, P.vFCH / 2))
    for sy in (+1, -1):
        b = b.union(cq.Workplane("XY", origin=(P.vFCL / 2 - 2, sy * 14, 0)).box(5, 20, 10))
    return b


def fc_tray():
    t = cq.Workplane("XY", origin=(0, 0, -P.vFCH / 2 - 16)).box(P.vFCL + 28, P.vFCW + 26, 3)
    for sx in (+1, -1):
        for sy in (+1, -1):
            t = t.union(tube(15, 4, 13, -P.vFCH / 2 - 13).translate(
                (sx * (P.vFCL + 10) / 2, sy * (P.vFCW + 8) / 2, 0)))
    return t


def companion_computer():
    b = rbox(P.vCCL, P.vCCW, P.vCCH, 3.0)
    b = b.union(cq.Workplane("XY", origin=(-P.vCCL / 2 + 3, 0, 0)).box(5, P.vCCW * 0.7, P.vCCH * 0.6))
    for k, y in enumerate((-24, 0, 24)):
        b = b.union(cq.Workplane("XY", origin=(P.vCCL / 2 - 1, y, -4)).box(4, 16, 8))
    return b


def compute_cooling():
    base = cq.Workplane("XY", origin=(0, 0, P.vCCH / 2 + 2)).box(P.vCCL, P.vCCW, 4)
    # CHECK 5a: fin height and fan station are set by the top service panel
    # at vPanelTopZ; the whole stack must clear it so the panel lifts off.
    f = fins(13, P.vCCL - 8, 2.0, 14, (P.vCCW - 10) / 12).translate((0, 0, P.vCCH / 2 + 14))
    fan = (tube(54, 46, 12).translate((P.vCCL / 2 - 32, 0, P.vCCH / 2 + 16))
           .union(tube(14, 0, 12).translate((P.vCCL / 2 - 32, 0, P.vCCH / 2 + 16))))
    return base.union(f).union(fan)


def pdb():
    b = rbox(P.vPDBL, P.vPDBW, P.vPDBH, 4.0)
    for sy in (+1, -1):
        b = b.union(cq.Workplane("XY", origin=(-P.vPDBL / 2 + 14, sy * 28, P.vPDBH / 2 + 6))
                    .box(20, 20, 12))
    for i in range(4):
        a = math.radians(45 + 90 * i)
        b = b.union(tube(11, 0, 9).translate((42 * math.cos(a), 32 * math.sin(a), P.vPDBH / 2)))
    b = b.union(cq.Workplane("XY", origin=(P.vPDBL / 2 - 22, 0, P.vPDBH / 2 + 5)).box(32, 28, 10))
    return b


def small_box(l, w, h, conn=0.0):
    b = rbox(l, w, h, 2.2)
    if conn:
        b = b.union(cq.Workplane("XY", origin=(l / 2 - 1, 0, 0)).box(5, conn, h * 0.55))
    return b


def antenna(bend=0.0):
    a = tube(8, 0, 112).union(tube(13, 0, 14, -14))
    return a.rotate((0, 0, 0), (0, 1, 0), bend) if bend else a


# ===========================================================================
# 05_PERCEPTION
# ===========================================================================
def lidar():
    b = tube(P.vLidarD, 0, P.vLidarH, -P.vLidarH / 2)
    b = b.cut(tube(P.vLidarD + 2, P.vLidarD - 6, 26, -13))
    b = b.union(tube(P.vLidarD - 6, 0, 26, -13))
    b = b.union(tube(P.vLidarD - 12, 0, 6, P.vLidarH / 2))
    b = b.union(tube(P.vLidarD + 6, 0, 5, -P.vLidarH / 2 - 5))
    return b


def lidar_mast():
    m = tube(P.vLidarMastD, P.vLidarMastD - 5, 96, -96 - P.vLidarH / 2)
    m = m.union(tube(P.vLidarMastD + 16, 0, 8, -104 - P.vLidarH / 2))
    return m


def gimbal():
    yaw = tube(50, 0, 28, -28).union(tube(42, 0, 10, -38))
    roll = cq.Workplane("XY", origin=(0, 0, -54)).box(32, 70, 28)
    yoke = None
    for sy in (+1, -1):
        arm = cq.Workplane("XY", origin=(0, sy * 42, -82)).box(22, 11, 62)
        yoke = arm if yoke is None else yoke.union(arm)
    yoke = yoke.union(cq.Workplane("XY", origin=(0, 0, -54)).box(22, 95, 11))
    cam = rbox(70, 58, 54, 5.0).translate((0, 0, -102))
    lens = (tube(38, 0, 32).rotate((0, 0, 0), (0, 1, 0), 90).translate((35, 0, -102))
            .union(tube(32, 0, 4).rotate((0, 0, 0), (0, 1, 0), 90).translate((67, 0, -102))))
    return yaw.union(roll).union(yoke).union(cam).union(lens)


def gimbal_boom(sy):
    gx, gy, gz = P.vGimbalPos
    b = (tube(P.vGimbalBoomOD, P.vGimbalBoomOD - 3, P.vGimbalBoomL)
         .rotate((0, 0, 0), (0, 1, 0), 90)
         .translate((gx - P.vGimbalBoomL, sy * abs(gy), gz + 14)))
    b = b.union(cq.Workplane("XY", origin=(gx - P.vGimbalBoomL + 8, sy * abs(gy), gz + 14))
                .box(16, 40, 28))
    return b


def depth_camera():
    b = rbox(P.vDepthL, P.vDepthW, P.vDepthH, 3.0)
    for y, d in ((-34, 13), (-8, 10), (12, 8), (34, 13)):
        b = b.cut(cq.Workplane("YZ", origin=(P.vDepthL / 2 - 1.5, y, 0)).circle(d / 2).extrude(4))
    b = b.union(cq.Workplane("XY", origin=(-P.vDepthL / 2 - 3, 0, 0)).box(6, 28, P.vDepthH - 6))
    return b


def gnss_antenna():
    a = tube(P.vGnssD, 0, P.vGnssH, -P.vGnssH / 2).faces(">Z").fillet(4.0)
    return a.union(tube(P.vGnssD - 14, 0, 5, -P.vGnssH / 2 - 5))


def gnss_mast():
    return tube(P.vGnssMastD, P.vGnssMastD - 4, 62, -62)


def lamp_cluster():
    out = None
    for i in range(3):
        a = math.radians(90 + 120 * i)
        p = tube(42, 0, 24, -12).translate((23 * math.cos(a), 23 * math.sin(a), 0))
        p = p.cut(tube(34, 0, 4, -13).translate((23 * math.cos(a), 23 * math.sin(a), 0)))
        out = p if out is None else out.union(p)
    return out.union(tube(88, 0, 7, 12))


# ===========================================================================
# 10_LANDING_GEAR
# ===========================================================================
def gear_leg(sx, sy):
    x = sx * P.vGearLegX
    y0, y1 = sy * P.vGearLegRootY, sy * P.vGearTrack / 2
    z0, z1 = -152.0, P.vGearGround + P.vGearSkidOD / 2
    ln = math.hypot(y1 - y0, z1 - z0)
    ang = math.degrees(math.atan2(y1 - y0, -(z1 - z0)))
    leg = tube(P.vGearLegOD, P.vGearLegID, ln)
    leg = leg.rotate((0, 0, 0), (1, 0, 0), 180 + ang).translate((x, y0, z0))
    lug = cq.Workplane("XY", origin=(x, y0, z0 + 8)).box(42, 30, 16)
    return leg.union(lug)


def gear_damper(sx, sy):
    y = sy * (P.vGearLegRootY + P.vGearTrack / 2) / 2
    z = (-152.0 + P.vGearGround) / 2
    d = tube(P.vGearDamperD, 0, P.vGearDamperH, -P.vGearDamperH / 2)
    ang = math.degrees(math.atan2(sy * (P.vGearTrack / 2 - P.vGearLegRootY),
                                  -(P.vGearGround - (-152.0))))
    return d.rotate((0, 0, 0), (1, 0, 0), 180 + ang).translate((sx * P.vGearLegX, y, z))


def gear_skid(sy):
    y = sy * P.vGearTrack / 2
    z = P.vGearGround + P.vGearSkidOD / 2
    return (tube(P.vGearSkidOD, P.vGearSkidID, P.vGearSkidLen)
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .translate((-P.vGearSkidLen / 2, y, z)))


def gear_foot(sx, sy):
    x = sx * (P.vGearSkidLen / 2 - 46)
    y = sy * P.vGearTrack / 2
    f = tube(P.vGearFootD, 0, P.vGearFootH, P.vGearGround).translate((x, y, 0))
    return f.cut(tube(P.vGearSkidOD + 0.3, 0, 120).rotate((0, 0, 0), (0, 1, 0), 90)
                 .translate((x - 60, y, P.vGearGround + P.vGearSkidOD / 2)))


# ===========================================================================
# 06_MANIPULATOR  --  every link a separate solid in its own frame
# ===========================================================================
def _housing(d, w, axis="Y", wall=3.2):
    """Thin machined shell around the actuator, NOT a solid billet."""
    h = tube(d, d - 2 * wall, w, -w / 2)
    h = h.union(tube(d, d * 0.42, 3.0, w / 2 - 3.0)).union(tube(d, d * 0.42, 3.0, -w / 2))
    if axis == "Y":
        h = h.rotate((0, 0, 0), (1, 0, 0), 90)
    elif axis == "X":
        h = h.rotate((0, 0, 0), (0, 1, 0), 90)
    for s in (+1, -1):
        c = tube(d * 0.6, d * 0.6 - 7, 6, w / 2)
        if axis == "Y":
            c = c.rotate((0, 0, 0), (1, 0, 0), 90 * s)
        elif axis == "X":
            c = c.rotate((0, 0, 0), (0, 1, 0), 90 * s)
        elif s < 0:
            c = tube(d * 0.6, 0, 6, -w / 2 - 6)
        h = h.union(c)
    return h


def _hardstop(d, w, ang):
    return (cq.Workplane("XY").box(11, w * 0.5, 13).translate((0, 0, -(d / 2 + 4)))
            .rotate((0, 0, 0), (0, 1, 0), ang))


def _cablerun(length, d, xo):
    return tube(d, d - 2.2, length, -length).translate((xo, 0, 0))


def _hose2(length, sep, xo):
    o = None
    for sy in (+1, -1):
        h = tube(P.vHoseOD, P.vHoseID, length, -length).translate((xo, sy * sep / 2, 0))
        o = h if o is None else o.union(h)
    return o


def _clip(din, ln, z):
    return tube(din + 7, din + 0.4, ln, z - ln / 2)


def arm_base_flange():
    f = tube(P.vJ1D + 16, P.vJ1D - 8, 6, -6)
    for i in range(P.vHubBoltN):
        a = math.radians(22.5 + 360.0 / P.vHubBoltN * i)
        f = f.cut(tube(5.4, 0, 14, -12).translate(
            (P.vHubBoltPCD / 2 * math.cos(a), P.vHubBoltPCD / 2 * math.sin(a), 0)))
    # CHECK 7: barrel length is bounded by the J2 housing swing radius
    # (vJ2D/2) below the base -- a longer skirt clashes with the shoulder.
    f = f.union(tube(P.vJ1D, P.vJ1D - 8, 34, -40))
    f = f.union(tube(P.vJ1D - 7, P.vJ1D - 28, 6, -46))
    f = f.union(cq.Workplane("XY", origin=(-P.vJ1D / 2 - 5, 0, -26)).box(15, 38, 20))
    for sy in (+1, -1):
        f = f.union(tube(12, 0, 15).rotate((0, 0, 0), (0, 1, 0), -90)
                    .translate((-P.vJ1D / 2 - 15, sy * 12, -26)))
    return f


def link1():
    L = P.vL0
    b = tube(P.vJ1D - 15, P.vJ1D - 22, L - 20, -(L - 20))
    b = b.union(tube(P.vJ1D - 6, P.vJ1D - 30, 8, -8))
    for sy in (+1, -1):
        yk = cq.Workplane("XY", origin=(0, sy * (P.vJ2W / 2 + 6), -L + 11)).box(
            P.vJ2D + 5, 8, 38)
        yk = yk.cut(tube(26, 0, 20, -L + 1).rotate((0, 0, 0), (1, 0, 0), 90)
                    .translate((0, sy * (P.vJ2W / 2 + 16), -L + 11)))
        b = b.union(yk)
    cross = cq.Workplane("XY", origin=(0, 0, -L + 28)).box(
        P.vJ2D + 5, P.vJ2W + 20, 6)
    for sy2 in (+1, -1):
        cross = cross.cut(tube(30, 0, 20, -L + 18).translate((0, sy2 * 40, 0)))
    b = b.union(cross)
    return b.union(_cablerun(L - 22, 10.0, -(P.vJ1D / 2 - 4)))


def link2():
    L = P.vL1
    h = _housing(P.vJ2D, P.vJ2W, "Y")
    h = h.union(_hardstop(P.vJ2D, P.vJ2W, 118)).union(_hardstop(P.vJ2D, P.vJ2W, -118))
    col = tube(P.vLinkTube1OD + 11, P.vLinkTube1OD - 1, 32, -44)
    tb = tube(P.vLinkTube1OD, P.vLinkTube1ID, L - 88, -(L - 44))
    cl = None
    for sy in (+1, -1):
        a = cq.Workplane("XY", origin=(0, sy * (P.vJ3W / 2 + 5), -L + 19)).box(P.vJ3D + 4, 8, 38)
        cl = a if cl is None else cl.union(a)
    cl = cl.union(tube(P.vLinkTube1OD + 9, P.vLinkTube1OD - 1, 28, -(L - 15)))
    b = h.union(col).union(tb).union(cl)
    b = b.union(_cablerun(L - 58, 10.0, P.vLinkTube1OD / 2 + 6))
    b = b.union(_hose2(L - 58, 14.0, -(P.vLinkTube1OD / 2 + 6)))
    for k in range(3):
        b = b.union(_clip(P.vLinkTube1OD, 8, -58 - k * (L - 125) / 2))
    return b


def link3():
    L = P.vL2
    h = _housing(P.vJ3D, P.vJ3W, "Y")
    h = h.union(_hardstop(P.vJ3D, P.vJ3W, 150)).union(_hardstop(P.vJ3D, P.vJ3W, -150))
    col = tube(P.vLinkTube2OD + 10, P.vLinkTube2OD - 1, 28, -38)
    tb = tube(P.vLinkTube2OD, P.vLinkTube2ID, L - 76, -(L - 38))
    cl = None
    for sy in (+1, -1):
        a = cq.Workplane("XY", origin=(0, sy * (P.vJ4W / 2 + 4), -L + 15)).box(P.vJ4D + 4, 7, 28)
        cl = a if cl is None else cl.union(a)
    cl = cl.union(tube(P.vLinkTube2OD + 8, P.vLinkTube2OD - 1, 24, -(L - 13)))
    b = h.union(col).union(tb).union(cl)
    b = b.union(_cablerun(L - 52, 9.0, P.vLinkTube2OD / 2 + 5.5))
    b = b.union(_hose2(L - 52, 13.0, -(P.vLinkTube2OD / 2 + 5.5)))
    for k in range(2):
        b = b.union(_clip(P.vLinkTube2OD, 7, -56 - k * (L - 115)))
    return b


def link4():
    """J4 wrist ROLL -- axis along the link, per the AVIAN brief."""
    L = P.vL3
    h = _housing(P.vJ4D, P.vJ4W, "Y")
    b = h.union(tube(P.vJ5D + 5, P.vJ5D - 1, L - 20, -(L - 11)))
    b = b.union(tube(P.vJ4D - 8, P.vJ4D - 22, 5, -13))
    for sy in (+1, -1):
        b = b.union(cq.Workplane("XY", origin=(0, sy * (P.vJ5W / 2 + 4), -L + 11))
                    .box(P.vJ5D + 4, 8, 24))
    return b.union(_hose2(L - 14, 11.0, -(P.vJ4D / 2 + 3)))


def link5():
    L = P.vL4
    h = _housing(P.vJ5D, P.vJ5W, "Y")
    # NOTE: tube OD must NOT equal vJ5D -- equal-radius perpendicular cylinders
    # meet tangentially and OCCT returns an invalid solid from the fuse.
    b = h.union(tube(P.vJ6D + 2, P.vJ6D - 3, L - 18, -(L - 9)))
    cl, cw, ch = 34.0, 30.0, 26.0
    cam = rbox(cl, cw, ch, 3.0).cut(rbox(cl - 5, cw - 5, ch - 5, 2.0)).translate(
        (P.vJ5D / 2 + cl / 2 - 3, 0, -L / 2))
    lens = tube(18, 0, 9).translate((P.vJ5D / 2 + cl / 2 - 3, 0, -L / 2 - ch / 2 - 4))
    return b.union(cam).union(lens).union(_hose2(L - 12, 11.0, -(P.vJ5D / 2 + 3)))


def link6():
    L = P.vL5
    h = _housing(P.vJ6D, P.vJ6W, "X")
    b = h.union(tube(P.vJ6D - 4, P.vJ6D - 10, L - 22, -(L - 7)))
    fl = tube(P.vFlangeD, 16, P.vFlangeT, -L)
    for i in range(6):
        a = math.radians(30 + 60 * i)
        fl = fl.cut(tube(4.2, 0, P.vFlangeT + 2, -L - 1)
                    .translate((17 * math.cos(a), 17 * math.sin(a), 0)))
    b = b.union(fl)
    for sy in (+1, -1):
        b = b.union(tube(11, P.vHoseID, L - 9, -(L - 3)).translate((0, sy * 12, 0)))
    return b


# ===========================================================================
# 07_TOOL_INTERFACE + 08_REPAIR_TOOLS
# ===========================================================================
def ft_sensor():
    d, h = P.vFTSD, P.vFTSH
    b = tube(d, 0, h, -h)
    for i in range(3):
        a = math.radians(60 + 120 * i)
        b = b.cut(tube(11, 0, h + 2, -h - 1).translate(
            ((d / 2 - 8) * math.cos(a), (d / 2 - 8) * math.sin(a), 0)))
    b = b.cut(tube(15, 0, h - 6, -h + 3))
    return b.union(cq.Workplane("XY", origin=(d / 2 + 3, 0, -h / 2)).box(11, 18, 8))


def tc_master():
    d, h = P.vTCMasterD, P.vTCMasterH
    m = tube(d, 0, h, -h).union(tube(d + 8, 0, 5, -5))
    m = m.cut(tube(d + 10, d - 6, 4, -h + 5))
    for i in range(P.vTCPinN):
        a = math.radians(90 + 120 * i)
        m = m.union(tube(P.vTCPinD, 0, 9, -h - 9).translate(
            (15 * math.cos(a), 15 * math.sin(a), 0)))
    m = m.union(tube(12, P.vHoseID, 11, -h - 11).translate((0, 14, 0)))
    m = m.union(cq.Workplane("XY", origin=(d / 2 - 6, 0, -h - 4)).box(12, 22, 8))
    return m


def tc_tool_plate():
    d, h = P.vTCToolD, P.vTCToolH
    t = tube(d, 0, h, -h).union(tube(d + 10, 0, 4, 0))
    for i in range(P.vTCPinN):
        a = math.radians(90 + 120 * i)
        t = t.cut(tube(P.vTCPinD + 0.4, 0, 10, -1).translate(
            (15 * math.cos(a), 15 * math.sin(a), 0)))
    t = t.cut(tube(12.4, 0, 12, -1).translate((0, 14, 0)))
    return t


def nozzle_tool():
    z = -P.vTCToolH
    b = tube(P.vNozzleBodyD, 0, P.vNozzleBodyL, z - P.vNozzleBodyL)
    b = b.union(tube(P.vNozzleBodyD + 6, 0, 6, z - 6))
    for i in range(4):
        a = math.radians(45 + 90 * i)
        b = b.cut(cq.Workplane("XY").box(8, 8, 12)
                  .rotate((0, 0, 0), (0, 0, 1), math.degrees(a))
                  .translate(((P.vNozzleBodyD / 2 + 3) * math.cos(a),
                              (P.vNozzleBodyD / 2 + 3) * math.sin(a),
                              z - P.vNozzleBodyL + 14)))
    zt = z - P.vNozzleBodyL
    return b.union(cone_tip(P.vNozzleTipD, 9.0, P.vNozzleTipL, zt - P.vNozzleTipL))


def gripper_tool():
    """Parallel two-jaw repair gripper."""
    z = -P.vTCToolH
    body = tube(P.vGripBodyD, 0, P.vGripBodyL, z - P.vGripBodyL)
    body = body.union(tube(P.vGripBodyD + 7, 0, 6, z - 6))
    body = body.union(cq.Workplane("XY", origin=(P.vGripBodyD / 2 + 4, 0, z - 26))
                      .box(10, 20, 22))
    zj = z - P.vGripBodyL
    out = body
    for sy in (+1, -1):
        y = sy * P.vGripStroke / 2
        car = cq.Workplane("XY", origin=(0, y, zj - 9)).box(30, 16, 18)
        jaw = cq.Workplane("XY", origin=(0, y, zj - 9 - P.vGripJawL / 2)).box(
            26, 11, P.vGripJawL)
        pad = cq.Workplane("XY", origin=(0, y - sy * 6, zj - 9 - P.vGripJawL + 16)).box(
            22, 5, 30)
        for k in range(4):
            pad = pad.cut(cq.Workplane("XY", origin=(0, y - sy * 6,
                                                     zj - 22 - P.vGripJawL + 10 + k * 7))
                          .box(24, 7, 2.2))
        out = out.union(car).union(jaw).union(pad)
    return out


# ===========================================================================
# 09_LIQUID_SERVICE
# ===========================================================================
def tank_shell():
    t = P.vTankWall
    s = rbox(P.vTankL, P.vTankW, P.vTankH, 12.0)
    s = s.cut(rbox(P.vTankL - 2 * t, P.vTankW - 2 * t, P.vTankH - 2 * t, 10.0))
    # Filler neck and vent are on the AFT face, not the top: the top of the
    # tank is the floor of the power-distribution shelf (CHECK 7).
    # Filler neck + vent on the +Y side face, reached with the side service
    # panel off. The top face is the power-distribution shelf and the aft face
    # is the pump/filter bay, so neither is available (CHECK 7 / 11).
    s = s.union(tube(40, 30, 10).rotate((0, 0, 0), (1, 0, 0), -90)
                .translate((-30, P.vTankW / 2 + 2, P.vTankH / 2 - 30)))
    s = s.union(tube(46, 0, 4).rotate((0, 0, 0), (1, 0, 0), -90)
                .translate((-30, P.vTankW / 2 + 6, P.vTankH / 2 - 30)))
    s = s.union(tube(9, 0, 8).rotate((0, 0, 0), (1, 0, 0), -90)
                .translate((30, P.vTankW / 2 + 6, P.vTankH / 2 - 8)))
    # CHECK 5b: the drain/outlet exits the AFT face, not the belly. A port
    # protruding below the lower service panel fouls its removal corridor and
    # is exposed to ground strike on a hard landing.
    s = s.union(tube(22, 0, 16).rotate((0, 0, 0), (0, 1, 0), -90)
                .translate((-P.vTankL / 2 - 16, 0, -P.vTankH / 2 + 22)))
    # sight strip on the aft face -- CHECK 4a keeps the side faces inside the
    # battery extraction corridor
    s = s.union(cq.Workplane("XY", origin=(-P.vTankL / 2 + 1, 0, 0))
                .box(4, 12, P.vTankH - 20))
    return s


def tank_baffles():
    o = None
    for k in range(P.vBaffleN):
        x = -P.vTankL / 2 + (k + 1) * P.vTankL / (P.vBaffleN + 1)
        b = cq.Workplane("XY", origin=(x, 0, 0)).box(3.0, P.vTankW - 6, P.vTankH - 6)
        for iy in (-1, 0, 1):
            for iz in (-1, 1):
                b = b.cut(tube(26, 0, 10, -5).rotate((0, 0, 0), (0, 1, 0), 90)
                          .translate((x - 5, iy * 28, iz * 24)))
        o = b if o is None else o.union(b)
    return o


def tank_fluid(litres):
    t = P.vTankWall
    inner_a = (P.vTankL - 2 * t) * (P.vTankW - 2 * t)
    h = min(P.vTankH - 2 * t, litres * 1e6 / inner_a)
    return cq.Workplane("XY", origin=(0, 0, -P.vTankH / 2 + t + h / 2)).box(
        P.vTankL - 2 * t, P.vTankW - 2 * t, h)


def pump():
    """Diaphragm pump, mounted UPRIGHT on its motor.

    Laid on its side the pump is 104 mm across including the motor, and the
    aft equipment bay is only 120 mm wide between the two battery extraction
    corridors -- that left no room for the filter or the valves. Standing it
    up puts the 104 mm on the z axis, where the bay is 173 mm deep, and frees
    the whole upper shelf (CHECK 8 / place.py).
    """
    body = rbox(P.vPumpW, P.vPumpW, P.vPumpL, 6.0)
    head = tube(44, 0, 18, P.vPumpL / 2 - 2)
    for sy in (+1, -1):
        head = head.union(
            tube(13, 7, 14).rotate((0, 0, 0), (1, 0, 0), 90)
            .translate((0, sy * 22, P.vPumpL / 2 + 8)))
    mot = tube(46, 0, 24, -P.vPumpL / 2 - 24)
    mot = mot.union(tube(30, 0, 5, -P.vPumpL / 2 - 29))
    foot = None
    for sx in (+1, -1):
        f = cq.Workplane("XY", origin=(sx * (P.vPumpW / 2 + 5), 0,
                                       -P.vPumpL / 2 + 8)).box(10, 40, 5)
        foot = f if foot is None else foot.union(f)
    return body.union(head).union(mot).union(foot)


def filter_unit():
    f = tube(P.vFilterD, 0, P.vFilterL, -P.vFilterL / 2)
    f = f.union(tube(P.vFilterD + 8, 0, 9, P.vFilterL / 2 - 9))
    for sz in (+1, -1):
        f = f.union(tube(10, 0, 12, sz * P.vFilterL / 2))
    return f


def valve(d=26.0, l=42.0):
    v = tube(d, 0, l, -l / 2).rotate((0, 0, 0), (0, 1, 0), 90)
    v = v.union(tube(10, 0, 13).rotate((0, 0, 0), (0, 1, 0), 90).translate((l / 2, 0, 0)))
    v = v.union(tube(10, 0, 13).rotate((0, 0, 0), (0, 1, 0), -90).translate((-l / 2, 0, 0)))
    v = v.union(cq.Workplane("XY", origin=(0, 0, d / 2 + 5)).box(16, 16, 12))
    return v


def hose_run(pts):
    return swept_hose(pts, P.vHoseOD, P.vHoseID)


def cartridge_frame():
    """Removable service-fluid cartridge frame.

    Carries the tank, pump, filter, check valve, relief valve and pressure
    sensor as one line-replaceable unit. Retained by two quarter-turn latches
    on the aft face and located by two tapered pins at the forward face, so it
    drops out downward once the lower service panel is removed.
    """
    L, W, H = P.vCartL, P.vCartW, P.vCartH
    t = 3.0
    f = None
    # four longerons
    for sy in (+1, -1):
        for sz in (+1, -1):
            r = cq.Workplane("XY", origin=(0, sy * (W / 2 - t / 2),
                                           sz * (H / 2 - 6))).box(L, t, 12)
            f = r if f is None else f.union(r)
    # end frames
    for sx in (+1, -1):
        ring = (cq.Workplane("XY", origin=(sx * (L / 2 - t / 2), 0, 0))
                .box(t, W, H)
                .cut(cq.Workplane("XY", origin=(sx * (L / 2 - t / 2), 0, 0))
                     .box(t + 2, W - 24, H - 24)))
        f = f.union(ring)
    # mid frame between tank and equipment bay
    xm = -L / 2 + P.vCartBayL
    f = f.union(cq.Workplane("XY", origin=(xm, 0, 0)).box(t, W, H)
                .cut(cq.Workplane("XY", origin=(xm, 0, 0))
                     .box(t + 2, W - 30, H - 30)))
    # locating pins forward, latch lugs aft
    for sy in (+1, -1):
        f = f.union(cone_tip(12, 6, 16, 0).rotate((0, 0, 0), (0, 1, 0), 90)
                    .translate((L / 2, sy * 30, 0)))
        f = f.union(cq.Workplane("XY", origin=(-L / 2 - 6, sy * 30, 0))
                    .box(12, 22, 26))
    # lift handle recessed into the AFT face: a handle proud of the belly
    # plane fouls the lower service panel corridor (CHECK 11)
    f = f.union(cq.Workplane("XY", origin=(-L / 2 - 5, 0, -H / 2 + 26))
                .box(10, 70, 16))
    return f
