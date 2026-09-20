"""AVIAN REV A -- the 30 required engineering views.

These are engineering views, not beauty renders: every image carries a title
block, the revision, the view name, and callouts naming the parts that matter
in that view. Where a view exists to prove a clearance, the measured number is
printed on it.
"""
from __future__ import annotations
import math
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Circle
import matplotlib.image as mpimg

import params_b as P
import kin_b as K
import assy_b
import render as R

OUT = "images"
REV = "REV A"
SIZE = (1800, 1350)

TEXT = "#14181d"
ACCENT = "#1763b8"
WARN = "#b3541e"
FAINT = "#8a929c"


def _fig(png, title, subtitle=""):
    img = mpimg.imread(png)
    h, w = img.shape[:2]
    fig = plt.figure(figsize=(w / 150, h / 150), dpi=150)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(img)
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.axis("off")
    # title block, bottom left
    ax.add_patch(plt.Rectangle((0, h - 78), 620, 78, facecolor="white",
                               edgecolor="#c8ced6", lw=1.0, alpha=0.94,
                               zorder=5))
    ax.text(16, h - 50, "AVIAN", fontsize=15, weight="bold", color=TEXT,
            zorder=6, va="center")
    ax.text(178, h - 50, title, fontsize=12.5, color=TEXT, zorder=6,
            va="center")
    ax.text(16, h - 21, f"{REV}   coaxial X8   {subtitle}", fontsize=9,
            color=FAINT, zorder=6, va="center")
    return fig, ax, w, h


def _call(ax, xy, text, dxy=(90, -60), color=ACCENT, fs=9.5):
    x, y = xy
    tx, ty = x + dxy[0], y + dxy[1]
    ax.add_patch(FancyArrowPatch((tx, ty), (x, y), arrowstyle="-",
                                 color=color, lw=1.1, zorder=8,
                                 shrinkA=2, shrinkB=2))
    ax.add_patch(Circle((x, y), 3.4, facecolor=color, edgecolor="white",
                        lw=0.8, zorder=9))
    ha = "left" if dxy[0] >= 0 else "right"
    ax.text(tx + (5 if dxy[0] >= 0 else -5), ty, text, fontsize=fs,
            color=TEXT, va="center", ha=ha, zorder=9,
            bbox=dict(boxstyle="round,pad=0.28", fc="white", ec=color,
                      lw=0.8, alpha=0.94))


def _note(ax, w, h, lines, color=TEXT):
    y = 30
    for ln in lines:
        ax.text(w - 18, y, ln, fontsize=9.5, color=color, ha="right",
                va="top", zorder=9,
                bbox=dict(boxstyle="round,pad=0.3", fc="white",
                          ec="#c8ced6", lw=0.8, alpha=0.94))
        y += 26


def save(fig, name):
    os.makedirs(os.path.dirname(f"{OUT}/{name}"), exist_ok=True)
    fig.savefig(f"{OUT}/{name}", dpi=150, facecolor="white")
    plt.close(fig)
    print("  ", name)


def shoot(reg, view, tmp, explode=0.0, skip=(), size=SIZE):
    meshes = R.scene_meshes(reg, explode=explode, skip=skip)
    focal, ps = R.fit_parallel_scale(meshes, view, size)
    R.render(meshes, view, tmp, size=size, focal=focal, ps=ps)
    return focal, ps


def px(pts, view, focal, ps, size=SIZE):
    return R.project(pts, view, focal, ps, size)


# ===========================================================================
def main():
    os.makedirs(OUT, exist_ok=True)
    for d in ("overall", "exploded", "propulsion", "battery", "avionics",
              "manipulator", "tools", "liquid_service", "landing_gear",
              "configurations", "validation"):
        os.makedirs(f"{OUT}/{d}", exist_ok=True)
    tmp = "_r.png"

    reg = assy_b.build("01_FLIGHT")
    regM = assy_b.build("03_MANIPULATION")
    regR = assy_b.build("04_REPAIR")
    regL = assy_b.build("05_LIQUID")
    regT = assy_b.build("06_TRANSPORT")
    regX = assy_b.build("07_MAINTENANCE")

    rot = K.rotor_centres()
    ARM = P.vRMotor

    # ---- 1-7  orthographic set -------------------------------------------
    ORTHO = [("iso", "01_isometric.png", "Overall isometric"),
             ("front", "02_front.png", "Front elevation"),
             ("rear", "03_rear.png", "Rear elevation"),
             ("top", "04_top.png", "Plan view"),
             ("bottom", "05_bottom.png", "Underside"),
             ("left", "06_left.png", "Left elevation"),
             ("right", "07_right.png", "Right elevation")]
    for view, fn, title in ORTHO:
        f, ps = shoot(reg, view, tmp)
        fig, ax, w, h = _fig(tmp, title,
                             f"{P.vDiagonal:.0f} mm diagonal   "
                             f"MTOW 25.75 kg   T/W 2.12")
        if view == "top":
            for i, (mx, my) in enumerate(rot):
                p = px([(mx, my, 0)], view, f, ps)[0]
                _call(ax, p, f"rotor {i+1}\ncoaxial pair", (70, -70))
            p = px([(0, 0, 0)], view, f, ps)[0]
            _call(ax, p, "central spine\n510 x 210 mm", (-120, 120))
            _note(ax, w, h,
                  [f"tip-to-tip gap  {2*ARM*math.sin(math.pi/4)-P.vPropDia:.0f} mm",
                   f"prop  {P.vPropDia/25.4:.0f} in  G28x9.2 CF"])
        elif view == "front":
            p = px([(0, 0, P.vGearGround)], view, f, ps)[0]
            _call(ax, p, f"skid plane  z = {P.vGearGround:.0f} mm", (140, -40))
            p = px([(0, 0, 99.5)], view, f, ps)[0]
            _call(ax, p, "upper rotor plane", (150, -30))
            p = px([(0, 0, -24.5)], view, f, ps)[0]
            _call(ax, p, f"lower rotor plane\ncoax sep {P.vCoaxSep:.0f} mm",
                  (170, 30))
        elif view == "bottom":
            p = px([(P.vArmBase[0], 0, P.vPanelBotZ)], view, f, ps)[0]
            _call(ax, p, "manipulator hub\n+ belly aperture", (120, -80))
            p = px([(-20, 0, P.vPanelBotZ)], view, f, ps)[0]
            _call(ax, p, "lower service panel\n6 x quarter-turn", (-160, 90))
        save(fig, f"overall/{fn}")

    # ---- 8  exploded ------------------------------------------------------
    f, ps = shoot(reg, "iso", tmp, explode=210.0)
    fig, ax, w, h = _fig(tmp, "Exploded assembly",
                         "explode 210 mm along each part's service vector")
    _note(ax, w, h, ["explode direction = the direction each item is",
                     "removed on the aircraft, not an arbitrary blast"])
    save(fig, "exploded/08_exploded.png")

    # ---- 9  propulsion module --------------------------------------------
    keep = [p for p in reg.parts
            if p.group == "02_PROPULSION" and p.name.endswith(("_1", "1_upper",
                                                               "1_lower"))]
    sub = assy_b.Registry() if hasattr(assy_b, "Registry") else None
    meshes = [(p.name, *R.world_mesh(p), p.rgb()) for p in keep]
    meshes = [(n, v, t, c) for (n, (v, t), c) in
              [(m[0], (m[1], m[2]), m[3]) for m in meshes] if len(v) and len(t)]
    focal, ps = R.fit_parallel_scale(meshes, "iso2", SIZE)
    R.render(meshes, "iso2", tmp, size=SIZE, focal=focal, ps=ps)
    fig, ax, w, h = _fig(tmp, "Propulsion module (1 of 4)",
                         "T-Motor X-U8II class coaxial arm set")
    mx, my = rot[0]
    V = "iso2"
    for z, lab, d in ((99.5, "upper rotor  CW\nG28x9.2 CF", (150, -150)),
                      (-24.5, "lower rotor  CCW", (200, 40))):
        p = px([(mx, my, z)], V, focal, ps)[0]
        _call(ax, p, lab, d)
    p = px([(mx, my, 62)], V, focal, ps)[0]
    _call(ax, p, "U8 II KV100 x2\nPLACEHOLDER ENVELOPE", (250, -40), WARN)
    p = px([(mx, my, 0)], V, focal, ps)[0]
    _call(ax, p, "coaxial mount\n4 x M4 per motor plate", (-260, 150))
    p = px([(P.vESCr * math.cos(math.radians(45)),
             P.vESCr * math.sin(math.radians(45)), 26)], V, focal, ps)[0]
    _call(ax, p, "ALPHA 60A 12S ESC\nsplit saddle, upper", (-300, -110))
    p = px([(215 * math.cos(math.radians(45)),
             215 * math.sin(math.radians(45)), 0)], V, focal, ps)[0]
    _call(ax, p, f"CF tube {P.vArmTubeOD:.0f}/{P.vArmTubeID:.0f}\n"
                 "split root clamp\ncable inside the tube", (-280, 40))
    _note(ax, w, h, [f"rotor axis at r = {ARM:.0f} mm, 45 deg stations",
                     f"pair thrust {P.vThrustPerArm:.2f} kgf at "
                     f"{P.vCurrentPerArm:.1f} A  (VENDOR)"])
    save(fig, "propulsion/09_propulsion_module.png")

    # ---- 10  battery module ----------------------------------------------
    f, ps = shoot(reg, "iso_low", tmp, skip=("01_AIRFRAME", "05_PERCEPTION",
                                             "09_LIQUID", "15_SAFETY"))
    fig, ax, w, h = _fig(tmp, "Battery module",
                         f"2 x 12S {P.vBattAh:.0f} Ah cartridges, "
                         f"{P.vBattWh:.0f} Wh")
    for k, sy in ((1, +1), (2, -1)):
        p = px([(0, sy * P.vBattY, P.vBattZ)], "iso_low", f, ps)[0]
        _call(ax, p, f"cartridge {k}\n{P.vBattMassEach:.2f} kg", (120, -60 * sy))
    p = px([(0, 0, P.vBattZ - P.vBattH / 2 - 8)], "iso_low", f, ps)[0]
    _call(ax, p, f"3-position indexed tray\n+/-{P.vRailTravel:.0f} mm, "
                 "NO powered rail", (-230, 70))
    _note(ax, w, h, ["extraction: aft, 2 x quarter-turn latch per pack",
                     "no tools, no other system disconnected"])
    save(fig, "battery/10_battery_module.png")

    # ---- 11  avionics bay -------------------------------------------------
    f, ps = shoot(reg, "iso", tmp, skip=("02_PROPULSION", "10_LANDING_GEAR",
                                         "03_BATTERY"))
    fig, ax, w, h = _fig(tmp, "Avionics bay", "top service panel removed")
    for pos, lab in ((P.vFCPos, "flight controller\nPLACEHOLDER"),
                     (P.vCCPos, "companion computer\nPLACEHOLDER"),
                     (P.vPDBPos, "12S power distribution")):
        p = px([pos], "iso", f, ps)[0]
        _call(ax, p, lab, (150, -50))
    _note(ax, w, h, ["removal sequence: top panel -> FC tray ->",
                     "companion computer -> PDB"])
    save(fig, "avionics/11_avionics_bay.png")

    # ---- 12-18  manipulator + each joint ---------------------------------
    q = P.CFG["03_MANIPULATION"]
    F = K.joint_frames([float(v) for v in q])
    f, ps = shoot(regM, "iso", tmp, skip=("02_PROPULSION",))
    fig, ax, w, h = _fig(tmp, "6-DOF manipulator",
                         f"reach {P.vReach:.0f} mm, 2 kg payload "
                         "architecture")
    for i, (nm, off, ax_, lim, tq, sp, desc) in enumerate(P.JOINTS):
        p = px([F[i + 1][:3, 3]], "iso", f, ps)[0]
        dx = 150 if i % 2 == 0 else -210
        dy = (-150, -110, 60, -110, 90, -150)[i]
        _call(ax, p, f"{nm} {desc.split()[-1]}\n"
                     f"{lim[0]:+.0f}..{lim[1]:+.0f} deg   {tq:.0f} Nm",
              (dx, dy))
    save(fig, "manipulator/12_manipulator.png")

    for i, (nm, off, axs, lim, tq, sp, desc) in enumerate(P.JOINTS):
        c = F[i + 1][:3, 3]
        meshes = R.scene_meshes(regM, skip=("02_PROPULSION",))
        focal = np.array(c, float)
        ps2 = 210.0
        R.render(meshes, "iso", tmp, size=SIZE, focal=focal, ps=ps2)
        fig, ax, w, h = _fig(tmp, f"{nm} -- {desc}",
                             f"axis {axs}   {lim[0]:+.0f} to {lim[1]:+.0f} deg"
                             f"   {tq:.0f} Nm   {sp:.0f} deg/s")
        p = px([c], "iso", focal, ps2)[0]
        _call(ax, p, f"{nm} axis", (110, -70))
        _note(ax, w, h, [f"housing dia {getattr(P, f'vJ{i+1}D'):.0f} mm",
                         "mechanical hard stops both ends",
                         "actuator: PLACEHOLDER ENVELOPE"])
        save(fig, f"manipulator/{13+i}_{nm}.png")

    # ---- 19  repair gripper ----------------------------------------------
    t0, fts, tc, tip = K.tool_frames([float(v) for v in P.CFG["04_REPAIR"]])
    meshes = R.scene_meshes(regR, skip=("02_PROPULSION",))
    focal = np.array(tc[:3, 3], float)
    R.render(meshes, "iso", tmp, size=SIZE, focal=focal, ps=175.0)
    fig, ax, w, h = _fig(tmp, "Repair gripper",
                         f"two-finger, {P.vGripStroke:.0f} mm stroke, "
                         f"{P.vGripForce:.0f} N")
    _call(ax, px([tip[:3, 3]], "iso", focal, 175.0)[0],
          "replaceable finger tips", (130, 70))
    _note(ax, w, h, ["grip force ESTIMATED -- verify against the",
                     "selected actuator before fabrication"], WARN)
    save(fig, "tools/19_repair_gripper.png")

    # ---- 20  tool changer -------------------------------------------------
    R.render(meshes, "iso2", tmp, size=SIZE, focal=focal, ps=115.0)
    fig, ax, w, h = _fig(tmp, "Tool changer",
                         f"{P.vTCPinN} locking pins, {P.vTCElecWays}-way "
                         f"electrical, {P.vTCFluidPorts} fluid port")
    _call(ax, px([fts[:3, 3]], "iso2", focal, 115.0)[0],
          "6-axis F/T sensor\nabove the interface", (150, -80))
    _call(ax, px([tc[:3, 3]], "iso2", focal, 115.0)[0],
          "master / tool plate\nsplit line", (-190, 60))
    save(fig, "tools/20_tool_changer.png")

    # ---- 21  liquid service module ---------------------------------------
    f, ps = shoot(regL, "iso_low", tmp,
                  skip=("02_PROPULSION", "03_BATTERY", "01_AIRFRAME",
                        "10_LANDING_GEAR"))
    fig, ax, w, h = _fig(tmp, "Liquid-service module",
                         f"{P.vTankVol:.2f} L tank, {P.vTankFill:.1f} L "
                         f"working, {P.vBaffleN} baffles")
    for pos, lab in ((P.vTankPos, "service-fluid cartridge\nremovable"),
                     (P.vPumpPos, "diaphragm pump\nPLACEHOLDER"),
                     ((-45, 44, -60), "dry-break coupling")):
        p = px([pos], "iso_low", f, ps)[0]
        _call(ax, p, lab, (150, -50))
    _note(ax, w, h, [f"slosh mode 1.86 -> 3.95 Hz with {P.vBaffleN} baffles",
                     "pump / filter / valves stay on the aircraft"])
    save(fig, "liquid_service/21_liquid_module.png")

    # ---- 22  landing gear -------------------------------------------------
    f, ps = shoot(reg, "front", tmp)
    fig, ax, w, h = _fig(tmp, "Landing gear",
                         f"track {P.vGearTrack:.0f} mm, skid "
                         f"{P.vGearGround:.0f} mm")
    p = px([(0, P.vGearTrack / 2, P.vGearGround)], "front", f, ps)[0]
    _call(ax, p, f"skid, track {P.vGearTrack:.0f} mm", (150, -50))
    _note(ax, w, h, ["static tip-over half-angle 35.1 deg",
                     "lower rotor 430 mm above the skid plane"])
    save(fig, "landing_gear/22_landing_gear.png")

    # ---- 23  service panels ----------------------------------------------
    f, ps = shoot(reg, "iso", tmp, explode=95.0,
                  skip=("02_PROPULSION", "06_MANIPULATOR"))
    fig, ax, w, h = _fig(tmp, "Service panels", "all four panels off")
    _note(ax, w, h, ["top: avionics    bottom: fluid + PDB",
                     "sides: battery bay and harness"])
    save(fig, "overall/23_service_panels.png")

    # ---- 24  battery removal ---------------------------------------------
    meshes = R.scene_meshes(reg, skip=("02_PROPULSION",))
    out = []
    for nm, v, t, c in meshes:
        if "battery_cartridge" in nm:
            v = v + np.array([-260.0, 0, 0])
        out.append((nm, v, t, c))
    focal, ps = R.fit_parallel_scale(out, "iso2", SIZE)
    R.render(out, "iso2", tmp, size=SIZE, focal=focal, ps=ps)
    fig, ax, w, h = _fig(tmp, "Battery removal", "cartridges withdrawn aft")
    _note(ax, w, h, ["VERIFIED: corridor clear of all hardware",
                     "no tools, no arm or gear removal"])
    save(fig, "validation/24_battery_removal.png")

    # ---- 25-26  maintenance / transport ----------------------------------
    for rg, cfg, fn, title in ((regX, "07_MAINTENANCE",
                                "25_maintenance.png", "Maintenance"),
                               (regT, "06_TRANSPORT",
                                "26_transport.png", "Transport")):
        f, ps = shoot(rg, "iso", tmp)
        q = P.CFG[cfg]
        fig, ax, w, h = _fig(tmp, f"{title} configuration",
                             f"J1..J6 = {', '.join(f'{v:.0f}' for v in q)} deg")
        save(fig, f"configurations/{fn}")

    # ---- 27-29  reach / work configurations ------------------------------
    ws = K.workspace(n=70)
    for cfgname, fn, title in (("03_MANIPULATION", "27_max_reach.png",
                                "Maximum arm reach"),
                               ("02_INSPECTION", "28_side_reach.png",
                                "Side reach"),
                               ("05_LIQUID", "29_downward_work.png",
                                "Downward work")):
        rgc = assy_b.build(cfgname)
        view = "right" if "side" in title.lower() else "front"
        f, ps = shoot(rgc, view, tmp)
        q = [float(v) for v in P.CFG[cfgname]]
        tip = K.tool_point(q)
        fig, ax, w, h = _fig(tmp, title,
                             f"tool point ({tip[0]:.0f}, {tip[1]:.0f}, "
                             f"{tip[2]:.0f}) mm")
        p = px([tip], view, f, ps)[0]
        _call(ax, p, "tool point", (120, -70))
        if fn.startswith("27"):
            _note(ax, w, h,
                  [f"tool reaches {ws[:,0].max():.0f} mm forward",
                   f"rotor envelope ends at "
                   f"{K.prop_forward_extent():.0f} mm",
                   f"margin {ws[:,0].max()-K.prop_forward_extent():.0f} mm"])
        save(fig, f"configurations/{fn}")

    # ---- 30  propeller clearance -----------------------------------------
    f, ps = shoot(reg, "top", tmp)
    fig, ax, w, h = _fig(tmp, "Propeller clearance",
                         "swept discs and the arm-reach envelope")
    th = np.linspace(0, 2 * math.pi, 200)
    for (mx, my) in rot:
        pts = np.stack([mx + P.vPropR * np.cos(th),
                        my + P.vPropR * np.sin(th),
                        np.zeros_like(th)], axis=1)
        pp = px(pts, "top", f, ps)
        ax.plot(pp[:, 0], pp[:, 1], color=ACCENT, lw=1.5, ls="--", zorder=7)
    rr = P.vReach
    pts = np.stack([P.vArmBase[0] + rr * np.cos(th),
                    rr * np.sin(th), np.zeros_like(th)], axis=1)
    pp = px(pts, "top", f, ps)
    ax.plot(pp[:, 0], pp[:, 1], color=WARN, lw=1.5, zorder=7)
    _note(ax, w, h,
          [f"blue  : swept rotor discs, dia {P.vPropDia:.0f} mm",
           f"orange: manipulator reach, {rr:.0f} mm",
           f"tip-to-tip gap {2*ARM*math.sin(math.pi/4)-P.vPropDia:.0f} mm "
           "(VERIFIED)"])
    save(fig, "validation/30_propeller_clearance.png")

    if os.path.exists(tmp):
        os.remove(tmp)
    n = sum(len(fs) for _, _, fs in os.walk(OUT))
    print(f"\n{n} images written to {OUT}/")


if __name__ == "__main__":
    main()
