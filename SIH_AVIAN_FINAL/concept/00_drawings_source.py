#!/usr/bin/env python3
"""SIH_AVIAN_FINAL - concept drawings for the new compact environment."""
import os, math, random
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon, FancyArrowPatch, Ellipse
from matplotlib.lines import Line2D

OUT = "/tmp/claude-0/-home-claude/b6aaac2a-7598-5c01-a008-1bf5dd7a3bbc/scratchpad/SIH_AVIAN_FINAL_CONCEPT"
os.makedirs(OUT, exist_ok=True)

INK      = "#16202a"
SUB      = "#5a6570"
FAINT    = "#aab3bb"
CONC     = "#ccd1d6"
CONC_E   = "#7d858d"
CONC_D   = "#b3b9bf"
WATER    = "#7fa8c4"
WATER_D  = "#5d8aa8"
GROUND   = "#cdc4b0"
GROUND_D = "#b3a98f"
METRO    = "#0b6e75"
METRO_L  = "#8fc4c7"
ROAD     = "#3f4d5c"
DEFECT   = "#c0392b"
ZONE     = "#c98a2e"
BG       = "#f5f3ef"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 9,
    "axes.edgecolor": SUB,
    "text.color": INK,
    "axes.labelcolor": INK,
    "xtick.color": SUB, "ytick.color": SUB,
    "figure.facecolor": BG, "axes.facecolor": BG,
})

# ----------------------------------------------------------------- geometry
L         = 360.0          # corridor length, m
PIERS     = [0, 45, 90, 135, 225, 270, 315, 360]
MAIN      = (135, 225)     # 90 m main span
RIV_C     = 180.0
RIV_W     = 90.0
RIV_X0, RIV_X1 = RIV_C - RIV_W/2, RIV_C + RIV_W/2
WATER_Z   = -2.0

RD_Y      = 0.0            # road centreline
RD_W      = 14.0
RD_DECK_Z = 14.0
RD_SOF_Z  = 11.5

MT_Y      = 28.0           # metro centreline
MT_W      = 9.0
MT_DECK_Z = 19.0
MT_SOF_Z  = 16.8
MT_PIERS  = list(np.arange(0, L + 1, 30.0))

ZONE_X0, ZONE_X1 = 90.0, 270.0   # research zone
N_DEF = 96

def save(fig, name):
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=170, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print("wrote", name)

def dim(ax, x0, x1, y, label, c=SUB, off=0, fs=8):
    ax.annotate("", (x0, y), (x1, y),
                arrowprops=dict(arrowstyle="<->", color=c, lw=0.9))
    ax.text((x0+x1)/2, y+off, label, ha="center", va="bottom",
            fontsize=fs, color=c)

def title(ax, n, t, sub):
    ax.annotate(n, (0, 1), xycoords="axes fraction", xytext=(0, 46),
                textcoords="offset points", fontsize=9, color=ZONE,
                fontweight="bold", va="bottom")
    ax.annotate(t, (0, 1), xycoords="axes fraction", xytext=(0, 16),
                textcoords="offset points", fontsize=15, color=INK,
                fontweight="bold", va="bottom")
    ax.annotate(sub, (1, 1), xycoords="axes fraction", xytext=(0, 22),
                textcoords="offset points", fontsize=8.5, color=SUB,
                ha="right", va="bottom")

# =========================================================== 1. PLAN VIEW
fig, ax = plt.subplots(figsize=(13, 6.4))
ax.set_facecolor(BG)

# ground
ax.add_patch(Rectangle((-20, -40), L+40, 110, fc=GROUND, ec="none", zorder=0))
# river (runs across, along Y)
ax.add_patch(Rectangle((RIV_X0, -40), RIV_W, 110, fc=WATER, ec=WATER_D,
                       lw=1.0, zorder=1))
ax.text(RIV_C, -36, "RIVER  90 m", ha="center", fontsize=9,
        color="#2c4f66", fontweight="bold")

# research zone band
ax.add_patch(Rectangle((ZONE_X0, -22), ZONE_X1-ZONE_X0, 74, fc=ZONE,
                       alpha=.10, ec=ZONE, ls="--", lw=1.1, zorder=2))
ax.text(ZONE_X0+3, 50, "RESEARCH ZONE  180 m", fontsize=8.5,
        color=ZONE, fontweight="bold", va="top")

# road deck
ax.add_patch(Rectangle((0, RD_Y-RD_W/2), L, RD_W, fc=CONC, ec=CONC_E,
                       lw=1.2, zorder=3))
ax.plot([0, L], [RD_Y, RD_Y], color="#e8e2d2", lw=1.0, ls=(0,(7,7)), zorder=4)
# metro deck
ax.add_patch(Rectangle((0, MT_Y-MT_W/2), L, MT_W, fc=CONC, ec=METRO,
                       lw=1.4, zorder=3))
# rails
for dy in (-1.7, 1.7):
    ax.plot([0, L], [MT_Y+dy, MT_Y+dy], color=METRO, lw=0.9, zorder=4)

# piers
for i, x in enumerate(PIERS):
    if i in (0, len(PIERS)-1):
        ax.add_patch(Rectangle((x-3 if i==0 else x-1, RD_Y-9), 4, 18,
                               fc=CONC_D, ec=CONC_E, lw=1, zorder=5))
    else:
        for dy in (-4.5, 4.5):
            ax.add_patch(Circle((x, RD_Y+dy), 0.9, fc=CONC_D, ec=CONC_E,
                                lw=1, zorder=5))
        ax.plot([x, x], [RD_Y-7.5, RD_Y+7.5], color=CONC_E, lw=2.4, zorder=4)
for x in MT_PIERS:
    if RIV_X0 < x < RIV_X1:
        continue
    ax.add_patch(Circle((x, MT_Y), 1.0, fc=CONC_D, ec=METRO, lw=1, zorder=5))

# metro train
tx0 = 132.0
for k in range(3):
    ax.add_patch(Rectangle((tx0 + k*23, MT_Y-3.0), 21, 6.0, fc=METRO,
                           ec="#054b50", lw=1, zorder=6))
ax.text(tx0+34, MT_Y+6.4, "METRO TRAIN  3 cars, 66 m", fontsize=8,
        color=METRO, ha="center", fontweight="bold")

# cars
rng = random.Random(7)
for _ in range(16):
    x = rng.uniform(6, L-10); dy = rng.choice([-5.2, -1.8, 1.8, 5.2])
    w = rng.choice([4.4, 4.4, 4.4, 9.0])
    ax.add_patch(Rectangle((x, RD_Y+dy-1.0), w, 2.0, fc=ROAD, ec="none",
                           zorder=6, alpha=.85))

# dims
dim(ax, 0, L, -24.5, "CORRIDOR  360 m", off=1.2, fs=9)
dim(ax, MAIN[0], MAIN[1], -15.5, "MAIN SPAN 90 m", c="#2c4f66", off=.8)
dim(ax, RD_Y+RD_W/2+0.0, 0, 0, "")  # noop
ax.annotate("", (L-24, RD_Y+RD_W/2), (L-24, MT_Y-MT_W/2),
            arrowprops=dict(arrowstyle="<->", color=DEFECT, lw=1.2))
ax.text(L-21, (RD_Y+RD_W/2+MT_Y-MT_W/2)/2,
        "INTER-STRUCTURE\nCORRIDOR  16.5 m", fontsize=8.2, color=DEFECT,
        va="center", fontweight="bold")

ax.text(-14, RD_Y, "ROAD\nBRIDGE", fontsize=9, color=INK, ha="right",
        va="center", fontweight="bold")
ax.text(-14, MT_Y, "METRO\nVIADUCT", fontsize=9, color=METRO, ha="right",
        va="center", fontweight="bold")

title(ax, "FIGURE 01", "SIH_AVIAN_FINAL — plan",
      "1 road bridge · 1 metro viaduct · 1 river · no city")
ax.set_xlim(-46, L+18); ax.set_ylim(-40, 58)
ax.set_aspect("equal"); ax.axis("off")
save(fig, "01_plan.png")

# ========================================================== 2. ELEVATION
fig, ax = plt.subplots(figsize=(13, 5.4))

# ground profile + river bed
gx = np.linspace(-10, L+10, 400)
gz = np.where((gx > RIV_X0-14) & (gx < RIV_X1+14),
              -6 - 4*np.cos(np.clip((gx-RIV_C)/(RIV_W/2+14),-1,1)*math.pi/2)**2, 0.0)
gz = np.minimum(gz, 0.0)
ax.fill_between(gx, gz, -16, color=GROUND, zorder=1)
ax.plot(gx, gz, color=GROUND_D, lw=1.1, zorder=2)
ax.fill_between([RIV_X0-13, RIV_X1+13], WATER_Z, -16, color=WATER,
                alpha=.85, zorder=3)
ax.plot([RIV_X0-13, RIV_X1+13], [WATER_Z, WATER_Z], color=WATER_D, lw=1.3,
        zorder=4)

# metro (behind)
ax.add_patch(Rectangle((0, MT_SOF_Z), L, MT_DECK_Z-MT_SOF_Z, fc="#e3e7ea",
                       ec=METRO_L, lw=1.0, zorder=4))
for x in MT_PIERS:
    if RIV_X0 < x < RIV_X1: continue
    ax.add_patch(Rectangle((x-1.0, 0), 2.0, MT_SOF_Z, fc="#e3e7ea",
                           ec=METRO_L, lw=1.0, zorder=4))
ax.text(L-4, MT_DECK_Z+1.6, "metro viaduct behind", fontsize=7.6,
        color=METRO_L, ha="right")

# road deck
ax.add_patch(Rectangle((0, RD_SOF_Z), L, RD_DECK_Z-RD_SOF_Z, fc=CONC,
                       ec=CONC_E, lw=1.3, zorder=6))
ax.add_patch(Rectangle((0, RD_DECK_Z), L, 1.1, fc=CONC_D, ec=CONC_E,
                       lw=0.8, zorder=6))
# deeper girder over main span
ax.add_patch(Rectangle((MAIN[0], RD_SOF_Z-1.4), MAIN[1]-MAIN[0], 1.4,
                       fc=CONC, ec=CONC_E, lw=1.0, zorder=6))

# piers
for i, x in enumerate(PIERS[1:-1], 1):
    base = float(np.interp(x, gx, gz))
    ax.add_patch(Rectangle((x-4.0, RD_SOF_Z-1.9), 8.0, 1.9, fc=CONC_D,
                           ec=CONC_E, lw=1, zorder=6))
    for dx in (-2.4, 2.4):
        ax.add_patch(Rectangle((x+dx-0.85, base, ), 1.7, RD_SOF_Z-1.9-base,
                               fc=CONC_D, ec=CONC_E, lw=1, zorder=5))
    ax.add_patch(Rectangle((x-3.2, base-1.2), 6.4, 1.4, fc=CONC_D,
                           ec=CONC_E, lw=1, zorder=5))
# abutments
for x, s in ((0, 1), (L, -1)):
    ax.add_patch(Polygon([(x, RD_SOF_Z), (x, -3), (x+s*9, -3),
                          (x+s*9, RD_SOF_Z)], fc=CONC_D, ec=CONC_E, lw=1,
                         zorder=5))

# defects on the main span + piers
rng = np.random.default_rng(20260921)
dx_, dz_ = [], []
for _ in range(60):
    if rng.random() < .55:
        x = rng.uniform(MAIN[0]+5, MAIN[1]-5); z = rng.uniform(RD_SOF_Z-1.3, RD_SOF_Z+2.2)
    else:
        p = PIERS[rng.integers(1, len(PIERS)-1)]
        x = p + rng.uniform(-3, 3); z = rng.uniform(0.5, RD_SOF_Z-2.5)
    dx_.append(x); dz_.append(z)
ax.scatter(dx_, dz_, s=9, c=DEFECT, marker="x", lw=1.0, zorder=8)

# dims
for a, b in zip(PIERS[:-1], PIERS[1:]):
    lab = "90 m" if (a, b) == MAIN else "45 m"
    c = "#2c4f66" if (a, b) == MAIN else SUB
    dim(ax, a, b, RD_DECK_Z+4.2, lab, c=c, off=.5, fs=7.6)
ax.annotate("", (RIV_C, WATER_Z), (RIV_C, RD_SOF_Z-1.4),
            arrowprops=dict(arrowstyle="<->", color="#2c4f66", lw=1.1))
ax.text(RIV_C+3, (WATER_Z+RD_SOF_Z)/2, "AIR DRAFT\n12.1 m", fontsize=8,
        color="#2c4f66", fontweight="bold", va="center")
ax.annotate("", (28, 0), (28, RD_SOF_Z),
            arrowprops=dict(arrowstyle="<->", color=SUB, lw=1.0))
ax.text(31, RD_SOF_Z/2, "soffit 11.5 m", fontsize=7.8, color=SUB, va="center")

ax.scatter([], [], s=16, c=DEFECT, marker="x", label="defect (96 total)")
ax.legend(loc="lower right", frameon=False, fontsize=8,
          bbox_to_anchor=(0.995, 0.02))

title(ax, "FIGURE 02", "Elevation — road bridge",
      "7 spans · 6 piers · main span 90 m over the river")
ax.set_xlim(-16, L+16); ax.set_ylim(-16, 27)
ax.set_aspect("equal"); ax.axis("off")
save(fig, "02_elevation.png")

# ======================================================= 3. CROSS SECTION
fig, ax = plt.subplots(figsize=(11.5, 7.2))

ax.fill_between([-16, 46], -16, 0, color=GROUND, zorder=1)
ax.plot([-16, 46], [0, 0], color=GROUND_D, lw=1.2, zorder=2)

# road deck section
ax.add_patch(Rectangle((RD_Y-RD_W/2, RD_SOF_Z), RD_W, RD_DECK_Z-RD_SOF_Z,
                       fc=CONC, ec=CONC_E, lw=1.4, zorder=5))
for gy in np.linspace(RD_Y-5.2, RD_Y+5.2, 5):      # I-girders
    ax.add_patch(Rectangle((gy-0.55, RD_SOF_Z-2.0), 1.1, 2.0, fc=CONC_D,
                           ec=CONC_E, lw=.9, zorder=5))
for s in (-1, 1):                                   # parapets
    ax.add_patch(Rectangle((s*RD_W/2 - (0.4 if s>0 else 0), RD_DECK_Z),
                           0.4, 1.1, fc=CONC_D, ec=CONC_E, lw=.9, zorder=6))
# cars in section
for dy in (-5.2, -1.8, 1.8, 5.2):
    ax.add_patch(Rectangle((dy-0.9, RD_DECK_Z), 1.8, 1.5, fc=ROAD,
                           ec="none", zorder=6, alpha=.85))
# road piers
for dx in (-4.5, 4.5):
    ax.add_patch(Rectangle((dx-0.85, 0), 1.7, RD_SOF_Z-2.9, fc=CONC_D,
                           ec=CONC_E, lw=1, zorder=4))
ax.add_patch(Rectangle((-5.9, RD_SOF_Z-2.9), 11.8, 1.0, fc=CONC_D,
                       ec=CONC_E, lw=1, zorder=4))

# metro box girder section
ax.add_patch(Rectangle((MT_Y-MT_W/2, MT_SOF_Z), MT_W, MT_DECK_Z-MT_SOF_Z,
                       fc=CONC, ec=METRO, lw=1.5, zorder=5))
ax.add_patch(Rectangle((MT_Y-2.6, MT_SOF_Z+0.45), 5.2, 1.35, fc=BG,
                       ec=METRO, lw=1.0, ls="--", zorder=6))
ax.text(MT_Y, MT_SOF_Z+1.12, "cell", fontsize=6.5, color=METRO,
        ha="center", va="center")
for dy in (-1.7, 1.7):
    ax.add_patch(Rectangle((MT_Y+dy-0.12, MT_DECK_Z), 0.24, 0.35,
                           fc=METRO, ec="none", zorder=6))
ax.add_patch(Rectangle((MT_Y-3.2, MT_DECK_Z+0.35), 6.4, 3.4, fc=METRO,
                       ec="#054b50", lw=1, zorder=6, alpha=.92))
ax.text(MT_Y, MT_DECK_Z+2.05, "TRAIN", fontsize=7.5, color="white",
        ha="center", va="center", fontweight="bold")
ax.add_patch(Rectangle((MT_Y-1.0, 0), 2.0, MT_SOF_Z-1.3, fc=CONC_D,
                       ec=METRO, lw=1, zorder=4))
ax.add_patch(Polygon([(MT_Y-3.4, MT_SOF_Z), (MT_Y+3.4, MT_SOF_Z),
                      (MT_Y+1.2, MT_SOF_Z-1.3), (MT_Y-1.2, MT_SOF_Z-1.3)],
                     fc=CONC_D, ec=METRO, lw=1, zorder=4))

# UAV standoffs
ax.add_patch(Ellipse((RD_Y, RD_SOF_Z-3.2), 3.0, 1.4, fc="none", ec=DEFECT,
                     lw=1.3, zorder=8))
ax.text(RD_Y, RD_SOF_Z-4.6, "UNDERSIDE\n1.5 m standoff", fontsize=7.5,
        color=DEFECT, ha="center", va="top", fontweight="bold")
ax.add_patch(Ellipse((14.5, 7.0), 3.0, 1.4, fc="none", ec=DEFECT, lw=1.3,
                     zorder=8))
ax.text(14.5, 5.4, "INTER-STRUCTURE\nCORRIDOR", fontsize=7.5, color=DEFECT,
        ha="center", va="top", fontweight="bold")

dim(ax, RD_Y-RD_W/2, RD_Y+RD_W/2, RD_DECK_Z+3.4, "14.0 m", off=.35, fs=8)
dim(ax, MT_Y-MT_W/2, MT_Y+MT_W/2, MT_DECK_Z+4.6, "9.0 m", c=METRO, off=.35, fs=8)
dim(ax, RD_Y+RD_W/2, MT_Y-MT_W/2, -4.5, "16.5 m clear", c=DEFECT, off=.4, fs=8)
ax.annotate("", (-13, 0), (-13, RD_DECK_Z),
            arrowprops=dict(arrowstyle="<->", color=SUB, lw=1))
ax.text(-12, RD_DECK_Z/2, "14.0 m", fontsize=8, color=SUB, va="center")
ax.annotate("", (42, 0), (42, MT_DECK_Z),
            arrowprops=dict(arrowstyle="<->", color=METRO, lw=1))
ax.text(43, MT_DECK_Z/2, "19.0 m", fontsize=8, color=METRO, va="center")

title(ax, "FIGURE 03", "Cross-section",
      "I-girder road deck · single-cell box metro · the gap between them")
ax.set_xlim(-16, 46); ax.set_ylim(-9, 27)
ax.set_aspect("equal"); ax.axis("off")
save(fig, "03_section.png")

# ===================================================== 4. DEFECT LAYOUT
fig, ax = plt.subplots(figsize=(12.6, 5.6))
ax.add_patch(Rectangle((ZONE_X0, -14), ZONE_X1-ZONE_X0, 46, fc=ZONE,
                       alpha=.09, ec=ZONE, ls="--", lw=1.1))
ax.add_patch(Rectangle((0, RD_SOF_Z), L, RD_DECK_Z-RD_SOF_Z, fc=CONC,
                       ec=CONC_E, lw=1.2, zorder=4))
for x in PIERS[1:-1]:
    ax.add_patch(Rectangle((x-1.5, 0), 3.0, RD_SOF_Z, fc=CONC_D, ec=CONC_E,
                           lw=1, zorder=3))
ax.add_patch(Rectangle((0, MT_SOF_Z), L, MT_DECK_Z-MT_SOF_Z, fc="#e6eaec",
                       ec=METRO_L, lw=1.1, zorder=3))

kinds = [("hairline crack", 24, "x"), ("longitudinal", 12, "x"),
         ("transverse", 10, "x"), ("network", 8, "+"),
         ("spall", 12, "o"), ("delamination", 8, "s"),
         ("rebar exposed", 6, "D"), ("corrosion stain", 10, "v"),
         ("joint deterioration", 6, "^")]
rng = np.random.default_rng(4242)
handles = []
for i, (nm, n, mk) in enumerate(kinds):
    xs, zs = [], []
    for _ in range(n):
        if rng.random() < .45:
            p = PIERS[rng.integers(2, 5)]
            xs.append(p + rng.uniform(-2.2, 2.2)); zs.append(rng.uniform(1, 10))
        else:
            xs.append(rng.uniform(ZONE_X0+6, ZONE_X1-6))
            zs.append(rng.uniform(RD_SOF_Z-1.0, RD_DECK_Z+0.6))
    sc = ax.scatter(xs, zs, s=15, marker=mk, facecolors="none",
                    edgecolors=DEFECT, lw=.9, zorder=8)
    handles.append(Line2D([], [], marker=mk, ls="", mfc="none", mec=DEFECT,
                          label=f"{nm} ({n})", ms=5))
ax.legend(handles=handles, loc="lower left", frameon=False, fontsize=7.6,
          ncol=3, bbox_to_anchor=(0.0, -0.22))
ax.text(ZONE_X0+4, 30, "RESEARCH ZONE  x 90–270 m  ·  96 defects  ·  9 types",
        fontsize=9, color=ZONE, fontweight="bold")
ax.text(L-4, 30, "HIGH-DETAIL SHADERS ONLY HERE", fontsize=8,
        color=SUB, ha="right")
title(ax, "FIGURE 04", "Defect layout",
      "96 defects · measured visibility + measured contrast, as before")
ax.set_xlim(-10, L+10); ax.set_ylim(-18, 36)
ax.set_aspect("equal"); ax.axis("off")
save(fig, "04_defects.png")

# ===================================================== 5. SCALE COMPARISON
fig, ax = plt.subplots(figsize=(13, 4.2))
# both drawn to the SAME scale, so the difference is the point
ax.add_patch(Rectangle((0, 0), 4500, 90, fc="#e4e6e8", ec=FAINT, lw=1.2))
for k in range(138):
    ax.plot([4500*k/137]*2, [0, -55], color=FAINT, lw=0.5)
ax.text(0, 130, "REV-C   4 500 m · 137 spans · 14 656 objects · 680 buildings",
        fontsize=11, color=SUB, fontweight="bold")
ax.text(4500, -95, "does not fit in one viewport", fontsize=8.5, color=SUB,
        ha="right")

ax.add_patch(Rectangle((0, -300), 360, 90, fc=CONC, ec=METRO, lw=1.6))
for x in PIERS:
    ax.plot([x, x], [-300, -355], color=METRO, lw=1.0)
ax.add_patch(Rectangle((0, -180), 360, 40, fc="none", ec=METRO, lw=1.2))
ax.text(380, -160, "metro viaduct", fontsize=8, color=METRO, va="center")
ax.text(470, -230, "SIH_AVIAN_FINAL\n360 m · 7 spans · ~900 objects · no city",
        fontsize=11, color=METRO, fontweight="bold", va="center")
ax.text(0, -395, "the whole structure in one frame · every defect reachable "
        "in a 30-minute mission", fontsize=8.5, color=METRO, fontweight="bold")

ax.annotate("", (0, -430), (360, -430),
            arrowprops=dict(arrowstyle="<->", color=METRO, lw=1.2))
ax.text(180, -470, "360 m", fontsize=9, color=METRO, ha="center",
        fontweight="bold")
ax.annotate("", (0, 160), (4500, 160),
            arrowprops=dict(arrowstyle="<->", color=SUB, lw=1.0))
ax.text(2250, 175, "4 500 m  —  12.5x longer", fontsize=9, color=SUB,
        ha="center")
ax.set_xlim(-160, 4700); ax.set_ylim(-560, 300)
ax.axis("off")
save(fig, "05_scale.png")

# ===================================================== 6. INSPECTION VIEW
X0, X1 = 118.0, 242.0      # zoom on the main span
fig, ax = plt.subplots(figsize=(12.2, 4.6))
ax.add_patch(Rectangle((X0-6, -14), X1-X0+12, 12, fc=WATER, alpha=.5, ec="none"))
ax.plot([X0-6, X1+6], [-2, -2], color=WATER_D, lw=1.3)
ax.add_patch(Rectangle((X0-6, RD_SOF_Z), X1-X0+12, RD_DECK_Z-RD_SOF_Z,
                       fc=CONC, ec=CONC_E, lw=1.4, zorder=5))
for x in (135.0, 225.0):
    ax.add_patch(Rectangle((x-4.0, RD_SOF_Z-1.9), 8.0, 1.9, fc=CONC_D,
                           ec=CONC_E, lw=1, zorder=6))
    for dx in (-2.4, 2.4):
        ax.add_patch(Rectangle((x+dx-0.85, -8), 1.7, RD_SOF_Z-1.9+8,
                               fc=CONC_D, ec=CONC_E, lw=1, zorder=4))
ax.add_patch(Rectangle((X0-6, MT_SOF_Z), X1-X0+12, MT_DECK_Z-MT_SOF_Z,
                       fc=CONC, ec=METRO, lw=1.4, zorder=5))

vols = [("DECK INSPECTION  2.5 m/s",     RD_DECK_Z+2.0, 4.4, "#4a6b8a"),
        ("INTER-STRUCTURE  confined",    MT_SOF_Z-4.4,  4.0, "#c0392b"),
        ("UNDERSIDE  GNSS-DENIED 1.2 m/s", RD_SOF_Z-5.4, 3.2, "#8d5f1c")]
for nm, z0, h, c in vols:
    ax.add_patch(Rectangle((X0-4, z0), X1-X0+8, h, fc=c, alpha=.14, ec=c,
                           ls="--", lw=1.1, zorder=3))
    ax.text(X0-1, z0+h/2, nm, fontsize=7.8, color=c, fontweight="bold",
            va="center")
ax.add_patch(Rectangle((128, -8), 14, 20, fc="#6b4a8a", alpha=.14,
                       ec="#6b4a8a", ls="--", lw=1.1, zorder=3))
ax.text(135, -10.4, "PIER\nINSPECTION", fontsize=7.6, color="#6b4a8a",
        ha="center", va="top", fontweight="bold")

def drone(ax, x, z, arm, col, lab):
    ax.add_patch(Circle((x, z), 1.3, fc=BG, ec=col, lw=1.5, zorder=9))
    for s in (-1, 1):
        ax.plot([x, x+s*2.5], [z, z+1.0], color=col, lw=1.3, zorder=9)
        ax.add_patch(Ellipse((x+s*2.5, z+1.0), 2.6, .4, fc="none", ec=col,
                             lw=1.1, zorder=9))
    if arm:
        ax.plot([x, x-1.2, x-2.8], [z-1.2, z-2.6, z-2.9], color=col, lw=1.8,
                zorder=9, solid_capstyle="round")
        ax.add_patch(Circle((x-2.8, z-2.9), .4, fc=col, ec="none", zorder=9))
    ax.text(x, z+3.4, lab, fontsize=8, color=col, ha="center",
            fontweight="bold", zorder=9)

drone(ax, 168, RD_SOF_Z-3.6, False, "#0b6e75",
      "SCANNER\ncamera · LiDAR · thermal · no arm")
drone(ax, 218, RD_SOF_Z-3.6, True, "#c0392b",
      "REPAIRER\n6-DOF arm · nozzle")

dim(ax, 135, 225, -12.6, "MAIN SPAN 90 m", c="#2c4f66", off=.6)
title(ax, "FIGURE 06", "Flight volumes and the two aircraft",
      "main span, zoomed · four inspection classes")
ax.set_xlim(X0-12, X1+8); ax.set_ylim(-16, 30)
ax.set_aspect("equal"); ax.axis("off")
save(fig, "06_flight.png")

print("\nAll figures in", OUT)
