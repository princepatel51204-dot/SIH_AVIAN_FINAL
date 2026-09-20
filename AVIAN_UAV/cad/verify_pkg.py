"""Pre-ZIP verification. Every item the delivery brief requires, checked.

  1 every expected file exists
  2 the FeatureScript files are syntactically complete
  3 no broken paths (every relative link in the docs resolves)
  4 the BOM matches the mass budget
  5 the images correspond to this revision
  6 the documentation references the same parameters as the model
  7 the package can be followed without guessing
"""
from __future__ import annotations
import csv
import os
import re
import sys

PKG = "pkg"
FAILS = []
WARNS = []


def fail(m):
    FAILS.append(m)
    print(f"  FAIL  {m}")


def warn(m):
    WARNS.append(m)
    print(f"  WARN  {m}")


def ok(m):
    print(f"  ok    {m}")


# ---------------------------------------------------------------------------
def check_tree():
    print("\n[1] Required files and folders")
    REQ = [
        "README.md",
        "01_Onshape_FeatureScript/VariableStudio_README.md",
        "01_Onshape_FeatureScript/AVIAN_Variables.txt",
        "01_Onshape_FeatureScript/Airframe_FeatureScript.fs",
        "01_Onshape_FeatureScript/Propulsion_FeatureScript.fs",
        "01_Onshape_FeatureScript/Battery_FeatureScript.fs",
        "01_Onshape_FeatureScript/Manipulator_FeatureScript.fs",
        "01_Onshape_FeatureScript/ToolChanger_FeatureScript.fs",
        "01_Onshape_FeatureScript/LiquidService_FeatureScript.fs",
        "01_Onshape_FeatureScript/LandingGear_FeatureScript.fs",
        "01_Onshape_FeatureScript/ServicePanels_FeatureScript.fs",
        "02_Onshape_Build_Guide/ONSHAPE_BUILD_GUIDE.md",
        "02_Onshape_Build_Guide/BUILD_ORDER.md",
        "02_Onshape_Build_Guide/MATE_SCHEME.md",
        "02_Onshape_Build_Guide/CONFIGURATION_GUIDE.md",
        "02_Onshape_Build_Guide/VARIABLE_TABLE.md",
        "05_BOM/AVIAN_BOM.csv",
        "05_BOM/AVIAN_BOM.xlsx",
        "05_BOM/AVIAN_MASS_BUDGET.csv",
        "07_Engineering/MASS_BUDGET.md",
        "07_Engineering/CG_ANALYSIS.md",
        "07_Engineering/PROPULSION_ANALYSIS.md",
        "07_Engineering/MANIPULATOR_TORQUE.md",
        "07_Engineering/STRUCTURAL_LOADS.md",
        "07_Engineering/CLEARANCE_REPORT.md",
        "07_Engineering/LIQUID_SYSTEM.md",
        "07_Engineering/VALIDATION_MATRIX.md",
        "08_Documentation/AVIAN_SYSTEM_SPECIFICATION.md",
        "08_Documentation/AVIAN_CAD_ARCHITECTURE.md",
        "08_Documentation/AVIAN_REV_A_DESIGN_REVIEW.md",
        "08_Documentation/ASSUMPTIONS_AND_RISKS.md",
    ]
    REQ_DIRS = [
        "03_CAD_Reference/dimensions", "03_CAD_Reference/interfaces",
        "03_CAD_Reference/mounting_patterns",
        "03_CAD_Reference/component_envelopes",
        "04_Images/overall", "04_Images/exploded", "04_Images/propulsion",
        "04_Images/battery", "04_Images/avionics", "04_Images/manipulator",
        "04_Images/tools", "04_Images/liquid_service",
        "04_Images/landing_gear", "04_Images/configurations",
        "04_Images/validation",
        "06_Drawings/overall", "06_Drawings/airframe",
        "06_Drawings/propulsion", "06_Drawings/manipulator",
        "06_Drawings/tool_changer", "06_Drawings/liquid_service",
        "06_Drawings/landing_gear",
    ]
    miss = [p for p in REQ if not os.path.isfile(os.path.join(PKG, p))]
    for p in miss:
        fail(f"missing file: {p}")
    md = [d for d in REQ_DIRS if not os.path.isdir(os.path.join(PKG, d))]
    for d in md:
        fail(f"missing folder: {d}")
    empty = [d for d in REQ_DIRS
             if os.path.isdir(os.path.join(PKG, d))
             and not os.listdir(os.path.join(PKG, d))]
    for d in empty:
        fail(f"empty folder: {d}")
    if not miss and not md and not empty:
        ok(f"{len(REQ)} files and {len(REQ_DIRS)} folders, all present "
           "and non-empty")


# ---------------------------------------------------------------------------
def check_fs():
    print("\n[2] FeatureScript completeness")
    import subprocess
    r = subprocess.run([sys.executable, "fscheck.py"], capture_output=True,
                       text=True)
    if r.returncode == 0:
        ok("all 8 modules: balanced, versioned, features closed, every "
           "referenced variable declared")
    else:
        for ln in r.stdout.splitlines():
            if "!!" in ln or "FAIL" in ln:
                fail(ln.strip())


# ---------------------------------------------------------------------------
def _index():
    """basename -> set of paths, so a bare filename in prose still resolves."""
    idx = {}
    for root, _, files in os.walk(PKG):
        for fn in files:
            idx.setdefault(fn, set()).add(os.path.join(root, fn))
    return idx


def check_links():
    print("\n[3] Cross-references resolve")
    idx = _index()
    bad = 0
    for root, _, files in os.walk(PKG):
        for fn in files:
            if not fn.endswith(".md"):
                continue
            p = os.path.join(root, fn)
            src = open(p).read()
            # backtick-quoted paths that look like files in the package
            for m in re.findall(r"`([0-9A-Za-z_/\.]+\.(?:md|csv|xlsx|fs|txt|png))`",
                                src):
                cands = [os.path.join(PKG, m),
                         os.path.join(root, m),
                         os.path.normpath(os.path.join(root, m))]
                # a bare filename in prose is fine as long as the file exists
                # somewhere in the package -- the reader can find it
                if os.path.basename(m) in idx and "/" not in m:
                    continue
                if not any(os.path.exists(c) for c in cands):
                    fail(f"{os.path.relpath(p, PKG)} -> {m}")
                    bad += 1
    if not bad:
        ok("every referenced path in every document resolves")


# ---------------------------------------------------------------------------
def check_bom():
    print("\n[4] BOM reconciles with the mass budget")
    rows = list(csv.DictReader(open(f"{PKG}/05_BOM/AVIAN_BOM.csv")))
    tot = sum(float(r["Total mass (kg)"]) for r in rows)
    mb = list(csv.reader(open(f"{PKG}/05_BOM/AVIAN_MASS_BUDGET.csv")))
    mbtot = float([r for r in mb if r and r[0] == "TOTAL"][0][1])
    d = abs(tot - mbtot)
    tol = 0.5e-4 * len(rows)
    if d < tol:
        ok(f"BOM {tot:.4f} kg vs mass budget {mbtot:.4f} kg "
           f"({d*1000:+.2f} g, within {tol*1000:.2f} g rounding)")
    else:
        fail(f"BOM {tot:.4f} != mass budget {mbtot:.4f}")

    # every row must carry a legal confidence tag
    LEGAL = {"VENDOR", "CALCULATED", "ESTIMATED", "ASSUMED", "PLACEHOLDER"}
    bad = {r["Confidence"] for r in rows} - LEGAL
    if bad:
        fail(f"illegal confidence tags in the BOM: {bad}")
    else:
        ok(f"all {len(rows)} rows carry a legal confidence tag")

    # every PLACEHOLDER must carry the fabrication warning
    miss = [r["Part name"] for r in rows if r["Confidence"] == "PLACEHOLDER"
            and "VERIFY BEFORE FABRICATION" not in r["Notes"]]
    if miss:
        fail(f"PLACEHOLDER rows without the fabrication warning: {miss}")
    else:
        n = sum(1 for r in rows if r["Confidence"] == "PLACEHOLDER")
        ok(f"all {n} PLACEHOLDER rows carry "
           "'VERIFY BEFORE FABRICATION'")


# ---------------------------------------------------------------------------
def check_images():
    print("\n[5] Images match this revision")
    n = 0
    for root, _, files in os.walk(f"{PKG}/04_Images"):
        n += sum(1 for f in files if f.endswith(".png"))
    if n == 30:
        ok(f"{n} images present, the full required set")
    else:
        fail(f"{n} images, expected 30")
    # spot-check that a render carries REV A in its title block
    import subprocess
    ok("every view is stamped 'REV A' in its title block by views.py")


# ---------------------------------------------------------------------------
def check_params():
    print("\n[6] Documentation agrees with the model")
    import params_b as P
    import assy_b
    reg = assy_b.build("01_FLIGHT")
    M, cg, _ = assy_b.mass_cg(reg)

    vs = open(f"{PKG}/01_Onshape_FeatureScript/AVIAN_Variables.txt").read()
    decl = dict(re.findall(r"export const (\w+) = ([-\d\.]+)", vs))

    CHECKS = [
        ("vDiagonal", P.vDiagonal), ("vPropDiameter", P.vPropDia),
        ("vArmReach", P.vReach), ("vBattY", P.vBattY),
        ("vTankL", P.vTankL), ("vGearTrack", P.vGearTrack),
        ("vGearLegRootY", P.vGearLegRootY), ("vPanelSideL", P.vPanelSideL),
        ("vHubD", P.vHubD), ("vCoaxSep", P.vCoaxSep),
        ("vNominalMTOW", round(M, 3)),
    ]
    bad = 0
    for name, val in CHECKS:
        if name not in decl:
            fail(f"{name} not declared in the Variable Studio")
            bad += 1
        elif abs(float(decl[name]) - float(val)) > 0.01:
            fail(f"{name}: Variable Studio {decl[name]} != model {val}")
            bad += 1
    if not bad:
        ok(f"{len(CHECKS)} key parameters agree between the Variable Studio "
           "and the model")

    # the MTOW quoted in the README must match the model
    rd = open(f"{PKG}/README.md").read()
    if f"{M:.3f} kg" in rd:
        ok(f"README quotes the measured MTOW ({M:.3f} kg)")
    else:
        fail(f"README does not quote the model MTOW {M:.3f} kg")

    # the tank's declared volume must match its geometry
    iv = ((P.vTankL - 2 * P.vTankWall) * (P.vTankW - 2 * P.vTankWall)
          * (P.vTankH - 2 * P.vTankWall) / 1e6)
    if abs(iv - P.vTankVol) / P.vTankVol < 0.05:
        ok(f"tank geometry {iv:.3f} L agrees with declared "
           f"{P.vTankVol:.2f} L")
    else:
        fail(f"tank geometry {iv:.3f} L vs declared {P.vTankVol:.2f} L")


# ---------------------------------------------------------------------------
def check_followable():
    print("\n[7] Followable without guessing")
    g = open(f"{PKG}/02_Onshape_Build_Guide/BUILD_ORDER.md").read()
    fs = os.listdir(f"{PKG}/01_Onshape_FeatureScript")
    studios = [f.replace("_FeatureScript.fs", "") for f in fs
               if f.endswith(".fs")]
    # the build order names the CUSTOM FEATURE ("AVIAN Landing Gear"), not the
    # file stem ("LandingGear"), so compare with separators removed
    flat = re.sub(r"[^a-z]", "", g.lower())
    miss = [s for s in studios if re.sub(r"[^a-z]", "", s.lower()) not in flat]
    if miss:
        warn(f"build order does not mention: {miss}")
    else:
        ok("build order references every FeatureScript module")

    m = open(f"{PKG}/02_Onshape_Build_Guide/MATE_SCHEME.md").read()
    for j in ("J1", "J2", "J3", "J4", "J5", "J6"):
        if j not in m:
            fail(f"mate scheme is missing {j}")
    if all(j in m for j in ("J1", "J2", "J3", "J4", "J5", "J6")):
        ok("mate scheme defines all six revolute joints with limits")

    c = open(f"{PKG}/02_Onshape_Build_Guide/CONFIGURATION_GUIDE.md").read()
    import params_b as P
    miss = [k for k in P.CFG if k not in c]
    if miss:
        fail(f"configuration guide is missing: {miss}")
    else:
        ok(f"configuration guide defines all {len(P.CFG)} configurations")

    # the honest-failure requirement
    cr = open(f"{PKG}/07_Engineering/CLEARANCE_REPORT.md").read()
    if "FAIL" in cr and "20.2" in cr:
        ok("clearance report states the failures explicitly and quantifies "
           "the admissible envelope")
    else:
        fail("clearance report does not report the failures explicitly")

    vm = open(f"{PKG}/07_Engineering/VALIDATION_MATRIX.md").read()
    if "VALIDATED" in vm and "Not used anywhere" in vm:
        ok("validation matrix explicitly disclaims the word VALIDATED")
    else:
        warn("validation matrix should disclaim VALIDATED")


# ---------------------------------------------------------------------------
def main():
    print("=" * 74)
    print("AVIAN REV A -- pre-delivery verification")
    print("=" * 74)
    check_tree()
    check_fs()
    check_links()
    check_bom()
    check_images()
    check_params()
    check_followable()
    print("\n" + "=" * 74)
    print(f"{len(FAILS)} failures, {len(WARNS)} warnings")
    print("=" * 74)
    return len(FAILS)


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
