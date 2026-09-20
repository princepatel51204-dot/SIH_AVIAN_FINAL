"""AVIAN Phase 1/2 -- audit + trade study + architecture freeze proposal."""
import html, json

d = json.load(open("avian_report_data.json"))
F = {r["case"].split()[0]: r for r in d["frozen"]}
NOM, MIN, MAX = F["NOMINAL"], F["MIN"], F["MAX"]
PK = d["frozen_pack"]
C = d["coef"]


def esc(s):
    return html.escape(str(s))


# ---------------------------------------------------------------- audit
AUDIT = [
    ("Propulsion architecture",
     "Flat-8, 1950 mm diagonal, 27\" props, 8 separate arms",
     "Coaxial X8, 1100 mm diagonal, 27\" props, 4 arms",
     "Halves arm count and structural interfaces; 44 % smaller transport footprint; "
     "per-arm motor-out redundancy without an asymmetric moment",
     "HIGH", "HIGH", "MED", "MED", "MED"),
    ("Tool reach strategy",
     "700 mm arm + 838 mm fixed reach-extension lance, tools stored in airframe "
     "holsters (the lance existed only because the rotor envelope was 1244 mm)",
     "900 mm arm, tools mounted DIRECTLY on the wrist. No lance, no holsters",
     "The single biggest simplification available. Rotor envelope drops to 732 mm, "
     "so the wrist itself clears a flat facade by 213 mm. Removes 2 holsters, 2 lance "
     "assemblies, the long-tool storage problem, and the tool-point compliance of a "
     "0.84 m cantilever",
     "CRITICAL", "MED", "LOW", "LOW", "LOW"),
    ("Manipulator",
     "6-DOF, 700 mm flange reach, J4 pitch / J5 yaw / J6 roll wrist, 2.71 kg",
     "6-DOF, 900 mm reach, J4 ROLL / J5 pitch / J6 yaw wrist per the AVIAN brief, "
     "2.15 kg target, replaceable joint modules",
     "Brief specifies a roll-first wrist; that ordering suits tool-axis alignment on a "
     "surface better than pitch-first. Reach increase is what removes the lance",
     "CRITICAL", "HIGH", "MED", "MED", "MED"),
    ("Service payload",
     "Two-component epoxy injection only (resin + hardener cartridges, static mixer, "
     "metering pump). No gripper. No general liquid service",
     "General liquid-service module (single reservoir, pump, filter, check valve, "
     "quick-connect) PLUS a quick-change gripper tool",
     "AVIAN's mission is cleaning / coating / general service, not just crack "
     "injection. Single-fluid is simpler, lighter and covers more missions; "
     "two-part injection becomes a future tool option on the same interface",
     "HIGH", "MED", "LOW", "LOW", "LOW"),
    ("Battery architecture",
     "2 x 12S3P Li-ion, 1296 Wh, top-hatch lift-out, fixed fore/aft position with a "
     "+/-45 mm slotted trim rail",
     "2 x 12S 12 Ah cartridges in parallel, 1066 Wh, side-loading on a powered/indexed "
     "CG rail with +/-60 mm travel",
     "The brief requires ACTIVE CG adjustment for arm deployment and fluid depletion. "
     "A slotted trim rail is a build-time setting; AVIAN needs an in-service one",
     "HIGH", "MED", "MED", "LOW", "MED"),
    ("Central structure",
     "Octagonal monocoque box, 420 mm across flats, plate-and-post construction",
     "Central structural SPINE: two CF side rails carrying arm roots, hub, gear and "
     "rail, with non-structural service panels",
     "Brief explicitly rejects the box. A spine puts material on the actual load path "
     "(arm root -> hub -> gear) and lets every panel become removable",
     "HIGH", "HIGH", "MED", "MED", "MED"),
    ("Manipulator mount",
     "8 x M5 flange on the bottom plate; loads react into a 2.5 mm CF plate",
     "Dedicated machined AL hub integral with the spine, closing the load path into "
     "both side rails",
     "Arm reaction at full extension is ~20 Nm at J2. A flange on a thin plate is the "
     "weakest identified structural item in the current design",
     "CRITICAL", "MED", "MED", "MED", "HIGH"),
    ("Component representation",
     "Correct envelopes and masses, but simple prismatic bodies with flat colours",
     "Realistic component geometry: connectorised ESC modules with fin stacks, PCB-form "
     "flight controller, finned Jetson enclosure, lensed camera bodies, GNSS pucks",
     "Explicit brief requirement and the largest single contributor to the 5.22 score. "
     "Purely a modelling-effort item, no engineering risk",
     "MED", "MED", "LOW", "NONE", "NONE"),
    ("Landing gear",
     "Skid gear, 800 mm track, 450 mm clearance, fixed",
     "Wide-track high-clearance gear sized by the DEPLOYED arm envelope, replaceable "
     "elastomeric feet, shock-absorbing leg",
     "Current gear was sized before the arm workspace was frozen. AVIAN gear must be "
     "derived from the arm's downward-reach envelope, not chosen first",
     "MED", "LOW", "LOW", "LOW", "LOW"),
    ("Cable management",
     "Conduits modelled per arm, harness carried as a distributed mass allowance",
     "Explicit routed channels in the spine, per-joint service loops, connector "
     "bulkheads at every module boundary",
     "Needed for the modularity claim to be real: if the harness is not routed in CAD, "
     "'replaceable module' is an assertion, not a design",
     "MED", "MED", "LOW", "LOW", "LOW"),
]

RANK = {"CRITICAL": "crit", "HIGH": "hi", "MED": "med", "LOW": "lo", "NONE": "no"}

audit_rows = "".join(
    f'<tr><td class="b">{esc(a)}</td><td class="dim">{esc(b)}</td><td>{esc(c)}</td>'
    f'<td class="dim sm">{esc(r)}</td>'
    f'<td><span class="pill {RANK[i]}">{i}</span></td>'
    f'<td class="mono n">{df}</td><td class="mono n">{co}</td>'
    f'<td class="mono n">{mf}</td><td class="mono n">{rk}</td></tr>'
    for a, b, c, r, i, df, co, mf, rk in AUDIT)

# ---------------------------------------------------------------- geometry
geo_rows = "".join(
    f'<tr><td class="mono n">{g["dia"]}</td>'
    f'<td class="mono n">{g["coax_x8"]["in"]}"</td>'
    f'<td class="mono n">{g["coax_x8"]["A"]}</td>'
    f'<td class="mono n">{g["coax_x8"]["fwd"]}</td>'
    f'<td class="mono n sep">{g["flat8"]["in"]}"</td>'
    f'<td class="mono n">{g["flat8"]["A"]}</td>'
    f'<td class="mono n">{g["flat8"]["fwd"]}</td></tr>' for g in d["geom"])

# ---------------------------------------------------------------- arch trade
def arch_row(r, mark=""):
    cls = ' class="hl"' if mark else ""
    return (f'<tr{cls}><td class="mono">{"COAX X8" if r["arch"]=="coax_x8" else "FLAT-8"}</td>'
            f'<td class="mono n">{r["dia"]}</td><td class="mono n">{r["prop"]}"</td>'
            f'<td class="mono n">{r["discs"]}</td><td class="mono n">{r["A"]}</td>'
            f'<td class="mono n">{r["mtow"]}</td><td class="mono n">{r["dl"]}</td>'
            f'<td class="mono n">{r["p"]}</td><td class="mono n b">{r["pl"]}</td>'
            f'<td class="mono n b">{r["end"]}</td><td class="mono n">{r["fwd"]}</td>'
            f'<td class="mono n">{r["frame"]}</td><td class="mono n">{r["prop_kg"]}</td></tr>')

arch_rows = "".join(arch_row(r, r["dia"] == 1100) for r in d["arch"])

# ---------------------------------------------------------------- reach
def reach_cell(v):
    ok = v >= 150
    return f'<td class="mono n {"ok" if ok else "bad"}">{v:+d}</td>'

reach_rows = "".join(
    f'<tr><td class="mono">{"COAX X8" if r["arch"]=="coax_x8" else "FLAT-8"}</td>'
    f'<td class="mono n">{r["dia"]}</td><td class="mono n">{r["fwd"]}</td>'
    + reach_cell(r["m700"]) + reach_cell(r["m800"]) + reach_cell(r["m900"])
    + "</tr>" for r in d["reach"])

# ---------------------------------------------------------------- battery
blim_rows = "".join(
    f'<tr><td>{esc(b["label"])}</td><td class="mono n">{b["wh"]}</td>'
    f'<td class="mono n">{b["kg"]}</td><td class="mono n">{b["burst"]}</td>'
    f'<td class="mono n">{b["cap20"]}</td><td class="mono n">{b["cap22"]}</td>'
    f'<td><span class="pill {"ok" if b["cap20"] >= NOM["mtow"] else "bad"}">'
    f'{"CLOSES" if b["cap20"] >= NOM["mtow"] else "SHORT"}</span></td></tr>'
    for b in d["blim"])

pack_rows = "".join(
    f'<tr class="{"hl" if p["ok"] else ""}"><td>{esc(p["lab"])}</td>'
    f'<td class="mono n">{p["s"]}S</td><td class="mono n">{p["ah"]}</td>'
    f'<td class="mono n">{p["wh"]}</td><td class="mono n">{p["kg"]}</td>'
    f'<td class="mono n">{p["mtow"]}</td><td class="mono n">{p["end"]}</td>'
    f'<td class="mono n">{p["i"]}</td><td class="mono n">{p["c"]}</td>'
    f'<td><span class="pill {"ok" if p["ok"] else "bad"}">'
    f'{"YES" if p["ok"] else "NO"}</span></td></tr>' for p in d["packs"])

# ---------------------------------------------------------------- mission
mission_rows = "".join(
    f'<tr><td class="b">{esc(r["case"].split()[0])}</td>'
    f'<td class="dim sm">{esc(" ".join(r["case"].split()[1:]))}</td>'
    f'<td class="mono n">{r["mtow"]}</td><td class="mono n">{r["dl"]}</td>'
    f'<td class="mono n">{r["p"]}</td><td class="mono n">{r["i"]}</td>'
    f'<td class="mono n b">{r["twr"]}</td><td class="mono n b">{r["end"]}</td></tr>'
    for r in (MIN, NOM, MAX))

# ---------------------------------------------------------------- torque
T = d["torque"]
JOINTS = [("J1", "base yaw", "Z", "±180", T["J1"], 90, 190, "harmonic + slew bearing"),
          ("J2", "shoulder pitch", "Y", "±115", T["J2"], 60, 285, "harmonic, brake"),
          ("J3", "elbow pitch", "Y", "±160", T["J3"], 70, 250, "harmonic, brake"),
          ("J4", "wrist roll", "X", "±180", T["J4"], 150, 85, "cycloidal / planetary"),
          ("J5", "wrist pitch", "Y", "±120", T["J5"], 130, 75, "cycloidal / planetary"),
          ("J6", "wrist yaw", "Z", "±180", T["J6"], 180, 65, "planetary")]
joint_rows = "".join(
    f'<tr><td class="mono b">{j}</td><td>{n}</td><td class="mono n">{ax}</td>'
    f'<td class="mono n">{lim}</td><td class="mono n b">{tq:.1f}</td>'
    f'<td class="mono n">{tq*1.6:.0f}</td><td class="mono n">{sp}</td>'
    f'<td class="mono n">{L}</td><td class="dim sm">{note}</td></tr>'
    for j, n, ax, lim, tq, sp, L, note in JOINTS)

# ---------------------------------------------------------------- rating
RATE = [
    ("Architecture", 5, 9, False, "Spine + coaxial X8 + modular arms is a defensible "
     "industrial architecture; loses a point until the spine is stress-checked"),
    ("Mechanical engineering", 5, 8, True, "Interfaces defined, loads calculated; "
     "no FEA yet"),
    ("Manipulator", 4, 9, False, "6-DOF with joint torques derived from the actual "
     "load case, replaceable modules, hard stops, routed cabling"),
    ("Repair capability", 4, 9, False, "Quick-change interface with mechanical, "
     "electrical and fluid paths; gripper + nozzle + expansion"),
    ("Liquid-service system", 2, 9, False, "Fully integrated reservoir/pump/filter/"
     "check-valve chain with CG accounted for"),
    ("Propulsion", 6, 9, True, "Architecture and sizing calculated from momentum "
     "theory; motor/prop combination not bench-verified"),
    ("Power", 4, 8, True, "Current limit identified as the binding constraint and "
     "designed around; endurance is an estimate until flight test"),
    ("CG", 5, 9, False, "Active CG rail with travel derived from the actual arm and "
     "fluid excursions"),
    ("Sensors", 6, 9, False, "Every position justified by FOV, occlusion and "
     "manipulator-corridor analysis"),
    ("Component realism", 3, 9, False, "Realistic envelopes, connectors, heat paths; "
     "1 point withheld until vendor CAD replaces placeholders"),
    ("Maintainability", 5, 9, False, "Every module removable without dismantling "
     "adjacent systems; verified by extraction-path checks in CAD"),
    ("Manufacturability", 6, 9, False, "CF plate/tube + CNC AL brackets + printed "
     "covers, standard fasteners throughout"),
    ("Structural robustness", 4, 7, True, "Sections sized by hand calculation; "
     "cannot exceed 7 without FEA and a landing-load case"),
    ("Industrial appearance", 5, 9, False, "Graphite/black with controlled blue "
     "identification accents, service panels, hardpoints"),
    ("Mission suitability", 6, 10, False, "Direct wrist tooling clears a flat facade "
     "by 213 mm; all seven configurations supported"),
]
cur = sum(r[1] for r in RATE) / len(RATE)
prj = sum(r[2] for r in RATE) / len(RATE)
TAGHTML = ' <span class="tag">CAD REVIEW ONLY &mdash; VALIDATION REQUIRED</span>'
rate_rows = "".join(
    f'<tr><td>{esc(n)}</td><td class="mono n">{a}</td><td class="mono n b">{b}</td>'
    f'<td class="mono n {"up" if b > a else ""}">{b-a:+d}</td>'
    f'<td class="dim sm">{esc(note)}{TAGHTML if v else ""}</td></tr>'
    for n, a, b, v, note in RATE)

HTML = f"""<title>AVIAN Architecture Freeze</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;700&family=Saira+Condensed:wght@500;600;700&family=Source+Sans+3:wght@400;600;700&display=swap">
<style>
:root {{
  --bg:#eceef1; --surf:#ffffff; --surf2:#f4f6f8; --surf3:#e6e9ed;
  --ink:#14181d; --ink2:#57616c; --ink3:#8b95a1;
  --rule:#d3d9e0; --rule2:#e6eaee;
  --acc:#1c5f96; --acc-bg:#e3edf5;
  --ok:#1a6f45; --ok-bg:#e6f2eb;
  --warn:#9c5c07; --warn-bg:#fbf1e0;
  --bad:#a32a26; --bad-bg:#fbe9e8;
  --sh:0 1px 2px rgba(20,24,29,.06),0 10px 26px -16px rgba(20,24,29,.25);
}}
@media (prefers-color-scheme:dark) {{
  :root:not([data-theme="light"]) {{
    --bg:#0d1116; --surf:#151b22; --surf2:#1a222b; --surf3:#212b35;
    --ink:#e8eef5; --ink2:#98a4b1; --ink3:#6b7784;
    --rule:#28323d; --rule2:#1f2831;
    --acc:#5aa9db; --acc-bg:#132a3b;
    --ok:#5cc08d; --ok-bg:#10251a;
    --warn:#deA24d; --warn-bg:#2a2012;
    --bad:#ef8b84; --bad-bg:#2c1614;
    --sh:0 1px 2px rgba(0,0,0,.5),0 10px 30px -18px rgba(0,0,0,.8);
  }}
}}
:root[data-theme="dark"] {{
  --bg:#0d1116; --surf:#151b22; --surf2:#1a222b; --surf3:#212b35;
  --ink:#e8eef5; --ink2:#98a4b1; --ink3:#6b7784;
  --rule:#28323d; --rule2:#1f2831;
  --acc:#5aa9db; --acc-bg:#132a3b;
  --ok:#5cc08d; --ok-bg:#10251a;
  --warn:#dea24d; --warn-bg:#2a2012;
  --bad:#ef8b84; --bad-bg:#2c1614;
  --sh:0 1px 2px rgba(0,0,0,.5),0 10px 30px -18px rgba(0,0,0,.8);
}}
*{{box-sizing:border-box}}
body{{margin:0;background:var(--bg);color:var(--ink);
 font-family:"Source Sans 3",system-ui,sans-serif;font-size:16px;line-height:1.6}}
.wrap{{max-width:1240px;margin:0 auto;padding:0 22px 100px}}
h1,h2,h3{{font-family:"Saira Condensed","Source Sans 3",sans-serif;margin:0;
 text-wrap:balance;letter-spacing:.005em}}
.mono{{font-family:"JetBrains Mono",ui-monospace,monospace;font-size:.88em;
 font-variant-numeric:tabular-nums}}
.n{{text-align:right}} .dim{{color:var(--ink2)}} .b{{font-weight:700}}
.sm{{font-size:13px;line-height:1.45}}
.up{{color:var(--ok);font-weight:700}}

header{{padding:46px 0 20px;border-bottom:3px solid var(--ink)}}
.eyebrow{{font-family:"JetBrains Mono",monospace;font-size:11px;letter-spacing:.2em;
 text-transform:uppercase;color:var(--acc);margin-bottom:12px}}
h1{{font-size:clamp(44px,7vw,86px);font-weight:700;line-height:.94;
 text-transform:uppercase;letter-spacing:-.01em}}
.tag1{{display:block;font-family:"Source Sans 3",sans-serif;font-size:clamp(15px,1.9vw,20px);
 font-weight:400;text-transform:none;color:var(--ink2);margin-top:14px;max-width:66ch;
 line-height:1.45;letter-spacing:0}}
.meta{{display:flex;flex-wrap:wrap;gap:2px 30px;margin-top:22px;
 font-family:"JetBrains Mono",monospace;font-size:11.5px;color:var(--ink2)}}
.meta b{{color:var(--ink)}}

.verdict{{background:var(--surf);border:1px solid var(--rule);border-top:4px solid var(--warn);
 padding:26px 30px;margin:30px 0 8px;box-shadow:var(--sh)}}
.verdict h2{{font-size:26px;font-weight:700;margin-bottom:12px;text-transform:uppercase}}
.verdict p{{max-width:88ch}}
.big{{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:1px;
 background:var(--rule);border:1px solid var(--rule);margin:26px 0}}
.big div{{background:var(--surf);padding:15px 17px}}
.big .v{{font-family:"Saira Condensed",sans-serif;font-size:30px;font-weight:700;
 line-height:1;font-variant-numeric:tabular-nums}}
.big .v s{{text-decoration:none;font-size:15px;color:var(--ink2);margin-left:3px}}
.big .k{{font-family:"JetBrains Mono",monospace;font-size:10px;letter-spacing:.11em;
 text-transform:uppercase;color:var(--ink3);margin-top:5px}}

section{{margin-top:60px}}
.sh{{display:flex;align-items:baseline;gap:15px;border-bottom:2px solid var(--rule);
 padding-bottom:8px;margin-bottom:22px}}
.sn{{font-family:"JetBrains Mono",monospace;font-size:11.5px;font-weight:700;
 color:var(--acc);letter-spacing:.1em;white-space:nowrap}}
h2{{font-size:clamp(22px,3vw,32px);font-weight:600;text-transform:uppercase}}
h3{{font-size:17px;font-weight:700;margin:30px 0 9px}}
p{{margin:0 0 14px;max-width:86ch}}
.lede{{font-size:17.5px;color:var(--ink2);max-width:76ch}}
ul{{margin:0 0 14px;padding-left:19px;max-width:86ch}} li{{margin-bottom:6px}}
code{{font-family:"JetBrains Mono",monospace;font-size:.86em;background:var(--surf3);
 padding:1px 5px;border-radius:2px}}

.tw{{overflow-x:auto;border:1px solid var(--rule);background:var(--surf);box-shadow:var(--sh)}}
table{{border-collapse:collapse;width:100%;font-size:13.5px;min-width:560px}}
th{{font-family:"JetBrains Mono",monospace;font-size:10px;letter-spacing:.09em;
 text-transform:uppercase;color:var(--ink3);font-weight:700;text-align:left;
 padding:10px 13px;background:var(--surf2);border-bottom:2px solid var(--rule);
 white-space:nowrap;vertical-align:bottom}}
th.n{{text-align:right}}
td{{padding:9px 13px;border-bottom:1px solid var(--rule2);vertical-align:top}}
tbody tr:last-child td{{border-bottom:none}}
tr.hl td{{background:var(--acc-bg)}}
td.sep{{border-left:1px solid var(--rule)}} th.sep{{border-left:1px solid var(--rule)}}
td.ok{{color:var(--ok);font-weight:700}} td.bad{{color:var(--bad);font-weight:700}}
.pill{{font-family:"JetBrains Mono",monospace;font-size:9.5px;font-weight:700;
 letter-spacing:.08em;padding:2px 7px;border:1px solid currentColor;white-space:nowrap}}
.pill.crit{{color:var(--bad);background:var(--bad-bg)}}
.pill.hi{{color:var(--warn);background:var(--warn-bg)}}
.pill.med{{color:var(--acc);background:var(--acc-bg)}}
.pill.lo,.pill.no{{color:var(--ink3)}}
.pill.ok{{color:var(--ok);background:var(--ok-bg)}}
.pill.bad{{color:var(--bad);background:var(--bad-bg)}}
.tag{{font-family:"JetBrains Mono",monospace;font-size:9px;letter-spacing:.06em;
 color:var(--warn);border:1px solid var(--warn);padding:1px 5px;white-space:nowrap}}

.box{{background:var(--surf);border:1px solid var(--rule);border-left:3px solid var(--acc);
 padding:19px 23px;margin:22px 0;box-shadow:var(--sh)}}
.box.warn{{border-left-color:var(--warn);background:var(--warn-bg)}}
.box.ok{{border-left-color:var(--ok)}}
.box h3{{margin-top:0}} .box p:last-child{{margin-bottom:0}}
.two{{display:grid;grid-template-columns:repeat(auto-fit,minmax(310px,1fr));gap:26px}}
.frozen{{font-family:"JetBrains Mono",monospace;font-size:12.5px;line-height:1.95;
 background:var(--surf);border:1px solid var(--rule);padding:18px 21px;
 box-shadow:var(--sh);overflow-x:auto}}
.frozen b{{color:var(--acc)}}
.gate{{background:var(--acc-bg);border:1px solid var(--acc);padding:24px 28px;margin:26px 0}}
.gate h3{{margin-top:0;color:var(--acc)}}
figure{{margin:26px 0;background:var(--surf);border:1px solid var(--rule);
 padding:20px;box-shadow:var(--sh)}}
figcaption{{font-size:13px;color:var(--ink2);margin-top:12px}}
svg{{display:block;width:100%;height:auto}}
footer{{margin-top:70px;padding-top:18px;border-top:3px solid var(--ink);
 font-family:"JetBrains Mono",monospace;font-size:11px;color:var(--ink3);
 display:flex;flex-wrap:wrap;gap:5px 26px}}
</style>

<div class="wrap">
<header>
  <div class="eyebrow">Phase 1 audit + Phase 2 trade study &middot; architecture freeze proposal &middot; rev A</div>
  <h1>AVIAN
    <span class="tag1">Industrial aerial inspection, repair, liquid-service and
    manipulation UAV. This document is the gate before any CAD is cut &mdash; it
    audits the baseline, resolves the propulsion and power architecture
    quantitatively, and proposes the parameter set to freeze.</span>
  </h1>
  <div class="meta">
    <div>STATUS <b>ARCHITECTURE NOT YET FROZEN</b></div>
    <div>GEOMETRY <b>NOT STARTED &mdash; BY INSTRUCTION</b></div>
    <div>CAD PLATFORM <b>ONSHAPE</b></div>
    <div>NEXT GATE <b>YOUR SIGN-OFF</b></div>
  </div>
</header>

<div class="verdict">
  <h2>The finding that drives everything</h2>
  <p><b>The briefed power system cannot fly the briefed mission, and the limit is
  current, not energy.</b> A single 6S 16 Ah LiPo delivers roughly 240 A of
  sustainable burst. Fed into momentum theory at a coaxial X8's disc area, that
  supports an aircraft of about <b>9.4 kg at T/W 2.0</b>. The briefed mission &mdash; a
  900 mm six-axis manipulator, a 2 kg payload budget and an integrated
  liquid-service module &mdash; builds an aircraft of about <b>{NOM['mtow']} kg</b>.
  The gap is a factor of 2.3, and no amount of structural optimisation closes it.</p>
  <p>The second finding is more useful: <b>the 700&ndash;1000 mm size class is also
  wrong, but shrinking is not the answer &mdash; the aircraft has to grow to about
  1100 mm.</b> Below that, disc loading and pack size spiral. Above about 1300 mm,
  the airframe grows faster than the aerodynamic gain. And at 1100 mm a
  <b>900 mm arm reaches a flat vertical facade with 213 mm of rotor clearance</b>,
  which eliminates the reach-extension lance the previous aircraft needed
  entirely.</p>
</div>

<div class="big">
  <div><div class="v">1100<s>mm</s></div><div class="k">Diagonal (proposed)</div></div>
  <div><div class="v">{NOM['mtow']}<s>kg</s></div><div class="k">MTOW, nominal</div></div>
  <div><div class="v">{NOM['twr']}</div><div class="k">T/W, nominal</div></div>
  <div><div class="v">{NOM['end']}<s>min</s></div><div class="k">Hover, estimate</div></div>
  <div><div class="v">900<s>mm</s></div><div class="k">Arm reach</div></div>
  <div><div class="v">+213<s>mm</s></div><div class="k">Wall clearance</div></div>
</div>
<p class="dim sm">All performance figures on this page are <b>CALCULATED</b> from the
stated model, driven by <b>ESTIMATE</b> and <b>ASSUMED</b> inputs. Nothing here is
<b>VERIFIED</b>. Verification requires a thrust-stand campaign and flight test.</p>

<section>
  <div class="sh"><span class="sn">PHASE 1</span><h2>Baseline audit &mdash; current &rarr; proposed</h2></div>
  <p class="lede">Audited against the aircraft actually in hand: a flat-8, 1950 mm,
  31.4 kg structural-repair platform with a 700 mm manipulator and an 838 mm
  reach-extension lance. Good subsystems are kept. Ranked by engineering impact,
  then by what each change costs.</p>
  <div class="tw"><table>
    <thead><tr><th>Subsystem</th><th>Current</th><th>Proposed for AVIAN</th>
    <th>Engineering reason</th><th>Impact</th><th class="n">Difficulty</th>
    <th class="n">Cost</th><th class="n">Mfg</th><th class="n">Risk</th></tr></thead>
    <tbody>{audit_rows}</tbody>
  </table></div>
  <div class="box ok">
    <h3>Kept unchanged &mdash; already sound</h3>
    <p>Arm-root clamp concept and elastomeric isolation; dual-antenna RTK with a
    300 mm heading baseline; LiDAR mounted above the rotor plane for an
    unobstructed horizon; off-centreline RGB gimbal (it keeps the manipulator out
    of the camera's sight line to the work point); the parametric build method
    itself, including the automated clearance-check suite. These carry across.</p>
  </div>
</section>

<section>
  <div class="sh"><span class="sn">PHASE 2.1</span><h2>Geometry &mdash; what propeller actually fits</h2></div>
  <p class="lede">This single table explains why coaxial X8 is the right
  architecture for a compact aircraft, and it is the opposite of the intuition
  that "coaxial halves your disc area".</p>
  <p>With eight in-plane arms the adjacent-motor spacing is
  <code>2R&middot;sin(22.5&deg;)</code>; with four arms it is
  <code>2R&middot;sin(45&deg;)</code> &mdash; <b>1.85&times; larger</b>. At the same
  diagonal a coaxial X8 fits a propeller nearly twice the diameter of a flat-8's,
  and disc area goes as D&sup2;. Four big discs beat eight small ones until the
  aircraft gets large enough for the flat-8's props to grow.</p>
  <div class="tw"><table>
    <thead><tr><th class="n">Diagonal<br>mm</th>
      <th class="n">Coax X8<br>max prop</th><th class="n">disc area<br>m&sup2;</th>
      <th class="n">fwd extent<br>mm</th>
      <th class="n sep">Flat-8<br>max prop</th><th class="n">disc area<br>m&sup2;</th>
      <th class="n">fwd extent<br>mm</th></tr></thead>
    <tbody>{geo_rows}</tbody>
  </table></div>
  <p class="dim sm">Assumes a tip-to-tip gap of {C['tip_gap']*100:.0f}&nbsp;% of
  propeller diameter. "Forward extent" is the largest +X coordinate any rotor disc
  reaches, in an X layout with no arm on the forward centreline.</p>
</section>

<section>
  <div class="sh"><span class="sn">PHASE 2.2</span><h2>Architecture trade &mdash; equal energy, equal mission</h2></div>
  <p class="lede">Both architectures given the identical {d['pack_ref']['wh']} Wh /
  {d['pack_ref']['kg']} kg pack and the identical nominal mission, then sized to
  convergence. This is the apples-to-apples comparison.</p>
  <div class="tw"><table>
    <thead><tr><th>Arch</th><th class="n">diag</th><th class="n">prop</th>
      <th class="n">discs</th><th class="n">A m&sup2;</th><th class="n">MTOW kg</th>
      <th class="n">DL N/m&sup2;</th><th class="n">P hover W</th>
      <th class="n">g/W</th><th class="n">endur min</th><th class="n">fwd mm</th>
      <th class="n">frame kg</th><th class="n">prop kg</th></tr></thead>
    <tbody>{arch_rows}</tbody>
  </table></div>
  <div class="two">
    <div>
      <h3>What the numbers say</h3>
      <ul>
        <li><b>Below 1100 mm, coaxial X8 wins</b> on hover power &mdash; its far
        larger propellers more than pay for the coaxial interference penalty.</li>
        <li><b>Above 1200 mm, flat-8 wins</b> on efficiency (8 independent discs,
        no interference) and carries roughly 1.4 kg less propulsion mass.</li>
        <li>The crossover sits at <b>1100&ndash;1200 mm</b>, and at the crossover the
        two are within 2 % &mdash; comfortably inside the uncertainty of the
        assumed coaxial coefficient (0.78&ndash;0.88).</li>
      </ul>
    </div>
    <div>
      <h3>Recommendation: coaxial X8 at 1100 mm</h3>
      <ul>
        <li>At 1100 mm the aerodynamic difference is <b>not decisive</b>, so the
        decision falls to the operational factors &mdash; and those favour coaxial
        clearly.</li>
        <li><b>Four arm roots instead of eight.</b> Half the structural interfaces,
        half the folding joints, a materially smaller transport case.</li>
        <li><b>Motor-out behaviour.</b> Losing one rotor of a coaxial pair leaves
        its partner producing thrust at the same station &mdash; no large
        asymmetric moment to trim out.</li>
        <li>It is also the architecture you specified. The trade study supports it
        at this size; it would not have at 1500 mm.</li>
      </ul>
    </div>
  </div>
</section>

<section>
  <div class="sh"><span class="sn">PHASE 2.3</span><h2>Arm reach vs rotor envelope</h2></div>
  <p class="lede">The constraint that killed the previous aircraft's simplicity.
  A multirotor doing contact work on a flat facade must reach past its own rotor
  disc, plus a margin for gust and position error.</p>
  <figure>
    <svg viewBox="0 0 900 300" role="img" aria-label="Plan view comparing rotor envelope with arm reach">
      <defs><marker id="ah" markerWidth="9" markerHeight="9" refX="8" refY="3"
        orient="auto"><path d="M0,0 L8,3 L0,6 z" fill="var(--acc)"/></marker></defs>
      <line x1="700" y1="20" x2="700" y2="280" stroke="var(--ink2)" stroke-width="3"/>
      <text x="708" y="42" fill="var(--ink2)" font-size="13"
        font-family="JetBrains Mono, monospace">flat vertical facade</text>
      <circle cx="180" cy="150" r="26" fill="none" stroke="var(--ink3)" stroke-width="2"/>
      <text x="180" y="196" fill="var(--ink2)" font-size="12" text-anchor="middle"
        font-family="JetBrains Mono, monospace">base_link</text>
      <circle cx="180" cy="150" r="330" fill="none" stroke="var(--bad)"
        stroke-width="1.6" stroke-dasharray="7 5"/>
      <text x="470" y="106" fill="var(--bad)" font-size="12.5"
        font-family="JetBrains Mono, monospace">rotor envelope  732 mm</text>
      <circle cx="180" cy="150" r="426" fill="none" stroke="var(--acc)" stroke-width="2"/>
      <text x="452" y="262" fill="var(--acc)" font-size="12.5"
        font-family="JetBrains Mono, monospace">tool point at 900 mm arm reach  945 mm</text>
      <line x1="510" y1="150" x2="700" y2="150" stroke="var(--acc)" stroke-width="2.2"
        marker-end="url(#ah)" marker-start="url(#ah)"/>
      <rect x="536" y="128" width="140" height="24" fill="var(--surf)"/>
      <text x="606" y="146" fill="var(--acc)" font-size="14" text-anchor="middle"
        font-weight="700" font-family="JetBrains Mono, monospace">+213 mm</text>
    </svg>
    <figcaption>Coaxial X8, 1100 mm diagonal, 27&Prime; propellers. The wrist itself
    reaches the surface &mdash; no reach-extension lance, no tool holsters, tools
    mounted directly on the wrist changer.</figcaption>
  </figure>
  <div class="tw"><table>
    <thead><tr><th>Arch</th><th class="n">diagonal</th><th class="n">fwd rotor extent</th>
      <th class="n">margin, 700 mm arm</th><th class="n">margin, 800 mm arm</th>
      <th class="n">margin, 900 mm arm</th></tr></thead>
    <tbody>{reach_rows}</tbody>
  </table></div>
  <p class="dim sm">Arm base at x = +45 mm. Green = at least 150 mm of clearance
  between the forward rotor tips and the surface, which is the minimum defensible
  standoff for RTK plus LiDAR wall-following in light gust. <b>A 900 mm arm is
  required</b>: 700 mm and 800 mm both fail at the diagonal the power system needs.</p>
</section>

<section>
  <div class="sh"><span class="sn">PHASE 2.4</span><h2>Power &mdash; the binding constraint</h2></div>
  <p class="lede">Sizing a pack by energy alone is the classic mistake here. What
  actually binds is the burst current needed to hit the thrust-to-weight target.</p>
  <h3>What each candidate pack can actually fly</h3>
  <div class="tw"><table>
    <thead><tr><th>Pack</th><th class="n">Wh</th><th class="n">kg</th>
      <th class="n">burst A</th><th class="n">max MTOW @ T/W 2.0</th>
      <th class="n">@ T/W 2.2</th><th>vs mission</th></tr></thead>
    <tbody>{blim_rows}</tbody>
  </table></div>
  <p class="dim sm">Burst limits: {C['c_lipo']:.0f}C sustained for LiPo, 8C for
  21700 Li-ion &mdash; both <b>ASSUMED</b>, conservative, and the single most
  important assumption on this page. Max MTOW is <b>CALCULATED</b> by inverting
  momentum theory at the pack's maximum deliverable electrical power.</p>

  <div class="box warn">
    <h3>The 6S question, answered with numbers</h3>
    <p>You asked me to keep 6S unless the analysis proves otherwise. Here is the
    honest result: <b>at the correct aircraft size, 6S is viable &mdash; it is just
    heavier.</b> To fly the same {NOM['mtow']} kg aircraft, a 6S system needs about
    <b>40 Ah / 900 Wh / 6.1 kg</b> and draws <b>154 A in hover</b>. A 12S system
    needs <b>19 Ah / 844 Wh / 5.7 kg</b> and draws <b>72 A</b>. Same endurance.
    The 6S pack is oversized by roughly 60 Wh purely to supply current it will
    never turn into flight time.</p>
    <p>The real cost of 6S is not the 0.4 kg. It is <b>154 A of continuous hover
    current</b> &mdash; 8 AWG main runs, high-current connectors, significant
    voltage sag under manipulator transients, and 40 A per ESC where 12S needs 20.
    For a machine that must hold position precisely while an arm moves, sag-induced
    thrust variation is a control problem, not just an efficiency one.</p>
    <p><b>Recommendation: 12S.</b> Not for endurance &mdash; voltage buys no
    endurance at equal Wh &mdash; but for current, wiring mass, ESC sizing and
    hover stability under load.</p>
  </div>

  <h3>Options built from packs you already own</h3>
  <div class="tw"><table>
    <thead><tr><th>Configuration</th><th class="n">S</th><th class="n">Ah</th>
      <th class="n">Wh</th><th class="n">kg</th><th class="n">MTOW</th>
      <th class="n">endur min</th><th class="n">A hover</th><th class="n">C hover</th>
      <th>Flies?</th></tr></thead>
    <tbody>{pack_rows}</tbody>
  </table></div>
  <p class="dim sm">Two 6S 16 Ah packs in <b>series</b> give 12S 16 Ah for the same
  mass as wiring them in parallel, at half the current &mdash; but it still lands
  just short of the mission. Three in parallel closes it and keeps 6S, at 176 A.</p>
</section>

<section>
  <div class="sh"><span class="sn">PHASE 2.5</span><h2>Proposed frozen architecture</h2></div>
  <div class="box">
    <h3>AVIAN &mdash; baseline for freeze</h3>
    <div class="frozen">
CONFIGURATION      <b>coaxial X8</b>, 4 arms at 90&deg;, X layout (no arm on the forward centreline)
DIAGONAL           <b>1100 mm</b> motor-to-motor        ROTOR   8 x {NOM['prop_in']}&Prime; CF, 4 coaxial pairs
DISC AREA          1.478 m&sup2;                              COAX SPACING   0.16 x D (176 mm)
MOTOR              8 x {NOM['tpm']} kgf class, {NOM['motor']} kg each   ESC   8 x {NOM['esc']:.0f} A, 12S
BATTERY            <b>12S {PK['ah']} Ah, {PK['wh']} Wh, {PK['kg']} kg</b> &mdash; 2 x 12S 12 Ah cartridges in parallel
MTOW               {MIN['mtow']} / <b>{NOM['mtow']}</b> / {MAX['mtow']} kg   (min / nominal / max mission)
THRUST / WEIGHT    {MIN['twr']} / <b>{NOM['twr']}</b> / {MAX['twr']}          &ge; 2.0 across the whole envelope
HOVER POWER        {MIN['p']} / <b>{NOM['p']}</b> / {MAX['p']} W
HOVER ENDURANCE    {MIN['end']} / <b>{NOM['end']}</b> / {MAX['end']} min      ESTIMATE &mdash; REQUIRES VALIDATION
DISC LOADING       {NOM['dl']} N/m&sup2; nominal
MANIPULATOR        <b>6-DOF, 900 mm reach</b>, 2.15 kg, tools direct on the wrist
ARM MOUNT          machined AL hub integral with the spine, at x = +45 mm
TOOL INTERFACE     quick-change: 3-pin mechanical lock + 12-way electrical + 1 fluid
LIQUID SERVICE     1.0 L reservoir, metering pump, 40 &micro;m filter, check valve
CG RAIL            indexed, &plusmn;60 mm travel in X, carries the battery cartridges
ARM TUBE           CF, {NOM['tube']:.0f} mm OD (root moment governs; min-gauge selected)
    </div>
  </div>
  <h3>Mission envelope</h3>
  <div class="tw"><table>
    <thead><tr><th>Case</th><th>Definition</th><th class="n">MTOW kg</th>
      <th class="n">DL N/m&sup2;</th><th class="n">P hover W</th><th class="n">A hover</th>
      <th class="n">T/W</th><th class="n">endurance min</th></tr></thead>
    <tbody>{mission_rows}</tbody>
  </table></div>
  <p class="dim sm">Mission payload defined as <b>2.0 kg = 1.0 L fluid + 0.38 kg
  gripper + 0.62 kg carried object</b>. The manipulator and the fluid-system
  hardware are aircraft equipment, not payload &mdash; the brief was ambiguous on
  this and I have taken the reading that makes the 2 kg figure achievable and
  meaningful. <b>Confirm or correct this.</b></p>
</section>

<section>
  <div class="sh"><span class="sn">PHASE 2.6</span><h2>Manipulator joint sizing</h2></div>
  <p class="lede">Torques derived from the governing load case: 1.0 kg at the tool
  point, arm fully extended horizontally at 900 mm, 1.5&times; dynamic factor.
  Actuator selection follows the torque, not the other way round.</p>
  <div class="tw"><table>
    <thead><tr><th>Joint</th><th>Function</th><th class="n">Axis</th>
      <th class="n">Range</th><th class="n">Static Nm</th><th class="n">Select &ge; Nm</th>
      <th class="n">Speed &deg;/s</th><th class="n">Link mm</th>
      <th>Actuator concept</th></tr></thead>
    <tbody>{joint_rows}</tbody>
  </table></div>
  <p class="dim sm">Wrist ordering is <b>J4 roll / J5 pitch / J6 yaw</b> per the
  AVIAN brief &mdash; a change from the previous aircraft's pitch-first wrist, and
  the better choice for aligning a tool axis to a surface normal. "Select" applies
  a 1.6&times; margin over the calculated static torque for stall and shock.</p>
</section>

<section>
  <div class="sh"><span class="sn">GATE</span><h2>What I need from you before any CAD</h2></div>
  <div class="gate">
    <h3>Four decisions, then I build</h3>
    <ol>
      <li><b>Diagonal 1100 mm.</b> This exceeds your stated 700&ndash;1000 mm class.
      At 1000 mm the aircraft still closes but hover power rises about 8 % and the
      900 mm arm's wall clearance drops to +287 mm (still fine). At 900 mm the
      pack must grow to 28.5 Ah and disc loading hits 241 N/m&sup2;.
      <b>Accept 1100 mm, or hold 1000 mm and take the efficiency hit?</b></li>
      <li><b>12S, {PK['ah']} Ah, {PK['wh']} Wh.</b> This replaces the 6S 16 Ah pack.
      6S remains possible at ~40 Ah and 154 A hover if you have a reason to stay on
      6S hardware. <b>Confirm 12S, or hold 6S?</b></li>
      <li><b>Manipulator reach 900 mm, not 700&ndash;800 mm.</b> This is what removes
      the reach-extension lance. 800 mm leaves only +113 mm of wall clearance at
      1100 mm diagonal &mdash; not enough. <b>Confirm 900 mm?</b></li>
      <li><b>MTOW ~{NOM['mtow']} kg.</b> Materially heavier than the baseline concept
      implies. If there is a hard mass ceiling I have not been told about, say so
      now &mdash; it changes the aircraft, not the detailing.</li>
    </ol>
    <p style="margin-top:14px"><b>On sign-off I proceed to Phase 3&ndash;15:</b> the
    full parameter table, the parametric model, the Onshape document structure with
    Part Studio and Assembly specifications, mate scheme, seven configurations,
    drawing set, BOM and validation checklist &mdash; built so you reconstruct AVIAN
    natively in Onshape rather than importing dumb geometry.</p>
  </div>
</section>

<section>
  <div class="sh"><span class="sn">REVIEW</span><h2>Design review rating</h2></div>
  <p class="lede">Current baseline scored against the AVIAN mission, and the score
  the proposed architecture can reach <b>once built</b>. Projected scores are the
  ceiling of this architecture, not a claim about work not yet done.</p>
  <div class="tw"><table>
    <thead><tr><th>Criterion</th><th class="n">Current</th><th class="n">Projected</th>
      <th class="n">&Delta;</th><th>Basis and limits</th></tr></thead>
    <tbody>{rate_rows}</tbody>
    <tfoot><tr><td class="b">OVERALL</td><td class="mono n b">{cur:.2f}</td>
      <td class="mono n b">{prj:.2f}</td><td class="mono n up">+{prj-cur:.2f}</td>
      <td class="dim sm">Projected {prj:.2f}/10 is achievable from CAD alone.
      Reaching 9.5+ requires the three validation items below; a genuine 10 is not
      claimable without flight test.</td></tr></tfoot>
  </table></div>
  <div class="box warn">
    <h3>What stands between {prj:.2f} and 9.5+</h3>
    <p><b>1. FEA on three parts</b> &mdash; the manipulator hub, the arm root clamp
    and the spine, under a 3g landing case plus full arm extension.
    <b>2. Thrust-stand data</b> for the actual motor/propeller/ESC combination at
    12S, which converts every performance number here from CALCULATED to VERIFIED.
    <b>3. A coaxial interference measurement</b> &mdash; the assumed 0.82 factor
    carries a &plusmn;6 % spread that propagates straight into endurance.</p>
  </div>
</section>

<section>
  <div class="sh"><span class="sn">RISK</span><h2>Remaining risks</h2></div>
  <div class="two">
    <div>
      <h3>Ranked by consequence</h3>
      <ul>
        <li><b>Coaxial interference factor.</b> Assumed 0.82; literature spans
        0.78&ndash;0.88. At 0.78 endurance falls to about {NOM['end']*0.94:.1f} min.
        Cheap to retire with a bench test.</li>
        <li><b>Manipulator mass.</b> 2.15 kg for a 900 mm / 1 kg arm is aggressive.
        Every 100 g over budget costs roughly 0.25 kg of MTOW after the propulsion
        spiral.</li>
        <li><b>Hover stability during arm motion.</b> A 2.15 kg arm swinging through
        900 mm shifts the CG by tens of millimetres in under a second. The CG rail
        helps with slow trends, not transients &mdash; this is a control problem,
        and it needs a feed-forward model.</li>
        <li><b>Fluid slosh.</b> A part-full 1.0 L reservoir is a moving mass near
        the CG. Baffling is required, and it is not free.</li>
      </ul>
    </div>
    <div>
      <h3>Accepted, with reason</h3>
      <ul>
        <li><b>{NOM['end']} min hover.</b> Short, but honest for this class. The
        mission is minutes of contact work, not hours of survey.</li>
        <li><b>{NOM['dl']} N/m&sup2; disc loading.</b> Higher than a survey drone;
        normal for a manipulation platform where compactness near structure matters
        more than endurance.</li>
        <li><b>1100 mm exceeds the stated class.</b> Deliberate, and quantified
        above.</li>
        <li><b>Single-fluid service.</b> Two-part injection returns later as a tool
        on the same interface, without touching the airframe.</li>
      </ul>
    </div>
  </div>
</section>

<footer>
  <div>AVIAN &middot; REV A &middot; PHASE 1&ndash;2</div>
  <div>MOMENTUM THEORY + BOTTOM-UP MASS MODEL</div>
  <div>CALCULATED / ESTIMATE / ASSUMED &mdash; NOTHING VERIFIED</div>
  <div>CAD PLATFORM: ONSHAPE</div>
  <div>GEOMETRY NOT STARTED PENDING SIGN-OFF</div>
</footer>
</div>
"""
open("avian_phase12.html", "w").write(HTML)
print("written", len(HTML))
