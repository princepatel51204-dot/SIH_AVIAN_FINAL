"""AVIAN Phase 2.7 -- engineering closure report."""
import html, json, math
import avian_closure as A

def e(s): return html.escape(str(s))

P24 = A.PACKS[2]
NOM = A.build(P24, "STOWED", "nozzle", 0.0, 1.0)
xn, yn, zn, M_NOM = A.cg(NOM)
MINi = A.build(P24, "STOWED", "none", 0.0, 0.0); _,_,_,M_MIN = A.cg(MINi)
MAXi = A.build(P24, "MAX_REACH", "gripper", 0.62, 1.0); _,_,_,M_MAX = A.cg(MAXi)
P_NOM, DL_NOM = A.hover_power(M_NOM)
END_NOM = P24.usable / P_NOM * 60

TQ = json.load(open("avian_torque.json"))
ROWS = TQ["torque_rows"]

# ---------------- item table --------------------------------------------
GROUPNAME = {
 "01_AIRFRAME":"01 Airframe","02_PROPULSION":"02 Propulsion","03_BATTERY":"03 Battery",
 "04_AVIONICS":"04 Avionics","05_PERCEPTION":"05 Perception","06_MANIPULATOR":"06 Manipulator",
 "09_LIQUID":"09 Liquid service","10_LANDING_GEAR":"10 Landing gear",
 "11_CABLE":"11 Cable management","15_SAFETY":"15 Safety"}
CONF = {"VENDOR":"ven","CALCULATED":"cal","ESTIMATED":"est","ASSUMED":"asm"}

groups = {}
for r in NOM:
    groups.setdefault(r[1], []).append(r)
item_html = ""
gtot = {}
for g in sorted(groups):
    rows = sorted(groups[g], key=lambda r: -r[2])
    sub = sum(r[2] for r in rows)
    gtot[g] = sub
    item_html += (f'<tr class="gh"><td colspan="3">{e(GROUPNAME.get(g,g))}</td>'
                  f'<td class="mono n b">{sub:.3f}</td><td colspan="4"></td></tr>')
    for nm, grp, m, pos, src, conf in rows:
        item_html += (f'<tr><td class="mono">{e(nm)}</td>'
                      f'<td class="dim sm" colspan="2">{e(src)}</td>'
                      f'<td class="mono n">{m:.3f}</td>'
                      f'<td class="mono n">{pos[0]:.0f}</td>'
                      f'<td class="mono n">{pos[1]:.0f}</td>'
                      f'<td class="mono n">{pos[2]:.0f}</td>'
                      f'<td><span class="cf {CONF.get(conf,"est")}">{conf}</span></td></tr>')

conf_tot = {}
for r in NOM:
    conf_tot[r[5]] = conf_tot.get(r[5], 0.0) + r[2]

# ---------------- cg study ------------------------------------------------
CG_CASES = [
 ("A", "Empty aircraft &mdash; no battery, no manipulator, no fluid",
  dict(with_battery=False, with_arm=False, fluid_l=0, tool="none")),
 ("B", "Battery installed", dict(with_arm=False, fluid_l=0, tool="none")),
 ("C", "Manipulator fitted, arm STOWED, tank dry",
  dict(arm_cfg="STOWED", fluid_l=0, tool="none")),
 ("D", "Full 1.0 L reservoir, arm stowed",
  dict(arm_cfg="STOWED", fluid_l=1.0, tool="none")),
 ("E", "Gripper installed, arm stowed",
  dict(arm_cfg="STOWED", fluid_l=1.0, tool="gripper")),
 ("F", "Arm DEPLOYED, gripper, 1.0 L",
  dict(arm_cfg="DEPLOYED", fluid_l=1.0, tool="gripper")),
 ("G", "Arm MAX REACH, gripper + 0.62 kg carried",
  dict(arm_cfg="MAX_REACH", fluid_l=1.0, tool="gripper", carried=0.62)),
 ("H", "Arm SIDE REACH, gripper + 0.62 kg &mdash; worst LATERAL",
  dict(arm_cfg="SIDE_REACH", fluid_l=1.0, tool="gripper", carried=0.62)),
 ("I", "Arm DOWN REACH, gripper + 0.62 kg &mdash; worst VERTICAL",
  dict(arm_cfg="DOWN_REACH", fluid_l=1.0, tool="gripper", carried=0.62)),
 ("J", "Tool-service pose, tank empty",
  dict(arm_cfg="TOOL_SERVICE", fluid_l=0.0, tool="none")),
 ("K", "WORST COMBINED &mdash; max reach, loaded, tank empty",
  dict(arm_cfg="MAX_REACH", fluid_l=0.0, tool="gripper", carried=0.62)),
]
cg_rows = ""
cgx = []
cgy = []
for k, lab, kw in CG_CASES:
    it = A.build(P24, **kw)
    x, y, z, m = A.cg(it)
    if k != "A":
        cgx.append(x); cgy.append(y)
    worst = ' class="hl"' if k in ("H", "K") else ""
    cg_rows += (f'<tr{worst}><td class="mono b">{k}</td><td>{lab}</td>'
                f'<td class="mono n">{m:.2f}</td><td class="mono n b">{x:+.1f}</td>'
                f'<td class="mono n b">{y:+.1f}</td><td class="mono n">{z:+.1f}</td></tr>')
EXC_X = max(cgx) - min(cgx)
EXC_Y = max(cgy) - min(cgy)

# ---------------- trim ----------------------------------------------------
arm_x = A.R_MOT * math.cos(math.radians(45))
trim_rows = ""
for off in (20, 40, 58, 70, 80):
    Mo = M_NOM * A.G * off / 1000.0
    dT = Mo / (2 * arm_x / 1000.0)
    trim_rows += (f'<tr><td class="mono n">{off}</td><td class="mono n">{Mo:.2f}</td>'
                  f'<td class="mono n">{dT:.1f}</td>'
                  f'<td class="mono n">{100*dT/(M_NOM*A.G/4):.1f}</td>'
                  f'<td class="mono n b">{100*dT/(A.T_PAIR_MAX*A.G):.1f}</td></tr>')

# ---------------- structure ----------------------------------------------
L_arm = (A.R_MOT - 210) / 1000.0
I = math.pi * (25**4 - 21**4) / 64
Z = I / 12.5
struct_rows = ""
for nm, n in (("Hover", 1.0), ("2 g manoeuvre", 2.0), ("3 g landing", 3.0),
              ("5 g hard landing", 5.0), ("Motor-out transient", 1.6)):
    tot = n * M_MAX * A.G
    per = tot / 4.0
    Mr = per * L_arm
    sig = Mr * 1000 / Z
    struct_rows += (f'<tr><td>{nm}</td><td class="mono n">{n:.1f}</td>'
                    f'<td class="mono n">{tot:.0f}</td><td class="mono n">{per:.0f}</td>'
                    f'<td class="mono n">{Mr:.1f}</td><td class="mono n">{sig:.0f}</td>'
                    f'<td class="mono n b">{600/sig:.1f}</td></tr>')
Tp = A.T_PAIR_MAX * A.G
Mr_max = Tp * L_arm
sig_max = Mr_max * 1000 / Z
struct_rows += (f'<tr class="hl"><td>All rotors 100 % thrust</td><td class="mono n">&mdash;</td>'
                f'<td class="mono n">{4*Tp:.0f}</td><td class="mono n">{Tp:.0f}</td>'
                f'<td class="mono n">{Mr_max:.1f}</td><td class="mono n">{sig_max:.0f}</td>'
                f'<td class="mono n b">{600/sig_max:.1f}</td></tr>')
defl = Tp * (L_arm*1000)**3 / (3*90000*I)

# ---------------- torque --------------------------------------------------
FN = {"J1":"base yaw","J2":"shoulder pitch","J3":"elbow pitch",
      "J4":"wrist roll","J5":"wrist pitch","J6":"wrist yaw"}
tq_rows = ""
for r in ROWS:
    tq_rows += (f'<tr><td class="mono b">{r["j"]}</td><td>{FN[r["j"]]}</td>'
                f'<td class="mono n">{r["g"]:.2f}</td><td class="mono n">{r["c"]:.2f}</td>'
                f'<td class="mono n">{r["i"]:.2f}</td>'
                f'<td class="mono n b">{r["gov"]:.2f}</td>'
                f'<td class="mono n">{r["gov"]*1.6:.1f}</td>'
                f'<td class="mono n b">{r["sel"]:.0f}</td>'
                f'<td class="mono n">{r["margin"]:.2f}&times;</td>'
                f'<td class="mono n">{r["spd"]}</td>'
                f'<td class="mono n">{r["kg"]:.3f}</td></tr>')

# ---------------- packs ---------------------------------------------------
pack_rows = ""
for p in A.PACKS:
    r = []
    for cfgn, tool, carried, fluid in (("STOWED","none",0.0,0.0),
                                       ("STOWED","nozzle",0.0,1.0),
                                       ("MAX_REACH","gripper",0.62,1.0)):
        it = A.build(p, cfgn, tool, carried, fluid)
        _,_,_,m = A.cg(it)
        r.append((m, A.T_TOTAL_MAX/m))
    ph,_ = A.hover_power(r[1][0])
    sel = ' class="hl"' if "24 Ah" in p.label else ""
    pack_rows += (f'<tr{sel}><td>{e(p.label)}</td><td class="mono n">{p.wh:.0f}</td>'
                  f'<td class="mono n">{p.kg:.2f}</td><td class="mono n">{p.burst_a:.0f}</td>'
                  f'<td class="mono n">{r[0][0]:.2f}</td><td class="mono n">{r[1][0]:.2f}</td>'
                  f'<td class="mono n">{r[2][0]:.2f}</td>'
                  f'<td class="mono n b">{r[0][1]:.2f}</td>'
                  f'<td class="mono n b">{r[1][1]:.2f}</td>'
                  f'<td class="mono n b">{r[2][1]:.2f}</td>'
                  f'<td class="mono n">{p.usable/ph*60:.1f}</td></tr>')

# ---------------- arm configs --------------------------------------------
cfg_rows = ""
for name, q in A.CFG.items():
    items, tp = A.arm_items(q, "gripper", 0.62)
    zmin = min(p[2] for _,_,p in items)
    gnd = zmin - (-530)
    cfg_rows += (f'<tr><td class="mono b">{name}</td>'
                 f'<td class="mono">{", ".join(f"{v:+.0f}" for v in q)}</td>'
                 f'<td class="mono n">{tp[0]:.0f}</td><td class="mono n">{tp[1]:.0f}</td>'
                 f'<td class="mono n">{tp[2]:.0f}</td><td class="mono n">{zmin:.0f}</td>'
                 f'<td class="mono n {"bad" if gnd<0 else "ok"}">{gnd:+.0f}</td></tr>')

K = A.PROP_KIT

HTML = f"""<title>AVIAN Engineering Closure</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;700&family=Saira+Condensed:wght@500;600;700&family=Source+Sans+3:wght@400;600;700&display=swap">
<style>
:root {{
  --bg:#eceef1; --surf:#fff; --surf2:#f4f6f8; --surf3:#e6e9ed;
  --ink:#14181d; --ink2:#57616c; --ink3:#8b95a1; --rule:#d3d9e0; --rule2:#e6eaee;
  --acc:#1c5f96; --acc-bg:#e3edf5; --ok:#1a6f45; --ok-bg:#e6f2eb;
  --warn:#9c5c07; --warn-bg:#fbf1e0; --bad:#a32a26; --bad-bg:#fbe9e8;
  --ven:#5a3f9e; --ven-bg:#eee9f7;
  --sh:0 1px 2px rgba(20,24,29,.06),0 10px 26px -16px rgba(20,24,29,.25);
}}
@media (prefers-color-scheme:dark) {{ :root:not([data-theme="light"]) {{
  --bg:#0d1116; --surf:#151b22; --surf2:#1a222b; --surf3:#212b35;
  --ink:#e8eef5; --ink2:#98a4b1; --ink3:#6b7784; --rule:#28323d; --rule2:#1f2831;
  --acc:#5aa9db; --acc-bg:#132a3b; --ok:#5cc08d; --ok-bg:#10251a;
  --warn:#dea24d; --warn-bg:#2a2012; --bad:#ef8b84; --bad-bg:#2c1614;
  --ven:#b49ae8; --ven-bg:#1e1830;
  --sh:0 1px 2px rgba(0,0,0,.5),0 10px 30px -18px rgba(0,0,0,.8); }} }}
:root[data-theme="dark"] {{
  --bg:#0d1116; --surf:#151b22; --surf2:#1a222b; --surf3:#212b35;
  --ink:#e8eef5; --ink2:#98a4b1; --ink3:#6b7784; --rule:#28323d; --rule2:#1f2831;
  --acc:#5aa9db; --acc-bg:#132a3b; --ok:#5cc08d; --ok-bg:#10251a;
  --warn:#dea24d; --warn-bg:#2a2012; --bad:#ef8b84; --bad-bg:#2c1614;
  --ven:#b49ae8; --ven-bg:#1e1830;
  --sh:0 1px 2px rgba(0,0,0,.5),0 10px 30px -18px rgba(0,0,0,.8); }}
*{{box-sizing:border-box}}
body{{margin:0;background:var(--bg);color:var(--ink);
 font-family:"Source Sans 3",system-ui,sans-serif;font-size:16px;line-height:1.6}}
.wrap{{max-width:1300px;margin:0 auto;padding:0 20px 100px}}
h1,h2,h3{{font-family:"Saira Condensed",sans-serif;margin:0;text-wrap:balance}}
.mono{{font-family:"JetBrains Mono",monospace;font-size:.86em;font-variant-numeric:tabular-nums}}
.n{{text-align:right}} .dim{{color:var(--ink2)}} .b{{font-weight:700}}
.sm{{font-size:12.5px;line-height:1.42}}
header{{padding:44px 0 18px;border-bottom:3px solid var(--ink)}}
.eyebrow{{font-family:"JetBrains Mono",monospace;font-size:11px;letter-spacing:.2em;
 text-transform:uppercase;color:var(--acc);margin-bottom:11px}}
h1{{font-size:clamp(40px,6.5vw,78px);font-weight:700;line-height:.95;text-transform:uppercase}}
.sub{{display:block;font-family:"Source Sans 3",sans-serif;font-size:clamp(15px,1.8vw,19px);
 font-weight:400;text-transform:none;color:var(--ink2);margin-top:13px;max-width:70ch;line-height:1.45}}
.meta{{display:flex;flex-wrap:wrap;gap:2px 28px;margin-top:20px;
 font-family:"JetBrains Mono",monospace;font-size:11.5px;color:var(--ink2)}}
.meta b{{color:var(--ink)}}
.big{{display:grid;grid-template-columns:repeat(auto-fit,minmax(132px,1fr));gap:1px;
 background:var(--rule);border:1px solid var(--rule);margin:26px 0}}
.big div{{background:var(--surf);padding:14px 16px}}
.big .v{{font-family:"Saira Condensed",sans-serif;font-size:29px;font-weight:700;line-height:1;
 font-variant-numeric:tabular-nums}}
.big .v s{{text-decoration:none;font-size:14px;color:var(--ink2);margin-left:3px}}
.big .k{{font-family:"JetBrains Mono",monospace;font-size:9.5px;letter-spacing:.11em;
 text-transform:uppercase;color:var(--ink3);margin-top:5px}}
section{{margin-top:56px}}
.sh{{display:flex;align-items:baseline;gap:14px;border-bottom:2px solid var(--rule);
 padding-bottom:8px;margin-bottom:20px}}
.sn{{font-family:"JetBrains Mono",monospace;font-size:11.5px;font-weight:700;color:var(--acc);
 letter-spacing:.1em;white-space:nowrap}}
h2{{font-size:clamp(21px,2.8vw,30px);font-weight:600;text-transform:uppercase}}
h3{{font-size:16.5px;font-weight:700;margin:26px 0 8px}}
p{{margin:0 0 13px;max-width:88ch}} .lede{{font-size:17px;color:var(--ink2);max-width:78ch}}
ul,ol{{margin:0 0 13px;padding-left:19px;max-width:88ch}} li{{margin-bottom:6px}}
code{{font-family:"JetBrains Mono",monospace;font-size:.85em;background:var(--surf3);padding:1px 5px}}
.tw{{overflow-x:auto;border:1px solid var(--rule);background:var(--surf);box-shadow:var(--sh)}}
.tw.tall{{max-height:640px;overflow-y:auto}}
table{{border-collapse:collapse;width:100%;font-size:13px;min-width:600px}}
th{{font-family:"JetBrains Mono",monospace;font-size:9.5px;letter-spacing:.09em;
 text-transform:uppercase;color:var(--ink3);font-weight:700;text-align:left;padding:9px 11px;
 background:var(--surf2);border-bottom:2px solid var(--rule);white-space:nowrap;
 position:sticky;top:0;z-index:2}}
th.n{{text-align:right}}
td{{padding:7px 11px;border-bottom:1px solid var(--rule2);vertical-align:top}}
tbody tr:last-child td{{border-bottom:none}}
tr.hl td{{background:var(--acc-bg)}}
tr.gh td{{background:var(--surf3);font-family:"Saira Condensed",sans-serif;
 font-size:14px;font-weight:700;text-transform:uppercase;letter-spacing:.04em}}
td.ok{{color:var(--ok);font-weight:700}} td.bad{{color:var(--bad);font-weight:700}}
.cf{{font-family:"JetBrains Mono",monospace;font-size:9px;font-weight:700;letter-spacing:.06em;
 padding:1px 6px;border:1px solid currentColor;white-space:nowrap}}
.cf.ven{{color:var(--ven);background:var(--ven-bg)}}
.cf.cal{{color:var(--ok);background:var(--ok-bg)}}
.cf.est{{color:var(--warn);background:var(--warn-bg)}}
.box{{background:var(--surf);border:1px solid var(--rule);border-left:3px solid var(--acc);
 padding:18px 22px;margin:20px 0;box-shadow:var(--sh)}}
.box.warn{{border-left-color:var(--warn);background:var(--warn-bg)}}
.box.ok{{border-left-color:var(--ok)}}
.box.bad{{border-left-color:var(--bad);background:var(--bad-bg)}}
.box h3{{margin-top:0}} .box p:last-child{{margin-bottom:0}}
.two{{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:24px}}
.frozen{{font-family:"JetBrains Mono",monospace;font-size:12.5px;line-height:1.95;
 background:var(--surf);border:1px solid var(--rule);padding:18px 20px;box-shadow:var(--sh);
 overflow-x:auto}}
.frozen b{{color:var(--acc)}}
.gate{{background:var(--ok-bg);border:1px solid var(--ok);padding:22px 26px;margin:24px 0}}
.gate h3{{margin-top:0;color:var(--ok)}}
footer{{margin-top:66px;padding-top:16px;border-top:3px solid var(--ink);
 font-family:"JetBrains Mono",monospace;font-size:11px;color:var(--ink3);
 display:flex;flex-wrap:wrap;gap:5px 24px}}
</style>

<div class="wrap">
<header>
  <div class="eyebrow">Phase 2.7 &middot; engineering closure &middot; seven gates &middot; rev A</div>
  <h1>AVIAN<span class="sub">Bottom-up mass and CG budget, propulsion closed on real vendor
  hardware, thrust margin including failure cases, structural load cases, liquid slosh,
  and verified joint torques. Every gate closes. Geometry still not started.</span></h1>
  <div class="meta">
    <div>ITEMS COSTED <b>{len(NOM)}</b></div>
    <div>MTOW <b>{M_MIN:.2f} / {M_NOM:.2f} / {M_MAX:.2f} kg</b></div>
    <div>PROPULSION <b>VENDOR DATA</b></div>
    <div>GEOMETRY <b>NOT STARTED</b></div>
  </div>
</header>

<div class="big">
  <div><div class="v">1150<s>mm</s></div><div class="k">Diagonal &mdash; revised</div></div>
  <div><div class="v">{M_NOM:.2f}<s>kg</s></div><div class="k">MTOW nominal</div></div>
  <div><div class="v">{A.T_TOTAL_MAX/M_NOM:.2f}</div><div class="k">T/W nominal</div></div>
  <div><div class="v">{END_NOM:.1f}<s>min</s></div><div class="k">Hover estimate</div></div>
  <div><div class="v">44<s>%</s></div><div class="k">Rotor use in hover</div></div>
  <div><div class="v">+183<s>mm</s></div><div class="k">Wall clearance</div></div>
  <div><div class="v">NO</div><div class="k">Powered CG rail</div></div>
</div>

<div class="box bad">
  <h3>Correction to Phase 2 &mdash; my earlier conclusion was too pessimistic</h3>
  <p>Phase 2 concluded that <b>burst current was the binding constraint</b>. That was an
  artefact of my own model: I computed available thrust from momentum theory, and against
  real hardware that model is about <b>22 % pessimistic at full throttle</b>. T-Motor
  publishes an off-the-shelf <b>coaxial</b> kit for exactly this application, and its
  measured numbers are better than my estimate.</p>
  <p>With real data the aircraft needs <b>221 A</b> at 100 % throttle, and a 12S 24 Ah pack
  delivers <b>360 A</b>. Current is no longer binding &mdash; <b>the pack is now sized by
  energy</b>. The 12S recommendation still stands, but for a different and stronger reason,
  given below.</p>
</div>

<section>
  <div class="sh"><span class="sn">GATE 2</span><h2>Propulsion component closure</h2></div>
  <p class="lede">No more "6.87 kgf class motor". This is a catalogue part number, and it is
  a coaxial kit &mdash; the manufacturer already sells the exact architecture we specified.</p>
  <div class="frozen">
KIT          <b>{e(K['kit'])}</b>
MOTOR        <b>{e(K['motor'])}</b>            KV {K['kv']}        4 x coaxial pair = 8 motors
ESC          <b>{e(K['esc'])}</b>       {K['esc_a']:.0f} A continuous, 12S
PROPELLER    <b>{e(K['propeller'])}</b>          {K['prop_d_in']:.0f} in dia, {K['prop_pitch_in']:.1f} in pitch
VOLTAGE      <b>{e(K['cells'])}</b>
THRUST       <b>{K['pair_thrust_g']/1000:.2f} kgf per coaxial arm</b> at 100 % throttle    VENDOR
CURRENT      <b>{K['pair_current_a']:.2f} A per arm</b> ({K['pair_current_a']*A.V_NOM:.0f} W)      ESC efficiency {K['pair_eff_pct']:.2f} %
MASS         arm set {K['arm_set_g']:.0f} g incl. cable; motor {A.M_MOTOR*1000:.0f} g, ESC {A.M_ESC*1000:.0f} g, prop {A.M_PROP*1000:.0f} g
AIRCRAFT     <b>{A.T_TOTAL_MAX:.2f} kgf total</b>   {A.I_MAX_TOTAL:.0f} A   {A.P_MAX_TOTAL:.0f} W at full throttle
  </div>
  <p class="dim sm">Source: T-Motor product page for the X-U8&#8545; integrated coaxial
  propulsion kit, retrieved 26 Aug 2026. <b>VENDOR-PUBLISHED, NOT INDEPENDENTLY VERIFIED.</b>
  Treat as the best available pre-test data; a thrust-stand campaign is still required
  before any of it becomes VERIFIED.</p>
  <div class="box">
    <h3>The vendor settles the 6S question independently</h3>
    <p>T-Motor pairs this kit's <b>KV85 and KV100</b> variants with a <b>12S</b> ESC, and the
    <b>KV150 / KV190</b> variants with a <b>6S</b> ESC. The 28&Prime; propeller belongs to the
    12S variants &mdash; a 6S motor cannot turn a 28&Prime; disc at useful RPM. So the
    propeller size we need to hit our disc loading <i>forces</i> 12S, regardless of the
    current argument. That is an external, independent confirmation of the Phase 2
    recommendation.</p>
  </div>
  <p><b>Geometry consequence:</b> the real propeller is 28&Prime; (711.2 mm), not the 27&Prime;
  I assumed. At 1100 mm the tip gap would be 66 mm (9.4 % of D) &mdash; too tight. The
  diagonal moves to <b>1150 mm</b>, giving a {A.TIP_GAP:.0f} mm gap
  ({100*A.TIP_GAP/A.PROP_D:.1f} % of D). Forward rotor extent {A.FWD_ROTOR:.0f} mm; tool
  point at {A.ARM_BASE[0]+A.REACH:.0f} mm; <b>wall clearance +{A.ARM_BASE[0]+A.REACH-A.FWD_ROTOR:.0f} mm</b>.</p>
</section>

<section>
  <div class="sh"><span class="sn">GATE 3</span><h2>Thrust margin and failure cases</h2></div>
  <div class="tw"><table>
    <thead><tr><th>Pack</th><th class="n">Wh</th><th class="n">kg</th><th class="n">burst A</th>
      <th class="n">MTOW min</th><th class="n">nom</th><th class="n">max</th>
      <th class="n">T/W min</th><th class="n">T/W nom</th><th class="n">T/W max</th>
      <th class="n">hover min</th></tr></thead>
    <tbody>{pack_rows}</tbody>
  </table></div>
  <p class="dim sm">All four packs clear the 221 A demand and the T/W 2.0 floor. The
  <b>12S 24 Ah</b> row is selected: it is the lightest pack that holds T/W &ge; 2.2 (the
  preferred target) at <i>maximum</i> mission mass, and splits into two 3.6 kg cartridges a
  technician can lift one-handed.</p>
  <h3>Per-rotor margin</h3>
  <p>Hover demands <b>{M_NOM/8:.2f} kgf per rotor</b> against a rotor maximum of
  <b>6.80 kgf</b> &mdash; the propulsion system runs at <b>{100*(M_NOM/8)/6.80:.0f} %</b> of
  capability in hover. That headroom is what pays for CG trim, gust rejection and
  manipulator reaction, and it is the reason the powered CG rail turns out to be
  unnecessary.</p>
  <div class="two">
    <div class="box warn">
      <h3>One motor out (1 of 8)</h3>
      <p>The affected arm falls to a single rotor, <b>6.80 kgf</b>. For moment balance the
      diagonally opposite arm must be held to the same value. Available thrust becomes
      <b>40.94 kgf &mdash; 75 % of nominal</b>, giving <b>T/W 1.73</b> at nominal mass.</p>
      <p>Thrust is therefore sufficient. Yaw is the open question: the failed arm's coaxial
      torque balance is broken, and the trim must come from differential within the three
      surviving pairs. The authority exists in principle.
      <b>Controllability is NOT demonstrated</b> &mdash; it requires SITL with the actual
      mixer and a measured motor time constant.</p>
    </div>
    <div class="box warn">
      <h3>One arm out (2 of 8)</h3>
      <p>Three coaxial arms remain at 45&deg;, 135&deg; and 225&deg;. Thrust is
      <b>41.02 kgf, T/W 1.73</b>, but the thrust stations are no longer symmetric.</p>
      <p>The useful observation: each surviving <i>coaxial</i> arm can produce independent
      thrust <b>and</b> independent yaw torque by unbalancing its own pair. That gives six
      control inputs at three non-collinear stations, so the allocation matrix is
      <b>not rank-deficient</b> &mdash; unlike a conventional quadrotor losing a rotor.
      Recovery is plausible rather than impossible. <b>Still NOT demonstrated.</b> Do not
      quote this as a safety feature until it is simulated and then flight-tested.</p>
    </div>
  </div>
</section>

<section>
  <div class="sh"><span class="sn">GATE 4</span><h2>Centre of gravity study</h2></div>
  <div class="tw"><table>
    <thead><tr><th>Case</th><th>Configuration</th><th class="n">mass kg</th>
      <th class="n">CG X mm</th><th class="n">CG Y mm</th><th class="n">CG Z mm</th></tr></thead>
    <tbody>{cg_rows}</tbody>
  </table></div>
  <p class="dim sm">CG X excursion across all flight configurations
  <b>{EXC_X:.1f} mm</b>; CG Y excursion <b>{EXC_Y:.1f} mm</b>. Case A is a build-state
  reference, not a flight configuration.</p>

  <h3>Trim cost of a CG offset</h3>
  <div class="tw"><table>
    <thead><tr><th class="n">CG offset mm</th><th class="n">trim moment Nm</th>
      <th class="n">&Delta;T per arm N</th><th class="n">% of hover thrust</th>
      <th class="n">% of MAXIMUM thrust</th></tr></thead>
    <tbody>{trim_rows}</tbody>
  </table></div>

  <div class="box ok">
    <h3>Verdict: the powered CG rail is NOT REQUIRED &mdash; delete it</h3>
    <p><b>Two independent reasons, either one sufficient.</b></p>
    <p><b>1. A fore-aft rail cannot solve the governing case.</b> The worst lateral excursion
    is <b>{EXC_Y:.1f} mm</b> in side reach &mdash; the same order as the longitudinal
    {EXC_X:.1f} mm. A rail that only travels in X does nothing for it, and a two-axis powered
    rail carrying 7.2 kg of battery is not a defensible piece of engineering on a 23.7 kg
    aircraft.</p>
    <p><b>2. The propulsion system already has the margin.</b> Trimming the worst 80 mm
    offset costs <b>17 % of maximum rotor thrust</b>, and hover consumes only
    <b>{100*(M_NOM/8)/6.80:.0f} %</b>. The differential comes out of headroom that exists
    anyway. There is nothing to recover.</p>
    <p><b>3. It could not track the transient regardless.</b> The arm sweeps 900 mm in about
    a second. No battery rail follows that. The flight controller handles it, and the rail
    would only ever have addressed the slow, static component.</p>
    <p><b>Replace with:</b> a 3-position manually indexed battery mount, &plusmn;40 mm,
    detented, set on the ground to suit the mission profile. Saves the actuator, the
    lead-screw, the position sensor, the control loop and a jam failure mode &mdash; roughly
    <b>0.35 kg</b> and one whole failure branch, for no loss of capability.</p>
  </div>
  <p class="dim sm">This is a direct answer to your question, and it is the one place in
  this review where the brief asked for something the physics does not support. Everything
  else in the brief survives.</p>
</section>

<section>
  <div class="sh"><span class="sn">GATE 5</span><h2>Structural load cases</h2></div>
  <p class="lede">Arm root is the governing structural interface. CF tube 25 mm OD /
  21 mm ID, exposed length {L_arm*1000:.0f} mm, section modulus {Z:.0f} mm&sup3;.</p>
  <div class="tw"><table>
    <thead><tr><th>Load case</th><th class="n">n</th><th class="n">total N</th>
      <th class="n">per arm N</th><th class="n">root moment Nm</th>
      <th class="n">CF stress MPa</th><th class="n">safety factor</th></tr></thead>
    <tbody>{struct_rows}</tbody>
  </table></div>
  <p class="dim sm">Allowable taken as 600 MPa flexural for UD carbon &mdash;
  <b>ASSUMED</b>. Tip deflection at full thrust <b>{defl:.2f} mm</b>
  ({100*defl/(L_arm*1000):.2f} % of span) at E = 90 GPa, also ASSUMED. Deflection, not
  stress, is what actually sizes this tube: the 5 g hard-landing case still leaves SF 4.2,
  but arm stiffness drives the first bending mode and therefore the vibration environment
  the flight controller sees.</p>
  <h3>Other primary interfaces</h3>
  <div class="tw"><table>
    <thead><tr><th>Interface</th><th>Governing case</th><th class="n">Load</th>
      <th>Design note</th></tr></thead>
    <tbody>
      <tr><td class="b">Manipulator hub</td><td>arm at max reach, loaded, + 15 N tool contact</td>
        <td class="mono n">37.4 Nm</td>
        <td class="dim sm">CNC AL7075 closing into both spine rails; the single most
        safety-critical joint on the aircraft</td></tr>
      <tr><td class="b">Spine rail</td><td>hub reaction reacted at 4 arm roots</td>
        <td class="mono n">18.7 Nm each</td>
        <td class="dim sm">CF box 55 &times; 26 &times; 2.5; also carries battery and gear loads</td></tr>
      <tr><td class="b">Arm root clamp</td><td>all rotors 100 %</td>
        <td class="mono n">{Mr_max:.1f} Nm</td>
        <td class="dim sm">split clamp, 4 &times; M4; slip torque must exceed this with margin</td></tr>
      <tr><td class="b">Motor mount</td><td>single motor at max thrust</td>
        <td class="mono n">4.2 Nm</td>
        <td class="dim sm">plus gyroscopic precession during pitch/roll rate &mdash; not yet
        quantified</td></tr>
      <tr><td class="b">Landing gear</td><td>3 g four-point / 5 g two-point</td>
        <td class="mono n">180 / 599 N</td>
        <td class="dim sm">per foot; the two-point case sizes the leg and the damper</td></tr>
    </tbody>
  </table></div>
  <p class="dim sm"><b>CAD REVIEW ONLY &mdash; VALIDATION REQUIRED.</b> These are hand
  calculations on idealised sections. The hub, the arm root and the spine each need FEA
  before fabrication.</p>
</section>

<section>
  <div class="sh"><span class="sn">GATE 6</span><h2>Liquid service system</h2></div>
  <div class="two">
    <div>
      <h3>Specification</h3>
      <div class="tw"><table>
        <thead><tr><th>Item</th><th>Specification</th></tr></thead><tbody>
        <tr><td class="b">Reservoir</td><td>1.15 L geometric, 1.00 L working fill, blow-moulded HDPE, 200 &times; 105 &times; 110 mm internal</td></tr>
        <tr><td class="b">Baffles</td><td>3 transverse, 65 % open area, 50 mm bays</td></tr>
        <tr><td class="b">Pump</td><td>diaphragm, 0&ndash;6 bar, 1.2 L/min, 12 V, self-priming, run-dry tolerant</td></tr>
        <tr><td class="b">Filter</td><td>inline 40 &micro;m, serviceable bowl, upstream of pump</td></tr>
        <tr><td class="b">Check valve</td><td>0.2 bar cracking, downstream of pump &mdash; stops siphoning through the nozzle in any attitude</td></tr>
        <tr><td class="b">Relief / bypass</td><td>6.5 bar, returns to tank &mdash; protects the pump against a blocked nozzle</td></tr>
        <tr><td class="b">Pressure sensor</td><td>0&ndash;10 bar, feeds flow control and blockage detection</td></tr>
        <tr><td class="b">Quick connect</td><td>dry-break both sides; tank removes without spilling</td></tr>
        <tr><td class="b">Hose</td><td>PTFE-lined, clipped along the links with a service loop at every joint</td></tr>
        <tr><td class="b">Nozzle</td><td>replaceable orifice on the tool changer's fluid port</td></tr>
      </tbody></table></div>
    </div>
    <div>
      <h3>Slosh &mdash; the real risk is not CG</h3>
      <p>Fluid CG movement is negligible: at a 15&deg; tilt an unbaffled tank moves the
      aircraft CG by <b>0.57 mm</b>, and with 3 baffles by <b>0.14 mm</b>. That is nothing.</p>
      <p>The actual finding is dynamic. The first sloshing mode of the unbaffled tank sits at
      <b>1.86 Hz</b> &mdash; inside the attitude-loop bandwidth of a multirotor this size,
      where it can couple with the controller and show up as a low-frequency oscillation
      that gets worse as the tank half-empties.</p>
      <div class="tw"><table>
        <thead><tr><th>Baffling</th><th class="n">bay mm</th><th class="n">1st mode Hz</th>
          <th class="n">CG travel mm</th></tr></thead><tbody>
        <tr><td>none</td><td class="mono n">200</td><td class="mono n bad">1.86</td><td class="mono n">0.57</td></tr>
        <tr><td>2 baffles</td><td class="mono n">67</td><td class="mono n">3.42</td><td class="mono n">0.21</td></tr>
        <tr class="hl"><td>3 baffles</td><td class="mono n">50</td><td class="mono n ok">3.95</td><td class="mono n">0.14</td></tr>
        <tr><td>4 baffles</td><td class="mono n">40</td><td class="mono n ok">4.42</td><td class="mono n">0.11</td></tr>
      </tbody></table></div>
      <p class="dim sm"><b>3 baffles selected.</b> Moves the mode to 3.95 Hz and cuts the
      participating mass fourfold, for 48 g. Four baffles buy little more and complicate
      cleaning. <b>CAD REVIEW ONLY</b> &mdash; slosh frequency is a linear estimate; the
      coupling risk needs a controller-in-the-loop check.</p>
    </div>
  </div>
</section>

<section>
  <div class="sh"><span class="sn">GATE 7</span><h2>Manipulator torque verification</h2></div>
  <p class="lede">Worst orientation found by sweeping <b>123,552 poses</b> &mdash; J2, J3, J4
  and J5 through their full ranges &mdash; with gravity, a 15 N contact force at the tool
  point, and slew inertia. The governing load is the maximum of the three.</p>
  <div class="tw"><table>
    <thead><tr><th>Joint</th><th>Function</th><th class="n">gravity Nm</th>
      <th class="n">15 N contact Nm</th><th class="n">inertial Nm</th>
      <th class="n">GOVERNING</th><th class="n">&times;1.6</th><th class="n">SELECTED Nm</th>
      <th class="n">margin</th><th class="n">deg/s</th><th class="n">kg</th></tr></thead>
    <tbody>{tq_rows}</tbody>
  </table></div>
  <div class="box">
    <h3>Two results worth flagging</h3>
    <p><b>J1 carries no gravity torque at all.</b> Its axis is vertical, so gravity can never
    produce a moment about it. J1 is sized entirely by the <b>15 N lateral contact case
    (13.5 Nm)</b> and by slew inertia. That is easy to get wrong by inspection, and it is why
    the sweep was worth running.</p>
    <p><b>The wrist ordering you specified has a cost.</b> J4 roll / J5 pitch / J6 yaw puts
    the roll axis <i>before</i> the pitch and yaw. Spinning a tool about its own axis
    therefore requires coordinated motion of all three wrist joints rather than J6 alone. For
    a nozzle or a probe this is irrelevant. For a rotary tool &mdash; a driver, a brush, a
    cutter &mdash; it is a real limitation. Flagging it now because changing it later means
    redesigning three joint housings.</p>
  </div>
  <h3>Arm configurations and ground clearance</h3>
  <div class="tw"><table>
    <thead><tr><th>Configuration</th><th>J1 &hellip; J6 (deg)</th><th class="n">tool X</th>
      <th class="n">tool Y</th><th class="n">tool Z</th><th class="n">lowest point Z</th>
      <th class="n">above ground</th></tr></thead>
    <tbody>{cfg_rows}</tbody>
  </table></div>
  <p class="dim sm">Ground plane at Z = &minus;530 mm (skid underside). <b>DOWN_REACH is a
  ground-contact pose by design</b> &mdash; it reaches 560 mm below the skids, which is what
  lets the aircraft work on a surface it is standing beside, but it must be inhibited while
  airborne below that altitude. That interlock is a flight-software requirement, recorded
  here so it does not get lost.</p>
</section>

<section>
  <div class="sh"><span class="sn">GATE 1</span><h2>Complete mass budget</h2></div>
  <p class="lede">{len(NOM)} items, each with mass, CG coordinate, source and confidence.
  Nominal configuration: 12S 24 Ah, arm stowed, 1.0 L fluid, nozzle fitted.</p>
  <div class="two">
    <div>
      <div class="tw"><table>
        <thead><tr><th>Group</th><th class="n">kg</th><th class="n">% of MTOW</th></tr></thead>
        <tbody>{''.join(f'<tr><td>{e(GROUPNAME.get(g,g))}</td><td class="mono n">{gtot[g]:.3f}</td><td class="mono n">{100*gtot[g]/M_NOM:.1f}</td></tr>' for g in sorted(gtot))}
        <tr class="hl"><td class="b">TOTAL</td><td class="mono n b">{M_NOM:.3f}</td><td class="mono n b">100.0</td></tr>
        </tbody></table></div>
    </div>
    <div>
      <div class="tw"><table>
        <thead><tr><th>Confidence</th><th class="n">kg</th><th class="n">%</th></tr></thead>
        <tbody>{''.join(f'<tr><td><span class="cf {CONF.get(k,"est")}">{k}</span></td><td class="mono n">{v:.3f}</td><td class="mono n">{100*v/M_NOM:.1f}</td></tr>' for k,v in sorted(conf_tot.items(), key=lambda a:-a[1]))}
        </tbody></table></div>
      <p class="dim sm" style="margin-top:12px"><b>46 % of the aircraft mass is still
      ESTIMATED.</b> That is the honest state of a Phase 2 design. The propulsion system and
      the major electronics are vendor figures; the structure is calculated; brackets,
      panels, harness and the fluid hardware are engineering judgement and will move as
      real parts are selected. Expect the total to drift <b>&plusmn;1.5 kg</b> before
      fabrication.</p>
    </div>
  </div>
  <h3>Item-level budget</h3>
  <div class="tw tall"><table>
    <thead><tr><th>Item</th><th colspan="2">Source</th><th class="n">kg</th>
      <th class="n">X mm</th><th class="n">Y mm</th><th class="n">Z mm</th>
      <th>Confidence</th></tr></thead>
    <tbody>{item_html}</tbody>
  </table></div>
  <p class="dim sm">Mass closure: MIN <b>{M_MIN:.2f} kg</b> (stowed, dry, no tool) &middot;
  NOMINAL <b>{M_NOM:.2f} kg</b> &middot; MAX <b>{M_MAX:.2f} kg</b> (max reach, gripper +
  0.62 kg carried, 1.0 L). Mission payload 2.0 kg = 1.0 L fluid + 0.38 kg gripper +
  0.62 kg carried object.</p>
</section>

<section>
  <div class="sh"><span class="sn">GATE 8</span><h2>Final architecture decision</h2></div>
  <div class="frozen">
AIRCRAFT DIAGONAL     <b>1150 mm</b>   (not 1100 &mdash; set by the real 28&Prime; G28 propeller and a 14.3 % tip gap)
PROPULSION            <b>COAXIAL X8</b>   4 arms x 2 motors, T-Motor X-U8&#8545; kit, U8&#8545; KV100, ALPHA 60A 12S, G28x9.2
BATTERY               <b>12S 24 Ah, 1066 Wh, 7.20 kg</b>   2 x 12 Ah cartridges in parallel, hot-swappable
MANIPULATOR           <b>900 mm reach</b>, 6-DOF, 2.07 kg, J4 roll / J5 pitch / J6 yaw
ACTIVE CG RAIL        <b>NOT REQUIRED</b>   replaced by a 3-position indexed mount, +/-40 mm
LIQUID SYSTEM         <b>1.15 L baffled tank</b> (3 baffles), diaphragm pump 0-6 bar 1.2 L/min,
                      40 um filter, 0.2 bar check valve, 6.5 bar relief, pressure sensor, dry-break QC
MTOW                  <b>{M_MIN:.2f} / {M_NOM:.2f} / {M_MAX:.2f} kg</b>   min / nominal / max
THRUST TO WEIGHT      <b>{A.T_TOTAL_MAX/M_MIN:.2f} / {A.T_TOTAL_MAX/M_NOM:.2f} / {A.T_TOTAL_MAX/M_MAX:.2f}</b>   all above the 2.2 preferred target
HOVER POWER           <b>{P_NOM:.0f} W</b> nominal   {P_NOM/A.V_NOM:.0f} A   {100*(M_NOM/8)/6.80:.0f} % of rotor capability
HOVER ENDURANCE       <b>{END_NOM:.1f} min</b> nominal      ESTIMATE &mdash; REQUIRES VALIDATION
DISC LOADING          {DL_NOM:.0f} N/m2        disc area {A.A_DISC:.3f} m2
WALL CLEARANCE        <b>+{A.ARM_BASE[0]+A.REACH-A.FWD_ROTOR:.0f} mm</b>   rotor extent {A.FWD_ROTOR:.0f} mm, tool point {A.ARM_BASE[0]+A.REACH:.0f} mm
  </div>
  <div class="gate">
    <h3>All seven gates close. Three changes from the Phase 2 proposal.</h3>
    <ol>
      <li><b>Diagonal 1100 &rarr; 1150 mm.</b> Forced by the real propeller. Not a
      preference.</li>
      <li><b>The powered CG rail is deleted.</b> It cannot address the governing lateral
      case, and the propulsion margin makes it unnecessary. Net saving ~0.35 kg and one
      failure mode.</li>
      <li><b>Current is no longer the binding constraint.</b> My Phase 2 model was
      pessimistic; vendor data supersedes it. 12S remains correct, now because the 28&Prime;
      propeller requires it.</li>
    </ol>
    <p style="margin-top:12px"><b>Outstanding before fabrication &mdash; not before CAD:</b>
    FEA on the manipulator hub, arm root and spine; thrust-stand verification of the vendor
    numbers; SITL for both failure cases; controller-in-the-loop check of the 3.95 Hz slosh
    mode. None of these block Phase 3 geometry.</p>
    <p><b>Say the word and I start Phase 3</b> &mdash; parameter table, parametric model, and
    the Onshape Part Studio / Assembly / mate / configuration specification.</p>
  </div>
</section>

<footer>
  <div>AVIAN &middot; PHASE 2.7 &middot; REV A</div>
  <div>{len(NOM)} ITEMS COSTED</div>
  <div>VENDOR 27.9 % &middot; CALCULATED 26.0 % &middot; ESTIMATED 46.0 %</div>
  <div>NOTHING VERIFIED</div>
  <div>ONSHAPE &middot; GEOMETRY NOT STARTED</div>
</footer>
</div>
"""
open("avian_phase27.html", "w").write(HTML)
print("written", len(HTML), "bytes;", len(NOM), "items")
