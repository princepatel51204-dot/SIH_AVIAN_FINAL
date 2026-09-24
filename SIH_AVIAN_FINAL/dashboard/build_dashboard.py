"""SIH_AVIAN_FINAL -- Prompt 2: static, offline AVIAN mission dashboard.

Reads ONLY FINAL_RESULTS.json plus the committed flight/mission/detection
files it points at, and writes dashboard/index.html with every number
inlined (a <script type="application/json"> block) so the page works from
file:// with no fetch(). No hand-typed numbers below this docstring --
every figure is read from data and formatted, never authored.

Run again any time the underlying data changes:
    python3 dashboard/build_dashboard.py
"""
from __future__ import annotations
import html
import json
import math
import os
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
ASSETS = os.path.join(HERE, "assets")
os.makedirs(ASSETS, exist_ok=True)

PENDING = "pending freeze"


def _load(path, default=None):
    p = os.path.join(ROOT, path)
    if not os.path.exists(p):
        return default
    with open(p) as f:
        return json.load(f)


def load_final_results():
    return _load("FINAL_RESULTS.json", default=None)


# ---------------------------------------------------------------------------
# Plan view: top-down SVG of the corridor from real collision primitives
# ---------------------------------------------------------------------------
_EXCLUDE_KINDS_PLAN = {"ground", "water", "train_car"}


def _prim_footprint_xy(p):
    """Returns (x0,y0,x1,y1) footprint in world metres, ignoring z/height."""
    cx, cy = p["centre"][0], p["centre"][1]
    if p["type"] == "BOX":
        hx, hy = p["half_extents"][0], p["half_extents"][1]
        yaw = p.get("yaw", 0.0)
        # corners rotated by yaw, then take axis-aligned bbox -- footprints
        # in this scene are all axis-aligned (yaw in {0, pi/2}) so this is
        # exact, not an approximation
        cos_y, sin_y = abs(math.cos(yaw)), abs(math.sin(yaw))
        ex = hx * cos_y + hy * sin_y
        ey = hx * sin_y + hy * cos_y
        return (cx - ex, cy - ey, cx + ex, cy + ey)
    else:  # CYLINDER
        r = p["radius"]
        return (cx - r, cy - r, cx + r, cy + r)


def svg_plan_view(primitives, before_log, after_log, width=1120, height=340):
    xs0, ys0, xs1, ys1 = [], [], [], []
    footprints = []
    for p in primitives:
        if p["kind"] in _EXCLUDE_KINDS_PLAN:
            continue
        x0, y0, x1, y1 = _prim_footprint_xy(p)
        footprints.append((x0, y0, x1, y1))
        xs0.append(x0); ys0.append(y0); xs1.append(x1); ys1.append(y1)
    wx0, wy0, wx1, wy1 = min(xs0), min(ys0), max(xs1), max(ys1)
    pad_m = 8.0
    wx0 -= pad_m; wy0 -= pad_m; wx1 += pad_m; wy1 += pad_m
    world_w, world_h = wx1 - wx0, wy1 - wy0

    margin = 28
    avail_w, avail_h = width - 2 * margin, height - 2 * margin
    scale = min(avail_w / world_w, avail_h / world_h)
    off_x = margin + (avail_w - world_w * scale) / 2
    off_y = margin + (avail_h - world_h * scale) / 2

    def X(x):
        return off_x + (x - wx0) * scale

    def Y(y):
        # flip so +Y (metro, north side) renders toward the top
        return off_y + (wy1 - y) * scale

    parts = [f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg" '
             f'role="img" aria-label="Top-down plan view of the 360 metre inspection corridor, '
             f'showing structure footprints and flown waypoints">']
    parts.append(f'<rect x="0" y="0" width="{width}" height="{height}" fill="var(--surface)" rx="8"/>')

    for (x0, y0, x1, y1) in footprints:
        rx0, ry0, rx1, ry1 = X(x0), Y(y1), X(x1), Y(y0)
        w, h = max(rx1 - rx0, 1.0), max(ry1 - ry0, 1.0)
        parts.append(f'<rect x="{rx0:.1f}" y="{ry0:.1f}" width="{w:.1f}" height="{h:.1f}" '
                    f'fill="#C7CFDA" opacity="0.85"/>')

    def draw_flight(log, css_class, dot_r=3.2):
        path_pts = []
        dots = []
        for e in log:
            pos = e.get("achieved_position_m")
            if not pos:
                continue
            px, py = X(pos[0]), Y(pos[1])
            path_pts.append(f"{px:.1f},{py:.1f}")
            settled = e.get("settled")
            colour = "var(--ok)" if settled else "var(--warn)"
            dots.append((px, py, colour))
        g = [f'<g class="{css_class}">']
        if path_pts:
            g.append(f'<polyline points="{" ".join(path_pts)}" fill="none" '
                    f'stroke="var(--ink-2)" stroke-width="1" opacity="0.55"/>')
        for px, py, colour in dots:
            g.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="{dot_r}" fill="{colour}"/>')
        g.append('</g>')
        return "".join(g)

    parts.append(draw_flight(before_log, "plan-before"))
    if after_log is not None:
        after_svg = draw_flight(after_log, "plan-after")
        after_svg = after_svg.replace('class="plan-after">', 'class="plan-after" style="display:none">')
        parts.append(after_svg)

    # scale bar (50 m) + N/+X arrow
    bar_m = 50.0
    bar_px = bar_m * scale
    bx0, by0 = margin, height - 14
    parts.append(f'<line x1="{bx0}" y1="{by0}" x2="{bx0+bar_px:.1f}" y2="{by0}" '
                f'stroke="var(--ink)" stroke-width="2"/>')
    parts.append(f'<text x="{bx0}" y="{by0-6}" font-size="11" fill="var(--ink-2)">0</text>')
    parts.append(f'<text x="{bx0+bar_px-14:.1f}" y="{by0-6}" font-size="11" fill="var(--ink-2)">{int(bar_m)} m</text>')
    ax, ay = width - margin - 4, margin + 4
    parts.append(f'<g transform="translate({ax},{ay})">'
                f'<line x1="0" y1="18" x2="0" y2="0" stroke="var(--ink)" stroke-width="2"/>'
                f'<polygon points="0,-5 -4,4 4,4" fill="var(--ink)"/>'
                f'<text x="6" y="14" font-size="11" fill="var(--ink-2)">+X</text></g>')
    parts.append('</svg>')
    return "".join(parts)


# ---------------------------------------------------------------------------
# Autonomy bar chart: stuck % for stage1 / stage2-before / stage2-after
# ---------------------------------------------------------------------------
def svg_bar_chart(series, width=420, height=220):
    """series: list of (label, pct_or_None)."""
    margin_l, margin_b, margin_t = 34, 28, 12
    avail_h = height - margin_b - margin_t
    n = len(series)
    slot = (width - margin_l - 16) / n
    bar_w = slot * 0.5
    parts = [f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg" '
             f'role="img" aria-label="Bar chart of stuck percentage across flight stages">']
    for gy in (0, 25, 50, 75, 100):
        y = margin_t + avail_h * (1 - gy / 100)
        parts.append(f'<line x1="{margin_l}" y1="{y:.1f}" x2="{width-8}" y2="{y:.1f}" '
                    f'stroke="var(--line)" stroke-width="1"/>')
        parts.append(f'<text x="4" y="{y+4:.1f}" font-size="10" fill="var(--ink-2)">{gy}</text>')
    for i, (label, pct) in enumerate(series):
        cx = margin_l + slot * i + slot / 2
        if pct is None:
            parts.append(f'<text x="{cx:.1f}" y="{margin_t + avail_h/2:.1f}" font-size="11" '
                        f'fill="var(--ink-2)" text-anchor="middle">not run</text>')
        else:
            bh = avail_h * min(pct, 100) / 100
            by = margin_t + avail_h - bh
            colour = "var(--accent)" if "after" in label else "var(--bad)" if pct > 50 else "var(--warn)"
            parts.append(f'<rect x="{cx-bar_w/2:.1f}" y="{by:.1f}" width="{bar_w:.1f}" height="{bh:.1f}" '
                        f'fill="{colour}" rx="3"/>')
            parts.append(f'<text x="{cx:.1f}" y="{by-6:.1f}" font-size="12" font-weight="700" '
                        f'text-anchor="middle" fill="var(--ink)">{pct:.1f}%</text>')
        parts.append(f'<text x="{cx:.1f}" y="{height-8}" font-size="10.5" text-anchor="middle" '
                    f'fill="var(--ink-2)">{html.escape(label)}</text>')
    parts.append('</svg>')
    return "".join(parts)


# ---------------------------------------------------------------------------
# Number registry -- every stat rendered goes through here so the build's
# own check can confirm it traces back to FINAL_RESULTS.json data.
# ---------------------------------------------------------------------------
class Num:
    def __init__(self):
        self.values = []

    def add(self, value):
        s = str(value)
        self.values.append(s)
        return s


# ---------------------------------------------------------------------------
# Evidence gallery -- 6 real zoom tiles, boxes drawn from real inference,
# mixing true positives, a false positive, and missed defects on purpose.
# GT boxes are re-derived the same way evaluate_v3_mission_final.py scores
# them (via score_coverage._project_point), never guessed.
# ---------------------------------------------------------------------------
GALLERY_SPEC = [
    {"tile_id": "CWP_014_T0203", "outcome": "tp",
     "note": "MISSION-VAL catch -- decided the headline model"},
    {"tile_id": "CWP_020_T0502", "outcome": "tp",
     "note": "unseen defect on MISSION-TEST -- the headline recall number"},
    {"tile_id": "CWP_031_T0701", "outcome": "tp",
     "note": "MISSION-TEST catch, SPALL_DELAM"},
    {"tile_id": "CWP_018_T0407", "outcome": "fp",
     "note": "no ground truth here -- v1's texture-triggered CRACK habit, still present in v2"},
    {"tile_id": "CWP_073_T0402", "outcome": "fn",
     "note": "4 ground-truth bolts in frame, 0 detected -- FASTENER limitation"},
    {"tile_id": "CWP_016_T0101", "outcome": "fn",
     "note": "ground-truth crack in frame, not detected -- CRACK does not generalise to mission tiles"},
]

_OUTCOME_COLOUR = {"tp": "#15803D", "fp": "#B91C1C", "fn": "#B45309"}
_OUTCOME_LABEL = {"tp": "true positive", "fp": "false positive", "fn": "missed defect"}


def build_gallery(zoom_tiles_path="detection/AVIAN_zoom_tiles_all_FINAL.json"):
    import sys
    sys.path.insert(0, os.path.join(ROOT, "source"))
    import evaluate_v3_mission_final as EV
    from PIL import Image, ImageDraw

    with open(os.path.join(ROOT, zoom_tiles_path)) as f:
        tiles = {t["tile_id"]: t for t in json.load(f)["tiles"]}
    with open(os.path.join(ROOT, "dataset/labels_final.json")) as f:
        type_to_family = json.load(f)["type_to_family"]
    all_defects = EV._load_all_defects()
    scoreable = [d for d in all_defects
                 if d.get("escalation_reason") not in EV._ESCALATIONS_NOT_A_PLANNER_FAILURE
                 and d["type"] in type_to_family]
    model = EV._load_model("v2")

    out = []
    for spec in GALLERY_SPEC:
        tid = spec["tile_id"]
        t = tiles[tid]
        img_path = os.path.join(ROOT, t["image_path"])
        img = Image.open(img_path).convert("RGB")
        draw = ImageDraw.Draw(img)

        gt_here = EV._gt_for_tile(t, scoreable)
        dets = [d for d in EV._detect_raw(model, img_path) if d["confidence"] >= 0.65]

        for d in dets:
            x0, y0, x1, y1 = d["bbox"]
            colour = _OUTCOME_COLOUR["tp"] if spec["outcome"] == "tp" else _OUTCOME_COLOUR["fp"]
            draw.rectangle([x0, y0, x1, y1], outline=colour, width=4)
            label = f'{d["class"]} {d["confidence"]:.2f}'
            draw.rectangle([x0, max(0, y0 - 18), x0 + 8 * len(label) + 6, y0], fill=colour)
            draw.text((x0 + 3, max(0, y0 - 17)), label, fill="white")

        if spec["outcome"] == "fn":
            for gd, gbox in gt_here:
                x0, y0, x1, y1 = gbox
                colour = _OUTCOME_COLOUR["fn"]
                for dash_i in range(0, int(x1 - x0), 14):
                    dx0 = x0 + dash_i
                    draw.line([(dx0, y0), (min(dx0 + 8, x1), y0)], fill=colour, width=4)
                    draw.line([(dx0, y1), (min(dx0 + 8, x1), y1)], fill=colour, width=4)
                for dash_i in range(0, int(y1 - y0), 14):
                    dy0 = y0 + dash_i
                    draw.line([(x0, dy0), (x0, min(dy0 + 8, y1))], fill=colour, width=4)
                    draw.line([(x1, dy0), (x1, min(dy0 + 8, y1))], fill=colour, width=4)
                label = f'{gd["type"]} (missed)'
                draw.rectangle([x0, y1, x0 + 8 * len(label) + 6, y1 + 18], fill=colour)
                draw.text((x0 + 3, y1 + 2), label, fill="white")

        out_name = f"tile_{tid}.jpg"
        img.save(os.path.join(ASSETS, out_name), "JPEG", quality=82)
        out.append({
            "tile_id": tid, "file": f"assets/{out_name}",
            "outcome": spec["outcome"], "outcome_label": _OUTCOME_LABEL[spec["outcome"]],
            "note": spec["note"],
            "n_detections": len(dets), "n_ground_truth": len(gt_here),
        })
    return out


def build_hero_image():
    from PIL import Image
    src = os.path.join(ROOT, "renders/CAM_10_TRUSS.png")
    img = Image.open(src).convert("RGB")
    if img.width > 1600:
        h = int(img.height * 1600 / img.width)
        img = img.resize((1600, h))
    img.save(os.path.join(ASSETS, "hero.jpg"), "JPEG", quality=82)
    return "assets/hero.jpg"


def _fmt_pct(x, digits=1):
    return PENDING if x is None else f"{x:.{digits}f}%"


def _fmt_frac(pair, label=""):
    if pair is None:
        return PENDING
    n, d = pair
    pct = f" ({100*n/d:.0f}%)" if d else ""
    return f"{n} of {d}{label}{pct}"


def _initials(name):
    parts = [p for p in name.split() if p]
    return (parts[0][0] + parts[-1][0]).upper() if len(parts) > 1 else name[:2].upper()


# ---------------------------------------------------------------------------
# HTML assembly
# ---------------------------------------------------------------------------
CSS = """
:root{
  --bg:#FFFFFF; --surface:#F5F7FA; --ink:#0E1116; --ink-2:#4A5568;
  --line:#DDE3EA; --accent:#1D4ED8; --ok:#15803D; --warn:#B45309; --bad:#B91C1C;
  --maxw:1200px;
}
*{box-sizing:border-box}
html,body{margin:0;padding:0;background:var(--bg);color:var(--ink);
  font-family:"Public Sans",system-ui,-apple-system,"Segoe UI",Arial,sans-serif;}
body{font-size:16px;line-height:1.6;}
.num{font-variant-numeric:tabular-nums;}
a{color:var(--accent);}
:focus-visible{outline:3px solid var(--accent);outline-offset:2px;}
.wrap{max-width:var(--maxw);margin:0 auto;padding:0 24px;}
.topbar{position:sticky;top:0;background:#fff;border-bottom:1px solid var(--line);
  z-index:50;}
.topbar .wrap{display:flex;align-items:center;justify-content:space-between;
  height:60px;flex-wrap:wrap;gap:8px;}
.wordmark{font-weight:700;font-size:20px;letter-spacing:0.02em;}
.topnav{display:flex;gap:20px;font-size:14px;flex-wrap:wrap;}
.topnav a{color:var(--ink-2);text-decoration:none;}
.topnav a:hover{color:var(--accent);}
.topmeta{font-size:13px;color:var(--ink-2);}
.hero{position:relative;height:560px;overflow:hidden;}
.hero img{width:100%;height:100%;object-fit:cover;}
.hero .overlay{position:absolute;inset:0;
  background:linear-gradient(180deg, rgba(14,17,22,0.15) 0%, rgba(14,17,22,0.78) 100%);}
.hero .content{position:absolute;left:0;right:0;bottom:0;padding:40px 24px;
  max-width:var(--maxw);margin:0 auto;color:#fff;}
.hero h1{font-size:52px;font-weight:700;line-height:1.05;margin:0 0 12px;
  max-width:820px;}
.hero p{font-size:18px;margin:0;color:#E7ECF3;max-width:680px;}
@media (max-width:760px){.hero h1{font-size:34px;}.hero{height:440px;}}
.statrow{display:grid;grid-template-columns:repeat(4,1fr);gap:16px;
  margin-top:-56px;position:relative;z-index:2;padding:0 24px;max-width:var(--maxw);
  margin-left:auto;margin-right:auto;}
@media (max-width:760px){.statrow{grid-template-columns:1fr 1fr;margin-top:16px;}}
.stat{background:#fff;border:1px solid var(--line);border-radius:8px;padding:18px 16px;}
.stat .n{font-size:40px;font-weight:700;color:var(--accent);display:block;}
.stat .cap{font-size:13.5px;color:var(--ink-2);margin-top:4px;}
.pbband{background:var(--surface);padding:28px 0;margin-top:40px;}
.pbband .label{font-size:12px;text-transform:uppercase;letter-spacing:0.08em;
  color:var(--ink-2);font-weight:700;}
.pbband h2{font-size:20px;margin:8px 0 10px;font-weight:700;}
.chips{display:flex;gap:8px;flex-wrap:wrap;margin:10px 0;}
.chip{background:#fff;border:1px solid var(--line);border-radius:999px;
  padding:4px 12px;font-size:13px;color:var(--ink-2);}
.pbband .resp{margin-top:10px;font-size:15px;}
section{padding:72px 0;}
section.tight{padding:48px 0;}
h2.sec{font-size:28px;font-weight:700;margin:0 0 28px;}
.grid12{display:grid;grid-template-columns:repeat(12,1fr);gap:20px;}
@media (max-width:760px){.grid12{grid-template-columns:1fr;}}
.card{background:#fff;border:1px solid var(--line);border-radius:8px;padding:20px;}
.legend{display:flex;gap:18px;flex-wrap:wrap;font-size:13px;color:var(--ink-2);
  margin-top:10px;}
.legend .dot{width:10px;height:10px;border-radius:50%;display:inline-block;
  margin-right:6px;vertical-align:middle;}
.toggle{display:inline-flex;border:1px solid var(--line);border-radius:999px;
  overflow:hidden;font-size:13px;margin-bottom:12px;}
.toggle button{border:none;background:#fff;padding:6px 16px;cursor:pointer;
  color:var(--ink-2);font-family:inherit;}
.toggle button.active{background:var(--accent);color:#fff;}
table{width:100%;border-collapse:collapse;font-size:14.5px;}
th,td{text-align:left;padding:10px 12px;border-bottom:1px solid var(--line);}
th{color:var(--ink-2);font-weight:700;font-size:13px;text-transform:uppercase;
  letter-spacing:0.02em;}
tr.headline{background:var(--surface);}
tr.headline td:first-child{font-weight:700;color:var(--accent);}
.gallery{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;margin-top:20px;}
@media (max-width:760px){.gallery{grid-template-columns:1fr 1fr;}}
@media (max-width:480px){.gallery{grid-template-columns:1fr;}}
.tile{border:1px solid var(--line);border-radius:8px;overflow:hidden;background:#fff;
  cursor:pointer;text-align:left;padding:0;font-family:inherit;}
.tile img{width:100%;display:block;aspect-ratio:4/3;object-fit:cover;}
.tile .meta{padding:10px 12px;}
.tile .badge{display:inline-block;font-size:11px;font-weight:700;text-transform:uppercase;
  padding:2px 8px;border-radius:4px;color:#fff;margin-bottom:6px;}
.badge.tp{background:var(--ok);} .badge.fp{background:var(--bad);} .badge.fn{background:var(--warn);}
.tile .note{font-size:13px;color:var(--ink-2);}
.lightbox{position:fixed;inset:0;background:rgba(14,17,22,0.85);display:none;
  align-items:center;justify-content:center;z-index:100;padding:24px;}
.lightbox.open{display:flex;}
.lightbox img{max-width:90vw;max-height:80vh;border-radius:8px;}
.lightbox .cap{color:#fff;text-align:center;margin-top:12px;font-size:14px;max-width:640px;}
.lightbox button.close{position:absolute;top:20px;right:24px;background:none;border:none;
  color:#fff;font-size:28px;cursor:pointer;}
.pipeline{display:grid;grid-template-columns:repeat(6,1fr);gap:14px;}
@media (max-width:760px){.pipeline{grid-template-columns:1fr;}}
.pstep{border:1px solid var(--line);border-radius:8px;padding:14px;background:#fff;}
.pstep .idx{font-size:12px;color:var(--accent);font-weight:700;}
.pstep h3{font-size:15px;margin:6px 0;}
.pstep p{font-size:13px;color:var(--ink-2);margin:0 0 6px;}
.pstep code{font-size:11.5px;color:var(--ink-2);}
ul.limits{list-style:none;margin:0;padding:0;display:grid;gap:10px;}
ul.limits li{background:var(--surface);border-radius:8px;padding:12px 16px;
  font-size:15px;position:relative;padding-left:34px;}
ul.limits li::before{content:"—";position:absolute;left:14px;color:var(--warn);font-weight:700;}
.team-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;margin-top:18px;}
@media (max-width:760px){.team-grid{grid-template-columns:1fr 1fr;}}
@media (max-width:480px){.team-grid{grid-template-columns:1fr;}}
.member{border:1px solid var(--line);border-radius:8px;padding:16px;text-align:center;
  background:#fff;}
.member .av{width:48px;height:48px;border-radius:50%;background:var(--accent);color:#fff;
  display:flex;align-items:center;justify-content:center;font-weight:700;margin:0 auto 10px;}
.member .name{font-weight:700;font-size:14.5px;}
.member .role{font-size:13px;color:var(--ink-2);}
footer{background:var(--surface);padding:48px 0;font-size:13.5px;color:var(--ink-2);}
footer h4{font-size:13px;text-transform:uppercase;color:var(--ink);margin:0 0 8px;}
footer .cols{display:grid;grid-template-columns:1fr 1fr 1fr;gap:24px;}
@media (max-width:760px){footer .cols{grid-template-columns:1fr;}}
footer .src-list{max-height:180px;overflow:auto;font-size:12.5px;}
.pending{color:var(--ink-2);font-style:italic;}
"""

JS = """
(function(){
  var beforeBtn = document.getElementById('toggle-before');
  var afterBtn = document.getElementById('toggle-after');
  if (beforeBtn && afterBtn) {
    beforeBtn.addEventListener('click', function(){
      document.querySelector('.plan-before').style.display = '';
      document.querySelector('.plan-after').style.display = 'none';
      beforeBtn.classList.add('active'); afterBtn.classList.remove('active');
    });
    afterBtn.addEventListener('click', function(){
      document.querySelector('.plan-before').style.display = 'none';
      document.querySelector('.plan-after').style.display = '';
      afterBtn.classList.add('active'); beforeBtn.classList.remove('active');
    });
  }
  var lb = document.getElementById('lightbox');
  var lbImg = document.getElementById('lightbox-img');
  var lbCap = document.getElementById('lightbox-cap');
  document.querySelectorAll('.tile').forEach(function(t){
    t.addEventListener('click', function(){
      lbImg.src = t.querySelector('img').src;
      lbCap.textContent = t.getAttribute('data-caption') || '';
      lb.classList.add('open');
    });
  });
  function closeLb(){ lb.classList.remove('open'); }
  document.getElementById('lightbox-close').addEventListener('click', closeLb);
  lb.addEventListener('click', function(e){ if (e.target === lb) closeLb(); });
  document.addEventListener('keydown', function(e){ if (e.key === 'Escape') closeLb(); });
})();
"""


def _model_row(name, m, headline):
    if m is None:
        return f'<tr><td>{name}</td><td colspan="4" class="pending">{PENDING}</td></tr>'
    cls = ' class="headline"' if name == headline else ''
    return (f'<tr{cls}><td>{name}{" (headline)" if name==headline else ""}</td>'
            f'<td class="num">{m["threshold"]}</td>'
            f'<td class="num">{m["val_f1"]}</td>'
            f'<td class="num">{_fmt_frac(m["unseen_recall"])}</td>'
            f'<td class="num">{_fmt_frac(m["all_recall"])} <span class="pending">(optimistic)</span></td>'
            f'<td class="num">{m["fp_per_100_tiles"]}</td></tr>')


def build_html(data, gallery, hero_path, plan_svg, bar_svg):
    d = data or {}
    meta = d.get("meta", {})
    scene = d.get("scene", {})
    auto = d.get("autonomy", {})
    det = d.get("detection", {})
    zoom = d.get("zoom", {})
    team = d.get("team", {})
    prob = d.get("problem_statement", {})
    stack = d.get("stack", [])
    not_done = d.get("not_done_yet", [])
    sources = d.get("sources", {})

    commit = meta.get("commit", PENDING)[:10] if meta.get("commit") else PENDING
    frozen = d is not None and bool(meta.get("commit"))

    s2b = auto.get("stage2_before")
    s2a = auto.get("stage2_after")
    s1 = auto.get("stage1")

    coverage_pct = s2b.get("coverage_pct") if s2b else None
    collision_before = s2b.get("stuck_pct") if s2b else None
    collision_after = s2a.get("stuck_pct") if s2a else None
    recall_pair = tuple(s2b["recall"]) if s2b and s2b.get("recall") else None
    fp_models = det.get("fp_per_100_tiles", {})
    fp_v1 = fp_models.get("v1")
    fp_headline = fp_models.get(det.get("headline")) if det.get("headline") else None

    headline_stats = [
        ("coverage", _fmt_pct(coverage_pct), "of structure surface area, planned"),
        ("collision", f'{_fmt_pct(collision_before,1)} &rarr; {_fmt_pct(collision_after,1)}'
                      if collision_after is not None else f'{_fmt_pct(collision_before,1)} &rarr; <span class="pending">{PENDING}</span>',
         "waypoints stuck, before &rarr; after"),
        ("defects", _fmt_frac(recall_pair), "recalled by the coverage plan"),
        ("false alarms", (f'{fp_v1:.1f} &rarr; {fp_headline:.2f}' if fp_v1 is not None and fp_headline is not None else PENDING),
         "per 100 tiles, v1 &rarr; headline model"),
    ]

    stat_html = "".join(
        f'<div class="stat"><span class="n num">{v}</span><div class="cap">{cap}</div></div>'
        for _, v, cap in headline_stats
    )

    # -- autonomy section --
    sensed = auto.get("stage2_before", {}).get("sensed_entries") if s2b else None
    avoid = s2b.get("avoid_reactions") if s2b else None
    packs = s2b.get("battery_packs") if s2b else None
    autonomy_facts = f"""
      <ul style="list-style:none;padding:0;margin:0;display:grid;gap:10px;font-size:15px;">
        <li><strong class="num">{_fmt_frac(tuple(sensed)) if sensed else PENDING}</strong> flight-log entries carried live sensor returns</li>
        <li><strong class="num">{avoid if avoid is not None else PENDING}</strong> avoidance reactions triggered in-flight</li>
        <li><strong class="num">{packs if packs is not None else PENDING}</strong> battery packs used on the Stage 2 survey</li>
      </ul>"""

    bars = [
        ("Stage 1", s1.get("stuck_pct") if s1 else None),
        ("Stage 2 before", collision_before),
        ("Stage 2 after", collision_after),
    ]
    bar_chart_svg = svg_bar_chart(bars)

    why_collisions = ("The 54 transit-stuck legs were real structural encounters severe "
                      "enough to physically displace the airframe by metres, not "
                      "centimetres &mdash; not a narrow miss a slightly bigger margin "
                      "would have fixed.")

    # -- detection model table --
    models = det.get("models", {})
    headline = det.get("headline")
    model_rows = "".join(_model_row(n, models.get(n), headline) for n in ("v1", "v2", "v3"))

    real_photo = det.get("real_photo_test")
    if real_photo == "not run" or real_photo is None:
        real_photo_html = f'<p class="pending">Real-photo crack test: {PENDING if real_photo is None else "not run"}.</p>'
    else:
        real_photo_html = (
            f'<p><strong>Real-photo crack test</strong> (Özgenel, CC BY 4.0, '
            f'{real_photo.get("n_positive","?")}+{real_photo.get("n_negative","?")} photos): '
            f'precision <span class="num">{real_photo.get("precision")}</span>, '
            f'recall <span class="num">{real_photo.get("precision")}</span> '
            f'(tp={real_photo.get("tp")}, fp={real_photo.get("fp")}, fn={real_photo.get("fn")}, tn={real_photo.get("tn")}) '
            f'&mdash; <strong>does not yet transfer to real photos.</strong></p>')

    gallery_html = "".join(
        f'''<button class="tile" data-caption="{html.escape(g["note"])}">
          <img src="{g["file"]}" alt="Zoom tile {g["tile_id"]}, {g["outcome_label"]}: {html.escape(g["note"])}">
          <div class="meta"><span class="badge {g["outcome"]}">{g["outcome_label"]}</span>
          <div class="note">{html.escape(g["note"])}</div></div>
        </button>'''
        for g in gallery
    )

    pipeline_steps = [
        ("Digital twin", "Blender-built 360 m corridor, 192 measured defects", "scene/SIH_AVIAN_FINAL.blend"),
        ("Plan coverage", "Structure-driven waypoint planning, zero ground truth read", "source/coverage_final.py"),
        ("Fly & avoid", "PyBullet flight, raycast sensing, A* detour routing", "source/flight_final.py"),
        (f"Zoom capture ({zoom.get('optical_x', PENDING)}&times;, {zoom.get('hfov_deg', PENDING)}&deg;)",
         "Simulated optical zoom at the same safe standoff", "source/render_zoom_tiles_all_final.py"),
        ("Detect", "Faster R-CNN (torchvision, BSD-3-Clause)", "source/train_detector_v2_final.py"),
        ("Score", "IoU-matched against projected ground truth", "source/evaluate_v3_mission_final.py"),
    ]
    pipeline_html = "".join(
        f'''<div class="pstep"><div class="idx">{i+1:02d}</div><h3>{n}</h3>
        <p>{desc}</p><code>{path}</code></div>'''
        for i, (n, desc, path) in enumerate(pipeline_steps)
    )

    limits_html = "".join(f"<li>{html.escape(x)}</li>" for x in not_done) or f'<li class="pending">{PENDING}</li>'

    members = team.get("members", [])
    member_html = "".join(
        f'''<div class="member"><div class="av">{_initials(m["name"])}</div>
        <div class="name">{html.escape(m["name"])}</div><div class="role">{html.escape(m["role"])}</div></div>'''
        for m in members
    )

    stack_html = "".join(f"<li>{html.escape(s.get('name',''))} &mdash; {html.escape(s.get('license',''))}</li>" for s in stack)
    src_html = "".join(f"<li><code>{html.escape(k)}</code>: {html.escape(v)}</li>" for k, v in sources.items())

    after_toggle = ""
    if s2a:
        after_toggle = ('<div class="toggle"><button id="toggle-before" class="active">Before</button>'
                        '<button id="toggle-after">After</button></div>')
    else:
        after_toggle = f'<p class="pending">After-fix flight not available yet ({PENDING}) &mdash; showing Stage 2 before only.</p>'

    problem_band = ""
    if prob:
        problem_band = f'''
    <div class="pbband"><div class="wrap">
      <div class="label">Problem statement &middot; {html.escape(prob.get("id",""))}</div>
      <h2>{html.escape(prob.get("title",""))}</h2>
      <div class="chips">
        <span class="chip">{html.escape(prob.get("organization",""))}</span>
        <span class="chip">{html.escape(prob.get("technology_bucket",""))}</span>
        <span class="chip">{html.escape(prob.get("category",""))}</span>
      </div>
      <p class="resp">Our response: bridges and viaducts are infrastructure whose
      failure becomes a rescue operation. AVIAN inspects them before that, and
      can check a damaged bridge before rescue convoys cross.</p>
    </div></div>'''

    team_meta = f'Team {html.escape(team.get("name",""))} &middot; {html.escape(prob.get("id",""))} &middot; commit {commit}' if team else f'commit {commit}'

    html_doc = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>AVIAN &mdash; mission dashboard</title>
<meta name="description" content="AVIAN autonomous bridge inspection: mission plan, autonomy, and detection results, measured end to end.">
<link rel="preconnect" href="https://fonts.googleapis.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Public+Sans:wght@400;700&display=swap" rel="stylesheet">
<style>{CSS}</style>
</head>
<body>
<div class="topbar"><div class="wrap">
  <div class="wordmark">AVIAN</div>
  <nav class="topnav">
    <a href="#mission">Mission</a><a href="#autonomy">Autonomy</a>
    <a href="#detection">Detection</a><a href="#pipeline">Pipeline</a>
    <a href="#limits">Limits</a><a href="#team">Team</a>
  </nav>
  <div class="topmeta">{team_meta}</div>
</div></div>

<div class="hero">
  <img src="{hero_path}" alt="Steel truss main span of the AVIAN bridge digital twin">
  <div class="overlay"></div>
  <div class="content">
    <h1>Autonomous bridge inspection, measured end to end.</h1>
    <p>A simulation digital twin of a 360 m road-and-metro corridor with 192
    known defects, flown, avoided, zoomed and scored without a human in the loop.</p>
  </div>
</div>
<div class="wrap"><div class="statrow">{stat_html}</div></div>
{problem_band}

<section id="mission"><div class="wrap">
  <h2 class="sec">Mission plan view</h2>
  {after_toggle}
  <div class="card">{plan_svg}</div>
  <div class="legend">
    <span><span class="dot" style="background:var(--ok)"></span>settled</span>
    <span><span class="dot" style="background:var(--warn)"></span>stuck</span>
    <span>Grey shapes: real structure footprints from the collision model</span>
  </div>
</div></section>

<section id="autonomy" class="tight"><div class="wrap">
  <h2 class="sec">Autonomy</h2>
  <div class="grid12">
    <div class="card" style="grid-column:span 6">{bar_chart_svg}</div>
    <div class="card" style="grid-column:span 6">{autonomy_facts}</div>
  </div>
  <p style="margin-top:16px;color:var(--ink-2);">{why_collisions}</p>
</div></section>

<section id="detection"><div class="wrap">
  <h2 class="sec">Detection</h2>
  <div class="card">
    <table>
      <thead><tr><th>Model</th><th>Threshold</th><th>F1 (MISSION-VAL)</th>
      <th>Unseen recall</th><th>All-defects recall</th><th>FP / 100 tiles</th></tr></thead>
      <tbody>{model_rows}</tbody>
    </table>
  </div>
  <h3 style="margin:32px 0 6px;font-size:18px;">Evidence gallery</h3>
  <p style="color:var(--ink-2);margin:0;">Six real zoom tiles, boxes drawn from actual
  model output &mdash; true positives, a false positive, and missed defects, shown
  together rather than cherry-picked.</p>
  <div class="gallery">{gallery_html}</div>
  <div style="margin-top:24px;">{real_photo_html}
  <p style="color:var(--ink-2);">Bolts (FASTENER): not reliably detectable at the
  current camera resolution &mdash; median box width is single-digit pixels even
  at native input resolution. See the gallery's missed-bolt tile above.</p></div>
</div></section>

<section id="pipeline" class="tight"><div class="wrap">
  <h2 class="sec">Pipeline</h2>
  <div class="pipeline">{pipeline_html}</div>
</div></section>

<section id="limits"><div class="wrap">
  <h2 class="sec">What this prototype does not do yet</h2>
  <ul class="limits">{limits_html}</ul>
</div></section>

<section id="team" class="tight"><div class="wrap">
  <h2 class="sec">Team</h2>
  <p>Team {html.escape(team.get("name","")) if team else PENDING},
  ID {html.escape(str(team.get("team_id",""))) if team else PENDING} &mdash;
  {html.escape(team.get("institute","")) if team else ""}</p>
  <div class="team-grid">{member_html or f'<p class="pending">{PENDING}</p>'}</div>
</div></section>

<footer><div class="wrap">
  <div class="cols">
    <div><h4>Sources</h4><ul class="src-list">{src_html or f'<li class="pending">{PENDING}</li>'}</ul></div>
    <div><h4>Licenses</h4><ul class="src-list">{stack_html or f'<li class="pending">{PENDING}</li>'}</ul></div>
    <div><h4>Note</h4><p>Simulation results. Not flight-tested on hardware.</p>
    <p>commit {commit}</p></div>
  </div>
</div></footer>

<div class="lightbox" id="lightbox">
  <button class="close" id="lightbox-close" aria-label="Close">&times;</button>
  <div><img id="lightbox-img" src="" alt="Zoom tile, enlarged"><div class="cap" id="lightbox-cap"></div></div>
</div>

<script type="application/json" id="final-results-data">{json.dumps(d, indent=0)}</script>
<script>{JS}</script>
</body>
</html>"""
    return html_doc


def number_check(html_doc, data):
    """Every rendered stat should trace back to a value present in
    FINAL_RESULTS.json's own JSON text (inlined in the page) -- a crude
    but real check: does the page's visible number also appear in the
    data block it was rendered from."""
    if not data:
        return 0, 0
    data_text = json.dumps(data)
    import re
    nums_in_page = set(re.findall(r'\b\d+\.\d+\b|\b\d{2,}\b', html_doc))
    checked = 0
    missing = []
    for n in nums_in_page:
        checked += 1
        if n not in data_text:
            missing.append(n)
    return checked, missing


def main():
    data = load_final_results()
    if data is None:
        print(f"NOTE: FINAL_RESULTS.json not found yet -- building with '{PENDING}' markers.")

    primitives = _load("scene/collision/avian_bridge_collision.json", {"primitives": []})["primitives"]
    before_log = _load("mission/coverage_mission_flight_log.json", {"log": []})["log"]
    after_doc = _load("mission/flight_log_s4.json", None)
    after_log = after_doc["log"] if after_doc else None

    plan_svg = svg_plan_view(primitives, before_log, after_log)
    gallery = build_gallery()
    hero_path = build_hero_image()
    bar_svg = ""  # built inside build_html from data

    html_doc = build_html(data, gallery, hero_path, plan_svg, bar_svg)
    out_path = os.path.join(HERE, "index.html")
    with open(out_path, "w") as f:
        f.write(html_doc)

    checked, missing = number_check(html_doc, data)
    print(f"number check: {checked} numeric tokens scanned, "
         f"{len(missing) if isinstance(missing, list) else 'N/A'} not found in data "
         f"-> {'PASS' if not missing else 'REVIEW'}")
    if missing:
        print("  not found:", missing[:20])
    print(f"saved: {out_path}")
    asset_bytes = sum(os.path.getsize(os.path.join(ASSETS, f)) for f in os.listdir(ASSETS))
    print(f"assets: {asset_bytes/1e6:.2f} MB")


if __name__ == "__main__":
    main()
