"""The inspection UI drawn over rendered frames.

Every box, class name and confidence drawn here comes from a detection record
produced by the real trained detector running on that exact rendered frame.
This module has no opinion about whether a defect is present -- it is a
renderer for detector output, and it draws nothing when the detector returned
nothing.

Colour language matches live_detector_node.py's FAMILY_COLOR so the film and
the flight software speak the same visual language.

Restraint rules, applied deliberately:
  * 2 px stroke, no glow, no drop shadow on the box itself
  * corner brackets rather than a full rectangle once settled -- reads as an
    instrument, not a game HUD
  * one accent colour per class, everything else neutral grey/white
  * HUD values are monospaced so digits do not jitter frame to frame
"""
from __future__ import annotations

import os

from PIL import Image, ImageDraw, ImageFont

FONT_DIR = "/usr/share/fonts/truetype/dejavu"
F_LABEL = f"{FONT_DIR}/DejaVuSansCondensed-Bold.ttf"
F_TEXT = f"{FONT_DIR}/DejaVuSansCondensed.ttf"
F_MONO = f"{FONT_DIR}/DejaVuSansMono.ttf"

FAMILY_COLOR = {
    "CRACK": (255, 60, 60),
    "SPALL_DELAM": (255, 170, 30),
    "CORROSION_COATING": (60, 170, 255),
    "FASTENER": (170, 60, 255),
    "OTHER": (200, 200, 200),
}
INK = (238, 242, 245)
DIM = (150, 160, 168)
PANEL = (14, 18, 22)

_FC: dict = {}


def font(path, size):
    key = (path, size)
    if key not in _FC:
        _FC[key] = ImageFont.truetype(path, size)
    return _FC[key]


def _ease_out(x):
    x = max(0.0, min(1.0, x))
    return 1.0 - (1.0 - x) ** 3


def _panel(draw, xy, radius=3, alpha=205, outline=None):
    x0, y0, x1, y1 = xy
    draw.rounded_rectangle(xy, radius=radius, fill=PANEL + (alpha,),
                           outline=outline, width=1 if outline else 0)


def _text(draw, xy, s, f, fill, anchor=None):
    draw.text(xy, s, font=f, fill=fill, anchor=anchor)


def draw_detection(layer, det, phase, W, H):
    """One detector box. `phase` 0..1 animates it in over the first ~0.2 s."""
    col = FAMILY_COLOR.get(det["family"], FAMILY_COLOR["OTHER"])
    x0, y0, x1, y1 = det["bbox"]
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    e = _ease_out(phase)
    # scale in from 108% so it "snaps" onto the defect
    k = 1.08 - 0.08 * e
    bx0, by0 = cx - (cx - x0) * k, cy - (cy - y0) * k
    bx1, by1 = cx + (x1 - cx) * k, cy + (y1 - cy) * k
    a = int(255 * e)
    d = ImageDraw.Draw(layer, "RGBA")

    # corner brackets, 18% of the shorter side
    L = max(14, int(min(bx1 - bx0, by1 - by0) * 0.18))
    for px, py, sx, sy in ((bx0, by0, 1, 1), (bx1, by0, -1, 1),
                           (bx0, by1, 1, -1), (bx1, by1, -1, -1)):
        d.line([(px, py), (px + sx * L, py)], fill=col + (a,), width=2)
        d.line([(px, py), (px, py + sy * L)], fill=col + (a,), width=2)
    # faint full rectangle underneath so the extent is unambiguous
    d.rectangle([bx0, by0, bx1, by1], outline=col + (int(a * 0.28),), width=1)

    if e < 0.45:
        return
    la = int(255 * _ease_out((e - 0.45) / 0.55))
    fl, fm = font(F_LABEL, 21), font(F_MONO, 21)
    name = det["family"]
    conf = f"{det['score']:.2f}"
    wn = d.textlength(name, font=fl)
    wc = d.textlength(conf, font=fm)
    pad, gap = 9, 14
    bw, bh = wn + wc + gap + pad * 2, 31
    lx, ly = bx0, by0 - bh - 6
    if ly < 4:
        ly = by1 + 6
    lx = max(4, min(lx, W - bw - 4))
    _panel(d, (lx, ly, lx + bw, ly + bh), alpha=int(210 * la / 255))
    d.rectangle([lx, ly, lx + 3, ly + bh], fill=col + (la,))
    _text(d, (lx + pad + 3, ly + bh / 2), name, fl, INK + (la,), anchor="lm")
    _text(d, (lx + pad + 3 + wn + gap, ly + bh / 2), conf, fm, col + (la,), anchor="lm")


def draw_hud(layer, hud, W, H, alpha=1.0):
    """Corner instrument readouts. Every value must come from measured data."""
    d = ImageDraw.Draw(layer, "RGBA")
    a = int(255 * alpha)
    fs, fm = font(F_TEXT, 16), font(F_MONO, 19)

    # top-left: what we are looking at
    if hud.get("title"):
        _text(d, (34, 30), hud["title"], font(F_LABEL, 19), INK + (a,), anchor="lm")
    if hud.get("subtitle"):
        _text(d, (34, 52), hud["subtitle"], fs, DIM + (a,), anchor="lm")

    # top-right: flight instruments, right-aligned, monospaced
    rows = [r for r in (
        ("ALT", hud.get("alt")), ("RANGE", hud.get("range")),
        ("WP", hud.get("wp")),
    ) if r[1] is not None]
    y = 30
    for k, v in rows:
        _text(d, (W - 34, y), str(v), fm, INK + (a,), anchor="rm")
        _text(d, (W - 34 - d.textlength(str(v), font=fm) - 10, y + 1), k, fs, DIM + (a,), anchor="rm")
        y += 24

    # bottom-left: running detection count
    if hud.get("count") is not None:
        _text(d, (34, H - 38), f"{hud['count']:02d}", font(F_MONO, 26),
              FAMILY_COLOR["SPALL_DELAM"] + (a,), anchor="lm")
        _text(d, (34 + 40, H - 38), "DETECTIONS", fs, DIM + (a,), anchor="lm")
    if hud.get("note"):
        _text(d, (W - 34, H - 38), hud["note"], fs, DIM + (int(a * 0.8),), anchor="rm")


def draw_card(layer, card, phase, W, H):
    """Annotation card. Only fields that were actually measured are passed in."""
    e = _ease_out(phase)
    if e <= 0.01:
        return
    a = int(235 * e)
    d = ImageDraw.Draw(layer, "RGBA")
    ft, fk, fv = font(F_LABEL, 20), font(F_TEXT, 15), font(F_MONO, 16)

    rows = card.get("rows", [])
    wk = max([d.textlength(k, font=fk) for k, _ in rows] + [60])
    wv = max([d.textlength(str(v), font=fv) for _, v in rows] + [60])
    bw = int(max(wk + wv + 46, d.textlength(card["title"], font=ft) + 34))
    bh = 46 + 23 * len(rows)
    x0 = 34
    y0 = H - 92 - bh
    slide = int((1 - e) * 26)

    _panel(d, (x0, y0 + slide, x0 + bw, y0 + bh + slide), radius=4, alpha=a)
    col = FAMILY_COLOR.get(card.get("family", "OTHER"), FAMILY_COLOR["OTHER"])
    d.rectangle([x0, y0 + slide, x0 + 3, y0 + bh + slide], fill=col + (a,))
    _text(d, (x0 + 16, y0 + 24 + slide), card["title"], ft, INK + (a,), anchor="lm")
    yy = y0 + 48 + slide
    for k, v in rows:
        _text(d, (x0 + 16, yy), k, fk, DIM + (a,), anchor="lm")
        _text(d, (x0 + bw - 16, yy), str(v), fv, INK + (a,), anchor="rm")
        yy += 23


def compose(base_png, out_png, dets=None, hud=None, card=None,
            det_phase=1.0, card_phase=0.0, hud_alpha=1.0):
    """Draw the UI for one frame. Returns the number of boxes drawn."""
    img = Image.open(base_png).convert("RGB")
    W, H = img.size
    layer = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    n = 0
    for det in (dets or []):
        draw_detection(layer, det, det_phase, W, H)
        n += 1
    if hud:
        draw_hud(layer, hud, W, H, alpha=hud_alpha)
    if card:
        draw_card(layer, card, card_phase, W, H)
    out = Image.alpha_composite(img.convert("RGBA"), layer).convert("RGB")
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    out.save(out_png)
    return n


def title_card(W, H, lines, out_png, sub=None):
    """A restrained title card. Callers pass only measured numbers."""
    img = Image.new("RGB", (W, H), (9, 12, 15))
    d = ImageDraw.Draw(img)
    y = H // 2 - (len(lines) * 30) // 2
    for i, ln in enumerate(lines):
        f = font(F_LABEL, 40 if i == 0 else 22)
        _text(d, (W // 2, y), ln, f, INK if i == 0 else DIM, anchor="mm")
        y += 52 if i == 0 else 32
    if sub:
        _text(d, (W // 2, H - 60), sub, font(F_TEXT, 15), DIM, anchor="mm")
    img.save(out_png)
    return out_png
