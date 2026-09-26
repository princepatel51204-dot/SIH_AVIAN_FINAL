"""Run the real detector over every rendered frame, draw its output, log it all.

Order matters and is not negotiable: the detector sees the finished render,
then the UI draws what the detector returned. Nothing is drawn that the model
did not produce on that exact frame, and every drawn box is written to the
detection log with the frame it came from.

Per beat the UI follows the reference timing in shotlist.py:

    approach   detector runs, results logged, nothing drawn -- the system has
               not settled and an inspection HUD that flickers during transit
               would be misrepresenting how it behaves
    hold       box animates in over 5 frames and stays
    card       annotation card slides in, box still up
    peel       both fade out

Wides get the HUD only. Title cards are generated stills.

Outputs:
    media/_film/<shot>/ui/f####.png   composited frames
    detection_log.json / .csv         every detection drawn, with provenance
"""
from __future__ import annotations

import csv
import glob
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import torch
import overlay as O
import paths as P
import shotlist as SL
from detect import load_model, detect

ROOT = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL"
FILM = f"{ROOT}/media/_film"
THRESH = 0.65
BOX_IN = 5          # frames the box takes to animate in
CARD_IN = 8

# Wides are composited after the beats, so the running counter is already at
# its final value by then. The count a viewer sees has to be the number of
# beats that have played before the wide in the film's edit order.
WIDE_COUNT = {"wide_open": 0, "wide_mid_a": 2, "wide_mid_b": 5, "wide_close": 8}

gt = {d["defect_id"]: d
      for d in json.load(open(f"{ROOT}/scene/AVIAN_defect_ground_truth_FINAL.json"))["defects"]}


def beat_phase(i, acquire):
    """-> (draw_box, box_phase, card_phase) for frame index i within a beat.

    `acquire` is the first frame index at or after the settle where the
    detector actually returned something. The box animates in from THAT frame,
    not from the nominal start of the hold: on some beats the detector only
    locks on part-way through the settle (beat 2 acquires at 0.776 several
    frames in), and animating from the nominal start would pop the box on at
    full opacity the moment it first fires.
    """
    a, h, c = SL.APPROACH, SL.HOLD, SL.CARD
    if i < a or acquire is None or i < acquire:
        return False, 0.0, 0.0
    if i < a + h:
        return True, min(1.0, (i - acquire + 1) / BOX_IN), 0.0
    if i < a + h + c:
        return True, 1.0, min(1.0, (i - a - h + 1) / CARD_IN)
    u = (i - a - h - c) / max(1, SL.PEEL - 1)
    f = max(0.0, 1.0 - u)
    return True, f, f


def size_text(d):
    """Only quote a dimension the ground truth actually carries."""
    bits = []
    if d.get("length_m"):
        bits.append(f"{d['length_m']:.2f} m long")
    if d.get("width_mm"):
        bits.append(f"{d['width_mm']:.2f} mm wide")
    if d.get("depth_mm"):
        bits.append(f"{d['depth_mm']:.1f} mm deep")
    return " · ".join(bits) if bits else None


def main():
    torch.set_num_threads(int(os.environ.get("TORCH_THREADS", "12")))
    model = load_model()
    rows = []
    n_drawn = 0
    counter = 0
    t0 = time.time()

    shots = sorted(glob.glob(f"{FILM}/*/meta.json"))
    order = []
    for w in SL.WIDES:
        if w["name"] == "open":
            order.append(f"{FILM}/wide_open/meta.json")
    for b in SL.BEATS:
        order.append(f"{FILM}/beat{b['n']}_{b['defect']}/meta.json")
    for w in SL.WIDES:
        if w["name"] != "open":
            order.append(f"{FILM}/wide_{w['name']}/meta.json")
    order = [p for p in order if p in shots]

    for mp in order:
        meta = json.load(open(mp))
        dirp = os.path.dirname(mp)
        uidir = f"{dirp}/ui"
        os.makedirs(uidir, exist_ok=True)
        frames = sorted(glob.glob(f"{dirp}/f*.png"))
        is_beat = meta["kind"] in ("real", "staged")
        beat = None
        geom = None
        if is_beat:
            beat = next(b for b in SL.BEATS if b["defect"] == meta["defect"])
            d = gt[meta["defect"]]
            # true per-frame altitude and camera-to-defect range, recomputed
            # from the same inputs the renderer used
            geom = P.beat_geometry(beat, len(frames), SL.FPS, SL.APPROACH,
                                   SL.HOLD, SL.CARD, SL.PEEL,
                                   SL.MIN_LEGAL_STANDOFF)

        # pass 1: ask the detector about every frame before drawing anything,
        # so the box can animate in from the frame it genuinely acquires on
        per_frame = []
        for fp in frames:
            dets, dt, _ = detect(model, fp, THRESH)
            per_frame.append((dets, dt))
        acquire = None
        if is_beat:
            for i, (dets, _) in enumerate(per_frame):
                if i >= SL.APPROACH and dets:
                    acquire = i
                    break
            if acquire is None:
                print(f"  !! {meta['shot']}: detector never fired in the settle "
                      f"phase -- no box will be drawn for this beat", flush=True)

        shot_hits = 0
        for i, fp in enumerate(frames):
            dets, dt = per_frame[i]
            draw, bph, cph = (True, 1.0, 0.0)
            if is_beat:
                draw, bph, cph = beat_phase(i, acquire)

            shown = dets if (draw and dets) else []
            if shown:
                shot_hits += 1

            hud = {}
            if is_beat:
                g = geom[i] if geom and i < len(geom) and geom[i] else None
                hud = {
                    "title": "AVIAN — AUTONOMOUS BRIDGE INSPECTION",
                    "subtitle": f"{meta['section']} · {meta['surface']}",
                    "alt": (f"{g['alt_m']:.1f} m" if g else None),
                    "range": (f"{g['range_m']:.1f} m" if g else None),
                    # distinct defects confirmed so far, not frames with a box
                    "count": counter + (1 if acquire is not None and i >= acquire else 0),
                    "note": ("full_pass_05 · flown position"
                             if meta["kind"] == "real" else "legal viewpoint · staged"),
                }
            else:
                hud = {"title": "AVIAN — AUTONOMOUS BRIDGE INSPECTION",
                       "subtitle": meta.get("label", "").upper(),
                       "count": WIDE_COUNT.get(meta["shot"], counter),
                       "note": "full_pass_05 · flown trajectory"}

            card = None
            if is_beat and cph > 0.01 and shown:
                best = max(shown, key=lambda x: x["score"])
                r = [("DEFECT ID", meta["defect"].replace("DEFECT_", "")),
                     ("DETECTOR", f"{best['family']} {best['score']:.3f}"),
                     ("STANDOFF", f"{(geom[i]['range_m'] if geom and geom[i] else meta['range_m_min']):.2f} m")]
                st = size_text(d)
                if st:
                    r.append(("SIZE", st))
                r.append(("WORLD X/Y/Z", " / ".join(f"{v:.1f}" for v in meta["defect_xyz"])))
                card = {"title": f"{d['type'].replace('_',' ')} — {meta['surface']}",
                        "family": best["family"], "rows": r}

            O.compose(fp, f"{uidir}/{os.path.basename(fp)}", dets=shown,
                      hud=hud, card=card, det_phase=bph, card_phase=cph)

            for dd in dets:
                g = geom[i] if (geom and i < len(geom) and geom[i]) else None
                rows.append({
                    "shot": meta["shot"], "frame_index": i,
                    "frame_file": os.path.basename(fp),
                    "drawn": bool(dd in shown),
                    "family": dd["family"], "confidence": dd["score"],
                    "bbox_x0": dd["bbox"][0], "bbox_y0": dd["bbox"][1],
                    "bbox_x1": dd["bbox"][2], "bbox_y1": dd["bbox"][3],
                    "shot_kind": meta["kind"],
                    "defect_id": meta.get("defect", ""),
                    "standoff_m": meta.get("range_m_min", ""),
                    "cam_xyz": ";".join(str(v) for v in meta.get("cam_xyz_first", [])),
                    "sim_time_window": ";".join(str(v) for v in meta.get("t_sim_window", [])),
                    "inference_s": round(dt, 4),
                    "frame_range_m": round(g["range_m"], 3) if g else "",
                    "frame_alt_m": round(g["alt_m"], 3) if g else "",
                })
                if dd in shown:
                    n_drawn += 1
        if is_beat and shot_hits:
            counter += 1
        acq = "-" if not is_beat else (str(acquire) if acquire is not None else "NEVER")
        print(f"  {meta['shot']:<34} {len(frames):4d} frames, "
              f"{shot_hits:4d} drawn, acquire@{acq}", flush=True)

    json.dump(rows, open(f"{ROOT}/scene/film/detection_log.json", "w"), indent=1)
    if rows:
        with open(f"{ROOT}/scene/film/detection_log.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
    print(f"\n===COMPOSE_DONE=== {time.time()-t0:.1f}s  "
          f"{len(rows)} detector records, {n_drawn} drawn")


if __name__ == "__main__":
    main()
