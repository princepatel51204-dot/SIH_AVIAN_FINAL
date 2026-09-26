"""Generate the opening and closing cards as held frame sequences.

Every number on these cards is measured and traceable:

  1161 / 1162 waypoints, 0 collisions, 81.10 % coverage
      commit 68d71d6, full_pass_05's committed results
  9050.3 m, 2.61 h
      summed directly from pose_audit_track.csv (160,052 samples)
  1.9 m/s cruise, 3.0 m clearance
      full_pass_05 mission_log.json speed_calc

Nothing is rounded up and nothing is inferred. The closing card also states
the real/staged split, because the film shows both and the viewer is entitled
to know which is which without reading the report.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import overlay as O
import shotlist as SL

ROOT = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL"
FILM = f"{ROOT}/media/_film"
W, H = 1920, 1080
HOLD = 96          # 4.0 s at 24 fps


def emit(name, lines, sub):
    d = f"{FILM}/{name}/ui"
    os.makedirs(d, exist_ok=True)
    one = f"{FILM}/{name}/_card.png"
    O.title_card(W, H, lines, one, sub=sub)
    from PIL import Image
    img = Image.open(one)
    for i in range(HOLD):
        img.save(f"{d}/f{i:04d}.png")
    print(f"  {name}: {HOLD} frames")


def main():
    n_real = sum(1 for b in SL.BEATS if b["kind"] == "real")
    n_stg = len(SL.BEATS) - n_real
    emit("title_open",
         ["AVIAN",
          "AUTONOMOUS BRIDGE INSPECTION",
          "Gazebo flight  ·  Blender survey  ·  live defect detection"],
         "Every box and confidence in this film is live output of the trained "
         "detector (v2, threshold 0.65) run on these rendered frames.")
    emit("title_end",
         ["full_pass_05",
          "1161 / 1162 waypoints reached  ·  0 collisions",
          "81.10 % measured coverage  ·  9050.3 m flown over 2.61 h",
          "1.9 m/s cruise  ·  3.0 m sensed clearance"],
         f"{n_real} of {len(SL.BEATS)} beats use camera positions actually flown in "
         f"full_pass_05; {n_stg} use legal staged viewpoints at >= 3.5 m standoff.")


if __name__ == "__main__":
    main()
