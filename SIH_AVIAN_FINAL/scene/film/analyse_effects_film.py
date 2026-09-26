"""Compare frames decoded from the finished mp4 against production / DoF-off /
motion-blur-off renders of the same moment. All in PIL/numpy, top-down on both
sides (no Blender pixel buffers, so no orientation flip).

Only pixels the UI overlay did not touch are compared (ui frame == raw frame).
"""
import subprocess, json, glob, os, numpy as np
from PIL import Image
from scipy.ndimage import binary_erosion
R = "/home/prince/avian_rev_c/SIH_AVIAN_FINAL"; V = f"{R}/media/avian_inspection_film.mp4"
FX = f"{R}/media/_vfilm/fx"; F = f"{R}/media/_film"
EDIT = ["title_open","wide_open","beat1","beat2","wide_mid_a","beat3","beat4","beat5","wide_mid_b","beat6","beat7","beat8","wide_close","title_end"]
LENS = dict(title_open=96,wide_open=144,beat1=192,beat2=192,wide_mid_a=48,beat3=192,beat4=192,beat5=192,wide_mid_b=48,beat6=192,beat7=192,beat8=192,wide_close=144,title_end=96)
starts = {}; s = 0
for n in EDIT: starts[n] = s; s += LENS[n] - 10
ld = lambda p: np.asarray(Image.open(p).convert("RGB"), dtype=np.float32) / 255
def sharp(x, m=None):
    g = x @ np.array([.2126,.7152,.0722], np.float32)
    lap = -4*g[1:-1,1:-1] + g[:-2,1:-1] + g[2:,1:-1] + g[1:-1,:-2] + g[1:-1,2:]
    return float(lap.var() if m is None else lap[m[1:-1,1:-1]].var())
res = []
for bn, i in [(4,30),(4,100),(8,30),(8,100)]:
    d = glob.glob(f"{F}/beat{bn}_*")[0]
    p = f"{FX}/film_b{bn}_f{i:04d}.png"
    subprocess.run(f'ffmpeg -y -v error -i "{V}" -vf "select=eq(n\\,{starts[f"beat{bn}"]+i})" -vsync 0 -frames:v 1 "{p}"', shell=True)
    film = ld(p); raw = ld(f"{d}/f{i:04d}.png"); ui = ld(f"{d}/ui/f{i:04d}.png")
    prod = ld(f"{FX}/b{bn}_f{i:04d}_prod.png"); dof = ld(f"{FX}/b{bn}_f{i:04d}_dofoff.png"); mb = ld(f"{FX}/b{bn}_f{i:04d}_mboff.png")
    mask = binary_erosion((np.abs(ui - raw).max(2) < 1/255), iterations=4)
    mad = lambda a, b: float(np.abs(a - b)[mask].mean())
    r = dict(beat=bn, frame=i, unmasked_frac=round(float(mask.mean()), 3),
        prodref_vs_disk_raw=round(mad(prod, raw), 6),
        film_vs_prod=round(mad(film, prod), 6), film_vs_dofoff=round(mad(film, dof), 6), film_vs_mboff=round(mad(film, mb), 6),
        ref_prod_vs_dofoff=round(mad(prod, dof), 6), ref_prod_vs_mboff=round(mad(prod, mb), 6),
        sharp_film=round(sharp(film, mask), 7), sharp_dofoff=round(sharp(dof, mask), 7), sharp_prod=round(sharp(prod, mask), 7),
        sharp_mboff=round(sharp(mb, mask), 7))
    r["dof_in_film"] = r["film_vs_prod"] < r["film_vs_dofoff"]
    r["mb_in_film"] = r["film_vs_prod"] < r["film_vs_mboff"]
    res.append(r); print(json.dumps(r), flush=True)
json.dump(res, open(f"{R}/media/_vfilm/effects_result.json", "w"), indent=1)

# --- targeted test: only where the effect actually changes pixels -------------
print("\nTARGETED (top 2% of pixels by |prod - reference| inside the un-overlaid mask)")
tgt = []
for bn, i in [(4,30),(4,100),(8,30),(8,100)]:
    d = glob.glob(f"{F}/beat{bn}_*")[0]
    film = ld(f"{FX}/film_b{bn}_f{i:04d}.png"); raw = ld(f"{d}/f{i:04d}.png"); ui = ld(f"{d}/ui/f{i:04d}.png")
    prod = ld(f"{FX}/b{bn}_f{i:04d}_prod.png"); ref = {"dofoff": ld(f"{FX}/b{bn}_f{i:04d}_dofoff.png"), "mboff": ld(f"{FX}/b{bn}_f{i:04d}_mboff.png")}
    mask = binary_erosion((np.abs(ui - raw).max(2) < 1/255), iterations=4)
    for name, r in ref.items():
        eff = np.abs(prod - r).mean(2); eff[~mask] = 0
        thr = np.quantile(eff[mask], 0.98); m2 = mask & (eff >= thr)
        fp = float(np.abs(film - prod)[m2].mean()); fr = float(np.abs(film - r)[m2].mean()); pr = float(np.abs(prod - r)[m2].mean())
        # fraction of those pixels individually closer to prod than to the reference
        closer = float((np.abs(film - prod).mean(2)[m2] < np.abs(film - r).mean(2)[m2]).mean())
        row = dict(beat=bn, frame=i, effect=name, n_px=int(m2.sum()), ref_prod_vs_ref=round(pr,5), film_vs_prod=round(fp,5), film_vs_ref=round(fr,5),
                   frac_px_closer_to_prod=round(closer,3), present=bool(fp < fr and closer > 0.5))
        tgt.append(row); print(json.dumps(row), flush=True)
json.dump(tgt, open(f"{R}/media/_vfilm/effects_targeted.json", "w"), indent=1)
