# Opening AVIAN_Smart_Infrastructure_City.blend on your laptop

Requires **Blender 4.2 or newer** (built and tested on 4.5 LTS).
Download: https://www.blender.org/download/

---

## 1. Unzip

```bash
unzip AVIAN_Smart_Infrastructure_City.zip
cd AVIAN_Smart_Infrastructure_City
```

## 2. Open the model

### Windows

Double-click `AVIAN_Smart_Infrastructure_City.blend`, or from **Command
Prompt / PowerShell**:

```powershell
"C:\Program Files\Blender Foundation\Blender 4.5\blender.exe" AVIAN_Smart_Infrastructure_City.blend
```

If Blender is on your PATH, this is enough:

```powershell
blender AVIAN_Smart_Infrastructure_City.blend
```

### macOS

```bash
open -a Blender AVIAN_Smart_Infrastructure_City.blend
```

or

```bash
/Applications/Blender.app/Contents/MacOS/Blender AVIAN_Smart_Infrastructure_City.blend
```

### Linux

```bash
blender AVIAN_Smart_Infrastructure_City.blend
```

---

## 3. First things to do once it opens

The scene is 4.5 km long, so the default viewport clipping will hide most of
it. **Do this first:**

1. Press `N` in the 3D viewport to open the sidebar → **View** tab
2. Set **Clip Start** to `0.1` and **End** to `60000`

Then:

| Action | How |
|---|---|
| Frame the whole scene | `Home` |
| Look through a named camera | `Ctrl` + `Numpad 0` with the camera selected, or **View ▸ Cameras** |
| Cycle cameras | Select one in the Outliner under `CAMERAS`, then `Ctrl` + `Numpad 0` |
| Fly through | `Shift` + `` ` `` (backtick), then `WASD`, mouse to look, `Shift` to speed up |
| Solid / material / rendered view | `Z` then pick, or the four spheres top-right |
| Isolate a collection | Click the checkbox next to it in the Outliner |

**Start with `CAMERA_04_RESEARCH_ZONE`** — that is the 900 m high-detail zone
where all 192 defects live. Then `CAMERA_05_UNDERBRIDGE` and
`CAMERA_06_DEFECT_CLOSEUP`.

> Rendered view (`Z` ▸ Rendered) uses Cycles and will be slow on CPU.
> Use **Material Preview** instead for navigating; it is EEVEE and interactive.

---

## 4. Where everything is (Outliner)

```
AVIAN_ENVIRONMENT
├── BRIDGE            DECK · PIERS · BEAMS · JOINTS · BARRIERS · DETAILS
├── RIVER             terrain, water, banks, rocks, jetty
├── ROADS             deck markings, surface roads, ramps, lighting
├── CITY              BUILDINGS · VEHICLES · STREET_LIGHTS · VEGETATION
├── STRUCTURAL_DAMAGE CRACKS · SPALLING · REBAR · CORROSION · JOINT_DAMAGE
├── UAV_AIRSPACE      33 wireframe volumes, never rendered
├── INSPECTION_SECTORS   6 sector volumes A–F
├── INSPECTION_SCENARIOS 9 difficulty markers (Empties)
├── CAMERAS           the 7 named views
└── LIGHTING_ENVIRONMENT  sun + sky
```

Airspace volumes are wireframe and have `hide_render = True`, so they show in
the viewport but never appear in a render. To hide them while navigating,
untick `UAV_AIRSPACE` in the Outliner.

---

## 5. Reading the ground truth out of the .blend

Every defect object carries its own metadata. Select one and look at
**Object Properties ▸ Custom Properties**, or in the Scripting workspace:

```python
import bpy

for ob in bpy.data.objects:
    if ob.get("avi_defect_id"):
        print(ob["avi_defect_id"],
              ob["avi_type"],
              "sev", ob["avi_severity"],
              ob["avi_host_surface"],
              ob["avi_sector"],
              "repairable" if ob["avi_repairable"] else "ESCALATE")
```

The same data, plus positions and normals, is in
`AVIAN_defect_ground_truth.json` and `.csv`.

Airspace volumes carry `avi_zone_class`, `avi_x_min_m` … `avi_z_max_m`,
`avi_rule` and `avi_gnss`. Same pattern:

```python
for ob in bpy.data.objects:
    if ob.get("avi_kind") == "airspace_volume":
        print(ob.name, ob["avi_zone_class"],
              ob["avi_x_min_m"], ob["avi_x_max_m"],
              ob["avi_z_min_m"], ob["avi_z_max_m"])
```

---

## 6. Rebuilding it from the scripts (optional)

The .blend is generated. If you want to change a dimension, edit `params.py`
and regenerate rather than editing the mesh by hand.

```bash
pip install bpy==4.5.*            # Blender as a Python module, needs Python 3.11
python build_scene.py             # build + run the 20 validation checks + save
python build_scene.py --render    # also render the 9 validation views
python build_scene.py --render --samples 64
```

Or run it through your installed Blender instead of `pip install bpy`:

```bash
blender --background --python build_scene.py
```

Build takes about 35 seconds. `params.SEED = 20260827` drives every random
placement, so the result is reproducible.

---

## 7. Rendering a still yourself

From the GUI: pick a camera, then `F12`. From the command line:

```bash
blender -b AVIAN_Smart_Infrastructure_City.blend \
        -o //out_#### -F PNG -f 1
```

To render through a specific camera, set it as the scene camera first
(select it, `Ctrl` + `Numpad 0`, save), or add
`--python-expr "import bpy; bpy.context.scene.camera = bpy.data.objects['CAMERA_05_UNDERBRIDGE']"`
before `-f 1`.

If you have an NVIDIA or AMD GPU, switch **Render Properties ▸ Device** to
GPU Compute — the scene was built on a 2-core CPU box and the render settings
reflect that.
