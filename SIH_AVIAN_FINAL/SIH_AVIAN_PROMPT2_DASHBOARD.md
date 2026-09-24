# PROMPT 2 of 2 — The AVIAN mission dashboard (professional, light theme)

**Start after Prompt 1's FINAL HANDOFF. Hard stop: 13:30 IST.** The deck
needs a screenshot by then.

**Goal:** one page that a judge or mentor opens and understands in 30
seconds: what the drone did, what it found, and what it can't do yet.
Every number on it is real.

---

## Visual direction: the design language of nasa.gov, not its branding

Take from nasa.gov:
- a clean white page
- large, confident imagery at the top
- strong near-black headlines with generous spacing
- a restrained palette with one accent colour
- a clear card grid
- a mission-report tone: factual and calm, no hype

**Do NOT use NASA's name, logo (the "meatball" or the "worm"), mission
names, or exact brand colours anywhere.** This is AVIAN's own identity,
inspired by that clarity, not a copy of it.

**Design tokens** (use these, don't improvise):

| Token | Value | Use |
|---|---|---|
| `--bg` | `#FFFFFF` | page |
| `--surface` | `#F5F7FA` | cards, table stripes |
| `--ink` | `#0E1116` | headlines, body |
| `--ink-2` | `#4A5568` | secondary text, captions |
| `--line` | `#DDE3EA` | borders, grid lines |
| `--accent` | `#1D4ED8` | links, key numbers, "after" series |
| `--ok` | `#15803D` | settled / true positive |
| `--warn` | `#B45309` | stuck / missed defect |
| `--bad` | `#B91C1C` | false positive / collision |

- **Type:** Public Sans (Google Fonts), falling back to
  `system-ui, -apple-system, "Segoe UI", Arial, sans-serif`. The page must
  look right offline using the fallback.
  - Headlines: 700 weight, tight line height, 40–56 px in the hero, 28 px
    for section titles.
  - Body: 16 px, line height 1.6.
  - Use `font-variant-numeric: tabular-nums` for all numbers.
- **Layout:**
  - 1200 px max width, 24 px side gutters.
  - 12-column grid that collapses to a single column under 760 px.
  - Section spacing 72–96 px.
  - 8 px corner radius on cards, 1 px `--line` borders, no heavy shadows.
- **Numbers:** big stat numbers are 44–56 px, in `--ink` or `--accent`,
  each with a small caption *and its sample size* (e.g. "39 of 73
  defects").

## Build approach: static, offline, generated from data

- Write `dashboard/build_dashboard.py`. It reads **only**
  `FINAL_RESULTS.json` plus the committed flight, mission and detection
  files, and writes `dashboard/index.html` with **all data inlined** (a
  `<script type="application/json">` block). The page then works when
  opened from `file://`, where browsers block `fetch()`.
- Copy the images the page uses into `dashboard/assets/`, resized to at
  most 1600 px wide JPG at quality 82. Keep the folder under 15 MB.
- No frameworks and no CDN scripts. Use inline SVG for all charts, drawn
  by Python, and a small amount of vanilla JS only for the toggle and the
  lightbox.
- **No hand-typed numbers in the HTML.** Every figure comes from the build
  script. Re-running it after any data change regenerates the page.

## Page sections, in order

1. **Top bar (sticky, white, 1 px bottom border):** "AVIAN" wordmark in
   text, not a logo. Anchor links: Mission · Autonomy · Detection ·
   Pipeline · Limits. On the right: `SIH26201 · commit <hash>`.
2. **Hero:**
   - A full-width render of the bridge (`CAM_10_TRUSS` or the best
     overview) with a light dark-to-transparent overlay so the text on it
     is readable.
   - Headline: "Autonomous bridge inspection, measured end to end."
   - Subline: one sentence saying this is a simulation digital twin of a
     360 m corridor with 192 known defects.
   - Below the image, a row of 4 stats:
     - coverage %
     - collision rate before → after
     - defects in view (n/73)
     - false alarms per 100 tiles (v1 → headline model)
3. **Mission plan view (the centrepiece):**
   - A top-down SVG of the corridor. Draw the structure footprints from
     the collision primitives' x/y extents in light grey.
   - Draw the flown path as a thin line.
   - Mark waypoints as dots: `--ok` when settled, `--warn` when stuck.
   - Add a **Before / After toggle** switching between the original
     Stage 2 flight and `flight_log_s4.json`. If "after" wasn't run, hide
     the toggle and add a note.
   - Include a legend, a scale bar in metres, and a north/+X arrow.
4. **Autonomy:**
   - A simple bar chart of stuck % for Stage 1, Stage 2 before, and
     Stage 2 after.
   - Beside it, three facts with sample sizes: sensed-avoidance log
     entries, avoidance reactions, battery packs.
   - One sentence explaining *why* collisions happened, taken from the
     diagnosis file.
5. **Detection:**
   - **Model table** for v1, v2 and v3. Columns: threshold, F1 on
     MISSION-VAL, MISSION-TEST unseen recall (n/N), all-defects recall
     (n/N, marked optimistic), false positives per 100 tiles. Highlight
     the headline row.
   - **Evidence gallery:** 6 zoom tiles, each with its boxes drawn and a
     label, **mixing outcomes on purpose**: true positives, false
     positives, and at least one missed defect (draw its ground-truth
     outline dashed in `--warn`). Click a tile to open it larger
     (lightbox).
   - Show the real-photo crack test result, or "not run" with the reason.
   - One line on bolts (FASTENER): not detectable at the current camera
     resolution.
6. **Pipeline:** six steps in a horizontal row, stacked on mobile:
   Digital twin → Plan coverage → Fly & avoid → Zoom capture (8.7×, 9°)
   → Detect → Score. Each gets a one-line description and the file that
   implements it.
7. **Limits: "What this prototype does not do yet".** Taken from
   `not_done_yet` in `FINAL_RESULTS.json`, shown as a plain list. Keep it
   visible; don't hide it at the bottom in small print.
8. **Footer:**
   - the data source of every figure (file@commit)
   - model licenses
   - "Simulation results. Not flight-tested on hardware."

## Quality bar (check before finishing)

- It opens from `file://` with no console errors, and works with the
  internet off.
- It looks right at 1440 px, 1024 px and 390 px wide, with no horizontal
  scrolling.
- Contrast is at least 4.5:1 on all text, and keyboard focus is visible.
- All images have `alt` text.
- Every number on the page appears in `FINAL_RESULTS.json`. Write a small
  check in the build script that asserts this and prints the result.

## Screenshots for the deck

Use headless Chrome, Chromium or Firefox, whichever is installed:
- `dashboard/screenshot_full.png`: the full page at 1440 px width.
- `dashboard/screenshot_hero.png`: 1920×1080, the hero plus the start of
  the plan view. This is the one the deck uses.

Commit `dashboard/` (including the assets and screenshots, not raw data),
with the message "AVIAN dashboard".

## End with this block, and stop

```
HANDOFF — DASHBOARD
Open:            dashboard/index.html (file://, offline OK)
Sections:        all 8 present? list any cut
Number check:    PASS/FAIL (n numbers checked)
Responsive:      1440 / 1024 / 390 — OK/issues
Screenshots:     paths
Assets size:     MB
Commit:          …
```
