# HANDOVER — SIH 2026 "AVIAN" prototype (Team @TRINETRA)

**Written 24 Sep 2026, 10:25 IST, for a new Claude chat taking over coordination.**
Read this whole file first, then the files listed in §4, in that order.

---

## 1. Your role

- **Coordinator, not implementer.** A separate **Claude Code session on the
  team's machine** does the coding, runs jobs and makes git commits. The
  user pastes its progress and "HANDOFF" blocks into this chat.
- **Your job:**
  - check each handoff against the committed files and the rules in §6
  - catch wrong numbers or arithmetic (this has already happened twice; see §7)
  - write the next prompt for Claude Code
  - build the final SIH deck
- **The user** is Prince (team leader). He prefers short answers, a clear
  "yes/no" when he asks for one, and one ready-to-paste prompt at a time.

## 2. Deadline and what's being submitted

- **Deadline: 24 Sep 2026, 17:00 IST.** Plan to **submit by 16:00**.
- **What's submitted:** a 6-slide PDF on the official SIH idea template.
  Judges read only the slides at this stage (see §5).
- **Remaining timeline (IST):**
  - **~11:30:** prototype frozen, `FINAL_RESULTS.json` written, git tag
    `sih-idea-submission`
  - **~13:30:** dashboard finished, screenshot saved
  - **13:30–15:00:** refresh the deck with the frozen numbers and the
    dashboard screenshot
  - **15:00–16:00:** the user reviews, exports the PDF and submits

## 3. Team and problem statement (copy exactly)

| Field | Value |
|---|---|
| Team name (portal) | **@TRINETRA** (use it with the "@" on the deck title slide) |
| Team ID | 129300 |
| Institute | Charotar University of Science & Technology, Anand |
| Leader | Prince Harshadbhai Patel |
| Members | Devanshi Sandip Patel · Kavan Shah · Pratham Shah · Anjana Nihal Amitbhai · Dev Nileshbhai Patel |
| PS ID | SIH26201 (AICTE; Ministry of Education's Innovation Cell) |
| PS title | Student Innovation-There is a need to design drones and robots that can solve some of the pressing challenges of India such as handling medical emergencies, search and rescue operations, etc. |
| Bucket / category | Robotics and Drones / Software |

**Privacy:** names only, anywhere public. Never include members' emails or
phone numbers.

## 4. Where everything is (the team's Linux machine "garudanex")

**Repo root:** `/home/prince/avian_rev_c/SIH_AVIAN_FINAL/` (git, branch
`master`).

**Environment, needed in every new terminal:**

```bash
export AVIAN_ENV_SRC=/home/prince/avian_rev_c/AVIAN_ENVIRONMENT/source
export AVIAN_COMMON_DIR=/home/prince/avian_rev_c/avian_common
cd /home/prince/avian_rev_c/SIH_AVIAN_FINAL
```

The flight stack lives in the sibling folder
`/home/prince/avian_rev_c/AVIAN_UAV/` (PyBullet). Its controller passes its
own gate 46/46.

### Read in this order

1. `README.md`: the scene (a 360 m corridor with a road bridge, a steel
   truss and a metro viaduct; 192 measured defects; 568 collision
   primitives; 1,200 bolts).
2. `SIH_AVIAN_AUTONOMOUS_MASTER_PROMPT.md`: the key concept. Stage 2
   plans from **geometry only**; ground truth is used only for scoring.
3. `SIH_AVIAN_REAL_AUTONOMY_MASTER_PROMPT.md`: sensed obstacle avoidance
   (ray-cast sensor), and the dataset bugs VD01/VD03/VD04/VD06.
4. `detection/AVIAN_detector_report_FINAL.md`: the full detector story
   across Sections 1, 2 and 3, including failures, the leak check and
   corrections.
5. `mission/AVIAN_collision_diagnosis_FINAL.md`: why collisions happen,
   and the Section 4 fixes.
6. `SIH_AVIAN_FINAL_SPRINT_MASTER_PROMPT.md`: the operating rules and
   Phases A/B/C in force right now.
7. `SIH_AVIAN_PROMPT1_FINISH_AND_FREEZE.md` and
   `SIH_AVIAN_PROMPT2_DASHBOARD.md` (including its ADDENDUM): the two
   prompts running now.
8. `SIH_AVIAN_RESEARCH_MASTER_PROMPT.md`: the research brief's scope. Its
   results were pasted in chat. The key facts used in the deck are in §8.

### Key folders

| Path | What's in it |
|---|---|
| `source/` | every script (`*_final.py`) |
| `mission/` | mission plans, flight logs, scores, collision diagnosis |
| `detection/` | detector report, weights (v1/v2/v3), eval JSONs, confusion matrices; `renders_zoom*/` are gitignored |
| `dataset/` | synthetic dataset manifests and splits (raw renders gitignored) |
| `data/` | the Özgenel real crack photos (gitignored) |
| `scene/` | `SIH_AVIAN_FINAL.blend`, ground-truth JSONs, `collision/avian_bridge_collision.json` |
| `gazebo/` | SDF world export (validated, **not used for any flight**) |
| `dashboard/` | `build_dashboard.py` → `index.html` (static, offline) plus `assets/` |
| `deck/` | `SIH26201_AVIAN_Idea.pptx` / `.pdf` (draft) and **`deck/src/`**: `build.py`, `template.pptx`, `CAM_10_TRUSS.jpg`. Rebuild with `cd deck/src && python3 build.py` (needs `python-pptx` and Pillow). |
| `FINAL_RESULTS.json` | **the single source of truth for every number.** Written at the freeze and doesn't exist until ~11:30. The deck and dashboard must use only this. |

## 5. How the prototype works (pipeline)

```
Blender scene (192 defects) ──► collision.json (568 primitives)
        │                              │
        │                              ├─► Stage 2 coverage planner (geometry only, 150 viewpoints, 8 m standoff)
        │                              │          └─► PyBullet flight: Cascade controller + ray-cast sensed avoidance
        │                              │                    + A* detours (Section 4) ──► flight_log (achieved poses)
        │                              └─► Gazebo SDF (exported only; NOT in the loop)
        └─► Blender renders zoom tiles (~8.7× optical, 9.06° HFOV, ~1.97 mm/px at 8 m) at ACHIEVED poses
                    └─► Faster R-CNN MobileNetV3 detector (torchvision, BSD-3) ──► scoring vs ground truth ──► dashboard
```

- **Stage 1** (older, ground-truth-driven, 104 waypoints) is a baseline
  only.
- **Stage 2** is the real autonomy claim: the route is planned without any
  defect data (grep-audited).

## 6. Rules to keep enforcing

1. **Measured numbers only.** If something wasn't run, write "not run",
   never an estimate.
2. **Ground truth never enters planning or tile selection.** It's used
   only for scoring. Grep gates enforce this.
3. **Split by defect instance, not by frame.** The leakage assertion must
   pass. Report "unseen defects" recall as the headline and "all defects"
   recall labelled optimistic.
4. **Headline rules are fixed before results are seen.** No
   cherry-picking.
5. **Hard stops beat completeness.** Never overwrite committed results;
   new runs get new filenames.
6. **Report sample sizes with every percentage.** Show failures as well as
   successes.
7. **Don't copy NASA's branding.** The dashboard follows NASA.gov's
   design style only: no NASA name, logo or brand colours.

## 7. Verified results so far (check them against `FINAL_RESULTS.json` once it exists)

### Autonomy

- **Stage 1:** 104 waypoints, 74 stuck (71%). Claude Code found no
  committed source file for this and is regenerating it with a full
  Stage 1 re-fly. Use the re-fly's number.
- **Stage 2, before the Section 4 fix:** 150 waypoints.
  - 67/150 stuck = **44.7%**
  - coverage **56.66%**
  - recall **39/73**
  - sensed avoidance in **239/239** log entries, **6,492** avoidance
    reactions
  - 17 respawns, 3 battery packs, worst position error 201 m
- **Section 4 diagnosis:** of the 67 stuck events, **13 were approach
  failures and 54 were transit failures**.
- **Section 4 fixes:**
  - transit margin raised from 0.05 m to **0.575 m** (airframe radius).
    The formula airframe + p95 cross-track would have been ~15.4 m, which
    made every route unsolvable; this deviation is documented.
  - segment sampling at 0.0125 m
  - A* detours, flown as intermediate setpoints
  - waypoint nudging before dropping
- **Section 4 re-fly ("after"): in progress.** It was restarted after a
  session interruption, with a go/no-go at ~10:45. If it isn't done, the
  "after" numbers are "not run".

### Detection (all on synthetic Blender images unless stated)

- **Synthetic test set** (24 held-out defects): mAP@0.5
  **v1 0.342 → v2 0.182 → v3 0.069**. This decline is real and reported
  as-is.
- **Headline model:** chosen by F1 on MISSION-VAL: **v2 at threshold
  0.65**. v2 and v3 tied at F1 0.1818; v1 was 0.0053.
- **MISSION-TEST:**
  - unseen-defect recall **1/3** for all three models
  - all-defect recall 3/20 (v1), **5/20 (v2)**, 4/20 (v3), labelled
    optimistic
  - The handoff first said "0.333 (1/21)". That was wrong and has been
    corrected.
- **False positives per 100 tiles:** v1 **37.59** → v2 **0.08** → v3
  0.04. False alarms are nearly eliminated; this is the real win.
- **Real photos** (Özgenel, CC BY 4.0, 1,000 images): **v2 made zero
  detections, precision and recall 0.** The model does not yet transfer
  to real photos. Say so plainly.
- **Leak check:** of Section 2's 10 in-scope defects, 3 were in train,
  4 in val, 1 in test and 2 in none of the splits. So Section 2's mission
  numbers were partly optimistic.
- **Bolts (FASTENER):** never detected. At the current resolution a bolt
  is only about 10 px wide; this is a declared limitation.
- **CODEBRIM rejected:** non-commercial license.
- **Zoom:** ~8.7× optical. The earlier "7.6–7.7×" was an angle ratio and
  has been corrected.

### Key commits

`063115e` · `3f2fce4` · `6f6b534` · `87b2965` · `348f807` · `f7f31d7` ·
`d76a2be` · `1596c67` · `b535a0e` · `492726c`

## 8. The deck (current draft; refresh after the freeze)

The draft is `deck/SIH26201_AVIAN_Idea.pptx` / `.pdf`, and its source is
`deck/src/build.py`. It fills the **official SIH template**: 6 slides
(Title, Idea, Technical Approach, Feasibility, Impact, References), with
the instructions slide removed and the template's pointer headings kept
word for word.

**To update:**

- `TEAM_NAME` in `build.py` is currently `"TRINETRA"`. Change it to
  **`"@TRINETRA"`** to match the portal.
- Feasibility slide:
  - "71% → 45%" becomes Stage 1 → Stage 2 **after**, if the Section 4
    re-fly completes and improves
  - the detection risk row gets the real-photo **0/1,000** result
  - add "false alarms 37.6 → 0.08 per 100 tiles"
- Technical Approach slide: add the A* detours, and the dashboard
  screenshot if space allows.

**Verified facts already in the deck** (sources checked):

- Standing Committee on Railways (Jan 2019): **1,47,523** railway bridges,
  **37,689** over 100 years old, inspection by "visual perception", about
  40% of bridge-staff posts vacant (4,517 of 7,669), so ~33 bridges per
  staff member.
- **Gambhira bridge collapse,** 9 Jul 2025: Vadodara–Anand, 40 years old,
  **22 dead**, residents had warned for years, 4 engineers suspended.
- China MoT UAV bridge-inspection guideline (Feb 2026): a **3 m minimum
  standoff**.
- Railway Board drone circular 2021/25/CE-III/BR/BMS/Drone; DGCA Drone
  Rules 2021 (R&D exemption; type certificate ~₹1.5 lakh at NTH).
- References: RCO-YOLOv5 (mAP 91.0%), CODEBRIM (arXiv 1904.08486),
  dacl10k (arXiv 2309.00460), Nature Communications 2025 (<1 in 5
  long-span bridges monitored), Flyability Elios 3.

**Framing that works:** lead with what's genuinely autonomous (a route
planned from geometry only, sensed avoidance, collisions cut). Present
detection as risk #1 with a plan: real bridge datasets, GPU training, an
engineer confirming each flag. SIH criteria reward honesty over inflated
claims.

## 9. What happens next (in order)

1. **Wait for FINAL HANDOFF — PROTOTYPE FROZEN** (~11:30). Check it
   against `FINAL_RESULTS.json`:
   - the arithmetic
   - sample sizes
   - the "after" numbers, or "not run"
   - the Stage 1 regenerated number
2. **Wait for HANDOFF — DASHBOARD** (~13:30). Check:
   - it opens offline
   - the number check passes
   - @TRINETRA / SIH26201 are shown with **no** emails or phones
   - `dashboard/screenshot_hero.png` exists
3. **Refresh the deck** (see §8), render it, visually check every slide,
   and export the PDF.
4. **The user submits by 16:00.**
5. **After the deadline (Grand Finale, Dec 2026):**
   - live flight in Gazebo + ROS 2 (+ PX4 SITL)
   - retrain the detector on real bridge photos with a GPU
   - bolt detection with 1920×1080 capture
   - a demo video

## 10. Useful commands

```bash
python3 dashboard/build_dashboard.py && xdg-open dashboard/index.html   # dashboard
cp mission/coverage_mission.json mission/demo_mission.json
python3 source/flight_final.py --mission=demo_mission.json --limit=10    # safe live demo (doesn't overwrite results)
blender scene/SIH_AVIAN_FINAL.blend                                      # view the scene
```

**Warning:** `flight_final.py` names its log after the mission file.
Never run it with `--limit` on `coverage_mission.json` or
`coverage_mission_s4.json`, because that overwrites the real results.
