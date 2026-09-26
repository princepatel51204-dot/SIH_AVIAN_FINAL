# PROMPT 1 of 2 — Finish Phase B and freeze the prototype

**Hard stop: 11:30 IST, 24 Sep 2026.** The deadline is 17:00 IST. The
dashboard (Prompt 2) starts right after this, and the deck after that.

`SIH_AVIAN_FINAL_SPRINT_MASTER_PROMPT.md` still applies: its operating
rules, its Phase B, and its Phase C. This prompt only adds what's pending
and moves the Phase C stop earlier. Don't start the dashboard here.

---

## 1. Finish B3: re-fly and re-score

- Complete the Stage 2 re-fly with the Section 4 fixes. Write
  `mission/flight_log_s4.json`, then run `score_coverage.py` on it.
- **Before → after**, reported whatever the result:
  - stuck/collision rate (was 67/150 = 44.7%)
  - coverage (56.66%) and recall (39/73)
  - respawns, worst position error, battery packs
  - waypoints nudged or dropped, and legs rerouted by A*
- Confirm in the report that the A* detour points were flown as
  intermediate setpoints. Paste the call site.
- If the "after" numbers are worse, say so, and keep "before" as the
  result.
- Post **HANDOFF — SECTION 4** in the format from the sprint prompt.

## 2. Two pending fixes (run in the background alongside B3)

1. **Recall arithmetic.** The Section 3 handoff says "unseen recall 0.333
   (1/21)", but 1/21 = 0.048. Give the exact numerator and denominator
   for MISSION-TEST unseen recall and all-defects recall, for v1, v2 and
   v3. Correct the report and commit.
2. **Real-photo test, one retry.**
   - Look for any saved result or log from the earlier run first.
   - If there's none, re-download the Özgenel crack set (CC BY 4.0) from
     its documented source URL into a gitignored `data/` folder, never
     `/tmp`.
   - Run v2 at threshold 0.65 as a crack / no-crack test and report
     precision and recall with counts.
   - Not done by 11:00 → "not run".

## 3. Phase C: freeze (done by 11:30)

Follow the sprint prompt's Phase C exactly:

- **`FINAL_RESULTS.json`** at the repo root. Every number carries its
  source file and commit. This file is the dashboard's and the deck's
  only data source, so structure it cleanly:

```json
{
  "meta": {"commit": "...", "tag": "sih-idea-submission", "generated_ist": "..."},
  "scene": {"corridor_m": 360, "defects_total": 192, "collision_primitives": 568},
  "autonomy": {
    "stage1": {"waypoints": 104, "stuck": 68, "stuck_pct": 65.4},
    "stage2_before": {"waypoints": 150, "stuck": 67, "stuck_pct": 44.7, "coverage_pct": 56.66, "recall": [39, 73], "sensed_entries": [239, 239], "avoid_reactions": 6492},
    "stage2_after": {"...": "from flight_log_s4.json, or null if not run"}
  },
  "detection": {
    "models": {"v1": {}, "v2": {}, "v3": {}},
    "headline": "v2", "threshold": 0.65,
    "synthetic_test": {"n_defects": 24},
    "mission_test": {"waypoints": 0, "tiles": 0, "unseen": [0, 0], "all": [0, 0], "fp_per_100_tiles": {}},
    "real_photo_test": "object or \"not run\""
  },
  "zoom": {"optical_x": 8.7, "hfov_deg": 9.06, "mm_per_px_at_8m": 1.97},
  "stack": [{"name": "...", "license": "..."}],
  "not_done_yet": ["..."],
  "sources": {"<key>": "<file>@<commit>"}
}
```

  Fill in every value from committed files. The zeros and `"..."` above
  show the shape only; never copy them in as data.

- A **"Results at a glance"** section and a **"What this prototype does
  NOT do yet"** section at the top of `README.md`.
- Commit, and tag `sih-idea-submission`.

End with the **FINAL HANDOFF — PROTOTYPE FROZEN** block from the sprint
prompt. Then stop and wait for Prompt 2.
