# SIH_AVIAN_FINAL — Detection Pass

**Scope change: repair is out. Detection is the whole product.**
**Steel truss main span · bolted connections · torque match marks · condition gradient.**

This file is the message pasted into Claude Code to start this pass, kept
here per the established convention (see `SIH_AVIAN_FINAL_MASTER_PROMPT.md`
at the project root for the original build).

## Locked pre-flight decisions (2026-09-22)

1. **Truss type: Warren with verticals.** Most gusseted joints of the three
   options considered, which is the point of the pass.
2. **The concrete main span is replaced, not kept as a second structure.**
   Matches §1's own table; keeps scope contained.
3. **Scanner sensor stays base RGB for this pass** (no zoom/macro camera).
   `min_detect_range_m` is measured against the stated 2.1478 mm/px @ 1 m
   GSD only; a payload change is explicitly a Phase 2 decision.

## What follows is the original prompt, verbatim

> **SIH_AVIAN_FINAL detection pass. Read `SIH_AVIAN_DETECTION_MASTER_PROMPT.md`
> in full. §3 of `AVIAN_REV_C_MASTER_PROMPT.md` still binds, including §3.10.**
>
> **The project is now detection-only.** No repair, no nozzle, no certification of
> repairs. The environment must support finding and grading damage — cracks,
> loose bolts, corrosion, cable and bearing faults — and measuring what a camera
> can actually resolve at inspection range.
>
> This is a large change: new steel geometry, ~1,200 fasteners, a new defect
> taxonomy, and a re-frozen baseline. Work through §8's checkpoints in order.
>
> Before writing code: give me **A–F** and only the blocking questions.

See the original pasted prompt (sections 1–11) for the full detail: steel
through-truss geometry (§2), fasteners and torque match marks (§3), the new
defect taxonomy (§4), the resolvability measurement (§5), the condition
gradient (§6), validation (§7), checkpoints (§8), cameras (§9), and the gate
(§10).
