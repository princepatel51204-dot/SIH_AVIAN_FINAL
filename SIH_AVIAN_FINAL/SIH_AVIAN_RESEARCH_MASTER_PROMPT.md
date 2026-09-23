# Research master prompt — paste this into a fresh Claude chat (no project context needed)

Copy everything below the line into a new chat. It's self-contained — that
chat doesn't need to know anything about the existing project.

---

## Context (for the researching Claude — treat this as background, not something to build)

I'm a student team building an autonomous bridge/infrastructure-inspection
drone prototype for **Smart India Hackathon 2026**, problem statement
**SIH26201** ("Student Innovation — there is a need to design drones and
robots that can solve some of the pressing challenges of India such as
handling medical emergencies, search and rescue operations, etc."),
Technology Bucket: Robotics and Drones, Category: Software.

Current state of the prototype (so you understand our starting point, not
so you critique the implementation): a Blender/Gazebo/PyBullet **simulation**
of a bridge inspection corridor (road bridge + metro viaduct + river), a
drone that flies a mission and photographs the structure, a coverage
planner that generates viewpoints from the structure's geometry alone (not
from known defect locations), and a defect-detection stage that's still a
placeholder. No real hardware yet. We want to know how to make this
**credible and competitive** against what SIH judges, mentors, and
potential funders will compare it to.

**SIH's own idea-selection criteria (from the official 2026 guidelines,
verbatim):** novelty of the idea, complexity, clarity and details in the
prescribed format, **feasibility, practicability, sustainability, scale of
impact, user experience, and potential for future work progression.**

**The mandatory idea-submission format is a 6-slide deck** (Title, Idea/
Proposed Solution, Technical Approach, Feasibility & Viability, Impact &
Benefits, Research & References) — so the research below should produce
material that can be distilled into those six buckets, especially
Technical Approach, Feasibility & Viability, and Impact & Benefits.

## What I need researched

I do NOT want you to write code or design our system. I want a research
report, organized under the six headings below, each answering the
specific questions listed — with sources.

### 1. How advanced organizations actually build this (technical benchmark)

- How does **DRDO** (or its labs — e.g. ADE, CAIR) approach autonomous
  UAV-based infrastructure/structural inspection, if there's public
  information? If DRDO-specific material is thin (likely, given
  classification), use its published UAV autonomy work (e.g. Rustom,
  Netra) as the closest public analogue for what "DRDO-grade autonomy"
  looks like architecturally (redundant sensing, GPS-denied navigation,
  certified flight controllers).
- How does **ISRO** or ISRO-adjacent work (aerial/satellite structural
  monitoring, SAR-based deformation tracking of bridges/dams) approach
  infrastructure health monitoring — even if not drone-specific, their
  standards for data validation and multi-sensor fusion are relevant.
- What do **Chinese companies** (DJI's structural inspection line, EHang,
  state-backed bridge-inspection drone programs reported in Chinese
  infrastructure ministries) publish about their autonomous bridge/
  structure inspection systems — sensor stack, coverage planning approach,
  defect-detection model type (CNN-based crack detection is common —
  which architectures, what datasets)?
- What does **Amazon** (Prime Air) publish about its autonomy stack
  (obstacle avoidance, BVLOS certification path with FAA) that's
  transferable to a non-delivery inspection drone — particularly how they
  handle real-world obstacle avoidance vs. our current ground-truth-based
  collision checking?
- Any published work from **global bridge-inspection drone companies**
  (Skydio's infrastructure product line, Cyberhawk, Sharper Shape,
  Percepto, Indian players like Garuda Aerospace, ideaForge, Marut Drones,
  Asteria Aerospace) — what's their actual autonomy level (remote-piloted
  with AI-assisted defect tagging vs. genuinely autonomous coverage
  planning), and what defect-detection accuracy do they publicly claim?
- **Synthesize a gap analysis**: specifically, what separates a
  hackathon-grade simulation prototype from what these organizations
  ship, across: localization (GPS-denied/visual-inertial), obstacle
  sensing (real depth/LiDAR vs. known-geometry collision checking),
  defect detection (trained CNN/transformer model vs. stub), and
  regulatory/safety posture.

### 2. Feasibility & viability (for that SIH deck section)

- What does India's **DGCA Drone Rules 2021 / UAS Rules** actually require
  for a BVLOS or infrastructure-inspection drone — certification tier,
  registration, no-fly zone handling near bridges/rail. This is what
  "feasibility" will actually be judged against.
- What are typical **hardware BOM costs** (in INR) for a drone capable of
  this mission profile (multirotor, ~1 kg-class camera payload, ~20-30 min
  endurance) — enough to state a believable "path from simulation to
  flying prototype" cost, not exact pricing.
- What are known **failure modes / challenges** reported by companies
  doing this commercially (GPS multipath under bridge decks, RF
  interference near metro catenary lines, wind loading near river
  crossings) — these should appear in our Feasibility & Viability slide's
  "risks and mitigation" section, since judges specifically credit teams
  who've thought about what breaks.

### 3. Impact & benefits (for that SIH deck section) — the funding case

- **Market size / need**: how many bridges in India are overdue for
  inspection or structurally deficient (MoRTH / Indian Railways bridge
  audit reports, NHAI data) — a real number carries far more weight with
  mentors than "bridges are important."
- **Cost/time comparison**: published cost and time for manual bridge
  inspection (rope-access teams, scaffold-based) vs. drone-based
  inspection, from any published case study (Indian or international).
- Any existing **government schemes or programs** this could plug into
  (Setu Bandhan, Bridge Management System / BMS under MoRTH, Indian
  Railways' bridge health monitoring initiatives) — naming a real program
  the prototype could integrate with is a concrete "scale of impact" and
  "future work progression" answer.

### 4. What SIH judges/mentors specifically reward (competitive read)

- Look for any publicly available past **SIH winning idea decks or
  winning team retrospectives** (blog posts, LinkedIn posts, YouTube) for
  Robotics-and-Drones or infrastructure-related problem statements in
  prior years — what made them stand out, specifically against the
  criteria list above (novelty, feasibility, scale of impact).
- Any advice from past SIH mentors/judges (interviews, articles) on common
  mistakes teams make in this exact 6-slide format, and what separates a
  "funded/pursued further" idea from one that just wins the demo day.

## Output format

Structure your findings under the four numbered headings above. Under
each: bullet points, each with a one-line citation (name + link). End with
a short "So what does this mean for our deck" section per heading —
translate each finding into one concrete sentence usable in the
Technical Approach / Feasibility & Viability / Impact & Benefits slides.
Don't editorialize about our specific implementation — you don't have
enough context to, and that's not what this research is for.
