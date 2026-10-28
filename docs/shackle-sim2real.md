# Shackle pin insertion: sim, then real (goal B4)

Started 2026-10-27. Part of goal B (autonomy) in
[viscous-haptics.md](viscous-haptics.md). Tick and date items; don't delete
them. Add dated lines when the plan changes.

## Gavin's intent

The part is a U-shaped (horseshoe) piece with two aligned holes, and a bolt that
goes through both. Gavin's words (2026-10-27): "all the robot arm does is pick
this up, put it into the hole, and then once it's inside the hole ... you just
rotate to tighten."

Why: "I think that there might be some in-context learning that ChatGPT 6 is
trying to do ... the key is whether I can take that and bring it to the real
world." Astra already did a flange bolt in sim and wrote its own `LESSONS.md`
from that episode (`scripts/viscous-dream1/runs/astra_flange/lessons_from_006/`).
The question here is whether what it learns in context in sim carries over to
the real arm (sim to real).

## Task setup

- The U-shape stands **vertically** in a small holder, so its two holes are
  aligned on a vertical axis: the **threaded hole at the bottom**, the
  **unthreaded hole on top**.
- The bolt starts disassembled, standing **vertically on the table**.
- The arm grasps the bolt, inserts it down through the top (clear) hole into the
  bottom (threaded) hole, then rotates it to tighten.

## Big tasks (in order)

- [ ] **1. 3D model of the part.** Codex (`gpt-6.1-sol`) builds a parametric
  CAD model from Gavin's photo and measurements. Blocked on the photo and
  measurements (2026-10-27).
  - 2026-10-27: the part is a stainless bow (anchor) shackle with an eye-head
    screw pin, roughly 3/4" (19 mm bow, ~22 mm pin). Gavin sent 14 caliper
    photos to @ViscousRobotics_bot (OneDrive "Viscous Robotics Files",
    `2026-10-27_2310*_m283..m296.jpg`; copies on gavin-linux in
    `~/dreamtrial/viscous_sim/shackle/photos/`). The pin stands upright on its
    threaded end (m288). Codex is running
    [BRIEF_SHACKLE_MODEL.md](../scripts/viscous-dream1/BRIEF_SHACKLE_MODEL.md)
    on gavin-linux (log `shackle_model_codex.log`, outputs
    `runs/shackle_model/`). Still unmeasured: the inside gap between the ears,
    the pin's total length and the thread pitch.
  - 2026-10-27, first model done (Codex, reviewed by Claude): parametric
    `scripts/viscous-dream1/shackle_assets.py`, meshes in `assets/shackle/`,
    report and photo comparisons in `runs/shackle_model/`
    (`review_contact_sheet.png`). Shape matches the photos. Codex reads m287
    (22.23) as the **thread length** and m293 (19.69) as the **pin shank
    diameter**, so its best guess is a ~20 mm pin, M20x2.5. Borrowed or
    estimated (marked `ESTIMATE` in `shackle_params.json`): ear inside gap
    41.15 mm (Suncor stainless table), pin length 107.9 mm, pitch 2.5 mm. Grip
    check: the Viscous gripper opens ~105 mm, so the pin shank, collar and tab
    and the bow stock all fit. Waiting on Gavin's measurements of the pin
    diameter, pitch, pin lengths and ear gap.
- [ ] **2. Holder.** A small printable stand that keeps the U-shape vertical on
  the table (and a bolt stand if the bolt won't stand on its own head).
- [ ] **3. Simulation scene.** Put the part, holder and bolt into the MuJoCo
  scene with our arm, with thread engagement modelled (reuse the flange work's
  thread gate and friction).
- [ ] **4. Astra does it in sim.** Codex model `gpt-6-astra` drives the arm:
  grasp, insert, tighten. Record its transcript and have it write `LESSONS.md`.
- [ ] **5. In-context learning check.** Do later episodes improve from the
  lessons alone (no weight updates)? Compare episode 1 to episode N.
- [ ] **6. Real world.** Print the holder, set up the real part, run Astra with
  its sim lessons on the real arm. Powered-run safety rules in
  viscous-haptics.md apply.
