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
  - 2026-10-28: Gavin's design, drawn on the 1:1 side view
    (`scripts/viscous-dream1/shackle/holder_sketch_gavin.png`): a triangular
    block under the lower outside of the bow, flat bottom on the table, the
    sloped face a cradle that hugs the bow's curve. He will hot-glue the shackle
    to it and glue or tape it to the table. The pin stands on its own threaded
    end, so it needs no stand. Codex is building it from
    [BRIEF_SHACKLE_HOLDER.md](../scripts/viscous-dream1/BRIEF_SHACKLE_HOLDER.md)
    (STL/3MF in mm for printing, plus a sim mesh), with a check of whether it
    tips when the arm pushes and twists the pin.
- 2026-10-28: Gavin wants the threads visible in the sim video ("it has to look
  nice"); how the physics models them doesn't matter. Codex is adding visual
  helical threads to the pin and the tapped ear, checked in MuJoCo's renderer
  ([BRIEF_SHACKLE_THREADS.md](../scripts/viscous-dream1/BRIEF_SHACKLE_THREADS.md)).
- 2026-10-28: Gavin asked for a 0.5 mm chamfer on every edge of the print ("I
  don't like the feeling of handling sharp edges of 3D prints") and accepted the
  loads: the shackle is hot-glued to the holder and the holder to the desk, so
  no redesign (`BRIEF_SHACKLE_HOLDER_FOLLOWUP.md`, resumed in the same Codex
  session).
- [ ] **3. Simulation scene.** Put the part, holder and bolt into the MuJoCo
  scene with our arm, with thread engagement modelled (reuse the flange work's
  thread gate and friction).
  - 2026-10-28, Gavin: "throw it into a MuJoCo table, like a proper setup ...
    ask Astra to do it. But before that, ask Dream-1 to do it, three
    attempts." Codex builds the scene, a scripted feasibility demo, the
    Dream-1 task and the Astra profile with no billed calls
    ([BRIEF_SHACKLE_SCENE.md](../scripts/viscous-dream1/BRIEF_SHACKLE_SCENE.md));
    Claude reviews it, then launches three live Dream-1 attempts (ledger
    1986/2212 before them), then Astra. The Astra lessons A/B
    (`astra_learn/ab.sh`) was running on gavin-linux at the time; the shackle
    Astra episode waits for it, because only one sim server may run.
  - 2026-10-28, scene done (`scripts/viscous-dream1/runs/shackle_scene/`):
    left Viscous arm with the Maker-style wrist camera, right arm parked,
    holder and shackle welded to the table, pin upright 18 cm away. The
    scripted (privileged, no-AI) demo picks the pin, inserts it and screws it
    8 turns to seated with 120 deg wrist strokes, so the task is doable in sim.
    Peak joint torque 11.75 Nm (J3) against the real arm's 5 Nm, so the same
    motion does not transfer to hardware as is. Open: blotchy white shading
    on the stainless (`BRIEF_SHACKLE_LOOK.md`, Codex fixing), and the parts
    are only a few pixels in Dream-1's 320x176 top view.
- [ ] **3b. Dream-1, three attempts** (zero-shot, before Astra).
  - 2026-10-28: launched `sweep.py --task shackle --episodes 3 --live
    --max-calls 30`, instruction "pick up the pin and screw it into the
    shackle"; outputs `runs/shackle_scene/dream/ep_00..02`.
  - Result: **0/3**, 90 calls (ledger 2076/2212). Dream-1 moved the gripper
    around the shackle but never grasped the pin; nothing inserted or turned.
  - 2026-10-28, Gavin changed the layout: "the bolt should be right in front of
    the arm, so when it starts the arm is already facing it. The holes should
    be closer to the arm, and the C-shaped part should be facing away from the
    arm." `BRIEF_SHACKLE_LAYOUT.md` (Codex, after the shading fix). The 0/3
    above is on the old layout (`layout_v1`).
  - Layout v2 done: pin 20 cm straight ahead of the left base (no base
    rotation to grasp), shackle ears toward the arm, bow away. The scripted
    demo still seats the pin; peak torque fell 11.75 -> 6.86 Nm (still above
    5 Nm). The white patches were shadow acne; the first fix turned shadows
    off, which looks flat, so `BRIEF_SHACKLE_ASTRA_PREP.md` asks for shadows
    back without the acne.
  - Gavin gave no answer on re-running Dream-1 on v2 before going to sleep;
    the stated default was to skip it and go to Astra (136 calls left).
- [ ] **4. Astra does it in sim.**
  - 2026-10-28: the lessons A/B (another session) holds `astra_op/server.py`
    for hours, and its runner refuses to start an episode while any
    `astra_op/server.py` process exists. So the shackle episodes get their own
    launcher (`shackle_learn/run_episode.py`) that starts `shackle_astra.py`
    directly and never matches that guard (`BRIEF_SHACKLE_ASTRA_PREP.md`).
    Episode 1 plan: flange lessons v006 as `LESSONS.md`, with a prompt saying
    they come from a different task, to see whether in-context lessons
    carry over to a new task.
- (4, continued) Codex model `gpt-6-astra` drives the arm:
  grasp, insert, tighten. Record its transcript and have it write `LESSONS.md`.
- [ ] **5. In-context learning check.** Do later episodes improve from the
  lessons alone (no weight updates)? Compare episode 1 to episode N.
- [ ] **6. Real world.** Print the holder, set up the real part, run Astra with
  its sim lessons on the real arm. Powered-run safety rules in
  viscous-haptics.md apply.
