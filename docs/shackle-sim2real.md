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
- 2026-11-06, **freestanding holder replaces the glue design** (Gavin):
  "cant tape or glue to the tank when its filled with water", so use a broad
  base and zip ties between shackle and print. New
  `scripts/viscous-dream1/shackle_tied_holder.py`: 150 × 130 mm base, two
  top-access tie channels (6.2 × 2.8 mm), two curved saddles, 0.5 mm edge
  treatment. A floor opening preserves zero added bow height. The 75° example
  is **provisional pending the other thread's layout/feasibility result**;
  change the CAD angle before printing. STL/3MF, body-frame sim meshes, fit and
  gravity-stability evidence, images and reproduction instructions are in
  [the holder handoff](../scripts/viscous-dream1/runs/shackle_tied_holder/README.md).
  Mesh/drop-in/pin-path/tie-routing checks pass. Gravity support margin ≥64.5 mm
  dry/wet, pin present/absent; wet minimum restoring moment 0.393 N m. This does
  not establish resistance to robot torque or wet-floor sliding. No physical
  print, load trial or hardware run. Use the **new** holder and tie meshes in
  the environment's pre-print acceptance test; original scene/meshes unchanged.
- 2026-11-06, **holder redesign and pin pickup cup** (Gavin): rejected the
  first freestanding CAD as "so ugly and bad", explicitly requested Astra,
  and asked for zip-tie channels like Prusa's open-source MK3 printed parts.
  Astra is implementing a new shape with recessed guided tie routing, using
  the MK3 `y-rod-holder.scad`, `bearing.scad` and `cable-holder.scad` as design
  references (Prusa commit `902885a41340eea19c19c10d36dfc4af8e209d47`, original
  implementation rather than copied GPL source). The old 75° exports are
  superseded review material, not the selected print.
  Gavin also wants a cylindrical cup "about 3cm" to hold the disassembled pin:
  **pickup from cup → insert into shackle → spin to tighten**. Working
  assumption: 30 mm cup height, loose bore sized to the modeled ~20 mm pin,
  drainage/air escape, independent stable foot and room to grasp above its lip.
  Final angle and cup placement wait for the environment session. This replaces
  the old assumption that the pin needs no stand.
- 2026-11-06, Astra round 1 complete and parent visually reviewed: continuous
  curved cradle, tapered elliptical 146 × 112 mm foot, two enclosed annular
  tie returns with tangential mouths and locking-head pockets. Bow height stays
  unchanged; 55°, 70°, 75° and 85° builds pass fit, export and 60 mm approach
  corridor checks. **Cup revision in progress:** the orientation session found
  an upright pickup would take the wrist above the water. Astra round 2 is
  adapting the 30 mm axial cup to a near-horizontal pin on a raised pedestal,
  with configurable tilt and pin-tip height. The round-1 upright cup is not the
  task's selected pickup geometry. Final environment pose is still pending.
- 2026-11-07, **orientation session's final screening layout** (Gavin in
  flight; he has not reviewed the numbers): shackle axis 85° from vertical,
  bore midpoint 575 mm ahead of left J1, about 98.5 mm above floor, lowest bow
  on floor. Pickup cup is off the forward line: pin tip 500 mm radially from
  J1 at +15° toward sim +X, height 80 mm, pin axis 85°, head toward J1.
  The cup's 30 mm external axial length and 3 mm stop leave 27 mm inserted;
  pickup TCP is 68 mm from the tip, fingers about 6.7 mm clear of the rim.
  Cup lies beside the task line because an on-line pedestal obstructs the
  inserting hand/wrist. Astra round 2 has these values. Parent's independent
  static hand/fixture screen includes the ±80 mm palm plate and finger bounds
  through 0–120° roll; the earlier 60 mm corridor alone is insufficient.
  This is not continuous-motion acceptance: seated fingertips vs clear ear,
  complete arm/cup/holder collisions and pickup-to-insertion travel remain
  for the end-to-end simulation test.
- 2026-11-06/07, **layout for the aquarium** (orientation session; selected
  numbers not yet seen by Gavin, who was in flight). Gavin's constraints, in
  order: the bolt "at 45 degrees, with the top pointed towards the arm",
  everything on the vertical plane straight ahead of the arm (pin axis, U and
  shackle); then the shackle flipped 180° about the pin so the bow points toward
  the arm and down; then the tank: "the water level is not much higher than j3",
  set precisely as **J3's height with the J2–J3 link vertical = 283 mm** above
  the floor, no arm part above it; "elbow cannot be behind j1, as it is the
  aquarium wall" (J1 at the very edge of the long tank; wall taken at the base's
  back edge, 40 mm behind J1); the tilt "does not have to be exactly 45deg";
  "regripping is fine, otherwise kinda impossible".
  Why 45° died: the wrist pitch sits ~248 mm back along the gripper axis from
  the grip, so a 45° approach puts it ~175 mm above the pin head at any
  distance (68–124 mm out of the water). Only near-horizontal pins at long
  reach stay under. **Selected:** pin **85° from vertical**, head toward J1, bow
  on the floor, bore midpoint **575 mm** ahead of J1 at h 98.5 mm (body spans
  s 521–620 mm). Worst moment is the head regrip after the shank push, 17 mm
  under the water; 550 mm still 6 mm under, 600 mm still reachable, 625 mm
  cannot seat. Regrip plan forced by the gripper: the palm is 53.4 mm behind the
  pad TCP, so ≤ ~50 mm of the 108 mm pin may sit behind the grip (shank grip 62
  mm above the tip, push until the fingertips near the clear ear, regrip on the
  head, push, screw). **Cup:** an upright cup is impossible underwater; on the
  line in front of the shackle it blocks the insertion path, so it goes beside
  the line: pin 85°, head toward J1, tip 500 mm from J1 with J1 turned 15°
  (~129 mm to the side), pin axis 80 mm up at the tip (pedestal), pickup grip 68
  mm above the tip (fingertips clear a 30 mm cup's rim). Reachable 480–520 mm.
  The task's start pose (top 388 mm) is out of the water and needs replacing.
  Screening only (IK poses with J1 = J5 = 0, self/table collisions, no motion
  continuity, no holder/shackle/cup collisions, no torque); open check: whether
  the seated head grip's fingertips (34 mm past the pads) clear the clear ear.
  Tools: `scripts/viscous-dream1/shackle_tilt45_water_check.py` (results in
  `runs/shackle_tilt45/water_check_*.json`), figure
  `runs/shackle_tilt45/shackle_water_line_283mm_planar.png`, orientation
  sketches `shackle_tilt45_sketch.py`.
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
  - 2026-10-28 03:42 CDT: shadows restored without the acne (8192 shadow map,
    key light moved; minor jagged edges remain, re-render later for polish).
    Launcher smoke-tested beside the A/B (its guard never saw the shackle
    server). Launched episode `shackle_L006_20261028_001` (lessons v006, 240
    min wall cap); outputs `runs/shackle_astra/` on gavin-linux.
  - Episode 1 result: **failed**, 31 min, 263 commands. Astra grasped and
    carried the pin, dropped it twice during regrasps (the second time off the
    table) and ended honestly with "incomplete". Its claim that the pin
    "entered the upper ear" is not confirmed by the sim (never inserted, never
    engaged, 0 turns).
- [ ] **5. In-context learning loop** (started 2026-10-28 ~04:20 CDT, Gavin
  asleep): Astra consolidates its own notes after each attempt (flange v006 +
  episode 1 -> shackle `s001`, then s002, ...), from its transcript and what the
  sim measured, and tries again with them. Up to 3 more episodes, stop on the
  first seated success, no new episode after 10:30 CDT
  ([BRIEF_SHACKLE_LEARN_CHAIN.md](../scripts/viscous-dream1/BRIEF_SHACKLE_LEARN_CHAIN.md);
  progress `runs/shackle_astra/chain.log`, table `summary.md`).
  - **2026-10-28 05:18 CDT: Astra seated the pin on attempt 2**
    (`shackle_S001_20261028_001`, lessons s001 = its own notes after attempt 1).
    Through the clear ear, 8.04 turns into the tapped ear, collar gap
    0.0004 mm; 494 commands, 947 s sim, 55 min wall; 2 pin drops, both
    recovered; final claim matches the sim; audit clean (README, LESSONS,
    `./robot` and its own images only). Attempt 1 (flange notes only) failed
    with the pin lost off the table. The chain stopped on success and wrote
    s002 from attempt 2.
  - What this does and does not show: one failure then one success on the
    same seed is consistent with in-context learning from its own notes, but
    it is n=1. A controlled claim needs repeated runs with and without the
    shackle notes, like the flange A/B (`astra_learn/ab.sh`).
  - Videos (`BRIEF_SHACKLE_VIDEO.md`, done 05:32 CDT), a replay of the
    recorded states only, speeds labelled:
    `runs/shackle_astra/shackle_S001_20261028_001/video/` (`shackle_social.mp4`
    ~49 s, `shackle_social_vertical.mp4`, `shackle_autonomous_10x.mp4`,
    `hero_seated.png`). Summary and social cut sent to Gavin's Telegram.
  - Next (for Gavin): his measurements (pin diameter, pitch, pin lengths, ear
    gap) and rebuild; repeat runs for a real in-context-learning claim; the
    torque gap (scripted peak 6.86 Nm vs the real 5 Nm) before task 6.
- (4, continued) Codex model `gpt-6-astra` drives the arm:
  grasp, insert, tighten. Record its transcript and have it write `LESSONS.md`.
- (5, original line) **In-context learning check.** Do later episodes improve from the
  lessons alone (no weight updates)? Compare episode 1 to episode N.
- [ ] **6. Real world.** Print the holder, set up the real part, run Astra with
  its sim lessons on the real arm. Powered-run safety rules in
  viscous-haptics.md apply.
- [ ] **7. Fine-tune Dream-1 on the shackle task** (Gavin, 2026-11-06): "dream1
  does not support my arm yet. can we fine tune dream1 to make it work for this
  simulated task ... assume that this is in air rather than water first."
  - Why a fine-tune: with any arm but YAM the public checkpoints read only the
    images (state withheld), which is enough for apple-to-bowl but gave 0/3 here.
    The installed client (`dreamlabs_client` 1.0.3) now does the whole loop
    ourselves: `add_embodiment` (our state/action spans and cameras),
    `validate_dataset` (local, free; LeRobot **v2.1** only), `upload_dataset`,
    `finetune(..., use_eef=True, lr_schedule="cosine")`, then
    `policy({"checkpoint": "dream-1-ft:..."})`. `use_eef` needs both arms' six
    EEF parts in YAM's pose convention; its docstring warns "You may not get
    high reliability on specialist tasks yet."
  - **Real first, then sim** (Gavin agreed 2026-11-06): set up the arm, cameras
    and shackle in the aquarium (dry first = "in air"), measure it, then build the
    sim to match, because Dream-1 lives or dies by what its cameras see (the
    wrist-camera finding in viscous-haptics.md). Building the sim first would
    mean regenerating every demo once the real setup differs.
  - Torque: Gavin, 2026-11-06, "dont worry too much about the torque limits";
    log peak torque, gate nothing on it.
  - Tonight (layout-independent): ChArUco camera target
    (`scripts/viscous-dream1/shackle_real/charuco_target.py`, 7x5 x 30 mm, A4 and
    Letter landscape, print at 100% and check the 100 mm bar, laminate); a
    closed-loop privileged expert that can re-plan from any state, with the
    scene's dimensions in one config; a sim -> LeRobot v2.1 writer checked with
    `validate_dataset`; a v3 -> v2.1 converter for real Lab recordings; a staged
    evaluation harness (grasped, inserted, engaged, turns, seated). No uploads
    and no billed calls until Gavin says so.
  - 2026-11-07 (hackathon day) capture at the dry tank: positions of holder,
    pin start, tank walls and floor and planned water line relative to the arm
    base (photo with a ruler); one frame per camera with the ChArUco board at a
    measured spot; camera models/resolution and the wrist mount; reference
    frames at the start pose; the tool point with `dreamcli tcp`; optionally
    20-50 dry teleop demos.
  - **Gate before Gavin prints the holder (morning of 2026-11-07):** once the
    other thread's tilted-layout environment is done, Claude runs one
    feasibility test of it: the printed holder STL is the sim's holder mesh;
    the pin clears both ears; IK reaches every task pose (grasp, lift,
    align, insert, every tightening stroke, regrasps) inside the left mount's
    joint limits; no arm/gripper collision with holder, shackle or tank; the
    scripted demo seats the pin end to end; pin and shackle visible in the
    overhead and wrist cameras; arm top vs the water line reported.
  - 2026-11-07, gate spec updated from the orientation and holder sessions
    (task 2's 2026-11-06/07 entry has the layout): use ONLY the holder
    session's final 85 deg assets (curved cradle, ties, tilted cup on its
    pedestal) at the paths it gives, never `assets/shackle/shackle_holder*` or
    `runs/shackle_tied_holder/tilt_75deg`. Placement: shackle bore s 575,
    h 98.5, pin 85 deg; cup pin tip 500 mm from J1 at J1 +15 deg (toward sim +X),
    80 mm up, pin 85 deg. Whole arm, end to end: the 283 mm water line at every
    step, the wall 40 mm behind J1 (elbow ahead of J1), a new start pose under
    the line, both regrips, and an explicit check that the seated head grip's
    fingertips (34.3 mm ahead of the pad TCP) clear the clear ear's outer face.
    `shackle_fixture_clearance.py` (holder session) only screens the static hand
    against the fixture; it is not this gate. Start only on Gavin's word or the
    holder session's CAD push.
  - Open: the layout (upright v2 vs 45 deg vs ~75 deg; the other thread found
    45 deg puts the wrist above J3's height whatever the distance); overhead
    camera above the water or housed; call-ledger cap for evaluation (~110
    calls per 4.4 min episode); fine-tune cost (ask Ryan).
