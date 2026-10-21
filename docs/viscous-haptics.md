# Viscous haptic teleoperation: objective, reasoning, progress

A running record of **why** this work exists and how decisions were made, so the
history can be reconstructed later. Newest results at the bottom of each section.

Everything here is a subtask of a larger goal. Each one is recorded with its
objective, **Gavin's intent** (why it matters, in his framing), what it unblocks
and where it stands. Quotes are Gavin's words from the working sessions. When a
subtask changes direction, add a dated line rather than rewriting the old one.

## Goal tree

**Mission: make the Viscous arm something ROV operators want.** Two branches:
pilot teleoperation that feels right, and autonomy through learned policies.
The 2026-10-19 session opened with autonomy ("I'm trying to make the viscous arm
autonomous") and asked for the Star leader to follow the follower for DAgger.
Gavin then reprioritised.

### A. Pilot teleoperation with force feedback (current priority)

- **Objective:** a pilot driving the Viscous with the Star leader feels what the
  follower touches.
- **Intent:** "a lot of the ROV pilots, people in the industry, kind of want force
  feedback or some haptic feeling" (2026-10-19). DAgger "is not my most important
  thing right now; my most important thing right now is teleoperation."
- **Guiding rule:** the pilot should feel contact, not the arm's own effort.
  "Lifting itself up because of its own weight ... is not something that is
  meaningful for the pilot to feel. If we hit something, yes, we should feel that."

**A1. A clean contact signal.** contact torque = measured motor torque − predicted
gravity torque.
- *Intent:* the haptic effort must exclude self-weight (the guiding rule above).
- *Unblocks:* A3. *Status:* validated live on unit 1 dry, 2026-10-20 (see
  "Contact-signal bench check"): a hand push reads ~2 N m against ~0.1 N m
  still noise. Free-motion floor ~0.6-1.2 N m p99 still to bring down.

**A1a. Gravity model, measured on the arm.**
- *Intent:* not what gravity compensation was for the Metal arm. "Before,
  gravity compensation for Ethan was mainly so that the leader arm would not
  feel so heavy. That's not really the case for us because our leader arm is
  very light." Here it exists to separate contact from load.
- *Method intent:* "Dry arm first, and once we've perfected this method, then we
  can do it for the wet one." The dry bench proves the method; the aquarium fit
  (buoyancy) is the one that matters in use.
- *Status:* dry fit done 2026-10-20 (results below). Wet fit pending.

**A1b. Gravity feedforward on the follower.**
- *Intent:* a side benefit, not the goal. Removing sag makes position error mean
  contact, and the follower goes where it is told.
- *Status:* A/B done 2026-10-20: sag 3.5-5.5 deg at 0 % falls to under 0.25 deg
  at 100 % on every gravity joint.

**A2. A position-drivable leader.** The Star's servos must be commanded toward
the follower with capped effort (`star_follow.py` draft).
- *Intent:* serves A3 and, later, DAgger (B1): one piece of hardware work for both.
- *Status:* draft code; Star actuation not yet characterised on hardware.
- *Protocol research (2026-10-20, [star-servo-protocol.md](star-servo-protocol.md)):*
  FashionStar position commands carry a per-command power limit in mW, but
  motorbridge 0.0.4's `set_angle` always sends power 0, which means the servo's
  protection CEILING, not zero effort. So `star_follow.py` as drafted drives
  the leader at full power: do not run it against a hand. A capped move needs
  the raw command (0x0D multi-turn angle + power) or a motorbridge patch;
  hold (0x11) cannot move its hold point. A zero effort gate must be an
  explicit release (0x10), never power 0. No documented bus-loss watchdog: the
  last target persists if the host dies, and stop writes are not acknowledged.
  Monitor current/power are unsigned magnitudes (30 mA / ~365 mW at rest,
  released), so they are not a signed torque reading.

**A3. Input-gated bilateral loop** (approach section below).
- *Intent:* the "spring between leader and follower" Gavin described, made to feel
  light in free motion and firm on contact. Chosen after checking how others do it.
- *Status:* not started; needs A1 and A2.

**A4. 1:1 posture mapping.** A horizontal leader gives a horizontal follower.
- *Intent:* pilots expect the follower to mirror their pose. It is also needed to
  judge the rest by eye: "When I hold [the wrist] horizontal, the follower arm is
  not horizontal, so then I can't really tell whether the gravity compensation
  is working."
- *Status:* waterproof unit trial "worked very well" (committed 2026-10-20).
  Unit 1 has a draft profile in viscous-arm-simulation, not yet in Lab.

### B. Autonomy (deferred, not abandoned)

**B1. DAgger handover.** The leader tracks a policy-driven follower so the
operator can take over without a jump. Needs A2, plus CAN-safe go-limp and
restore paths in the DAgger runner.

**B2. Policy data and rollouts** (ACT/SmolVLA, `scripts/record-rollouts.sh`).

**B3. Dream-1 (DreamLabs' hosted world action model) on the Viscous arm.**
- *Intent:* the Viscous arm is very close to the Maker Arm, which Dream-1
  officially supports, so Gavin wants Dream-1 to drive it, possibly through "a
  translation layer ... so that ... the Dream One model just pretends that it is
  ... using the maker arm but then in reality it's using my arm" (2026-10-20).
- *Status (2026-10-20):* zero-shot MuJoCo trial done; the result is "completely
  stupid ... hopelessly bad, no resemblance of any meaningful actions". Gavin wants
  to find out why. See "Dream-1 trial" below.
- *Status (2026-10-20, later):* cause found. Dream-1 never sees a non-YAM arm's
  state, and from images alone it does not aim at the object when the arm is the
  Viscous (it does on YAM). The Viscous IK and frames are fine. The task (2 cm white
  cube) also fails even on YAM. Next: a Maker-tuned checkpoint or a Viscous fine-tune,
  after Gavin sets the tool point with `dreamcli tcp`. Details under "Findings".

### P. How the work is done (process intent)

- **Nothing lost.** Commit and push across both machines. The repo must be
  self-contained: "If I have a brand new machine, I can just get everything from
  GitHub and then it's all there."
- **Record intent, not just objectives.** "Record down the intention of the
  objective ... so that when we go back in history ... we can see the thought
  process." This document is that record.
- **Hardware runs:** say plainly which joints move. Prove the stop works before
  energizing. Gavin keeps the power switch as the hard stop. Gavin accepts
  motion that is surprising but not dangerous: "it actually was not doing
  anything dangerous, just surprised me." He set the thermal ceiling to 85 °C,
  the driver's own stop, so scripts abort just below it.

## Objective (haptics detail)

ROV pilots teleoperating the Viscous arm with the purple Star Arm 102-HD leader
should **feel** what the follower touches (contact, collisions, grip) through
the leader, the way industry pilots ask for "force feedback / haptic feel".

What they should *not* feel: the follower holding up its own weight, or water
drag that carries no information. That distinction drives most of the work below.

Priority set on 2026-10-19: teleoperation and haptics first. DAgger handover
(the leader following a policy-driven follower) is deferred; it needs the same
leader drive, so that part is shared groundwork.

## Approach and why

**Bilateral control family.** Options considered (2026-10-19 literature check):

| Scheme | Needs | Feel |
|---|---|---|
| Position–position (virtual spring between arms) | position-controlled actuators | stable, but heavy/sticky: you feel the follower's inertia, friction, sag |
| Force reflection | torque-controllable leader + force estimate | good |
| 4-channel (Bi-ACT, ALPHA-α, Yamane et al.) | torque motors at ~1 kHz + dynamics model | best |
| **Input-gated bilateral** (Kanai et al., arXiv 2509.08226) | position servos only | light in free motion, firm on contact |

Chosen: **input-gated bilateral**. The Star's FashionStar servos accept position
commands (`set_angle`) and stop modes (0x10 free, 0x11 hold, 0x12 damping) but
no torque mode, which rules out force reflection and 4-channel on this leader.
Input-gating drives the leader toward the follower's position but caps the
leader's effort by the follower's *contact* effort, which fixes position–position's
heaviness. It was shown on Dynamixel position servos at 100 Hz with default gains.
(A gravity-compensated, torque-capable leader such as the Metal leader would
allow full force reflection later.)

Correction recorded: `arms/can_common.py` says the Star leader has "encoders and
no motors". That is wrong for the 102-HD: its lock button holds the pose with
the servos, and `set_angle` positions them.

On unit 1's Star (gavin-linux) the lock button is dead (Gavin, 2026-10-20: "the
button is long dead"): id 7 does not answer a ping, so leave
`button.enabled` at its default `false`. Servos 0-6 answer at 1 Mbaud. Holding
or driving the leader has to be commanded over the bus (stop modes,
`set_angle`), never through the button.

**Why a gravity model first.** The follower's effort signal is RS00 motor
torque, which contains gravity, friction, drag and contact. The pilot should feel
only contact:

    contact torque = measured motor torque − predicted gravity torque(q)

So gravity compensation here is *not* what it was for the Metal arm (making a
heavy leader feel light; Ethan's Pinocchio model, Isaac's gravity-compensated
Metal leader). Our Star leader is already light. For us the model (1) cleans
the haptic signal and (2) as feedforward, removes the follower's sag so position
error means contact.

**Pinocchio, not hand-rolled numpy.** Proven URDF parsing (fixed and mimic
joints, frame conventions), the joint-torque regressor needed for
identification, and Coriolis/inertia terms for later. Its cost is no Windows
wheels, so it is the optional `arm-dynamics` extra, imported lazily.

**Measure, don't trust CAD.** The CAD URDF's masses are estimates without
cables, screws or potting, and the Metal arm's model ended up with hand-copied
1.1–1.2× factors. Gravity torque is linear in each body's mass and first moment,
so a least-squares fit to torques measured at static poses, pulled toward CAD
only where the data can't decide, gives numbers measured on this arm. Each pose
is approached from above and below so friction cancels. Air first (unit 1, dry
bench) to prove the method; then the aquarium, where the same fit absorbs
buoyancy.

## Results (2026-10-20, unit 1 `viscous_01`, dry bench)

- **RS00 torque telemetry is trustworthy**: it agrees with stiffness × error at
  slope 0.98–1.00, r = 1.00. (The Metal arm's Damiao telemetry was not usable.)
- **CAD is already close**: measured/CAD 1.05 shoulder, 0.98 elbow, 0.95 wrist.
  With 27 holds the fit beats CAD on held-out poses (shoulder 0.33 → 0.23 Nm
  RMS); elbow and wrist are about equal to CAD.
- **Static friction**: ~0.11 Nm shoulder, 0.04 elbow, 0.07 wrist. This and the
  ~0.2 Nm model error set the floor for contact detection.
- **Feedforward works**: at 50% in GUI teleop, the elbow's stiffness torque at
  the fold fell to under half its gravity load, matching the model. Sign is right.
- **The dry shoulder is overloaded**: 33 → 65 °C in under 2 min at 5–6 Nm; and
  moving between two heavy poses (~5.6 Nm predicted, after it had held 6.5 Nm)
  it lost its hold and the arm dropped until Gavin caught it. No fault logged;
  probably thermal derating or a protective cutout. The RS00 is rated ~5 Nm
  continuous and **14 Nm peak** (Gavin, 2026-10-20: "max torque is 14nm"), so
  the drop was not the motor's torque limit; the dry-arm 3.5 Nm planning cap is
  a continuous-load and heat caution, not a ceiling the motor imposes. The arm
  is all-RS00 because it is designed to be near-neutral in water.

## Safety lessons (apply to every powered run)

- State every joint that will move before energizing (J5/J6 moving surprised
  Gavin when only "the vertical plane" was described).
- Prove the stop works before energizing. A process started with `nohup … &`
  ignores SIGINT unless it installs its own handler; the first "stop" did nothing.
  Check liveness with `kill -0 <pid>`, never `pgrep -f` inside the same ssh command.
- Dry arm: cap planned load ≈3.5 Nm, trip at ~10° lag (25° was far too late),
  recover toward the low-load upright pose, cool before heavy poses, log per-tick
  torque and motor status.
- Releasing torque at the fold tips the dry arm forward ~25° onto its gripper
  (every stop, including Lab teleop's). Not yet fixed. Cause (2026-10-20, from
  the fit): the elbow is fine, gravity pushes the forearm INTO its fold stop
  (~4.1 N m the stop carries). `shoulder_lift` sits just past balance at the
  fold and gravity pulls it AWAY from its stop (0.2 N m, 0.6 N m by -25 deg),
  so the arm rolls forward until the gripper lands. Gravity never holds the
  shoulder in the fold; the fix is to lower the arm under control onto where it
  ends up (gripper on the bench) and release there.

## How to use what exists

- Gravity feedforward in teleop (Linux service drop-in or environment):
  `MAKERMODSLAB_GRAVITY_FF=configs/viscous/viscous_01/gravity-air-fit.json`,
  optional `MAKERMODSLAB_GRAVITY_FF_SCALE=0.5`. Off when unset.
- Identify another arm/condition: `scripts/viscous-gravity-collect.py --robot NAME
  [--condition water] --dry-run`, then without `--dry-run`, then
  `scripts/viscous-gravity-fit.py <runs…>`.
- Code: `makermodslab/arm_gravity.py` (model, fit, planner, collision checker),
  `makermodslab/gravity_feedforward.py`, `makermodslab/star_follow.py` (draft
  leader drive), model in `configs/viscous/model/`, data in `configs/viscous/viscous_01/`.

## Next steps

1. ~~Contact-torque signal live~~ and ~~feedforward A/B~~: done 2026-10-20
   (see "Contact-signal bench check"). Friction term + refit done offline
   (`gravity-air-fit-v2.json`) and confirmed live the same day; v2 is the
   contact check's default. Remaining floor: shoulder model error at load.
2. Feedforward at 100 % in GUI teleop on unit 1 (the drop-in is still at 50 %).
3. Characterize the Star as an actuator: power-capped moves (does the protocol's
   power field work through `set_angle`?), damping levels, latency, feel.
4. Input-gated leader drive built on `star_follow.py`.
5. 1:1 posture mapping for unit 1 (draft profile in viscous-arm-simulation
   `leader/mapping/profiles/viscous_01.json`).
6. Water fit in the aquarium (waterproof unit).
7. Stop path: avoid the tip-forward on release at the fold.

## Dream-1 trial (2026-10-20, MuJoCo only, goal B3)

**Where it ran:** gavin-linux, scratch venv `~/dreamtrial/.venv` (Python 3.12,
`dreamlabs==1.0.1`, `dreamlabs_client==1.0.3`), API key in
`~/.dreamlabs/credentials` (0600, `DREAM_API_KEY=`; never commit it). The key was
pasted into a chat during the trial: revoke and replace it. The trial's code,
briefs, reports and the two live runs' `log.json` + `run.mp4` are committed in
[`scripts/viscous-dream1/`](../scripts/viscous-dream1/). The script reads the
scene from `~/Documents/viscous-arm-model/demo/pick_place_scene.xml` and
`viscous_arm.json` points at that repo's URDF by a relative path; run it from
`~/dreamtrial/viscous_sim` or fix those paths. Modes: `--preview` (no client),
default dry run (client path with a local stub, no network), `--live` (billed,
capped by `--max-calls`); `--rig bench|yam`, `--start neutral|work`, `--model beta|flash`.

**How Dream-1 handles robots (read from the installed client, not their website):**
- The client's `robot` setting takes only `"yam"` or `"other_embodiment"`
  (`dream/client.py:68-72`). The checkpoints the key may run are only
  `dreamlabs/dream-1-beta` and `-beta-flash`, both `embodiment: yam`, cameras
  `overhead`, `wrist_l`, `wrist_r`.
- On `"other_embodiment"` the model gets zeros in its joint slots. It reads the
  cameras, the prompt and each arm's hand pose (`state.<side>_eef_xyz`, `_eef_rot6`,
  grip), and answers hand poses in 48-row chunks at 20 Hz. The client's own IK
  (`dream_ik`) solves them onto the arm.
- "Maker support" is client-side: `ik: {"arm": "makers"}` selects
  `dream_ik/specs.py` `MAKERS_ARM` (`maker_arm_ik.urdf`, tool point 0.1696 m along
  `link_007` -z relabelled into YAM axes, rest pose from 94 Maker episodes, camera
  mount "not measured", no joint mapping to YAM). `"metal"` (`METAL_ARM`) is the
  same plus a YAM joint-sign map and "our Metal policy" tuning. With the public
  YAM checkpoints, Maker and Metal are `other_embodiment` exactly like Viscous, so
  disguising Viscous as a Maker changes nothing the model sees. It only matters
  with a Maker- or Metal-fine-tuned checkpoint ("Metal and Makers checkpoints read
  the URDF's joints"), which this key cannot run. Ask Ryan/DreamLabs for one.
- Today's runs used `ik.arm` = `viscous_arm.json` (tool point `tool0`, the
  finger-pad midpoint, never confirmed with `dreamcli tcp`), head `eef`.

**Results (both on beta, 79 billed calls, about 10¢):**
1. `runs/live_beta_1`, bench rig (overhead camera beside the arm, ~0.28 m up;
   `wrist_r` black): 40 calls. Cube never moved; the arm swung away then rose to
   z 0.57 m; up to 13 of 30 executed rows per chunk clipped by the 2 rad/s cap.
2. `runs/live_yamrig_1`, Viscous placed where YAM's left arm sits in Dream's own
   station, YAM's calibrated overhead camera (`yamrig_report.md`): 39 calls. The
   gripper opened and hovered 13-23 cm above the table; cube never moved.

**Why it might be failing, to check in this order (2026-10-21):**
1. *Baseline:* run Dream's own `dream_examples/yam_mujoco` (installed in the
   venv) on a similar pick-and-place. If YAM also flails, the fault is in how we
   call the API, not the Viscous arm.
2. *Start state out of distribution:* both live runs used `--start neutral`
   (all joints 0, gripper closed); YAM resets with its gripper open, and
   `--start work` (the arm file's rest, gripper open) was never run live.
   Compare the hand pose we send at step 0 with what `yam_mujoco` sends at its reset.
3. *Hand-pose frame:* base-local EEF with no world transform, and the
   tool-point axes (`approach`, `jaw` in `viscous_arm.json`) never checked
   against YAM's convention (z = approach, x = jaw travel). A wrong frame makes the
   model see a nonsense hand pose and its answers solve to nonsense.
4. *Images:* Viscous meshes look nothing like YAM; the cube is ~5 px wide in the
   320x176 overhead; wrist camera mount differs from YAM's.
5. *Execution:* 30 of 48 rows run per chunk, speed clipping, IK fallbacks.
6. Only after 1-5: zero-shot on an unseen arm may simply not work. Then the
   options are a Maker/Metal fine-tuned checkpoint, or ~50 Viscous demos and
   Dream's fine-tune service (LeRobot v2.1 only; we record v3.0, so convert).

**Findings (2026-10-20, second session).** Gavin chose to work B3 in its own
session, in parallel with a haptics session.
- *The model is blind to our arm's state, by design.* For any arm but YAM the
  client sends `robot: other_embodiment`, and the serve then "withholds the whole
  state, joints, hand pose and grip alike: the model reads none of it, and the hand
  pose sent is only the anchor its answer is composed on" (`dream/client_ik.py`
  docstring; `dream/__init__.py` on `robot`). On the Viscous, Dream-1 acts from
  the three images and the prompt alone. Dream's own `yam_mujoco` example never
  uses this mode: for beta it sends the YAM station keys and the model reads the
  joints. So the earlier comparison with "YAM works in Dream's sim" was not like
  for like.
- *Hypothesis 3 (hand frame) ruled out* with dream's own FK
  (`scripts/viscous-dream1/frame_check.py`): with the hand straight forward,
  our arm file gives approach = +x and jaw = +y (left) in base, the same as YAM
  and Makers and as Dream's convention requires. `--start neutral` puts the hand at
  (0.22, 0, 0.13) m pointing forward 9° down, near YAM's rest (0.26, 0, 0.21) m.
- *Hypothesis 1 done properly, and the vision-only mode is NOT the problem.*
  `scripts/viscous-dream1/yam_baseline.py` runs Dream's YAM scene ("put apple
  into the bowl") in two modes: `station` (the model reads the state) and
  `vision` (`other_embodiment`, client IK: the Viscous path). 20 billed calls
  each, beta. Both succeed: the left arm grasps, lifts and drops the apple in the
  bowl (closest 0.008 m station, 0.004 m vision; vision did it by call 6-7, station
  by 8-9). Evidence in `scripts/viscous-dream1/runs/yam_{station,vision}_live1/`.
  So Dream-1 can act from images alone, and something specific to the Viscous
  trial broke it: the arm's appearance, the task/scene (white cube into a black
  square), or our executor (30 of 48 rows, 2 rad/s per-row clip, gripper
  closed at start).
- *The 2x2: both new cells fail, so there are two separate problems.*

  | | apple -> bowl (Dream's scene) | white cube -> black square |
  |---|---|---|
  | YAM | works (`yam_vision_live1`) | fails (`yam_cube_vision_live1`, 20 calls) |
  | Viscous | fails (`viscous_apple_vision_live1`, 10 calls) | fails (earlier runs) |

  (1) *The task:* even YAM never goes for a 2 cm white cube; it wanders around
  the bottle and orange. The cube is a few pixels in the 320x176 top image.
  Dream-1 zero-shot needs objects and prompts it knows. (2) *The arm:* the
  Viscous, in Dream's scene with Dream's prompt and the baseline's executor (48
  rows, no extra speed clip, grippers open), rises near its base and never touches
  the apple. A scripted Viscous pinch does lift the apple 7.8 cm, so the gripper
  can do it. Call 10 answered a grip of -0.089, which the example's check rejects
  (clip it instead next time). Scene builder: `cross_scenes.py`; report
  `runs/2x2_report.md`.
- *Next: why the arm fails.* Either the model is confused by how the Viscous looks
  (it aims wrong), or it aims right and the Viscous cannot reach the hand poses
  it asks for (the IK compromises, pulled toward `rest`). The runs logged where the
  hand went, not the poses the model asked for, so log the reply's hand poses and the
  IK's tracking error, then replay YAM's successful hand path through the Viscous IK
  offline (free).
- *Verdict: the model aims wrong; the Viscous reaches fine* (`runs/ik_reach_report.md`,
  15 billed calls per arm, checked against the raw `chunks.jsonl`). On YAM the
  requested left-hand path goes straight to the apple by call 3-4 (within 7 mm),
  closes, lifts, carries and opens over the bowl. On the Viscous, with the same scene,
  prompt and executor, the requests dither around the start for five calls, then
  drift up and away, never closer than 19 cm. The Viscous IK follows those
  requests to 6.7 mm / 1.1° median, as close as YAM's own (4.9 mm / 0.7°), with no
  joint at a limit when it matters. So with the state withheld, what the model sees
  of the Viscous (its look in the top camera, and/or its wrist camera views) makes
  it act without purpose. Nothing in our kinematics, frames or executor causes it.
- *A second, smaller problem: the tool point.* Replaying YAM's successful hand path
  on the Viscous (no calls), the commanded joints track it to ~5 mm, but the Viscous
  fingers hit the table on the way down. The real hand ends ~8 cm off and only
  nudges the apple. The arm file's `tcp` is the finger-pad midpoint; Dream defines it
  as "the point between the closed fingertips", and says a person sets it on the real
  gripper with `dreamcli tcp`, never an agent. Gavin to do that before any model that
  does aim right (a fine-tune, a Maker checkpoint) is run.
- *What this means for B3 (2026-10-20):* zero-shot Dream-1 will not drive the
  Viscous; disguising it as a Maker arm changes nothing with the public YAM
  checkpoints, because the model reads images, not the arm's identity. The realistic
  routes are (1) a checkpoint that has seen a Maker-like arm (ask Ryan/DreamLabs for
  the Maker fine-tune; Gavin's translation-layer idea fits that checkpoint) or (2)
  ~50 Viscous demos through Dream's fine-tune service (LeRobot v2.1; we record v3.0).
  Either way, use objects and prompts the model knows. Optional cheap diagnostic
  (~30 calls): black out the wrist cameras on both arms to see whether the wrist
  view or the top view of the Viscous causes the confusion. Total billed today: 100
  calls (40 baseline, 30 2x2, 30 reach), about the same per call as 2026-10-20's ~10¢
  for 79.
- *Object size, confirmed (2026-10-20, Gavin's idea).* Gavin guessed Dream-1 was
  not trained on objects as small as the 2 cm cube and put the real Feetech servo
  box (about 5 x 5 x 3 cm, white top, black sides; `real/topdown_box_20261020.jpg`)
  on the bench beside it. The same box in Dream's scene, YAM, vision-only mode,
  "pick up the black and white box and put it on the black square": the left arm
  grasps it at about call 6, lifts it 17.6 cm, carries it and sets it on the square
  by call 13 (centre about 2 cm from the square's centre; the 5 cm box covers the
  4.5 cm square). Later calls nudge it a little. 20 billed calls,
  `runs/yam_box_vision_live1`, `runs/box_report.md`. So on a real bench, use
  objects of about this size or larger for any Dream-1 trial; 120 billed calls today.
- *Maker and Metal in sim also fail (2026-10-20).* Gavin: Dream supports the Maker
  arm ("works 60% of the time") and the Metal, so follow their documented setup in
  sim, Maker first, then Metal. Dream's docs are behind the platform login; the
  client is the documentation we have. This key runs only the two public YAM base
  models, so "Maker support" for us is `dream-1-beta` + `ik: {"arm": "makers"}`
  (`other_embodiment`). Two Maker arms at YAM's bases in Dream's scene, built from
  our `frontend/public/maker-urdf` and checked against `dream_ik`'s own Maker FK to
  0.0003 mm / 0.00003 deg; scripted pinches lift the apple and the box. Live, beta:
  Maker apple 20 calls, Maker box 20, Metal apple 20. No object contact in any run;
  the requested hands never came within ~23 cm of an object (Metal 30 cm). Report
  `runs/maker_metal_report.md`, scripts `supported_arms.py`,
  `supported_arm_check.py`, `analyse_supported_runs.py`. 180 billed calls today.
  Two open questions decide what this means: (1) the wrist cameras. Dream calls the
  Maker mount "not measured", and our improvised one is clearly wrong: it looks at the
  back of the gripper with the table upside-down (`runs/maker_metal_cameras/`), where
  YAM's shows the scene with fingertips in the corners. (2) Whether Dream's "60%"
  is this base model zero-shot or a Maker fine-tune. The Maker spec's rest pose comes
  from 94 recorded Maker-station episodes, and the client speaks of "Metal and Makers
  checkpoints", which suggests fine-tunes this key cannot run.
- *It was the wrist camera: the Maker works once its camera matches the real one
  (2026-10-20).* Gavin pointed at the MakerMods brain: every Maker and Metal box ships
  the KD-USB Sonix UVC camera (`2C7F:4A60`, 1080p, fisheye; undistorted HFOV 109.6°
  / VFOV 77.2°, best focus ~0.2 m; mount never measured). Real Maker wrist frames
  from two Maker datasets in gavin-linux's Hub cache (`real/makerwrist_*.jpg`) show
  the camera above the gripper, behind the jaws, looking forward along the fingers,
  which rise from the bottom centre. Ryan also confirmed (2026-10-19) that Maker data
  trained Dream-1. With the sim camera on the CAD camera assembly on `link_007`, 20°
  down, VFOV 77.2°, 16:9 (`runs/maker_wrist_match/compare.png`), beta + `ik makers`
  zero-shot: Maker apple succeeds (apple in the bowl at call 12 and stays there);
  Maker box grasped, lifted 12.9 cm and carried to 1.8 cm of the square but never
  released. 40 billed calls, `runs/maker_{apple,box}_vision_live2`,
  `runs/maker_wrist_report.md`. So the earlier Maker failure was our improvised
  camera, and wrist-camera placement is decisive. The real Viscous wrist camera is
  mounted the other way round (jaws enter from the TOP of the image), which may
  explain much of the Viscous failure. Next: the Viscous with a Maker-style wrist
  camera in sim (`BRIEF_VISCOUS_MAKERWRIST.md`). 220 billed calls today.
- *The Viscous works too, with a Maker-style wrist camera (2026-10-20).* Same rig as
  `viscous_apple_vision_live*`, only the wrist cameras moved to the Maker-relative
  pose (above the gripper behind the jaws, ~20° down, VFOV 77.2°, 16:9;
  `runs/viscous_wrist_match/compare.png`; the old mount is `--viscous-wrist legacy`).
  Zero-shot, beta, `viscous_arm.json` unchanged: **apple success**. The left arm
  reaches it by calls 5-7, lifts it 11.6 cm at call 9 and drops it in the bowl at
  call 10, where it rests (9 mm from centre) to call 20. The box was targeted (requested hand
  within 2 cm) but dragged and tipped, not placed. The Viscous fingers touched the table on
  418 (apple) and 292 (box) rows: the tool-point problem above. 40 billed calls,
  `runs/viscous_{apple,box}_makerwrist_live1`, `runs/viscous_makerwrist_report.md`.
  **So the main cause of the Viscous failure was its wrist camera view.** For the
  real arm: (1) mount the Viscous wrist camera like the Maker's, ideally the same
  KD-USB camera, looking forward along the fingers from above, fingers rising from the
  bottom of the image; (2) Gavin sets the tool point with `dreamcli tcp`; (3) a centre
  top camera placed like Dream's station's; (4) objects of about 5 cm or more; (5)
  a Lab runner for Dream-1 on the real follower with the safety limits. 260 billed
  calls today.

## Contact-signal bench check (2026-10-20, goals A1 and A1b, next steps 1-2)

*Intent:* prove the haptic signal on the arm before building the leader loop
(A3) on it: contact torque near zero in free motion, a push standing clear of
that floor, and the feedforward removing sag. "If we hit something, yes, we
should feel that."

- Code: `makermodslab/contact_torque.py` (`ContactEstimator`: reported torque
  minus fitted gravity minus the fit's telemetry bias; the feedforward never
  commands that bias). `gravity_feedforward.install` reads `status["scale"]`
  every frame so one session can A/B it. `scripts/viscous-contact-check.py`
  runs it; `scripts/viscous_bench.py` holds the helpers it shares with the
  gravity collector.
- Run: three poses x feedforward 0/100 % x approach above/below, 50 Hz log of
  every move, then a 60 s push hold at `forward`. `--analyze run.json` prints
  sag, telemetry-vs-(kp*err + ff) slope, the free-motion floor (p99) and how
  much of the push window clears it.
- Pose choice: the planner caps fitted load at the 0 % sag of both approaches,
  not just the nominal pose. With the 3.5 N m dry cap that limits the shoulder
  to ~1.5 N m, so the first-draft poses (up to 2.8 N m) were refused and
  replaced. The legs out of and back into the fold are exempt from the cap
  (collision-checked only): leaving the fold carries ~4 N m on any route, as
  every Lab teleop unfold does.
- First powered run (`evidence/contact-check-20261020-180543.json`) aborted
  1.3 s into the unfold on a 10 deg elbow lag with nothing wrong: at 0 %
  feedforward the elbow carries ~4 N m leaving the fold (~8 deg sag at kp 30)
  and the driver's startup sync stalled lifting the forearm off its stop. Fixed
  by tripping on lag beyond the expected sag and ramping feedforward on at the
  fold. Even that run showed the signal working: with the forearm resting on its
  stop, elbow contact read about -3.6 N m, the stop carrying the load.

**Results, unit 1 dry, `evidence/contact-check-20261020-181208.json`:**

- *Feedforward:* sag at 0 % 3.5-5.5 deg (the model predicts it within ~10 %;
  it under-predicts the shoulder at the two heavy poses), at 100 % at most
  0.21 deg on every gravity joint. Gravity feedforward works at full scale.
- *Telemetry includes the feedforward:* reported torque = kp x error + tau_ff
  at slope 0.99-1.00, r = 1.00, at both scales. So contact = reported - gravity
  is valid with feedforward on, which is how teleop will run.
- *Static holds:* contact within +/-0.31 N m at all 12 holds (friction band
  0.01-0.13 deg, wrist up to 0.5 deg).
- *Free motion (15 deg/s legs, 2917 ticks):* p99 1.17 shoulder, 0.79 elbow,
  0.61 wrist; rms 0.27-0.36 N m. Not the fold: excluding it changes nothing.
  Coulomb friction ~0.2 N m shows as the v>0 / v<0 split (+0.16 / -0.24);
  subtracting a fitted friction term brings elbow and wrist p99 to 0.54 / 0.39.
  The shoulder's remainder (0.74) looks like model error at its larger loads.
- *Push test (at `forward`, feedforward on):* two clean pushes, about 4 s each,
  reading +2.1 / -2.2 N m shoulder and +1.5 / -1.3 N m elbow (3-4 deg of give),
  against a still baseline of 0.06-0.12 N m rms (wrist 0.4). That is about 20x
  the still noise but only ~2x the free-motion floor, so A3 needs the floor
  brought down (friction compensation, a shoulder refit) before it can gate on
  contact during motion. The wrist's still offset stepped -0.4 to +0.2 N m
  across the first push: stiction, the floor for a held pose. Gavin pushed
  only up and down that run, so nothing showed on pan, yaw or roll.
- *Sideways (rerun `evidence/contact-check-20261020-182410.json`, down / up /
  left / right):* down and up repeat run 2 (lift +1.5 / -1.7, elbow +2.0 /
  -1.5 N m). Left and right land on `shoulder_pan`, +1.65 / -2.25 N m against
  a still baseline of about +/-0.03 N m: pan carries no gravity, so its
  contact is raw torque and it is the cleanest channel (50-70x its noise). The
  right push also showed +1.0 N m on `wrist_yaw`. Yaw and roll sit at a steady
  -0.15 to -0.2 N m at rest, a bias the fit's offsets do not remove.
- *Lowering the free-motion floor (offline, 2026-10-20):* `gravity-air-fit-v2.json`
  refits gravity on all 51 holds (the two collection runs plus both complete
  contact checks) and adds a per-joint friction term (`scripts/viscous-contact-fit.py`,
  Coulomb x tanh(v / v_s) + viscous x v on COMMANDED velocity, zero at rest).
  Trained on one complete run and scored on the other, both ways, moving-arm p99
  falls shoulder 0.86-0.97 -> 0.74-0.79, elbow 0.60-0.64 -> 0.42-0.46, wrist_flex
  0.55 -> 0.38-0.41 N m; pushes unchanged; yaw/roll rest bias halves. The
  estimator subtracts friction only when given velocities, and holds each term
  at the fastest speed it was identified at (16 deg/s): never extrapolate, since
  a wrong friction torque reads as contact. Pan never moved and yaw/roll only
  under 2 deg/s, so they carry no friction term. v2 stays opt-in (`--fit`) until
  a live run confirms it; the shoulder's tail (model error at load) is still
  the limiting channel. Details: `configs/viscous/viscous_01/contact-floor-report-20261020.md`.
- *Live confirmation (`evidence/contact-check-20261020-192434.json`, a run
  neither fit saw):* moving-arm p99, old fit -> v2 + friction: shoulder
  0.87 -> 0.77, elbow 0.63 -> 0.45, wrist_flex 0.56 -> 0.41, wrist_yaw
  0.53 -> 0.45, roll 0.48 -> 0.43 N m; still rms falls on every joint. Pushes
  read the same (shoulder +/-2.3, elbow +/-1.8, pan +/-2.2 N m; the right push
  also +1.2 / -1.5 on wrist_yaw). Feedforward holds at <= 0.27 deg with v2. A
  2 N m push now clears the moving floor ~4-5x on elbow and wrist, ~2.5x on the
  shoulder. v2 is now the contact check's default fit. The teleop feedforward
  drop-in on gavin-linux still names the original fit (gravity only; both
  hold sag under 0.3 deg). Run with `--max-torque 3.55`: v2 puts the elbow at
  3.50 N m on the back approach at 0 % (Gavin approved 3.55 for this run).
- Known unrelated test failure: `test_single_device_configs_carry_the_port_and_id[viscous]`
  (missing commissioned `cal-f` profile), present at `HEAD` too.

## Handoff: state and how to resume (any agent, any tool)

This section is the source of truth for picking the work back up. Nothing about
this project should live only in one assistant's private memory: update this
file instead.

**Machines** (Gavin's):
- **MacBook Air** (this repo at `~/Documents/makermodslab-viscous`). The
  waterproof arm `viscous_waterproof_01` is connected here: CANable serial
  `205038953846` (`/dev/cu.usbmodem2050389538461`), Star leader on
  `/dev/cu.wchusbserial*`. Lab state lives in `~/.makermods/makermodslab-viscous`.
- **gavin-linux** (Tailscale `100.64.91.4`, `ssh gavin-linux`; same repo path).
  Unit 1 `viscous_01` (dry, non-waterproof) is connected here: CANable
  `/dev/serial/by-id/usb-Openlight_Labs_CANable2_..._207E37A55445-if00`, Star on
  `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`. Lab runs as the user
  service `makermodslab-viscous` (`http://100.64.91.4:8000`).
- The Mac repo has a `linux` remote (`gavin-linux:Documents/makermodslab-viscous`)
  for moving unpushed work between machines. Git on Linux authenticates through
  `gh` (`gh auth setup-git` is done); its GitHub fetches occasionally time out, so retry.

**State at the end of 2026-10-20:**
- Everything is committed and pushed on both machines (`feat/viscous-arm`).
- On gavin-linux the service has a drop-in,
  `~/.config/systemd/user/makermodslab-viscous.service.d/gravity-ff.conf`, that turns
  on gravity feedforward for `viscous_01` teleop at 50%
  (`MAKERMODSLAB_GRAVITY_FF`, `MAKERMODSLAB_GRAVITY_FF_SCALE=0.5`). Delete it and
  `systemctl --user restart makermodslab-viscous` to turn it off.
- Unit 1 was left torque-off, tipped forward ~25° onto its gripper (the release
  tip described under Safety lessons). Refold it before any powered run; the
  collection planner refuses to start from the tipped pose.
- Pinocchio is installed in both venvs. On a new machine: `uv sync --extra test
  --extra arm-dynamics`.

**Resume here (2026-10-21):** Gavin's first item is goal B3, finding why
Dream-1 drives the Viscous sim so badly: work the checklist under "Dream-1 trial"
in order. Haptics: steps 1-2 done 2026-10-20, friction term and shoulder
refit done and confirmed live (v2 fit). Before energizing, show Gavin the
dry-run plan and say exactly which joints will move.

**Resume haptics here (evening of 2026-10-20, Gavin stopped to eat; next
session):** goal A2, the Star leader as an actuator (next step 3).
`scripts/star-actuation-check.py` (raw FashionStar transport
`makermodslab/fashionstar_raw.py`, tests `tests/test_fashionstar_raw.py`) is
built and dry-run on gavin-linux (pings + reads only; evidence
`configs/viscous/viscous_01/evidence/star-actuation-20261021T010608.003041Z-*`).
It energizes ONE servo, wrist_flex = id 3, with every other servo released,
using single-turn angle command 0x08 with an explicit power cap (power 0 is
refused: it means the protection ceiling). Phases: hold at 100/200/400/800 mW
(Gavin pushes), move +/-10 deg over 1 s (Gavin may block it), damping 0x12 at
each power (Gavin moves it), a 2.5 deg latency step. Every exit path releases
all seven servos twice; stops are unacknowledged and there is no bus-loss
watchdog, so the Star's power supply is the hard stop.
1. Ask Gavin where the Star's power switch / plug is (unanswered), and confirm
   wrist_flex as the first joint.
2. Re-run `--dry-run --joint wrist_flex` (the arm may have moved), show him the
   printed plan.
3. Best run by Gavin himself in a terminal on gavin-linux (ssh from the Mac),
   so the Enter-to-advance prompts and cue lines are in front of him at the
   arm: `.venv/bin/python scripts/star-actuation-check.py --joint wrist_flex`.
   A detached run works too, but cues then lag through chat.
4. Record his feel per power level alongside the logged deflection/current;
   that decides the power range the input-gated loop (A3) can use.
Unit 1 (the follower) is torque-off and probably tipped forward again; it is
not needed for the Star test.

**Open item (Gavin, 2026-10-20):** unit 1's wrist camera image is flipped. The
`viscous_01` record's `wrist` camera (opencv index 0, 640x480 MJPG) has no
rotation field; lerobot's `OpenCVCameraConfig` takes `rotation` (0/90/180/-90),
but check whether the Lab's camera config passes it through before relying on
it. Ask which way it is flipped (180 deg vs mirrored) before fixing: a mirror
is not a rotation. Fix it before recording any dataset or policy run on unit 1.

## Working rules for this project

How Gavin wants the work done. These apply to every agent and tool.

- **Frame every task as a subtask of the goal tree** above. Record the objective
  and Gavin's intent behind it (his words where possible) in this file, adding
  dated lines rather than rewriting history. Commit messages say *why*.
- **Never leave progress unsaved.** At milestones, check `git status -sb` and
  unpushed commits on BOTH machines and all related repos (`makermodslab-viscous`,
  `viscous-arm-simulation` a.k.a. `~/Documents/viscous-arm-model`, the lerobot
  forks). Commit AND push.
- **The repo must be self-contained.** A fresh clone must have every URDF, mesh,
  calibration, profile, fit and config it needs. Committed data stores
  repo-relative paths, never `~/Documents/...`. Never commit `frontend/dist`
  (rebuild locally) or multi-GB render output.
- **No AI attribution trailers** (no `Co-Authored-By`) in commits or PRs.
- **Powered hardware runs:** say exactly which joints move before energizing.
  Prove the stop path on the actual launch method first; a process started in the
  background ignores SIGINT unless it installs its own handler. Check liveness
  with `kill -0 <pid>`, not `pgrep -f` inside the same ssh command. Gavin's power
  switch is the hard stop. The thermal ceiling is the driver's 85 °C stop;
  scripts abort just below it (82 °C) and cool at 80 °C. On the dry arm keep
  planned loads ≤ ~3.5 Nm (see Results: the shoulder dropped the arm at ~5.6 Nm).

