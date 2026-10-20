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
- *Unblocks:* A3. *Status:* model done for unit 1 dry; live validation next.

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
- *Status:* sign verified at 50% in GUI teleop; A/B in degrees pending.

**A2. A position-drivable leader.** The Star's servos must be commanded toward
the follower with capped effort (`star_follow.py` draft).
- *Intent:* serves A3 and, later, DAgger (B1): one piece of hardware work for both.
- *Status:* draft code; Star actuation not yet characterised on hardware.

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
  probably thermal derating or a protective cutout. The RS00 is rated ~5 Nm;
  the arm is all-RS00 because it is designed to be near-neutral in water.

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
  (every stop, including Lab teleop's). Not yet fixed.

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

1. **Contact-torque signal** live in the log: measured − gravity, per joint;
   verify ~0 in free motion and a clear jump when the follower is pushed.
2. Automated A/B of the feedforward (0 vs 100%) at moderate poses: sag in degrees.
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
- *Next: the 2x2.* Run A = YAM in Dream's scene doing our cube/square task;
  Run B = the Viscous in Dream's scene doing the apple task, with the baseline's
  executor (48 rows, no extra clip, gripper open). Which cell fails names the cause.

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
- Status: offline only. Unit tests and a synthetic run through `--analyze`
  pass; the detached stop test on gavin-linux passed (SIGTERM, `kill -0`).
  The read-only dry run on gavin-linux got no handshake reply from any of the
  seven motors (arm power off?), so no plan from the real pose yet.
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
in order. Haptics: next step 2 (automated feedforward A/B), then 1 (live
contact-signal validation). Before energizing, show Gavin the dry-run plan and
say exactly which joints will move.

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

