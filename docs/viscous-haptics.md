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
