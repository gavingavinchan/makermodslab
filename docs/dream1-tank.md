<!-- Copyright 2026 MakerMods. All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-->

# Dream-1 in the aquarium: the keep-out box and the real-run plan

2026-11-07 · branch `feat/viscous-dream1-tank-box` · goal B3 in
[viscous-haptics.md](viscous-haptics.md), building on [tank-measure.md](tank-measure.md).

Gavin: "can we now build a bounding box to run dream1?" The box must gate
every command sent to the real waterproof arm (`viscous_waterproof_01`) so it
can never hit the glass. And: "the floor of the aquarium is the only part of
the aquarium the arm is allowed to touch anyways". Until now Dream-1 has run
on the Viscous in MuJoCo only.

**State: the box is built and tested offline. Nothing drives the arm yet.**
No powered motion, port opening or Lab restart was done for this work. The
real-run plan below is a proposal for Gavin to approve before any powered step.

## The box

Code: [`makermodslab/tank_guard.py`](../makermodslab/tank_guard.py) (pure:
numpy plus the CAD FK in `tank_geometry.py`, no hardware imports). Config:
[`keepout_viscous_waterproof_01.json`](../configs/viscous/environments/aquarium_landen_75/keepout_viscous_waterproof_01.json),
where every number carries its source. Tests:
[`tests/test_tank_guard.py`](../tests/test_tank_guard.py).

For each new joint target, and for the straight joint-space path to it from
the last accepted target, it checks:

- **Every vertex of every moving link** (9,059 CAD collision vertices, from
  `yaw_link` to the fingers) against the walls and rim, keeping a margin plus
  per-link cable padding. This uses the mesh extremes, not the tip point. On
  the floor the tip read 0 mm straight down but 15-22 mm tilted, because the
  finger edge touches first.
- **The floor as allowed contact.** A vertex may go at most 10 mm below the
  floor bound (padding ignored), so a command can touch the floor but never
  ask for a deep push into the glass.
- **Joint limits**: the target is clipped into them.
- **Jumps**: a request more than 20° from the last accepted target, on any
  arm joint, is refused outright, never clipped and never slid toward. This
  catches a J1 wrap (+182 → −178) and IK branch flips.

A target that is fine is passed through. A path that would reach the
keep-out is cut at the last clear point (bisection), so the arm stops at the
margin instead of freezing far from it. A jump, or a start that is already
inside the keep-out, is refused.

**The path is proven clear, not sampled hopefully.** For each joint the guard
precomputes an upper bound on how far any downstream vertex sits from that
joint's axis (J1 732 mm, J2 678, J3 518, J4 368, J5 281, J6 221; the gripper
moves a finger 0.46 mm per motor degree). Those bounds limit how far any
point can move along a segment. The segment is cut into samples so that no
point moves more than 4 mm between them, and every sample must clear by 2 mm,
so every point between samples is clear too. A test checks the bound against
real vertex motion on random steps.

Speed: about 0.5 ms per pose. A small step (1°) takes about 2 ms. A typical
Dream row at 15 deg/s (0.75° per joint) takes about 5 ms. A request that must
be cut at a wall first took about 260 ms (14 bisection rounds, each
re-checking the whole path). It now scans the samples once and refines only
inside the last gap: a 19° J1 swing into the wall takes 17.5 ms on the Mac,
and the granted point is unchanged.

### The numbers (J1 base frame, mm)

| Surface | Plane | Status | Required clearance |
|---|---|---|---|
| +y wall ("left" in the model) | 210 | measured (trace) | 30 margin + 40 cable padding; fingers 30 |
| −y wall ("right") | −224 | measured (trace) | same |
| near/back wall (+x, behind J1) | 40 | **estimate, not traced** | same; see the base-riding links below |
| far wall | −694 | estimate (out of reach) | same |
| floor | −6 | bound (highest the evidence allows) | contact allowed, at most 10 mm press |
| rim | 427 | estimate (Gavin's −15 floor + 442) | 30 margin + 40 padding |

Where they come from (the evidence trace is committed:
`configs/viscous/viscous_waterproof_01/evidence/tank-trace-20261108T044404Z.jsonl`):

- **Side walls.** These are the numbers Gavin accepted ("lets go with this for
  now"). Re-fitting the trace with every mesh vertex, the arm actually
  reached +218.5 mm (plateau mean 213) and −223 to −231 mm (before the base
  slid). So both planes sit 3-8 mm on the conservative side. Yaw is taken as
  0; the every-vertex line along the +y wall shows 0.25°.
- **Floor.** The fingers' lowest point reached −6.2 mm, so the glass is at −6
  or lower. Gavin says it is about −15. The box uses −6, which bounds how hard
  a command can press: at most 10 mm into −6, so at most about 1 mm into the
  real −15.
- **Rim.** The lowest plausible rim, from Gavin's −15 floor.
- **Cable padding.** 40 mm is the default for unmeasured links. The fingers
  carry no cables and are the measured probe, so they get 0.
- **Base-riding links.** `yaw_link` turns with J1 only. Its extent behind J1
  is 30-42.5 mm at every J1 angle, so it is exempt: it cannot approach a wall
  it doesn't already touch. The upper arm's heel sits 28 mm behind J1
  whenever J2 ≤ −100°, and reached 36.9 mm in the trace, so it cannot keep
  70 mm off a wall estimated at 40. Until the near wall is traced, the heel
  only has to stay behind the estimated plane. That still forbids the upper
  arm tipping back past about vertical (the heel reaches 58 mm at J2 = −80).
  Every link beyond the elbow keeps the full near rule: that is the J3 hazard
  Gavin named.
- **Starting outside limits.** The arm rests in the tank outside three soft
  limits (trace: J1 191 vs 187.5, J2 −201.7 vs −199.4, gripper −0.5 vs −2.9).
  A start up to 5° outside is accepted, as the Lab's slow mode already does.
  The first target is then clipped inside along a checked path.

### What the box does NOT cover

- **The loose base.** It slides about 30 mm under a hand push, and the 30 mm
  margin is not meant for that. A powered arm can shove it, and FK has no way
  to see it. **Clamp or weight the base before any powered run.**
- **The straight-path proof assumes the targets really move in a straight
  line.** A per-joint slew limiter downstream bends that path, because joints
  finish at different times. So the executor must retime each row
  synchronously (all joints arrive together at ≤ the speed cap), or the guard
  must sit after the limiter.
- **Force.** The box decides where a target may go, not how hard the arm
  pushes when blocked. The floor press is bounded by the 10 mm allowance.
- **Tracking error.** The measured pose can lag or sag behind the target.
  `monitor()` stops the run if the measured pose eats more than 15 mm into the
  margin.
- **The support tower and cables outside the URDF**, beyond the padding.
  The tower is not measured yet.
- **Room.** The allowed fingertip region spans most of the tank's width and
  about 600 mm down it (workspace map below). Which way J1 can turn depends on
  the pose. On the −y side it can go down to about 150-165°. On the +y side
  the **J1 soft limit of 187.5°** binds before the glass does. The arm rests at
  189-191° by hand, and that limit dates from an older J1 frame, so it may be
  stale. That is a question for Gavin. Cable padding matters: 20 mm instead of
  40 mm allows 43% more sampled poses, and 0 mm allows 85% more.

## Validation (offline)

Codex ran it on gavin-linux from Claude's brief; Claude reviewed it. Full
report, scripts and evidence:
[`scripts/viscous-dream1/runs/tank_guard_validation/report.md`](../scripts/viscous-dream1/runs/tank_guard_validation/report.md).

- **No unsafe grant anywhere.** 21,261 adversarial requests: random
  targets, J1 wraps of ±360/720°, IK-style flips, slow ramps and 0.02° creep
  into every wall, the rim, the near wall with the elbow folded back, floor
  dives, and fingers opening against a wall. On top of those come every
  granted step of 220 replayed sim arm streams. Every accepted or clamped
  path was re-checked by an independent checker. It transforms every vertex
  itself and does not call the guard's clearance code, sampling at ≤ 0.05°:
  25,019 paths and 562,508 poses, worst slack +0.00001 mm. The motion bound
  held on 3,000 random steps.
- **Tonight's trace:** the shipped box accepts 540 of its 857 poses (the rest
  are the deliberate wall touches inside the margin). With no margin or
  padding, the mesh passes the +y plane by up to 8.5 mm (the conservative
  210) and the −y plane by up to 32 mm (the base sliding), as expected.
- **Dream's sim tasks do not fit the tank as laid out.** The sim's requested
  hands go up to 320 mm to the side, 110 mm past the +y glass; only 125 of 218
  arm streams stay inside the raw planes. Dream's folded starting posture
  fails the box (forearm against the estimated near wall or the rim), so a
  replay that ramps from the arm's tank rest stalls at the rim. Sim chunks
  therefore do not predict tank behaviour. The real run needs its own start
  pose (arm reaching down the tank, hand above the floor) and objects placed
  down the tank's centre line, where the box has room.
- The 260 ms clamp the report times was fixed afterwards (see above). A
  further speed-up (a convex-hull vertex subset, 9,059 → 2,794 vertices, exact
  for planes) is proposed, not applied.

## The real-run plan (proposal; nothing here runs without Gavin's go-ahead)

Research behind it:
[`scripts/viscous-dream1/runs/real_executor_research/report.md`](../scripts/viscous-dream1/runs/real_executor_research/report.md)
(read-only, from the installed client; no billed calls).

**Where it runs: everything on the Mac.** The arm's CAN adapter and the
cameras are USB on the Mac. The Dream client and its IK are pure Python plus
JAX on the CPU, which has Apple Silicon wheels, so there is no reason to
split the loop across machines. This needs a new Mac venv (Python 3.12,
`dreamlabs` 1.0.1); never copy the Linux one. The API key goes into the Mac's
`~/.dreamlabs/credentials` (0600) by Gavin's hand, and is never committed.
It is not yet tested on the Mac.

**How Dream reaches the arm.** The model returns 48 tool poses at 20 Hz in the
arm's base frame; the client's IK turns them into joint targets.

1. *A tank IK model.* No arm-file field can rotate the base, and J1 rests at
   +182..191°, outside the IK URDF's J1 limit. So ship a tank-specific IK URDF
   with a fixed `dream_base` root turned 180°, and use J1_IK = J1_motor − 180°.
   Dream then sees "forward" as down the tank, and the arm's resting J1
   becomes +2..11°. Give its J1 limits the verified motor limits minus 180°.
   Fix the arm file's `../../Documents/...` URDF path at the same time.
2. *Per row:* IK joints (rad) → motor degrees with `motor_to_urdf.json`, then
   J1 + 180°. Use one unwrapped J1 branch throughout, never a wrap. The grip
   fraction is clipped to [0, 1], then mapped to motor −2.9 (closed) … −115
   (open).
3. *Retime* each row so no joint exceeds the speed cap (15 deg/s for the first
   runs) with all joints arriving together. This keeps the path straight for
   the guard. At 15 deg/s about 18% of sim rows would be stretched.
4. *Guard every write:* `GuardedTargets.step()` on every sub-step, then the
   CAN write through the existing slow-mode bus. A refused jump (chunk seams
   jumped up to 65° in sim) means hold and stop, not freeze silently.
5. *Monitor:* every measured pose goes through `monitor()`. A violation, a
   stale reading, a CAN error or a temperature at 82 °C stops the run.
6. *Between chunks* the arm holds its last target for 1.3-1.6 s (median call;
   worst 6 s), while CAN service and the cameras keep running.

**Start pose and task layout.** Not Dream's folded rest: it fails the box.
Hand-place the arm reaching down the tank with the hand ~100-150 mm above
the floor, and set the tank IK file's `rest` to that pose. Put the object on
the tank's centre line 300-450 mm in front of J1, where the workspace map is
densest.

**Start gate.** Fresh readings from all seven joints, the measured pose
accepted by `start_gate()`, and the first target equal to the measured pose.
Gavin presses start. There is no move to a "rest" pose first.

**Stop.** A key press holds the current target. From there, a slow, guarded
return to the start pose, with the path checked like every other move.
Torque goes off only once the arm is resting on the floor (it has no brakes).
Gavin's power switch stays the hard stop.

**Cameras.** Dream needs `overhead`, `wrist_l` and `wrist_r`.

- *Wrist:* mount it like the Maker's (the KD-USB camera): 79 mm above the
  fingers, 87 mm behind the tool point, 20° down, with the fingers rising
  from the bottom of the image. The sim showed this placement decides
  everything (the Viscous went from failing to 6/6).
- *Overhead:* in the sim the camera sat 0.78 m above the base, looking 55°
  down along the working direction. Over the tank that means a rig above the
  open top, looking down the tank, with the water surface in the way, so
  glare and refraction are untested.
- *Second arm:* the sim's best one-arm result needed a second arm parked in
  view with its own wrist camera (one arm alone: 2/6; with a parked arm: 6/6).
  In the tank there is one arm. This is an open question.

**Billing.** Cap calls per run (`--max-calls`, 10-20) with a ledger, as in the
sim sweeps.

**Staged trials, each with Gavin's go-ahead:**

0. *Offline on the Mac:* install the client and run the tank IK and the
   executor against recorded sim chunks with no arm and no network. This
   checks retiming, the guard and logging.
1. *Dream in the loop, arm not moving:* torque off, real cameras, about 10
   billed calls. Log what the arm would have done and every guard verdict.
2. *Powered, no Dream:* replay a short recorded chunk sequence in the tank at
   15 deg/s near the middle, with Gavin at the power switch and each phase
   relayed live.
3. *Live Dream:* a short run (10 calls) at 15 deg/s, the same supervision.

**Before step 2 (hardware, Gavin):** clamp or weight the base; trace the
near/back wall; ruler-check the floor gap; confirm the glass thickness; ideally
measure cable padding; mount the cameras; confirm which side is "left".

## Open questions for Gavin

1. Which side is "left" from where you stand? The model calls +y "left", and
   that was the first wall traced (J1 above about 182° swings the hand toward it).
2. Trace the near/back wall before running? It is still the 40 mm estimate,
   and J3 can hit it.
3. The base clamp: what can hold it without squeezing the rimless glass edge?
4. The glass thickness from the box label (8 mm assumed; the measured 434 mm
   width agrees).
5. Cable padding: keep 40 mm (a narrow box), or press each link's cable side on
   a wall once to measure it?
6. A second, parked arm in view for Dream (6/6 vs 2/6 in sim), or try one arm
   first?
7. Is J1's soft limit of 187.5° real (cables, a hard stop) or left over from
   the old J1 frame? It, not the glass, stops the hand on the +y side, and the
   arm already rests at 189-191° by hand.
8. Design calls I made, to confirm: `yaw_link` exempt; the upper arm's heel
   allowed up to the estimated near plane; a start up to 5° outside the soft
   limits accepted; jumps over 20° refused; floor press capped at 10 mm.
