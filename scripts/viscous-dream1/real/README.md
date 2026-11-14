<!-- Copyright 2026 MakerMods. All rights reserved. Licensed under the Apache License, Version 2.0. -->

# Dream-1 on the real Viscous arm (waterproof unit)

Goal B3 in [docs/viscous-haptics.md](../../../docs/viscous-haptics.md). `dream_real.py` lets Dream-1
(`dreamlabs/dream-1-beta`, through Dream's own client and client IK) drive the real waterproof arm
(`viscous_waterproof_01`) in the aquarium, with the Lab's CAN machinery underneath. **No agent has
run it on hardware or made a billed call with it.** Everything below `--live` is code that has
only run against fakes.

Tool point and camera check (Gavin's steps): [GAVIN_TCP_AND_INPUT_CHECK.md](GAVIN_TCP_AND_INPUT_CHECK.md).
Keep-out box and the real-run plan: [docs/dream1-tank.md](../../../docs/dream1-tank.md) (branch
`feat/viscous-dream1-tank-box`).

## Files

| File | What it is |
|---|---|
| `dream_real.py` | The runner: `--dry-run` (default), `--preview`, `--live`; `--rehearse` (no Dream) alone or with `--live` |
| `stop_button.py` | The big red button: an always-on-top window that sends SIGTERM to the runner's `--pid-file` pid |
| `dream_real_core.py` | Pure helpers: frame maps, limits, retiming, workspace box, trips, refusals, ledger, images |
| `make_tank_arm.py` | Generates the tank IK model below, with provenance |
| `tank_model/viscous_waterproof_tank.urdf` | The arm URDF with a `dream_base` root turned 180 deg and the unit's own limits |
| `tank_model/viscous_waterproof_tank.provenance.json` | Sources, hashes, checks and `approval` (approved by Gavin 2026-11-08, provisionally, on condition every live motion stays <= 15 deg/s) |
| `tank_model/viscous_waterproof_tank_arm.json` | Dry-run arm file only (no `tcp_calibration`; tcp copied from `viscous_arm.json`, never confirmed) |
| `workspace_box_viscous_waterproof_01.json` | Per-point boxes in the physical base frame (proposal, pending Gavin) |
| `tests/test_viscous_dream_real.py` (repo root) | Offline tests: pure helpers, the loop on fakes, the stop button's logic |

## Why a tank model

In the tank the arm works toward its base **-x** (shoulder_pan about +182..186 deg). Dream assumes
base **+x** is forward, an arm file has no field that turns the base, and the URDF's pan limits
(-185.4..127.6) are from an older J1 frame. `make_tank_arm.py` adds a root link `dream_base`, turned
180 deg about z above `base_link`, and turns `shoulder_pan`'s origin by 180 deg as well, so the chain
is unchanged and **IK pan = motor pan - 180** (+182..191 becomes +2..11; one branch, never wrapped).
Joint limits come from the unit's profile
(`configs/viscous/viscous_waterproof_01/evidence/lab-hardware-profile.json`) through
`configs/viscous/model/motor_to_urdf.json` (J5 sign -1), with pan kept within +-90 deg of Dream's
forward. A built-in check confirms tool0 in `dream_base` = Rz(180) x tool0 in `base_link` to 1e-15 m
at 200 random poses; Dream's own `arm_file.register` accepts the file.

## Commands

All from the repo root with the repo venv. Dream's client is loaded from `~/viscous-dream-real/.venv`
(dreamlabs 1.0.37 / client 1.1.3), appended after the repo's packages.

```bash
# Dry run: the plan, its plan id, live readiness, and the loop on fakes. No port, camera, network, billed call.
.venv/bin/python scripts/viscous-dream1/real/dream_real.py
# Every safety trip fired once on the fake arm, each ending with re-anchor, gripper off, return legs, then torque off:
.venv/bin/python scripts/viscous-dream1/real/dream_real.py --scenario all
# Rehearsal on the fake arm (no Dream): every phase printed as it starts, then the stop path:
.venv/bin/python scripts/viscous-dream1/real/dream_real.py --rehearse [--hold-s 120]
# Inputs preview, no motion (Gavin runs it; it opens the record's CAMERAS, never CAN/serial or the network):
.venv/bin/python scripts/viscous-dream1/real/dream_real.py --preview --overhead <camera> [--wrist-r black|copy|camera:<name>]
# Regenerate / export the tank model (offline):
.venv/bin/python scripts/viscous-dream1/real/make_tank_arm.py [--rest-motor-deg '{...}'] [--export ~/viscous-dream-real/tank_model]
# Tests:
.venv/bin/python -m pytest tests/test_viscous_dream_real.py -q
```

`--preview` saves the frames exactly as Dream would get them (`overhead.jpg`, `wrist_l.jpg`,
`wrist_r.jpg`, `sheet.jpg`), prints the state handed to the client (radians) and as the client sends
it, and prints the 12-hex **inputs id**. Never run `python -m dream.check` on this arm: it moves it.

## Preconditions for `--live`, in order

`--live` refuses, before opening anything, unless all of these hold. The dry run lists what is missing.

1. **Tank model approved (Gavin).** Review `tank_model/` (frame, limits, rest). Then in
   `viscous_waterproof_tank.provenance.json` set `approval.status` to `approved`, `by`, `at`, and
   `urdf_sha256_normalized` to the hash the file names. Regenerate first if the rest pose or limits
   change (`--rest-motor-deg`). The rest is Gavin's teleoped tank start pose (2026-11-08,
   `runs/start_pose/20261108T183949Z.json`).
2. **Tool point on the approved model (Gavin), `dreamcli tcp`.** The `tcp_calibration` digest covers
   the URDF's bytes and base/flange/tcp/approach/jaw, so it must be set **on the tank model**, not on
   the original URDF: `make_tank_arm.py --export ~/viscous-dream-real/tank_model`, then in `dreamcli tcp`
   choose that folder (base `dream_base`, end `wrist_roll_link`). An arm file made on the original URDF
   is refused (base `base_link`). Changing the URDF later means redoing this step. The runner loads the
   arm file with the URDF copy dreamcli wrote beside it and never edits tool keys. dreamcli writes
   `rest` as zeros (a straight, level arm); `rest` is outside the digest, so the runner gives Dream the
   tank model's rest through a byte-identical copy in the run folder (`--keep-arm-rest` to opt out).
3. **Cameras.** The record has only `wrist` (rotation 180). Add the overhead camera to the record
   (Robot settings, Flip 180 if mounted upside down) and pass `--overhead <name>`. Choose `--wrist-r`:
   there is no second arm (sim: one arm alone 2/6, a parked second arm with its camera 6/6).
4. **Inputs checked (Gavin).** Run `--preview` with the final cameras and settings, look (upright,
   jaws from the bottom, natural colours, right keys, radians), then record the id it prints with
   `~/viscous-dream-real/.venv/bin/python -m dream.input_check <id>`. The live preflight builds the
   observation with the real cameras before any CAN traffic and refuses unless that id is recorded;
   Dream's own reminder is printed, never suppressed. The runner never records it.
5. **Joint signs.** J5 (`wrist_yaw`, sign -1) and J6 (`wrist_roll`, sign +1) were hand-tested
   2026-11-08 (`configs/viscous/viscous_waterproof_01/evidence/j5-j6-sign-test-20261108.json`). If a
   status in `motor_to_urdf.json` ever says pending/unverified/assumed again, `--live` needs
   `--ack-unverified-sign <joint>` for it.
6. **Tank guard merged.** `makermodslab/tank_guard.py` (branch `feat/viscous-dream1-tank-box`) gates
   every substep at mesh-vertex level; `--live` refuses without it. Its hardware items too: clamp or
   weight the base (it slides about 30 mm), trace the near wall. Before every powered run: **post
   between the marks and flush on the back glass** (the 2026-11-13 mount; the keep-out's back plane
   and both side walls assume it, and nothing in the software can check it).
7. **Credentials** `~/.dreamlabs/credentials` (0600, a fresh key) written by Gavin. Never printed or logged.
8. **Plan id.** Run the dry run with exactly the live settings and `--arm-file`; pass its plan id as
   `--plan-id`. Its "Joints that may move" list is the statement of what moves.
9. **Lab idle.** No session, no tank measurement (`/api/v1/sessions/current`), nothing holding the
   follower port (`lsof`). `--lab-down-ok` if the server is stopped.

Live, with Gavin at the power switch and the red button on screen, backgrounded and relayed phase by
phase (each phase prints a `phase:` line as it starts):

```bash
.venv/bin/python scripts/viscous-dream1/real/stop_button.py --pid-file /tmp/dream_real.pid &   # the red button
.venv/bin/python scripts/viscous-dream1/real/dream_real.py --live --arm-file ~/viscous-dream-real/station/arms/viscous-waterproof-tank.json \
  --overhead <camera> --plan-id <id> \
  --max-calls 10 --pid-file /tmp/dream_real.pid
kill -TERM "$(cat /tmp/dream_real.pid)"   # the same as the button: stop; a second signal aborts the return and releases
```

The button: first press (or Space / Enter / Escape with the window focused) is STOP; it then reads
RELEASE ALL TORQUE NOW (arm drops), and a click on it (at least 1 s later; keys never do this, so a
held key cannot skip the return) sends the second SIGTERM. It shows whether the pid is alive and
refuses to signal a pid that is gone or is not `dream_real.py`. It is its own process, so it works
even if the runner's loop is stuck.

**Rehearsal on the arm (no Dream, no billed call, no cameras, no credentials or inputs check):** the
same hardware path, stopping at the Dream start pose. It keeps the hardware refusals (tank model
approved, joint signs, plan id, Lab idle, port free, tank guard). Its plan id differs from the Dream
run's; take it from `--rehearse` (the fake form) with the same settings.

```bash
.venv/bin/python scripts/viscous-dream1/real/dream_real.py --rehearse --live --plan-id <id> --pid-file /tmp/dream_real.pid [--hold-s 120]
```

Connect, start gate on the measured rest, engage, approach, hold at the start pose (fresh feedback,
every trip and the guard monitor running) until the button or `--hold-s`, then the stop path.

## What a live run does

**Speed: every arm motion is <= 15 deg/s** (Gavin: "as long as we go slow, must keep the 15deg/s
limits"; the tank model is approved on that condition). `--speed-deg-s` above 15 is refused, the bus
backstop is set to the same speed, the approach and every return leg run at it, and a planned
substep faster than it trips before it is sent. The gripper keeps its own 60 deg/s cap.

1. Refusals above. 2. Preflight: real cameras, real client, the first request built and stopped
locally (nothing sent): refuse unless its inputs id is recorded. 3. Connect the follower like Lab
hosting does (bus backstop at the arm speed, <= 15 deg/s, installed first, `connect(calibrate=False)`,
then zero effort and torque off); a failed connect runs the de-energize recovery. 4. **Start gate** on
a fresh torque-off reading of the **rest** (the arm lying flat on the tank floor): limits (5 deg
tolerance) and the tank guard. The workspace box does not apply to the rest (lying flat, its tcp can
be below the box floor). The **Dream start pose** (the arm file's `rest`, IK radians, through the joint
map; gripper as measured) must be inside the limits and the box. The approach is planned here, every
substep through the guard, so a refusal never energizes. 5. `remote_can.engage` (hold the measured
rest). 6. **Approach**: rest to the start pose along `core.approach_waypoints` (lift the shoulder and
fold the elbow first, then straight to the start), each leg retimed into 20 Hz substeps at the arm
speed. Frames go to Dream, but no call is made until the start pose is reached. 7. Loop at 20 Hz:
fresh feedback, trips, 5 Hz frames to Dream; one billed call at a time (ledger first), executing each
48-row chunk, holding while the next call is in flight. Dream is told the six arm joints at the last
COMMANDED target and the gripper as MEASURED (`--dream-state commanded`, the default, in the plan id):
the arm sags ~5 cm under its command at the start pose, and told the measured joints Dream "corrected"
that by lifting the hand and folding the arm back toward the back wall at the start of every run (live
20261108T215535Z); the measured gripper shows a grasp that blocks the jaws. Every trip, the guard
monitor, the start gate and the stop still use the measured pose (`--dream-state measured` = the old
behaviour). The run stops after `--max-stuck-calls` (3) calls in a row that each execute at most
`--stuck-substeps` (2) substeps (215535Z burned 19 calls at one substep each). 8. **Every exit** (trips, signals, the
button, the call cap): (a) **re-anchor**: the six arm joints' goals go at once to their fresh measured
pose plus each joint's standing error (`core.HoldError`, see Safety layers), and the bus backstop's
last-sent target is rebased there (`can_speed_limit.rebase`), so a pushed or blocked arm is held where
it is and never pulled back by the old goal; (b) the gripper's torque goes off at once, and it may drop
what it holds (a zero-effort MIT frame, then the RobStride disable for that one motor; the other six
keep holding); (c) the six arm joints return to the measured rest leg by leg at the arm speed on fresh
feedback, every leg planned from the measured pose (the return helper's own fresh reading; the guard
check and the leg choice use the re-anchor's reading). Past the approach that means to the start pose
(skipped when the arm is within the 10 deg lag trip of it: it holds there with its sag), then
`core.return_waypoints` (the approach reversed). Mid-approach it means back along the legs travelled.
Each leg's straight path is checked by the guard first, and every reading during the legs goes
through the measured-speed trip. A leg that stops short but within the 10 deg lag trip of its target
on every arm joint (a fresh reading after it) counts as arrived and the return goes on: the shoulder
holds the start pose 7.45 deg short, more than the helper's 6 deg settle band; (d) `remote_can.release`
(zero effort, torque off, cameras closed), at the rest. **The arm is never released mid-air** except by
the second signal: a leg that ends further off than the lag trip, an overspeed during the return, a
failed or missing fresh reading, or a path the guard does not clear all HOLD: the goals re-anchored at
the fresh measured pose (+ standing error; with no fresh reading, the last goal held and the re-anchor
retried every 0.1 s), torque on, `RETURN HELD (...): HOLDING ... Support the arm, then press the stop
button again`, until the second signal (the button's RELEASE ALL).

Per row: IK radians to motor degrees (J1 + 180, J5 sign), grip 0..1 to motor -2.9 (closed) ..
-115.1 (open), jump check, clamp 1 deg inside the limits, then **retimed** into equal 20 Hz substeps so
no arm joint exceeds 15 deg/s (gripper 60) and all joints arrive together (the guard's straight-path
proof needs that), each substep through the box and the tank guard. The whole chunk is planned before
any of it runs, so a refusal stops the arm before the chunk starts. The guard proves each substep by a
walk along it (`tank_guard._scan`): a pose that sits on a line (where the guard cut the last chunk) can
move away from it or along another face that stays clear, never closer; the stop's leg check is the
same proof. Before 2026-11-08 it could not: every segment from a pose on a line failed at its first
sample, which froze the arm folded against the back wall in 215535Z.

**Before the IK, the slide** (`SlideTransport`; Gavin: when Dream asks for the hand below the floor it
should go down until it reaches the limit, stop going down there, and still go forward when asked, and
the jaws must not press on the glass). Each row of Dream's left hand target is first clipped into the
box's tcp limits (`core.clamp_eef_rows`, 40 mm inside far/near, 100 mm inside the sides). Then
`core.FloorSlide` raises the row's tcp z just enough that the hand's lowest mesh vertex (wrist_roll_link,
both fingers at the row's grip, and wrist_yaw_link swept over every roll angle, at the orientation Dream
asked for) sits at the keep-out floor line + `core.FLOOR_BUFFER_MM` (0 since Gavin's "sagging a bit to
slide on the bottom of the aquarium is ok, it's kinda inevitable", 2026-11-08; was 10) + the predicted
sag, so the jaws reach the glass and slide on it lightly; the tank guard's 10 mm floor press bounds the
push. x, y, orientation and grip stay as Dream asked, so the hand follows Dream down, then slides along
the floor. Dream's `eef_rot6` is the flange rotation times the arm file's tool axes (z = approach,
x = jaw), re-fit from logged replies against FK of their own joint rows (`tests/test_viscous_dream_slots.py`). The sag term is there because the arm hangs
below its command by more than the buffer: with the 90/4 shoulder/elbow gains, 27-37 mm at 350-390 mm reach
and 53 mm at 490 mm (fit 224 mm/m^2 x reach^2, residual sd 2 mm); in live 20261108T213138Z the commanded
lowest point stayed 6-10 mm above the floor line and the jaws still lay on the glass, the glass carrying
the arm's weight. Re-fit `core.SAG_MM_PER_M2` whenever `FOLLOWER_GAINS` change. The box and the tank guard
still check every substep after the IK and cut the chunk where it would leave them; `chunks.jsonl` logs
each row's floor lift (`floor_lift_mm`, beside the per_key AS SOLVED, i.e. after the slide).

## Trips (every one ends in re-anchor, gripper off, return legs to the rest, then torque off)

| Trip | Default |
|---|---|
| Jump | any arm joint target > 20 deg from the previous (J1 wrap, IK flip): chunk refused |
| Workspace box | tcp / wrist / wrist_pitch / elbow outside their boxes on any substep: chunk refused |
| Tank guard | substep rejected, or measured pose into the keep-out beyond its allowance |
| Lag | measured vs commanded > 10 deg on an arm joint for 2 ticks (gripper excluded) |
| Torque | > 12 Nm for 2 ticks (instant; RS00 peak 14), or > 6.5 Nm for 5 s (sustained; the approach lift holds ~6 Nm for 5-6 s). Visitors may push about 10 Nm for up to 5 s; the start pose holds at ~3.9 Nm on the shoulder. Was 5 Nm / 4.5 Nm for 2 s |
| Overspeed | any arm joint's MEASURED speed (successive fresh positions over their timestamps) > 30 deg/s for 2 readings, gripper excluded; in the loop and during every return leg (there it ends the return and HOLDS until the second signal). The 2026-11-08 rehearsals peaked at 20.3 deg/s on one reading, 18.0 over two, at <= 15 deg/s commanded |
| Temperature | >= 80 C (warn 70; the driver stops at 85) |
| Feedback | no fresh reply, motor fault, camera read error |
| Dream | error, malformed or non-finite chunk; inputs not checked (refused locally, not billed) |
| Start gate | rest more than 5 deg outside a limit or refused by the guard; Dream start pose outside the limits or the box; approach substep rejected by the guard: never energized |
| Speed | a planned arm substep faster than 15 deg/s (cannot happen by construction; checked before every send) |
| Stop | `stop_button.py`, SIGINT / SIGTERM / SIGHUP (handlers installed, so `nohup` cannot swallow them): re-anchor at the measured pose, gripper torque off at once, return legs to the rest at <= 15 deg/s, then all torque off at the rest (anything that keeps the arm from the rest holds instead). A second press / signal aborts the return or the hold and releases everything |
| Billing | `--max-calls` (<= 20) per run and `runs/ledger.json` (cap 100, raised only by editing it) |

If the guard says a return leg's straight path is not clear, or anything else keeps the return from
the rest (above), the arm holds (torque on, gripper already limp) and waits for a second signal (the
button's RELEASE ALL), so a person can support it before torque goes off.

## Safety layers (Gavin: "not impact with momentum onto glass in any circumstances")

Each layer stands alone: none relies on the tank guard's boundaries or on another layer.

1. **Commanded speed <= 15 deg/s** (runner): every Dream substep, approach and return leg is retimed
   to it and checked before it is sent (`speed` trip). Catches a planner or Dream asking for fast motion.
2. **Bus backstop at 15 deg/s** (`makermodslab/can_speed_limit.py`): every arm `Goal_Position` write is
   slewed from the last one sent, whatever code sent it. Catches a runner bug that skips layer 1.
3. **Tank guard and workspace box**: every substep and every measured pose checked against the keep-out
   at mesh-vertex level, and the box. Catches a path into the glass at any speed.
4. **Measured-speed trip** (30 deg/s for 2 readings): from fresh positions only, so it does not trust
   the commands or the guard. Catches what commands cannot: a push, a spring-back, a joint losing its
   hold or a controller misbehaving, in the loop and during the return.
5. **Lag trip** (10 deg for 2 ticks): the arm far from where it was told. Catches blocking, a push, a
   weak joint. With kp 30 Nm/rad a 10 deg error is ~5 Nm, so on most joints this ends a visitor's push
   before the torque tiers do.
6. **Torque tiers** (12 Nm instant, 6.5 Nm for 5 s) and **heat** (80 C trip, 70 C warning): the sustained
   tier is the heat guard, since heat comes from sustained torque (the dry shoulder ran 33 -> 65 C in under
   2 min at 5-6 Nm); the 80 C trip is the backstop.
6a. **Limp trip** (< 0.25 Nm while > 3 deg behind the goal, 1 reading; arm joints, not the gripper): a
   motor that cut out. Rehearsal 20261114T082911Z: the shoulder held the start pose at 5.1 Nm, 48 C, then
   read 0.00 Nm with fresh, continuous positions and the arm fell. Every read sends the RobStride
   clear-fault frame, so no fault is ever seen, and the driver decodes no mode or fault bits from the MIT
   reply (position, speed, torque, temperature only; the reply's CAN id is never looked at). MIT torque is kp * error - kd * speed, so a live joint 3 deg
   off makes >= ~1.2 Nm even on the soft wrists; over all 23 logged runs (17k ticks) the closest healthy
   cases were 1.29 Nm at 3.1 deg and 1.8 deg at 0.16 Nm. It fires on the tick of the drop, one tick
   (50 ms) before the measured-speed trip; it runs in the loop, the return legs and the hold.
7. **No spring-back on any stop** (the re-anchor, step 8a above): the goal moves to where the arm is,
   so the position controller's stored error cannot throw a pushed-away arm back. It keeps each joint's
   standing error (MIT position control holds the arm up with a position error: shoulder_lift kp 30
   Nm/rad at 3.85 Nm = 7.4 deg, measured 7.45), or the shoulder would drop ~7 deg on every stop. That
   error is a 1 s average moving at most 2 deg/s: a 12-18 deg/s push leaks ~1-1.5 deg into it, a slow
   push (<= ~5 deg/s) leaks most of itself, so the re-anchor still lets a slowly pushed arm move back by
   up to a few degrees.
8. **Return from the measured pose**: the return plans every leg from a fresh reading, never the
   cached command.
9. **No mid-air release**: torque goes off by itself only at the rest. A return that cannot get there
   (stopped beyond the lag trip, overspeed, a limp joint, no fresh reading, guard) holds where it is,
   re-anchored, until a person presses RELEASE ALL. Every reading the stop and the hold take (about
   10 Hz while holding) is logged to `ticks.jsonl` under its `stop: ...` phase, and the summary's
   `at_trip` holds every joint's torque, temperature, pose and lag at the trip.
10. **People**: the red button (stop; then RELEASE ALL), and Gavin at the power switch.

**The honest gap: a motor suddenly losing its hold** (the dry shoulder lost its hold at ~5.6 Nm, hot,
with no fault logged; the wet shoulder cut out at 5.1 Nm, 48 C, in 082911Z) is mitigated by the torque
and heat limits, not prevented. A motor that cuts out trips `limp` on the next reading; one that slips
while still making torque trips the measured-speed trip within about 0.1-0.15 s. Either way the other
joints hold, re-anchored where they are, until the second signal. With a limp joint the stop does not
return at all (moving the others with a dead one is motion nothing models) and prints
`LIMP: <joint> went limp ...: holding is not possible; SUPPORT THE ARM`. The dead motor is not
re-enabled: it most likely cut out on its own overload or under-voltage protection, and re-enabling it
under the same load would ask it to do again what it just refused, at the moment someone's hands are
going under the arm. Nothing here can hold a motor that is not making torque, so the arm swings about it
under gravity. Supporting the arm before pressing RELEASE ALL is the person's job.

## Known limits and findings

- The dry run's arm is a fake; the only real component exercised is Dream's client and IK, offline.
  `LabArm` (the live hardware adapter) has never run.
- At the tank trace pose (tcp 526 mm out, nearly straight), a 2 cm hand move made the elbow swing
  about 40 deg per chunk in the dry run: near full stretch the arm is kinematically sensitive. Start
  with the hand closer in (the box keeps the tcp 120-600 mm out).
- The rest pose in the tank model (it steers the IK) is Gavin's teleoped start pose from 2026-11-08:
  motor pan 176.59, lift -122.9, elbow 99.34, flex 8.56, yaw -9.0, roll -3.99; tcp 462 mm out, 143 mm
  up, inside the box. Re-record with `record_start_pose.py` and regenerate if the scene changes.
- Gripper mapping (closed -2.9, open -115.1) follows the Maker convention; confirm on the arm.
