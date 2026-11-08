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
| `dream_real.py` | The runner: `--dry-run` (default), `--preview`, `--live` |
| `dream_real_core.py` | Pure helpers: frame maps, limits, retiming, workspace box, trips, refusals, ledger, images |
| `make_tank_arm.py` | Generates the tank IK model below, with provenance |
| `tank_model/viscous_waterproof_tank.urdf` | The arm URDF with a `dream_base` root turned 180 deg and the unit's own limits |
| `tank_model/viscous_waterproof_tank.provenance.json` | Sources, hashes, checks and **`approval.status: pending_gavin_approval`** |
| `tank_model/viscous_waterproof_tank_arm.json` | Dry-run arm file only (no `tcp_calibration`; tcp copied from `viscous_arm.json`, never confirmed) |
| `workspace_box_viscous_waterproof_01.json` | Per-point boxes in the physical base frame (proposal, pending Gavin) |
| `tests/test_viscous_dream_real.py` (repo root) | 29 offline tests: pure helpers and the loop on fakes |

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
# Every safety trip fired once on the fake arm, each ending with return-to-rest then torque off:
.venv/bin/python scripts/viscous-dream1/real/dream_real.py --scenario all
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
   change (`--rest-motor-deg` with the hand-placed tank start pose: the current rest is Dream's sim pose,
   a placeholder).
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
5. **Joint signs.** J5 (`wrist_yaw`, sign -1) is photo-inferred and J6 (`wrist_roll`) assumed. Hand
   test with torque off: turn each joint by hand in the motor's + direction and confirm the model's
   pose turns the same way. Until `motor_to_urdf.json` says so, `--live` needs
   `--ack-unverified-sign wrist_yaw --ack-unverified-sign wrist_roll`.
6. **Tank guard merged.** `makermodslab/tank_guard.py` (branch `feat/viscous-dream1-tank-box`) gates
   every substep at mesh-vertex level; `--live` refuses without it. Its hardware items too: clamp or
   weight the base (it slides about 30 mm), trace the near wall.
7. **Credentials** `~/.dreamlabs/credentials` (0600, a fresh key) written by Gavin. Never printed or logged.
8. **Plan id.** Run the dry run with exactly the live settings and `--arm-file`; pass its plan id as
   `--plan-id`. Its "Joints that may move" list is the statement of what moves.
9. **Lab idle.** No session, no tank measurement (`/api/v1/sessions/current`), nothing holding the
   follower port (`lsof`). `--lab-down-ok` if the server is stopped.

Live, with Gavin at the power switch, backgrounded and relayed phase by phase:

```bash
.venv/bin/python scripts/viscous-dream1/real/dream_real.py --live --arm-file ~/viscous-dream-real/station/arms/viscous-arm.json \
  --overhead <camera> --plan-id <id> --ack-unverified-sign wrist_yaw --ack-unverified-sign wrist_roll \
  --max-calls 10 --pid-file /tmp/dream_real.pid
kill -TERM "$(cat /tmp/dream_real.pid)"   # stop; a second signal aborts the return and releases
```

## What a live run does

1. Refusals above. 2. Preflight: real cameras, real client, the first request built and stopped
locally (nothing sent): refuse unless its inputs id is recorded. 3. Connect the follower like Lab
hosting does (30 deg/s bus backstop installed first, `connect(calibrate=False)`, then zero effort and
torque off); a failed connect runs the de-energize recovery. 4. Start gate on a fresh torque-off
reading. 5. `remote_can.engage` (hold the measured pose). 6. Loop at 20 Hz: fresh feedback, trips,
5 Hz frames to Dream; one billed call at a time (ledger first), executing each 48-row chunk, holding
while the next call is in flight. 7. Every exit: return to the start pose (arm joints; the gripper
keeps its grip) at the speed cap on fresh feedback, then `remote_can.release` (zero effort, torque
off, cameras closed).

Per row: IK radians to motor degrees (J1 + 180, J5 sign), grip 0..1 to motor -2.9 (closed) ..
-115.1 (open), jump check, clamp 1 deg inside the limits, then **retimed** into equal 20 Hz substeps so
no arm joint exceeds 15 deg/s (gripper 60) and all joints arrive together (the guard's straight-path
proof needs that), each substep through the box and the tank guard. The whole chunk is planned before
any of it runs, so a refusal stops the arm before the chunk starts.

## Trips (every one ends in return-to-rest, then torque off)

| Trip | Default |
|---|---|
| Jump | any arm joint target > 20 deg from the previous (J1 wrap, IK flip): chunk refused |
| Workspace box | tcp / wrist / wrist_pitch / elbow outside their boxes on any substep: chunk refused |
| Tank guard | substep rejected, or measured pose into the keep-out beyond its allowance |
| Lag | measured vs commanded > 10 deg on an arm joint for 2 ticks (gripper excluded) |
| Torque | > 5 Nm for 2 ticks, or > 3.5 Nm (dry planning cap) for 2 s |
| Temperature | >= 80 C (warn 70; the driver stops at 85) |
| Feedback | no fresh reply, motor fault, camera read error |
| Dream | error, malformed or non-finite chunk; inputs not checked (refused locally, not billed) |
| Start gate | start more than 5 deg outside a limit, outside the box or the guard: never energized |
| Stop | SIGINT / SIGTERM / SIGHUP (handlers installed, so `nohup` cannot swallow them) |
| Billing | `--max-calls` (<= 20) per run and `runs/ledger.json` (cap 100, raised only by editing it) |

If the guard says the straight return path is not clear, the arm holds (torque on) and waits for a
second signal, so a person can support it before torque goes off.

## Known limits and findings

- The dry run's arm is a fake; the only real component exercised is Dream's client and IK, offline.
  `LabArm` (the live hardware adapter) has never run.
- At the tank trace pose (tcp 526 mm out, nearly straight), a 2 cm hand move made the elbow swing
  about 40 deg per chunk in the dry run: near full stretch the arm is kinematically sensitive. Start
  with the hand closer in (the box keeps the tcp 120-600 mm out).
- The rest pose in the tank model is Dream's sim pose (placeholder) and steers the IK; set it to the
  hand-placed start pose.
- Gripper mapping (closed -2.9, open -115.1) follows the Maker convention; confirm on the arm.
