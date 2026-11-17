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

## Environments

`--environment tank` is the default and keeps the existing aquarium files and plan id.
For the waterproof arm on its same mount on a desk, use `--environment dry_air` (2026-11-16):
`configs/viscous/environments/dry_air_desk/keepout_viscous_waterproof_01.json` and
`workspace_box_viscous_waterproof_01_dry_air.json` are selected together, including rehearsal,
preview and floor slide. The post face stays at 106 mm; Claude's later dry-air adjustments
use 15 mm post clearance and elbow x maximum 91 mm (commits `1e941965`, `4cca758f`). Other
point x maxima stay. The desk floor is -10 mm with 10 mm press (vertex backstop -20 mm):
Gavin confirms the mount bottom stands on the desk, below its 10 mm CAD floor plate;
base_link collision z min is 0. Was -28 mm from the old aquarium base. FloorSlide
reads the selected keep-out floor. Point z minima stay as coarse tool/origin bounds;
the mesh guard binds the floor. The shallow press matters with lag/limp trips off
and torque trip/cap at 14 Nm. Side/far/rim
planes and the other point bounds move outside model reach. The plan prints both files and records
the environment in `plan.json`, summaries and raw bundle metadata. Each environment has its own plan
id; use the id from a dry run with the same environment and settings.

Gavin's dry-air decision (2026-11-16): `--speed-deg-s` may be up to **45 deg/s** (default remains
15); the planner, planned-substep check and CAN bus backstop use that run speed/ceiling.
`--no-lag-trip`, `--no-limp-trip`, `--no-overspeed-trip` turn off those reasons to end the run
in approach, pause, Dream and rehearsal hold. Each flag requires `--environment dry_air`.
The plan prints `speed cap 45 deg/s (dry air)` and each disabled trip with the decision date;
the dry-air plan id binds the cap and all three flags. With overspeed disabled for the run,
its threshold may be below the run cap; it still guards the stop. All monitors keep reading:
the stop still re-anchors, judges arrival using lag, and HOLDS on limp or overspeed. **Returns
and their CAN backstop stay at <= 15 deg/s**. Gripper speed, feedback, Dream, jump, planned-speed,
keep-out/box, start gate and refusals remain. Tank retains its 15 deg/s cap, all trips and
existing plan ids. The trip defaults below describe tank; the dry-air plan shows any overrides.
The earlier dry-air torque/temperature choices remain explicit flags:
`--torque-trip-nm 14 --torque-cap-nm 14 --temp-trip-c 82`.

```bash
.venv/bin/python scripts/viscous-dream1/real/dream_real.py --internal-stub --environment dry_air --robot viscous_waterproof_01 --prompt "pick up the screwdriver"
```

## Overlapping Dream calls (dry air only, 2026-11-17)

Gavin: “can we make it fast enough?” `--overlap` asks for the next chunk while
sending the current one. Default pacing stays synchronous. `--replan-after N`
(default 30, 1..48) is the number of completed Dream rows before takeover; the
request starts N minus the median round trip in rows early (at least one row).
Speed-stretched substeps do not count as extra Dream rows. Both settings bind
the dry-air plan id. Tank refuses overlap and a non-default replan horizon;
its plan data and pinned ids stay unchanged.

The median of the last five completed calls selects pacing: above 700 ms,
finish the whole chunk before asking again; at or below 700 ms, overlap.
A change is announced in the console. In particular, an 800 ms connection
uses synchronous pacing under this rule. A late reply consumes the remaining
chunk, then the arm holds its last commanded pose until it arrives. The first
call and other calls sent while holding install whole. A call sent during
motion drops `round(elapsed_seconds * 20)` stale rows, up to its whole horizon.

The installed client's `dream.seam_join` fits/blends the new window onto the
actual last command and the next guarded substeps. This extra anchor matters:
client IK/smoothing does not know which rows our speed/box/keep-out checks
stretched or clipped. Every blended row goes back through `plan_chunk` and its
normal jump, limit, workspace, guard and speed checks. The stateful guard's
planning reference returns to the actual command; its original start pose and
clearance relaxation stay intact. Floor slide and box clipping still happen
inside SlideTransport before IK. All stop and feedback paths remain active.

Stopping sets cancellation immediately to prevent another request. A separate
cleanup thread lets the outstanding request finish or time out, joins retained
worker handles and closes the client. It never interrupts the socket: doing so
can trigger the transport's stale-connection POST retry outside the ledger.
Reply collection polls thread completion without waiting. Cleanup-start failures
are logged, and an inner finally guarantees the arm's existing stop still runs.
The arm stop never waits for request teardown; the background join may finish
later. An uncertain sent call stays chargeable in the existing ledger. The worker never touches the arm.

`--internal-stub --overlap` makes no Dream request but still needs the installed
client's seam helper from `--dream-venv`. `--stub-latency-s` sets virtual latency
(default 1.5 s) without sleeping. Dry-air `summary.json` includes
`pacing_timeline`: send/reply times, completed/remaining rows, dropped rows and
idle time per request. These are simulated queue-idle times; real planning/IK
cost and hardware tracking still require Gavin's supervised review/run.

## The recipe that first grasped (2026-11-17): use this, then change one thing at a time

For a NEW setup (e.g. back into the tank), first walk through [SETUP_RUNBOOK.md](SETUP_RUNBOOK.md): every
step we took on 11-16/17 to get here, with what to redo and what went wrong.

Dry air, waterproof arm on the desk. Two live runs, both grasped and lifted the cube:
`live-20261117T073809Z` (grasp + lift at calls 8-9, released after the back-post guard blocked
the pull-back) and `live-20261117T074056Z` (cube reached in 3 calls, grasped twice, holding it
lifted at the end). Every earlier run (screwdriver, our arm as Dream's LEFT arm) closed the jaws
in call 1 and never reached. What changed, in order of importance:

1. **Our arm is Dream's RIGHT arm** (`--arm-slot right --wrist-r left --wrist-l black`). Gavin:
   "this is so obviously the right arm!!! its based on the overhead camera placement relative to
   the arm". Dream's own single-arm convention is also right (`dream_ik/specs.py:656`). As the
   left arm, Dream drove the phantom right arm more than ours in 38 of 42 calls.
2. **The object:** a 25 mm blue cube ("done many times before"), prompt `"pick up the blue
   cube"`, its centre 32.5 cm forward of J1 (= 43.1 cm in front of the post's inner face) and
   10 cm to the left seen from behind the arm (`gavin-linux:~/cube-spot/REPORT.md`: clear of the
   post for a pick and pull-back, ~29 px in the wrist image).
3. Jaws open at the start; nobody touches the object during the run.
4. Already in place: the KD camera as `head` overhead (same pitch as the Maker station's), desk
   floor -10 mm (`fdb8580a`), the camera lock (restart Lab after pulling), synchronous pacing,
   `--dream-state measured`, 18 calls.

```bash
# dry run first: prints the plan id
.venv/bin/python scripts/viscous-dream1/real/dream_real.py --speed-deg-s 45 --no-lag-trip \
  --no-limp-trip --no-overspeed-trip --torque-trip-nm 14 --torque-cap-nm 14 --temp-trip-c 82 \
  --environment dry_air --robot viscous_waterproof_01_dry_air \
  --arm-file ~/viscous-dream-real/station/arms/viscous-waterproof-dry-air.json --keep-arm-rest \
  --prompt "pick up the blue cube" --overhead head --wrist-r left --wrist-l black \
  --arm-slot right --dream-state measured --max-calls 18 --pause-s 10
# then the red button, then the same command + --live --plan-id <id> --pid-file /tmp/dream_real.pid
.venv/bin/python scripts/viscous-dream1/real/stop_button.py --pid-file /tmp/dream_real.pid
```

What still limits it: after lifting, Dream pulls the hand back toward the base and the back-post
guard skips those rows (calls 11/15/16 of the first run, 17 of the second), so it stalls mid-lift
and may let go. Next: start pose ~5 cm further out/left (`cube-spot` report), then `--overlap`
(reviewed, untested live), then empirical gravity compensation (branch
`feat/viscous-gravity-comp`).

**In the water tank** (`--environment tank`: 15 deg/s, torque/temperature/lag/limp/overspeed
trips on, tank keep-out with the -28 mm floor): keep the recipe's slot, cameras, cube and prompt;
use the tank arm file and the robot record whose cameras are the KD overhead (`head`) and the
wrist camera (`left`, rotated 180): `--environment tank --robot viscous_waterproof_01_dry_air
--arm-file ~/viscous-dream-real/station/arms/viscous-waterproof-tank.json --keep-arm-rest
--prompt "pick up the blue cube" --overhead head --wrist-r left --wrist-l black --arm-slot right
--dream-state measured --max-calls 18 --pause-s 10` (dry run, then `--live --plan-id`). Place the
overhead camera so the arm appears on the RIGHT, as on the desk, and put the cube where the wrist
camera sees it from the tank start pose.

## Choosing Dream's arm slot (2026-11-17; tank allowed since 41feb738)

Gavin identified our arm as the RIGHT arm from its position in the overhead image. Allowed in the tank too (41feb738).
`--arm-slot right` sends our joints/grip as `state.right_*`, executes `q_r_chunk`
from `action.right_*`, and clips/slides only the right EEF targets before IK.
The left side is idle at the same arm-file rest/open grip previously used for
idle right, and its answers are dropped. The default `--arm-slot left` preserves
existing state, motion, plan JSON and ids. Tank refuses right at the CLI, plan,
client-adapter and episode boundaries. A right plan explicitly binds the slot.
Overlap uses the chosen side for its commanded seed, stale-row removal and seam.

Camera flags name Dream slots independently of the arm flag. To put the physical
wrist camera called `left` on Dream's right: `--overhead head --wrist-r left
--wrist-l black --arm-slot right`. `--wrist-r camera:left` remains supported;
`--wrist-r copy` still copies wrist_l. No automatic camera swap occurs.

A custom arm file is registered by Dream's client as the same ArmSpec on both
sides, with no side-base placement. Each EEF is in that file's base frame.
`dream_base` remains x-forward/y-left/z-up on our existing mount, with its
180-degree turn relative to the physical `base_link`. No extra right-side
translation, reflection, YAM mount offset or URDF edit is required. The idle
left also uses this custom model, preserving the former idle-right report.
Dream's YAM-trained checkpoint still receives zero joint slots for a custom
arm; its EEF/grip are our measured FK/grip (or the existing commanded-state
option). Changing the slot does not change that client convention.

`chunks.jsonl` includes `arm_slot`, both returned joint chunks, and `raw_reply`
(both chunks and per_key before our overlap splice). The idle side's joint and
EEF replies remain unaltered, including on a planner refusal. The top-level
working chunk is the one actually offered to the planner after any splice;
substeps remain the guarded/retimed motor commands. The raw EEF values here
are the client's reply, after SlideTransport; the idle EEF side is never slid.

Offline proof, no DreamClient, transport, API key, hardware or cameras:

```bash
.venv/bin/python scripts/viscous-dream1/real/check_slot_roundtrip.py
# Optional: --arm-file <arm.json> --ticks <recorded-ticks.jsonl> --solve tick|traj --out <proof.json>
```

The checker blocks network calls before loading Dream, uses recorded measured
feedback, compares client FK to independent URDF FK and to the physical base
frame, then feeds those EEF poses through ClientIK.solve for both slots.
It checks joint/TCP numerical error and identical left/right results. Default
inputs are committed; the installed Dream client is optional in pytest. On
tonight's 2026-11-17 dry-air recording and actual arm file, the trajectory solve
stays within 0.0064 deg and 0.009 mm TCP error over 48 rows; small nonzero drift
comes from the client's regularized solver. Detailed evidence and the complete
dry-run plan blocks: `~/dry-air-ref/REPORT-right-slot.md`. Claude reviews before
Gavin's powered run. This change has only been exercised offline.

## Files

| File | What it is |
|---|---|
| `dream_real.py` | The runner: `--dry-run` (default), `--preview`, `--live`; `--rehearse` (no Dream) alone or with `--live` |
| `stop_button.py` | The big red button: an always-on-top window that sends SIGTERM to the runner's `--pid-file` pid |
| `run_data.py` | Raw run bundles outside git (a ring of the last 10, KEEP to keep one forever) and their CLI |
| `dream_real_core.py` | Pure helpers: frame maps, limits, retiming, workspace box, trips, refusals, ledger, images |
| `check_slot_roundtrip.py` | Offline measured-FK/EEF/IK proof for both slots, including the physical base transform |
| `make_tank_arm.py` | Generates the tank IK model below, with provenance |
| `tank_model/viscous_waterproof_tank.urdf` | The arm URDF with a `dream_base` root turned 180 deg and the unit's own limits |
| `tank_model/viscous_waterproof_tank.provenance.json` | Sources, hashes, checks and `approval` (approved by Gavin 2026-11-08, provisionally, on condition every live motion stays <= 15 deg/s) |
| `tank_model/viscous_waterproof_tank_arm.json` | Dry-run arm file only (no `tcp_calibration`; tcp copied from `viscous_arm.json`, never confirmed) |
| `workspace_box_viscous_waterproof_01.json` | Per-point boxes in the physical base frame (proposal, pending Gavin) |
| `tests/test_viscous_dream_real.py` (repo root) | Offline tests: pure helpers, the loop on fakes, the stop button's logic |

## Raw run data (every run, the last 10 kept)

Gavin (2026-11-14): "all run from now to the future, keep all the last 10 run logs, like all the raw
data, if possible even camera in the future, so we can always go back in time to figure out what is
wrong. then discard by default unless needed to be kept".

Every run of `dream_real.py` (dry, rehearse, preview, live) also writes a raw bundle to
`~/.makermods/makermodslab-viscous/run-data/<run-name>/` (`run_data.RUN_DATA`; `--run-data DIR`
elsewhere, `--no-run-data` none): `meta.json` (command line, git SHA, `git diff --stat`, host, Lab
home, ports, cameras), `console.log` (everything printed), `ticks.jsonl` and `chunks.jsonl` mirrored as
written (every tick at full rate, the stop path and the hold included; per scenario in a dry run),
`inputs/` (keep-out, box, arm file, URDF, robot record), `run/` (the runs/ folder at the end: plan,
summary, Dream logs, preview frames), and, live with Dream only, `frames/` (below). Each summary
names its bundle (`run_data`). The small run folders under `runs/` are unchanged: they stay the
evidence the docs point to.

The ring: at the start of each run, the oldest bundles beyond the newest 10 are deleted. A bundle with
a `KEEP` file is kept forever and does not count. Prune never deletes an `ACTIVE` bundle whose pid is
alive, a bundle written in the last 2 min, a symlink, or anything that is not a folder directly under
`run-data/`.

```bash
.venv/bin/python scripts/viscous-dream1/real/run_data.py list
.venv/bin/python scripts/viscous-dream1/real/run_data.py keep live-20261114T084532Z "return refused, shoulder cut out in the hold"
.venv/bin/python scripts/viscous-dream1/real/run_data.py unkeep live-20261114T084532Z
.venv/bin/python scripts/viscous-dream1/real/run_data.py prune
```

Cameras: no extra read. The frames the live loop already hands to Dream (5 Hz, every record camera,
full 640x480) go to a background writer as JPEG q80, `frames/<camera>/<t>.jpg` plus
`frames/index.jsonl`; a full queue drops a frame (counted in `frames/stats.json`) rather than wait.
About 50-85 KB a frame, so two cameras are ~30-50 MB a minute; it stops by itself past 2 GB in a run
or under 4 GB of free disk. Rehearsals open no cameras, and the stop path closes them, so frames end
at the stop: the stop and the hold are in the ticks, not on video.

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
Each leg's straight path is checked by the guard first (`check_return_segment`: a start up to the
15 mm tracking allowance inside a requirement may leave it along a path on which nothing gets more than
0.5 mm worse than at the start; live 20261114T084532Z held mid-air because its re-anchored goal sat
1.84 mm over the right-wall line), and every reading during the legs goes through the measured-speed
and limp trips. A leg that stops short but within the 10 deg lag trip of its target
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
10. **People**: the red button (stop; then RELEASE ALL), and Gavin at the power switch. Under the
   button is the runner's own status (`<pid-file>.status.json`, written every tick, in every phase, the
   stop path and the hold included: run state, per-joint limp/torque/temperature, the time of the last
   fresh reading, one line), not just "pid alive" (Gavin, 2026-11-14: "the arm will suddenly die, but
   the line under the big stop button still says alive, so thats wrong"). It reads, in large type:
   `SHOULDER WENT LIMP: SUPPORT THE ARM, then RELEASE` (blinking red), `HOLDING MID-AIR: support the
   arm, then press RELEASE` (orange; the button is then the release press, by click only), `NO DATA FROM
   THE RUN` (purple: the file or the arm's readings older than 1 s while the pid lives), `RUNNER GONE`
   if it died energized, and `RELEASED: torque off` at the end. A missing or corrupt file falls back to
   the pid check.

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

## Empirical dry-air gravity sweep (offline build, 2026-11-17)

Goal B1/B3: Gavin wants the waterproof unit's dry-air holding load measured
empirically because the waterproof connectors make its CAD inertials inaccurate.
`--rehearse --gravity-sweep --environment dry_air` replaces the rehearsal hold
with a deterministic STAR around the arm file's Dream start pose. The executor
is still `Episode`: its fresh readings, trips, status file/button, re-anchor,
return, gripper release and HOLD-on-refusal are shared with rehearsal.

```bash
PYTHONPATH=$PWD ~/Documents/makermodslab-viscous/.venv/bin/python scripts/viscous-dream1/real/dream_real.py \
  --rehearse --gravity-sweep --environment dry_air --no-run-data --out /tmp/gravity-sweep
```

This form opens no hardware/cameras, imports no Dream client, reads no
credentials and makes no Lab/network query. It plans in the torque-off start
gate, prints the coverage/predictor/loads/duration, then runs the complete
schedule on `FakeArm`. `--no-run-data` keeps this example's evidence in `/tmp`;
omit it to retain the ordinary raw bundle. Fake samples are marked and the
fitter refuses them. The fake's nominal heating is slowed for a long sweep;
its telemetry is not an empirical load/thermal prediction.

Sweep joints that may move: **shoulder_lift, elbow_flex, wrist_flex, wrist_yaw,
wrist_roll**. Pan holds the hub's command during the STAR. The normal approach
and returns may also move pan. Engage holds the measured gripper once; no
subsequent gripper targets are sent during approach or sweep, and the ordinary
stop releases it. Geometry uses its measured starting opening. As with the
existing rehearsal, a person must not change the opening during execution.

Defaults and checks:

- `--sweep-seed 0 --sweep-poses 30 --sweep-hold-s 2 --sweep-max-torque-nm 6
  --sweep-hot-c 65 --sweep-start-at 0`. Holds must be 1–60 s to provide a full
  last second; planning torque may be lowered from 6 Nm and hub heat from 65 C.
- Motor-degree box: the full five-joint soft range less 8 deg (the broad
  hub box exceeds those limits). `--sweep-box` accepts
  JSON absolute bounds for exactly those five joints. The approach uses a radial lift/elbow/flex vector with maximum
  component 4 deg, then tries per-joint signs and a common 4 deg vector. Both
  directions must plan; approach endpoints retain the 8 deg limit margin.
- Draw max(96, 24*count) candidates using numpy's seeded generator. Prove all
  retained candidates before farthest-point thinning over all five joints,
  reserve six poses in tonight's measured pocket with varied yaw/roll and order
  from that pocket outward. Conditional single-joint rays report the first
  cap/guard/limit boundary from a pocket reference; these are conditional
  bounds, not independent joint limits for arbitrary combinations. Half the
  random pool retains the earlier narrower box for density. Refuse if fewer
  than the requested count (or six pocket poses) pass.
  The STAR is hub -> plus approach -> pose/hold -> minus approach -> pose/hold
  -> hub, Both approaches are mandatory; a rejected direction rejects that vector
  or candidate. The sign label names the vector side; samples retain actual
  per-joint motion signs for the friction fit.
- Every 20 Hz STAR substep must pass strict limits, workspace and mesh guard
  checks, including its predicted sag (`q - degrees(torque/kp)`, no clipping
  through a limit). Holding load must fit the cap everywhere on the legs,
  not just at the sampled poses. Each commanded and sagged substep proves a
  straight return to the hub. The existing stop's within-lag shortcut to the
  first reverse-approach waypoint is also proved, using its actual return
  guard. These checks do not alter the normal stop or its runtime refusal.
- Refuse tank, Dream mode, any trip-disabling flag, speed >15 deg/s, or weaker
  trip thresholds than the runner defaults. The runtime torque tiers remain
  12 Nm / 6.5 Nm for 5 s; lag, limp, overspeed and temperature remain enabled.
  Approach, STAR and return all stay <=15 deg/s, including the CAN backstop.
- On fresh feedback during a pose's legs or holds, measured arm torque
  above the planning cap +0.5 Nm skips the pose. Its sent movement substeps
  are replayed in reverse back to the hub, with fresh feedback, every trip,
  measured guard and planned-speed check still active. The ordinary stop
  button wins on any tick. Two consecutive skips end through the normal
  stop after retracing to the hub. `samples.json` records `skipped_poses`;
  partial holds remain in ticks.
- At initial/subsequent hub visits, any winding >=65 C ends via `stop()` with
  reason `sweep paused: hot`. The summary/phase prints `--sweep-start-at K`;
  same seed/count/box/predictor gives the same poses, skipping the first K.
  K equal to count means all poses are complete.
- `ticks.jsonl` names every hold/direction. `samples.json` stores each completed
  hold's zero-based pose index, last-motion sign per joint, approach side,
  medians of pos/cmd/torque/temp over the last 1.0 s, and tick count. It is
  mirrored atomically to the raw bundle as holds finish; a partial hold is
  kept in ticks but not advertised as a completed sample. `sweep-plan.json`
  records the actual checked poses and substeps. Runtime trips still use
  measured feedback and are the protection against predictor error.

The sweep plan id binds seed/count/hold/load/heat/resume/box, fixed delta and
margin, candidate count, gains/map/CAD hash, predictor-file hash and fallback
reference hash. Normal plans receive none of these fields: four pinned tank /
dry-air x Dream / rehearsal IDs remain byte-identical. A changed file or
robot record requires regenerating the id, including on a different station.

### Fit and planning predictor

`gravity_fit.py` accepts real dry-air `samples.json`, `ticks.jsonl` or their
run folders; metadata must establish dry air, and fake runs are refused.
For raw ticks it uses nonoverlapping >=0.5 s windows with every arm joint's
position span <1 deg/s, no gaps >0.15 s or trips, and the entire window at
least 1 s after a phase change. Only energized approach/Dream/hold/sweep
phases qualify; stop/support readings do not. Last commanded motion per joint
sets the Coulomb sign. Static external contact cannot be distinguished from
gravity by this filter, which is a limitation of the existing logs.

```bash
PYTHONPATH=$PWD ~/Documents/makermodslab-viscous/.venv/bin/python scripts/viscous-dream1/real/gravity_fit.py \
  ~/dream-run-archive/mac-run-data/live-20261117T*/ticks.jsonl \
  ~/dream-run-archive/mac-run-data/rehearse-live-20261117T*/ticks.jsonl \
  --no-offsets --no-friction \
  --save-selected configs/viscous/viscous_waterproof_01/evidence/gravity-dry-air-selected.json
```

**Offsets and friction are pinned to zero for the logs-only fit (2026-11-17).** On the narrow logged
pocket a constant per-joint offset cannot be told from gravity: the first fit put -6.4/-7.8/-5.5 Nm into
the shoulder/elbow/wrist offsets and +10.9/+13.7/+6.7 Nm into gravity at the start pose, and the
feedforward (gravity terms only) would have commanded 2-2.5x the measured holding torque (2.98/4.61/0.54
Nm at the 023712Z/024005Z holds). Refit: 2.95/4.37/1.34 Nm there. `gravity_comp.load` now refuses a fit
with any offset above 0.3 Nm. The logs have one approach direction per pose, so friction is not
separable either; the sweep (both directions) is what identifies it.

**Measured-pose gate.** Between the logged poses, still inside every joint's range, the fit swung to
+-12 Nm: the logged poses lie along Dream's correlated trajectories, not across the box. So the fit
carries `support` (every pose it was measured at) and the feedforward is multiplied by
`gravity_feedforward.support_weight`: 1 within 4 deg (Euclidean over the five gravity joints) of the
nearest measured pose, cosine to 0 at 12 deg, on top of the per-joint `region_deg` fade. A provisional
fit without `support` is refused. About 60 % of tonight's energized dry-air Dream ticks lie within 4 deg
of a measured pose (the start pose and the hover pocket); big moves between them run without
feedforward.

The output is `configs/viscous/viscous_waterproof_01/gravity-dry-air-fit.json`
and its adjacent Markdown report. The selected windows are committed as
portable evidence with source hashes. Inertia-free kinematic regressors use
`motor_to_urdf.json` in both position and torque coordinates; Pinocchio locks
the gripper at neutral. SVD identifies scaled static parameter combinations
(columns and basis stored in JSON), then fits zero-centered ridge (0.01), six
offsets and six signed Coulomb terms. Cross-validation recomputes the basis
using training runs/poses only: leave one run out for logs, one pose out for
sweep holds. CAD supplies only the reference RMS. Tests change every CAD
mass/COM and verify that both the regressor and empirical solution stay the
same. Existing unit-1 `GravityModel` callers retain identity coordinates.

The log-only fit is **provisional and refused by default for feedforward**;
only the explicit runner override below permits it (the environment installer
still refuses it).
It has narrow coverage and poor wrist-roll R²; offsets/friction are weakly
separated. Planner usability requires the matching robot/condition/map/URDF,
finite parameters/friction/CV errors, static rank >=6 and worst held-out RMS
<=1 Nm. Coulomb magnitude is added conservatively for load/sag planning. If
unusable, CAD is scaled per joint against the nearest archived measured
holding pose (not a fresh torque-off reading, which cannot measure holding
load); near-zero CAD axes keep scale 1. The plan explicitly names this
fallback and source. No new calibration measurement is claimed.

No powered run, Dream call or OS signal was performed for this build. The
later live command uses the same arguments plus `--live --plan-id <id>
--pid-file /tmp/dream_real.pid`, after the separate review and Gavin's
existing button/power-switch procedure. Never use the older October
collector as this sweep's executor.

Offline validation evidence:
`configs/viscous/viscous_waterproof_01/evidence/gravity-sweep-offline-20261117.json`
contains the 30 checked poses, coverage/loads, 36 completed fake holds, plan id
`0c39930eda`, normal stop result and test counts. The fake ran for 331.05 s
including approach and return. Focused files: runner 104, gravity model 10,
sweep/fitter 22, feedforward 16 (152 passed). Ruff passes. Full suite:
4,784 passed, 18 skipped, three failures described in `docs/viscous-haptics.md`.

To reproduce full-suite validation while keeping the offline signal/device/
network rules (the helper is opt-in and leaves ordinary pytest unchanged):

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=$PWD:$PWD/scripts/viscous-dream1/real \
  ~/Documents/makermodslab-viscous/.venv/bin/python -m pytest -q --maxfail=0 -p offline_test_guard tests/
```

## Dry-air gravity feedforward (2026-11-17)

Gavin: "i want all technique to work on air first before water". `--gravity-comp
SCALE` (0..1, default 0) is supported for rehearsal and Dream in **dry_air only**.
It refuses `--gravity-sweep`, because the sweep measures raw holding torque.
`--gravity-fit PATH` defaults to the waterproof unit's `gravity-dry-air-fit.json`.
The fit's robot must equal the selected record's name, its condition must equal
`dry_air`, and its six joints, commissioned physical map and URDF must match.
A provisional fit is refused unless `--allow-provisional-fit` is explicit. The
plan prints the selected path, SHA256, scale and provisional status; the run
bundle copies the fit. Replacing the fit requires a new plan ID. In particular,
a fit for `viscous_waterproof_01` refuses the distinct `_dry_air` robot record;
use the correct commissioned record/fit pair rather than changing this check.

At zero, no fit is read, no hook is installed, no `ff` field is logged and the
legacy plan JSON/IDs, ticks, fake events and summaries are unchanged. Eight
committed reference hashes came from pre-feedforward `be5b6d40`, spanning both
environments, rehearsal/Dream (internal stub), and both arm slots.

The installed MakerFollower sends each joint through `bus.write(Goal_Position)`.
The RobStride bus's `_mit_control(motor,kp,kd,q,0,tau_ff)` encodes torque in Nm
in the MIT packet; normal driver writes hard-code its last argument to zero.
`maker_can.py` only changes the gs_usb connection; it does not alter MIT effort.
The reused `gravity_feedforward.install` follows `_base` beneath
`RateLimitedRobstrideBus`, preserving the existing slew limit. It evaluates
kinematic gravity at the follower's last measured, whole-turn-corrected motor
pose. Telemetry offsets and Coulomb friction are excluded from commanded
feedforward; each torque is clamped to 6 Nm, with the existing 2 s ramp from
the first nonzero-gain arm position write. The gripper always gets zero ff.
Install follows the backstop and successful connect/initial neutralize and
precedes engage. No inertial changes, gains or torque thresholds were made.

Write-path audit of the installed fork and this runner:

| Path | MIT effort with compensation enabled |
|---|---|
| connect handshake/configuration, initial neutralize/disable | Hook not installed yet; original zero ff; disable protocol frames are separate |
| engage's `remote_can.neutralize` | Kp=Kd=0 frames pass to the original writer with zero ff; do not start the ramp |
| engage's backstop fresh preload, motor enable, measured-pose hold | Preload/hold position frames carry ramped ff; enable's protocol frame is separate |
| approach, rehearsal hold, Dream `remote_can.send_action` -> follower -> slew wrapper | Limited position targets carry ff on all six arm joints |
| `can_speed_limit.rebase` / re-anchor | Fresh measured+standing-error goals intentionally bypass slew, then pass through the innermost ff hook |
| `release_gripper` | Gripper Kp=Kd=0, zero-effort Goal_Position, then its disable frame; the arm's ff stays on |
| rest-return helper / `Fresh.send_action` / settling hold writes | Follower position writes retain ff and pass through the slew wrapper |
| mid-air HOLD | Last re-anchored MIT goals and their ff remain active; feedback continues; HOLD does not advance the ramp by sending new motion frames |
| verified rest ramp-down | Same final arm goals at 20 Hz; live ff scale falls to zero over 1 s; gripper stays disabled |
| final `LabArm.release` -> `remote_can.release` | Detach ff hook first; original neutralize sends zero-effort MIT frames, then independent `de_energize_can_device` disables/disconnects |
| failed connect / de-energize recovery | No hook was installed before successful connect. The recovery's disable/disconnect does not depend on a model or wrapped position writer |
| second signal / RELEASE ALL | Skip any remaining return/ramp/HOLD as today, detach hook and release immediately |

The stop deliberately keeps ff on during re-anchor, return and any HOLD:
removing it in mid-air changes equilibrium by tau/kp, and the learned standing
error belongs to the compensated equilibrium. Only after the existing rest
arrival verdict does it hold the final goals and ramp ff to zero over 1 s,
then release. Every ramp tick reads feedback, checks all trips and the measured
guard, logs `ff`, and updates button status. A ramp fault holds with the current
remaining ff and waits for RELEASE ALL; it does not finish removing support.
The second signal is checked every ramp tick (50 ms), without waiting a second.

Effects on the existing protections:

- FloorSlide's sag coefficient becomes `SAG_MM_PER_M2 * (1-scale)` in the actual
  pre-IK transport and in the plan ID. Geometry, buffer and keep-out are unchanged.
  This linear sag approximation is subject to fit error and 6 Nm ff saturation.
- HoldError continues averaging the **raw** command-minus-position error, now
  the residual sag with ff present. Re-anchor preserves that error while ff
  remains present. Lag remains raw position tracking error; overspeed remains
  measured motion, independent of ff.
- Torque tiers still use the motor's **total measured torque**, including ff.
  Never subtract ff when checking heat/load protection.
- With ff enabled only, limp tests zero measured torque against the combined
  MIT demand `kp*(cmd-pos)+ff`, expressed as equivalent position error
  `cmd-pos+degrees(ff/kp)`, retaining 0.25 Nm / 3 deg / one reading. Otherwise a
  healthy joint with opposing ff and stiffness can have zero total torque at
  >3 deg raw error. This correction applies in the loop, return, HOLD and rest
  ramp and binds the plan ID. Raw lag/HoldError do not use this equivalent goal.
  It omits the velocity damping term, as the original limp rule did.
- `ticks.jsonl` adds the six-joint `ff` dictionary only when scale >0, including
  stop readings. These values describe the latest successfully sent effort.

### Gavin's rehearsal progression (commands below were NOT run live)

The current log fit is provisional. These IDs were generated by matching
30-second FakeArm rehearsals using this checkout's **default robot record**,
arm file and current fit SHA256
`45d29ebdefc8c72bf3ea1b0610141753e2d327adad8ba863d254564f2fa5b09c`.
All three completed the hold, returned and ramped to zero. Re-run the same
command without `--live --plan-id ...` on the arm machine to regenerate its ID
if its record/port, arm file or fit differs. The sweep's measured fit should
replace the provisional fit; remove the override and regenerate all IDs then.

| Scale | Plan ID | Floor sag coefficient mm/m² |
|---|---|---:|
| 0.25 | `825105ac75` | 168 |
| 0.5 | `113cb1400d` | 112 |
| 1.0 | `6caee7f868` | 0 |

Run in order with the existing button/power-switch procedure. Each approaches
with all seven joints available, holds 30 s at the start pose, releases the
gripper on stop, and returns the six arm joints at <=15 deg/s.

```bash
PYTHONPATH=$PWD ~/Documents/makermodslab-viscous/.venv/bin/python scripts/viscous-dream1/real/dream_real.py --rehearse --environment dry_air --gravity-comp 0.25 --allow-provisional-fit --hold-s 30 --live --plan-id 825105ac75 --pid-file /tmp/dream_real.pid
PYTHONPATH=$PWD ~/Documents/makermodslab-viscous/.venv/bin/python scripts/viscous-dream1/real/dream_real.py --rehearse --environment dry_air --gravity-comp 0.5 --allow-provisional-fit --hold-s 30 --live --plan-id 113cb1400d --pid-file /tmp/dream_real.pid
PYTHONPATH=$PWD ~/Documents/makermodslab-viscous/.venv/bin/python scripts/viscous-dream1/real/dream_real.py --rehearse --environment dry_air --gravity-comp 1.0 --allow-provisional-fit --hold-s 30 --live --plan-id 6caee7f868 --pid-file /tmp/dream_real.pid
```

Watch command-minus-position and vertical sag shrink without reversal/lifting,
oscillation, guard contact, heat/total torque rise or trips. Check `ff` rises
smoothly during startup, remains present on return/HOLD, and reaches zero at
rest before release. At scale 1 the current provisional fit saturates shoulder
and elbow ff at 6 Nm at this start pose: scale 1 is therefore not full gravity
cancellation. Compare settled holds with:

```bash
PYTHONPATH=$PWD ~/Documents/makermodslab-viscous/.venv/bin/python scripts/viscous-dream1/real/gravity_comp_report.py /path/to/run
# ticks.jsonl or the run's nominal/ directory also works; --arm-file restores a moved bundle's TCP file
```

The report prints per-joint median raw command-minus-position, measured torque
and ff, and median vertical TCP sag from command/position FK on the committed
tank-model URDF through the commissioned joint map (pan shift included).
It uses only phase `hold`, discarding its first second. Positive sag means the
measured TCP is below its command. It does not confuse return/HOLD readings
with the initial rehearsal hold.

For Dream after these rehearsals, add `--gravity-comp S --gravity-fit PATH`
(and the explicit provisional override only if still needed) to the existing
reviewed dry-air/right-slot recipe with the matching robot/fit, generate its
new plan ID offline, then use that ID. No Dream request was made for this build.
Do not reuse a rehearsal ID for Dream.

The opt-in nominal FakeArm uses a bounded synthetic load matched to the chosen
ff scale, easing it in away from its supported rest; explicit sag fixtures
instead lose `degrees(ff/kp)` of their sag. This verifies sequencing and the
wrapper, not real tracking, fit accuracy or thermal behavior. The zero-sag
scale-zero fake is unchanged. The report's zero fake sag is not an arm result.

Offline verification: 262 passed +31 subtests across the runner, gravity model,
sweep, feedforward, compensation, right-slot/slot mapping, real CAN wrapper and
remote-CAN tests. The 23 compensation cases include eight legacy zero-scale
run hashes, fit refusals, changed-fit refusal before connect, ramp-in, hold and
return support, zero at rest, mid-air HOLD, immediate second-signal release,
ramp send/feedback faults, limp cancellation, floor sag and the FK report.
Portable results: `configs/viscous/viscous_waterproof_01/evidence/gravity-comp-offline-20261117.json`.


### Part 3 priority: measured region and live-input rehearsal (2026-11-17)

The earlier default-record rehearsal IDs above are superseded for tonight's
`viscous_waterproof_01_dry_air` run. This record uses follower calibration
`viscous_waterproof_01`; the fit declares that exact dry-air record as an alias
of the same physical unit. Arbitrary names still refuse; the plan binds the
selected record, fit hash, five-joint `region_deg` and `fade_deg`.

The fitter now derives region from measured min/max of the samples actually
used, never the requested sweep box. Pan is ungated. Each gravity joint has
weight 1 inside its measured interval, then `(1+cos(pi*d/fade))/2` for distance
d outside, and 0 at/after fade (10 deg). The product multiplies all six arm
feedforward torques at the measured pose, before the existing clamp/ramp.
`ticks.jsonl` records `w`, `w_per_joint` and the latest sent `ff`. The hook
stays installed through return/HOLD; region fading still applies there. The
normal one-second rest ramp and immediate RELEASE ALL are unchanged.
Full floor-sag allowance is retained because support can vanish outside the
region; a global `(1-scale)` reduction is no longer justified.

The logs-only fit was regenerated from the 339 committed selected windows.
It remains provisional and requires `--allow-provisional-fit`. The hub is
outside the strict region on elbow/flex/roll but inside their 10-degree fades.
Six FakeArm rehearsals use the supplied live Lab home, arm file, keep-rest
and right slot: scales 0.25/0.5/1, with both default trips and Claude's dry-air
flags. Exact outputs, plan IDs, hub weights/torques and unexecuted live commands
are in `~/codex_runs/gravity-p3/partA.md`. No hardware or Dream was used.
