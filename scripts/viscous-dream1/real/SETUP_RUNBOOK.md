# Bringing Dream-1 up on a new Viscous setup: the steps of 2026-11-16/17

Gavin (2026-11-17): "workflow to be saved means all the steps we did from earlier today. cuz the
tanks setup will be different so we will have to repeat a lot of these steps". This is that list,
in the order we did it on the desk ("dry air"), ending in the first grasps
(`live-20261117T073809Z`, `live-20261117T074056Z`). Each step says what to redo for a new setup
(e.g. back into the water tank) and what went wrong the first time. The recipe that worked is in
[README.md](README.md), "The recipe that first grasped"; the dated history is in
`docs/viscous-haptics.md` B3.

All commands run from the repo root on the Mac that holds the arm's USB. Big analysis jobs go to
gavin-linux (Codex there), never the Mac.

## 0. Power, bus, Lab

1. **Motor power on.** A run on 11-17 failed at connect with "Handshake failed. did not respond:
   [all seven]" because the arm was unplugged from power. Read-only check that every motor answers
   (energizes nothing):
   `MAKERMODSLAB_HOME=~/.makermods/makermodslab-viscous .venv/bin/python scripts/viscous-can-ping.py --port /dev/cu.usbmodem2050389538461`
2. **Lab on the current code.** After every `git pull`, restart Lab while the arm is idle
   (`kill <lab pid>`, then `MAKERMODSLAB_HOME=~/.makermods/makermodslab-viscous nohup .venv/bin/makermodslab &`).
   An old Lab answers HTTP 405 on `POST /api/v1/camera-preview/release` and the runner then refuses
   (camera lock, `bc8354f6`). Check: `curl -X POST localhost:8000/api/v1/camera-preview/release` -> 200.

## 1. Robot record and cameras

1. Make (or copy) a Lab robot record for the setup under `~/.makermods/makermodslab-viscous/robots/`.
   Cameras are matched by `unique_id`, not index: the KD overhead `head` = `0x11300002c7f4a60`,
   the wrist camera `left` = `0x11400000c45636b` with `rotation: 180`. (The old tank record
   `viscous_waterproof_01` still lists a different `head`; the 11-17 tank dry run used
   `viscous_waterproof_01_dry_air`'s cameras.)
2. Look at both previews in the Lab GUI (Robot settings -> Cameras -> On): sun glare, focus, framing.
   **The overhead must show our arm on the RIGHT of the image** (that is why our arm is Dream's right
   arm, step 7). Keep people/clutter out where possible. Turn the previews off before a run (the
   runner also asks Lab to release them, and Lab refuses new previews while a run holds the cameras).

## 2. Boundaries (the keep-out) for this setup

Ask Gavin which boundaries matter (11-16, desk: "the only boundary that is still important here is
the back post"). Files: `configs/viscous/environments/<env>/keepout_viscous_waterproof_01.json` and
`scripts/viscous-dream1/real/workspace_box_viscous_waterproof_01*.json`; the runner picks them with
`--environment {tank,dry_air}`.

- **Floor:** measure it, do not assume it. The desk floor was wrongly left at the tank's -28 mm;
  Gavin: "the bottom of the base arm (the one part that also has the back post), is in contact with
  the top of the table" -> the mount's 10 mm plate -> -10 mm (`fdb8580a`). In the tank, -28 was
  measured with the jaws resting on the glass (live 213138Z, 11-08).
- **Back post** (x = 106 mm in base_link, the L-bracket's post): dry air keeps upper arm/forearm
  15 mm from it (Gavin: "the back post limit is too strict ... there is 2cm gap"); the tank keeps
  66/70 mm. If the arm's REST leans on the post, `rest_allowance_mm` lets the run start there
  ("its fine if when resting it touches the post") and never lets it get closer than at rest.
- **Workspace box:** the elbow x max had to follow the post rule (36 -> 91 mm, `4cca758f`).
- Each change: tests (`tests/test_tank_guard.py`), then a dry run (a new plan id).

## 3. Limits for this setup (Gavin's rules)

- **Tank (glass):** 15 deg/s hard cap, all trips on (torque, temperature, lag, limp, overspeed).
- **Dry air:** no runner torque/temperature limits ("not in a glass tank"; "the motors themselves
  have temp protection"): `--torque-trip-nm 14 --torque-cap-nm 14 --temp-trip-c 82`; 45 deg/s and
  no lag/limp/overspeed trips: `--speed-deg-s 45 --no-lag-trip --no-limp-trip --no-overspeed-trip`
  (dry air only; the stop path still uses them to HOLD). Stop returns stay <= 15 deg/s.
- Known cost in air: the RS00 shoulder's own thermal cutout went limp at 74 C holding the old tank
  start pose; prefer start poses with low shoulder load.

## 4. Rest pose

The rest is where the arm settles with torque off; every stop returns there and releases. Have
Gavin put it there ("let this be the new rest pose"), read it torque-off (Lab tank-measure, or the
record button below), and check it passes the start gate in a dry run. In the tank the rest lies on
the floor glass.

## 5. Start pose (where Dream takes over)

1. Gavin poses the arm by hand, torque off, and clicks:
   `.venv/bin/python scripts/viscous-dream1/real/record_pose_button.py --robot <record>`
   (saves `runs/start_pose/<utc>-...json`; refuses while the runner runs).
2. Turn it into the arm file's `rest` (IK radians; IK pan = motor pan - 180):
   `make_tank_arm.py` (see `record_start_pose.py`'s printed command), copy the arm file under
   `~/viscous-dream-real/station/arms/`, run with `--keep-arm-rest`.
3. Check from that pose: the wrist camera sees the work area and the jaws; the elbow is clear of
   the post; the shoulder load is low. The cube-spot analysis suggested moving the dry-air start
   ~5 cm further out/left for more post clearance during the pull-back (not done yet).

## 6. Rehearsal, no Dream calls

Rest -> start -> hold -> red button -> back to rest -> torque off. Dry, then live:
`dream_real.py --rehearse <same flags>` (prints the plan id), then
`dream_real.py --rehearse --live --plan-id <id> --pid-file /tmp/dream_real.pid <same flags>`.
Watch it at the arm; it found: too weak / slipping off the rest, overspeed on the wrist, shoulder
torque in air, the start gate refusing a rest on the post.

## 7. Dream settings that worked

- `--arm-slot right --wrist-r left --wrist-l black --overhead head` (our arm = Dream's RIGHT arm).
- Object: the 25 mm blue cube; prompt `"pick up the blue cube"`; jaws open at the start; nobody
  touches it during the run. Thin objects (the screwdriver) failed.
- `--dream-state measured --max-calls 18 --pause-s 10`, synchronous pacing (`--overlap` exists for
  dry air, reviewed, not yet run live).
- **Place the object with the geometry analysis**: `gavin-linux:~/cube-spot/` (Codex brief:
  grid the floor, simulate a pick + pull-back through the client IK and the guard, check the wrist
  view). Dry air answer: centre 32.5 cm forward of J1, 10 cm left. Redo it for the tank geometry.

## 8. Dry run, then live

1. Dry run (no `--live`) with exactly the live flags -> plan id. Any flag change = a new id.
2. Inputs check: the runner refuses the first request unless `~/.dreamlabs/inputs_checked` holds
   the inputs id. That check is Gavin's act (`dream.input_check`), never an agent's.
3. Billed calls: ledger `runs/ledger.json` (cap 200, 197 used on 11-17). Ask Gavin to raise it;
   he tops up on request.
4. Kill any old stop button, start a fresh one (it reopens where it was last left):
   `pkill -f "[s]top_button.py"; .venv/bin/python scripts/viscous-dream1/real/stop_button.py --pid-file /tmp/dream_real.pid &`
   Only one button window at a time.
5. Gavin says go -> `dream_real.py --live --plan-id <id> --pid-file /tmp/dream_real.pid <flags>`
   in the background; relay each phase from the run's `console.log`
   (`~/.makermods/makermodslab-viscous/run-data/live-*/console.log`).

## 9. After each run

- `touch <run-data>/KEEP` (the ring keeps only 10 bundles of every kind) and
  `rsync -a <run-data> gavin-linux:dream-run-archive/mac-run-data/`.
- Per call: Dream's grip and hand target (`chunks.jsonl` per_key `action.right_*`), guard skips
  (`console.log` "override"), measured vs commanded gripper + gripper torque in `ticks.jsonl` (the
  jaws stalling at ~0.2 while commanded lower, with 2-5 Nm, = holding the cube), and a contact
  sheet of `frames/left` + `frames/head`.

## Things that went wrong once (check them first next time)

| Symptom | Cause | Fix |
|---|---|---|
| Handshake: no motor responds | arm unplugged from power | plug in, `viscous-can-ping.py` |
| Runner refuses at the camera step / 405 | Lab on old code | restart Lab |
| `latest frame is too old` at camera open | Lab GUI previews held the cameras | camera lock (`bc8354f6`) |
| Dream closes the jaws in call 1, never reaches | our arm sent as Dream's LEFT arm | `--arm-slot right` |
| Hand sinks to the desk call by call | post guard cuts Dream's lift + arm sag | object/start further out; gravity comp (in progress) |
| Lift stalls, cube released | pull-back toward the base blocked by the post rule | start pose further out (next) |
| Floor slide lets the jaws into the desk | floor value from another setup | measure the floor (step 2) |
| Two red buttons open | stale button | `pkill -f "[s]top_button.py"` before each launch |
| `timeout: command not found` | macOS has no `timeout` | do not use it |
