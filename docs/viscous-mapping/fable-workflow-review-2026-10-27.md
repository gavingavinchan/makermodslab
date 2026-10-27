# Third opinion: the "fixes stay fixed" workflow — 2026-10-27

Read-only review (Claude Fable 5.1). Only this file was written. No hardware, no
service changes, no commits. Evidence below is command + the output that matters.

## Verdict in three lines

The plan is aimed at the wrong problem first. Claim 1 (Linux runs stale code) is
wrong, not merely unproven; the start failures in both machines' logs are hardware
readiness, port drift and one limits bug that is already fixed. Both of Gavin's
complaints trace to one cause: unit 1 has no committed, measured profile, so it
runs on the fork's 08-28 preset while its gripper zero moved ~33 deg. Fix that
first; deploy tooling and CI are useful but second.

## Claims, checked

**1. "The Linux service does not have d28d5fc4" — WRONG.**
```
ssh gavin-linux 'git reflog --date=iso | grep d28d5fc4'
  d28d5fc4 HEAD@{2026-10-22 17:58:04 -0500}: commit: Let teleop start ...   (committed ON Linux)
ssh gavin-linux 'ls --time-style=full-iso makermodslab/can_speed_limit.py; git diff --quiet HEAD -- makermodslab/can_speed_limit.py && echo clean'
  2026-10-22 17:34:31  (last edit)   clean
systemctl --user show makermodslab-viscous -p ExecMainStartTimestamp -> Thu 2026-10-22 17:34:42 CDT
__pycache__/can_speed_limit.cpython-312.pyc: src_mtime 2026-10-22 17:34:31 size 8742 == source
```
The file was edited in place on Linux, the service restarted 11 s later, and the
same bytes were committed at 17:58. The running process (lazy import at
`teleoperate.py:757` reads the file as it is on disk) has the fix. The journal
shows the fix already working at 14:47 that day under an intermediate wording
("just outside its limits", never committed, `git log -S` finds nothing), and
teleop started. Also: **no teleop session has been started on Linux since the
17:34 restart** (21,049 journal lines, zero `Starting teleoperation`); the last
Lab teleop there was 10-22 15:15. The "stale deploy" theory has had nothing to
act on. Codex's "treat as unknown" is too cautious; Claude's claim is false.

**2. Arm state lives outside git and differs — CONFIRMED, with a twist.**
Live hazard confirmed: on Linux both records name the same CANable `207E37A55445`
and the same Star; `ls /dev/serial/by-id/` shows exactly one CANable, unit 1's.
Linux records have `port_identities = None`; the Mac's carry VID/PID/serial.
The twist neither review drew out: the Mac's installed waterproof profile
(`viscous_profiles/viscous_waterproof_01.json`, 2026-10-13) is NEWER than the
committed bundle (last commit 4745784d, 2026-10-09): gripper max `-0.034` vs
`-2.92`, offset `0` vs `-3`. "Machines use the committed file" would silently
regress the Mac unless the Mac copy is committed first. The self-contained-repo
rule is already broken in the direction the plan does not expect.

**3. No workflow on feature-branch pushes — CONFIRMED.** `tests.yml`: push/PR on
`main`/`staging` only, with path filters (`makermodslab/**`, `tests/**`,
`pyproject.toml`) that exclude `configs/**` and `scripts/**`.

**4. "Nothing checks measured positions or adapter identity" — PARTLY.** Exists:
`MakerFollower.connect()` runs `_detect_full_turn_offsets()` (live pose vs limits,
20 deg grace, whole-turn correction) BEFORE `enable_torque()`; the speed limiter
requires fresh finite feedback per joint before any preload/enable and, since
d28d5fc4, warns instead of refusing on an out-of-limit start; `usb_ports.resolve_ports`
rebinds by VID/PID/serial and blanks the port when ambiguous, but only when the
record has `port_identities` (Mac yes, Linux no); `ViscousFamily.build_single_configs`
checks configured leader ranges against configured follower limits. Missing: any
link between the physical unit and the profile (an adapter serial is a proxy, and
on Linux not even that), and any check that the limits still describe the arm.
The stale-zero refusal fires in `send_action()`, i.e. after torque is on.

## Why "teleop fails to start" recurs — what the logs say

Mac `server.log` (09-16..10-18) and Linux journal (09-15..10-22), failures by cause:
```
Mac:   10  Star leader not answering / wrong port (wchusbserial10 -> 110 -> 1110 across days;
            "Servo not found for shoulder_pan"); the CH340 has no USB serial, so rebinding is by model only
        6  CAN "Handshake failed. did not respond: [all 7]" (arm unpowered / not in MIT mode); retry succeeded
            within 5-60 s every time (10-13, 10-15, 10-16, 10-18)
        3  "wrist_yaw starts outside its commissioned limits; refusing a clamped jump" (10-18)
        1  "Resource busy" (port held by a bench script), 1 missing /tmp/viscous-can-bridge.json (experiment)
Linux:  3  "elbow_flex starts outside its commissioned limits" (10-22 14:23-14:41) -> fixed same day (d28d5fc4)
        7  connect failures in September (pre-commissioning), 1 adapter unplugged, 1 Codex test path
```
So it is not one bug. Three classes: (a) a joint parked a few degrees past a soft
limit — fixed 10-22 on both machines' code; (b) hardware not ready at the moment
of the click — not a software bug, retry works; (c) macOS port-path drift on the
serial-less Star adapter. Every one of them surfaces as the same sentence ("Could
not connect ... Check that the CAN adapter is plugged in ... Details: ...",
`teleoperate.py:783`), which is why they feel like one recurring bug. The next
member of class (a) is already waiting: with the gripper's real open stop at -156
and its soft limit at -120, a gripper powered up fully open sits 36 deg out, past
the 20 deg grace, lands in `_stale_zero`, and `send_action` refuses the session.
The gripper mapping bug and the "fails to start" bug are the same bug.

## Why the gripper does not open fully

`configs/viscous/viscous_01/evidence/gripper-range-20261027T002654.json` (committed):
follower stops -32.6..-156 vs preset limits -2.8..-120; the window is the right
width, 35 deg off. Unit 1 has no profile (`ls viscous_profiles/` on both
machines: waterproof only), so `ViscousFamily._commissioned_config` takes the
`viscous_01` preset-fallback branch and the limits live in the lerobot fork. No
test can catch this; only a measurement, a committed file, and a drift check.
`scripts/gripper-haptics-poc.py:119` builds its own `ViscousFollowerConfig` and
reads `config.joint_limits` from the preset, so a Lab-side profile will not reach
it unless it is routed through the family loader.

## Judgement of the combined plan

- Deploy script + commit in `/health` (step 1): keep, but small. `/health` reports
  `version: 0.1.0` only; capture the SHA at import time (Codex is right that
  reading HEAD per request would lie). A 20-line script per machine is enough:
  refuse if a session is live, `git pull --ff-only`, `uv sync --locked --extra test
  --extra arm-dynamics`, `pytest -q -x` of the fast tests, restart, curl `/health`.
  Codex's staged venv and joint rollback are overbuilt for one operator.
- CI on the branch (step 2): one-line change, near-zero value for these bugs.
  None of the observed failures was a logic regression a test would catch. Do it,
  but last, and widen the path filters when you do.
- One committed profile per unit (step 3): THE step. The machinery exists:
  `_commissioned_config` already loads `viscous_profiles/<id>.json` when present,
  `scripts/restore-viscous-waterproof.py` + `manifest.json` already checksum and
  install a bundle. Unit 1 needs a bundle with a `viscous_profiles/viscous_01.json`,
  nothing new. Then delete the preset-fallback branch so a missing profile refuses.
  Refuse at session start for that record (Codex is right), not at boot.
- Pre-energize preflight (step 4): mostly exists. Add two things: an
  `expected_adapter_serial` in the unit's profile checked against the resolved port
  (kills the Linux hazard for good), and a rest-pose check against the profile's
  measured stops (a slipped coupling shows up as every joint reading 30 deg off
  while folded). Keep d28d5fc4's easing; do not reintroduce a blanket refusal.
- Bug log (step 5): yes, in `docs/viscous-haptics.md`, one line per bug with the
  `Details:` string, the SHA, and the profile revision. The Details string is the
  classifier Gavin does not currently see.

## Where I disagree with Codex

1. Claim 1 is wrong, not unproven; the evidence settles it and no Linux teleop has
   run since the restart anyway. A deploy script is hygiene, not the fix.
2. Codex's ordering puts "runtime evidence and guarded deployment" second; I put
   the unit 1 profile second, because it is the only step that removes either
   symptom. Deployment hygiene removes a symptom nobody has observed.
3. Codex's deployment design (staged venv, rollback of deps + settings together,
   session-race prevention) is sized for a fleet. Two bench machines and one
   operator need a script that refuses to run while a session is live.
4. Codex leans on the "RobStride clear-fault read is not passive" observation.
   It is one unreplicated feel report; `connect()` already reads every joint
   before enabling, so a profile check at the same point adds no new reads.
5. Codex did not notice the committed waterproof bundle is older than the Mac's
   live copy. Direction of sync (installed -> repo first) matters before enforcement.

## First step, then two

1. **Today, no code (15 min):** on gavin-linux, blank the ports in
   `robots/viscous_waterproof_01.json` (or delete it; Gavin's call) and re-save
   `viscous_01` in the UI with the adapters plugged so `capture_identities`
   writes `port_identities` (`utils/config.py:1149`). Commit the Mac's live
   waterproof profile into the bundle so repo == reality.
2. **Commission unit 1 (needs Gavin's re-zero vs recapture answer):** measure
   all seven joints' stops and matched leader poses torque-off, write
   `configs/viscous/viscous_01/viscous_profiles/viscous_01.json` + manifest +
   evidence, restore it on both machines with the existing script, add the
   expected adapter serial, delete the preset fallback, route
   `gripper-haptics-poc.py` through the family loader. Offline tests: profile
   loads, wrong-serial refuses, missing-profile refuses.
3. **Deploy + evidence:** import-time SHA in `/health`, the deploy script, the
   bug log, and only then CI on `feat/viscous-arm` with widened paths.
