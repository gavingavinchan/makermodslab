# Brief: why does unit 1's (dry Viscous arm) joint mapping keep going wrong, and how do we fix it for good?

You are investigating, READ-ONLY. Do not edit any file, do not commit, do not open serial ports or
run anything that touches hardware, do not ssh anywhere. Produce a written report only. Gavin (the
owner) wants to discuss the proposal before anything changes.

## Gavin's request (his words)
"see how we can update all the teleop or urdf or whatever for this dry arm so this does not keep
happening. this is a recurring problem."

## Context
Repo: /Users/gavin/Documents/makermodslab-viscous (branch feat/viscous-arm). MakerMods Lab, a FastAPI +
React wrapper around a lerobot fork. Read CLAUDE.md (repo root) and docs/viscous-haptics.md first
(goal tree; sections A4, "Star actuation check", "Gripper haptics proof of concept", "Handoff").

Two Viscous arms (7-DOF, all RobStride RS00 motors on CAN), each driven by a purple Star Arm 102-HD
leader (FashionStar UART servos):
- unit 1 `viscous_01` (DRY, non-waterproof) on gavin-linux, leader calibration id `star_viscous`.
- `viscous_waterproof_01` on the Mac, leader id `star_viscous_waterproof_01`.

Snapshot of both machines' Lab state (robot records, commissioned profiles, lerobot calibration
files) is at configs/viscous/state-snapshots/2026-10-27/{gavin-linux,mac}/ (originally /tmp/viscous-state) (paths mirror $HOME). Note what exists for one
unit and not the other.

The lerobot fork is installed at
/Users/gavin/Documents/makermodslab-viscous/.venv/lib/python3.12/site-packages/lerobot
(robots/viscous_follower, robots/maker_follower, teleoperators/rebot_102_leader incl.
config_rebot_102_leader_viscous.py, motors/robstride). The sibling simulation/model repo is
/Users/gavin/Documents/viscous-arm-model (URDF, meshes, leader/mapping/profiles/ incl. a draft
viscous_01 profile). Lab-side: makermodslab/arms/viscous.py (`_commissioned_config`),
makermodslab/joint_posture_mapping.py, makermodslab/arms/urdf.py, frontend/src/lib/urdfConfigs.ts,
frontend/public/ (URDFs), configs/viscous/ (fits, evidence), scripts/restore-viscous-waterproof.py.
Use git log / git log -S on these paths to see how mappings changed over time.

## The symptom today (2026-10-27, unit 1)
The follower gripper never closes past about -33 deg, although its soft limits are (-120, -2.8)
(captured at 0.05 rad back-off from the mechanical stops). The Star gripper's closed stop reads
about -0.7..-4 deg raw. The Viscous Star preset maps gripper as follower = raw * -1.778 + (-33),
so a closed Star gives -33 and the closing ~30 deg of the follower's travel is unreachable, in
normal Lab teleop too. On 2026-10-21 the derived Star closed bound (-16.9 raw) already disagreed
with the measured -2.2. The follower config comment says zero = "gripper open"; the data suggests
-33 is the closed side and -120 open; check which is true from the sources. Also: after a power
cycle the follower gripper read +327.5 (a whole-turn reset, corrected by MakerFollower's
+-360 logic).

Other recurring mapping problems on these arms, from the doc: 1:1 posture mapping exists for the
waterproof unit but unit 1's profile is only a draft in the sim repo; the Star's measured travel is
narrower than the follower's on some joints; unit 1's wrist camera is flipped; the leader's
`arms/can_common.py` comment wrongly says the Star has no motors.

## What to deliver (markdown, concise, cite file:line)
1. Inventory: every place a unit-1 joint mapping, limit, zero, direction, scale, offset or range is
   defined (lerobot preset, Lab profiles/records, calibration JSONs, posture mapping, URDF, sim repo
   profiles, frontend), with unit 1's actual values per joint (table), which one wins at runtime in
   Lab teleop (trace the code path), and where sources disagree. Do the same comparison for the
   waterproof unit so the difference is visible.
2. Root cause: why this keeps recurring (structural reason, not just today's numbers).
3. Proposal for a fix that stays fixed: a single source of truth per physical unit (what file, what
   schema, where it lives so a fresh clone has it: the repo must be self-contained), how teleop,
   haptics scripts, URDF/visualization and the sim consume it, and a check that catches drift
   (a test, a startup check, or both). Prefer reusing existing mechanisms (e.g. `viscous_profiles`)
   over new ones. Note anything that would need a lerobot fork change vs Lab-only.
4. The gripper specifically: the mapping so a fully closed Star = fully closed follower jaws and
   fully open = fully open, and exactly what must be MEASURED on hardware to set it (and a short
   read-only measurement procedure: the arm is powered, torque off, a human moves joints by hand).
5. Risks: anything in the proposal that could make a powered arm move unexpectedly.

Keep it under ~1500 words plus tables. Say where you are unsure rather than guessing.
