# Task: a step-by-step robot control interface so an LLM agent (Codex Astra) can try the flange task

Work in `~/dreamtrial/viscous_sim` on gavin-linux (venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`,
`TMPDIR=$PWD/.cache/tmp`). Put all new code under `astra_op/` there. Read `runs/flange_dream2_report.md` and the
flange code (`flange_scene.py`, `flange_holding.py`, `flange_start.py`, `flange_pregrasp_demo.py`, the flange task
in `sweep.py`). **Another agent session is working on the propeller task in this same directory: do not touch
propeller files, the runner lock or the ledger, and make no Dream-1 calls.**

## Why (Gavin, 2026-10-24)
Zero-shot Dream-1 failed the flange task 0/12 (it brings the nut near the bolt but never aligns or turns it).
Gavin: "try with Astra then". Codex model `gpt-6-astra` can view images and run commands, so it will act as the
robot's brain: look through the cameras, decide, command the arms, look again. This brief builds and tests the
interface it will use. A later run hands it to Astra.

## The interface
A long-lived sim server (one MuJoCo flange scene, the trial-2 physics: flange-only +-25 Nm arm limits, 160 N jaws,
contact-gated grasp welds, the ideal thread model, unchanged fixture/cameras/masses) plus a CLI `robot` that talks
to it over a unix socket. Every command steps real physics (no teleporting objects) and returns text.
Commands (`<arm>` is `left` or `right`; world frame metres; describe the frame in the README):
- `robot look` saves `top.png`, `wrist_l.png`, `wrist_r.png` (same camera poses as Dream-1's, but 960x540) into
  the workspace `obs/` with a step number, and prints proprioception: per arm the hand (tool point) position, its
  approach and jaw axes as unit vectors, gripper opening (mm), the six joint angles, and `grasp: holding|empty`
  (from jaw contact force, as a real gripper's current would tell). No object poses, ever.
- `robot point <camera> <u> <v>` returns the 3D world point at that pixel (a calibrated depth camera; raycast on
  the rendered depth), so the agent can turn what it sees into coordinates.
- `robot move <arm> x y z [ax ay az]`: move the tool point to an absolute position, optionally with the approach
  axis pointing along (ax,ay,az), through the native client IK, executed as a smooth trajectory at a bounded speed
  with physics running; print the reached pose, the residual error, and whether IK hit a limit.
- `robot nudge <arm> dx dy dz` (millimetres, world frame), `robot turn <arm> <degrees>` (wrist roll, for screwing:
  the held nut must turn with it), `robot grip <arm> open|close|<0..1>`.
- `robot status`, and `robot done "<one-line claim>"` to end the episode.
Server side (hidden from the agent): every command, the measured insertion depth, through-both, nut engaged,
turns and advance, contacts, torques and attach/release events go to `episode.jsonl`, and a frame per physics
tick (subsampled) goes into an H.264 yuv420p `run.mp4` with three panes: top, the inspection close-up of the flange
joint, and the right wrist. Conditions as in trial 2: `G3` (bolt through, nut in the right gripper), `G1` (both
held), `T` (table start); pick with a server flag and a seed. A command cap per episode (default 200) and a
wall-clock cap.

## The agent's workspace
`~/dreamtrial/astra_ws/<episode>/` containing ONLY: the `robot` wrapper, `obs/`, and a `README.md` for the
operator: what the robot is (two Viscous arms, 6 joints + parallel gripper each), the task, the frame and units,
the camera intrinsics/extrinsics, every command with an example, practical limits (reach, jaw opening 105 mm,
speed), and that threads need the nut within ~3 mm of the bolt axis and turned like a real nut (right-hand thread,
3.5 mm pitch). It must not mention internal object coordinates. Nothing else from `viscous_sim` may be copied there.

## Validate (no Dream calls, no Astra yet)
1. Unit-check each command (IK residuals, grip, turn rotates a held nut, `point` matches the ground truth within
   a few mm on visible surfaces).
2. A scripted operator that uses ONLY the CLI (it may use privileged poses just to choose its targets, clearly
   marked as a validation cheat) completes G3: nut on the bolt, engaged, >= 2 turns. If G3 cannot be completed
   through this interface, find out why and fix the interface (not the physics). Show the run.mp4.
3. Old tasks unchanged: rerun the existing no-call checks that do not need the runner lock; if the lock is held by
   the other session, say so instead of breaking it.

## Deliverables
`astra_op/` code, `astra_op/README_OPERATOR.md` (the template copied into each workspace), `runs/astra_flange/
validation/` (unit checks, scripted G3 `run.mp4`, `episode.jsonl`), and a short `runs/astra_flange/harness_report.md`:
commands, what the agent can and cannot know, validation results, how to launch an episode.

## Hard rules
No Dream-1 calls, no ledger access, no hardware. Only write inside `~/dreamtrial/viscous_sim/astra_op`,
`~/dreamtrial/viscous_sim/runs/astra_flange` and `~/dreamtrial/astra_ws`. Do not edit installed packages, the Lab
repo, `viscous_arm.json`, or any propeller file. Never print the API key.
