# Task: make Dream-1 work with ONE Viscous arm in sim (no fine-tuning)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`,
`TMPDIR=$PWD/.cache/tmp`. Read first: `BRIEF_SWEEP.md`, `SWEEP_README.md`, `runs/viscous_sweep_report.md`, and
`sweep.py` / `sweep_scene.py`, which you wrote.

## Where we are
Viscous + Maker-style wrist camera, zero-shot `dreamlabs/dream-1-beta`, vision mode, apple task ("put apple into the
bowl", cap 18 calls per episode, seeds 20261020 + episode index):
- `B0` (two arms): 6/6.
- `single` (right arm removed from the scene, right state = arm-file rest pose with open grip, `wrist_r` frames
  black, right chunk ignored): 2/6.

The real bench has one arm, so one arm is what has to work. Dream-1 was trained on two-arm stations; the guess is that
a black right wrist image and an empty right side push the input away from what it knows. Test two ways to make the
one-arm input look more like a two-arm one.

## Build first (no calls)
Add two configuration knobs to `sweep.py` / `sweep_scene.py`, off by default, so B0 and every existing configuration
behave exactly as before (prove it with the existing no-call checks):
1. `wrist_r=copy` (used with `single`): the right arm is still removed from the scene, but the `wrist_r` image sent to
   Dream is the LEFT wrist frame of the same step, unchanged (not mirrored). Right state and chunk handling as in
   `single`. On the real bench this costs nothing, so it is the preferred fix if it works.
2. `arms=parked`: the right arm stays in the scene at the same base as B0, held motionless at the arm-file rest pose
   with an open grip for the whole episode (position-held, not limp; it must not move when bumped by contact checks or
   physics settling). Its real `wrist_r` camera renders normally, the right state sent is its actual measured joints,
   and its chunk is ignored. On the real bench this needs a second physical arm or a dummy in view, so say how much it
   helps compared with `wrist_r=copy`.

Dry-run both (no calls): save one contact sheet per configuration showing all three camera images Dream receives at
call 1, and confirm the right arm in `parked` does not move over a full dry episode (log its max joint deviation).

## Sweep (live)
Apple task only, 6 episodes per configuration, the same seeds as the existing B0 and `single` apple episodes so the
results pair episode by episode.
- Stage 1: `single_copy` and `parked`.
- Stage 2, only if neither reaches 5/6: the better of the two from stage 1 (ties: fewer calls to success) AND plain
  `single`, each with the apple's spawn centre and the bowl moved toward the left (working) arm's side of the table.
  Work out from the geometry how far to move them so both stay well inside the left arm's comfortable reach and still
  in view of the top camera; keep the +-2 cm jitter; explain the shift with a picture of the scene before and after.

## Hard rules
- Billed: this task may bill at most 440 live calls. Raise the existing ledger's cap by exactly that much over what it
  has already used (872 billed on 2026-10-20; check the file) and keep the same crash-safe accounting. If a stage would
  exceed the cap, run fewer episodes and say so. `pool_warming` (DA-2000) refusals are not billed and may be retried.
- No hardware (no serial, CAN, /dev/video*, systemd). Only write inside `~/dreamtrial/viscous_sim` (TMPDIR there).
  Do not edit installed packages, the Lab repo, `viscous_arm.json` or `viscous_arm_fingertip.json`. Never print the
  API key. Do not touch anything under `~/sonar-recon` or its processes (another job is running there).
- Disk: per episode only log.json, chunks.jsonl, run.mp4 (and a contact sheet for the first episode of each config).

## Report (short, to `runs/viscous_onearm_report.md`)
One table: configuration, episodes, successes, grasps, median calls to success, finger-table contact rows, with the
existing B0 and `single` apple rows beside them. Episode-by-episode pairing against `single` (which seeds flipped).
Which fix helped and by how much, what it would take on the real bench, billed calls used, and the paths of one good
and one bad video per configuration. Files added or changed. Say plainly what is measured and what is a guess.
