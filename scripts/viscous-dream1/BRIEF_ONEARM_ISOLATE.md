# Task: find out WHY the parked second arm fixes one-arm Dream-1 in sim

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`,
`TMPDIR=$PWD/.cache/tmp`. Read first: `BRIEF_ONEARM.md` and your report `runs/viscous_onearm_report.md`, and the
`parked` / `single_copy` code you added to `sweep.py` / `sweep_scene.py`.

## Where we are
Apple task, zero-shot `dreamlabs/dream-1-beta`, vision mode, seeds 20261020-20261025: `B0` 6/6, `single` 2/6,
`single_copy` 2/6, `parked` 6/6 (median 9 calls). The `parked` contact sheet
(`runs/sweep/dry/parked/apple/ep_00/contact_sheet.jpg`) shows two things changed at once:
(a) the top camera sees a second arm, and (b) the right wrist camera, on the upright parked arm, looks down at the
table from the right and sees the bowl and other objects: a second real view of the workspace, which `single`
(black) and `single_copy` (a duplicate of the left view) never had. On the real bench, (b) alone could be a cheap
fixed camera, while (a) needs a second arm or a dummy. Separate them.

## Build first (no calls)
Two configuration knobs, off by default; every existing configuration must behave exactly as before (rerun the
existing no-call checks):
1. `single_fixedcam`: the right arm is removed from the scene as in `single` (not visible in any camera), but the
   `wrist_r` image is rendered from a FIXED camera placed at exactly the pose (position, orientation, FOV, aspect)
   of the parked arm's wrist camera at rest. The right state sent is exactly what `parked` sends (its measured rest
   joints, open grip); right chunk ignored. Note that the parked arm's own body (gripper fingers) appears in its
   wrist view; say whether the fixed camera's image lacks them and show both side by side.
2. `parked_blind`: exactly `parked`, except the `wrist_r` frames sent are black, as in `single`.

Dry-run both (no calls): one call-1 contact sheet each, plus a side-by-side of the `parked` and `single_fixedcam`
`wrist_r` images at call 1 with the pixel difference.

## Sweep (live)
Apple task only, 6 episodes each, the same seeds as before so results pair episode by episode:
`single_fixedcam` and `parked_blind`.

## Hard rules
- Billed: the existing ledger (cap 1312, 1029 used) has 283 calls left; this task may use at most 220 of them. Do
  not raise the cap. If you would exceed 220, run fewer episodes and say so. `pool_warming` (DA-2000) refusals are
  not billed and may be retried.
- No hardware (no serial, CAN, /dev/video*, systemd). Only write inside `~/dreamtrial/viscous_sim` (TMPDIR there).
  Do not edit installed packages, the Lab repo, `viscous_arm.json` or `viscous_arm_fingertip.json`. Never print the
  API key. Do not touch anything under `~/sonar-recon` or its processes.
- Disk: per episode only log.json, chunks.jsonl, run.mp4 (and a contact sheet for the first episode of each config).

## Report (short, to `runs/viscous_onearm_isolate_report.md`)
One table with `B0`, `single`, `single_copy`, `parked` and the two new rows: episodes, successes, grasps, median
calls to success, finger-table contact rows. Paired outcomes per seed. Which of (a) the visible second arm and
(b) the second real view explains the `parked` result, or both, or neither cleanly. What that means for the real
one-arm bench (for example: a fixed camera where a right wrist would be). Billed calls used, one good and one bad
video per new configuration, files added or changed. Say plainly what is measured and what is a guess.
