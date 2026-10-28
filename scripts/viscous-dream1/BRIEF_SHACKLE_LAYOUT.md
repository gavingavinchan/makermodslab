# Task: re-lay out the shackle task in front of the arm (NO billed calls)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`,
`TMPDIR=$PWD/.cache/tmp`. You built this scene: read `runs/shackle_scene/shackle_scene_report.md`,
`runs/shackle_look/shackle_look_report.md` (the shading fix that just landed; keep it) and the shackle_* modules.

## Why (Gavin, 2026-10-28)
"I want the bolt to be in line with the arm. The bolt should be right in front of the arm, so when it starts, the
arm is already facing it. The holes should be closer to the arm, and the C-shaped part should be facing away from
the arm." (Dream-1 went 0/3 on the current layout and never grasped the pin; that run is finished, don't touch
`runs/shackle_scene/dream/`.)

## The change
- **Pin**: standing upright on the active (left) arm's straight-ahead line: the direction the arm's tool faces at
  its start pose with the base joint at its start angle, at a comfortable pick distance, so grasping it needs
  little or no base rotation. Say the exact distance and why.
- **Shackle (with its glued holder, same relative transform)**: rotated so the ears/pin axis are the part
  nearest the arm base and the C-shaped bow points radially away from the arm. Gavin did not say where it sits;
  put it near that same line (beside the pin, or just beyond it, offset enough that picking the pin cannot knock
  it and the carried pin cannot hit it), choosing the place that keeps joint torques lowest (closer to the base
  is better: the last scripted run peaked at 11.75 Nm against the real arm's 5 Nm). Report the peak torques
  before and after. Both must stay clearly visible in the top camera and the wrist camera at the start.
- Nothing else changes: same arms, cameras, lights, materials (with the look fix), physics, thread/seat model,
  instruction text, Dream-1 and Astra code paths.

## Deliverables
Re-run and overwrite what depends on the layout, keeping the old outputs under `runs/shackle_scene/layout_v1/`:
reach check (`reach.json`), the scripted demo (`scripted_demo.mp4`, must still seat the pin), the overview
renders (front three-quarter, top, close-up) and the policy contact sheet, the dry run, and the shackle and legacy
checks. Add a top-view `layout_compare.png` (old vs new, the arm base and its straight-ahead line drawn on).
Update `shackle_scene_report.md` with a dated "Layout v2" section.

## Hard rules
- No billed calls, no `--live`, don't touch `runs/sweep/ledger.json`, `runner.lock` or `runs/shackle_scene/dream/`.
- An Astra A/B may still be running (`astra_learn/ab.sh`, an `astra_op/server.py`): do not start another
  `astra_op/server.py`, and do not touch `astra_learn/` or other profiles. Other tasks must stay byte-identical.
- No hardware. Only write inside `~/dreamtrial/viscous_sim`.
