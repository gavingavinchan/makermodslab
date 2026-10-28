# Task: fix the stainless shading on the shackle and pin in MuJoCo (looks only, NO billed calls)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`,
`TMPDIR=$PWD/.cache/tmp`. Read `runs/shackle_scene/shackle_scene_report.md` and the scene code you wrote
(`shackle_scene.py`, `shackle_render.py`).

## Why (Gavin, 2026-10-28)
The shackle task's sim videos are what Gavin will show, and "it has to look nice". In
`runs/shackle_scene/overview_detail.png` the shackle body and the pin have blotchy, jagged-edged white patches on
the bow and both ears (and some on the pin), which real stainless does not have. Find the cause (candidates:
`reflectance` on curved meshes, shadow acne / shadow-map resolution and bias, the split-normal OBJ with
`smoothnormal="false"` on the smooth bow, specular/shininess, z-fighting with the collision cells if any are
visible) and fix it so the stainless reads as clean brushed/polished steel with smooth highlights, the threads
still crisp.

## Deliverables (under `runs/shackle_look/`)
- before/after crops of the same views (detail close-up, front three-quarter, top) at 1280x720, and a short
  `shackle_look_report.md`: the cause, what you changed, and why it does not change physics.
- the policy contact sheet (`contact_sheet.jpg` style) before/after, to show what Dream-1/Astra would see.

## Hard rules
- **A live Dream-1 run is in progress** (`sweep.py --task shackle --live`, output `runs/shackle_scene/dream/`):
  do not stop it, do not touch `runs/sweep/ledger.json`, `runner.lock` or `runs/shackle_scene/dream/`, and make
  no live calls. Your edits must not affect that running process (it already imported its modules); keep any
  change to a visual material/render setting in the shackle task only, with physics, layout, cameras and the
  other tasks byte-identical. Say exactly which files changed.
- **An Astra A/B is running** (`astra_learn/ab.sh`, an `astra_op/server.py` process): do not start another
  `astra_op/server.py`, do not touch `astra_op/` files other than the shackle profile, or `astra_learn/`.
- No hardware. Only write inside `~/dreamtrial/viscous_sim`.
