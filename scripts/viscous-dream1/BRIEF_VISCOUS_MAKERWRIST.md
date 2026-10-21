# Task: the Viscous arm with a MAKER-STYLE wrist camera: does Dream-1 now drive it?

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`.
Read `BRIEF_MAKER_WRIST.md` and your report `runs/codex_maker_wrist_report.md` first, and `BRIEF_2X2.md` /
`BRIEF_IK_REACH.md` for the Viscous rig (`yam_baseline.py --rig viscous`, `cross_scenes.py`).

## Why
With its wrist camera re-mounted to match the real Maker KD-USB view (`runs/maker_wrist_match/compare.png`: camera
above the gripper behind the jaws, looking forward along the fingers ~20 deg down, fingers rising from the bottom
centre, VFOV 77.2 deg, 16:9), the Maker arm now picks the apple into the bowl and grasps the box. Before that fix it
never aimed at anything. The Viscous runs so far used a wrist camera copied from the REAL Viscous mount, which looks
the other way round: the jaws come down from the TOP of the image (`real/wrist_box_20261020.jpg`,
`real/wrist_cam_index2.jpg`). Dream-1 was trained on Maker data, so give the Viscous the Maker's view.

## What to do
1. In the Viscous rig, mount each arm's wrist camera the way the Maker's now is relative to its own gripper: same
   optics (VFOV 77.2, 16:9, Dream's capture path), behind and above the jaws, looking along the fingers ~20 deg down,
   fingers rising from the bottom centre at about the same apparent size as in the Maker/real comparison. Make a
   side-by-side `runs/viscous_wrist_match/compare.png` of the Viscous sim wrist view next to the Maker sim wrist view
   and a real Maker frame (`real/makerwrist_*.jpg`) at a similar hand pose. Keep the old Viscous wrist mount
   selectable (a flag) so earlier runs stay reproducible.
2. Dry run; scripted pinch still lifts the apple and the box; existing modes pass.
3. Live, Viscous, apple: "put apple into the bowl", `--max-calls 20`, out `runs/viscous_apple_makerwrist_live1`.
4. Live, Viscous, box: "pick up the black and white box and put it on the black square", `--max-calls 20`,
   out `runs/viscous_box_makerwrist_live1`.
5. Same analysis as before: closest requested hand to object (L/R), object to target, grasp, success, what the arms
   did from the frames, IK tracking error. Note any table collisions of the Viscous fingers (the arm file's tool
   point is the finger-pad midpoint, not the fingertips; do NOT change `viscous_arm.json`, just report).

## Hard rules
- Billed: at most 20 calls per run, 40 in total. `pool_warming` may be retried. No other live calls.
- No hardware (no serial, CAN, /dev/video*, systemd). Only write inside `~/dreamtrial/viscous_sim` (TMPDIR too).
  Do not edit installed packages, the Lab repo, or `viscous_arm.json`. Never print the API key. Keep modes working.

## Report (short)
The camera pose and comparison image; per run the numbers above and what the arms did; files changed.
