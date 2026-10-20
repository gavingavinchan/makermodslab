# Task: can Dream-1 grab a bigger object? YAM, Dream's scene, a 5 x 5 x 3 cm black-and-white box

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`.
Read `BRIEF_2X2.md` and `runs/codex_2x2_report.md` (Run A) first: you built `--task cube` in `yam_baseline.py` /
`cross_scenes.py`, and YAM never went for the 2 cm white cube in 20 calls (`runs/yam_cube_vision_live1`). Gavin
thinks Dream-1 was never trained on objects that small, and put a bigger object on the real bench to compare.

## The real object
`real/topdown_box_20261020.jpg` (real top-down camera) and `real/wrist_box_20261020.jpg` (real wrist camera). The
object is the black-and-white Feetech servo box next to the small white cube: about 5 x 5 cm footprint, 3 cm tall,
lying flat. Its top face is white with black graphics and the FEETECH logo; its sides are mostly black with white
printing. The black square outline (~4.5 cm, thin black tape) is the place target, smaller than the box.

## What to do
1. Add `--task box` beside `--task cube`: in Dream's YAM scene, replace the apple with a box 0.05 x 0.05 x 0.03 m
   (lying flat, 3 cm tall) at the apple's spawn position, ~40 g, friction like Dream's fruit, graspable by YAM's
   gripper (opening 2 x 47.5 mm). Make it look like the real one as far as cheap: white top, black sides; a simple
   procedural texture for the top (a black band or logo-like dark marks on white) is enough. Keep the black square
   outline at the bowl's position as in `--task cube` (same size). Prompt:
   "pick up the black and white box and put it on the black square". Success: the box resting with its centre
   inside the square's outline (it cannot fit inside; on top of it counts). Log box-to-square distance and box
   height, as for the cube.
2. Dry run, and a top-camera preview PNG next to `real/topdown_box_20261020.jpg` so the look can be compared.
   Optionally a short scripted YAM pinch to confirm the box can be lifted (skip if not cheap).
3. Live run 1: `--mode vision --task box --live --max-calls 20 --out runs/yam_box_vision_live1`.
4. Only if run 1 never closes the gripper on the box: live run 2 with prompt "pick up the box"
   (`--task box --prompt ...` or a `box_pick` task), `--max-calls 15`, out `runs/yam_box_pick_vision_live1`.
5. Use the chunk logging from the IK-reach task (`chunks.jsonl`): report the minimum distance from the model's
   requested left- and right-hand positions to the box, as for the apple.

## Hard rules
- Billed: at most 20 calls in run 1, 15 in run 2, 35 in total. `pool_warming` (DA-2000) may be retried.
- No hardware: no serial, CAN, /dev/video*, systemd. Only write inside `~/dreamtrial/viscous_sim` (TMPDIR there too).
  Never print the API key. Do not edit installed packages. Keep existing modes working (dry-run them).

## Report (short)
Per run: billed calls, what the arms did (from frames), closest requested hand-to-box and box-to-square distances,
grasped yes/no, success yes/no. Compare with the 2 cm cube run. Files changed.
