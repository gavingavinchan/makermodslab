# Task: re-mount the Maker wrist cameras to match the real Maker Arm's KD-USB wrist camera, then retry Dream-1

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`.
Read `BRIEF_MAKER.md` and your report `runs/codex_maker_report.md` first: you built `supported_arms.py` (Maker and
Metal stations in Dream's scene, FK-matched to dream_ik), and Dream-1 never aimed at an object on either arm.

## Why
Our improvised Maker wrist camera was wrong: it looks at the back of the gripper with the table upside-down at the
top (`runs/makers_apple_dry/camera_comparison.png`). Dream-1 leans on its wrist views, and Maker Arm data WAS used to
train Dream-1 (Ryan, 2026-10-19), so the real Maker wrist view is in its training distribution.

## The real camera (MakerMods' records)
- Every Maker Arm uses the **KD-USB Sonix UVC camera** (`2C7F:4A60`), 1920x1080 (16:9) at 30 fps, fisheye lens.
  Undistorted pinhole equivalent: **HFOV 109.6 deg, VFOV 77.2 deg** (fx 677.1, fy 676.2 at 1920x1080). Best focus
  ~0.20 m. Its pose on the arm (extrinsics) was never measured.
- Real Maker wrist frames, from two Maker Arm datasets: `real/makerwrist_andre_{0,1,2}.jpg` and
  `real/makerwrist_maker_{0,1,2}.jpg` (640 wide, 4:3 crops of the recorded stream, as the dataset stored them).
  What they show: the camera sits ABOVE the gripper, behind the jaws, looking forward along the fingers and slightly
  down. The two black finger blades rise from the bottom centre of the image, tips meeting near the middle-bottom;
  the table and the objects in front fill the rest of the frame; the far wall/horizon is in the upper part. The image
  is upright (table at the bottom, room at the top).

## What to do
1. Re-mount the Maker wrist camera (both arms) on the gripper link so the sim view matches those frames: position
   behind and above the jaws, optical axis along the approach direction tilted down as in the photos, image up =
   away from the table when the hand points forward. Use the KD optics: render 16:9 with VFOV 77.2 deg (HFOV 109.6),
   then feed the model exactly as Dream's `capture` does (it crops and resizes to 320x176). Use the Maker CAD in
   `assets/supported_arms/maker-urdf` (look for a camera bracket mesh; if there is one, mount the camera on it).
   Iterate until a side-by-side of sim vs real (same hand pose roughly: forward, slightly down, a small object
   ~10-20 cm in front) shows the fingers in the same place and size. Save `runs/maker_wrist_match/compare.png`.
2. Dry run, scripted pinch still works, existing modes still pass.
3. Live, Maker, box: "pick up the black and white box and put it on the black square", `--max-calls 20`,
   out `runs/maker_box_vision_live2`.
4. Live, Maker, apple: "put apple into the bowl", `--max-calls 20`, out `runs/maker_apple_vision_live2`.
5. Same analysis as before (closest requested hand to object, object to target, grasp, success; frames, contact
   sheet, video).

## Hard rules
- Billed: at most 20 calls per run, 40 in total. `pool_warming` may be retried. No other live calls.
- No hardware (no serial, CAN, /dev/video*, systemd). Only write inside `~/dreamtrial/viscous_sim` (TMPDIR too).
  Do not edit installed packages or the Lab repo. Never print the API key. Keep existing modes working.

## Report (short)
The final wrist camera pose (link, position, orientation, fov) and the comparison image; per run the numbers above
and what the arms did; files changed.
