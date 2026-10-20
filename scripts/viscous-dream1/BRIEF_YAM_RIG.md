# Task: a YAM-station replica with Viscous arms, for one Dream-1 zero-shot test

Work in `~/dreamtrial/viscous_sim` on gavin-linux. Python: `~/dreamtrial/.venv/bin/python`. Read `BRIEF.md` (your
previous task here), `astra_report.md` and `viscous_dream_sim.py` first: you built the current bench scene.

## Why
A live Dream-1 run on the current bench scene failed (`runs/live_beta_1/`: the arm swung away from the cube and ended
pointing straight up). Leading hypothesis: Dream-1 was trained on YAM stations, whose overhead camera sits BEHIND two
forward-facing arms, about 0.94 m above the table, so "arm forward" is "image up". Our overhead camera is beside the
arm, 0.28 m up, rotated 90 degrees. The user wants one test with the station set up exactly like a YAM station, but
with Viscous arms in place of the YAMs.

## Ground truth for the YAM station (Dream's own files, read-only)
`~/dreamtrial/.venv/lib/python3.12/site-packages/dream_examples/yam_mujoco/`:
- `assets/sim-eval-yam.json`: arm base poses (`arms.left/right.position`, both facing +x, 0.61 m apart), the table,
  and the cameras (`cameras.top` position + `quaternion_opengl` + intrinsics at 640x480; wrist cameras on `link_6`).
- `yam_env.py`: how Dream's own sim turns that calibration into MuJoCo cameras (`calibrated_camera`, including
  `TOP_CAMERA_LOWER_M` and `TOP_CAMERA_TILT_UP_DEG`, the quaternion convention), the table/lighting/oak material.
- `dream_control.py`: the frame size and windows their sim sends.
Copy their conventions exactly (Dream-1 is known to work in their sim). You may reuse their assets (Apache-2.0) such as
`assets/oak.png` by path; do not copy files into Dream's package directory.

## What to build
Add a `--rig yam` option to `viscous_dream_sim.py` (keep the current scene as `--rig bench`, still working). In the
yam rig:
1. The driven Viscous arm (the LEFT arm) has its base at YAM's left-arm base pose; a SECOND Viscous arm sits at YAM's
   right-arm base pose, parked at the start pose and held there (kinematically is fine), visible in the cameras.
2. The overhead camera reproduces YAM's `top` camera relative to the arm bases exactly as `yam_env.py` builds it
   (position, orientation, the lower/tilt adjustments, field of view from the intrinsics), rendered 640x480.
3. `wrist_l`: keep the Viscous wrist camera you matched to the real mount. `wrist_r`: render the parked right arm's
   wrist camera (same mount) instead of the black frame.
4. Table, lighting and colours like Dream's YAM sim (oak table etc.), not our grey bench.
5. White 2 cm cube and black 4.5 cm square outline in front of the left arm, reachable by Viscous (its reach is much
   shorter than YAM's: keep both within roughly 0.15-0.30 m of the left base, toward the space between the arms as a
   YAM task would place them). Task prompt unchanged: "pick up the white cube and place into black square".
6. Start pose: both arms at `--start neutral` (all joints 0, gripper closed), as the current script does.
7. The observation state for the right arm must be the parked arm's actual joints and grip, in that arm's own base
   frame (the IK arm file already treats each side in its own base frame; each side's base frame is its arm's base).

## Validate
- `--rig yam --preview --out runs/yamrig_preview`: overhead, wrist_l, wrist_r and third-person PNGs.
- Render Dream's own YAM sim overhead camera (instantiate their `YamEnv` from `yam_env.py`, no network, no Dream
  calls) and save a side-by-side `runs/yamrig_preview/compare_overhead_yam_vs_viscous.png`. The two arms should sit in
  the same image places, facing the same way.
- Dry run still passes: `--rig yam --model beta --out runs/yamrig_dry` (no `--live`): "3 calls (none billed)",
  0 limit_rows, 0 clipped_rows. Also confirm `--rig bench` preview and dry run still pass.

## Hard rules
- NEVER pass `--live` and never call Dream's network API. The user approves live runs.
- No hardware: no serial ports, CAN or /dev/video*.
- Write only inside `~/dreamtrial/viscous_sim`. Do not modify Dream's installed package or
  `~/Documents/viscous-arm-model`. Do not change `viscous_arm.json` or the IK/control logic unless required; explain.

## Report (final message)
What changed, the exact camera and base poses used and where each number came from, the comparison image path, what
still differs from the YAM station, and the dry-run lines for both rigs.
