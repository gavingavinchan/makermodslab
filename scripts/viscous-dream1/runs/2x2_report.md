**Both new cells failed:** cube/square also fails on YAM, and Viscous still fails on Dream’s working apple task. These trials implicate both task and embodiment transfer.

Commands used `MUJOCO_GL=egl ~/dreamtrial/.venv/bin/python`:

- **A:** `yam_baseline.py --mode vision --task cube --live --max-calls 20 --out runs/yam_cube_vision_live1`  
  **20 billed calls; success: no.** Left arm attempted grasps and moved toward the square empty-handed; cube stayed put. Right stayed near home. Closest distance: **20.8626 cm**. Minor gripper clipping; no joint-limit notices. [Frames](runs/yam_cube_vision_live1/contact_sheet.jpg).

- **B:** `yam_baseline.py --mode vision --rig viscous --live --max-calls 20 --out runs/viscous_apple_vision_live1`  
  **10 billed calls; success: no.** Left arm moved upward near its base; right moved slightly and closed. Neither contacted the apple. Closest distance: **20.8659 cm**. Call 10 returned gripper **−0.089**, rejected before execution; nine chunks ran. No retry. [Frames](runs/viscous_apple_vision_live1/contact_sheet.jpg).

Changed `yam_baseline.py` for scene options, logging, preflight, and the required custom-arm grip-key adapter. Added `cross_scenes.py` for scene composition and `cross_scene_check.py` for dry checks. Existing modes pass dry checks.

B’s [top-camera framing](runs/viscous_apple_vision_live1/compare_top_yam_vs_viscous.png) matches the baseline’s table/object layout; arm shapes and shadows differ. A scripted Viscous pinch successfully lifted the apple **7.8 cm**.

**30 billed calls total.** One hard-rule miss: the first dry run generated a URDF under `/tmp`; subsequent temporary files stayed inside the workspace. [Full report](runs/codex_2x2_report.md).