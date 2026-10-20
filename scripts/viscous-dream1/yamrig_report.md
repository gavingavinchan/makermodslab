# YAM station with Viscous arms

`viscous_dream_sim.py --rig yam` uses the two-arm station; `--rig bench` remains the default. Scene composition is in `yam_rig.py`. Both start at neutral (six zero joints, closed grippers) by default. The right arm retains actual joint coordinates, restored with zero velocity after every physics step, and its own tool-mounted camera. Observations and commanded right-arm history read that parked state. No world transform is added to either arm's IK state.

No live calls or hardware access were made. The installed Dream package, reference Viscous model and `viscous_arm.json` were not edited. Dry runs retain the existing network-denying audit hook and stub metadata/transport.

## Exact geometry and camera sources

All positions are metres; quaternions are **w,x,y,z**.

`dream_examples/yam_mujoco/assets/sim-eval-yam.json` defines calibration-space bases:

- Left: `(0.2525, 0.305, 0.76)`, quaternion `(1,0,0,0)`.
- Right: `(0.2525,-0.305, 0.76)`, quaternion `(1,0,0,0)`.

The same `WORLD_ROTATION` and `WORLD_OFFSET` as `yam_env.py` are applied to the station: `(x,y,z) -> (y,0.4925-x,z)`. The compiled Viscous base poses are therefore:

- Left: `(0.305,0.24,0.76)`.
- Right: `(-0.305,0.24,0.76)`.
- Both quaternions: `(0.7071067811865476,0,0,-0.7071067811865475)`.

The arms are 0.61 m apart. Local +x forward becomes world -y and points toward image-up.

The calibration's top camera is at `(0.08600512146949768,-0.008999999612569809,1.7043205499649048)`, OpenGL quaternion `(0.68301,0.18301,-0.18301,-0.68301)`. Rather than reimplement the optics, the helper reads the camera element produced by `yam_env.build_xml()` / `calibrated_camera()`: normalize that quaternion, apply the above world transform, subtract `TOP_CAMERA_LOWER_M=0.15` in world z, and postmultiply by a `TOP_CAMERA_TILT_UP_DEG=5` rotation about camera-local +x.

Compiled overhead pose:

- Position: `(-0.008999999612569809,0.4064948785305023,1.5543205499649049)`.
- Quaternion: `(0,0,0.30070322265070765,0.953717763223208)`.
- Render: 640×480. Sensor: 0.0064×0.0048 m.
- Intrinsics from calibration: fx=392.195617675781, fy=391.722351074219, cx=318.389434814453, cy=237.876312255859 pixels.
- MuJoCo `principalpixel=(320-cx,cy-240)`, preserving the off-center principal point and unequal fx/fy. Derived vertical FOV: **62.989841846744326°**.
- Height above the actual table surface: 0.7943205499649049 m (after the sim's 15 cm lowering).

Both wrists keep the real-bench-matched Viscous `tool0` mount: position `(0,0.055,-0.09)`, target `(0,0.025,0.08)`, up `(0,-1,0)`, vertical FOV 90°. Compiled local quaternion: `(-0.08722511445123816,0.9961886264202019,0,0)`. The right camera uses `right_tool0` with the identical mount.

The human-only third-person view is at `(0.95,1.15,1.65)`, aimed at `(0,0.02,0.92)`, vertical FOV 50°, rendered 1280×720. These are presentation choices, not calibrated policy parameters.

The oak texture/material, skybox, lights, table, and overhead camera stand are taken from Dream's own scene builder. Its table center is `(0,-0.12,0.73)` with half-sizes `(0.65,0.55,0.03)`, hence surface z=0.76. This intentionally follows the working simulator's table rather than the different table metadata in the calibration JSON.

Task placement is chosen for Viscous: cube base-local horizontal offset `(0.20,-0.07)` (0.212 m radius), target `(0.23,-0.15)` (0.275 m radius). World cube center is initially `(0.235,0.04,0.7701)` and settles to z≈0.7699973; square center is `(0.155,0.01,0.76)`. Cube side 0.02 m; square outer width 0.045 m and line width 0.004 m. The success check now measures cube height relative to the square's table height.

## Image pipeline and comparison

Preview PNGs retain the full 640×480 render. For policy observations only, YAM follows `yam_env.images()` and `dream_control.capture()`: crop rows `[60:420]`, bilinear resize to 320×176, JPEG quality 85. Client windows are explicitly nine frames at 5 Hz; nine real ticks are collected before the first request. The existing Viscous client-side IK configuration remains in use rather than telling the server it controls YAM hardware.

`runs/yamrig_preview/compare_overhead_yam_vs_viscous.png` contains **unmodified Dream YamEnv | Viscous YAM rig**. The reference is instantiated locally and rendered directly through camera `top`, at its reset pose. The table framing and forward direction match. Both arm footprints occupy the corresponding image locations; meshes and gripper silhouettes differ.

`validate_yam_rig.py` checks exact compiled overhead position, quaternion, intrinsics, sensor size, resolution and FOV against an instantiated YamEnv. It also checks calibrated base translations, the parked right state after stepping, nonblack 320×176 policy images, and bounded static tool-pose reachability. Results are in `runs/yamrig_preview/validation.json`. Reachability errors are <0.22 mm with downward-tool alignment error <0.005; this is geometric reachability, not a collision-free trajectory or grasp-success guarantee.

## Remaining differences and dry-run scope

- Viscous geometry, reach, dynamics and wrist optics replace YAM's. Both Viscous grippers start closed; the reference YamEnv resets its grippers open.
- Calibration specifies base z=0.760 m, used here as requested; `yam_env.py` hardcodes its YAM base bodies at z=0.772 m. This is a deliberate, documented 12 mm vertical difference.
- The requested cube/square replaces Dream's fruit, bowl, plate and bottle.
- The right arm is parked, and returned right-arm actions are ignored.
- Production IK and servo equations are unchanged. The additional forward-kinematics update after stepping keeps rendered/state transforms current, including after parking the right arm.
- The YAM dry stub uses hand poses obtained by FK along a small joint bend (shoulder -0.04 rad, elbow +0.08 rad per synthetic chunk), which the real client then solves via IK. The bench stub retains its original 2 cm forward / 4 cm down Cartesian motion. Early Cartesian-stub trials produced small soft-limit excursions near neutral; their logs are preserved under `runs/yamrig_dry/initial_*` and `runs/yamrig_diagnostic/`. Limit counting and clipping thresholds were not weakened.

Validation command outputs are in `runs/yamrig_dry/output.txt` and `runs/benchrig_dry/output.txt`; full request summaries are in each directory's `log.json`.

Both previews passed. Each of three calls on each rig reported `"clipped_rows": 0, "limit_rows": 0`. All YAM request summaries contain nine images per camera and the parked right arm’s zero joints, closed grip, and base-local EEF `(0.2223,0,0.13)` (rounded by the logger).

```text
3 calls (none billed); log in runs/yamrig_dry
3 calls (none billed); log in runs/benchrig_dry
```

The raised-table success check also passed a settled cube inside the square and rejected it outside.
