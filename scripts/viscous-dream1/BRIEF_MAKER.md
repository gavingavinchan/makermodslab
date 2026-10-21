# Task: Dream-1 on the MAKER arm (then the METAL arm) in Dream's simulated station, following Dream's own Maker support

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`.
Read first: `BRIEF_2X2.md`, `BRIEF_IK_REACH.md`, `BRIEF_BOX.md` and your reports in `runs/` (codex_2x2_report.md,
codex_ik_reach_report.md, codex_box_report.md). You built `yam_baseline.py`, `cross_scenes.py` (`--rig viscous`,
`--task cube|box`, chunk logging). Dream client source: `~/dreamtrial/.venv/lib/python3.12/site-packages/{dream,dream_ik}`.

## Why
Dream-1 works on its own YAM in vision-only mode (`robot: other_embodiment`, client IK): apple -> bowl, and the
5 x 5 x 3 cm black-and-white box -> black square (`runs/yam_box_vision_live1`). On our Viscous arm it never aims at
the object. Dream officially supports two other arms, the MakerMods **Maker Arm v1** (`ik` arm `"makers"`) and the
**Metal arm** (`ik` arm `"metal"`); Gavin reports the Maker arm works about 60% of the time for them. Our API key
can run only the public base models (`dreamlabs/dream-1-beta`, `-beta-flash`, both trained on YAM), so Dream's
Maker support IS the base model + `ik: {"arm": "makers", "head": "eef"}` (it becomes `other_embodiment`).
Goal: reproduce that support faithfully in sim. If the Maker works, the Viscous problem is its look and we have a
working reference arm; if the Maker also fails, try the Metal.

## How Dream defines the Maker arm (read these, follow them exactly)
- `dream_ik/specs.py` `MAKERS_ARM` and `dream_ik/assets/maker_arm_ik.urdf` (header comment): the vendor chain from
  `makermods-robotics/maker-arm-sdk` `urdf/maker_arm/robot.urdf` with a new fixed root `ik_base_joint` (y-up -> z-up
  about x, then a half turn about z, so the base is YAM's: x forward, y left, z up), the wrist-yaw axis
  (`link_006_joint`) flipped, revolute limits = the station follower's soft limits. Joints `link_002_joint` ..
  `link_007_joint`, zero = the folded calibration pose. TCP 0.1696 m along `link_007` -z. Rest (the median posture of
  94 episodes on a two-arm Maker station): (0, -1.2, 0.81, -0.1, 0, 0). Speed cap 120 deg/s.
- `dream/client_ik.py`, `dream/__init__.py` (`ik`, `robot`): the client sends a YAM-trained model zeros for the
  joints, the hand pose from FK, `state.<side>_gripper` from `state.<side>_grip`, cameras in the model's roles.
  `METAL_ARM` likewise for the Metal (read its spec and asset header, including its joint-sign map).

## The sim arms
- Maker visual + collision model: the MakerMods Lab repo on this machine,
  `~/Documents/makermodslab-viscous/frontend/public/maker-urdf/` (`robot.urdf`, `meshes/`, `README.md`,
  `revision_report.json`), which is the same vendor model. Metal: `.../frontend/public/metal-urdf/`
  (`metal_with_gripper.urdf`). Read-only: copy what you need into the workspace.
- Build a MuJoCo station like `--rig viscous`: two arms at YAM's bases in Dream's scene, Dream's calibrated top
  camera, Dream's objects. Mount each arm so that its base frame equals the IK base frame Dream uses (apply the same
  `ik_base_joint` rotation), and VERIFY: for several random joint vectors, MuJoCo's TCP pose (in the arm's base frame)
  must match `dream_ik` FK for `makers` (and later `metal`) within ~1 mm / 0.5 deg, after the joint mapping
  (including the flipped wrist yaw). Report the check. If our URDF's zero is not Dream's zero, find the offsets.
- Gripper: use the model's own gripper joints if it has them; map grip 0..1 to closed..open. Make sure it can pinch
  the apple and the box (scripted pinch in the dry run, as before).
- Wrist cameras: mount one per arm looking along the jaws like the YAM's (`yam_env` wrist cameras: copy their pose
  relative to the TCP as a starting point). Dream says the Maker camera mount is "not measured", so pick a sensible
  wrist-camera view and show a preview PNG of each camera next to the YAM's.
- Start: both arms at Dream's `rest_pose` for that arm, grippers open. Execution identical to `--mode vision`
  (48 rows/chunk, sim held during inference, `commanded`, grip clipped to [0, 1], joint-limit clip), chunk logging on.

## Runs (stop early as stated)
1. Maker, apple: "put apple into the bowl", live `--max-calls 20`, out `runs/maker_apple_vision_live1`.
2. Maker, box: "pick up the black and white box and put it on the black square", `--max-calls 20`,
   out `runs/maker_box_vision_live1`.
3. ONLY if the Maker never grasps in either run: Metal (`ik` arm `"metal"`), apple, `--max-calls 20`,
   out `runs/metal_apple_vision_live1`; and if that grasps or comes close, Metal box, `--max-calls 20`.

## Hard rules
- Billed: at most 20 calls per run, at most 80 in total. `pool_warming` (DA-2000) may be retried. Dry-run first.
- No hardware: no serial, CAN, /dev/video*, systemd. Only write inside `~/dreamtrial/viscous_sim` (TMPDIR there).
  Do not edit installed packages or the Lab repo. Never print the API key. Keep existing modes working.

## Report (short)
The FK-match check; preview images; per run: billed calls, what the arms did (judged from frames), closest
requested-hand-to-object and object-to-target distances, grasped yes/no, success yes/no. Files added/changed.
