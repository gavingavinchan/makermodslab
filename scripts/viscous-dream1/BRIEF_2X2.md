# Task: fill the 2x2 — is the Viscous failure the ARM or the TASK?

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`.
Read `BRIEF_YAM_BASELINE.md` and your own report from that task first (you wrote `yam_baseline.py` changes,
`runs/yam_*_live1`). Dream client source: `~/dreamtrial/.venv/lib/python3.12/site-packages/{dream,dream_ik,dream_examples}`.

## What we know
- YAM in Dream's own scene, "put apple into the bowl", works in BOTH modes, including `vision`
  (`robot: other_embodiment`, client IK, state withheld): apple in the bowl by call ~6 (`runs/yam_vision_live1`).
- The Viscous arm, "pick up the white cube and place into black square" (`viscous_dream_sim.py --rig yam`,
  `runs/live_yamrig_1`): meaningless motion, gripper hovering 13-23 cm above the table, cube never touched.
- The Viscous arm file's hand frame matches Dream's convention (checked with dream's FK, `frame_check.py`).

Those two runs differ in both the ARM and the TASK/scene. Fill in the other two cells:

| | apple -> bowl (Dream's scene) | cube -> square |
|---|---|---|
| YAM | works | **Run A** |
| Viscous | **Run B** | fails |

## Run A: YAM, Dream's scene, our cube/square task (vision mode)
Add a `--task cube` option to `yam_baseline.py` (or a small helper module beside it) that, in Dream's YAM scene,
replaces the apple with a ~2 cm WHITE cube at the apple's spawn position and replaces the bowl with a thin BLACK
square outline (~4.5 cm outer, 4 mm line, flat on the table, no collision) at the bowl's position. Leave the other
objects. Build it by editing the XML string that `yam_env.build_xml()` returns, in our own code; never edit the
installed package. Prompt: "pick up the white cube and place into black square". Success: the cube resting inside
the square. Run in `--mode vision`, live, `--max-calls 20`, out `runs/yam_cube_vision_live1`.

## Run B: Viscous, Dream's scene, Dream's apple task (vision path, as the Viscous must run)
Put the Viscous arms in Dream's station with Dream's objects and prompt. `yam_rig.py` already transplants Dream's
table, lights and calibrated top camera around two Viscous arms placed at YAM's bases; extend that (or write a new
module) so the scene also has Dream's apple, orange, banana, bottle, bowl and plate exactly where `yam_env` puts
them (same world frame as the arms and camera; reuse Dream's asset builders from `yam_env` rather than copying
meshes). Prompt: "put apple into the bowl".

Keep the execution IDENTICAL to `yam_baseline.py --mode vision` so only the arm differs:
- client config as `vision_config` but `ik: {"arm": "<abs path to viscous_arm.json>", "head": "eef"}`;
  embodiment = Dream's YAM_EMBODIMENT; robot left to default (it becomes other_embodiment).
- 9-frame windows at 5 Hz with Dream's crop/resize (`capture`/`feed`), sim held while a request is in flight,
  48 rows executed per chunk, 20 Hz rows, `commanded` passed each call.
- NO extra per-row speed clipping of our own (the old `viscous_dream_sim.py` clipped joints at 2 rad/s per row and
  ran only 30 of 48 rows; Dream's IK already respects the arm file's speed caps). Do clip to joint limits.
- Start: Viscous joints at neutral (all zeros) with the grippers OPEN (Dream's YAM resets open), both arms.
- Right arm: also a Viscous at YAM's right base, executing its own chunk (as YAM's right arm does in the baseline).
Before going live: a dry run, and a preview PNG of the top camera next to `runs/yam_vision_live1/frame_00.png`,
so the framing can be compared. Check the apple is graspable by the Viscous gripper (finger opening 2 x 52 mm) with
a short scripted pinch in the dry run if cheap; skip it if not.
Live: `--max-calls 20`, out `runs/viscous_apple_vision_live1`.

## For both runs
Log per call: object-to-target distance, object height, each hand's TCP xyz, gripper commands, limit notices.
Save frames (third-person | model's top camera) and a contact sheet. Judge plainly from the frames what each arm did.

## Hard rules
- Billed: at most 20 live calls per run, at most 45 live calls in total for this task. `pool_warming` (DA-2000)
  refusals are not billed and may be retried. Never loop live calls otherwise.
- No hardware: no serial, CAN, /dev/video*, systemd. A real robot is connected to this machine; leave it alone.
- Only write inside `~/dreamtrial/viscous_sim`. Keep Dream's logs and caches inside it too (`DREAM_LOG_DIR`).
  Never print or copy the API key. Do not edit installed packages or `viscous_arm.json`. Keep `viscous_dream_sim.py`
  and `yam_baseline.py`'s existing modes working.

## Report (final message, short)
Per run: command, billed calls, what the arms did, closest object-to-target distance, success yes/no, oddities.
Then: files added/changed and why, and how Run B's top-camera framing compares with the YAM baseline's.
