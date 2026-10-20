# Task: does the Viscous fail because the model aims wrong, or because the arm cannot reach where it aims?

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`.
Read `BRIEF_2X2.md` and `runs/codex_2x2_report.md` first (your previous task). Dream client source:
`~/dreamtrial/.venv/lib/python3.12/site-packages/{dream,dream_ik}`; `dream/client_ik.py` is the client-side IK.

## Where we are
- YAM, Dream's scene, "put apple into the bowl", vision mode (`robot: other_embodiment`, client IK, state
  withheld): works (`runs/yam_vision_live1`).
- Viscous in the same scene, same prompt, same executor (`--rig viscous`): rises near its base, never touches the
  apple (`runs/viscous_apple_vision_live1`). A scripted Viscous pinch can lift the apple, so the gripper is fine.
- Two explanations: (a) the model is confused by the Viscous's look and asks for hand poses that are wrong;
  (b) it asks for sensible hand poses (near the apple) but the Viscous IK cannot reach them (position or
  orientation), so `dream_ik`'s solve compromises (it leans toward the arm file's `rest`) and the hand goes elsewhere.
  The logs record where the hand went, but not what the model asked for. This task records both and replays.

## Steps
1. Instrument `yam_baseline.py` (vision mode only; keep station mode's behaviour unchanged). Per call save, to a
   `chunks.jsonl` in the run directory: the anchor hand pose the client sent for each side (`state.<side>_eef_xyz`,
   `state.<side>_eef_rot6`, as the client computed them; get them without changing what is sent, e.g. by
   wrapping `ClientIK.observe` to record its return value), the reply's `per_key` hand poses and grips for each side
   (`action.<side>_eef_xyz`, `_eef_rot6`, `_grip`, 48 rows), and the solved joint chunk. For every executed row
   also compute the IK tracking error: FK of the solved joints (dream's own FK for that arm, the same tool point)
   versus the requested pose: position error (m) and rotation error (deg). Also log the arm joints per step in
   `trajectory.json`. In the Viscous rig, clip the reply's grip to [0, 1] with a logged notice instead of rejecting
   the chunk (the model answered -0.089 once). Dry runs of all existing modes must still pass.
2. Live, YAM reference: `--mode vision --live --max-calls 15 --out runs/yam_vision_live2` (apple task). This gives
   the hand path of a successful run and YAM's own tracking error as a reference.
3. Live, Viscous: `--mode vision --rig viscous --live --max-calls 15 --out runs/viscous_apple_vision_live2`.
4. Offline replay, no calls: write `replay_hand_path.py` that takes the per-call requested hand poses and grips from
   `runs/yam_vision_live2/chunks.jsonl` and drives the Viscous in the Viscous apple scene open loop, with the same
   client-side solve the live path uses (dream's IK on the Viscous arm file, the same rows executed per chunk, the
   chunk's poses in each arm's own base frame exactly as the model gave them to YAM). Report per row the tracking
   error, and whether the Viscous picks up the apple and puts it in the bowl. Save frames + contact sheet + video to
   `runs/replay_yam_path_on_viscous/`. If the Viscous IK needs a seam/start that differs, explain what you did.
5. Analyse and report, comparing:
   - YAM live2 vs Viscous live2: tracking errors (median and 95th percentile, position and rotation); where the
     model's requested left-hand positions go relative to the apple (min distance from requested xyz to the apple, in
     the arm's base frame), for both arms. If the model asks the Viscous to go to the apple but the hand does not
     get there, that is (b); if the requests never go near the apple, that is (a).
   - The replay: can the Viscous follow YAM's successful hand path (errors), and does the pick succeed?
   - Which joints sit at limits or at the solve's rest pull when the Viscous misses.

## Hard rules
- Billed: at most 15 live calls per run, at most 30 live calls in total. `pool_warming` (DA-2000) refusals are not
  billed and may be retried. No other live calls.
- No hardware: no serial, CAN, /dev/video*, systemd. A real robot is connected to this machine; leave it alone.
- Only write inside `~/dreamtrial/viscous_sim` (temporary files too: set TMPDIR inside it). Never print or copy the
  API key. Do not edit installed packages or `viscous_arm.json`. Keep existing modes and scripts working.

## Report (final message, short)
Billed calls per run; the numbers above in a small table; your verdict (a) or (b) or both, with the evidence; the
replay result; files added/changed.
