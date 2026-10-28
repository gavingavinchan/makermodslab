# Shackle Astra preparation — 2026-10-28

**Ready for Claude to launch. No real Astra episode, billed Dream call, hardware operation, or A/B modification was performed.** Both local smoke operators exited successfully. The A/B driver, runner and server remained alive; no shackle server remains running.

The old key produced a jagged shadow-map terminator across the curved steel; disabling shadows hid it but flattened the scene. In `shackle_scene.py`, restored key shadows, moved the key from `(-.8,.2,2.7)` to `(.8,-.7,2.0)` aimed along `(-.6,.8,-1.2)`, raised `shadowsize` 4096→8192, and set `shadowscale=.8`. Light placement removes the broad white patches from the inspected bow; robot, table and part shadows return. Tiny raster serrations remain at some hard shadow edges. Stainless, normals, geometry and fill are unchanged. This follows MuJoCo's [shadow-volume/light-placement guidance](https://mujoco.readthedocs.io/en/latest/XMLreference.html#body-light).

Before/after: [detail](runs/shackle_look/v2/detail_comparison.jpg), [front](runs/shackle_look/v2/front_comparison.jpg), [top](runs/shackle_look/v2/top_comparison.jpg). Full PNGs are in `runs/shackle_look/v2/{before,after}/`. [Validation](runs/shackle_look/v2/validation.json): only key-light and visual shadow settings differ in XML; all remaining XML—including cameras, layout and physics—is identical. Across 482 compiled arrays only light arrays differ. Five seconds of contact dynamics matched bit for bit; ten legacy task/wrist XML hashes and 7,527 protected source/asset/accounting hashes matched. [Source diffs](runs/shackle_look/v2/shackle_scene.py.diff).

[Launcher](shackle_learn/run_episode.py) uses its own lock and checks actual Python argv for shackle servers only. It reserves fresh output and sibling `~/dreamtrial/astra_ws/NAME` workspaces, starts `shackle_astra.py` directly (table/T, seed 20261028), and uses exactly the A/B's Astra model/effort/Codex flags. It records the same read-only memory check, copies optional lessons, asserts lessons absent otherwise, enforces the operator wall cap and waits for complete server/video shutdown before outcome/timeline/audit. Success means final **seated**; outcome includes inserted, engaged, turns, signed seat gap, commands, sim/operator wall seconds, drops, end reason and final done claim with a plain truth comparison.

The adapter now uses `runs/shackle_astra/` and sibling workspaces. Its shared server import redirects only the unconditional cache-root binding to `shackle_learn/.cache`; shared source is read-only. Record, workspace, command dispatch and screw methods are unchanged. Video saves exact states during commands and exports **H.264/yuv420p, 1280×400, 10 fps**, top + ears/threads close-up after shutdown. Both smoke movies fully decoded (13 frames each). [Archived-pose preview](runs/shackle_look/v2/video_preview.mp4) and [frame](runs/shackle_look/v2/video_insertion_frame.png) verify actual insertion/thread visibility through the same compositor; this replay executed no new task or operator. [Video checks](runs/shackle_look/v2/video_validation.json).

| Smoke | Lessons | Commands / sim s / operator wall s | Result |
|---|---|---|---|
| `shackle_smoke_20261028_L` | v006, byte-identical copy | 2 / 1.2 / 5.20 | done “smoke”, unseated, 0 drops |
| `shackle_smoke_20261028_N` | asserted absent | 2 / 1.2 / 5.20 | done “smoke”, unseated, 0 drops |

Each test ran `./robot status && sleep 5 && ./robot done "smoke"`; the pause allowed the host guard inspection. Both had **0 memory rows**, operator/server exit 0, complete outcomes, timelines and initial/final workspace audits. Concurrent launches and reused names were refused. Scoring fixtures passed seating-only success, continuous drop/re-support, and positive/negative seating-claim checks.

[Exact guard proof](runs/shackle_look/v2/guard_summary.json): `pgrep -f '[a]stra_op/server.py'` returned the identical prelaunch PID set while shackle PIDs **1470104** and **1470489** were ready; neither shackle PID matched. Existing matches were A/B server **1396181** and preparation parent command text quoting the task/script. The guard was not expected to return empty while the A/B's own server runs. [Full smoke evidence](runs/shackle_look/v2/smoke_proof.json) confirms **3,147 files unchanged**, including all `astra_learn/`, all `astra_op/` including cache, A/B controller outputs, and every ledger/lock checked. The active A/B's own episode log naturally continues under its owner.

The [lessons prompt diff](shackle_learn/prompt.diff) is exactly one appended paragraph; every byte of the original prompt is preserved:

```diff
+
+`LESSONS.md` holds notes you wrote after earlier work screwing a nut onto a bolt in a different (flange) task. Read it first, use what applies to this shackle task, and ignore what is specific to the flange task.
```

Exact real launch command, also in [LAUNCH.md](shackle_learn/LAUNCH.md); **written only, not executed**:

```bash
cd /home/gavin/dreamtrial/viscous_sim
MUJOCO_GL=egl PYTHONDONTWRITEBYTECODE=1 \
TMPDIR=/home/gavin/dreamtrial/viscous_sim/shackle_learn/.cache/tmp \
nohup setsid /home/gavin/dreamtrial/.venv/bin/python shackle_learn/run_episode.py \
  --episode shackle_L006_20261028_001 \
  --prompt ASTRA_OPERATOR_PROMPT_SHACKLE_LESSONS.md \
  --lessons astra_learn/lessons/v006.md \
  --wall-cap-min 240 \
  > runs/shackle_astra/launcher_shackle_L006_20261028_001.log 2>&1 < /dev/null &
```
