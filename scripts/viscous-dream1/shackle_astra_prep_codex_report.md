Prepared and smoke-tested. **No real Astra episode or billed Dream call was launched. The A/B remains running.**

Shadows are restored. Moving the key light toward the inspection side, using an 8192 shadow map and `.8` shadow scale removed the broad white patches; minor hard-edge serrations remain. Physics, layout and cameras are unchanged; legacy XML hashes matched. Before/after: [detail](runs/shackle_look/v2/detail_comparison.jpg), [front](runs/shackle_look/v2/front_comparison.jpg), [top](runs/shackle_look/v2/top_comparison.jpg).

The [launcher](shackle_learn/run_episode.py) uses a separate lock, standalone shackle server, fresh sibling workspaces, the A/B’s exact Astra flags and memory check, optional lessons, and post-export scoring/audits. H.264 top-plus-close-up video fully decoded.

Both lessons/no-lessons smoke runs passed: two commands, 1.2 simulation seconds, zero drops, `done "smoke"`, and zero memory rows. Concurrent/reused launches were refused. The exact A/B `pgrep` result stayed unchanged while both shackle servers were ready; neither shackle PID matched. **3,147 protected files—including A/B files and ledgers—were unchanged.** [Evidence](runs/shackle_look/v2/guard_summary.json).

The [prompt diff](shackle_learn/prompt.diff) appends only:

> `LESSONS.md` holds notes you wrote after earlier work screwing a nut onto a bolt in a different (flange) task. Read it first, use what applies to this shackle task, and ignore what is specific to the flange task.

Claude’s exact launch command is saved in [LAUNCH.md](shackle_learn/LAUNCH.md), **not executed**:

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

[Detailed report](shackle_learn/PREP_REPORT.md).