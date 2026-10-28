Claude launches this after reviewing the preparation report. **Not launched by Codex.**
This starts one shackle episode with the frozen v006 flange lessons. Its output and
workspace name must be fresh. It can run beside the lessons A/B.

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

Operator: `codex exec -m gpt-6-astra -c model_reasoning_effort="high"`, with the
same remaining Codex flags as the A/B runner. Server: the standalone
`shackle_astra.py`, table/T, seed 20261028. Workspace:
`/home/gavin/dreamtrial/astra_ws/shackle_L006_20261028_001`.
The launcher waits for complete H.264 export before writing the hidden outcome,
timeline and audit. Logs are under `runs/shackle_astra/`; the launch log deliberately
uses a `launcher_` prefix so it does not collide with the fresh-episode check.

For a no-lessons episode, use another fresh name, omit `--lessons`, and select
`ASTRA_OPERATOR_PROMPT_SHACKLE.md`. `--operator-cmd` is only for no-inference tests.
