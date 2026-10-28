# Task: shackle shadows back + a shackle Astra launcher that runs beside the A/B (NO Astra episode yet)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`.
You built the shackle scene, the look fix and the layout v2: read `runs/shackle_scene/shackle_scene_report.md`,
`runs/shackle_look/shackle_look_report.md`, `shackle_astra.py`, `astra_learn/runner.py`, `astra_learn/outcome.py`,
`astra_learn/README.md`.

## Why (Gavin, 2026-10-28)
Next is Astra (`gpt-6-astra`) doing the shackle task in sim, overnight while Gavin sleeps. Two blockers:
1. Looks: disabling shadows removed the white patches but the whole scene is now flat (no robot or table
   shadows), and Gavin wants the video to "look nice". Restore shadows for the table, arms and parts, and remove
   the acne another way (e.g. `<quality shadowsize>` up, a tighter light cutoff/extent, shadow bias via light
   placement, splitting smooth/crease normals on the bow, whatever actually works in MuJoCo 3.16). Render-only:
   physics, layout, cameras and other tasks stay byte-identical. Before/after renders of the detail, front and top
   views in `runs/shackle_look/v2/`. If shadows cannot be made clean, say why and keep the best compromise.
2. Launcher: an Astra lessons A/B (`astra_learn/ab.sh`) is running and will for hours. Its runner refuses to
   launch an episode while any process matching `pgrep -f '[a]stra_op/server.py'` exists, so a shackle episode
   started through `astra_op/server.py` would make the A/B's next episode crash. The shackle profile does not need
   that file: `shackle_astra.py` has its own `main()`. Each episode's socket lives in its own output dir.

## Build `shackle_learn/run_episode.py` (new dir; do not edit `astra_learn/`)
Modelled on `astra_learn/runner.py`, for the shackle profile, and safe to run beside the A/B:
- start the server as `python shackle_astra.py --profile shackle --scene table --condition T --seed 20261028
  --episode NAME --output runs/shackle_astra/NAME` (its command line must NOT contain `astra_op/server.py`);
  its own lock file, refuse only if another *shackle* server runs; fresh episode/workspace names only
  (workspace under `~/dreamtrial/astra_ws/`);
- `--prompt FILE`, optional `--lessons FILE` (copied in as `LESSONS.md` only if given, asserted absent
  otherwise), `--wall-cap-min` (default 240), `--operator-cmd` for tests only; operator =
  `codex exec -m gpt-6-astra -c model_reasoning_effort="high" ...` exactly as the A/B runner does;
- the same Codex-memory check as the A/B runner (`stage1_outputs` must have 0 rows) recorded in the metadata;
- after the operator ends: wait for the video export, then write `outcome.json` from the hidden log (success =
  seated; plus inserted, engaged, turns, seat gap, commands, sim/wall seconds, pin drops, ended_by, the final
  claim and a plain `claim_vs_truth` sentence), the timeline and the workspace audit, like the A/B pipeline.
- the episode video must show the threads and the pin entering the ears (top view + a close-up), H.264.
Also write `ASTRA_OPERATOR_PROMPT_SHACKLE_LESSONS.md`: identical to `ASTRA_OPERATOR_PROMPT_SHACKLE.md` plus one
paragraph saying `LESSONS.md` holds notes the operator wrote after earlier work screwing a nut onto a bolt in a
different (flange) task: read it first, use what applies, and ignore what is specific to that task. Show the diff.

## Test, but do not launch Astra
Smoke-test the launcher end to end with a trivial `--operator-cmd` (`./robot status`, then `./robot done "smoke"`),
once with `--lessons astra_learn/lessons/v006.md` and once without, while the A/B keeps running: prove the A/B's
guard does not see your server (run its exact pgrep while yours is up) and that its files and ledger are untouched.
Then write the exact launch command for the real episode to `shackle_learn/LAUNCH.md` and stop. Claude launches it.

## Report: `shackle_astra_prep_codex_report.md` (short)
Shadow fix (cause, change, before/after), launcher design, smoke results, the guard proof, the prompt diff, launch
command. Hard rules: no billed Dream calls, no real Astra episode, don't stop/touch the A/B, `astra_learn/`,
`astra_op/` or other profiles; no hardware; only write inside `~/dreamtrial/viscous_sim` and `~/dreamtrial/astra_ws/`.
