# Task: Astra's in-context learning loop on the shackle task (overnight chain; NO billed Dream calls)

Work in `~/dreamtrial/viscous_sim` on gavin-linux. Read `shackle_learn/README.md`, `shackle_learn/PREP_REPORT.md`,
`astra_learn/README.md`, `astra_learn/consolidate.sh`, `astra_learn/ASTRA_CONSOLIDATE_PROMPT.md`, and episode 1:
`runs/shackle_astra/shackle_L006_20261028_001*` (outcome.json, timeline.txt, audit.txt, the Astra log).

## Why (Gavin, 2026-10-27/28)
Gavin's question is whether Astra (`gpt-6-astra`) "learns in context", and whether that learning carries over
(to a new task now, to the real arm later). Episode 1 on the shackle (flange lessons v006 as LESSONS.md) failed:
it grasped and carried the pin, dropped it twice during regrasps, the second time off the table, and ended with an
honest "incomplete" after 31 min (263 commands). Its claim "pin entered the upper ear" is not confirmed by the sim
(`ever_inserted` false). Gavin is asleep; run the loop now so he has results in the morning.

## Build (new files under `shackle_learn/` only; do not edit `astra_learn/`)
1. `shackle_learn/consolidate.sh --from FILE --episode NAME --out FILE`: like `astra_learn/consolidate.sh`, a fresh
   workspace with `LESSONS.md` (FILE), `previous_attempt.log` (that episode's Astra log), `outcome.txt` (plain-text
   outcome.json: what the simulator measured, including claim_vs_truth), then Astra (`gpt-6-astra`, high) with a
   consolidation prompt based on `ASTRA_CONSOLIDATE_PROMPT.md`, adapted: the new notes are for the **shackle pin
   task**; keep general lessons from the flange notes that still hold, correct what this attempt showed wrong,
   drop flange-only details, treat `outcome.txt` as ground truth over its own impressions, under ~100 lines, no
   absolute coordinates or joint angles. Same mechanical checks (line count, coordinate regex, one retry).
   Store as `shackle_learn/lessons/s001.md`, `s002.md`, ... with a `CHANGELOG.md` (parent, episode, outcome line).
2. `shackle_learn/chain.sh`: starting from s001 (consolidated from v006 + episode 1), repeat: run an episode with
   `run_episode.py` (prompt `ASTRA_OPERATOR_PROMPT_SHACKLE_LESSONS.md`, but change its lessons paragraph for
   s-files to say the notes are from the operator's own earlier attempts at **this same shackle task**: make that
   a second prompt file, `ASTRA_OPERATOR_PROMPT_SHACKLE_OWN_LESSONS.md`, and show the diff), wall cap 240 min,
   names `shackle_S00k_20261028_00k`; then consolidate it into the next s-file. Stop after the first seated
   success, or after 3 chained episodes, or at 11:30 CDT (do not start a new episode after 10:30 CDT). Append one
   progress line per step to `runs/shackle_astra/chain.log` and keep `runs/shackle_astra/summary.md` (a table:
   episode, lessons file, success, seated, ever inserted/engaged, turns, drops, commands, sim s, wall s, ended_by,
   claim vs truth).
3. Run consolidation for s001 yourself first and check it, then **launch `chain.sh` detached** (`nohup setsid ...
   < /dev/null &`) and confirm its first episode started. Then stop: do not wait for the chain.

## Hard rules
- No billed Dream calls; the only model use is Astra (`gpt-6-astra`) via `codex exec`, as in the A/B.
- An A/B (`astra_learn/ab.sh`) is running: never start `astra_op/server.py`, don't touch `astra_learn/`,
  `astra_op/` or its runs. Use only `shackle_learn/run_episode.py` (it is guard-safe). One shackle episode at a time.
- Never put a workspace where the operator can see other episodes, the lessons store or `episode.jsonl`; fresh
  names only; never overwrite a run. Bracket a letter in any `pgrep -f` pattern, and don't let a wait loop match its
  own command line.
- Report: `shackle_learn_chain_codex_report.md`: s001 summary (what it kept, changed, dropped vs v006), the prompt
  diff, the chain design, and the exact command to check progress.
