# Shackle Astra chain — 2026-10-28

Implemented host-only orchestration in `shackle_learn/`. No Dream model/client was
called. Model subprocesses use only `codex exec -m gpt-6-astra` with high reasoning,
closed stdin, the A/B approval/sandbox flags, and fresh isolated sibling workspaces.
No edits to `astra_learn/`, `astra_op/`, or the A/B runs. The A/B driver PID 1374358
and existing server PID 1396181 were observed alive during preparation. All 3,132
snapshotted files under astra_learn/astra_op were unchanged after consolidation.

## s001 checked

Produced by Astra from `astra_learn/lessons/v006.md` plus
`shackle_L006_20261028_001`: **89 lines, first attempt, exit 0, no lint errors**.
Memory check: zero stage1_outputs rows. Mechanically checked and manually read.
The immutable notes, unified diff, checks JSON, model log, final response and
CHANGELOG are in `shackle_learn/lessons/`.

- Kept: README authority, fresh camera/depth checks, actual part-axis alignment,
  small final nudges, grasp/clearance planning, reached-motion and load checks,
  support before opening, measured thread advance, and honest seating uncertainty.
- Changed: task success is collar seating on the upper ear; separate upper-ear
  passage, lower-thread engagement, relative advance, and seating. The current
  shackle pitch is 2.5 mm, subject to the next README. Plan the grasp/regrasp for
  reach and finger clearance; apparent descent and small TCP errors do not prove
  insertion. Loaded opening and withdrawal are warning signs, not a release recipe.
- Corrected the transcript: no insertion or engagement was ever measured; two
  drops, zero turns, 263 commands, 397.25 sim s, 1881.44 operator wall s, unseated.
  “Pin entered the upper ear” is unsupported. The scorer's “No disagreement
  detected” was only a wording heuristic and missed that historical claim.
- Dropped: flange nut/bolt backup, ROV one-/two-hand material, old 3.5 mm pitch,
  two-turn success threshold, old successful flange measurements, and scene recipes.
  No successful shackle procedure is invented from the failed episode.

## Exact operator prompt diff

The second prompt is `shackle_learn/ASTRA_OPERATOR_PROMPT_SHACKLE_OWN_LESSONS.md`.
Only the lessons paragraph changes; the original prompt is untouched.

```diff
--- ASTRA_OPERATOR_PROMPT_SHACKLE_LESSONS.md
+++ shackle_learn/ASTRA_OPERATOR_PROMPT_SHACKLE_OWN_LESSONS.md
@@ -9,4 +9,4 @@
 - Stay inside this workspace. Use only `./robot`, `README.md` and files in `obs/`. Do not read other files or directories, or reach the simulator another way.
 - When you believe the task is done, look again, run `./robot done "<what you achieved>"`, then give a short summary.
 
-`LESSONS.md` holds notes you wrote after earlier work screwing a nut onto a bolt in a different (flange) task. Read it first, use what applies to this shackle task, and ignore what is specific to the flange task.
+`LESSONS.md` holds notes you wrote after your own earlier attempts at this same shackle task. Read it first and use those lessons to guide this attempt; the current workspace's README remains authoritative.
```

## Chain design

`consolidate.sh --from FILE --episode NAME --out FILE` gives Astra exactly
LESSONS.md, the episode's complete Astra transcript as previous_attempt.log, and
plain-text outcome.txt in a fresh sibling workspace. It supplies simulator truth
including claim_vs_truth, explicitly explains the wording heuristic, and excludes
host session/accounting paths and the hidden episode.jsonl. Notes are limited to
100 lines with the original coordinate/joint regex plus shackle/general labels.
There is one retry. Existing versions/attempt outputs are refused. Successful
versions get parent/episode/outcome CHANGELOG entries and mechanical evidence.

`chain.sh` has an exclusive lock and refuses existing chain outputs, episode
workspaces/runs/logs, or next lesson versions. It sequentially invokes only the
existing guard-safe `shackle_learn/run_episode.py`; that runner owns its separate
lock, memory checks, fresh operator workspace, server/operator cleanup, video,
scoring and audit. Operator workspaces contain only README.md, robot, obs/, and
the selected LESSONS.md. The boundary remains instructional, as in the A/B.

Plan: s001 -> shackle_S001_20261028_001 -> s002 ->
shackle_S002_20261028_002 -> s003 -> shackle_S003_20261028_003 -> s004.
Each completed episode is consolidated (including a success), then the chain
stops on the first final seated=true or after three new episodes. A missing
outcome or failed consolidation stops for inspection. Failed episodes with
usable outcomes/logs can still inform the next notes.

No episode launches at/after **10:30 CDT**, and the chain stops operation at
**11:30 CDT on 2026-10-28**. The operator cap is at most 240 minutes, shortened
near the deadline to reserve six minutes for queued cleanup and video export.
An absolute watchdog terminates an overrun at 11:30; cleanup/export/summary may
finish later, but no new operator/consolidation starts. An unfinished
consolidation is not published without successful checks.

Progress steps append timestamps to runs/shackle_astra/chain.log. summary.md is
updated at each phase with the v006 baseline and chain rows: episode, lessons,
success, seated, ever inserted/engaged, turns, drops, commands, sim/wall seconds,
ended_by, and final claim versus truth. chain_state.json retains structured data.
Live metrics remain blank until final outcome; baseline's missed entry claim is
explicitly annotated.

## Verification

Passed shell/Python syntax, cutoff boundaries, effective caps, coordinate/line
lint, and heuristic-miss annotation. Mocked host-flow checks covered three
failures, first seated success, start cutoff, missing outcome, consolidation
failure, and episode deadline. Harmless real child-process tests checked absolute
deadline termination. Mocked consolidation checked exact three-file isolation,
Astra flags, one lint retry, immutable versions, diff/checks and changelog. These
checks made no model calls and ran no simulator. The real s001 consolidation is
the only model work performed before chain launch.

## Detached launch and progress

```bash
cd /home/gavin/dreamtrial/viscous_sim
MUJOCO_GL=egl PYTHONDONTWRITEBYTECODE=1 \
TMPDIR=/home/gavin/dreamtrial/viscous_sim/shackle_learn/.cache/tmp \
nohup setsid shackle_learn/chain.sh \
  > runs/shackle_astra/chain_launcher.log 2>&1 < /dev/null &
```

Exact progress check:

```bash
cd /home/gavin/dreamtrial/viscous_sim
cat runs/shackle_astra/summary.md
tail -n 30 runs/shackle_astra/chain.log
```

Launch confirmation is recorded below after the first operator starts.

Confirmed at **2026-10-28T04:23:02-05:00**: detached chain PID **1574919**, runner PID **1574926**, shackle server PID **1574927**, Astra operator PID **1575194**. First episode **shackle_S001_20261028_001** has started with **gpt-6-astra/high**, session `01a181d1-0f79-70d2-adca-d8edd101c09e`, **240-minute** cap, and the checked s001 hash. Exactly one shackle server is live. Initial operator inventory is exactly `LESSONS.md`, `README.md`, `obs/`, `robot`. The A/B driver and original A/B server remain alive; post-launch hashes of all 3,132 protected files also match. Evidence: `shackle_learn/chain_launch_confirmation.json`. **Stopped monitoring after this launch confirmation; did not wait for the chain.**
