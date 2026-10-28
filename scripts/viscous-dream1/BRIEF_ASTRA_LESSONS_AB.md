# Brief: Astra running lessons + lessons A/B (2026-10-27)

You are Sol, working on gavin-linux in `/home/gavin/dreamtrial/viscous_sim` (the Astra flange harness;
`astra_op/README.md` documents it). Read `overnight_astra_backup.sh`, `ASTRA_LESSONS_PROMPT.md`,
`ASTRA_OPERATOR_PROMPT_BACKUP.md` and `~/dreamtrial/astra_ws/lessons_from_006/` before starting.

## Why

Gavin asked whether Astra "learns in context" (episode 6 took six approaches in one 23-min session and got
much better) and whether that learning can be saved so a new session does not start from scratch. It already
is, partly: after ep 6 Astra wrote `LESSONS.md` from its own transcript, and every later episode read it
(ROV ep 1: first try, 102 commands vs 743). Two gaps:

1. **The lessons never grow.** Five episodes have run since (`astra_rov_g3_20261026_001`,
   `astra_rov_backup_g3_20261026_001..003`, `astra_rov_backup2_g3_20261026_001`), all on ep 6's file. They taught
   new things (e.g. Astra claimed "seated" twice when the sim measured 2 mm short; the wrist housing hit the pipe).
2. **No controlled evidence.** The scene and harness changed between ep 6 and the lessons runs, so we cannot yet
   say how much the lessons help. Gavin wants to be able to claim it with numbers.

You build the tooling and run the no-inference checks. **Do not launch any real Astra operator episode** in
this task; Claude reviews your work and launches the A/B. Consolidation calls (Astra writing lessons, step 3)
are allowed.

## Deliverables (all under `astra_learn/`, plus a report)

### 1. `astra_learn/outcome.py EPISODE_OUTPUT_DIR` -> `outcome.json`

Ground truth from `episode.jsonl` (the hidden sim log; reading it here is post-hoc scoring, never shown to the
operator during an episode). Fields: `success` (relative `nut_turns` >= 2 at the end), `nut_seated`,
`nut_face_gap_mm`, `nut_turns`, `bolt_turns` (where logged), `left_holds_bolt` fraction during tightening (where
logged), `commands`, `sim_s`, `wall_s` (from the Astra log / timestamps), `nut_drops` (define it from the log:
the nut leaving both grippers and the bolt and falling; say in the report exactly how you detect it, and
validate the detector on ep 6, which had 3 drops, and the ROV/backup episodes, which had 0),
`thread_engagements`, `ended_by` (done / command cap / wall cap / crash), `final_claim` (the operator's
`done` text), and `claim_vs_truth` (one plain sentence when they disagree, e.g. "claimed seated; sim 2.0 mm
short"). Run it on all six past episodes (ep 6 and the five above) and put the table in the report.

### 2. `astra_learn/run_episode.sh` (generalise `overnight_astra_backup.sh`)

Args: `--episode NAME --profile PROFILE --prompt FILE [--lessons FILE] [--wall-cap-min 240]`.
Launch the server (same flags as the overnight script, seed 20261024, condition G3), wait for ready, create the
workspace, copy `LESSONS.md` in **only if `--lessons` is given** (and assert it is absent otherwise), run
`codex exec -m gpt-6-astra -c model_reasoning_effort="high"` exactly as the overnight script does, enforce the
wall cap (on expiry: kill Codex, `./robot done "wall cap reached"`, record `ended_by: wall_cap`), wait for the
video export, then write `outcome.json`, the timeline and the audit (`/tmp/audit_astra.py` and `/tmp/timeline.py`
exist; copy them into `astra_learn/` so the pipeline does not depend on `/tmp`). Refuse to start if a sim server
is already running. Before each episode assert that `~/.codex/memories_1.sqlite` table `stage1_outputs` has 0
rows (Codex's cross-session memory; it is empty today, and a non-empty table would contaminate the A/B) and
record the check in `outcome.json`.

Smoke-test it end to end with a trivial operator instead of Astra (add `--operator-cmd` for tests only: e.g. a
shell that runs `./robot status` then `./robot done "smoke"`), once with and once without `--lessons`.

### 3. Running lessons: `astra_learn/consolidate.sh` + `ASTRA_CONSOLIDATE_PROMPT.md`

Versioned store: `astra_learn/lessons/v001.md` = ep 6's current `LESSONS.md` verbatim; `lessons/CHANGELOG.md`
lists each version, its parent, the episode it learned from, and that episode's outcome line.

`consolidate.sh --from vNNN --episode NAME` builds a workspace with `LESSONS.md` (vNNN), `previous_attempt.log`
(the episode's Astra log, as `lessons_from_006` did), and `outcome.txt` (the plain-text `outcome.json`: what the
simulator measured afterwards, including `claim_vs_truth`). Then it runs Astra (`gpt-6-astra`, high) with the
consolidation prompt and saves the result as the next version. Write the prompt so that Astra:
- **revises rather than appends**: keeps what still held, corrects what this episode showed was wrong, merges
  duplicates, and drops what no longer applies;
- marks which lessons are scene-specific (tabletop / ROV one-hand / ROV two-hand) versus general;
- treats `outcome.txt` as ground truth over its own impressions (the overclaiming of "seated" is the example);
- stays under ~100 lines, with no absolute coordinates or joint angles (relative quantities are fine);
- only reads and writes in that directory.

Run the catch-up now, sequentially and in date order, on the five post-ep-6 episodes: v001 + rov_001 -> v002,
+ backup_001 -> v003, + backup_002 -> v004, + backup_003 -> v005, + backup2_001 -> v006. Check each output's
length and rules mechanically (line count, a regex for coordinates); retry once if it breaks them.

### 4. A/B prompts and runner (prepare, do not launch)

- `ASTRA_OPERATOR_PROMPT_BACKUP_NOLESSONS.md`: byte-identical to `ASTRA_OPERATOR_PROMPT_BACKUP.md` except the
  paragraph that mentions `LESSONS.md` is removed. Show the diff in the report.
- `astra_learn/ab.sh`: 6 episodes on `--profile astra_rov_backup2`, alternating arms to spread any drift:
  `astra_ab_L1, astra_ab_N1, astra_ab_L2, astra_ab_N2, astra_ab_L3, astra_ab_N3` (L = with lessons **v001**,
  N = without). v001 is deliberate: it isolates "notes from the 23-minute session in the video" and stays fixed
  so the L arm does not drift. Wall cap 240 min each. After each, append the outcome row to
  `runs/astra_flange/ab/summary.md` and a progress line to `runs/astra_flange/ab/ab.log`. A crashed episode is
  recorded and the chain continues. At the end, write the summary table (per-episode rows, then per-arm:
  successes, seated, median commands / sim time / wall time, total drops, claim-vs-truth mismatches).
- `astra_learn/README.md`: how to run an episode, consolidate, and run the A/B.

## Rules

- No changes to the harness physics, profiles, `screw` or the README the operator sees. This task is
  orchestration and scoring only.
- Never put the operator's workspace in a place where it can see other episodes, the lessons store, or
  `episode.jsonl`. The audit must stay clean (README, LESSONS if given, `./robot`, own images only).
- Use fresh episode names; never overwrite an existing run directory.
- Keep `nohup setsid` patterns; `codex exec ... < /dev/null`; bracket a letter in any `pgrep -f` pattern.

## Report: `astra_lessons_ab_codex_report.md`

The six-episode outcome table, the drop-detector validation, the smoke-test results, v002..v006 with a
short summary of what each version changed, the prompt diff, the exact command to launch `ab.sh`, and
anything you could not do.
