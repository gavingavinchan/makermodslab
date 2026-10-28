# Astra running lessons and controlled comparison

Host-side tools only. Do not put this directory, hidden logs, outcomes, or other
transcripts in an operator workspace. The server creates a fresh sibling workspace
under `~/dreamtrial/astra_ws/NAME` containing robot, README.md and obs; the runner
adds LESSONS.md only when explicitly requested. This is the existing harness's
instruction boundary, not an OS sandbox. Physics and operator README are unchanged.

From `/home/gavin/dreamtrial/viscous_sim`:

```bash
# Post-hoc score (writes EPISODE/outcome.json; never available to the live operator)
python3 astra_learn/outcome.py runs/astra_flange/EPISODE

# Real episode, with a fresh name and optional notes. Requires no other sim server.
astra_learn/run_episode.sh --episode FRESH --profile astra_rov_backup2 \
  --prompt ASTRA_OPERATOR_PROMPT_BACKUP.md --lessons astra_learn/lessons/v001.md

# Without notes: omit --lessons and use the no-lessons prompt.
astra_learn/run_episode.sh --episode FRESH_N --profile astra_rov_backup2 \
  --prompt ASTRA_OPERATOR_PROMPT_BACKUP_NOLESSONS.md --wall-cap-min 240

# Test only, no model inference; status/done produces a short exported video.
astra_learn/run_episode.sh --episode FRESH_SMOKE --profile astra_rov_backup2 \
  --prompt ASTRA_OPERATOR_PROMPT_BACKUP_NOLESSONS.md \
  --operator-cmd './robot status && ./robot done "smoke"'

# Revise notes from a completed episode; invokes Astra, never the robot.
astra_learn/consolidate.sh --from v006 --episode COMPLETED_EPISODE

# Review the fixed six-episode plan without starting anything.
astra_learn/ab.sh --dry-run

# After review, launch the real A/B (up to 24 operator hours plus export).
nohup setsid astra_learn/ab.sh > runs/astra_flange/ab_launcher.log 2>&1 < /dev/null &
```

The runner uses gpt-6-astra/high with the overnight runner's Codex flags and stdin
closed, seed 20261024 and G3, and waits for server readiness and complete shutdown
(video export) before scoring, timeline and audit generation. Its wall cap starts
at operator launch; expiry kills the entire operator process group, requests
`./robot done "wall cap reached"`, and persists `ended_by: wall_cap`. The existing
server's independent six-hour cap and 1000-command cap remain in force. Cleanup
may wait for a command already in flight (up to the 300 s screw timeout).

Every launch checks SQLite `~/.codex/memories_1.sqlite`, `stage1_outputs`, read-only,
and refuses if missing/unreadable/nonempty. Successful checks and timestamps are
saved in the sibling `NAME_runner.json` and `outcome.json`. A nonblocking runner
lock and bracketed pgrep prevent concurrent starts. Existing output directories,
workspaces and sibling logs are refused. Failures after reservation still get an
outcome/audit; preflight refusals are recorded as crashes by the A/B driver.

Runner stdout is suitable for a host log. Server, Astra stdout/final text and
runner metadata are sibling files `NAME_server.log`, `NAME_astra.log`,
`NAME_astra_final.md`, `NAME_runner.json`. Outcome, timeline and audit are inside
the hidden output directory. `audit_astra.py` and `timeline.py` were copied from
the original /tmp tools; the former is a heuristic tool/path inventory for human
review, not a security proof. Test operators have no Codex session; their audit
records the exact command and initial workspace inventory.

Scoring uses final relative nut turns >=2 for success, simulator nut_seated/gap
for seating, unique completed command numbers (includes done), final sim time,
and Codex session timestamps for historical wall time. New wall time is measured
by the runner's monotonic clock from operator launch through exit. A drop is one
continuous interval without either gripper's nut attachment or thread engagement
in which world z falls >=10 mm from the interval's first physics sample. Re-support
rearms the detector. Thread engagements count `engage` events in physics rows.
The left-head hold fraction is time weighted during positive `turn right` or
`screw right` commands, including internal regrasp phases; absent telemetry is null.
Claim mismatch detection covers positive seated/snug claims and common explicit
turn-count/success claims; it is a deterministic language heuristic, not a complete
semantic judge. Null means no detected disagreement, not independently proved prose.

Consolidation uses a fresh isolated sibling workspace with only current notes,
that episode's transcript, and post-hoc outcome.txt. It revises LESSONS.md, lints
<=100 lines and coordinate/joint patterns plus scene labels, and retries once if
necessary. Versions are immutable, with diffs, validation results, model logs and
parent/episode/outcome entries in lessons/CHANGELOG.md. Regex lint is conservative;
human review is still appropriate for semantic rules. v001 is verbatim ep-6 notes.

A/B order is L1,N1,L2,N2,L3,N3; L always uses v001, never running/latest notes.
The A/B directory must be new. Each completed/crashed run appends a row to
`runs/astra_flange/ab/summary.md` and a timestamp to ab.log. A crash does not halt
the chain. Final per-arm medians include available failed/capped metrics; missing
crash metrics are excluded and success counts retain the full denominator. The
outcomes.json file preserves all per-run records. No A/B launch occurs in dry-run.
