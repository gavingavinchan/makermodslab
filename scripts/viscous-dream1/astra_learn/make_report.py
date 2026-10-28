#!/usr/bin/env python3
"""Regenerate the implementation report from saved evidence; no inference or launches."""
import hashlib
import json
from pathlib import Path
import subprocess as sp
from outcome import ROOT, RUNS, PAST, score
from consolidate import validate


def fmt(x, digits=2):
    return '—' if x is None else f'{x:.{digits}f}' if isinstance(x,float) else str(x)


def main():
    outcomes=[score(RUNS/n) for n in PAST]
    text = '''# Astra running lessons and A/B preparation — 2026-10-27

Implemented the post-hoc scorer, episode runner, versioned consolidation pipeline,
fixed-v001 alternating A/B, and usage documentation under `astra_learn/`.
No real Astra operator episode or A/B episode was launched. The only inference calls
were the five authorized sequential lesson consolidations.

## Historical outcomes

Success means final **relative** nut turns >=2. Seating is a separate simulator measurement.
Commands include `done`. All six episodes ended by operator `done`. Missing bolt/hold
telemetry is shown as —; it is not assumed zero. The historical memory checks were not
recorded and are explicitly marked unchecked in their outcome files.

| Episode | Success | Seated | Gap mm | Relative nut turns | Bolt turns | Left hold fraction | Commands | Sim s | Wall s | Drops | Engagements |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
'''
    for o in outcomes:
        vals=[o['episode'],o['success'],o['nut_seated'],fmt(o['nut_face_gap_mm'],3),fmt(o['nut_turns'],3),
              fmt(o['bolt_turns'],4),fmt(o['left_holds_bolt'],4),o['commands'],fmt(o['sim_s']),fmt(o['wall_s']),o['nut_drops'],o['thread_engagements']]
        text+='| '+' | '.join(map(str,vals))+' |\n'
    text+='''
Historical wall time is the first-to-last timestamp in the recorded Codex session,
identified by its session ID in the Astra log. Episode 6's approximately 23 minutes
is **simulation time** (1410.15 s); its Codex session spans 9701.02 s (~162 min).
New episodes use the runner's monotonic operator-start-to-exit interval. Video export
is excluded from operator wall time. Left hold fraction is time weighted over positive
right-hand `turn`/`screw` commands, including internal release/reset phases.

Each hidden output directory now has `outcome.json`, including the full operator
`done` claim. Those claims and detected disagreements are reproduced here:

'''
    for o in outcomes:
        text+=f"- **{o['episode']}**: “{o['final_claim']}”\n  Scorer: {o['claim_vs_truth'] or 'No detected disagreement.'}\n"
    text+='''
The two false seating claims were backup 002 (2.03 mm gap) and backup 003 (2.21 mm).
The turn-count check additionally flags backup 003 and backup2 001; reported hand/world
rotation can exceed the simulator's final relative thread count. Claim comparison is a
limited deterministic language heuristic, not a complete semantic judge. These historical
runs changed scene/harness, so their command/time differences do not establish a causal
lessons benefit. The prepared controlled comparison has not run.

## Drop detector validation

Process physics samples in order. A supported nut has `nut_engaged` true or a `nut`
attachment in either gripper. Start an unsupported interval at its first physics sample.
Count one drop when world z falls at least 10 mm below that sample; count no more until
thread/gripper support returns. This rejects supported downward motions, threaded screw
releases, and stationary unengaged perching. The threshold is explicit; it is not a
claim to detect every possible slip.

| Episode | Expected | Detected | Unsupported start → detection (sim s) |
| --- | --- | --- | --- |
'''
    for o in outcomes:
        times=', '.join(f"{e['start_sim_s']:.2f} → {e['detected_sim_s']:.2f}" for e in o['drop_events']) or '—'
        text+=f"| {o['episode']} | {3 if o['episode']==PAST[0] else 0} | {o['nut_drops']} | {times} |\n"
    text+='''
These match the three known episode-6 drops and zero later drops. Evidence is saved
in `astra_learn/historical_validation.json`; scorer fixtures separately check support,
continued falling without recount, and rearming after recovery.

## No-inference verification

The final with/without-lessons smoke runs used only `./robot status` then
`./robot done "smoke"`; their failures of task success are expected (no manipulation).
Additional smoke runs exercised process-group wall termination and an operator exit 7.

| Smoke episode | Lessons | Commands | Ended by | Operator exit | Outcome / video / timeline / audit |
| --- | --- | --- | --- | --- | --- |
'''
    for n in ['learn_smoke_N_20261027_02','learn_smoke_L_20261027_02','learn_smoke_cap_20261027_01','learn_smoke_crash_20261027_01']:
        p=RUNS/n/'outcome.json'
        if p.exists():
            o=json.loads(p.read_text());ok=all((RUNS/n/x).is_file() and (RUNS/n/x).stat().st_size for x in ['outcome.json','run.mp4','timeline.txt','audit.txt'])
            text+=f"| {n} | {'v001' if '_L_' in n else 'absent'} | {o['commands']} | {o['ended_by']} | {o['operator_exit_code']} | {'all present' if ok else 'CHECK'} |\n"
    text+='''
The L workspace's notes are byte-identical to v001; the N workspace asserts absence.
Both contain only the expected README, robot, obs and optional LESSONS. All launches
recorded zero `stage1_outputs` rows. Caps and crashes have no operator final claim
when only host cleanup called `done`. Earlier `_01` L/N smoke artifacts are retained;
the `_02` runs verify the final export-aware cleanup code.

`python3 astra_learn/checks.py`: six tests passed (drop/support/rearm and scoring;
truncated logs; memory refusal using a disposable SQLite fixture; export completion; lessons lint;
A/B continues through a simulated crash and summarizes all six rows). Python compile
and shell syntax checks passed. A stand-in process with a server-matching command
line also verified launch refusal before output/workspace allocation
(`preflight_validation.log`). `ab.sh --dry-run` verified the six planned commands
without creating `runs/astra_flange/ab` or starting a server. Real video exports were
verified by decoding all 13 frames of each 2560×400 clip through imageio/FFmpeg
(`smoke_validation.json`); standalone ffprobe is unavailable. Smoke audits contain
the exact test command and initial workspace;
no Codex operator session exists for these tests.

The copied audit helper inventories model tools/paths/commands for human review; it
is not an OS security boundary or a semantic proof. The current server's workspace
instruction boundary is unchanged. The runner refuses existing runs/workspaces/logs,
uses a launch lock and bracketed pgrep, checks memory read-only, and waits for the
server's exit after video export. It detects an already logged end before cleanup,
because the socket can remain open during export. No harness physics, profile,
`screw`, or operator-facing README was edited.

## Running lesson versions

v001 is byte-identical to `~/dreamtrial/astra_ws/lessons_from_006/LESSONS.md`.
Its SHA256 is `'''+hashlib.sha256((ROOT/'astra_learn/lessons/v001.md').read_bytes()).hexdigest()+'''`.
The five later transcripts were consolidated sequentially in the requested order.
Each fresh consolidation workspace contains only notes, its single transcript, and
post-hoc ground truth; no operator was run. Full parent/episode/outcome entries are
in `astra_learn/lessons/CHANGELOG.md`; diffs and model logs are retained beside versions.

'''
    for v in range(2,7):
        version=f'v{v:03}';p=ROOT/f'astra_learn/lessons/{version}.md'
        if not p.exists():
            text+=f'- **{version}**: pending consolidation.\n';continue
        errors=validate(p.read_text());checks=json.loads((p.with_name(version+'_checks.json')).read_text())
        final=p.with_name(f'{version}_attempt{checks[-1]["attempt"]}_final.md').read_text().strip()
        text+=f'- **{version}**, parent v{v-1:03}, {len(p.read_text().splitlines())} lines, attempt {checks[-1]["attempt"]}; lint {"PASS" if not errors else errors}. {final}\n'
    diff=sp.run(['diff','-u','ASTRA_OPERATOR_PROMPT_BACKUP.md','ASTRA_OPERATOR_PROMPT_BACKUP_NOLESSONS.md'],cwd=ROOT,capture_output=True,text=True).stdout
    text+='''
Lint enforces <=100 lines, no numeric coordinate tuples/Cartesian assignments/joint
recipes, and explicit general/scene labels. Relative offsets, rates and turn/feed
increments remain allowed. Semantic review complements the regex; the source v001
is preserved verbatim rather than retroactively rewritten.

## Prompt diff

The no-lessons prompt is byte-identical after removing exactly this paragraph and
its following blank line; all other bytes are preserved:

```diff
'''+diff+'''```

## Launch after review

From `/home/gavin/dreamtrial/viscous_sim`:

```bash
nohup setsid astra_learn/ab.sh > runs/astra_flange/ab_launcher.log 2>&1 < /dev/null &
```

Order: `astra_ab_L1, astra_ab_N1, astra_ab_L2, astra_ab_N2, astra_ab_L3, astra_ab_N3`.
All use astra_rov_backup2, seed 20261024, G3 and 240 min operator caps. L uses fixed
v001 (hash checked throughout), N has no notes. No consolidation occurs during A/B.
The new `runs/astra_flange/ab` receives per-episode appended summary rows, progress
log lines and outcomes.json. A crashed run is recorded and the chain continues.
The final per-arm table reports successes, seated, median commands/sim/wall times,
total drops and claim mismatches. Medians include available failed/capped metrics;
missing crash metrics are excluded, with all three runs retained in success denominators.

Usage is documented in `astra_learn/README.md`. The A/B and any real Astra operator
episodes remain deliberately unlaunched for Claude's review. No controlled treatment
effect can be reported until those runs complete. Coordinate lint and claim parsing
have the heuristic limitations described above; no other requested work remains.
'''
    (ROOT/'astra_lessons_ab_codex_report.md').write_text(text)
    (ROOT/'astra_learn/report.md').write_text(text)


if __name__ == '__main__':main()
