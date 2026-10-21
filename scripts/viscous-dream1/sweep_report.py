"""Regenerate the short report and runner documentation from durable sweep records."""
import json
from pathlib import Path
from sweep import ROOT, HERE, summarize, parse_config


def report():
    names = ['B0', 'tip', 'single', 'r24', 'table']
    summaries = {n: summarize(parse_config(n)) for n in names if (ROOT/n/'summary.json').exists()}
    ledger = json.loads((ROOT/'ledger.json').read_text())
    total_episodes = sum(v['episodes'] for s in summaries.values() for v in s['tasks'].values())
    docs = '''# Sweep runner

Run from ~/dreamtrial/viscous_sim with MUJOCO_GL=egl, TMPDIR=$PWD/.cache/tmp, and ~/dreamtrial/.venv/bin/python.

- One configuration: `sweep.py --config tip_single_r24 --task both --episodes 6 --live`.
- Full staged experiment / resume: `sweep.py --stages --live`.
- No-call checks: `sweep.py --check`, `sweep_runner_check.py`, `sweep_baseline_check.py`.
- Regenerate report: `sweep_report.py`.
- Without --live, no inference is requested. Existing yam_baseline.py modes remain unchanged.

Configuration names combine tip, single, r24, table with underscores; B0 is all defaults.

B0 uses both arms, unchanged Maker-style wrist views, the original pad TCP, 48 executed rows, and no region.
`tip` uses viscous_arm_fingertip.json, a copy of the original with only tcp changed. At grip 0, visual_043 and
visual_042 are the closed finger blades. Transform each CAD mesh into wrist_roll_link coordinates, select the
furthest face along approach (100 micrometre tolerance), take each face bounding-box centre, and average the two.
The new point is (0.166308001117, -0.000000000081, -0.027090638862) m, 3.433 cm forward along approach. Axes are
unchanged. MuJoCo's site_sameframe optimization is disabled for the moved diagnostic site so its FK actually moves.
The cameras and physical meshes do not move. Real hardware needs Gavin's dreamcli tcp calibration.

`single` removes the entire right body tree, camera, contacts, equality and actuators. It sends arm-file rest joints
with an open grip for the right state, black right wrist frames, and ignores the right chunk. Left base is unchanged.
`r24` executes the first 24 of 48 rows at 20 Hz before requesting again.
`table` uses documented dream/client_ik.py ik.region.left: world x [-.08,.40], y [-.32,.16], z [.785,1.06] m.
Table z=.76, so the box is 2.5 to 30 cm above it and covers object spawn jitter and targets in front of the arm.
In left base coordinates: lo [.08,-.385,.013], hi [.56,.095,.288] m. The client clips targets inward by its collision
margin and adds a region cost on arm centreline points; its hard fence is off by default. IK still determines pose
reachability. The dry test directly verifies the native solver's target clipping.

Seeds: 20261020 + episode index for apple, 20271020 + index for box. Uniform independent x/y jitter ±2 cm, paired
across configurations. Fresh physics scene per episode. Existing geometry/rest success check must pass at two
consecutive call endpoints; the confirming call is included in calls to success. Grasp means >3 cm centre-height
increase from settled spawn. Contact rows count any finger/table contact once per executed row. Requested-hand
proximity uses raw model positions against the object's position immediately before each row, for executed rows.

Every request reserves a ledger slot, atomically written and fsynced before get_action. Only explicit DA-2000
pool_warming refunds it. Unknown outcomes remain charged conservatively. A global file lock prevents concurrent
runner processes. Chunks are fsynced before execution. On restart, saved joint chunks replay from the seeded scene
with no requests; completed episodes skip. A new client resumes from that recovered scene, resetting seam history
for the next chunk. A finished episode whose video is missing is reconstructed. Only log.json, chunks.jsonl, run.mp4
and a first-episode contact sheet are saved inside episode folders.

Stage selection aggregates successes across tasks in that stage, breaking ties by total calls to confirmed success.
Stage 1 exact tie keeps B0; stage 2 exact tie carries single as requested. Stage 3 compares equal seed prefixes when
shortened and combines knobs only when each improves that score. Stage 4 reuses existing episodes of its selected
configuration, keeping six total per configuration/task; it budgets conservatively for missing episodes at full caps.

Validation artifacts: runs/sweep/validation/checks.json (32 scene/task combinations, four client FK/clamp variants,
ten FK samples each), runner_check.console.log (fake-client two-call stop and interrupted/finished resume),
baseline_check.console.log (all 18 previous modes dry), final_audit.json (counts, caps, files, video checks, hashes).
Validation used zero inference calls. Original viscous_arm.json SHA256:
cbbd055358a98b8ef39eecde33df87db265ed2ebd7b06f8fd46f32674fe0d2b0.
'''
    (HERE/'SWEEP_README.md').write_text(docs)
    lines = ['# VISCOUS Dream-1 sweep', '',
        f"**{ledger['confirmed_billed']} billed calls; {ledger['uncertain']} uncertain; {900-ledger['budget_used']} unused.** {total_episodes} seeded episodes, no fine-tuning. Success requires two consecutive call-end rest checks; grasp means >3 cm lift. Median calls includes the confirming call.", '']
    labels = dict(B0='B0', tip='Fingertip TCP', single='B0 + single arm', r24='B0 + 24 rows', table='B0 + table region')
    for task in ('apple', 'box'):
        lines += [f'## {task.capitalize()}', '', '| Configuration | Episodes | Successes | Grasps | Median calls to success | Finger/table rows |', '|---|---:|---:|---:|---:|---:|']
        for name, s in summaries.items():
            v=s['tasks'][task]
            if not v['episodes']: continue
            lines.append(f"| {labels[name]} | {v['episodes']} | {v['successes']} ({v['successes']/v['episodes']:.0%}) | {v['grasps']} | {v['median_calls_to_success'] if v['median_calls_to_success'] is not None else '—'} | {v['finger_table_contact_rows']}/{v['executed_rows']} ({v['finger_table_contact_rows']/v['executed_rows']:.1%}) |")
        lines.append('')
    lines += ['## Interpretation', '',
        '**Keep B0 for the two-arm apple simulation. No tested configuration makes box placement or the one-arm simulation reliable.** Fingertip TCP produced the only box success (1/6), but reduced apple success to 4/6; one box success is weak evidence of improvement. Single-arm reduced apple to 2/6 and box stayed 0/6. Twenty-four rows gave box 0/4. The region cut box table contact to 2.3% and also eliminated lifts in those trials; it did not improve placement.', '',
        'Stage 1 selected B0 by combined successes (6 versus 5). Stage 2 retained its single-arm results but carried B0 (6 versus 2). Stage 3 used **four paired seeds per option**, because six could exceed the remaining budget; B0, 24 rows and region all scored 0/4 on those seeds. Stage 4 therefore selected the already-tested B0; **no new stage-4 episodes** were run. Its six episodes per task were reused. The remaining 28 calls cannot fund a fresh apple+box pair at full caps (42).', '',
        'The fingertip copy moves only TCP, 3.433 cm forward to the closed CAD tip-face midpoint; axes remain unchanged. [Old/new tool-point picture](sweep/validation/tool_points.png). The table region is world x [−8,40], y [−32,16] cm, from 2.5 to 30 cm above the table; native client target clamp and region cost, with the default hard fence off.', '',
        '## Representative videos', '', 'Good/bad means the automated success result. A missing outcome is marked none.', '', '| Configuration/task | Good video | Bad video |', '|---|---|---|']
    for name,s in summaries.items():
        for task,v in s['tasks'].items():
            if not v['episodes']: continue
            def link(key):
                path=v[key]
                return f'[{Path(path).relative_to(HERE/"runs")}]({path})' if path else 'none'
            lines.append(f"| {name}/{task} | {link('good_video')} | {link('bad_video')} |")
    lines += ['', '## Files and checks', '',
        'Added `sweep.py`, `sweep_scene.py`, `sweep_check.py`, `sweep_runner_check.py`, `sweep_baseline_check.py`, `sweep_report.py`, `viscous_arm_fingertip.json`, `SWEEP_README.md`, this report, and `runs/sweep/` artifacts. Existing source and `viscous_arm.json` unchanged. All 18 previous modes pass dry; sweep geometry, native FK/clamp, early stop, ledger cap and resume checks pass. [Runner details](../SWEEP_README.md); [audit](sweep/validation/final_audit.json).']
    path=HERE/'runs/codex_viscous_sweep_report.md';path.write_text('\n'.join(lines)+'\n');return path

if __name__=='__main__': print(report())
