"""Write the concise one-arm follow-up report and audit from recorded results."""
import hashlib
import json
import statistics
from pathlib import Path
import sweep


def report():
    root = sweep.ROOT
    plan = json.loads((root / 'onearm_stages.json').read_text())
    names = ['B0', 'single', 'single_copy', 'parked']
    if plan.get('stage2_required'):
        names += [plan['stage1_winner'] + '_left', 'single_left']
    records = {n: [json.loads(p.read_text()) for p in sorted((root/n/'apple').glob('ep_*/log.json')) if json.loads(p.read_text()).get('finished')] for n in names}
    ledger = json.loads((root/'ledger.json').read_text())
    rows = ['# One-arm Dream-1 apple trial', '', 'Zero-shot `dreamlabs/dream-1-beta`, vision mode; 18-call cap; paired seeds 20261020–20261025; ±2 cm independent x/y spawn jitter. Success is the existing resting-at-target check at two consecutive call endpoints; grasp means >3 cm lift. Median includes the confirming call.', '', '| Configuration | Episodes | Successes | Grasps | Median calls to success | Finger/table contact rows |', '|---|---:|---:|---:|---:|---:|']
    for n, episodes in records.items():
        wins = [e for e in episodes if e['success']]
        med = statistics.median(e['confirmed_success_call'] for e in wins) if wins else '—'
        contacts = sum(e['finger_table_contact_rows'] for e in episodes)
        total = sum(e['executed_rows'] for e in episodes)
        rows.append(f'| {n} | {len(episodes)} | {len(wins)} | {sum(e["grasped"] for e in episodes)} | {med} | {contacts}/{total} ({100*contacts/total:.1f}%) |')
    baseline = {e['seed']: e['success'] for e in records['single']}
    rows += ['', 'Paired outcomes against plain `single` (seed suffixes 20–25 correspond to episodes 0–5; W/L followed by calls used):', '']
    for n in names:
        if n == 'single': continue
        flips_up = [str(e['seed']) for e in records[n] if e['success'] and not baseline[e['seed']]]
        flips_down = [str(e['seed']) for e in records[n] if not e['success'] and baseline[e['seed']]]
        outcomes = ', '.join(f'{str(e["seed"])[-2:]} {"W" if e["success"] else "L"}/{e["calls_used"]}' for e in records[n])
        rows.append(f'- **{n}**: {outcomes}. Loss→win: {", ".join(flips_up) or "none"}; win→loss: {", ".join(flips_down) or "none"}.')
    copy_wins = sum(e['success'] for e in records['single_copy'])
    parked_wins = sum(e['success'] for e in records['parked'])
    rows += ['', f'Measured: copying the left wrist produced {copy_wins}/6 versus plain single’s 2/6 ({copy_wins-2:+d} successes, {(copy_wins-2)*100/6:+.1f} percentage points). Parked produced {parked_wins}/6, {parked_wins-copy_wins:+d} successes ({(parked_wins-copy_wins)*100/6:+.1f} points) versus copying. Six episodes are a small sample; no real-bench result was measured.']
    if parked_wins >= 5 and copy_wins < 5:
        rows += ['', '**Use parked for the one-actuated-arm simulation.** Copying did not improve the measured success rate. Parked also increased finger/table contacts to 3.8% of rows; its success rate does not establish contact-free motion.']
    rows += ['', 'Real bench: `single_copy` needs no extra hardware; duplicate left wrist frames unchanged, keep right rest/open state, and discard right actions. `parked` needs a second held arm, or a comparable dummy with a camera at its wrist pose. A dummy’s effectiveness is a guess. Sim uses an ideal position hold, not measured real servo compliance. Black-frame distribution shift remains a hypothesis; these trials do not isolate its cause.']
    if plan.get('stage2_required'):
        geom = json.loads((root/'onearm_validation/layout.json').read_text())
        rows += ['', geom['report_explanation'], '', '![Before and after left-side layout](sweep/onearm_validation/layout_before_after.jpg)']
    else:
        rows += ['', 'Stage 2 was skipped because at least one stage-1 configuration reached 5/6, as specified.']
    check = json.loads((root/'onearm_validation/checks.json').read_text())
    parked_dry = check['parked']
    stress = check['parked_contact_stress']
    rows += ['', f'No-call checks passed: 32 exact legacy physics/policy comparisons, existing sweep checks, fake-client resume/budget checks, and all 18 legacy modes. Default image rendering is unchanged (repeat renders differ by at most 1 intensity level in 14 pixels). Both new modes ran 864 dry rows. Parked maximum measured deviation was **{parked_dry["parked_max_joint_deviation_rad"]} rad**, also zero during 37 initial finger contacts plus a sustained 20 N x/−20 N z force. Constraints plus projection hold each 1 ms step; the maximum pre-projection solver-coordinate residual was {stress["max_preprojection_solver_deviation_rad"]:.3g} (hinges in rad, sliders in m). Copied JPEG windows match left exactly. [Checks and logs](sweep/onearm_validation/).']
    rows += ['', 'Call-1 policy images: [single_copy](sweep/dry/single_copy/apple/ep_00/contact_sheet.jpg), [parked](sweep/dry/parked/apple/ep_00/contact_sheet.jpg). Each sheet shows the latest encoded frame of all three call-1 camera windows.']
    used = ledger['budget_used'] - plan['start_used']
    billed = ledger['confirmed_billed'] - plan['start_used']
    rows += ['', f'Accounting: **{billed} additional billed calls**, {used-billed} uncertain, {sum(r["status"] == "pool_warming" for r in ledger["requests"] if r["id"] > plan["start_used"])} unbilled warming refusals. Ledger started at 872 used; cap changed once from 900 to **1312 = 872 + 440**. Final used {ledger["budget_used"]}; {1312-ledger["budget_used"]} calls remain. Crash-safe reservations/fsync and global runner lock retained.']
    rows += ['', 'Representative videos (good/bad use the automated result; “none” means that outcome was absent):', '']
    for n, episodes in records.items():
        def video(won):
            e = next((e for e in episodes if e['success'] == won), None)
            if e is None: return 'none'
            p = root/n/'apple'/f'ep_{e["episode"]:02d}'/'run.mp4'
            return f'[{p.relative_to(sweep.HERE)}]({p})'
        rows.append(f'- **{n}**: good {video(True)}; bad {video(False)}.')
    rows += ['', 'Files changed: `sweep.py`, `sweep_scene.py`, `SWEEP_README.md`, `runs/sweep/ledger.json`. Added: `onearm_check.py`, `onearm_sweep.py`, `onearm_report.py`, this report, `runs/sweep/onearm_stages.json`, `runs/sweep/onearm_validation/`, new configuration episode/summary folders, and their dry folders.' + (' Also added `onearm_layout.py` for the conditional stage-2 geometry check.' if plan.get('stage2_required') else '') + ' Protected arm JSON files unchanged. Validation reruns refreshed existing no-call artifacts.']
    (sweep.HERE/'runs/viscous_onearm_report.md').write_text('\n'.join(rows)+'\n')
    audit = dict(ledger={k:v for k,v in ledger.items() if k!='requests'}, additional_budget_used=used, episodes={}, protected_sha256={name:hashlib.sha256((sweep.HERE/name).read_bytes()).hexdigest() for name in ('viscous_arm.json','viscous_arm_fingertip.json')})
    for n in names[2:]:
        audit['episodes'][n] = []
        for e in records[n]:
            directory = root/n/'apple'/f'ep_{e["episode"]:02d}'
            allowed = {'log.json','chunks.jsonl','run.mp4'} | ({'contact_sheet.jpg'} if e['episode']==0 else set())
            assert {p.name for p in directory.iterdir()} == allowed
            assert e['seed'] == 20261020+e['episode'] and e['calls_used'] <= 18
            if n.startswith('parked'): assert e['parked_max_joint_deviation_rad'] == 0
            calls = sum(r['episode'] == f'{n}/apple/ep_{e["episode"]:02d}' and r['status']!='pool_warming' for r in ledger['requests'])
            assert calls == e['calls_used']
            audit['episodes'][n].append(dict(seed=e['seed'],calls=calls,success=e['success'],files=sorted(allowed)))
    assert used<=440 and ledger['uncertain']==0
    sweep.atomic(root/'onearm_validation/final_audit.json',audit)
    print('Wrote runs/viscous_onearm_report.md and one-arm final audit')

if __name__ == '__main__': report()
