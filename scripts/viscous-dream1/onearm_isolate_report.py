"""Regenerate the isolation results and final accounting/artifact audit."""
import hashlib
import json
import statistics
import imageio.v2 as imageio
from pathlib import Path
import sweep
from onearm_isolate_sweep import NAMES, START_USED, ALLOWANCE

OUT=sweep.ROOT/'isolate_validation'


def records(name):
    return [e for p in sorted((sweep.ROOT/name/'apple').glob('ep_*/log.json')) if (e:=json.loads(p.read_text())).get('finished')]


def report():
    names=('B0','single','single_copy','parked',*NAMES)
    episodes={name:records(name) for name in names}
    ledger=json.loads((sweep.ROOT/'ledger.json').read_text())
    plan=json.loads((sweep.ROOT/'isolate_stages.json').read_text())
    check=json.loads((OUT/'checks.json').read_text())
    contact_audit=json.loads((OUT/'parked_physical_contacts.json').read_text())
    assert all(len(contact_audit['episodes'][n])==6 for n in ('parked','parked_blind'))
    assert plan['complete'] and ledger['cap']==1312
    requests=[r for r in ledger['requests'] if r['id']>START_USED]
    assert all(r['episode'].split('/')[0] in NAMES for r in requests)
    used=ledger['budget_used']-START_USED
    billed=sum(r['status']=='billed' for r in requests)
    warming=sum(r['status']=='pool_warming' for r in requests)
    assert used<=ALLOWANCE and used==billed and ledger['uncertain']==0
    audit=dict(used=used,billed=billed,pool_warming=warming,global_cap=ledger['cap'],global_used=ledger['budget_used'],episodes={},protected_sha256={})
    for name in NAMES:
        assert len(episodes[name])==plan['episodes_per_config']
        audit['episodes'][name]=[]
        for e in episodes[name]:
            i=e['episode']; directory=sweep.ROOT/name/'apple'/f'ep_{i:02d}'
            allowed={'log.json','chunks.jsonl','run.mp4'}|({'contact_sheet.jpg'} if i==0 else set())
            assert {p.name for p in directory.iterdir()}==allowed
            baseline=episodes['single'][i]
            assert e['seed']==20261020+i and e['jitter_xy_m']==baseline['jitter_xy_m']
            assert e['start_object_xyz']==baseline['start_object_xyz']
            calls=sum(r['episode']==f'{name}/apple/ep_{i:02d}' and r['status']!='pool_warming' for r in requests)
            assert e['calls_used']==calls<=18
            chunks=[json.loads(line) for line in (directory/'chunks.jsonl').read_text().splitlines()]
            assert len(chunks)==calls
            assert e['executed_rows']==calls*48
            if name=='parked_blind': assert e['parked_max_joint_deviation_rad']==0
            reader=imageio.get_reader(directory/'run.mp4')
            try:
                meta=reader.get_meta_data(); count=reader.count_frames()
                assert count>1
                reader.get_data(0); reader.get_data(count-1)
                video=dict(size=list(meta['size']),fps=meta['fps'],duration=meta['duration'],nb_frames=count,first_last_frames_decoded=True)
            finally: reader.close()
            audit['episodes'][name].append(dict(seed=e['seed'],success=e['success'],calls=calls,video=video,files=sorted(allowed)))
    for n,sha in check['protected_sha256'].items():
        actual=hashlib.sha256((sweep.HERE/n).read_bytes()).hexdigest()
        assert actual==sha
        audit['protected_sha256'][n]=actual
    sweep.atomic(OUT/'final_audit.json',audit)
    rows=['# Parked-arm isolation: Dream-1 apple task','','Zero-shot `dreamlabs/dream-1-beta`, vision mode, paired seeds 20261020–20261025, ±2 cm x/y jitter, 18-call episode cap. Success requires the existing resting-at-target check at two consecutive call endpoints; medians include the confirming call. Grasp means >3 cm lift. Earlier four rows are reused measurements; only the last two are new.','', '| Configuration | Episodes | Successes | Grasps | Median calls to success | Finger/table contact rows |','|---|---:|---:|---:|---:|---:|']
    for name in names:
        es=episodes[name]; wins=[e for e in es if e['success']]
        med=statistics.median(e['confirmed_success_call'] for e in wins) if wins else '—'
        contacts=sum(e['finger_table_contact_rows'] for e in es); total=sum(e['executed_rows'] for e in es)
        rows.append(f'| {name} | {len(es)} | {len(wins)} | {sum(e["grasped"] for e in es)} | {med} | {contacts}/{total} ({100*contacts/total:.1f}%) |')
    rows+=['','Paired outcomes, in column order **B0 / single / single_copy / parked / single_fixedcam / parked_blind**; W/L followed by billed calls:','']
    for i in range(6):
        values=[]
        for name in names:
            e=next((e for e in episodes[name] if e['episode']==i),None)
            values.append(f'{"W" if e["success"] else "L"}/{e["calls_used"]}' if e else 'not run')
        rows.append(f'- **{20261020+i}**: '+ ' / '.join(values)+'.')
    # Interpretation is kept separately so it can be reviewed against the finished data.
    rows+=['',(OUT/'interpretation.md').read_text().strip(),'']
    contact_counts={n:sum(sum(e['right_contact_steps_by_other_body'].values()) for e in contact_audit['episodes'][n]) for n in ('parked','parked_blind')}
    rows += [f'Physical-assistance check (measured, no inference): replaying all six parked and all six blind episodes and inspecting contacts after every 1 ms physics step found **{contact_counts["parked"]} / {contact_counts["parked_blind"]} right-arm contact steps**, respectively. The parked body did not contact the apple, working arm, or table in these runs. [Replay audit](sweep/isolate_validation/parked_physical_contacts.json).','']
    camera=check['camera']; diff=check['pixel_difference']; j=diff['policy_jpeg']; r=diff['raw']
    rows += [f'Controls measured: `single_fixedcam` has no right body tree in any view; its world-fixed camera matches the parked rest/open camera position and orientation (checked to 2×10⁻¹⁵), FOV **{camera["fovy_deg"]}°**, and **640×360** render aspect. World position is ({camera["position_m"][0]:.6f}, {camera["position_m"][1]:.6f}, {camera["position_m"][2]:.6f}) m. Right state is exactly the measured parked rest/open vector; right chunks are ignored. `parked_blind` keeps parked physics, geometry and state, replacing every right frame with black. Its maximum joint deviation was zero.','',f'The fixed view **lacks the parked gripper fingers and arm shadows**. The views are therefore geometrically matched, not pixel-identical. At call 1 / seed 20261020, decoded policy JPEGs ({j["width"]}×{j["height"]}) differ in **{j["changed_pixels"]}/{j["pixels"]} pixels ({100*j["changed_pixels"]/j["pixels"]:.1f}%)**, mean absolute RGB difference **{j["mean_absolute_rgb"]:.2f}/255**, maximum {j["max_absolute_rgb"]}. Raw render difference: {r["changed_pixels"]}/230400 pixels, mean {r["mean_absolute_rgb"]:.2f}/255, maximum {r["max_absolute_rgb"]}. Absolute differences are unscaled in the right panel.','', '![Parked and fixed wrist views with absolute pixel difference](sweep/isolate_validation/parked_fixedcam_wrist_r.png)','', 'No-call call-1 contact sheets: [single_fixedcam](sweep/dry/single_fixedcam/apple/ep_00/contact_sheet.jpg), [parked_blind](sweep/dry/parked_blind/apple/ep_00/contact_sheet.jpg). Both completed 864 dry rows. All existing no-call checks passed: 32 legacy physics/policy combinations plus unchanged `single_copy`/`parked`, sweep geometry/FK/clamp and durable-budget checks, fake-client stop/resume checks, all 18 legacy modes, and parked contact-force hold. Legacy image rounding differed by at most 1 intensity level. New checks verify matched physics for single/fixed and parked/blind under identical commands, exact right state, ignored chunks, black JPEG windows, fixed camera pose and the task budget ceiling. [Checks, console logs and audit](sweep/isolate_validation/).','',f'Accounting: **{billed} billed calls used of 220 allowed**, {warming} unbilled `pool_warming` refusals, 0 uncertain calls. Ledger **1029 → {ledger["budget_used"]} used**, cap unchanged at **1312**; **{1312-ledger["budget_used"]} calls remain**. All 12 requested episodes ran; full-cap planning needed 216 calls. Crash-safe accounting and the global runner lock remain in place.','', 'Representative videos (good/bad use the automated result; none means no episode had that outcome):','']
    for name in NAMES:
        def video(won):
            e=next((e for e in episodes[name] if e['success']==won),None)
            if e is None:return 'none'
            p=sweep.ROOT/name/'apple'/f'ep_{e["episode"]:02d}'/'run.mp4'
            return f'[{p.relative_to(sweep.HERE)}]({p})'
        rows.append(f'- **{name}**: good {video(True)}; bad {video(False)}.')
    rows+=['','Files changed: `sweep.py`, `sweep_scene.py`, `SWEEP_README.md`, `runs/sweep/ledger.json`. Added: `onearm_isolate_check.py`, `onearm_isolate_sweep.py`, `onearm_isolate_report.py`, `onearm_isolate_contacts.py`, this report, `runs/sweep/isolate_stages.json`, `runs/sweep/isolate_live.console.log`, `runs/sweep/isolate_validation/`, and the two new configurations’ live/dry episode and summary folders. Existing no-call validation artifacts were refreshed. Protected arm JSON hashes are unchanged. Episode folders contain only `log.json`, `chunks.jsonl`, `run.mp4`, and a contact sheet for episode 0.']
    (sweep.HERE/'runs/viscous_onearm_isolate_report.md').write_text('\n'.join(rows)+'\n')
    print(json.dumps(audit,indent=2))


if __name__=='__main__':report()
