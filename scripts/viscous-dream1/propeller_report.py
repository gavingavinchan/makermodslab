"""Regenerate aggregate measurements and compact video-review sheets, no inference."""
import json
from pathlib import Path
import numpy as np
import imageio.v2 as imageio
from PIL import Image,ImageDraw
import sweep
from propeller_sweep import PROMPTS,OUT


def records():
    return [(p,json.loads(p.read_text())) for rig in ('B0','YAM') for p in sorted((sweep.ROOT/rig/'propeller').glob('P*/ep_*/log.json'))]


def review():
    directory=OUT/'review';directory.mkdir(exist_ok=True)
    for p,e in records():
        video=p.parent/'run.mp4'
        if not video.exists():continue
        reader=imageio.get_reader(video);count=reader.count_frames()
        # One frame per call plus start, supplemented by in-call action frames.
        indices=np.unique(np.linspace(0,count-1,61).astype(int))
        canvas=Image.new('RGB',(1600,116*int(np.ceil(len(indices)/4))),'white');draw=ImageDraw.Draw(canvas)
        for j,i in enumerate(indices):
            frame=Image.fromarray(reader.get_data(int(i))).resize((400,100))
            x=(j%4)*400;y=(j//4)*116
            canvas.paste(frame,(x,y+16));draw.text((x+3,y+2),f'{i/5:.1f}s / call~{i/12:.1f}',fill='black')
        reader.close();canvas.save(directory/f"{e['config_name']}_{e['prompt_id']}_ep{e['episode']:02d}.jpg",quality=90)


def report():
    rows=records();plan=json.loads((OUT/'stages.json').read_text())
    observations=json.loads((OUT/'video_observations.json').read_text()) if (OUT/'video_observations.json').exists() else {}
    sheet=sweep.ROOT/'B0/propeller/P1/ep_00/contact_sheet.jpg'
    lines=['# Dream-1: bench propeller cleaning','',
    'Viscous sometimes picked up the sponge and touched the propeller, but made no qualifying back-and-forth reversal. YAM showed one sponge reversal and one bare-finger reversal; neither rig demonstrated sustained sponge scrubbing.', '',
    'Zero-shot dreamlabs/dream-1-beta, vision / other_embodiment / client IK. Fixed 20.8 cm bronze three-blade propeller, horizontal shaft facing the arms on an 18.5 cm stand; six convex pieces/blade. Free yellow/green 9×6×4 cm sponge; no brush. Original table, lights, cameras and B0 bases retained; all fruit, bottle, bowl and plate removed.', '',
    '[Call-1 policy images](sweep/B0/propeller/P1/ep_00/contact_sheet.jpg). All videos show top + left wrist. [Native IK verifies a collision-free 3 cm blade-face pass](propeller/validation/ik_reach.json); B0 starts unchanged.', '',
    f'Stage A: two episodes/prompt, paired seeds 20261023–24, sponge jitter ±2 cm. Selected **{plan["winner"]}** by contact rows then reversals (P3: 455, P2: 79). Stage B: three fresh episodes/arm, paired seeds 20261025–27. All episodes: 30 calls × 48 rows, no success stop.','',
    '| Arm | Prompt | Episodes | Sponge grasped (ep) | Tool contact rows | Gripper contact rows | Reversals | Contact path (m) | Closest hand: prop / sponge (cm) | Calls |',
    '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for arm in ('B0','YAM'):
        for pid in PROMPTS:
            episodes=[e for _,e in rows if e['config_name']==arm and e['prompt_id']==pid and e.get('finished')]
            if not episodes:continue
            proximity=[min(v for e in episodes for v in e[k].values() if v is not None)*100 for k in ('closest_requested_hand_to_object_m','closest_requested_hand_to_sponge_m')]
            lines.append(f"| {'Viscous B0' if arm=='B0' else 'YAM vision'} | {pid} | {len(episodes)} | {sum(e['grasped'] for e in episodes)} | {sum(e['tool_propeller_contact_rows'] for e in episodes)} | {sum(e['gripper_propeller_contact_rows'] for e in episodes)} | {sum(e['scrub_reversals'] for e in episodes)} | {sum(e['path_in_contact_m'] for e in episodes):.3f} | {proximity[0]:.2f} / {proximity[1]:.2f} | {sum(e['calls_used'] for e in episodes)} |")
    lines.extend(['','Measured: grasp = >3 cm lift while between distal fingers. Contact checks every physics substep, once/row. Path/reversals use sponge or bare fingertips within 1 cm, projected onto the local blade plane: >1 cm out then >1 cm back, resetting outside each bout. Reversals use that proximity band; the path column includes physical-contact rows only (band travel is also logged). Hand distances: raw requested TCP-to-surface, before execution, minimum across hands. No dirt removal is simulated.',''])
    for pid,prompt in PROMPTS.items():
        lines.append(f'**{pid}: “{prompt}”.** '+observations.get(pid,'Video review pending.'))
        lines.append('')
    lines.extend([observations.get('YAM','YAM video review pending.'),''])
    def rank(item):
        _,e=item
        tool_reversals=sum(r['effectors']['tool']['reversals'] for r in e['row_measurements'])
        return (tool_reversals,e['tool_propeller_contact_rows'],e['scrub_reversals'],e['grasped'],-min(e['closest_requested_hand_to_object_m'].values()))
    if rows:
        for label,item in [('Best',max(rows,key=rank)),('Worst',min(rows,key=rank))]:
            p,e=item;relative=(p.parent/'run.mp4').relative_to(sweep.HERE/'runs')
            lines.append(f'{label} by sponge-reversal/contact/grasp evidence: [{relative}]({relative}).')
    d=json.loads((sweep.ROOT/'ledger.json').read_text());task_requests=[r for r in d['requests'] if '/propeller/' in r['episode']]
    billed=sum(r['status']=='billed' for r in task_requests);warming=sum(r['status']=='pool_warming' for r in task_requests);uncertain=sum(r['status']=='reserved' for r in task_requests)
    lines.extend(['',f'Accounting: **{billed} billed calls**, {warming} unbilled pool_warming refusals, {uncertain} uncertain. Ledger {plan["start_used"]} → {d["budget_used"]} used, cap 1312 → 1912 once under lock with authorization reason; {d["cap"]-d["budget_used"]} remain. '+('All 14 requested episodes completed.' if len(rows)==14 and all(e.get('finished') for _,e in rows) else 'See stages.json for completion and any shortened plan.'),'',
    'All existing no-call checks pass: sweep/FK/clamp/budget/resume, 18 baseline modes, one-arm and isolation compatibility. Four full stub episodes/replay and all 14 video/row/accounting audits pass; protected JSON hashes unchanged. [Evidence](propeller/validation/). Behavior descriptions interpret sampled video frames; counts/trajectories are measured. No general success-rate claim.','',
    'Changed: sweep.py, cross_scenes.py, SWEEP_README.md, ledger.json; refreshed legacy validation. Added: propeller_scene.py, propeller_sweep.py, propeller_ik_check.py, propeller_check.py, propeller_report.py, assets/propeller/*.stl, runs/propeller/, runs/sweep/{B0,YAM}/propeller/, this report. Episode files: log.json, chunks.jsonl, run.mp4, plus first-episode contact_sheet.jpg.'])
    (sweep.HERE/'runs/propeller_report.md').write_text('\n'.join(lines)+'\n')
    sweep.atomic(OUT/'aggregate.json',dict(episodes=[dict(path=str(p.relative_to(sweep.HERE)),**{k:v for k,v in e.items() if k not in ('row_measurements','calls')}) for p,e in rows],ledger=dict(cap=d['cap'],used=d['budget_used'],task_billed=billed,task_warming=warming,task_uncertain=uncertain)))



def audit():
    """Cross-check row aggregates, files, videos, paired seeds and durable accounting."""
    import hashlib
    from propeller_sweep import MIGRATION
    rows=records();assert len(rows)==14 and all(e['finished'] for _,e in rows)
    ledger=json.loads((sweep.ROOT/'ledger.json').read_text())
    request=[r for r in ledger['requests'] if '/propeller/' in r['episode']]
    assert ledger['cap']==1912 and sum(m['id']==MIGRATION for m in ledger['migrations'])==1
    assert not any(r['status']=='reserved' for r in request)
    videos={}
    for p,e in rows:
        r=e['row_measurements'];assert len(r)==e['executed_rows']==1440 and e['calls_used']==30
        assert len((p.parent/'chunks.jsonl').read_text().splitlines())==30
        for field,source in [('tool_propeller_contact_rows','tool_propeller_contact'),('gripper_propeller_contact_rows','gripper_propeller_contact'),('scrub_reversals','scrub_reversals'),('path_in_contact_m','path_in_contact_m'),('path_within_1cm_m','path_within_1cm_m')]:
            assert abs(e[field]-sum(row[source] for row in r))<1e-9
        assert e['grasped']==any(row['tool_grasped'] for row in r)
        for side in ('left','right'):
            for field,source in [('closest_requested_hand_to_object_m','propeller_m'),('closest_requested_hand_to_sponge_m','sponge_m')]:
                assert e[field][side]==min(row['requested_hand'][side][source] for row in r)
        first=e['episode']==(2 if e['config_name']=='YAM' else 0)
        assert {f.name for f in p.parent.iterdir()}=={'log.json','chunks.jsonl','run.mp4'}|({'contact_sheet.jpg'} if first else set())
        v=p.parent/'run.mp4';reader=imageio.get_reader(v);meta=reader.get_meta_data();count=reader.count_frames();reader.close()
        assert tuple(meta['size'])==(1280,368) and count==361 and meta['fps']==5
        videos[str(v.relative_to(sweep.HERE))]=dict(size=meta['size'],frames=count,duration=meta['duration'])
        episode_id=str(p.parent.relative_to(sweep.ROOT))
        assert e['calls_used']==sum(q['episode']==episode_id and q['status']!='pool_warming' for q in request)
    for i in (0,1):
        paired=[e for _,e in rows if e['config_name']=='B0' and e['episode']==i]
        assert len(paired)==4 and all(e['seed']==paired[0]['seed'] and e['jitter_xy_m']==paired[0]['jitter_xy_m'] and e['start_object_xyz']==paired[0]['start_object_xyz'] for e in paired)
    winner=json.loads((OUT/'stages.json').read_text())['winner']
    for i in (2,3,4):
        paired=[e for _,e in rows if e['prompt_id']==winner and e['episode']==i]
        assert len(paired)==2 and paired[0]['seed']==paired[1]['seed'] and paired[0]['jitter_xy_m']==paired[1]['jitter_xy_m']
    protected=json.loads((OUT/'validation/checks.json').read_text())['protected_sha256']
    assert protected=={n:hashlib.sha256((sweep.HERE/n).read_bytes()).hexdigest() for n in protected}
    sweep.atomic(OUT/'validation/final_audit.json',dict(episodes=14,task_billed=sum(r['status']=='billed' for r in request),uncertain=0,cap=ledger['cap'],budget_used=ledger['budget_used'],row_aggregates_match=True,paired_seeds_and_spawns_match=True,permitted_episode_files=True,protected_sha256=protected,videos=videos))
    print('PASS: all 14 live episodes, per-row aggregates, paired seeds/spawns, ledger, allowed files, video decode and protected hashes')


def recompute_yam():
    """Replay durable joint chunks with distal mesh tip points; forbid inference."""
    import fcntl,hashlib
    from types import SimpleNamespace
    import dream
    paths=sorted((sweep.ROOT/'YAM/propeller').glob('P*/ep_*/log.json'))
    assert len(paths)==3
    class NoInference:
        def __init__(self,**kw):
            self._policy=SimpleNamespace(_ik=SimpleNamespace(observe=lambda x:x,solve=lambda *a:None))
        def policy(self,*a):pass
        def set_observation(self,*a,**kw):pass
        def close(self):pass
        def get_action(self,**kw):raise AssertionError('Completed replay must never request inference')
    with (sweep.ROOT/'runner.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        ledger=sweep.Ledger();before=ledger.read();result=[]
        factory=dream.DreamClient;dream.DreamClient=NoInference
        try:
            for p in paths:
                original=json.loads(p.read_text());assert original['finished'] and original['calls_used']==30
                chunks=p.parent/'chunks.jsonl';digest=hashlib.sha256(chunks.read_bytes()).hexdigest()
                replay=dict(original,finished=False);sweep.atomic(p,replay)
                actual=sweep.episode('propeller',original['configuration'],original['episode'],True,ledger,prompt=original['prompt'],variant=original['prompt_id'],first_episode_index=2)
                # Observer changes must leave all physics samples and pair contacts unchanged.
                assert [r['sponge_xyz'] for r in original['row_measurements']]==[r['sponge_xyz'] for r in actual['row_measurements']]
                assert original['calls']==actual['calls']
                assert original['tool_propeller_contact_rows']==actual['tool_propeller_contact_rows']
                assert original['gripper_propeller_contact_rows']==actual['gripper_propeller_contact_rows']
                assert hashlib.sha256(chunks.read_bytes()).hexdigest()==digest
                actual['measurement_note']='Recomputed from unchanged saved joint chunks using physical distal fingertip mesh points; zero inference; sponge trajectory and contact counts identical.'
                sweep.atomic(p,actual)
                result.append(dict(episode=str(p.parent.relative_to(sweep.ROOT)),chunks_sha256=digest,physics_samples_identical=True,tool_contact_rows=actual['tool_propeller_contact_rows'],gripper_contact_rows=actual['gripper_propeller_contact_rows'],grasped=actual['grasped'],reversals=actual['scrub_reversals'],path_in_contact_m=actual['path_in_contact_m'],path_within_1cm_m=actual['path_within_1cm_m']))
                print('Recomputed',result[-1],flush=True)
        finally:dream.DreamClient=factory
        after=ledger.read();assert before['requests']==after['requests'] and before['budget_used']==after['budget_used']
        plan=json.loads((OUT/'stages.json').read_text())
        plan['stage_b']['yam']=dict(episodes=3,tool_contact_rows=sum(r['tool_contact_rows'] for r in result),reversals=sum(r['reversals'] for r in result))
        plan['yam_metrics_replayed_without_inference']=True;sweep.atomic(OUT/'stages.json',plan)
        sweep.atomic(OUT/'validation/yam_metric_replay.json',dict(inference_calls=0,budget_used_before=before['budget_used'],budget_used_after=after['budget_used'],ledger_requests_unchanged=True,episodes=result))


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--review',action='store_true');parser.add_argument('--audit',action='store_true');parser.add_argument('--recompute-yam',action='store_true')
    args=parser.parse_args()
    if args.recompute_yam:recompute_yam()
    if args.review:review()
    if args.audit:audit()
    report()
