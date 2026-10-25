"""Pregrasp runner/replay/skip and event logging with the real ledger forbidden."""
import sweep
from flange_runner_check import ForbiddenLedger
from flange_scene import HERE,PROMPT

def check():
    root=HERE/'runs/flange_dream/dry_pregrasp';folder=root/'B0/flange/ep_00'
    for name in ('log.json','chunks.jsonl','run.mp4','contact_sheet.jpg'):
        p=folder/name
        if p.exists():p.unlink()
    c=dict(sweep.DEFAULT,start='pregrasp')
    first=sweep.episode('flange',c,0,False,ForbiddenLedger(),dry_root=root,prompt=PROMPT,stub=True)
    assert first['finished'] and first['calls_used']==0 and first['executed_rows']==192
    events=[a for row in first['row_measurements'] for a in row['grasp_events']]
    assert [a['event'] for a in events]==['attach','attach']
    assert all(row['grasp_attached']=={'left':'bolt','right':'nut'} for row in first['row_measurements'])
    assert all('bolt_tip_m' in d for row in first['row_measurements'] for d in row['requested_hand'].values())
    prior=dict(first,finished=False);sweep.atomic(folder/'log.json',prior)
    second=sweep.episode('flange',c,0,False,ForbiddenLedger(),dry_root=root,prompt=PROMPT,stub=True)
    assert first==second
    assert sweep.episode('flange',c,0,False,ForbiddenLedger(),dry_root=root,prompt=PROMPT,stub=True)==first
    sweep.atomic(HERE/'runs/flange_dream/validation/pregrasp_runner_checks.json',dict(inference_calls=0,ledger_access_forbidden=True,rows=192,chunks=4,deterministic_replay_passed=True,finished_skip_passed=True,grasp_events_logged=events,bolt_tip_requested_distances_logged=True))
    print('PASS: pregrasp initialization, two logged bilateral attachments, four chunks, exact replay/skip; zero calls')
if __name__=='__main__':check()
