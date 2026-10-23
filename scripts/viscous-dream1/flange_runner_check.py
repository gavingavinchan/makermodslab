"""Four full local stub chunks and deterministic replay, with ledger access forbidden."""
import json
from pathlib import Path
import sweep
from flange_scene import HERE,PROMPT

class ForbiddenLedger:
    def __getattribute__(self,name):raise AssertionError('No ledger access allowed: '+name)


def check():
    root=HERE/'runs/flange_v2/dry';folder=root/'B0/flange/ep_00'
    # This is a flange-only disposable local fixture; regenerate against current scene.
    for name in ('log.json','chunks.jsonl','run.mp4','contact_sheet.jpg'):
        p=folder/name
        if p.exists():p.unlink()
    first=sweep.episode('flange',sweep.DEFAULT,0,False,ForbiddenLedger(),dry_root=root,prompt=PROMPT,stub=True)
    assert first['finished'] and first['calls_used']==0 and first['executed_rows']==192
    assert len(first['row_measurements'])==192 and first['stub_calls']==4
    assert len((folder/'chunks.jsonl').read_text().splitlines())==4
    prior=first.copy();prior['finished']=False;sweep.atomic(folder/'log.json',prior)
    second=sweep.episode('flange',sweep.DEFAULT,0,False,ForbiddenLedger(),dry_root=root,prompt=PROMPT,stub=True)
    assert first==second
    assert sweep.episode('flange',sweep.DEFAULT,0,False,ForbiddenLedger(),dry_root=root,prompt=PROMPT,stub=True)==first
    sweep.atomic(HERE/'runs/flange_v2/validation/runner_checks.json',dict(inference_calls=0,ledger_access_forbidden=True,chunks=4,rows=192,deterministic_replay_passed=True,finished_skip_passed=True,files=sorted(p.name for p in folder.iterdir())))
    print('PASS: flange plugs into original runner; 4 x 48 stub rows, exact replay/skip, no ledger access or client')

if __name__=='__main__':check()
