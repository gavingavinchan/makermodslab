"""Replay one real episode per rig into disposable artifacts, with zero inference."""
import json,shutil
from pathlib import Path
import numpy as np
import sweep
from flange_runner_check import ForbiddenLedger

OUT=sweep.HERE/'runs/flange_dream2'
def run(rig_filter='both'):
    record=OUT/'validation/real_replay_checks.json'
    results=json.loads(record.read_text()) if record.exists() else []
    for rig,name in [('viscous','B0'),('yam','YAM')]:
        if rig_filter not in ('both',rig):continue
        sources=sorted((OUT/'episodes'/name/'flange/G3').glob('ep_*/log.json'))
        candidates=[p for p in sources if json.loads(p.read_text()).get('finished')]
        if not candidates:continue
        source=candidates[0];original=json.loads(source.read_text());index=original['episode']
        root=OUT/'validation/replay';folder=root/name/'flange/G3'/f'ep_{index:02d}'
        folder.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(source.parent/'chunks.jsonl',folder/'chunks.jsonl')
        for n in ('log.json','run.mp4'):
            if (folder/n).exists():(folder/n).unlink()
        replay=sweep.episode('flange',original['configuration'],index,False,ForbiddenLedger(),dry_root=root,
            prompt=original['prompt'],variant='G3',stub=True,first_episode_index=2 if rig=='yam' else 0)
        assert original['row_measurements']==replay['row_measurements']
        for key in ('arm_torque_max_abs_Nm','arm_time_above_s','max_bolt_inserted_depth_m','max_nut_turns','nut_engaged'):
            assert original[key]==replay[key]
        results=[r for r in results if r['rig']!=rig]
        results.append(dict(rig=rig,episode=index,rows=original['executed_rows'],inference_calls=0,exact_row_and_torque_replay=True))
    sweep.atomic(record,results);print(results)
if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--rig',choices=['both','viscous','yam'],default='both');run(p.parse_args().rig)
