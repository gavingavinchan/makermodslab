"""Focused no-call metrics, accounting, replay and full stub checks."""
import json
from pathlib import Path
import tempfile
import hashlib
import numpy as np
import mujoco
import sweep
from propeller_scene import Measurements,Reversal,CENTER
from propeller_sweep import authorize,MIGRATION,PROMPTS,OUT


def check_yam_fingertips():
    e,*_=sweep.initialize('propeller',dict(sweep.DEFAULT,rig='yam'),20261023)
    try:
        o=Measurements(e);p=o.finger_points('left').mean(0)
        # The physical pads are near TCP; link frames are about 8 cm behind it.
        assert np.linalg.norm(p-e.data.site('left_tcp').xpos)<.02
        e.data.joint('sponge_free').qpos[:3]=p;mujoco.mj_forward(e.model,e.data)
        r=o.row(0,0,{})
        assert r['tool_grasped'] and r['between_fingers']['left']
        assert r['path_in_contact_m']<=r['path_within_1cm_m']
    finally:e.close()
    print('PASS: YAM distal mesh pads detect a lifted between-finger sponge; zero inference',flush=True)


def check():
    OUT.mkdir(parents=True,exist_ok=True)
    protected={n:hashlib.sha256((sweep.HERE/n).read_bytes()).hexdigest() for n in ('viscous_arm.json','viscous_arm_fingertip.json')}
    r=Reversal();total=0;path=0
    for x in [0,.006,.013,.020,.016,.009,.004,.010,.018,.025]:
        n,p=r.add(np.array([x,0.,0.]),np.array([0.,0.,1.]));total+=n;path+=p
    assert total==2 and abs(path-.057)<1e-8,(total,path)
    r.reset();assert r.add(np.zeros(3),np.array([0.,0.,1.]))==(0,0.)
    # Pure normal motion contributes neither path nor reversal.
    r=Reversal()
    assert sum(r.add(np.array([0.,0.,z]),np.array([0.,0.,1.]))[1] for z in [0,.02,0,.02])==0
    with tempfile.TemporaryDirectory(dir=sweep.HERE/'.cache/tmp') as d:
        fixture=Path(d);sweep.atomic(fixture/'ledger.json',dict(cap=1312,budget_used=1206,requests=[]))
        ledger=authorize(fixture);authorize(fixture)
        data=json.loads(ledger.path.read_text());assert data['cap']==1912 and sum(m['id']==MIGRATION for m in data['migrations'])==1
    e,*_=sweep.initialize('propeller',sweep.DEFAULT,20261023)
    try:
        o=Measurements(e);before=e.data.body('propeller').xpos.copy()
        assert e.model.body('propeller').jntnum[0]==0 and len(o.prop)==20
        assert e.model.body('sponge').jntnum[0]==1
        assert len(o.fingers['left'])>0 and len(o.fingers['right'])>0
        # Set the sponge at the measured finger midpoint, lifted >3 cm.
        p=o.finger_points('left').mean(0)
        e.data.joint('sponge_free').qpos[:3]=p;mujoco.mj_forward(e.model,e.data)
        sample=o.row(0,0,{});assert sample['tool_grasped'] and sample['between_fingers']['left']
        # Raised but away from both jaws is not a grasp.
        e.data.joint('sponge_free').qpos[:3]=[.4,-.3,1.1];mujoco.mj_forward(e.model,e.data)
        assert not o.row(0,1,{})['tool_grasped']
        # Real geom-pair contact; repeated substep sampling counts only once this row.
        e.data.joint('sponge_free').qpos[:3]=CENTER+[.065,.01,.02];mujoco.mj_forward(e.model,e.data)
        for _ in range(4):o.contacts()
        sample=o.row(0,2,{});assert sample['tool_propeller_contact']
        assert not o.row(0,3,{})['tool_propeller_contact']
        for _ in range(4):e.step(*e.state().values())
        np.testing.assert_array_equal(before,e.data.body('propeller').xpos)
    finally:e.close()
    check_yam_fingertips()
    # Full 30-reply dry episode for each prompt via the runner, no client constructed.
    import propeller_sweep
    propeller_sweep.run(False)
    for pid in PROMPTS:
        folder=OUT/'dry/B0/propeller'/pid/'ep_00';log=json.loads((folder/'log.json').read_text())
        assert log['executed_rows']==1440 and len(log['row_measurements'])==1440 and log['calls_used']==0
        assert len((folder/'chunks.jsonl').read_text().splitlines())==30
        assert {p.name for p in folder.iterdir()}=={'log.json','chunks.jsonl','run.mp4','contact_sheet.jpg'}
        # Finished rerun skips; a replay reconstructs the same measurements and video without calls.
        original=log.copy();log['finished']=False;sweep.atomic(folder/'log.json',log)
        actual=sweep.episode('propeller',sweep.DEFAULT,0,False,sweep.Ledger(),dry_root=OUT/'dry',prompt=PROMPTS[pid],variant=pid,stub=True)
        assert actual==original
    assert protected=={n:hashlib.sha256((sweep.HERE/n).read_bytes()).hexdigest() for n in protected}
    sweep.atomic(OUT/'validation/checks.json',dict(inference_calls=0,protected_sha256=protected,reversal_hysteresis_passed=True,normal_motion_excluded=True,grasp_between_fingers_passed=True,contact_once_per_row_passed=True,fixed_propeller_passed=True,locked_idempotent_cap_migration_passed=True,full_stub_prompts=4,rows_per_prompt=1440,replay_passed=True))
    print('PASS: propeller metrics, fixed geometry, grasp, contact, reversal, ledger migration, four full stub episodes and replay; zero calls',flush=True)

if __name__=='__main__':check()
