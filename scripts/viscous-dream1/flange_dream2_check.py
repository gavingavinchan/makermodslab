"""No-network flange torque isolation, G3 fixture, two-endpoint stop and replay."""
import json
from types import SimpleNamespace
import numpy as np
import sweep,dream

OUT=sweep.HERE/'runs/flange_dream2/validation'

def check():
    OUT.mkdir(parents=True,exist_ok=True)
    limits={}
    for task in ('apple','box','cube','propeller','flange'):
        e=sweep.SweepEnv(task,sweep.DEFAULT)
        arm={s:e.model.actuator_forcerange[e.aids[s]].tolist() for s in e.sides}
        for a in arm.values():np.testing.assert_array_equal(a,np.tile([-25,25] if task=='flange' else [-5,5],(6,1)))
        jaw={s:e.model.actuator(s+'_gripper_left_joint_drive').forcerange.tolist() for s in e.sides}
        for a in jaw.values():np.testing.assert_array_equal(a,[-160,160] if task=='flange' else [-10,10])
        limits[task]=dict(arm=arm,jaw=jaw);e.close()
    out=OUT/'episode_fixture';ledger=sweep.Ledger(out/'budget',cap=2)
    if ledger.path.exists():ledger.path.unlink()
    folder=out/'B0/flange/stop/ep_00'
    for name in ('log.json','chunks.jsonl','run.mp4','contact_sheet.jpg'):
        p=folder/name
        if p.exists():p.unlink()
    initialize,factory=sweep.initialize,dream.DreamClient;state={}
    def target_scene(task,c,seed):
        result=initialize(task,c,seed);e=result[0];state['env']=e
        assert e.grasps.active=={'right':'nut'}
        np.testing.assert_allclose(e.data.body('bolt').xpos,[.08,-.12451505843608915,.9171645709543638],atol=4e-6)
        assert e.data.eq_active[e.model.equality('flange_bolt_fixture').id]
        # Mock only the success signal; the runner still executes real physics/measurements.
        def signal():e.thread.engaged=True;e.thread.turns=2.;e.thread.advance=.007
        e.thread.step=signal
        return result
    class Fake:
        def __init__(self,**kw):self._policy=SimpleNamespace(_ik=SimpleNamespace(observe=lambda x:x,solve=lambda *a:None))
        def policy(self,c):pass
        def set_observation(self,*a,**kw):pass
        def close(self):pass
        def get_action(self,**kw):
            e=state['env'];q=e.state()
            return dict(per_key={f'action.{s}_eef_xyz':np.repeat(sweep.base_object(e,'bolt',s)[None],48,axis=0) for s in e.sides},
                q_l_chunk=np.repeat(q['left'][None],48,axis=0),q_r_chunk=np.repeat(q['right'][None],48,axis=0))
    try:
        sweep.initialize,dream.DreamClient=target_scene,Fake
        c=dict(sweep.DEFAULT,start='inserted',flange_trial='dream2')
        first=sweep.episode('flange',c,0,True,ledger,dry_root=out,variant='stop')
        assert first['success'] and first['calls_used']==2 and first['confirmed_success_call']==2 and first['executed_rows']==96
        rows=first['row_measurements']
        for s in ('left','right'):
            np.testing.assert_allclose(first['arm_torque_max_abs_Nm'][s],np.max([r['arm_torque_peak_abs_Nm'][s] for r in rows],axis=0),atol=1e-12)
            assert max(first['arm_torque_max_abs_Nm'][s])<=25
            for rating in ('5','14'):
                np.testing.assert_allclose(first['arm_time_above_s'][rating][s],np.sum([r['arm_time_above_s'][rating][s] for r in rows],axis=0),atol=1e-10)
                assert max(first['arm_time_above_s'][rating][s])<=4.8+1e-8
        sweep.atomic(folder/'log.json',dict(first,finished=False))
        second=sweep.episode('flange',c,0,True,ledger,dry_root=out,variant='stop')
        assert first==second
        assert sweep.episode('flange',c,0,True,ledger,dry_root=out,variant='stop')==first
        assert ledger.read()['budget_used']==2
    finally:sweep.initialize,dream.DreamClient=initialize,factory
    sweep.atomic(OUT/'dream2_checks.json',dict(inference_calls=0,force_limits=limits,G3_fixture_passed=True,
        torque_row_episode_aggregation_passed=True,two_endpoint_stop_passed=True,deterministic_replay_passed=True,finished_skip_passed=True))
    print('PASS: force isolation, fixture-held G3, torque aggregation, two-endpoint success and exact replay; fake client only',flush=True)

if __name__=='__main__':check()
