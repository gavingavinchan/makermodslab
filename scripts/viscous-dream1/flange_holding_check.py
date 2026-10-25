"""Flange-only force, contact-gating, pregrasp holding and release, no calls."""
import json
import numpy as np
import mujoco
import sweep
from flange_start import setup
from flange_scene import HERE

def check():
    output=HERE/'runs/flange_dream/validation';output.mkdir(parents=True,exist_ok=True)
    force={}
    for task in ('apple','box','cube','propeller','flange'):
        e=sweep.SweepEnv(task,sweep.DEFAULT)
        force[task]=e.model.actuator('left_gripper_left_joint_drive').forcerange.tolist()
        np.testing.assert_array_equal(force[task],[-160,160] if task=='flange' else [-10,10])
        e.close()
    e=sweep.SweepEnv('flange',sweep.DEFAULT);g=e.grasps
    e.gripper_command={'left':0.,'right':0.};g.step();assert not g.active
    setup(e,'pregrasp');m,d=e.model,e.data
    # Move one opposing jaw well away and inspect the instantaneous contacts.
    saved=d.qpos.copy();d.joint('left_finger_1').qpos[0]=.05
    d.joint('left_finger_-1').qpos[0]=.0228;mujoco.mj_forward(m,d)
    g.step();assert 'left' not in g.active,'single jaw must never attach'
    d.qpos[:]=saved;mujoco.mj_forward(m,d)
    q={s:np.r_[a[:6],0.] for s,a in e.state().items()};maxforce=0.;maxdrift=0.;contact_rows=0
    start={s:d.site(s+'_tcp').xpos.copy() for s in e.sides}
    for _ in range(100):
        e.step(q['left'],q['right'])
        maxforce=max(maxforce,*[float(np.max(np.abs(d.actuator_force[e.aids[s]]))) for s in e.sides])
        for s,p in g.active.items():
            h=d.body(s+'_wrist_roll_link');obj=d.body(p);k=m.equality(f'grasp_{s}_{p}').id
            maxdrift=max(maxdrift,float(np.linalg.norm(h.xmat.reshape(3,3).T@(obj.xpos-h.xpos)-m.eq_data[k,3:6])))
        for c in d.contact:
            names=[m.body(m.geom_bodyid[a]).name for a in (c.geom1,c.geom2)]
            if any(n.startswith(('flange_','pipe_')) for n in names) and any(n.startswith(('left_','right_')) for n in names):contact_rows+=1
    print('HOLD',g.active,maxdrift,contact_rows,maxforce,flush=True)
    assert g.active=={'left':'bolt','right':'nut'} and maxdrift<.002 and contact_rows==0
    assert maxforce<5.,maxforce
    sag={s:float(np.linalg.norm(d.site(s+'_tcp').xpos-start[s])) for s in e.sides}
    # Opening below release threshold retains; opening above releases both.
    e.gripper_command={'left':.6,'right':.6};g.step();assert len(g.active)==2
    e.gripper_command={'left':.8,'right':.8};g.step();assert not g.active
    assert [a['event'] for a in g.events]==['attach','attach','release','release']
    events=g.events
    result=dict(inference_calls=0,forces_N=force,no_contact_no_attach=True,single_jaw_no_attach=True,contact_gated_attach=True,opening_threshold=.65,release_passed=True,hold_seconds=5,pregrasp_clear_of_fixture=True,maximum_arm_force_Nm=maxforce,maximum_weld_translation_residual_m=maxdrift,start_tcp_displacement_m=sag,events=events)
    sweep.atomic(output/'holding_checks.json',result);print(json.dumps(result,indent=1));e.close()
if __name__=='__main__':check()
