"""No-call tests of real clearance holes, fixed fixture and thread manifold."""
import json,hashlib
import numpy as np
import mujoco
import sweep
from flange_scene import HERE, HOLE, LENGTH, PITCH, NUT_HALF


def check():
    e,*_=sweep.initialize('flange',sweep.DEFAULT,20261023);m,d=e.model,e.data
    assert all(m.body(n).jntnum[0]==0 for n in ('flange_-1','flange_1','pipe_-1','pipe_1'))
    original=d.qpos.copy()
    bj=m.joint('bolt_free');qa=bj.qposadr[0]
    d.qpos[qa:qa+3]=HOLE+[.038,0,0]
    mujoco.mju_axisAngle2Quat(d.qpos[qa+3:qa+7],np.array([0.,1,0]),-np.pi/2)
    mujoco.mj_forward(m,d)
    def flange_contacts():
        out=[]
        for c in d.contact:
            bodies=[m.body(m.geom_bodyid[g]).name for g in (c.geom1,c.geom2)]
            if 'bolt' in bodies and any(n.startswith('flange_') for n in bodies):out.append(bodies)
        return out
    assert not flange_contacts(),flange_contacts()
    d.qpos[qa+1]+=.005;mujoco.mj_forward(m,d)
    hits=flange_contacts();assert any('flange_1' in c for c in hits) and any('flange_-1' in c for c in hits)
    d.qpos[qa+1]-=.005;mujoco.mj_forward(m,d)
    b=d.body('bolt');R=b.xmat.reshape(3,3).copy();bp=b.xpos.copy()
    nj=m.joint('nut_free');na=nj.qposadr[0]
    def nut(theta=0,radial=0,axial=0):
        d.qpos[na:na+3]=bp+R@np.array([radial,0,LENGTH+NUT_HALF+axial])
        spin=np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
        mujoco.mju_mat2Quat(d.qpos[na+3:na+7],(R@spin).ravel());mujoco.mj_forward(m,d)
    nut(radial=.004);e.thread.step();assert not e.thread.engaged
    nut(axial=.005);e.thread.step();assert not e.thread.engaged
    nut();e.thread.step();assert e.thread.engaged
    # Apply externally generated spin; projection must couple it to actual advance.
    for theta in np.linspace(0,4*np.pi,161)[1:]:
        spin=np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
        mujoco.mju_mat2Quat(d.qpos[na+3:na+7],(R@spin).ravel());mujoco.mj_forward(m,d);e.thread.step()
        assert e.thread.engaged
    assert abs(e.thread.turns-2)<1e-9 and abs(e.thread.advance-2*PITCH)<1e-9
    actual=R.T@(d.body('nut').xpos-d.body('bolt').xpos)
    assert abs(actual[2]-(LENGTH+NUT_HALF-2*PITCH))<1e-8
    d.qpos[na:na+3]+=R[:,2]*.011;mujoco.mj_forward(m,d);e.thread.step();assert not e.thread.engaged
    np.testing.assert_array_equal(d.qpos[qa:qa+3],bp)
    # Local-only guard happens before ledger, scene or client construction.
    try:sweep.episode('flange',sweep.DEFAULT,0,True,None)
    except ValueError:pass
    else:raise AssertionError('local-only guard absent')
    result=dict(inference_calls=0,fixed_fixture=True,clearance_holes_passed=True,offset_contacts_both_flanges=True,engagement_radial_gate_passed=True,two_turns_advance_m=2*PITCH,pull_release_passed=True,no_bolt_projection=True,live_guard_passed=True)
    sweep.atomic(HERE/'runs/flange/validation/physics_checks.json',result);e.close();print(json.dumps(result,indent=2))

if __name__=='__main__':check()
