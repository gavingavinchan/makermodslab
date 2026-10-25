"""Flange start poses, initialized once before settling; never policy-time setters."""
import numpy as np
import mujoco
from flange_scene import HOLE,LENGTH
from flange_ik_check import LocalIK,orientation

def local_ik(e):
    # Pure local least-squares geometry, no client, metadata, JAX or network.
    ik=LocalIK.__new__(LocalIK);ik.env=e
    ik.limits={s:np.array([e.model.joint(f'{s}_joint{i}').range for i in range(1,7)]) for s in e.sides}
    ik.rng=np.random.default_rng(23)
    return ik

def put(e,p,xyz,R):
    j=e.model.joint(p+'_free');a=j.qposadr[0]
    e.data.qpos[a:a+3]=xyz;mujoco.mju_mat2Quat(e.data.qpos[a+3:a+7],R.ravel())
    e.data.qvel[j.dofadr[0]:j.dofadr[0]+6]=0
    mujoco.mj_forward(e.model,e.data)

def setup(e,start='table',seed=20261024):
    if start=='table':return
    rng=np.random.default_rng(seed);ik=local_ik(e)
    records={}
    for side,sg in ([('right',-1)] if start=='inserted' else [('left',1),('right',-1)]):
        axis=np.array([-sg*.75,-np.sqrt(3)/4,-.5]);R=orientation(axis,[0,0,1])
        p=HOLE+[sg*.16,.18,.05]-.038*axis
        if getattr(e,'rig',None)=='yam':
            axis=np.array([-sg,0.,0.]);R=orientation(axis,[0,0,1])
            p=HOLE+[sg*.14,.18,.05]
        # Paired start jitter; 2 mm, comfortably smaller than the 20 mm loose-part sweep.
        jitter=rng.uniform(-.002,.002,3);p+=jitter
        q,res=ik.fit(side,p,R,starts=12)
        tolerance=.003 if getattr(e,'rig',None)=='yam' else .002
        if np.linalg.norm(res[:3])>tolerance or np.linalg.norm(res[3:])/.15>.035:raise ValueError('pregrasp IK failed')
        e.data.qpos[e.qadr[side]]=q;e.data.ctrl[e.aids[side]]=q
        for sign in (-1,1):e.data.joint(f'{side}_finger_{sign}').qpos[0]=.024
        if getattr(e,'rig',None)=='yam':
            for sign in (-1,1):e.data.actuator(f'{side}_finger_{sign}').ctrl[0]=0
        else:e.data.actuator(side+'_gripper_left_joint_drive').ctrl[0]=0
        mujoco.mj_forward(e.model,e.data)
        h=e.data.site(side+'_tcp');H=h.xmat.reshape(3,3).copy()
        # Rotate hex by 30 deg to put parallel flats against jaw opening X.
        angle=np.pi/6;C=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
        part='bolt' if side=='left' else 'nut'
        offset=0. if getattr(e,'rig',None)=='yam' else .038 if part=='bolt' else .028
        put(e,part,h.xpos+offset*H[:,2],H@C)
        records[side]=dict(jitter_xyz=jitter.tolist(),q=q.tolist(),target=p.tolist(),R=R.tolist(),position_residual_m=float(np.linalg.norm(res[:3])),angle_residual_rad=float(np.linalg.norm(res[3:])/.15))
    if start=='inserted':
        R=orientation([-1,0,0],[0,1,0])
        put(e,'bolt',HOLE+[.080,0,0],R)
        k=e.model.equality('flange_bolt_fixture').id
        e.model.eq_data[k,3:6]=e.data.body('bolt').xpos
        e.model.eq_data[k,6:10]=e.data.body('bolt').xquat
        e.data.eq_active[k]=1
    e.start_info=dict(start=start,seed=seed,hands=records,bolt_fixture_held=start=='inserted')
