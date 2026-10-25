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
    for side,sg in [('left',1),('right',-1)]:
        axis=np.array([-sg*.75,-np.sqrt(3)/4,-.5]);R=orientation(axis,[0,0,1])
        p=HOLE+[sg*.16,.18,.05]-.038*axis
        # Paired start jitter; 2 mm, comfortably smaller than the 20 mm loose-part sweep.
        jitter=rng.uniform(-.002,.002,3);p+=jitter
        q,res=ik.fit(side,p,R,starts=12)
        if np.linalg.norm(res[:3])>.002 or np.linalg.norm(res[3:])/.15>.035:raise ValueError('pregrasp IK failed')
        e.data.qpos[e.qadr[side]]=q;e.data.ctrl[e.aids[side]]=q
        for sign in (-1,1):e.data.joint(f'{side}_finger_{sign}').qpos[0]=.024
        e.data.actuator(side+'_gripper_left_joint_drive').ctrl[0]=0
        mujoco.mj_forward(e.model,e.data)
        h=e.data.site(side+'_tcp');H=h.xmat.reshape(3,3).copy()
        # Rotate hex by 30 deg to put parallel flats against jaw opening X.
        angle=np.pi/6;C=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
        part='bolt' if side=='left' else 'nut'
        put(e,part,h.xpos+(.038 if part=='bolt' else .028)*H[:,2],H@C)
        records[side]=dict(jitter_xyz=jitter.tolist(),q=q.tolist(),target=p.tolist(),R=R.tolist())
    e.start_info=dict(start=start,seed=seed,hands=records)
