"""Local native client-IK checks. No DreamClient or inference is constructed."""
import json
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from dream.client_ik import ClientIK
from dream.models import Model
import sweep
from flange_scene import HOLE, HERE


def orientation(approach,jaw):
    z=np.asarray(approach,float);z/=np.linalg.norm(z)
    x=np.asarray(jaw,float);x-=z*np.dot(x,z);x/=np.linalg.norm(x)
    return np.column_stack([x,np.cross(z,x),z])


class LocalIK:
    def __init__(self,env):
        self.env=env
        model=Model(sweep.y.BETA_MODEL,'public',sweep.y.BETA_MODEL,'yam',True,None)
        self.ik=ClientIK({'arm':str(HERE/'viscous_arm.json'),'head':'eef'},20,sweep.y.YAM_EMBODIMENT,model)
        assert self.ik.traj_ready.wait(60)
        self.solver=self.ik._solver
        self.limits={s:np.array([env.model.joint(f'{s}_joint{i}').range for i in range(1,7)]) for s in env.sides}
        self.rng=np.random.default_rng(23)
    def fit(self,side,point,rot,seed=None,starts=12):
        e=self.env;lim=self.limits[side];saved=e.data.qpos.copy()
        def residual(q):
            e.data.qpos[e.qadr[side]]=q;mujoco.mj_forward(e.model,e.data)
            site=e.data.site(side+'_tcp')
            return np.r_[site.xpos-point,.15*Rotation.from_matrix(rot.T@site.xmat.reshape(3,3)).as_rotvec()]
        initial=sweep.PARK[:6] if seed is None else seed
        fits=[least_squares(residual,np.clip(q,lim[:,0]+1e-7,lim[:,1]-1e-7),bounds=(lim[:,0],lim[:,1]),max_nfev=110) for q in [initial,*self.rng.uniform(lim[:,0],lim[:,1],size=(starts,6))]]
        fit=min(fits,key=lambda f:np.linalg.norm(f.fun));q=fit.x
        e.data.qpos[:]=saved;mujoco.mj_forward(e.model,e.data)
        return q,fit.fun
    def native(self,side,point,rot,seed):
        e=self.env;sol=self.solver;sides=sol.sides
        q0=np.array([e.state()[s][:6] for s in sides]);q0[sides.index(side)]=seed
        targets=np.stack([np.repeat(sol.fk(q0[i:i+1],s),48,axis=0) for i,s in enumerate(sides)])
        b=e.data.body(side+'_base_link');R=b.xmat.reshape(3,3)
        i=sides.index(side);targets[i,:,:3,3]=R.T@(point-b.xpos);targets[i,:,:3,:3]=R.T@rot
        result=sol.solve(targets,q0);q=result.q_solution[i,-1];fk=sol.fk(q[None],side)[0]
        return q,float(np.linalg.norm(fk[:3,3]-targets[i,-1,:3,3])),float(np.degrees(Rotation.from_matrix(fk[:3,:3].T@targets[i,-1,:3,:3]).magnitude()))
    def contacts(self,side,q,grip=1.):
        e=self.env;saved=e.data.qpos.copy();e.data.qpos[e.qadr[side]]=q
        for k in (-1,1):e.data.joint(f'{side}_finger_{k}').qpos[0]=grip*sweep.y.OPEN if hasattr(sweep.y,'OPEN') else grip*.0524125
        mujoco.mj_forward(e.model,e.data)
        contacts=[]
        for c in e.data.contact:
            bodies=[e.model.body(e.model.geom_bodyid[g]).name for g in (c.geom1,c.geom2)]
            if any(n.startswith(side+'_') for n in bodies):contacts.append(dict(bodies=bodies,dist_m=float(c.dist)))
        e.data.qpos[:]=saved;mujoco.mj_forward(e.model,e.data)
        return contacts


def check():
    e,*_=sweep.initialize('flange',sweep.DEFAULT,20261023);ik=LocalIK(e)
    records=[]
    # TCP is the original pad point; the distal fingertip is 34.3 mm beyond it.
    down=orientation([0,0,-1],[0,1,0])
    a=np.deg2rad(15);approach=np.array([0,-np.cos(a),-np.sin(a)])
    oblique=orientation(approach,[0,np.sin(a),-np.cos(a)])
    poses=[('bolt_pick','left',e.data.body('bolt').xpos+[.010,0,.030],down),
           ('hole_entrance_bolt_axis_minus_x','left',HOLE+[.170,0,0]-.028*approach,oblique),
           ('bolt_through_both_grip_axis_minus_x','left',HOLE+[.060,0,0]-.028*approach,oblique),
           ('nut_pick','right',e.data.body('nut').xpos+[0,0,.034],down),
           ('far_side_nut','right',HOLE+[-.132,0,0],orientation([1,0,0],[0,-2**-.5,2**-.5]))]
    for label,side,p,R in poses:
        q,res=ik.fit(side,p,R,starts=15);q,pe,ae=ik.native(side,p,R,q)
        rec=dict(name=label,side=side,tool_target_world_m=p.tolist(),approach_world=R[:,2].tolist(),position_error_m=pe,orientation_error_deg=ae,joints=q.tolist(),contacts=ik.contacts(side,q,grip=.45))
        if 'axis_minus_x' in label:
            saved=e.data.qpos.copy();e.data.qpos[e.qadr[side]]=q;mujoco.mj_forward(e.model,e.data)
            hand=e.data.site(side+'_tcp');H=hand.xmat.reshape(3,3).copy();axis=-H[:,1]
            bolt=e.model.joint('bolt_free');ba=bolt.qposadr[0]
            bodyrot=np.column_stack([H[:,0],H[:,2],axis])
            e.data.qpos[ba:ba+3]=hand.xpos+.028*H[:,2]+.010*axis
            mujoco.mju_mat2Quat(e.data.qpos[ba+3:ba+7],bodyrot.ravel());mujoco.mj_forward(e.model,e.data)
            collisions=[]
            for c in e.data.contact:
                names=[e.model.body(e.model.geom_bodyid[g]).name for g in (c.geom1,c.geom2)]
                if 'bolt' in names and any(n.startswith('flange_') for n in names):collisions.append(names)
            rec.update(held_bolt_axis_world=axis.tolist(),bolt_axis_error_deg=float(np.degrees(np.arccos(np.clip(np.dot(axis,[-1,0,0]),-1,1)))),held_bolt_fixture_contacts=collisions)
            e.data.qpos[:]=saved;mujoco.mj_forward(e.model,e.data)
        records.append(rec);print(label,pe,ae,len(rec['contacts']),flush=True)
    result=dict(inference_calls=0,pad_tcp=True,maximum_jaw_opening_m=2*.0524125,hex_across_flats_m=.046,targets=records,pose_reach_passed=all(r['position_error_m']<.003 and r['orientation_error_deg']<3 and not r.get('held_bolt_fixture_contacts',[]) and not [c for c in r['contacts'] if not any(n in ('bolt','nut') for n in c['bodies'])] for r in records))
    sweep.atomic(HERE/'runs/flange/validation/ik_reach.json',result);e.close()
    print(json.dumps(result,indent=1),flush=True)

if __name__=='__main__':check()
