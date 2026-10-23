"""No calls: native Dream client IK reach at a blade-scrubbing orientation."""
import json
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import sweep
from propeller_scene import CENTER,blade_vertices
from dream.client_ik import ClientIK
from dream.models import Model


def check():
    env,*_=sweep.initialize('propeller',sweep.DEFAULT,20261023)
    model=Model(sweep.y.BETA_MODEL,'public',sweep.y.BETA_MODEL,'yam',True,None)
    ik=ClientIK({'arm':str(sweep.HERE/'viscous_arm.json'),'head':'eef'},20,sweep.y.YAM_EMBODIMENT,model)
    assert ik.traj_ready.wait(60)
    solver=ik._solver
    q0=np.array([sweep.PARK[:6] for s in solver.sides])
    targets=np.stack([np.repeat(solver.fk(q0[i:i+1],s),48,axis=0) for i,s in enumerate(solver.sides)])
    base=env.data.body('left_base_link');R=base.xmat.reshape(3,3)
    # Front face of outer-right blade. Approach normal to its local pitched surface.
    v=blade_vertices(0);p=v[7,1,3]+CENTER
    radial=np.array([np.cos(.2),0,np.sin(.2)]);pitch=np.deg2rad(38-21*7/12)
    tangent=np.array([-np.sin(.2)*np.cos(pitch),np.sin(pitch),np.cos(.2)*np.cos(pitch)])
    normal=np.cross(radial,tangent);normal*=np.sign(normal[1])
    approach=-normal; jaw=radial;up=np.cross(approach,jaw)
    rot=np.column_stack([up,-jaw,approach])
    # 4 mm in front of the blade; also solve tangential offsets for a 3 cm scrub pass.
    records=[]
    limits=np.array([env.model.joint(f'left_joint{i}').range for i in range(1,7)])
    rng=np.random.default_rng(17)
    def residual(q,point):
        env.data.qpos[env.qadr['left']]=q;mujoco.mj_forward(env.model,env.data)
        site=env.data.site('left_tcp')
        return np.r_[site.xpos-point,.1*Rotation.from_matrix(rot.T@site.xmat.reshape(3,3)).as_rotvec()]
    target=p+.004*normal-.015*radial
    fits=[least_squares(residual,np.clip(start,limits[:,0]+1e-6,limits[:,1]-1e-6),args=(target,),bounds=(limits[:,0],limits[:,1]),max_nfev=160) for start in [q0[0],*rng.uniform(limits[:,0],limits[:,1],size=(22,6))]]
    fit=min(fits,key=lambda f:np.linalg.norm(f.fun))
    print('multistart residual',fit.fun,'joints',fit.x,flush=True)
    q0[0]=fit.x
    for delta in (-.015,0,.015):
        point=p+.004*normal+delta*radial
        targets[0,:,:3,3]=R.T@(point-base.xpos);targets[0,:,:3,:3]=R.T@rot
        result=solver.solve(targets,q0)
        q=result.q_solution[0,-1];fk=solver.fk(q[None],'left')[0]
        pos=float(np.linalg.norm(fk[:3,3]-targets[0,-1,:3,3]))
        angle=float(np.degrees(np.arccos(np.clip((np.trace(fk[:3,:3].T@targets[0,-1,:3,:3])-1)/2,-1,1))))
        env.data.qpos[env.qadr['left']]=q;mujoco.mj_forward(env.model,env.data)
        contacts=[]
        for con in env.data.contact:
            bodies=[env.model.body(env.model.geom_bodyid[g]).name for g in (con.geom1,con.geom2)]
            if any(n.startswith('left_') for n in bodies):contacts.append(dict(bodies=bodies,distance_m=float(con.dist)))
        records.append(dict(target_world_m=point.tolist(),position_error_m=pos,orientation_error_deg=angle,joints=q.tolist(),robot_contacts=contacts))
        q0[0]=q
    result=dict(inference_calls=0,blade=0,approach_world=approach.tolist(),scrub_path_length_m=.03,targets=records,passed=all(r['position_error_m']<.01 and r['orientation_error_deg']<10 and not r['robot_contacts'] for r in records))
    sweep.atomic(sweep.HERE/'runs/propeller/validation/ik_reach.json',result)
    print(json.dumps(result,indent=1),flush=True)
    env.close()
    assert result['passed']

if __name__=='__main__':check()
