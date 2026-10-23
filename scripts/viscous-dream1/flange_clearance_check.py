"""No-call geometric path checks with both arms, the full pipe and installed studs.
Checks joint interpolations at <=0.01 rad increments, keeping the other arm present.
This establishes kinematic clearance, not loaded tracking or dynamic grasp success.
"""
import json
import numpy as np
import mujoco
import sweep
from flange_scene import HERE

OUT=HERE/'runs/flange_v2/validation'

def check():
    e,*_=sweep.initialize('flange',sweep.DEFAULT,20261023);m,d=e.model,e.data
    records=json.loads((OUT/'ik_reach.json').read_text())
    assert records['pose_reach_passed']
    targets={r['name']:r for r in records['targets']};saved=d.qpos.copy()
    fixture={i for i in range(m.nbody) if m.body(i).name.startswith(('flange_','pipe_'))}
    arms={i for i in range(m.nbody) if m.body(i).name.startswith(('left_','right_'))}
    table={i for i in range(m.ngeom) if 'table' in (m.geom(i).name or '')}
    checks=0
    def collisions():
        nonlocal checks
        checks+=1;mujoco.mj_forward(m,d);hits=[]
        for c in d.contact:
            if c.dist>=0:continue
            a,b=int(c.geom1),int(c.geom2);ba,bb=int(m.geom_bodyid[a]),int(m.geom_bodyid[b])
            if (ba in arms and (bb in fixture or b in table)) or (bb in arms and (ba in fixture or a in table)):
                hits.append(dict(geoms=[m.geom(a).name,m.geom(b).name],depth_m=-float(c.dist)))
            if ba in arms and bb in arms:
                sa=m.body(ba).name.split('_')[0];sb=m.body(bb).name.split('_')[0]
                if sa!=sb:hits.append(dict(geoms=[m.geom(a).name,m.geom(b).name],depth_m=-float(c.dist)))
        return hits
    rest_hits=collisions();assert not rest_hits,rest_hits
    paths=[]
    def plan(side,name):
        rec=targets[name];start=d.qpos[e.qadr[side]].copy();goal=np.array(rec['joints'])
        for sign in (-1,1):d.joint(f'{side}_finger_{sign}').qpos[0]=.45*.0524125
        def valid(q):d.qpos[e.qadr[side]]=q;return not collisions()
        def edge(a,b,resolution=.045):
            for u in np.linspace(0,1,max(2,int(np.ceil(np.max(np.abs(b-a))/resolution))+1)):
                if not valid(a+(b-a)*u):return False
            return True
        assert valid(start),(side,name,'start collides');assert valid(goal),(side,name,'goal collides')
        route=[start,goal]
        dense=[]
        for a,b in zip(route,route[1:]):
            assert edge(a,b,.01),(side,name,'dense swept collision')
            dense.extend([a+(b-a)*u for u in np.linspace(0,1,max(2,int(np.ceil(np.max(np.abs(b-a))/.01))+1))])
        valid(goal)
        result=dict(side=side,target=name,waypoints=[q.tolist() for q in route],samples=len(dense),max_joint_step_rad=.01,fixture_table_and_other_arm_contacts=0)
        paths.append(result);print('PASS',side,name,'waypoints',len(route),'samples',len(dense),flush=True)
    try:
        # Each arm picks/retracts before the assembly approaches; the left remains
        # at insertion while the right reaches the far-side nut target.
        for side,pick in [('left','bolt_pick'),('right','nut_pick')]:
            initial=d.qpos[e.qadr[side]].copy();plan(side,pick)
            targets[side+'_rest']=dict(joints=initial.tolist());plan(side,side+'_rest')
        plan('left','hole_entrance_bolt_axis_minus_x');plan('left','bolt_through_both_grip_axis_minus_x');plan('right','far_side_nut')
        result=dict(inference_calls=0,rest_fixture_contacts=rest_hits,all_paths_passed=True,collision_evaluations=checks,paths=paths,scope='Kinematic arms versus full fixture, table and other arm. Loose-part contact allowed. Loaded dynamics and grasping not proven.')
        sweep.atomic(OUT/'clearance_paths.json',result)
    finally:d.qpos[:]=saved;e.close()

if __name__=='__main__':check()
