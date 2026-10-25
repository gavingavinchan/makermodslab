# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Retarget observed tangential path to original mounted ROV, never bench q.

The observed physics pad centre is projected onto the SAME corrected blade;
its actual t/u trace is adapted as documented in selected_pose.json. The cinematic pad contact
anchor is .101 m from native tool0, with a free pad-axis roll.
This is kinematics, not a wet dynamics validation.
"""
from pathlib import Path
import argparse,hashlib,json,sys,numpy as np
from surface import surface,orientation,BENCH_CENTER
HERE=Path(__file__).parent
sys.path.insert(0,str(HERE/'vendor'))
from bimanual_common import inverse,fk,NAMES,JD
p=argparse.ArgumentParser();p.add_argument('--draft',action='store_true');a=p.parse_args()
pose=json.loads((HERE/'selected_pose.json').read_text());B=np.array(pose['B']);base=np.array(pose['base']);M=np.eye(4);M[:3,:3]=B;M[:3,3]=base
if a.draft:
    source=[dict(time_s=t,actual_t=.70-.20*np.cos(np.pi*(t%7)/7) if int(t//7)%2==0 else .70+.20*np.cos(np.pi*(t%7)/7),actual_u=[0,.18,.36][min(2,int(t//7))],contact_duty={'simultaneous':1},lane_index=min(2,int(t//7))) for t in np.linspace(0,21,421)]
else:
    inp=json.loads((HERE/'assets/coverage_plan.json').read_text());assert inp['schema']=='propeller-edge-coverage-v1'
    source=[x for x in inp['frames'] if x['stage']=='scripted scrub']
    assert source, 'Frozen contract has no scripted scrub stage'
ts=np.array([x['time_s'] for x in source]);ts-=ts[0]
FPS=24;END=672;start=3.;finish=25.
seeds={k:np.array(v) for k,v in pose['seeds'].items()};records=[];errors=[]
for f in range(1,END+1):
    sec=(f-1)/FPS;u_time=np.clip((sec-start)/(finish-start),0,1)*ts[-1]
    t=float(np.interp(u_time,ts,[x['actual_t'] for x in source]));u=float(np.interp(u_time,ts,[x['actual_u'] for x in source]));i=min(len(source)-1,int(np.searchsorted(ts,u_time)))
    source_t=t;source_u=u;u=pose.get('chord_pivot',0)+(u-pose.get('chord_pivot',0))*pose.get('chord_scale',1)
    outer=np.clip((t-.80)/.10,0,1);late=np.clip((u-.18)/.04,0,1)
    t-=pose.get('late_outer_radial_trim_t',0)*(outer*outer*(3-2*outer))*(late*late*(3-2*late))
    t+=pose.get('radial_offset_t',0)
    point,n=surface(t,u);rot=orientation(-n,pose.get('tool_x',[-1,0,0]));gap=.018*max(0,1-sec/start)**2
    pos=point+(.101+gap)*n
    joints={}
    for side,target,Rw in [('left',np.array(pose['leftp']),np.array(pose['leftR'])),('right',pos,rot)]:
        q,e=inverse(side,B.T@(target-base),B.T@Rw,seeds[side]);seeds[side]=q
        if e>1e-5:raise RuntimeError((f,side,e,t,u))
        joints.update(zip(NAMES[side],q.tolist()));errors.append(e)
    # Opening is displacement per jaw from CAD closed. Tool sleeve radius=16 mm.
    joints['right_gripper_left_joint']=.012;joints['left_gripper_left_joint']=.0008
    T=M@fk('right',seeds['right']);actual_pad=T[:3,3]+.101*T[:3,2]
    gate=float(source[i]['contact_duty']['simultaneous']) if start<=sec<=finish and gap<1e-6 else 0
    records.append(dict(frame=f,time_s=sec,joints=joints,pad=actual_pad.tolist(),normal=n.tolist(),t=t,u=u,source_t=source_t,source_u=source_u,contact_duty=gate,lane_index=source[i]['lane_index'],source_time_s=float(u_time)))
report=dict(fps=FPS,end=END,frames=records,rov_transform=M.tolist(),max_ik_residual=max(errors),pad_radius_m=.026,pad_anchor_m=.101,tool_mount_offset_m=.020,contact_frames=[round(start*FPS)+1,round(finish*FPS)+1],draft=a.draft,source='draft geometric plan' if a.draft else 'assets/coverage_plan.json',retarget='Observed t/u projected to identical corrected surface; native IK and actual native-FK tool footprint; free pad-axis roll selected for native limits; physical pad tilt replaced by surface-normal alignment. Cinematic retiming to 22s.',physics_claim='Kinematic presentation only; no wet dynamics, thrust, torque, force, autonomy or abrasion simulation')
report['source_contract_sha256']=hashlib.sha256((HERE/'assets/coverage_plan.json').read_bytes()).hexdigest() if not a.draft else None
report['original_contract_sha256']=hashlib.sha256((HERE/'assets/coverage_plan.json').read_bytes()).hexdigest() if not a.draft else None
(HERE/'results/motion.json').write_text(json.dumps(report,separators=(',',':')))
print('MOTION',len(records),'max residual',max(errors),'t',min(x['t'] for x in records),max(x['t'] for x in records),flush=True)
