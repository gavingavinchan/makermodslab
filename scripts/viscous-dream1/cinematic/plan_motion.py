# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Retarget observed tangential path to original mounted ROV, never bench q.

The observed stripe geometry is smoothed by continuous_path.py and projected
onto the same corrected blade, using the level pose in selected_pose.json. The cinematic pad contact
anchor is .101 m from native tool0, with a free pad-axis roll.
This is kinematics, not a wet dynamics validation.
"""
from pathlib import Path
import argparse,hashlib,json,sys,numpy as np
from surface import surface,orientation,BENCH_CENTER
HERE=Path(__file__).parent
sys.path.insert(0,str(HERE/'vendor'))
from bimanual_common import inverse,fk,NAMES,JD
from scipy.optimize import least_squares
from continuous_path import continuous_path
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
path,path_report=continuous_path(source,finish-start)
seeds={k:np.array(v) for k,v in pose['seeds'].items()};records=[];errors=[]
def axis_inverse(side,pos,axis,seed):
    lo=np.array([float(JD[n].find('limit').get('lower'))+.015 for n in NAMES[side]])
    hi=np.array([float(JD[n].find('limit').get('upper'))-.015 for n in NAMES[side]])
    hi[1]=min(hi[1],pose.get('shoulder_lift_cap',hi[1]))
    def residual(q):
        T=fk(side,q)
        return np.r_[10*(T[:3,3]-pos),2*(T[:3,2]-axis),.001*(q-seed)]
    sol=least_squares(residual,np.clip(seed,lo+1e-8,hi-1e-8),bounds=(lo,hi),max_nfev=150,xtol=1e-11,ftol=1e-11,gtol=1e-11)
    T=fk(side,sol.x)
    return sol.x,float(np.linalg.norm(np.r_[5*(T[:3,3]-pos),T[:3,2]-axis]))
for f in range(1,END+1):
    sec=(f-1)/FPS;u_time=np.clip((sec-start)/(finish-start),0,1)*ts[-1]
    t=float(np.interp(u_time,ts,[x['actual_t'] for x in source]));u=float(np.interp(u_time,ts,[x['actual_u'] for x in source]));i=min(len(source)-1,int(np.searchsorted(ts,u_time)))
    t,u,i=path(sec-start)
    source_t=source[i]['actual_t'];source_u=source[i]['actual_u'];u_time=ts[i]
    u=pose.get('chord_pivot',0)+(u-pose.get('chord_pivot',0))*pose.get('chord_scale',1)
    outer=np.clip((t-.80)/.10,0,1);late=np.clip((u-.18)/.04,0,1)
    t-=pose.get('late_outer_radial_trim_t',0)*(outer*outer*(3-2*outer))*(late*late*(3-2*late))
    t+=pose.get('radial_offset_t',0)
    point,n=surface(t,u);rot=orientation(-n,pose.get('tool_x',[-1,0,0]));gap=pose.get('initial_gap_m',.018)*max(0,1-sec/start)**2
    pos=point+(.101+gap)*n
    joints={}
    for side,target,Rw in [('left',np.array(pose['leftp']),np.array(pose['leftR'])),('right',pos,rot)]:
        if side=='right' and pose.get('free_pad_roll'):
            q,e=axis_inverse(side,B.T@(target-base),B.T@Rw[:,2],seeds[side])
        else:q,e=inverse(side,B.T@(target-base),B.T@Rw,seeds[side])
        seeds[side]=q
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
report.update(free_pad_axis_roll=pose.get('free_pad_roll',False),initial_gap_m=pose.get('initial_gap_m',.018),require_horizontal=pose.get('require_horizontal',False))
report['continuous_path']=path_report
if pose.get('free_pad_roll'):
    report['retarget']='Smoothed observed stripe geometry traversed with positive arc-length speed through rounded lane changes. Native IK constrains surface contact position and pad axis with continuous wrist roll. Original native mounts; strict level vehicle. Authored kinematic path, not revalidated bench physics.'
active=[r for r in records if r['contact_duty']>=.999999]
xyz=np.array([r['pad'] for r in active]);speeds=np.linalg.norm(np.diff(xyz,axis=0),axis=1)*FPS
report['continuous_path'].update(minimum_frame_speed_m_s=float(speeds.min()),median_frame_speed_m_s=float(np.median(speeds)),maximum_frame_speed_m_s=float(speeds.max()),intermediate_dwell_intervals=int(np.count_nonzero(speeds<.001)))
assert speeds.min()>.004, ('Unexpected intermediate dwell',speeds.min())
(HERE/'results/motion.json').write_text(json.dumps(report,separators=(',',':')))
print('MOTION',len(records),'max residual',max(errors),'t',min(x['t'] for x in records),max(x['t'] for x in records),flush=True)
