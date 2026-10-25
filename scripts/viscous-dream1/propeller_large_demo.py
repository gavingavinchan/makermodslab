# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Scripted physical scrub feasibility from an explicit pregrasp, zero inference.

The only pose setters are initialisation/IK scratch evaluations, restored before
physics. Every recorded trajectory sample advances the existing B0 servos and
MuJoCo contacts. No grasp welds, external forces or kinematic object animation.
"""
import argparse
import json
import hashlib
import importlib.metadata
import numpy as np
import mujoco
import imageio.v2 as imageio
from PIL import Image,ImageDraw
from flange_start import local_ik
from flange_ik_check import orientation
from propeller_scene import Reversal
from propeller_large_scene import LargeEnv as CardinalEnv,CENTER,HERE,PAD_Z,surface,ASSETS,LEFT_OPEN,SCRUB_T,SCRUB_AMPLITUDE
from propeller_large_check import setup,save_views,OUT


class Audit:
    def __init__(self,e):
        self.e=e;self.rows=[];self.total_ticks=0;self.clear()
        self.names=[e.model.geom(i).name or '' for i in range(e.model.ngeom)]
        self.bodies=[e.model.body(e.model.geom_bodyid[i]).name or '' for i in range(e.model.ngeom)]
        self.reversal=Reversal();self.lastpad=None;self.events={};self.unexpected={}
        self.previous_contact=False;self.intended_pairs={}
    def clear(self):
        self.counts=dict(pad=0,left_bilateral=0,right_bilateral=0,simultaneous=0)
        self.force_ticks={s:0 for s in self.e.sides};self.force_peak={s:0. for s in self.e.sides}
        self.ticks=0;self.penetration=0.;self.unwanted_penetration=0.;self.peak_normal=0.;self.left_radius=[]
    def tick(self):
        e=self.e;left=set();right=set();pad=False;normal=0.
        for s in e.sides:
            force=float(np.max(np.abs(e.data.actuator_force[e.aids[s]])))
            self.force_ticks[s]+=int(force>4.999);self.force_peak[s]=max(self.force_peak[s],force)
        for i,c in enumerate(e.data.contact):
            a,b=int(c.geom1),int(c.geom2);ns=[self.names[a],self.names[b]];bs=[self.bodies[a],self.bodies[b]]
            if not any(s.startswith(('left_','right_')) or s=='cardinal_tool' for s in bs):continue
            target=any(n.startswith('cardinal_blade_0_') for n in ns)
            intended=False
            for j,jaw in enumerate(('gripper_left','gripper_right')):
                if target and 'left_'+jaw in bs:
                    left.add(j);intended=True;self.left_radius.append(float(c.pos[0]-CENTER[0]))
                if 'cardinal_handle' in ns and 'right_'+jaw in bs:right.add(j);intended=True
            if 'cardinal_pad' in ns and target:
                pad=True;intended=True;f=np.zeros(6);mujoco.mj_contactForce(e.model,e.data,i,f);normal+=max(0.,float(f[0]))
            if intended:
                self.penetration=max(self.penetration,float(-c.dist))
                key=' / '.join(sorted(ns));self.intended_pairs[key]=max(self.intended_pairs.get(key,0),float(-c.dist))
            else:
                self.unwanted_penetration=max(self.unwanted_penetration,float(-c.dist))
                key=' / '.join(sorted(ns));self.unexpected[key]=max(self.unexpected.get(key,0),float(-c.dist))
        self.counts['pad']+=int(pad);self.counts['left_bilateral']+=int(len(left)==2);self.counts['right_bilateral']+=int(len(right)==2)
        self.counts['simultaneous']+=int(pad and len(left)==2 and len(right)==2)
        self.peak_normal=max(self.peak_normal,normal);self.ticks+=1;self.total_ticks+=1
    def row(self,stage,q,ik_error):
        e=self.e;pad=e.data.geom('cardinal_pad').xpos.copy();normal=surface(SCRUB_T)[1]
        actual_contact=self.counts['pad']==self.ticks
        rev,path=(0,0.)
        if actual_contact:rev,path=self.reversal.add(pad,normal)
        else:self.reversal.reset()
        row=dict(arm_force_limit_substeps=self.force_ticks.copy(),arm_peak_force_Nm=self.force_peak.copy(),time_s=float(e.data.time),stage=stage,substeps=self.ticks,contact_substeps=self.counts.copy(),pad_xyz=pad.tolist(),reversals=rev,tangent_path_continuous_contact_m=path,max_intended_penetration_m=self.penetration,max_unwanted_penetration_m=self.unwanted_penetration,pad_peak_normal_N=self.peak_normal,left_contact_radial_range_m=[min(self.left_radius),max(self.left_radius)] if self.left_radius else None,ik_error=ik_error,command={s:q[s].tolist() for s in e.sides},actual={s:e.state()[s].tolist() for s in e.sides},tcp={s:e.data.site(s+'_tcp').xpos.tolist() for s in e.sides},arm_force_Nm={s:e.data.actuator_force[e.aids[s]].tolist() for s in e.sides})
        self.rows.append(row);self.clear();return row


def run(render=True):
    OUT.mkdir(parents=True,exist_ok=True)
    e=CardinalEnv();save_views(e,'neutral')
    poses,q=setup(e);save_views(e,'pregrasp')
    audit=Audit(e);e.row_observer=audit.tick;ik=local_ik(e)
    renderer=mujoco.Renderer(e.model,width=960,height=540) if render else None
    writer=imageio.get_writer(OUT/'scripted_feasibility.mp4',fps=10,codec='libx264',pixelformat='yuv420p',macro_block_size=1,quality=8) if render else None
    index=0
    states=[];velocities=[];times=[]
    def step(stage,error=None):
        nonlocal index
        e.step(q['left'],q['right']);r=audit.row(stage,q,error)
        states.append(e.data.qpos.copy());velocities.append(e.data.qvel.copy());times.append(e.data.time)
        if render and index%2==0:
            renderer.update_scene(e.data,camera='cardinal_front');front=renderer.render().copy()
            imgs=e.images();frame=Image.new('RGB',(1600,940),'#16222b');frame.paste(Image.fromarray(front),(0,0))
            renderer.update_scene(e.data,camera='cardinal_detail');detail=renderer.render().copy();frame.paste(Image.fromarray(detail).resize((640,360)),(960,0))
            for x,key in enumerate(('top','left','right')):frame.paste(Image.fromarray(imgs[key]).resize((533,300)),(x*533,580))
            d=ImageDraw.Draw(frame)
            d.text((12,12),'SCRIPTED PHYSICS FEASIBILITY / initialized pregrasp / NO DREAM CALLS',fill='white')
            d.text((980,370),'96 cm propeller / 3x geometry / fixed shaft',fill='white')
            d.text((980,395),'LEFT arm holds OUTER EDGE of LEFT blade',fill='#ffca70')
            d.text((980,420),'RIGHT arm presses circular pad on SAME blade',fill='#ffca70')
            d.text((12,552),f'{stage}  t={r["time_s"]:.1f}s  | actual pad contact: {r["contact_substeps"]["pad"]}/50 substeps | bilateral left hold: {r["contact_substeps"]["left_bilateral"]}/50 | no weld',fill='white')
            for x,key in enumerate(('OVERHEAD','LEFT WRIST','RIGHT WRIST')):d.text((x*533+10,888),key+' (unaltered policy pixels, resized for video)',fill='white')
            d.text((12,914),'Servo-driven motion; pad is a free body. No material removal or abrasive spin model. Pickup/approach is not demonstrated.',fill='#ffca70')
            writer.append_data(np.asarray(frame))
        index+=1
    # Initial jaws close on the already-positioned blade/tool. No pose resets thereafter.
    for i in range(40):
        for side,opening in [('left',LEFT_OPEN),('right',.017)]:q[side][6]=opening/.0524125*max(0.,1-(i+1)/12)
        step('settle and close pregrasp')
    save_views(e,'ready')
    for i in range(240):
        # Two smooth commanded cycles over the enlarged outer working patch.
        t=SCRUB_T+SCRUB_AMPLITUDE*np.sin(2*np.pi*i/120)
        p,n=surface(t);R=orientation(-n,[1,0,0])
        # Gradually load compliant abrasive pad by 1 mm over the first second.
        gap=.006-.001*min(1.,i/20)
        point=p+(PAD_Z+.020+gap)*n
        fit,res=ik.fit('right',point,R,seed=q['right'][:6],starts=0)
        err=dict(position_m=float(np.linalg.norm(res[:3])),orientation_deg=float(np.degrees(np.linalg.norm(res[3:])/.15)))
        q['right']=np.r_[fit,0.]
        step('scripted scrub',err)
        if i==75:save_views(e,'scrub')
    for _ in range(20):step('finish hold')
    save_views(e,'final')
    if writer:writer.close();renderer.close()
    np.savez_compressed(OUT/'physics_replay.npz',qpos=states,qvel=velocities,time=times)
    rows=audit.rows;scrub=[r for r in rows if r['stage']=='scripted scrub'];ticks=sum(r['substeps'] for r in scrub)
    fractions={k:sum(r['contact_substeps'][k] for r in scrub)/ticks for k in audit.counts}
    limits={s:e.model.actuator(f'{s}_shoulder_lift_drive').forcerange.tolist() for s in e.sides}
    summary=dict(task='propeller_large',inference_calls=0,billed_calls=0,scripted_not_policy_success=True,simulation_only=True,attachment='none; free tool held by bilateral jaw contact',start='pregrasp initialized once; no pickup/approach demonstration',duration_s=float(e.data.time),scrub_s=12,diameter_m=.96,center_world_m=CENTER.tolist(),target='blade_0, world +X = screen-left from +Y arms/front view',blade_count=4,blade_axes_world=[[1,0,0],[0,0,1],[-1,0,0],[0,0,-1]],poses=poses,scrub_contact_fraction=fractions,scrub_reversals=sum(r['reversals'] for r in scrub),scrub_tangent_path_continuous_contact_m=sum(r['tangent_path_continuous_contact_m'] for r in scrub),pad_motion_xyz_range_m=np.ptp([r['pad_xyz'] for r in scrub],axis=0).tolist(),max_intended_penetration_m=max(r['max_intended_penetration_m'] for r in rows),max_unwanted_penetration_m=max(r['max_unwanted_penetration_m'] for r in rows),intended_pair_max_penetration_m=audit.intended_pairs,unexpected_contact_pairs=audit.unexpected,max_pad_normal_N=max(r['pad_peak_normal_N'] for r in rows),max_arm_force_Nm=max(abs(x) for r in rows for s in e.sides for x in r['arm_force_Nm'][s]),arm_force_limits_Nm=limits,max_scrub_ik_error_m=max(r['ik_error']['position_m'] for r in scrub),max_scrub_ik_angle_deg=max(r['ik_error']['orientation_deg'] for r in scrub),versions={n:importlib.metadata.version(n) for n in ('mujoco','numpy','scipy','trimesh','dreamlabs','dreamlabs_client','imageio','Pillow')},files_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [HERE/'propeller_large_scene.py',HERE/'propeller_large_check.py',HERE/'propeller_large_demo.py']})
    radial=Reversal();radial_count=0
    for r in scrub:radial_count+=radial.add(np.array([r['pad_xyz'][0],0.,0.]),np.array([0.,1.,0.]))[0]
    summary['radial_motion_reversals_10mm']=radial_count
    summary['gripper_force_limits_N']={s:e.model.actuator(s+'_gripper_left_joint_drive').forcerange.tolist() for s in e.sides}
    summary['arm_bases_world_m']={s:e.data.body(s+'_base_link').xpos.tolist() for s in e.sides}
    summary['max_intended_scrub_penetration_m']=max(r['max_intended_penetration_m'] for r in scrub)
    summary['gripper_pad_friction']=e.model.geom('cardinal_pad').friction.tolist()
    summary['ready_contact_substeps']=rows[39]['contact_substeps']
    summary['left_edge_radial_range_m']=[min(r['left_contact_radial_range_m'][0] for r in scrub if r['left_contact_radial_range_m']),max(r['left_contact_radial_range_m'][1] for r in scrub if r['left_contact_radial_range_m'])]
    summary['arm_force_limit_fraction']={s:sum(r['arm_force_limit_substeps'][s] for r in scrub)/ticks for s in e.sides}
    summary['peak_arm_force_by_side_Nm']={s:max(r['arm_peak_force_Nm'][s] for r in rows) for s in e.sides}
    summary['feasibility_passed']=bool(fractions['simultaneous']>.90 and summary['scrub_reversals']>=2 and summary['max_unwanted_penetration_m']<.001 and summary['max_scrub_ik_error_m']<.002 and all(p['error_m']<.002 for p in poses.values()))
    # Release is an independent post-demo physical-grasp check, not part of the clip.
    e.row_observer=None;z=float(e.data.body('cardinal_tool').xpos[2]);q['right'][6]=1.
    for _ in range(30):e.step(q['left'],q['right'])
    summary['release_tool_drop_m']=z-float(e.data.body('cardinal_tool').xpos[2])
    summary['weld_count']=int(sum(t==mujoco.mjtEq.mjEQ_WELD for t in e.model.eq_type))
    (OUT/'check.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (OUT/'trajectory.jsonl').open('w') as f:
        for r in rows:f.write(json.dumps(r)+'\n')
    print(json.dumps(summary,indent=2),flush=True);e.close();return summary

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--no-render',action='store_true');args=parser.parse_args();run(not args.no_render)
