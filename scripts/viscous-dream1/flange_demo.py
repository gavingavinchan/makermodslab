"""Scripted flange arm demo. No inference; optional contact-gated grasp welds.
Joint waypoints use local IK, followed by the existing B0 actuator servos.
The free bolt/nut move only from gravity, collisions and ThreadModel when engaged.
"""
import json
import numpy as np
import mujoco
import imageio.v2 as imageio
from PIL import Image,ImageDraw
from scipy.spatial.transform import Rotation,Slerp
import sweep
from flange_scene import HERE,HOLE,Measurements
from flange_ik_check import LocalIK,orientation

OUT=HERE/'runs/flange_dream/scripted_table'


def run():
    (OUT/'validation').mkdir(parents=True,exist_ok=True)
    e,*_=sweep.initialize('flange',sweep.DEFAULT,20261023)
    ik=LocalIK(e);measure=Measurements(e)
    renderer=mujoco.Renderer(e.model,width=640,height=360)
    writer=imageio.get_writer(OUT/'scripted_demo.mp4',fps=10,codec='libx264',pixelformat='yuv420p',macro_block_size=1,quality=8)
    rows=[];stages=[];rowindex=0;stage='initial';qcmd=e.state()
    def grab():
        top=e.images()['top'];renderer.update_scene(e.data,camera='flange_detail');close=renderer.render().copy()
        im=Image.new('RGB',(1920,400),'#14212b');im.paste(Image.fromarray(top),(0,0));im.paste(Image.fromarray(close),(640,0));im.paste(Image.fromarray(e.images()['left']),(1280,0))
        dr=ImageDraw.Draw(im);t=e.thread
        dr.text((12,366),f'{stage}   |   t={e.data.time:.1f}s   |   engaged={t.engaged}   turns={t.turns:.2f}   advance={t.advance*1000:.1f}mm',fill='white')
        writer.append_data(np.asarray(im))
        if rowindex%40==0:im.save(OUT/f'validation/demo_frame_{rowindex:04d}.jpg')
    def step(left,right):
        nonlocal rowindex
        e.step(left,right);r=measure.row(0,rowindex,{})
        r['joint_actuator_force']={s:e.data.actuator_force[e.aids[s]].tolist() for s in e.sides}
        r['gripper_actuator_force']={s:float(e.data.actuator_force[e.model.actuator(s+'_gripper_left_joint_drive').id]) for s in e.sides}
        r['joint_tracking_error_rad']={s:(e.state()[s][:6]-qcmd[s][:6]).tolist() for s in e.sides}
        normal=capacity=0.;effective=[]
        for ci,c in enumerate(e.data.contact):
            ids=(int(c.geom1),int(c.geom2))
            if any(g in measure.objects['bolt'] for g in ids) and any(g in measure.fingers['left'] for g in ids):
                force=np.zeros(6);mujoco.mj_contactForce(e.model,e.data,ci,force)
                normal+=max(0.,float(force[0]));capacity+=max(0.,float(force[0]))*float(c.friction[0]);effective.append(float(c.friction[0]))
        r['bolt_left_normal_force_N']=normal;r['bolt_left_friction_capacity_N']=capacity;r['bolt_left_effective_friction']=effective
        r.update(stage=stage,sim_time_s=float(e.data.time),command={s:qcmd[s].tolist() for s in e.sides},tcp={s:e.data.site(s+'_tcp').xpos.tolist() for s in e.sides},arm_joints={s:e.state()[s].tolist() for s in e.sides});rows.append(r)
        if rowindex%2==0:grab()
        rowindex+=1
    def hold(seconds=1):
        for _ in range(round(seconds/.05)):step(qcmd['left'],qcmd['right'])
    def mark(label):
        nonlocal stage
        stage=label;stages.append(dict(stage=label,time_s=float(e.data.time),bolt_xyz=e.data.body('bolt').xpos.tolist(),nut_xyz=e.data.body('nut').xpos.tolist()))
        print('STAGE',label,'bolt',e.data.body('bolt').xpos.tolist(),flush=True)
    def joint_move(side,target,seconds=2.):
        start=qcmd[side].copy()
        for t in np.linspace(0,1,round(seconds/.05)):
            u=t*t*(3-2*t);qcmd[side]=start+u*(target-start);step(qcmd['left'],qcmd['right'])
    def pose(side,p,R,grip=None,seconds=2,starts=3):
        seed=qcmd[side][:6];q,res=ik.fit(side,np.asarray(p),R,seed,starts=starts)
        error=np.linalg.norm(res[:3]);angle=np.linalg.norm(res[3:])/.15
        if error>.005 or angle>np.deg2rad(4):
            print('IK LIMIT',side,p,error,np.degrees(angle),flush=True)
            stages.append(dict(stage='ik_limit',target=np.asarray(p).tolist(),pos_m=float(error),rot_deg=float(np.degrees(angle))))
        target=np.r_[q,qcmd[side][6] if grip is None else grip];joint_move(side,target,seconds)
        return error
    def cartesian(side,p,R,seconds=3):
        startp=e.data.site(side+'_tcp').xpos.copy();startR=e.data.site(side+'_tcp').xmat.reshape(3,3).copy()
        rotations=Slerp([0,1],Rotation.from_matrix(np.stack([startR,R])))
        for t in np.linspace(0,1,20)[1:]:
            pose(side,startp+(np.asarray(p)-startp)*t,rotations(t).as_matrix(),seconds=seconds/19,starts=0)
    try:
        grab();mark('left: approach loose bolt')
        pick_approach=np.array([0.,-2**-.5,-2**-.5])
        down=orientation(pick_approach,[0,2**-.5,-2**-.5])
        head=e.data.body('bolt').xpos.copy()+[-.040,0,0]
        pose('left',head-.044*pick_approach+[0,0,.075],down,grip=1,seconds=2)
        cartesian('left',head-.044*pick_approach,down,seconds=2)
        mark('left: close jaws on bolt shank')
        joint_move('left',np.r_[qcmd['left'][:6],.06],1);hold(.8)
        mark('left: lift bolt')
        cartesian('left',head-.044*pick_approach+[0,0,.075],down,seconds=3);hold(.8)
        grasp_after_lift=measure.row(0,-1,{})
        lifted=max(r['grasp']['bolt']['lift_m'] for r in rows)>.03
        inserted=0.
        if lifted:
            mark('left: rotate around bolt axis toward rim')
            a=np.deg2rad(15);approach=np.array([0,-np.cos(a),-np.sin(a)])
            oblique=orientation(approach,[0,np.sin(a),-np.cos(a)])
            # Rotation about world x preserves the already horizontal bolt axis.
            cartesian('left',HOLE+[.170,0,0]-.018*approach,oblique,seconds=4);hold(.5)
            mark('left: contact insertion along minus x')
            cartesian('left',HOLE+[.060,0,0]-.018*approach,oblique,seconds=5);hold(1)
            mark('left: release supported bolt and regrasp head')
            joint_move('left',np.r_[qcmd['left'][:6],.9],.6)
            current=e.data.site('left_tcp').xpos.copy()
            cartesian('left',current+[.050,0,0],oblique,seconds=2)
            joint_move('left',np.r_[qcmd['left'][:6],.20],.8);hold(.5)
            mark('left: final head-held insertion')
            cartesian('left',HOLE+[.060,0,0]-.018*approach,oblique,seconds=3);hold(1)
            inserted=max(r['bolt_inserted_depth_m'] for r in rows)
        else:
            mark('bolt pick failed: no sustained lift; insertion skipped')
            # Retreat before demonstrating the independent right-arm nut grasp.
            pose('left',head+[.03,.04,.05],down,grip=1,seconds=2)
        mark('right: approach loose nut')
        nut=e.data.body('nut').xpos.copy()
        pose('right',nut-.044*pick_approach+[0,0,.075],down,grip=1,seconds=2)
        cartesian('right',nut-.044*pick_approach,down,seconds=2)
        mark('right: grasp and lift nut')
        joint_move('right',np.r_[qcmd['right'][:6],.30],1);hold(.6)
        cartesian('right',nut-.044*pick_approach+[0,0,.080],down,seconds=2)
        mark('right: attempt far-side nut pose (bolt insertion incomplete)' if not lifted else 'right: bring nut to far-side bolt tip')
        a=np.deg2rad(15);far_approach=np.array([0,-np.cos(a),-np.sin(a)])
        far=orientation(far_approach,[0,np.sin(a),-np.cos(a)])
        pose('right',HOLE+[-.17,0,0]-.028*far_approach,far,seconds=4,starts=15)
        bolt=e.data.body('bolt');tip=bolt.xpos+.120*bolt.xmat.reshape(3,3)[:,2]
        target=np.array([max(-.16, min(-.10, tip[0]-.013-.022)),HOLE[1],HOLE[2]])
        pose('right',target-.028*far_approach,far,seconds=2,starts=12);hold(1)
        if e.thread.engaged:
            for cycle in range(8):
                mark(f'right: thread roll stroke {cycle+1}')
                q=qcmd['right'].copy();q[5]+=.8;joint_move('right',q,1)
                joint_move('right',np.r_[q[:6],.9],.3)
                q[5]-=.8;joint_move('right',q,.7)
                joint_move('right',np.r_[q[:6],.30],.3)
        else:mark('stop: nut did not enter thread engagement gate')
        hold(2)
        result=dict(bolt_mass_kg=float(e.model.body('bolt').mass[0]),nut_mass_kg=float(e.model.body('nut').mass[0]),inference_calls=0,object_pose_overrides=False,grasp_welds=e.grasps.enabled,grasp_events=e.grasps.events,rows=len(rows),sim_seconds=float(e.data.time),stages=stages,bolt_grasped=any(r['grasp']['bolt']['grasped'] for r in rows),nut_grasped=any(r['grasp']['nut']['grasped'] for r in rows),max_bolt_inserted_depth_m=inserted,max_nut_turns=max(r['nut_turns'] for r in rows),nut_ever_engaged=any(r['nut_engaged'] for r in rows),thread_events=e.thread.events,row_measurements=rows)
        sweep.atomic(OUT/'scripted_demo_log.json',result)
        print({k:v for k,v in result.items() if k not in ('row_measurements','stages')},flush=True)
    finally:writer.close();renderer.close();e.close()

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start',choices=['table','pregrasp'],default='table')
    parser.add_argument('--no-render',action='store_true')
    args=parser.parse_args()
    if args.start=='pregrasp':
        from flange_pregrasp_demo import run as pregrasp_run
        pregrasp_run(not args.no_render)
    else:run()
