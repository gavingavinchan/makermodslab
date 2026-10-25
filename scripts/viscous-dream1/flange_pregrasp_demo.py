"""Loaded pregrasp feasibility, contact-gated weld fallback; zero inference."""
import argparse,json
from pathlib import Path
import numpy as np,mujoco,imageio.v2 as imageio
from PIL import Image,ImageDraw
from scipy.spatial.transform import Rotation,Slerp
import sweep
from flange_scene import HOLE,Measurements,HERE
from flange_start import setup,local_ik
from flange_ik_check import orientation

OUT=HERE/'runs/flange_dream/scripted'

def run(render=True,force_only=False):
    OUT.mkdir(parents=True,exist_ok=True)
    e=sweep.SweepEnv('flange',dict(sweep.DEFAULT,flange_attach=not force_only));setup(e,'pregrasp')
    ik=local_ik(e);measure=Measurements(e);qcmd=e.state();qcmd={s:np.r_[q[:6],0.] for s,q in qcmd.items()}
    renderer=mujoco.Renderer(e.model,width=640,height=360) if render else None
    writer=imageio.get_writer(OUT/'run.mp4',fps=10,codec='libx264',pixelformat='yuv420p',macro_block_size=1,quality=8) if render else None
    rows=[];chunks=[];ik_errors=[];stage='pregrasp: close on head and nut';idx=0
    desired={side:e.data.site(side+'_tcp').xpos.copy() for side in e.sides}
    def grab():
        images=e.images();renderer.update_scene(e.data,camera='flange_detail');close=renderer.render().copy()
        # Crop only the recording view to make the joint readable; policy cameras are unchanged.
        close=np.asarray(Image.fromarray(close).crop((100,30,500,255)).resize((640,360),Image.Resampling.LANCZOS))
        im=Image.new('RGB',(1920,400),'#14212b')
        for x,imdata,label in [(0,images['top'],'TOP'),(640,close,'FLANGE JOINT'),(1280,images['left'],'LEFT WRIST')]:
            im.paste(Image.fromarray(imdata),(x,0));ImageDraw.Draw(im).text((x+8,7),label,fill='#ffcf70')
        dr=ImageDraw.Draw(im);dr.text((12,367),f'{stage} | t={e.data.time:.1f}s | depth={rows[-1]["bolt_inserted_depth_m"]*1000 if rows else 0:.1f}mm | engaged={e.thread.engaged} | turns={e.thread.turns:.3f} | attach={e.grasps.active}',fill='white')
        if rows:
            error=max(np.max(np.abs(np.array(rows[-1]['qactual'][s][:6])-qcmd[s][:6])) for s in e.sides)
            dr.text((12,384),f'5 Nm arm limits | max joint tracking error {np.degrees(error):.1f} deg | no inference | contact-gated physical grasp weld',fill='#ffcf70')
        writer.append_data(np.asarray(im))
        if idx%40==0:im.save(OUT/f'frame_{idx:04d}.jpg')
    def step():
        nonlocal idx
        requested={}
        for side in e.sides:
            base=e.data.body(side+'_base_link');requested[side]=base.xmat.reshape(3,3).T@(desired[side]-base.xpos)
        distances=measure.requested(requested)
        e.step(qcmd['left'],qcmd['right']);r=measure.row(0,idx,distances)
        r.update(stage=stage,time=float(e.data.time),qcmd={s:q.tolist() for s,q in qcmd.items()},qactual={s:q.tolist() for s,q in e.state().items()},tcp={s:e.data.site(s+'_tcp').xpos.tolist() for s in e.sides},joint_force={s:e.data.actuator_force[e.aids[s]].tolist() for s in e.sides})
        rows.append(r);chunks.append(dict(row=idx,left=qcmd['left'].tolist(),right=qcmd['right'].tolist()))
        if render and idx%2==0:grab()
        idx+=1
    def hold(t):
        for _ in range(round(t/.05)):step()
    def mark(s):
        nonlocal stage
        stage=s;print('STAGE',s,'bolt',e.data.body('bolt').xpos.tolist(),'nut',e.data.body('nut').xpos.tolist(),'welds',e.grasps.active,'turns',e.thread.turns,flush=True)
    def joint(side,q,seconds=2):
        initial=qcmd[side].copy()
        for t in np.linspace(0,1,max(2,round(seconds/.05))):
            u=t*t*(3-2*t);qcmd[side]=initial+u*(q-initial);step()
    def pose(side,p,R,seconds=2,starts=0):
        desired[side]=np.asarray(p).copy()
        q,res=ik.fit(side,np.asarray(p),R,qcmd[side][:6],starts=starts)
        ik_errors.append(dict(side=side,stage=stage,target=np.asarray(p).tolist(),position_m=float(np.linalg.norm(res[:3])),rotation_deg=float(np.degrees(np.linalg.norm(res[3:])/.15))))
        if np.linalg.norm(res[:3])>.002 or np.linalg.norm(res[3:])/.15>.04:
            print('IK_LIMIT',side,np.linalg.norm(res[:3]),np.degrees(np.linalg.norm(res[3:])/.15),flush=True)
        joint(side,np.r_[q,qcmd[side][6]],seconds)
    def cart(side,p,R,seconds=3):
        initial=e.data.site(side+'_tcp').xpos.copy();rot=e.data.site(side+'_tcp').xmat.reshape(3,3).copy();slerp=Slerp([0,1],Rotation.from_matrix(np.stack([rot,R])))
        for t in np.linspace(0,1,31)[1:]:pose(side,initial+(p-initial)*t,slerp(t).as_matrix(),seconds/30)
    try:
        hold(1);mark('left: orient bolt forward at entrance')
        a=np.pi/3;R=orientation([-1,0,0],[0,np.cos(a),np.sin(a)])
        # Head-origin target x=.12: tip begins at x=0, so first align above/in front,
        # then approach the hole along +y while the tip stays outside x=.037.
        p=HOLE+[.19,.075,.05];cart('left',p,R,4);hold(.5)
        mark('left: align bolt with lower hole')
        cart('left',HOLE+[.195,0,0],R,4);hold(.5)
        mark('left: insert through both rings')
        cart('left',HOLE+[.118,0,0],R,6);hold(1)
        mark('right: orient nut at far-side bolt end')
        a=np.deg2rad(240);N=orientation([1,0,0],[0,np.cos(a),np.sin(a)])
        cart('right',HOLE+[-.16,.045,.02],N,4)
        cart('right',HOLE+[-.09,0,0],N,4);hold(.5)
        mark('right: advance nut to thread gate')
        b=e.data.body('bolt');tip=b.xpos+.120*b.xmat.reshape(3,3)[:,2]
        target=tip-.013*np.array([1.,0,0])-.028*N[:,2]
        cart('right',target,N,3);hold(1)
        if e.thread.engaged:
            for cycle in range(12):
                mark(f'right: tighten roll / regrasp {cycle+1}')
                startR=e.data.site('right_tcp').xmat.reshape(3,3).copy();startp=e.data.site('right_tcp').xpos.copy()
                axis=-np.array([1.,0,0]);delta=1.3
                targetR=Rotation.from_rotvec(axis*delta).as_matrix()@startR
                # Positive thread turns around bolt axis -X; nut advances +X.
                cart('right',startp+[.0035*delta/(2*np.pi),0,0],targetR,3);hold(.2)
                if e.thread.turns>=2:break
                joint('right',np.r_[qcmd['right'][:6],.85],.5);hold(.2)
                cart('right',e.data.site('right_tcp').xpos.copy(),startR,2)
                joint('right',np.r_[qcmd['right'][:6],0.],.6);hold(.3)
        else:mark('STOP: thread engagement failed')
        hold(1)
    finally:
        result=dict(inference_calls=0,finished=True,start=e.start_info,ik_errors=ik_errors,holding='force_only' if force_only else '160N + contact gated MuJoCo weld',success=bool(any(r['bolt_through_both'] for r in rows) and max([r['nut_turns'] for r in rows],default=0)>=2),max_bolt_inserted_depth_m=max([r['bolt_inserted_depth_m'] for r in rows],default=0),max_nut_turns=max([r['nut_turns'] for r in rows],default=0),nut_ever_engaged=any(r['nut_engaged'] for r in rows),grasp_events=e.grasps.events,thread_events=e.thread.events,row_measurements=rows)
        sweep.atomic(OUT/'log.json',result)
        with (OUT/'chunks.jsonl').open('w') as f:
            for c in chunks:f.write(json.dumps(c)+'\n')
        print({k:v for k,v in result.items() if k not in ('row_measurements','start','ik_errors')},flush=True)
        if writer:writer.close();renderer.close()
        e.close()
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--no-render',action='store_true');p.add_argument('--force-only',action='store_true');a=p.parse_args();run(not a.no_render,a.force_only)
