"""IK-only feasibility script, actual MuJoCo contacts; no inference.
Privileged state selects verification targets, never moves the pin or fixtures.
"""
import argparse,json
import numpy as np,mujoco
from scipy.spatial.transform import Rotation,Slerp
from shackle_env import ShackleEnv
from shackle_scene import *
from flange_start import local_ik
from flange_ik_check import orientation

class Demo:
 def __init__(self,render=False):
  self.e=ShackleEnv();self.ik=local_ik(self.e);self.m=Measurements(self.e);self.q=self.e.state()['left'].copy();self.correction=np.zeros(6);self.rows=[];self.errors=[];self.stage='start';self.render=render
  if render:
   import imageio.v2 as imageio
   self.r=mujoco.Renderer(self.e.model,width=640,height=360)
   self.w=imageio.get_writer(OUT/'scripted_demo.mp4',fps=10,codec='libx264',pixelformat='yuv420p',macro_block_size=1,quality=8)
 def step(self):
  command=self.q.copy();command[:6]+=self.correction
  self.e.step(command,self.e.state()['right'])
  self.correction=np.clip(self.correction+.2*(self.q[:6]-self.e.state()['left'][:6]),-.12,.12)
  row=self.m.row(0,len(self.rows));row.update(stage=self.stage,qreference=self.q.tolist(),qactual=self.e.state()['left'].tolist(),tcp=self.e.data.site('left_tcp').xpos.tolist());self.rows.append(row)
  if self.render and len(self.rows)%2==0:
   from PIL import Image,ImageDraw
   ims=self.e.images();self.r.update_scene(self.e.data,camera='shackle_detail')
   canvas=Image.new('RGB',(1280,400),'#15212a');canvas.paste(Image.fromarray(ims['top']),(0,0));canvas.paste(Image.fromarray(self.r.render()),(640,0))
   dr=ImageDraw.Draw(canvas);dr.text((8,8),'TOP',fill='white');dr.text((648,8),'SHACKLE | inspection',fill='white')
   dr.text((8,365),f'{self.stage} | {row["time"]:.1f}s | engaged {row["engaged"]} | turns {row["turns"]:.2f} | seat gap {row["gap_to_seated_m"]*1000:.2f} mm',fill='white')
   dr.text((8,384),'Scripted IK, no inference | actual contact-gated grasp | 25 Nm sim arm limits',fill='#ffcf70');self.w.append_data(np.asarray(canvas))
 def hold(self,t):
  for _ in range(round(t/.05)):self.step()
 def mark(self,s):
  self.stage=s;print(s,'pin',self.e.data.body('pin').xpos.tolist(),'grasp',self.e.grasps.active,'thread',self.e.thread.engaged,self.e.thread.turns,flush=True)
 def joint(self,q,t=2):
  start=self.q.copy()
  for u in np.linspace(0,1,max(2,round(t/.05))):
   self.q=start+(q-start)*(u*u*(3-2*u));self.step()
 def pose(self,p,R,t=2,starts=0):
  q,res=self.ik.fit('left',np.array(p),R,self.q[:6],starts=starts)
  self.errors.append(dict(stage=self.stage,p=np.array(p).tolist(),pos=float(np.linalg.norm(res[:3])),ang=float(np.degrees(np.linalg.norm(res[3:])/.15))))
  if np.linalg.norm(res[:3])>.003:print('IK error',self.errors[-1],flush=True)
  self.joint(np.r_[q,self.q[6]],t)
 def cart(self,p,R,t=3):
  h=self.e.data.site('left_tcp');p0=h.xpos.copy();R0=h.xmat.reshape(3,3).copy();sl=Slerp([0,1],Rotation.from_matrix(np.stack([R0,R])))
  for u in np.linspace(0,1,21)[1:]:self.pose(p0+(np.array(p)-p0)*u,sl(u).as_matrix(),t/20)
 def grip(self,g,t=1):self.joint(np.r_[self.q[:6],g],t);self.hold(.3)
 def object_move(self,p,R,t=3):
  e=self.e;b=e.data.body('pin');p0=b.xpos.copy();r0=b.xmat.reshape(3,3).copy();sl=Slerp([0,1],Rotation.from_matrix(np.stack([r0,R])))
  count=max(2,round(t/.05))
  for i in range(count+30):
   u=min(1.,(i+1)/count);pd=p0+(np.array(p)-p0)*u;rd=sl(u).as_matrix()
   b=e.data.body('pin');h=e.data.site('left_tcp');H=h.xmat.reshape(3,3).copy();B=b.xmat.reshape(3,3).copy()
   local_R=H.T@B;local_offset=H.T@(h.xpos-b.xpos)
   ht=rd@local_R.T;pt=pd+ht@local_offset
   q,res=self.ik.fit('left',pt,ht,self.q[:6],starts=0)
   self.errors.append(dict(stage=self.stage,pos=float(np.linalg.norm(res[:3])),ang=float(np.degrees(np.linalg.norm(res[3:])/.15))))
   self.q[:6]=q;self.step()
   if e.thread.seat_latched:break
 def run(self):
  R=orientation([0,0,-1],[1,0,0]);e=self.e
  try:
   self.mark('approach upright pin');self.pose(PIN+[0,0,.064],R,3,starts=12)
   self.mark('close on shank');self.grip(0)
   if not e.grasps.active:raise RuntimeError('bilateral shank pickup did not attach')
   self.grip(max(0,e.state()['left'][6]-.004),.3)
   self.mark('lift pin');self.object_move(PIN+[0,.06,.025],np.eye(3),4)
   self.mark('carry over clear ear');self.object_move(np.r_[CENTER[:2],TOP+.004],np.eye(3),5)
   self.mark('insert through clear ear');self.object_move(np.r_[CENTER[:2],TOP-.021],np.eye(3),5)
   if e.data.body('pin').xpos[2]>TOP-.019:raise RuntimeError('pin blocked in clear ear')
   self.mark('release shank for head regrasp');self.grip(.8);self.hold(.5)
   self.mark('approach head');R=orientation([0,0,-1],[1,0,0])
   tip=e.data.body('pin').xpos.copy();self.cart(tip+[0,0,.124],R,3);self.grip(0)
   if not e.grasps.active:raise RuntimeError('head regrasp did not attach')
   self.grip(max(0,e.state()['left'][6]-.004),.3)
   self.mark('lower tip into tapped mouth');self.object_move(np.r_[CENTER[:2],MOUTH+.0003],np.eye(3),4)
   for cycle in range(65):
    if e.thread.seat_latched:break
    self.mark(f'tighten / regrasp {cycle+1}')
    b=e.data.body('pin');pos=b.xpos.copy();rot=b.xmat.reshape(3,3).copy();delta=np.deg2rad(120)
    self.object_move(pos+[0,0,-PITCH/3],Rotation.from_rotvec([0,0,-delta]).as_matrix()@rot,3)
    if e.thread.seat_latched:break
    self.grip(.8)
    # Reset around the pin axis, not around an off-centre old TCP. The head
    # stays supported by its thread while open; no part coordinates are set.
    self.cart(e.data.body('pin').xpos+[0,0,.124],orientation([0,0,-1],[1,0,0]),2)
    self.grip(0)
    if not e.grasps.active:self.hold(1.)
    if not e.grasps.active:raise RuntimeError('lost head on roll regrasp')
    self.grip(max(0,e.state()['left'][6]-.004),.3)
   self.mark('seated' if self.rows[-1]['seated'] else 'stopped short')
   if e.thread.seat_latched:self.grip(.8)
   self.hold(1)
  except Exception as ex:
   self.mark('STOP: '+str(ex));self.hold(.5)
  finally:
   result=dict(inference_calls=0,success=bool(self.rows[-1]['seated']),final=self.rows[-1],max_inserted_depth_m=max(r['inserted_depth_m'] for r in self.rows),max_turns=max(r['turns'] for r in self.rows),ever_engaged=any(r['engaged'] for r in self.rows),grasp_events=e.grasps.events,thread_events=e.thread.events,ik_errors=self.errors,rows=self.rows,**self.m.torque_summary())
   (OUT/'scripted_log.json').write_text(json.dumps(result,indent=1));print('RESULT',{k:v for k,v in result.items() if k not in ('rows','ik_errors','grasp_events','thread_events')},flush=True)
   if self.render:self.w.close();self.r.close()
   e.close()
  return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--render',action='store_true');a=p.parse_args();Demo(a.render).run()
