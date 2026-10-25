import sys
sys.path.insert(0,'.')
import numpy as np,mujoco
from flange_yam import yam_env
from flange_start import put
from flange_scene import HOLE
E=yam_env();s='right';q=[-.01964249,1.51974227,.80661211,-1.5708,-1.55796403,.85757085]
E.data.qpos[E.qadr[s]]=q
for sign in (-1,1):E.data.joint(f'{s}_finger_{sign}').qpos[0]=.0225
mujoco.mj_forward(E.model,E.data);h=E.data.site(s+'_tcp');p=h.xpos.copy();H=h.xmat.reshape(3,3).copy()
print('tcp',p,'R',H,flush=True)
for offset in np.arange(-.07,.071,.01):
 C=np.array([[np.cos(np.pi/6),-np.sin(np.pi/6),0],[np.sin(np.pi/6),np.cos(np.pi/6),0],[0,0,1]])
 put(E,'nut',p+offset*H[:,2],H@C)
 touched=set()
 for c in E.data.contact:
  a,b=int(c.geom1),int(c.geom2)
  for j,ids in enumerate(E.grasps.jaws[s]):
   if (a in E.grasps.parts['nut'] and b in ids) or (b in E.grasps.parts['nut'] and a in ids):touched.add(j)
 print(round(offset,3),touched,flush=True)
E.close()
