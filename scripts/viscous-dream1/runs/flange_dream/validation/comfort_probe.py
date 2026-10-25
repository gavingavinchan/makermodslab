import sys
sys.path.insert(0,'.')
import sweep,numpy as np,mujoco,json
from flange_start import local_ik
from flange_ik_check import orientation
from flange_scene import HOLE
E=sweep.SweepEnv('flange',sweep.DEFAULT);ik=local_ik(E);out=[]
side='left'
for dy in (.10,.18):
 for dz in (.05,.15,.25):
  for yaw in (30,60):
   for down in (0,30,60):
    y,a=np.deg2rad([yaw,down]);axis=np.array([-np.cos(y)*np.cos(a),-np.sin(y)*np.cos(a),-np.sin(a)])
    R=orientation(axis,[0,0,1]);p=HOLE+[.16,dy,dz]-.038*axis
    q,res=ik.fit(side,p,R,starts=5)
    if np.linalg.norm(res[:3])>.001 or np.linalg.norm(res[3:])/.15>.02:continue
    E.data.qpos[E.qadr[side]]=q;mujoco.mj_forward(E.model,E.data)
    jp=np.zeros((3,E.model.nv));jr=np.zeros_like(jp);mujoco.mj_jac(E.model,E.data,jp,jr,p+.093*axis,E.model.body(side+'_wrist_roll_link').id)
    tau=E.data.qfrc_bias[E.vadr[side]]+jp[2,E.vadr[side]]*.65*9.81
    c=ik.contacts(side,q,.44);c=[c for c in c if c['dist_m']<-.0003 and not any(n in ('bolt','nut') for n in c['bodies'])]
    r=dict(dy=dy,dz=dz,yaw=yaw,down=down,q=q.tolist(),p=p.tolist(),R=R.tolist(),max_tau=float(max(abs(tau))),contacts=len(c))
    out.append(r);print(dy,dz,yaw,down,r['max_tau'],len(c),flush=True)
json.dump(out,open('runs/flange_dream/validation/comfort_probe.json','w'),indent=1);E.close()
