import sys
sys.path.insert(0,'.')
import sweep,numpy as np,mujoco,json
from flange_start import local_ik
from flange_ik_check import orientation
from flange_scene import HOLE
E=sweep.SweepEnv('flange',sweep.DEFAULT);ik=local_ik(E);out=[]
for side,sg in [('left',1),('right',-1)]:
 for y in (.06,.16):
  for x in (.16,.22):
   for tilt in (0,30,60):
    a=np.deg2rad(tilt);axis=np.array([-sg*np.cos(a),-np.sin(a),0]);R=orientation(axis,[0,0,1]);p=HOLE+[sg*x,y,.05]-.038*axis
    q,res=ik.fit(side,p,R,starts=4)
    c=ik.contacts(side,q,.44)
    c=[c for c in c if c['dist_m']<-.0003 and not any(n in ('bolt','nut') for n in c['bodies'])]
    r=dict(side=side,xyz=p.tolist(),R=R.tolist(),q=q.tolist(),pe=float(np.linalg.norm(res[:3])),ae=float(np.degrees(np.linalg.norm(res[3:])/.15)),contacts=c)
    out.append(r);print(side,y,x,tilt,round(r['pe']*1000,2),round(r['ae'],1),len(c),flush=True)
json.dump(out,open('runs/flange_dream/validation/start_probe.json','w'),indent=1);E.close()
