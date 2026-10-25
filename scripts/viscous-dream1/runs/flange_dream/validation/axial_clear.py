import sys
sys.path.insert(0,'.')
import sweep,numpy as np,mujoco,json
from flange_start import local_ik
from flange_ik_check import orientation
from flange_scene import HOLE
E=sweep.SweepEnv('flange',sweep.DEFAULT);ik=local_ik(E);out=[]
for side,xx,axis in [('left',.118,[-1,0,0]),('left',.158,[-1,0,0]),('right',-.081,[1,0,0])]:
 for deg in range(0,360,30):
  a=np.deg2rad(deg);R=orientation(axis,[0,np.cos(a),np.sin(a)]);p=HOLE+[xx,0,0]
  q,res=ik.fit(side,p,R,starts=8);c=ik.contacts(side,q,.44)
  c=[c for c in c if c['dist_m']<-.0003 and not any(n in ('bolt','nut') for n in c['bodies'])]
  r=dict(side=side,x=xx,roll=deg,xyz=p.tolist(),R=R.tolist(),q=q.tolist(),pe=float(np.linalg.norm(res[:3])),ae=float(np.degrees(np.linalg.norm(res[3:])/.15)),contacts=c)
  out.append(r);print(side,xx,deg,round(r['pe']*1000,2),round(r['ae'],1),len(c),flush=True)
json.dump(out,open('runs/flange_dream/validation/axial_clear.json','w'),indent=1);E.close()
