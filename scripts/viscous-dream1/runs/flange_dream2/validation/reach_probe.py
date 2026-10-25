import sys
sys.path.insert(0,'.')
import sweep,numpy as np,json
from flange_start import local_ik
from flange_ik_check import orientation
from flange_scene import HOLE
E=sweep.SweepEnv('flange',sweep.DEFAULT);ik=local_ik(E);out=[]
for x in (.195,.180,.158,.140,.118):
 for roll in (0,60,120,180,240,300):
  a=np.deg2rad(roll);R=orientation([-1,0,0],[0,np.cos(a),np.sin(a)])
  q,res=ik.fit('left',HOLE+[x,0,0],R,starts=12)
  r=dict(x=x,roll=roll,q=q.tolist(),pe=float(np.linalg.norm(res[:3])),ae=float(np.linalg.norm(res[3:])/.15))
  out.append(r);print(r,flush=True)
json.dump(out,open('runs/flange_dream2/validation/reach_probe.json','w'),indent=1);E.close()
