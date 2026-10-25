import sys
sys.path.insert(0,'.')
import sweep,numpy as np,json
from flange_start import local_ik
from flange_ik_check import orientation
from flange_scene import HOLE
E=sweep.SweepEnv('flange',sweep.DEFAULT);ik=local_ik(E)
a=np.pi/3;R=orientation([-1,0,0],[0,np.cos(a),np.sin(a)])
q=np.array([-3.105,-.441,3.409,.335,-1.534,1.047]);out=[]
for offset in [[.118,0,0],[.15,0,0],[.18,0,0],[.20,0,0],[.21,.04,.02],[.22,.08,.04],[.22,.12,.06],[.22,.15,.08]]:
 q,res=ik.fit('left',HOLE+offset,R,q,starts=0)
 print(offset,'pe',np.linalg.norm(res[:3]),'ae',np.linalg.norm(res[3:])/.15,'q',np.round(q,3),'c',ik.contacts('left',q,.44),flush=True)
 out.append(dict(offset=offset,q=q.tolist(),res=res.tolist()))
json.dump(out,open('runs/flange_dream2/validation/branch_probe.json','w'),indent=1)
E.close()
