import sys
sys.path.insert(0,'.')
import sweep,numpy as np,mujoco,json
from flange_ik_check import LocalIK,orientation
from flange_scene import HOLE
E,*_=sweep.initialize('flange',sweep.DEFAULT,20261024);ik=LocalIK(E)
for side,dx,axis in [('left',.23,[-1,0,0]),('left',.08,[-1,0,0]),('right',-.18,[1,0,0]),('right',-.11,[1,0,0])]:
 for jaw in ([0,0,1],[0,1,0]):
  R=orientation(axis,jaw);p=HOLE+[dx,0,0]-.034*R[:,2]
  q,res=ik.fit(side,p,R,starts=12)
  print(side,dx,jaw,np.linalg.norm(res[:3]),np.linalg.norm(res[3:])/.15*180/np.pi,q.tolist(),flush=True)
E.close()
