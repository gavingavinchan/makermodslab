import sys
sys.path.insert(0,'.')
import numpy as np,mujoco
from flange_yam import yam_env
from flange_start import local_ik
from flange_ik_check import orientation
from flange_scene import HOLE
E=yam_env();ik=local_ik(E)
print('rest',[(s,E.data.site(s+'_tcp').xpos.tolist(),E.data.site(s+'_tcp').xmat.reshape(3,3).tolist()) for s in E.sides],flush=True)
for axis in ([.75,-.433,-.5],[1,0,0],[0,-1,0],[.3,-.8,-.52],[0,0,-1]):
 for dx in (-.14,-.20):
  p=HOLE+[dx,.18,.05];R=orientation(axis,[0,0,1] if axis!=[0,0,-1] else [1,0,0])
  q,res=ik.fit('right',p,R,starts=5)
  print(axis,dx,'pe',np.linalg.norm(res[:3]),'ae',np.linalg.norm(res[3:])/.15,'q',q.tolist(),flush=True)
E.close()
