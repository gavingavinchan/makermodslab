"""Native ClientIK reach certification. Local metadata only, no Dream client."""
import json,numpy as np,mujoco
from scipy.spatial.transform import Rotation
from shackle_scene import *
from shackle_env import ShackleEnv
from flange_ik_check import LocalIK,orientation

def run():
 e=ShackleEnv();ik=LocalIK(e);records=[];R=orientation([0,0,-1],[1,0,0])
 H=orientation([0,0,-1],[1,0,0])
 poses=[('pin_shank_grasp',PIN+[0,0,.064],R),('above_clear_ear_pin_vertical',np.r_[CENTER[:2],TOP+.004+.0618],R),('tapped_mouth_head_grasp',np.r_[CENTER[:2],MOUTH+.124],H),('seated_head_grasp',np.r_[CENTER[:2],SEAT+.124],H)]
 for label,p,H in poses:
  q,res=ik.fit('left',p,H,starts=12);q,pe,ae=ik.native('left',p,H,q)
  rec=dict(label=label,tool_point_world_m=p.tolist(),rotation=H.tolist(),q=q.tolist(),native_position_error_m=pe,native_angle_error_deg=ae,contacts=ik.contacts('left',q,.8));records.append(rec);print(label,pe,ae,flush=True)
 for depth in (MOUTH,SEAT):
  for angle in (-120,-105,-90,-75,-60,-45,-30,-15,0):
   H=Rotation.from_rotvec([0,0,np.deg2rad(angle)]).as_matrix()@orientation([0,0,-1],[1,0,0]);p=np.r_[CENTER[:2],depth+.124]
   q,res=ik.fit('left',p,H,starts=6);q,pe,ae=ik.native('left',p,H,q)
   records.append(dict(label=f'head_roll_{angle}_tip_{depth:.6f}',tool_point_world_m=p.tolist(),q=q.tolist(),native_position_error_m=pe,native_angle_error_deg=ae,contacts=ik.contacts('left',q,.8)));print(records[-1]['label'],pe,ae,flush=True)
 result=dict(inference_calls=0,targets=records,passed=all(r['native_position_error_m']<.003 and r['native_angle_error_deg']<3 and not r['contacts'] for r in records))
 (OUT/'reach.json').write_text(json.dumps(result,indent=2));e.close();assert result['passed']
if __name__=='__main__':run()
