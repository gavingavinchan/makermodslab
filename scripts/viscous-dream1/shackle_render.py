"""Final EGL stills, including an actual contact-demo insertion pose."""
import json
import numpy as np,mujoco
from PIL import Image
from shackle_env import ShackleEnv
from shackle_scene import OUT,TOP
from cross_scenes import viscous_xml,OPEN

def run():
 e=ShackleEnv();r=mujoco.Renderer(e.model,width=1280,height=720)
 for camera,name in [('shackle_overview','overview_front'),('shackle_plan','overview_top')]:
  r.update_scene(e.data,camera=camera);Image.fromarray(r.render()).save(OUT/(name+'.png'))
 log=OUT/'scripted_log.json'
 if log.exists():
  rows=json.loads(log.read_text())['rows'];rows=[x for x in rows if x['stage']=='insert through clear ear']
  row=min(rows,key=lambda x:abs(x['pin_xyz'][2]-(TOP-.005)))
  e.data.qpos[e.qadr['left']]=row['qactual'][:6]
  for sign in (-1,1):e.data.joint(f'left_finger_{sign}').qpos[0]=OPEN*row['qactual'][6]
  j=e.model.joint('pin_free');q=j.qposadr[0];e.data.qpos[q:q+7]=row['pin_xyz']+row['pin_quat_wxyz'];mujoco.mj_forward(e.model,e.data)
 r.update_scene(e.data,camera='shackle_detail');Image.fromarray(r.render()).save(OUT/'overview_detail.png')
 (OUT/'scene.xml').write_text(viscous_xml('shackle'));r.close();e.close()
if __name__=='__main__':run()
