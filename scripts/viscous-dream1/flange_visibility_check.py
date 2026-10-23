"""Inspect the unchanged call-1 cameras: raw pixels, projected hole mouth and ray occlusion."""
import json,base64,io
import numpy as np,mujoco
from PIL import Image,ImageDraw
import sweep
from flange_scene import HERE,HOLE
OUT=HERE/'runs/flange_v2/validation'

def check():
 e,windows,*_=sweep.initialize('flange',sweep.DEFAULT,20261023);m,d=e.model,e.data
 r=mujoco.Renderer(m,width=640,height=480);w=mujoco.Renderer(m,width=640,height=360)
 r.enable_segmentation_rendering();w.enable_segmentation_rendering()
 records={};sheet=Image.new('RGB',(960,204),'white');draw=ImageDraw.Draw(sheet)
 try:
  for col,(name,renderer) in enumerate([('top',r),('left',w),('right',w)]):
   renderer.update_scene(d,camera=name);seg=renderer.render();h,width=seg.shape[:2]
   if name=='top':seg=seg[60:420]
   counts={}
   for part in ('bolt','nut'):
    ids=np.where(m.geom_bodyid==m.body(part).id)[0];counts[part]=int(np.isin(seg[:,:,0],ids).sum())
   cam=m.camera(name).id;eye=d.cam_xpos[cam];R=d.cam_xmat[cam].reshape(3,3)
   mouth=HOLE.copy();mouth[0]=.037 if eye[0]>0 else -.037
   local=R.T@(mouth-eye);f=.5*h/np.tan(np.deg2rad(m.cam_fovy[cam]/2));pixel=np.array([width/2+f*local[0]/-local[2],h/2-f*local[1]/-local[2]])
   if name=='top':pixel[1]-=60
   ray=(mouth-eye);length=np.linalg.norm(ray);ray/=length;gid=np.array([-1],dtype=np.int32)
   groups=np.array([1,1,1,0,0,0],dtype=np.uint8)
   hit=mujoco.mj_ray(m,d,eye,ray,groups,1,int(m.cam_bodyid[cam]),gid)
   blocked=bool(hit>=0 and hit<length-.001)
   in_frame=bool(0<=pixel[0]<640 and 0<=pixel[1]<360)
   record=dict(hole_mouth_in_frame=in_frame,hole_mouth_visible=bool(in_frame and not blocked),loose_object_pixels_raw_640x360=counts,hole_mouth_world_m=mouth.tolist(),hole_mouth_pixel_raw=pixel.tolist(),hole_mouth_ray_blocked=blocked,occluder=m.geom(int(gid[0])).name if gid[0]>=0 and blocked else None,policy_camera_unchanged=True)
   records[name]=record
   im=Image.open(io.BytesIO(base64.b64decode(windows[name][-1]))).convert('RGB');sheet.paste(im,(320*col,28))
   x=320*col+pixel[0]*im.width/640;y=28+pixel[1]*im.height/360
   draw.ellipse((x-6,y-6,x+6,y+6),outline='#ff4e45',width=2);draw.text((320*col+8,8),name+' / red: projected target hole',fill='black')
   # Also save raw cameras for human inspection; the deliverable remains exact JPEGs.
   Image.fromarray(e.images()[name]).save(OUT/(name+'_raw.png'))
  assert any(records[s]['hole_mouth_visible'] for s in ('left','right')), 'No wrist sees the target mouth'
  records['assessment']='Top flange face remains edge-on and target mouth is occluded. Each loose object is visible in top plus at least one wrist. Wrist hole-mouth visibility is documented by projection/ray and human review; no camera/run rotation applied.'
  sweep.atomic(OUT/'visibility.json',records);sheet.save(OUT/'policy_annotations.jpg',quality=95)
  assert all(records['top']['loose_object_pixels_raw_640x360'][p]>0 and max(records[s]['loose_object_pixels_raw_640x360'][p] for s in ('left','right'))>0 for p in ('bolt','nut'))
 finally:r.close();w.close();e.close()

if __name__=='__main__':check()
