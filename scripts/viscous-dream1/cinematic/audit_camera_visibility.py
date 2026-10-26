# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Check framing and sampled visible surfaces in the actual animated camera."""
from pathlib import Path
import argparse,hashlib,json
import bpy,numpy as np
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view

HERE=Path(__file__).parent;OUT=HERE/'results'
p=argparse.ArgumentParser();p.add_argument('--step',type=int,default=1);a=p.parse_args()
bpy.ops.wm.open_mainfile(filepath=str(OUT/'final_scene.blend'));s=bpy.context.scene
hands={side:[o for o in s.objects if o.type=='MESH' and o.parent and o.parent.name in [side+'_gripper_left',side+'_gripper_right']] for side in ['left','right']}
groups={'left_hand':hands['left'],'right_hand':hands['right'],'pad':[o for o in s.objects if o.name.startswith(('ToolPad','ToolAbrasive')) and o.type=='MESH'],'polisher':[o for o in bpy.data.collections['CleaningTool'].all_objects if o.type=='MESH']}
local={}
for objects in groups.values():
 for o in objects:
  indices=np.linspace(0,len(o.data.polygons)-1,min(96,len(o.data.polygons))).astype(int)
  local[o.name]=[(o.data.polygons[int(i)].center.copy(),o.data.polygons[int(i)].normal.copy()) for i in indices]
mins={g:dict(visible_fraction=1.,frame=0) for g in groups};framing={g:1. for g in groups};issues=[]
blade=bpy.data.objects['Propeller / exact corrected blade 0'];blade_points=np.array([tuple(blade.matrix_world@v.co) for v in blade.data.vertices]);min_blade_margin=1;min_mount_margin=1;min_body_margin=1;tested=0
body=[o for o in s.objects if o.type=='MESH' and o.name.startswith(('left_','right_','rov_base__','thruster_'))]
for f in range(1,s.frame_end+1,a.step):
 s.frame_set(f);bpy.context.view_layer.update();dg=bpy.context.evaluated_depsgraph_get();cam=s.camera;origin=cam.matrix_world.translation
 viewproj=np.array(cam.calc_matrix_camera(dg,x=1280,y=720))@np.array(cam.matrix_world.inverted())
 def projected(points):
  clip=np.c_[np.asarray(points),np.ones(len(points))]@viewproj.T
  return clip[:,:2]/clip[:,3,None]*.5+.5,clip[:,3]
 for label,objects in groups.items():
  names={o.name for o in objects};candidates=[];inside=0;total=0
  for o in objects:
   normal_matrix=o.matrix_world.to_3x3().inverted().transposed()
   for co,nn in local[o.name]:
    point=o.matrix_world@co;n=(normal_matrix@nn).normalized();to_cam=(origin-point).normalized()
    if n.dot(to_cam)<=.03:continue
    point=point+n*.00002;candidates.append(point)
  if not candidates:raise RuntimeError((f,label,'No facing surface samples'))
  indices=np.linspace(0,len(candidates)-1,min(48,len(candidates))).astype(int);visible=0
  for i in indices:
   point=candidates[int(i)];uv,depth=projected([tuple(point)]);in_frame=bool(np.all((uv[0]>=.02)&(uv[0]<=.98)) and depth[0]>0)
   inside+=in_frame;total+=1
   direction=point-origin;dist=direction.length
   hit,loc,n,index,obj,matrix=s.ray_cast(dg,origin,direction.normalized(),distance=dist+.0001)
   if in_frame and (not hit or obj.name in names):visible+=1
  fraction=visible/total;framing[label]=min(framing[label],inside/total)
  if fraction<mins[label]['visible_fraction']:mins[label]=dict(visible_fraction=fraction,frame=f)
  if fraction<.08:issues.append(dict(frame=f,group=label,visible_fraction=fraction))
 # Entire working blade and both mounting points must fit, not just fingertips.
 uv,_=projected(blade_points);min_blade_margin=min(min_blade_margin,float(np.minimum(uv,1-uv).min()))
 mount_points=[tuple(bpy.data.objects[side+'_mount_origin'].matrix_world.translation) for side in ['left','right']]
 uv,_=projected(mount_points);min_mount_margin=min(min_mount_margin,float(np.minimum(uv,1-uv).min()))
 body_points=[tuple(o.matrix_world@Vector(corner)) for o in body for corner in o.bound_box]
 uv,_=projected(body_points);min_body_margin=min(min_body_margin,float(np.minimum(uv,1-uv).min()))
 tested+=1
 if tested%48==0:print('CAMERA_VISIBILITY',f,'issues',len(issues),flush=True)
report=dict(passed=not issues and min_blade_margin>.02 and min_mount_margin>.02 and min_body_margin>.02,samples=tested,frame_step=a.step,scene_sha256=hashlib.sha256((OUT/'final_scene.blend').read_bytes()).hexdigest(),minimum_visible_facing_surface_fraction=mins,minimum_sampled_framing_fraction=framing,minimum_whole_working_blade_frame_margin=min_blade_margin,minimum_mount_origin_frame_margin=min_mount_margin,minimum_native_body_bbox_frame_margin=min_body_margin,issues=issues,method='Project entire working blade and native mount origins; ray cast up to48 camera-facing polygon samples per hand, pad and complete polisher against full visible scene at every film frame',limits='Finite surface samples support visibility; visual review remains required. Same-group self-occlusion is accepted; no pixel segmentation or physical-contact claim.')
(OUT/'camera_visibility.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
assert report['passed']
