# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Visible native CAD triangle intersections at film and half-frame poses.

Broad phase AABB followed by BVH triangle tests. Adjacent hardware interfaces
are explicitly excluded; this is a geometric audit, not dynamics validation.
"""
from pathlib import Path
import argparse,hashlib,json,time,sys,bpy,numpy as np
from mathutils.bvhtree import BVHTree
HERE=Path(__file__).parent;OUT=HERE/'results'
sys.path.insert(0,str(HERE/'vendor'))
from bimanual_common import fk,NAMES
scene_sha=hashlib.sha256((OUT/'final_scene.blend').read_bytes()).hexdigest()
p=argparse.ArgumentParser();p.add_argument('--step',type=float,default=.5);a=p.parse_args()
bpy.ops.wm.open_mainfile(filepath=str(OUT/'final_scene.blend'));s=bpy.context.scene
native={o.name:o for o in s.objects if o.type=='MESH' and (o.name.startswith(('left_','right_','rov_base__','thruster_')))}
static=[o for o in native.values() if o.name.startswith('rov_base__')]
rotors=[o for o in native.values() if o.name.startswith('thruster_')]
arms={side:[o for o in native.values() if o.name.startswith(side+'_') and o.parent.name not in [side+'_base_link',side+'_yaw_link']] for side in ['left','right']}
mount_hardware={side:[o for o in native.values() if o.name.startswith(side+'_') and o.parent.name in [side+'_base_link',side+'_yaw_link']] for side in ['left','right']}
tools=[o for o in bpy.data.collections['CleaningTool'].all_objects if o.type=='MESH']
ships=[o for o in s.objects if o.type=='MESH' and o.name.startswith(('Ship /','Propeller /'))]
allobjs=list({o.name:o for o in static+rotors+arms['left']+arms['right']+mount_hardware['left']+mount_hardware['right']+tools+ships}.values())
local={};faces={};permanent={};matrixcache={}
for o in allobjs:
 e=o.evaluated_get(bpy.context.evaluated_depsgraph_get());m=e.to_mesh();v=np.empty(len(m.vertices)*3,dtype=np.float32);m.vertices.foreach_get('co',v);local[o.name]=v.reshape(-1,3);faces[o.name]=[list(p.vertices) for p in m.polygons];e.to_mesh_clear()

def geom(o):
 name=o.name;M=np.asarray(o.matrix_world)
 if name in matrixcache and np.array_equal(matrixcache[name],M):return permanent[name]
 xyz=local[name]@M[:3,:3].T+M[:3,3];r=[xyz.min(0),xyz.max(0),None,xyz];matrixcache[name]=M.copy();permanent[name]=r;return r

def overlap(oa,ob):
 ga,gb=geom(oa),geom(ob)
 if np.any(ga[1]<gb[0]) or np.any(gb[1]<ga[0]):return 0
 if ga[2] is None:ga[2]=BVHTree.FromPolygons(ga[3],faces[oa.name])
 if gb[2] is None:gb[2]=BVHTree.FromPolygons(gb[3],faces[ob.name])
 return len(ga[2].overlap(gb[2]))
# Native continuous joint pins/covers and wrist nested bearing components meet
# at their designed interfaces. Non-adjacent articulated links are still tested.
order=['upper_arm_link','forearm_link','wrist_pitch_link','wrist_yaw_link','wrist_roll_link','gripper_left','gripper_right']
pairs=[]
for side,items in arms.items():
 for arm in items:
  for body in static+rotors:pairs.append((arm,body,'arm/vehicle'))
  for ship in ships:
   if side=='left' and 'gripper_' in arm.parent.name and ship.name.endswith('blade 0'):continue
   pairs.append((arm,ship,'arm/ship'))
 for i,aa in enumerate(items):
  for bb in items[i+1:]:
   pa,pb=aa.parent.name[len(side)+1:],bb.parent.name[len(side)+1:]
   if pa==pb:continue
   ia,ib=order.index(pa),order.index(pb)
   if abs(ia-ib)<=1 or (ia>=3 and ib>=3):continue
   pairs.append((aa,bb,'nonadjacent self'))
for aa in arms['left']:
 for bb in arms['right']:pairs.append((aa,bb,'mutual arms'))
for tool in tools:
 for arm in arms['left']+arms['right']:
  if tool.name=='ToolGraspSleeve' and arm.parent.name.startswith('right_gripper_'):continue
  pairs.append((tool,arm,'tool casing/arm'))
 for ship in ships:
  if tool.name.startswith(('ToolPad','ToolAbrasive')) and ship.name.endswith('blade 0'):continue
  pairs.append((tool,ship,'tool/ship'))
for body in static+rotors:
 for ship in ships:pairs.append((body,ship,'vehicle/ship'))
# Mount bearing interfaces with their native chassis/upper arm are intentional.
# Fixed mount hardware is still checked against the ship, opposite arm, tool,
# and distal links; the entire tool envelope is also checked against the chassis.
for side,items in mount_hardware.items():
 for aa in items:
  for bb in ships:pairs.append((aa,bb,'mount/ship'))
  for bb in arms['right' if side=='left' else 'left']+mount_hardware['right' if side=='left' else 'left']:pairs.append((aa,bb,'mount/opposite arm'))
  for bb in arms[side]:
   if bb.parent.name!=side+'_upper_arm_link':pairs.append((aa,bb,'mount/distal arm'))
  for bb in tools:pairs.append((aa,bb,'mount/tool'))
for tool in tools:
 for body in static+rotors:pairs.append((tool,body,'tool/vehicle'))
issues={};tested=0;max_fk_error=0;start=time.monotonic()
for f in np.arange(1,s.frame_end+.01,a.step):
 s.frame_set(int(f),subframe=float(f%1));bpy.context.view_layer.update()
 for side in ['left','right']:
  q=[bpy.data.objects[n+'_origin']['q'] for n in NAMES[side]]
  expected=np.asarray(bpy.data.objects['rov_base'].matrix_world)@fk(side,q)
  max_fk_error=max(max_fk_error,float(np.max(abs(expected-np.asarray(bpy.data.objects[side+'_tool0'].matrix_world)))))
 assert max_fk_error<1e-6,('STALE_DRIVER_OR_RIG_FK',f,max_fk_error)
 for oa,ob,kind in pairs:
  count=overlap(oa,ob)
  if count:
   key=(oa.name,ob.name,kind);item=issues.setdefault(key,dict(a=oa.name,b=ob.name,kind=kind,first_frame=float(f),last_frame=float(f),max_triangle_pairs=0,worst_frame=float(f),samples=0))
   item['last_frame']=float(f);item['samples']+=1
   if count>item['max_triangle_pairs']:item['max_triangle_pairs']=count;item['worst_frame']=float(f)
 tested+=1
 if tested%24==0:print('COLLISION_AUDIT',float(f),'issues',len(issues),'elapsed',round(time.monotonic()-start,1),flush=True)
 if tested==24 and issues:
  print('EARLY_ISSUES',json.dumps(list(issues.values())),flush=True)
# Grip inspection with exact CAD jaw triangle surfaces, per-cylinder sample.
s.frame_set(350);grip=[]
for side in ['left','right']:
 for jaw in ['left','right']:
  objs=[o for o in arms[side] if o.parent.name==side+'_gripper_'+jaw];v=[];ff=[]
  for o in objs:
   gg=geom(o);offset=len(v);v.extend(gg[3]);ff.extend([[offset+i for i in face] for face in faces[o.name]])
  bt=BVHTree.FromPolygons(v,ff)
  if side=='right':
   from mathutils import Vector
   T=bpy.data.objects['CleaningToolRoot'].matrix_world
   probes=[T@Vector((.016*np.cos(th),.016*np.sin(th),z)) for z in np.linspace(-.034,.034,41) for th in np.linspace(0,2*np.pi,128,endpoint=False)]
   distances=[bt.find_nearest(x)[3] for x in probes]
   idx=int(np.argmin(distances));nearest=bt.find_nearest(probes[idx])[0]
   grip.append(dict(side=side,jaw=jaw,cylinder_to_jaw_min_distance_m=float(min(distances)),closest_sleeve_point_local_m=list(T.inverted()@probes[idx]),closest_jaw_point_in_tool_local_m=list(T.inverted()@nearest),tool_root_to_tcp_translation_m=[0,0,.020],sleeve_z_band_local_m=[-.034,.034],scope='D32 mm sleeve surface sampled 41 axial x128 angular points using exact mounted CleaningToolRoot matrix against CAD triangles'))
  else:
   blade=bpy.data.objects['Propeller / exact corrected blade 0'];bg=geom(blade)
   if bg[2] is None:bg[2]=BVHTree.FromPolygons(bg[3],faces[blade.name])
   intersections=bt.overlap(bg[2]);distances=[bg[2].find_nearest(x)[3] for x in v]
   grip.append(dict(side=side,jaw=jaw,blade_surface_intersection_triangle_pairs=len(intersections),nearest_vertex_blade_distance_m=float(min(distances))))
report=dict(passed=not issues,max_native_fk_vs_blender_matrix_error=max_fk_error,scene_sha256=scene_sha,samples=tested,frame_step=a.step,film_frames=s.frame_end,collision_pair_count=len(pairs),elapsed_s=time.monotonic()-start,issues=list(issues.values()),grip_checks=grip,method='AABB broad phase then exact visible-mesh BVH triangle intersections; native mesh transforms evaluated at every film frame and half-frame',excluded=['Native base/yaw-link mounting interfaces vs vehicle and own upper-arm bearing assembly; mounts remain checked against ship, opposite arm, tool and distal links','Same rigid link meshes, adjacent articulated link pairs, nested wrist bearing/roll/finger interfaces','Left finger/blade_0 intended pinch','Right fingers/ToolGraspSleeve intended grip; other tool casing remains checked','ToolPad*/ToolAbrasive* vs blade_0 intended scrub contact'],limits='Triangle intersection test is not a signed-distance clearance certificate; no hydrodynamics, force, joint backlash or soft pad deformation model')
(OUT/'geometry_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
