# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Carry forward a BVH audit only after exact render-geometry pose equivalence.

Presentation changes do not require repeating identical triangle intersection queries.
This checks all mesh/curve geometry and all non-camera transforms at the same
1,343 audited poses, plus light and packed-mask identities. No physical result
is inferred from camera visibility.
"""
from pathlib import Path
import argparse,hashlib,json
import bpy,numpy as np

HERE=Path(__file__).parent;OUT=HERE/'results'
p=argparse.ArgumentParser();p.add_argument('--baseline-scene',type=Path,required=True);p.add_argument('--baseline-audit',type=Path,required=True);a=p.parse_args()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
base_audit=json.loads(a.baseline_audit.read_text());assert base_audit['passed'] and base_audit['samples']==1343
assert sha(a.baseline_scene)==base_audit['scene_sha256']

def snapshot(path):
 bpy.ops.wm.open_mainfile(filepath=str(path));s=bpy.context.scene;s.frame_set(1);dg=bpy.context.evaluated_depsgraph_get()
 objects=[o for o in s.objects if o.type!='CAMERA' and not o.name.startswith('Focus /')]
 geometry={};lights={};images={};materials={}
 for o in objects:
  if o.type in {'MESH','CURVE'}:
   materials[o.name]=tuple(slot.material.name if slot.material else None for slot in o.material_slots)
   assert o.animation_data is None and o.data.animation_data is None,('Animated local geometry requires a full audit',o.name)
   assert not getattr(o.data,'shape_keys',None),('Deforming geometry requires a full audit',o.name)
   assert not getattr(o.cycles,'use_adaptive_subdivision',False),('Camera-dependent subdivision',o.name)
   ev=o.evaluated_get(dg);m=ev.to_mesh();xyz=np.empty(len(m.vertices)*3,dtype=np.float32);m.vertices.foreach_get('co',xyz)
   indices=np.concatenate([np.array(f.vertices,dtype=np.int32) for f in m.polygons])
   geometry[o.name]=hashlib.sha256(xyz.tobytes()+indices.tobytes()).hexdigest();ev.to_mesh_clear()
  if o.type=='LIGHT':lights[o.name]=(o.data.type,o.data.energy,tuple(o.data.color))
 for im in bpy.data.images:
  if im.name=='Exact accumulated contact time':
   values=np.empty(len(im.pixels),dtype=np.float32);im.pixels.foreach_get(values);images[im.name]=hashlib.sha256(values.tobytes()).hexdigest()
 frames={}
 for f in np.arange(1,s.frame_end+.01,.5):
  s.frame_set(int(f),subframe=float(f%1));bpy.context.view_layer.update()
  frames[float(f)]={o.name:np.array(o.matrix_world) for o in objects}
 return geometry,lights,images,frames,materials
bg,bl,bi,bp,bm=snapshot(a.baseline_scene);ng,nl,ni,np_,nm=snapshot(OUT/'final_scene.blend')
assert bg==ng,'Render geometry changed'
assert bl==nl and bi==ni,'Lighting or packed contact image changed'
from native_appearance import TARGETS,NAME
changed_materials={name:dict(before=bm[name],after=nm[name]) for name in bm if bm[name]!=nm[name]}
assert set(changed_materials)==set(TARGETS),changed_materials
for name,source in TARGETS.items():assert bm[name]==(source,) and nm[name]==(NAME,),changed_materials
canister=bpy.data.materials[NAME];bs=canister.node_tree.nodes['Opaque anodized aluminium']
assert bs.inputs['Alpha'].default_value==1 and bs.inputs['Transmission Weight'].default_value==0
assert canister.animation_data is None and canister.node_tree.animation_data is None
assert bp.keys()==np_.keys() and all(bp[f].keys()==np_[f].keys() for f in bp),'Non-camera scene object set changed'
error=max(float(abs(bp[f][n]-np_[f][n]).max()) for f in bp for n in bp[f]);assert error==0,error
report=dict(passed=True,baseline_scene_sha256=sha(a.baseline_scene),scene_sha256=sha(OUT/'final_scene.blend'),baseline_audit_sha256=sha(a.baseline_audit),samples=len(bp),mesh_curve_geometry_identical=True,local_geometry_has_no_animation=True,authorized_material_overrides=changed_materials,canister_alpha=1.,canister_transmission=0.,lighting_identical=True,packed_contact_image_identical=True,non_camera_transform_max_difference=error,scope='Static evaluated mesh/curve geometry matches at frame1, exact non-camera transforms at all1343 audited poses; only cameras, focus helpers and the two authorized canister material assignments change')
(OUT/'camera_only_verification.json').write_text(json.dumps(report,indent=2)+'\n')
carried=dict(base_audit);carried.update(scene_sha256=report['scene_sha256'],carried_forward_from_scene_sha256=report['baseline_scene_sha256'],invariance_verification='camera_only_verification.json',invariance_verification_sha256=sha(OUT/'camera_only_verification.json'),method=base_audit['method']+'; exact same geometry and 1,343 non-camera poses verified in camera_only_verification.json before carrying the intersection result to this camera revision')
(OUT/'geometry_audit.json').write_text(json.dumps(carried,indent=2)+'\n')
print(json.dumps(report,indent=2),flush=True)
