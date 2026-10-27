# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Still-only logo QA, exact preservation proof, and badge triangle audit."""
from pathlib import Path
import argparse,json,hashlib,math,time
import bpy,numpy as np
from mathutils import Matrix,Vector
from mathutils.bvhtree import BVHTree
HERE=Path(__file__).resolve().parent;OUT=HERE/'results/logo_check'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def val(x):
    if isinstance(x,bpy.types.ID):return x.name_full
    if hasattr(x,'to_list'):return x.to_list()
    if isinstance(x,(str,int,float,bool)) or x is None:return x
    try:return [val(v) for v in x]
    except TypeError:return str(x)
def props(x,depth=0):
    out={}
    for p in x.bl_rna.properties:
        k=p.identifier
        if k in ('rna_type','session_uid'):continue
        v=getattr(x,k)
        if p.type=='POINTER':
            if isinstance(v,bpy.types.ID) or v is None:out[k]=val(v)
            elif depth<2:out[k]=props(v,depth+1)
        elif p.type!='COLLECTION':out[k]=val(v)
    return out
def anim(id):
    a=id.animation_data
    if not a:return None
    drivers=[]
    for f in a.drivers:
        drivers.append(dict(path=f.data_path,index=f.array_index,expression=f.driver.expression,type=f.driver.type,variables=[dict(name=v.name,type=v.type,targets=[dict(id=val(t.id),data_path=t.data_path,bone_target=t.bone_target,transform_type=t.transform_type,transform_space=t.transform_space) for t in v.targets]) for v in f.driver.variables]))
    return dict(action=val(a.action),drivers=drivers,nla=[props(t) for t in a.nla_tracks])
def snapshot(path,keep=None):
    bpy.ops.wm.open_mainfile(filepath=str(path));s=bpy.context.scene
    objects={o.name:o for o in s.objects if keep is None or o.name in keep}
    saved_frame=s.frame_current;saved_camera=val(s.camera)
    state=dict(render=props(s.render),cycles=props(s.cycles),view=props(s.view_settings),frame=[s.frame_start,s.frame_end,s.frame_step,s.render.fps,s.render.fps_base],current_frame=saved_frame,active_camera=saved_camera,world=val(s.world),markers=[dict(name=m.name,frame=m.frame,camera=val(m.camera)) for m in s.timeline_markers],objects={},actions={})
    for o in objects.values():
        d=dict(type=o.type,parent=val(o.parent),parent_type=o.parent_type,basis=val(o.matrix_basis),parent_inverse=val(o.matrix_parent_inverse),anim=anim(o),custom={k:val(o[k]) for k in o.keys()},constraints=[props(c) for c in o.constraints],modifiers=[props(m) for m in o.modifiers])
        if o.type in ('CAMERA','LIGHT'):d['data']=props(o.data);d['data_anim']=anim(o.data)
        if o.type=='MESH':
            v=np.empty(len(o.data.vertices)*3,dtype=np.float32);o.data.vertices.foreach_get('co',v)
            faces=np.array([i for f in o.data.polygons for i in f.vertices],dtype=np.int32)
            d['mesh_sha256']=hashlib.sha256(v.tobytes()+faces.tobytes()).hexdigest();d['materials']=[val(x.material) for x in o.material_slots]
        state['objects'][o.name]=d
    for a in bpy.data.actions:
        curves=[]
        for layer in a.layers:
            for strip in layer.strips:
                for bag in strip.channelbags:
                    for f in bag.fcurves:curves.append(dict(path=f.data_path,index=f.array_index,keys=[dict(co=val(k.co),left=val(k.handle_left),right=val(k.handle_right),interpolation=k.interpolation,easing=k.easing) for k in f.keyframe_points]))
        state['actions'][a.name]=curves
    transforms=[];cameras=[]
    for frame in range(s.frame_start,s.frame_end+1):
        s.frame_set(frame);bpy.context.view_layer.update();transforms.append([np.array(o.matrix_world) for o in objects.values()]);cameras.append(val(s.camera))
    return state,np.array(transforms),cameras

def verify():
    reports={}
    for label,path in [('native',HERE/'assets/native_rov.blend'),('film',HERE/'results/final_scene.blend')]:
        before=path.with_suffix('.prelogo.blend');a,ap,ac=snapshot(before);b,bp,bc=snapshot(path,set(a['objects']))
        diffs=[k for k in a if a[k]!=b[k]]
        err=float(abs(ap-bp).max());report=dict(passed=not diffs and err==0 and ac==bc,baseline_sha256=sha(before),scene_sha256=sha(path),preexisting_objects=len(a['objects']),compared_frames=len(ap),differing_sections=diffs,world_transform_max_difference=err,camera_bindings_identical_all_frames=ac==bc,cameras_markers_active_camera_render_lights_geometry_material_slots_animation_identical=not diffs)
        reports[label]=report
        if diffs:
            (OUT/f'{label}_before.json').write_text(json.dumps(a,indent=2));(OUT/f'{label}_after.json').write_text(json.dumps(b,indent=2))
        print(label,report,flush=True)
    (OUT/'unchanged.json').write_text(json.dumps(reports,indent=2)+'\n');assert all(r['passed'] for r in reports.values())

def openfilm():
    bpy.ops.wm.open_mainfile(filepath=str(HERE/'results/final_scene.blend'));return bpy.context.scene

def stills():
    source_scene=openfilm();s=source_scene.copy();s.name='LOGO_CHECK_TEMP_SCENE'
    # Camera-bound timeline markers override the active camera when rendering.
    # Render a disposable scene copy with an empty, independent marker list.
    s.timeline_markers.clear();bpy.context.window.scene=s;original_camera=s.camera
    # The saved file is never written during QA. All camera/render overrides are
    # temporary in this process, then the temporary camera datablock is removed.
    d=bpy.data.cameras.new('LOGO_CHECK_TEMP');cam=bpy.data.objects.new('LOGO_CHECK_TEMP',d);s.collection.objects.link(cam)
    s.render.engine='CYCLES';s.cycles.samples=8;s.cycles.use_denoising=True;s.render.resolution_x=720;s.render.resolution_y=480;s.render.resolution_percentage=100;s.render.image_settings.file_format='PNG';s.render.use_persistent_data=True
    prefs=bpy.context.preferences.addons['cycles'].preferences;prefs.compute_device_type='OPTIX';prefs.get_devices()
    for dev in prefs.devices:dev.use=dev.type=='OPTIX'
    s.cycles.device='GPU';d.type='ORTHO';d.clip_start=.001;d.clip_end=200
    for frame in [1,(s.frame_start+s.frame_end)//2,s.frame_end]:
        s.frame_set(frame);bpy.context.view_layer.update();d.type='ORTHO'
        for arm in ('left','right'):
            parent=bpy.data.objects[arm+'_forearm_link'];M=parent.matrix_world
            for side,label in [(1,'PosY'),(-1,'NegY')]:
                target=M@Vector((.057,side*.035,.05879));back=M.to_3x3()@Vector((0,side,0));up=M.to_3x3()@Vector((0,0,1));right=up.cross(back).normalized()
                rot=Matrix((right,up,back)).transposed().to_4x4();rot.translation=target+back*.255;cam.matrix_world=rot;d.ortho_scale=.182;s.camera=cam
                s.render.filepath=str(OUT/f'{arm}_{label}_{frame:04d}.png');bpy.ops.render.render(write_still=True,scene=s.name)
        reference=bpy.data.objects['CAM_Context_ThreeQuarter'];cam.matrix_world=reference.matrix_world.copy();d.type='PERSP';d.lens=reference.data.lens;d.sensor_width=reference.data.sensor_width;s.camera=cam
        s.render.filepath=str(OUT/f'wide_{frame:04d}.png');bpy.ops.render.render(write_still=True,scene=s.name)
        print('STILLS_FRAME',frame,flush=True)
    s.camera=original_camera;bpy.data.objects.remove(cam,do_unlink=True);bpy.data.cameras.remove(d);bpy.context.window.scene=source_scene;bpy.data.scenes.remove(s)

def audit():
    s=openfilm();badges=list(bpy.data.collections['LOGO_BADGES'].objects)
    others=[o for o in s.objects if o.type=='MESH' and o.name.startswith(('left_','right_','rov_base__','thruster_'))]
    objs=badges+others;local={};faces={};cache={};geom={}
    for o in objs:
        v=np.empty(len(o.data.vertices)*3,dtype=np.float32);o.data.vertices.foreach_get('co',v);local[o.name]=v.reshape(-1,3);faces[o.name]=[list(p.vertices) for p in o.data.polygons]
    def get(o):
        M=np.asarray(o.matrix_world);key=o.name
        if key not in cache or not np.array_equal(cache[key],M):
            xyz=local[key]@M[:3,:3].T+M[:3,3];geom[key]=[xyz.min(0),xyz.max(0),None,xyz];cache[key]=M.copy()
        return geom[key]
    issues={};pairs=0;start=time.monotonic()
    # Own parent link is the intentional flush mounting interface, excluded.
    candidates=[(a,b) for a in badges for b in others if a.parent!=b.parent]
    for frame in range(s.frame_start,s.frame_end+1):
        s.frame_set(frame);bpy.context.view_layer.update()
        for a,b in candidates:
            aa,bb=get(a),get(b)
            if np.any(aa[1]<bb[0]) or np.any(bb[1]<aa[0]):continue
            pairs+=1
            if aa[2] is None:aa[2]=BVHTree.FromPolygons(aa[3],faces[a.name])
            if bb[2] is None:bb[2]=BVHTree.FromPolygons(bb[3],faces[b.name])
            hits=aa[2].overlap(bb[2])
            if hits:issues.setdefault(a.name+' / '+b.name,[]).append(frame)
        if frame%100==0:print('BADGE_AUDIT',frame,'seconds',round(time.monotonic()-start,1),flush=True)
    report=dict(frames=s.frame_end-s.frame_start+1,stride=1,badges=len(badges),other_meshes=len(others),candidate_pairs=len(candidates),broadphase_survivors=pairs,intersections=issues,method='World-space native mesh triangle BVH surface intersections at every integer film frame. Own carrier link excluded as intentional mounting contact; every other arm link and all native ROV/thruster meshes included.',limitations='Triangle intersections report surface crossings, not signed clearance or containment; no motion changed.',scene_sha256=sha(HERE/'results/final_scene.blend'))
    (OUT/'interference.json').write_text(json.dumps(report,indent=2)+'\n');print('INTERFERENCE',len(issues),'pairs',flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['verify','stills','audit']);a=p.parse_args();OUT.mkdir(exist_ok=True)
    {'verify':verify,'stills':stills,'audit':audit}[a.mode]()
