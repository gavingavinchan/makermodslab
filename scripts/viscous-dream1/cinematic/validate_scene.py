# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""CAD, native FK, joint limits, contact mask and sampled geometry audit."""
from pathlib import Path
import hashlib,json,sys,bpy,numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree
HERE=Path(__file__).parent;OUT=HERE/'results'
sys.path.insert(0,str(HERE/'vendor'))
from bimanual_common import fk,NAMES,JD,JT
from surface import surface

def signature():
    result={}
    for o in bpy.context.scene.objects:
        if o.type!='MESH':continue
        xyz=np.empty(len(o.data.vertices)*3,dtype=np.float32);o.data.vertices.foreach_get('co',xyz)
        result[o.name]=dict(vertices=len(o.data.vertices),polygons=len(o.data.polygons),sha256=hashlib.sha256(xyz.tobytes()).hexdigest(),parent=o.parent.name if o.parent else None,basis=np.array(o.matrix_basis).tolist())
    return result
bpy.ops.wm.open_mainfile(filepath=str(HERE/'assets/native_rov.blend'));original=signature()
mounts={n:np.array(bpy.data.objects[n+'_origin'].matrix_basis).copy() for n in ['left_mount','right_mount']}
original_joint_frames={n:np.array(bpy.data.objects[n+'_origin'].matrix_basis).copy() for n in JT}
bpy.ops.wm.open_mainfile(filepath=str(OUT/'final_scene.blend'));s=bpy.context.scene;current=signature()
differences=[k for k,v in original.items() if current.get(k)!=v]
mount_delta=max(float(abs(np.array(bpy.data.objects[n+'_origin'].matrix_basis)-v).max()) for n,v in mounts.items())
urdf_rig_error=max(float(abs(original_joint_frames[n]-JT[n]).max()) for n in JT)
motion=json.loads((OUT/'motion.json').read_text());M=np.array(motion['rov_transform']);limit_excursion=0;pad_error=0;blender_fk_error=0;speed=0
for prev,rec in zip([motion['frames'][0]]+motion['frames'][:-1],motion['frames']):
    for name,q in rec['joints'].items():
        lim=JD[name].find('limit');limit_excursion=max(limit_excursion,float(lim.get('lower'))-q,q-float(lim.get('upper')))
        speed=max(speed,abs(q-prev['joints'][name])*24)
    T=M@fk('right',[rec['joints'][n] for n in NAMES['right']]);actual=T[:3,3]+T[:3,2]*.101
    pad_error=max(pad_error,float(np.linalg.norm(actual-rec['pad'])))
# Sampled surface BVH intersections; exclude mechanically adjacent links.
cache={}
def worldmesh(o):
    eval=o.evaluated_get(bpy.context.evaluated_depsgraph_get());mesh=eval.to_mesh()
    v=[eval.matrix_world@x.co for x in mesh.vertices];f=[list(x.vertices) for x in mesh.polygons]
    eval.to_mesh_clear();return v,f

def tree(o):
    key=o.name
    static=key in original and original[key]['parent']=='rov_base'
    if static and key in cache:return cache[key]
    v,f=worldmesh(o);xyz=np.array(v);result=(BVHTree.FromPolygons(v,f,all_triangles=False),xyz.min(0),xyz.max(0))
    if static:cache[key]=result
    return result
collisions=[];finger_clearances=[];sample_frames=[1,73,150,240,350,450,550,601,672]
# Native body stays static, native movable links are audited as full visible CAD.
body=[o for o in s.objects if o.name in original and o.parent.name=='rov_base']
arms={side:[o for o in s.objects if o.name in original and o.name.startswith(side+'_') and o.parent.name not in [side+'_base_link',side+'_yaw_link']] for side in ['left','right']}
blade=bpy.data.objects['Propeller / exact corrected blade 0'];bt=tree(blade)
for f in sample_frames:
    s.frame_set(f);rec=motion['frames'][f-1]
    for side in ['left','right']:
        expected=M@fk(side,[rec['joints'][n] for n in NAMES[side]])
        blender_fk_error=max(blender_fk_error,float(abs(np.array(bpy.data.objects[side+'_tool0'].matrix_world)-expected).max()))
    trees={o.name:tree(o) for o in arms['left']+arms['right']+body}
    pairs=[(a,b,'arm_body') for side in arms for a in arms[side] for b in body]+[(a,b,'left_right') for a in arms['left'] for b in arms['right']]
    for a,b,kind in pairs:
        ta,la,ha=trees[a.name];tb,lb,hb=trees[b.name]
        if np.any(ha<lb) or np.any(hb<la):continue
        overlap=ta.overlap(tb)
        if overlap:collisions.append(dict(frame=f,kind=kind,a=a.name,b=b.name,triangle_pairs=len(overlap)))
    if f==350:
        for side in ['left','right']:
            T=bpy.data.objects[side+'_tool0'].matrix_world.inverted()
            for jaw in ['left','right']:
                objs=[o for o in arms[side] if o.parent.name==side+'_gripper_'+jaw]
                vv=[v for o in objs for v in worldmesh(o)[0]]
                local=np.array([T@v for v in vv]);band=local[(abs(local[:,2])<.034)]
                if side=='right':finger_clearances.append(dict(side=side,jaw=jaw,grip_band_min_radial_distance_m=float(np.linalg.norm(band[:,:2],axis=1).min()) if len(band) else None))
                else:
                    near=[bt[0].find_nearest(v)[3] for v in vv];finger_clearances.append(dict(side=side,jaw=jaw,nearest_vertex_blade_distance_m=float(min(x for x in near if x is not None)),tcp_local_bounds=[local.min(0).tolist(),local.max(0).tolist()]))
    print('AUDIT_FRAME',f,flush=True)
mask=np.load(OUT/'texture_mask_evidence.npz');points=mask['points'];times=mask['clean_at'];bad=0;max_radius=0
for f in np.unique(times[times<10000]).astype(int):
    rec=motion['frames'][f-1];assert rec['contact_duty']>=.999999
    delta=points[times==f]-rec['pad'];plane=delta@np.array(rec['normal']);rad=np.sqrt(np.maximum(0,np.einsum('ij,ij->i',delta,delta)-plane*plane));max_radius=max(max_radius,float(rad.max()))
    bad+=int(np.count_nonzero((rad>.026000001)|(abs(plane)>.003000001)))
contact=[r for r in motion['frames'] if r['contact_duty']>=.999999]
radial=np.array([np.hypot(r['pad'][0],r['pad'][2]) for r in contact])
deviation=[np.linalg.norm(np.array(r['pad'])-surface(r['source_t'],r['source_u'])[0]) for r in contact]
residual=[np.linalg.norm(np.array(r['pad'])-surface(r['t'],r['u'])[0]) for r in contact]
margins={n:min(min(r['joints'][n]-float(JD[n].find('limit').get('lower')),float(JD[n].find('limit').get('upper'))-r['joints'][n]) for r in motion['frames']) for side in NAMES for n in NAMES[side]}
peak_step=max(abs(b['joints'][n]-a['joints'][n]) for a,b in zip(motion['frames'],motion['frames'][1:]) for side in NAMES for n in NAMES[side])
grid=points.reshape(*mask['shape'],3);cell_area=np.linalg.norm(np.cross(np.gradient(grid,axis=1),np.gradient(grid,axis=0)),axis=2)
unique_area=float(np.sum(cell_area[times.reshape(mask['shape'])<10000]))
report=dict(original_mesh_instances=len(original),mesh_parent_local_transform_differences=differences,native_mount_transform_difference=mount_delta,urdf_to_native_rig_joint_origin_max_difference=urdf_rig_error,mount_separation_m=float(np.linalg.norm(mounts['left_mount'][:3,3]-mounts['right_mount'][:3,3])),native_thruster_joints=len([n for n in JD if n.startswith('thruster_')]),max_joint_limit_excursion=limit_excursion,max_joint_speed_rad_s=speed,max_actual_native_FK_pad_error_m=pad_error,max_Blender_to_native_FK_matrix_error=blender_fk_error,sampled_frames=sample_frames,sampled_surface_intersections=collisions,finger_contact_checks=finger_clearances,mask_outside_nominal_contact_footprint_pixels=bad,mask_max_radius_m=max_radius,mask_scope='Nominal compliant 52 mm face footprint with ±3mm normal tolerance; cinematic visual approximation, not exact bench ellipsoid intersection or material abrasion physics',source_rig_sha256=hashlib.sha256((HERE/'assets/native_rov.blend').read_bytes()).hexdigest(),final_blend_bytes=(OUT/'final_scene.blend').stat().st_size,external_unpacked_images=[i.name for i in bpy.data.images if i.source=='FILE' and not i.packed_file],physics_scope=motion['physics_claim'])
allowed=np.load(HERE/'assets/allowed_nominal_mask.npz')['allowed'].ravel()
report['mask_outside_exported_nominal_union_pixels']=int(np.count_nonzero((times<10000)&~allowed))
report.update(source_contract_sha256=motion['original_contract_sha256'],tool_asset_sha256=hashlib.sha256((HERE/'tool/polisher.blend').read_bytes()).hexdigest(),static_vehicle_transform=M.tolist(),vehicle_forward_up_angle_deg=float(np.degrees(np.arcsin(M[2,0]))),minimum_joint_limit_margins_rad=margins,minimum_joint_limit_margin_rad=min(margins.values()),peak_joint_step_rad=peak_step,peak_joint_speed_rad_s=peak_step*24,max_pad_to_retargeted_surface_error_m=max(residual),max_observed_to_native_surface_path_deviation_m=max(deviation),actual_native_radial_span_m=float(np.ptp(radial)),actual_native_u_range=[min(r['u'] for r in contact),max(r['u'] for r in contact)],rendered_unique_nominal_area_m2=unique_area,nominal_face_area_m2=float(cell_area.sum()),native_tool_mount_translation_m=[0,0,.020],native_TCP_to_pad_contact_m=[0,0,.101],right_jaw_displacement_m=.012,left_jaw_displacement_m=.0008,retarget_adaptation='u = .13 + .60*(observed_u-.13); observed radial t shifted inward by .003 (1.185mm), preserving full radial span; four distinct offset alternating passes; pad roll selected for native limits; normal aligned to same corrected analytic face; 25.5s source scrub retimed to22s')
pose=json.loads((HERE/'selected_pose.json').read_text())
report['vehicle_roll_deg']=float(np.degrees(np.arctan2(M[2,1],M[2,2])))
report['vehicle_up_axis_error']=float(np.linalg.norm(M[:3,2]-[0,0,1]))
report['retarget_adaptation']=dict(chord_scale=pose.get('chord_scale',1),chord_pivot=pose.get('chord_pivot',0),radial_offset_t=pose.get('radial_offset_t',0),free_pad_axis_roll=pose.get('free_pad_roll',False),left_approach_adjustment_rad=pose.get('left_approach_adjustment_rad',0),initial_gap_m=pose.get('initial_gap_m',.018))
report['retarget_adaptation']['stripe_motion']=motion.get('continuous_path')
continuity=json.loads((OUT/'motion_continuity.json').read_text())
assert continuity['passed'] and continuity['scene_sha256']==hashlib.sha256((OUT/'final_scene.blend').read_bytes()).hexdigest()
report['saved_native_motion_continuity']=continuity
ship_provenance=json.loads((HERE/'assets/ship/provenance.json').read_text())
for filename,expected in ship_provenance['files'].items():
    assert hashlib.sha256((HERE/'assets/ship'/filename).read_bytes()).hexdigest()==expected
report['ship_CAD']=dict(source=ship_provenance['source_page'],uniform_scale=ship_provenance['adaptation']['uniform_scale'],meshes=[o.name for o in s.objects if o.name.startswith('Ship / imported')],source_files_verified=True)
report['core_checks_passed']=not differences and not collisions and mount_delta<1e-9 and urdf_rig_error<2e-7 and limit_excursion<1e-9 and pad_error<1e-7 and max(residual)<1e-5 and blender_fk_error<1e-6 and bad==0 and report['mask_outside_exported_nominal_union_pixels']==0
if pose.get('require_horizontal'):report['core_checks_passed'] &= report['vehicle_up_axis_error']<1e-12
# Verify that the saved shader carries the audited first-touch image and a
# continuously increasing frame clock; no material reset can hide in a cut.
image=bpy.data.images['Exact accumulated contact time']
pixels=np.empty(len(image.pixels),dtype=np.float32);image.pixels.foreach_get(pixels)
image_error=float(np.max(abs(pixels.reshape(-1,4)[:,0]*10000-times)))
clock=bpy.data.materials['Blade 0 | exact high-resolution footprint clock'].node_tree.nodes['Cinematic frame']
clock_error=0;camera_aim_error=0;camera_up_error=0
from camera_sequence import CAMERA_TARGETS
camera_targets=CAMERA_TARGETS
for f in range(1,s.frame_end+1):
    s.frame_set(f);clock_error=max(clock_error,abs(clock.outputs[0].default_value-f))
    cam=s.camera;C=np.array(cam.matrix_world);expected=np.array(camera_targets[cam.name])-C[:3,3];expected/=np.linalg.norm(expected)
    actual=-C[:3,2];camera_aim_error=max(camera_aim_error,float(np.arccos(np.clip(expected@actual,-1,1))))
    up=np.array([0.,0.,1.]);up-=expected*(expected@up);up/=np.linalg.norm(up)
    camera_up_error=max(camera_up_error,float(np.arccos(np.clip(up@C[:3,1],-1,1))))
report.update(saved_mask_image_max_time_error_frames=image_error,saved_shader_clock_max_error_frames=clock_error,clean_mask_frame_range=[float(times.min()),float(times[times<10000].max())],packed_image_verified=bool(image.packed_file),final_scene_sha256=hashlib.sha256((OUT/'final_scene.blend').read_bytes()).hexdigest())
report.update(max_active_camera_aim_error_deg=float(np.degrees(camera_aim_error)),max_active_camera_up_error_deg=float(np.degrees(camera_up_error)))
report['core_checks_passed'] &= camera_aim_error<.002 and camera_up_error<.002
assert image_error<.01 and clock_error<.001 and image.packed_file
expected_hashes={'assets/coverage_plan.json':'b18d8c4e36ad3d784e0d4e498b34cf3c4bfb0410bec9e2fc9d1c628dc98df7db','assets/native_rov.blend':'5eeb60fba40c2f6fc5e4f6744a9f94274fb8e361310a61aa9e573cf82ad721ef','tool/polisher.blend':'5e5f76d54c99232f463a3004f1babd84705d8ff6bac6b4582cc9245306124b87'}
# Frozen source identities are task inputs, not dynamically accepted checksums.
report['frozen_input_hashes']={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in expected_hashes}
assert report['frozen_input_hashes']==expected_hashes
mask_provenance=json.loads((HERE/'assets/allowed_mask_provenance.json').read_text())
assert mask_provenance['sha256']=='d42f26c86538f776895291a761205d83b810189db84ee2f10d235e29ff528972'
report['exported_source_mask_sha256']=mask_provenance['sha256']
peak_frame=max(zip(motion['frames'],motion['frames'][1:]),key=lambda pair:max(abs(pair[1]['joints'][n]-pair[0]['joints'][n]) for side in NAMES for n in NAMES[side]))[1]['frame']
selection=dict(outermost_native_pad_frame=contact[int(np.argmax(radial))]['frame'],largest_source_to_native_deviation_frame=contact[int(np.argmax(deviation))]['frame'],peak_joint_step_frame=peak_frame,interpretation='Outermost native contact, maximum deviation from the corresponding source arc-parameter sample, and largest native joint step. The continuous path changes geometry and timing; these are not dynamics measurements.')
(OUT/'evidence_selection.json').write_text(json.dumps(selection,indent=2)+'\n')
(OUT/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));assert report['core_checks_passed']
