# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Native badge/arm/ROV triangle audit at all recorded 10 Hz episode poses."""
import json,time
from pathlib import Path
import numpy as np,mujoco,trimesh,fcl
from scipy.spatial.transform import Rotation
from rov_scene import make_env,ROOT
OUT=ROOT/'runs/astra_flange/logo_check';ASSETS=ROOT/'astra_op/logo_assets'
e=make_env(True,'backup2');m,d=e.model,e.data;mujoco.mj_forward(m,d)
parts={};badge_groups={};vehicle=[];registration={}
for i in range(m.ngeom):
    name=m.geom(i).name;body=m.body(int(m.geom_bodyid[i])).name
    if m.geom_group[i] not in (2,4) or m.geom_type[i]!=mujoco.mjtGeom.mjGEOM_MESH:continue
    if name.startswith('badge_'):
        key=name.removesuffix('_blue').removesuffix('_white');badge_groups[key]=body
    elif body.startswith(('left_','right_')):key=body
    elif body=='rov_base' or body.startswith('thruster_'):key='vehicle'
    else:continue
    mid=m.geom_dataid[i];v=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]].astype(float)
    faces=m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid]+m.mesh_facenum[mid]].copy()
    v=v@d.geom_xmat[i].reshape(3,3).T+d.geom_xpos[i]
    if key!='vehicle':v=(v-d.body(body).xpos)@d.body(body).xmat.reshape(3,3)
    parts.setdefault(key,[]).append((v,faces))
    if name in ('native_left_forearm_link_00','native_right_forearm_link_00'):
        arm=body.split('_')[0];target=trimesh.Trimesh(v,faces,process=False);T=np.array(json.loads((ASSETS/'registration.json').read_text())['arms'][arm]['source_preview_arm_to_link_mm']);plates={}
        for spec in json.loads((ASSETS/'source_brackets.json').read_text()):
            xyz=(np.array(spec['vertices'])@T[:3,:3].T+T[:3,3])*.001
            _,dist,_=trimesh.proximity.closest_point(target,xyz);dist*=1000
            plates[spec['name']]=dict(rms_mm=float(np.sqrt(np.mean(dist*dist))),max_mm=float(dist.max()))
        registration[arm]=dict(body=body,geom=name,source_bracket_residuals=plates)
objects={}
for key,pieces in parts.items():
    vv=[];ff=[];count=0
    for v,fa in pieces:vv.append(v);ff.append(fa+count);count+=len(v)
    v=np.concatenate(vv);fa=np.concatenate(ff).astype(np.int32)
    bvh=fcl.BVHModel();bvh.beginModel(len(v),len(fa));bvh.addSubModel(v,fa);bvh.endModel();objects[key]=fcl.CollisionObject(bvh)
request=fcl.CollisionRequest(num_max_contacts=1,enable_contact=True);issues={};start=time.monotonic()
cap=np.load(ROOT/'runs/astra_flange/astra_rov_backup2_g3_20261026_001/video_states.npz')
for index,(q,t) in enumerate(zip(cap['qpos'],cap['time'])):
    d.qpos[:]=q;mujoco.mj_kinematics(m,d)
    for key,obj in objects.items():
        if key=='vehicle':continue
        body=badge_groups.get(key,key);b=d.body(body);obj.setTransform(fcl.Transform(b.xmat.reshape(3,3),b.xpos))
    for badge,body in badge_groups.items():
        for other,obj in objects.items():
            if other==body or other in badge_groups:continue
            if fcl.collide(objects[badge],obj,request,fcl.CollisionResult()):
                issues.setdefault(badge+' / '+other,[]).append(dict(capture=index,time_s=float(t)))
    if index%1000==0:print('SIM_BADGE_AUDIT',index,'elapsed',round(time.monotonic()-start,1),flush=True)
report=dict(captured_poses=len(cap['time']),capture_rate_hz=10,physics_steps=0,intersections=issues,registration=registration,method='FCL triangle BVHs using compiled native visual mesh vertices transformed to each body. Each badge checked against all other arm bodies and ROV/thrusters at every captured episode pose; own mounting body excluded. Surface crossings only; containment and motion between 10 Hz samples are not certified.')
(OUT/'interference.json').write_text(json.dumps(report,indent=2)+'\n');e.close();print('SIM_INTERFERENCE',len(issues),flush=True)
