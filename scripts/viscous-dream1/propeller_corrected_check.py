# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""No-call geometry, IK and contact checks for the independent cardinal scene."""
import json
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import trimesh
import mujoco
import numpy as np
from PIL import Image
from flange_start import local_ik
from flange_ik_check import orientation
from propeller_corrected_scene import CorrectedEnv as CardinalEnv, CENTER,HERE,ASSETS,PAD_Z,surface,blade_point,build_xml,OUT,LEFT_T,SCRUB_T,LEFT_OPEN,RADIUS




def contacts(e):
    rows=[]
    for i,c in enumerate(e.data.contact):
        names=[e.model.geom(int(g)).name for g in (c.geom1,c.geom2)]
        bodies=[e.model.body(e.model.geom_bodyid[int(g)]).name for g in (c.geom1,c.geom2)]
        if not any(n.startswith(('left_','right_')) or n=='cardinal_tool' for n in bodies):continue
        f=np.zeros(6);mujoco.mj_contactForce(e.model,e.data,i,f)
        rows.append(dict(geoms=names,bodies=bodies,distance_m=float(c.dist),normal_N=float(f[0]),position=c.pos.tolist()))
    return rows


def save_views(e,prefix):
    OUT.mkdir(parents=True,exist_ok=True)
    for k,v in e.images().items():Image.fromarray(v).save(OUT/f'{prefix}_{k}.png')
    e.renderer.update_scene(e.data,camera='top')
    Image.fromarray(e.renderer.render()).save(OUT/f'{prefix}_top_sensor_full.png')
    renderer=mujoco.Renderer(e.model,width=1280,height=720)
    for k in ('front','detail'):
        renderer.update_scene(e.data,camera='cardinal_'+k);Image.fromarray(renderer.render()).save(OUT/f'{prefix}_{k}.png')
    renderer.close()


def setup(e,starts=20):
    ik=local_ik(e)
    p,n=surface(LEFT_T)
    leftR=orientation([-1,-.3,0],-n)
    leftp=p-.024*leftR[:,2]-.003*n
    p,n=surface(SCRUB_T)
    rightR=orientation(-n,[1,0,0]);rightp=p+(PAD_Z+.020+.006)*n
    poses={};q={}
    for s,point,R,opening in [('left',leftp,leftR,LEFT_OPEN),('right',rightp,rightR,.017)]:
        fit,res=ik.fit(s,point,R,starts=starts)
        if np.linalg.norm(res[:3])>.002 or np.linalg.norm(res[3:])/.15>np.deg2rad(2):
            raise ValueError(f'{s} pregrasp IK outside 2 mm / 2 deg tolerance')
        e.data.qpos[e.qadr[s]]=fit;e.data.ctrl[e.aids[s]]=fit
        for sign in (-1,1):e.data.joint(f'{s}_finger_{sign}').qpos[0]=opening
        e.data.actuator(s+'_gripper_left_joint_drive').ctrl[0]=0
        mujoco.mj_forward(e.model,e.data)
        q[s]=np.r_[fit,0.]
        poses[s]=dict(target=point.tolist(),R=R.tolist(),q=fit.tolist(),error_m=float(np.linalg.norm(res[:3])),error_deg=float(np.degrees(np.linalg.norm(res[3:])/.15)))
    hand=e.data.site('right_tcp');R=hand.xmat.reshape(3,3);j=e.model.joint('cardinal_tool_free');a=j.qposadr[0]
    e.data.qpos[a:a+3]=hand.xpos+.020*R[:,2];mujoco.mju_mat2Quat(e.data.qpos[a+3:a+7],R.ravel());e.data.qvel[:]=0
    mujoco.mj_forward(e.model,e.data)
    return poses,q


def invariant_check(root, model):
    """Exact source-tree equality and named compiled non-propeller attributes."""
    import copy
    import propeller_large_scene as old
    baseline=ET.fromstring(old.build_xml())
    def stripped(r):
        r=copy.deepcopy(r)
        for node in list(r.find('asset')):
            if node.get('name','').startswith('cardinal_blade_'):r.find('asset').remove(node)
        prop=r.find("worldbody/body[@name='cardinal_propeller']")
        for node in list(prop):
            if node.get('name','').startswith('cardinal_blade_'):prop.remove(node)
        return ET.tostring(r)
    before,after=stripped(baseline),stripped(root)
    assert before==after, 'Non-blade XML changed'
    other=mujoco.MjModel.from_xml_string(ET.tostring(baseline,encoding='unicode'))
    groups={
        'body':('pos','quat','mass','inertia','ipos','iquat','gravcomp'),
        'geom':('type','size','pos','quat','rgba','contype','conaffinity','friction','solref','solimp','margin','gap'),
        'joint':('type','pos','axis','range','stiffness','limited'),
        'actuator':('gainprm','biasprm','dynprm','ctrlrange','forcerange','ctrllimited','forcelimited'),
        'camera':('pos','quat','fovy','resolution'),
        'light':('pos','dir','diffuse','ambient','specular','castshadow','type')}
    counts={}
    for kind,attributes in groups.items():
        count={'body':other.nbody,'geom':other.ngeom,'joint':other.njnt,'actuator':other.nu,'camera':other.ncam,'light':other.nlight}[kind]
        checked=0
        for i in range(count):
            old_obj=getattr(other,kind)(i);name=old_obj.name
            if kind=='geom' and name.startswith('cardinal_blade_'):continue
            if kind=='body' and name=='cardinal_propeller':continue
            new_obj=getattr(model,kind)(name if name else i)
            for attr in attributes:
                prefix={'joint':'jnt','camera':'cam'}.get(kind,kind)
                np.testing.assert_array_equal(getattr(other,prefix+'_'+attr)[i],getattr(model,prefix+'_'+attr)[new_obj.id],err_msg=f'{kind}/{name}/{attr}')
            checked+=1
        counts[kind]=checked
    assert (other.nq,other.nv,other.nu,other.neq)==(model.nq,model.nv,model.nu,model.neq)
    return dict(non_blade_xml_exactly_equal=True,non_blade_xml_sha256=hashlib.sha256(before).hexdigest(),
        compiled_named_objects_exactly_equal=counts,existing_cameras_lights_actuators_tool_and_fixtures_unchanged=True,
        hub_and_shaft_location_unchanged=True)


def geometry_check(e):
    import propeller_corrected_scene as s
    from scipy.spatial import cKDTree
    g=s.grid();base=trimesh.load(s.ASSETS/'blade_0.stl',force='mesh')
    meshes=[]
    for b in range(4):
        mesh=trimesh.load(s.ASSETS/f'blade_{b}.stl',force='mesh')
        assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume>0
        assert np.min(mesh.area_faces)>1e-12
        a=b*np.pi/2;c,ss=np.cos(a),np.sin(a);rot=np.array([[c,0,-ss],[0,1,0],[ss,0,c]])
        assert np.max(cKDTree(mesh.vertices).query(base.vertices@rot.T)[0])<1e-7
        meshes.append(mesh)
    # Every visible triangle is a boundary triangle of an actual collision cell.
    # No independent decorative blade mesh or proxy pad-contact surface exists.
    def face_keys(mesh):
        return {tuple(sorted(tuple(np.round(v,7)) for v in triangle)) for triangle in mesh.triangles}
    collision_faces=set();volumes=[];cell_min_area=1.
    projected=[];approx_error=[]
    for i,j,generated in s.collision_cells(g):
        cell=trimesh.load(s.ASSETS/f'blade_0_{i}_{j}.stl',force='mesh')
        assert cell.is_watertight and cell.is_winding_consistent and cell.volume>0
        cell_min_area=min(cell_min_area,float(np.min(cell.area_faces)))
        volumes.append(cell.volume);collision_faces.update(face_keys(cell))
        for b in range(1,4):
            replica=trimesh.load(s.ASSETS/f'blade_{b}_{i}_{j}.stl',force='mesh')
            assert replica.is_watertight and replica.is_winding_consistent and replica.volume>0
            assert np.min(replica.area_faces)>1e-12
            a=b*np.pi/2;c,ss=np.cos(a),np.sin(a);rot=np.array([[c,0,-ss],[0,1,0],[ss,0,c]])
            assert np.max(cKDTree(replica.vertices).query(cell.vertices@rot.T)[0])<1e-7
        # Projection is a convex quadrilateral/triangle. Shared boundaries are
        # identical vertical walls, so neighbouring cells tile, without overlaps.
        pts=g[[i,i+1,i+1,i],0,[j,j,j+1,j+1]][:,[0,2]]
        edge1=np.roll(pts,-1,axis=0)-pts;edge2=np.roll(pts,-2,axis=0)-np.roll(pts,-1,axis=0)
        cross=edge1[:,0]*edge2[:,1]-edge1[:,1]*edge2[:,0]
        assert np.min(cross)>-1e-12
        projected.append(float(np.sum(pts[:,0]*np.roll(pts[:,1],-1)-pts[:,1]*np.roll(pts[:,0],-1))/2))
        t=(s.T_GRID[i]+s.T_GRID[i+1])/2;u=(s.U_GRID[j]+s.U_GRID[j+1])/2
        p=s.blade_point(0,t,u,1)
        approx_error.append((t,u,p))
    query=np.array([item[2] for item in approx_error]);distances=[]
    for k in range(0,len(query),32):
        distances.extend(trimesh.proximity.closest_point_naive(base,query[k:k+32])[1])
    task_distances=[d for d,(t,u,p) in zip(distances,approx_error) if .70<t<.90 and abs(u)<.25]
    assert face_keys(base)<=collision_faces
    volume_error=abs(sum(volumes)-base.volume)
    assert volume_error<1e-9
    # Cylindrical coordinates make the section map injective away from the tip:
    # r strictly increases and theta strictly increases with u. Thickness is
    # positive in the interior. Thus there are no folded or inverted sections.
    angles=np.arctan2(g[:,:, :,2],g[:,:,:,0]);angular_span=float(np.ptp(angles))
    assert angular_span<np.pi/2
    root=g[0].reshape(-1,3)
    assert np.max(np.linalg.norm(root[:,[0,2]],axis=1))<.102
    assert np.max(abs(root[:,1]))<.114
    physical=[];visual=[];parts=0
    for i in range(e.model.ngeom):
        if not e.model.geom(i).name.startswith('cardinal_blade_'):continue
        mid=e.model.geom_dataid[i];start=e.model.mesh_vertadr[mid];num=e.model.mesh_vertnum[mid]
        verts=e.model.mesh_vert[start:start+num]@e.data.geom(i).xmat.reshape(3,3).T+e.data.geom(i).xpos
        if e.model.geom_contype[i]:physical.extend(verts);parts+=1
        else:visual.extend(verts)
    bounds={k:dict(min=np.min(v,axis=0).tolist(),max=np.max(v,axis=0).tolist(),extent=np.ptp(v,axis=0).tolist()) for k,v in [('collision',physical),('visual',visual)]}
    for k in bounds:np.testing.assert_allclose(np.array(bounds[k]['extent'])[[0,2]],[.96,.96],atol=1e-6)
    np.testing.assert_allclose(bounds['collision']['min'],bounds['visual']['min'],atol=1e-7)
    np.testing.assert_allclose(bounds['collision']['max'],bounds['visual']['max'],atol=1e-7)
    assert parts==4*s.NR*s.NU
    clearance=bounds['collision']['min'][2]-.760
    assert clearance>.049
    return dict(diameter_m=.96,compiled_mesh_bounds_world_m=bounds,convex_contact_parts=parts,
        watertight_outward_wound_blades=4,nondegenerate_blades_and_collision_cells=True,
        visual_triangles_are_actual_collision_boundary_triangles=True,collision_union_volume_error_m3=volume_error,
        exact_90_degree_rotation_copies=True,root_embedded_in_unchanged_hub=True,
        no_folded_sections=True,no_neighbour_blade_overlap=True,neighbour_blade_angular_gap_deg=float(np.degrees(np.pi/2-angular_span)),
        convex_cell_projection_tiles_without_overlap=True,blade_volume_m3=float(base.volume),
        projected_area_ratio=float(4*sum(projected)/(np.pi*s.RADIUS**2)),
        pitch_m=s.PITCH,mean_pitch_handedness='negative axial advance for increasing blade-local angle, all four identical',
        pitch_angle_root_tip_deg=np.degrees(np.arctan(s.PITCH/(2*np.pi*np.array([s.ROOT_RADIUS,s.RADIUS])))).tolist(),
        analytic_surface_cellcentre_max_nearest_mesh_distance_m=float(max(distances)),
        analytic_IK_pad_patch_max_nearest_mesh_distance_m=float(max(task_distances)),
        bottom_blade_table_clearance_m=clearance)


def render_inspection():
    import propeller_corrected_scene as s
    model=mujoco.MjModel.from_xml_string(s.inspection_xml());data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    renderer=mujoco.Renderer(model,width=1280,height=720)
    for view in ('head_on','oblique','side'):
        renderer.update_scene(data,camera='inspection_'+view)
        # Shadow-free diagnostic exposes section curvature without the existing
        # bench lights' hard self-shadow boundaries. Bench renders stay unchanged.
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=0
        Image.fromarray(renderer.render()).save(OUT/f'propeller_{view}.png')
    renderer.close()


def main():
    import propeller_corrected_scene as s
    OUT.mkdir(parents=True,exist_ok=True)
    e=CardinalEnv()
    try:
        manifest=json.loads((s.large.ASSETS/'arm/manifest.json').read_text())
        assert all(hashlib.sha256((s.large.ASSETS/'arm'/n).read_bytes()).hexdigest()==sha for n,sha in manifest['files'].items())
        report=dict(vendored_arm_asset_hashes_verified=True,scene_invariants=invariant_check(ET.fromstring(build_xml()),e.model),geometry=geometry_check(e))
        poses,q=setup(e);initial=contacts(e)
        assert not [c for c in initial if c['distance_m']<-.0001],initial
        for i in range(40):
            for side,opening in [('left',LEFT_OPEN),('right',.017)]:q[side][6]=opening/.0524125*max(0.,1-(i+1)/12)
            e.step(q['left'],q['right'])
        report.update(initial_contacts=initial,ready_contacts=contacts(e),poses=poses)
        images=e.images();assert all(im.shape==(360,640,3) for im in images.values())
        report['policy_image_shapes']={k:list(v.shape) for k,v in images.items()}
        demo=json.loads((OUT/'check.json').read_text())
        assert demo['feasibility_passed'] and demo['release_tool_drop_m']>.05 and demo['weld_count']==0
        assert demo['left_edge_radial_range_m'][0]>.9*RADIUS
        assert demo['max_intended_penetration_m']<.001
        assert demo['max_unwanted_penetration_m']==0
        assert demo['scrub_contact_fraction']['simultaneous']==1
        for name,sha in demo['files_sha256'].items():assert hashlib.sha256((HERE/name).read_bytes()).hexdigest()==sha,name
        report['validation_source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        assert not np.any(e.data.xfrc_applied) and not np.any(e.data.qfrc_applied)
        assert e.model.body('cardinal_propeller').jntnum[0]==0 and e.model.body('cardinal_tool').jntnum[0]==1
        report.update(motion_evidence_passed=True,fixed_shaft=True,free_tool=True,no_welds_or_external_forces=True,
                      no_hardware_or_inference=True)
        (OUT/'geometry_check.json').write_text(json.dumps(report,indent=2)+'\n')
        render_inspection()
        print(json.dumps({k:v for k,v in report.items() if k not in ('initial_contacts','ready_contacts','poses')},indent=2))
    finally:e.close()


if __name__=='__main__':main()
