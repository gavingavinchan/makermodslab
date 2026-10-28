"""Local geometric and renderer checks for the visual-only thread build."""
from pathlib import Path
import hashlib
import json
import os
import sys
import xml.etree.ElementTree as ET

os.environ.setdefault('MUJOCO_GL','egl')
ROOT=Path(__file__).resolve().parents[2]; RUN=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
import shackle_assets as s
import mujoco
import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont
from render_threads import set_pose, capture, write_scene


def validate():
    result={}
    hashes=json.loads((RUN/'collision_hashes_before.json').read_text())
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    result['frozen_collision']={'files_identical_sha256':len(hashes),'all_identical':True}
    body=trimesh.load_mesh(s.ASSETS/'shackle_body.stl'); pin=trimesh.load_mesh(s.ASSETS/'shackle_pin.stl')
    for label,m in [('body',body),('pin',pin)]:
        assert m.is_watertight and m.is_winding_consistent and m.volume>0
        assert len(m.split())==1
        result[label]={'faces':len(m.faces),'watertight':bool(m.is_watertight),'components':1}
    seated=(s.EAR_OUTER_SPAN/2-s.PIN_UNDERHEAD_LENGTH)*.001
    pitch=s.THREAD_PITCH*.001
    rows=[]
    # Actual exported triangle solids, including the unthreaded shank/collar,
    # at non-integer turn offsets. Intersection checks catch phase/sign errors.
    offsets=[0,.137*pitch,.5*pitch,.87*pitch,1.3*pitch,s.EAR_THICKNESS*.0005,
             s.EAR_THICKNESS*.001+.0008]
    for remaining in offsets:
        p=pin.copy(); angle=2*np.pi*remaining/pitch
        p.apply_transform(trimesh.transformations.rotation_matrix(angle,[0,0,1]))
        p.apply_translation([0,0,seated+remaining])
        common=trimesh.boolean.intersection([body,p],engine='manifold')
        volume=abs(float(common.volume))*1e9
        # The inherited bow neck reaches 0.0295 mm above the clear ear's
        # nominal seating plane. Separate that collar contact from the tapped
        # ear, whose actual triangle solids must have NO intersection at all.
        below_midpoint=common.triangles_center[:,2]<0
        assert not np.any(below_midpoint),('thread interpenetration',remaining,volume)
        if volume>1e-4:
            assert remaining==0 and common.bounds[0,2]>.0399249 and common.bounds[1,2]<.039955
        rows.append({'remaining_mm':remaining*1000,'spin_rad':angle,
                     'tapped_ear_intersection_mm3':0.0,'collar_seating_overlap_mm3':volume})
        print('Triangle intersection',rows[-1],flush=True)
    result['exported_solid_engagement']=rows
    wrong=pin.copy();wrong.apply_transform(trimesh.transformations.rotation_matrix(np.pi,[0,0,1]))
    wrong.apply_translation([0,0,seated])
    control=trimesh.boolean.intersection([body,wrong],engine='manifold')
    assert control.volume*1e9>1
    result['wrong_half_turn_positive_control_intersection_mm3']=float(control.volume*1e9)
    # Dense profile test throughout engagement, and a second diameter/pitch/phase
    # without editing source or files. Both sides must derive from the same inputs.
    original=(s.THREAD_MAJOR_DIAMETER,s.THREAD_PITCH,s.THREAD_START_ANGLE)
    variants=[]
    try:
        for diameter,pp,phase in [original,(20.2,2.54,.731)]:
            s.THREAD_MAJOR_DIAMETER=diameter;s.THREAD_PITCH=pp;s.THREAD_START_ANGLE=phase
            z=np.linspace(-s.EAR_OUTER_SPAN/2+.02,-s.EAR_INNER_GAP/2-.02,400)[:,None]
            angle=np.linspace(0,2*np.pi,361)[None,:]
            seated_mm=seated*1000
            gaps=[]
            for offset in np.linspace(0,s.EAR_THICKNESS,23):
                local_z=z-seated_mm-offset
                occupied=(local_z>=0)&(local_z<=s.PIN_THREAD_LENGTH)
                male=s.thread_radius(local_z,angle-2*np.pi*offset/pp)
                female=s.thread_radius(z,angle,internal=True)
                gap=np.broadcast_to(female-male,(400,361))
                mask=np.broadcast_to(occupied,gap.shape)
                if mask.any():gaps.append(float(gap[mask].min()))
            assert min(gaps)>=s.THREAD_RADIAL_CLEARANCE-1e-9
            m=s.helical_solid(0,s.PIN_THREAD_LENGTH)
            b=s.threaded_bore()
            assert m.is_watertight and b.is_watertight
            variants.append({'major_mm':diameter,'pitch_mm':pp,'phase_rad':phase,
                             'minimum_profile_radial_gap_mm':min(gaps),
                             'pin_helix_watertight':bool(m.is_watertight),
                             'female_tool_watertight':bool(b.is_watertight)})
    finally:s.THREAD_MAJOR_DIAMETER,s.THREAD_PITCH,s.THREAD_START_ANGLE=original
    result['parametric_variants']=variants
    path=write_scene(); model=mujoco.MjModel.from_xml_path(str(path)); data=mujoco.MjData(model)
    renderer=mujoco.Renderer(model,height=720,width=1280)
    normal_check={}; pictures=[]
    try:
        set_pose(model,data,pin_only=True,loose=True)
        pictures.append(capture(renderer,model,data,'pin_thread_closeup'))
        for label in ['body','pin']:
            mid=model.mesh(f'shackle_{label}').id
            start=model.mesh_faceadr[mid]; count=model.mesh_facenum[mid]
            fn=model.mesh_facenormal[start:start+count]
            assert np.array_equal(fn,np.arange(count*3).reshape(-1,3))
            normal_start=model.mesh_normaladr[mid]; normal_count=model.mesh_normalnum[mid]
            nn=model.mesh_normal[normal_start:normal_start+normal_count]
            assert np.all(np.isfinite(nn))
            assert np.allclose(np.linalg.norm(nn,axis=1),1,atol=1e-5)
            normal_check[label]={'obj_faces':int(count),'obj_normals':int(model.mesh_normalnum[mid]),
                                 'all_obj_per_corner_normal_indices_preserved':True,
                                 'all_normals_finite_and_unit':True}
    finally:renderer.close()
    # Compile the same scene with only the pin's visual loader switched to STL;
    # render with identical pose, light and material for a direct shading check.
    tree=ET.parse(path); node=tree.getroot().find("asset/mesh[@name='shackle_pin']")
    node.set('file',str(s.ASSETS/'shackle_pin.stl'))
    tree.getroot().find('compiler').set('meshdir',str(s.ASSETS))
    stl_model=mujoco.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding='unicode'))
    stl_data=mujoco.MjData(stl_model); rr=mujoco.Renderer(stl_model,height=720,width=1280)
    try:
        set_pose(stl_model,stl_data,pin_only=True,loose=True)
        pictures.append(capture(rr,stl_model,stl_data,'pin_thread_closeup'))
        mid=stl_model.mesh('shackle_pin').id
        normal_check['pin']['stl_normals']=int(stl_model.mesh_normalnum[mid])
        normal_check['pin']['stl_vertices']=int(stl_model.mesh_vertnum[mid])
        assert stl_model.mesh_normalnum[mid]==stl_model.mesh_vertnum[mid]
    finally:rr.close()
    im=Image.new('RGB',(2560,768),'#1b2835'); draw=ImageDraw.Draw(im)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',24)
    for i,(pic,label) in enumerate(zip(pictures,['OBJ: independent corner normals (default)','STL: MuJoCo generated vertex normals'])):
        im.paste(Image.fromarray(pic),(1280*i,48));draw.text((1280*i+20,12),label,font=font,fill='white')
    im.save(RUN/'normals_obj_vs_stl.png')
    result['mujoco_normals']=normal_check
    poses_path=RUN/'animation_poses.json'
    if poses_path.exists():
        poses=json.loads(poses_path.read_text()); engaged=[r for r in poses if r['time_s']>=2.15]
        error=max(abs(r['tip_z_m']-seated-pitch*r['spin_about_plus_z_rad']/(2*np.pi)) for r in engaged)
        assert error<1e-12
        result['animation']={'frames':len(poses),'fps':30,'duration_s':len(poses)/30,
                             'maximum_engaged_constraint_error_m':error}
    result['mujoco_version']=mujoco.__version__
    (RUN/'thread_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':validate()
