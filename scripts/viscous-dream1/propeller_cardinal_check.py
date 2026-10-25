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
from propeller_cardinal_scene import CardinalEnv,CENTER,HERE,ASSETS,PAD_Z,surface,blade_point,build_xml

OUT=HERE/'runs/propeller_cardinal'


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
    renderer=mujoco.Renderer(e.model,width=1280,height=720)
    for k in ('front','detail'):
        renderer.update_scene(e.data,camera='cardinal_'+k);Image.fromarray(renderer.render()).save(OUT/f'{prefix}_{k}.png')
    renderer.close()


def setup(e,starts=20):
    ik=local_ik(e)
    p,n=surface(.91)
    leftR=orientation([-1,-.3,0],-n)
    leftp=p-.024*leftR[:,2]-.003*n
    p,n=surface(.47)
    rightR=orientation(-n,[1,0,0]);rightp=p+(PAD_Z+.020+.006)*n
    poses={};q={}
    for s,point,R,opening in [('left',leftp,leftR,.010),('right',rightp,rightR,.017)]:
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


def main():
    # Deliberately invalidate the legacy out-of-repo source; this task must not use it.
    import cross_scenes
    cross_scenes.SCENE=HERE/'does-not-exist.xml'
    manifest=json.loads((ASSETS/'arm/manifest.json').read_text())
    assert all(hashlib.sha256((ASSETS/'arm'/name).read_bytes()).hexdigest()==sha for name,sha in manifest['files'].items())
    root=ET.fromstring(build_xml())
    for node in root.find('asset'):
        if not node.get('file'):continue
        path=Path(node.get('file'))
        directory=root.find('compiler').get('meshdir' if node.tag=='mesh' else 'texturedir','')
        assert (path if path.is_absolute() else Path(directory)/path).is_file(),str(path)
    e=CardinalEnv()
    try:
        m=e.model
        assert m.body('cardinal_propeller').jntnum[0]==0 and m.body('cardinal_stand').jntnum[0]==0
        assert m.body('cardinal_tool').jntnum[0]==1
        assert not any(t==mujoco.mjtEq.mjEQ_WELD for t in m.eq_type)
        assert len([g for g in range(m.ngeom) if m.geom(g).name.startswith('cardinal_blade_') and m.geom_contype[g]])==192
        cardinal=[]
        for b in range(4):
            mesh=trimesh.load(ASSETS/f'blade_{b}.stl',force='mesh')
            assert mesh.is_watertight and mesh.volume>0
            point=blade_point(b,1,0);axis=np.array([[1,0,0],[0,0,1],[-1,0,0],[0,0,-1]][b])
            assert np.linalg.norm(point-.16*axis)<1e-10
            cardinal.append(point.tolist())
        from dream_examples.yam_mujoco.yam_env import build_xml as original_xml
        original=ET.fromstring(original_xml())
        assert root.find("worldbody/geom[@name='table']").attrib==original.find("worldbody/geom[@name='table']").attrib
        assert root.find("worldbody/camera[@name='top']").attrib==original.find("worldbody/camera[@name='top']").attrib
        poses,q=setup(e);initial=contacts(e)
        assert not [c for c in initial if c['distance_m']<-.0001],initial
        for i in range(40):
            for side,opening in [('left',.010),('right',.017)]:q[side][6]=opening/.0524125*max(0.,1-(i+1)/12)
            e.step(q['left'],q['right'])
        ready=contacts(e)
        assert all(any(jaw in c['bodies'] and 'cardinal_propeller' in c['bodies'] for c in ready) for jaw in ('left_gripper_left','left_gripper_right'))
        assert all(any(jaw in c['bodies'] and 'cardinal_handle' in c['geoms'] for c in ready) for jaw in ('right_gripper_left','right_gripper_right'))
        assert any('cardinal_pad' in c['geoms'] and any(n.startswith('cardinal_blade_0_') for n in c['geoms']) for c in ready)
        images=e.images();assert all(im.shape==(360,640,3) for im in images.values())
        checks=dict(inference_calls=0,vendored_arm_hashes_passed=True,no_external_arm_checkout_required=True,table_and_top_camera_identical_to_Dream=True,cardinal_tips_world_relative_m=cardinal,watertight_blades=4,convex_blade_contact_parts=192,no_welds=True,free_tool=True,fixed_hub=True,policy_images={k:list(im.shape) for k,im in images.items()},poses=poses,initial_contacts=initial,ready_contacts=ready)
        demo=json.loads((OUT/'check.json').read_text())
        assert demo['feasibility_passed'],demo
        assert demo['release_tool_drop_m']>.05 and demo['weld_count']==0
        for name,sha in demo['files_sha256'].items():assert hashlib.sha256((HERE/name).read_bytes()).hexdigest()==sha
        checks['scripted_artifact_checks_passed']=True
        (OUT/'geometry_check.json').write_text(json.dumps(checks,indent=2)+'\n')
        print(json.dumps({k:v for k,v in checks.items() if k not in ('ready_contacts','initial_contacts','poses')},indent=2))
    finally:
        e.close()

if __name__=='__main__':main()
