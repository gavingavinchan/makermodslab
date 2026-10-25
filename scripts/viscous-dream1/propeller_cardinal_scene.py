# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Independent propeller_cardinal environment. No clients, policy or hardware.

World +X is LEFT from the arms (+Y) looking at the work (-Y); +Z is up.
The target is blade 0 (+X). Four swept blade centrelines end on cardinal axes.
Reuses B0 dynamics/camera pipeline; arm source is vendored, packages read-only.
"""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh
from dream_examples.yam_mujoco import yam_env
from cross_scenes import ViscousAppleEnv, JOINTS, OPEN
from supported_arms import wrist_mount

HERE = Path(__file__).resolve().parent
ASSETS = HERE / 'assets/propeller_cardinal'
CENTER = np.array([-.10, -.12, .97])
RADIUS = .160
TABLE_Z = .760
PAD_Z = .075
TASK = 'propeller_cardinal'
PROMPT = 'Hold the outer edge of the left-pointing propeller blade with the left gripper and scrub that same blade face back and forth with the circular pad in the right gripper.'


def numbers(v):
    return ' '.join(f'{x:.10g}' for x in np.asarray(v).ravel())


def blade_point(blade, t, u, h=0):
    angle = blade * np.pi / 2
    radial = np.array([np.cos(angle), 0., np.sin(angle)])
    tangent = np.array([-np.sin(angle), 0., np.cos(angle)])
    radius = .029 + (RADIUS-.029)*t
    width = .017 + .035*np.sin(np.pi*t)**.70
    # Rounded tip, explicit endpoint centred on the cardinal axis.
    width *= 1-.30*t**12
    radius -= .012*u*u*t**10
    sweep = .019*np.sin(np.pi*t)*t
    pitch = np.deg2rad(-30+14*t)
    camber = .0035*np.sin(np.pi*t)*(1-u*u)
    return radial*radius + tangent*(sweep+u*width*np.cos(pitch)) + np.array([0.,u*width*np.sin(pitch)+camber+h,0.])


def surface(t, u=0):
    p=blade_point(0,t,u,.003)
    dt=(blade_point(0,t+1e-5,u)-blade_point(0,t-1e-5,u))/2e-5
    du=(blade_point(0,t,u+1e-5)-blade_point(0,t,u-1e-5))/2e-5
    normal=np.cross(du,dt);normal/=np.linalg.norm(normal)
    if normal[1]<0:normal=-normal
    return CENTER+p, normal


def generate_assets():
    ASSETS.mkdir(parents=True,exist_ok=True)
    # Fine visual mesh; stripwise convex collision hulls preserve pitch/curvature.
    for blade in range(4):
        v=np.array([[[blade_point(blade,t,u,h) for u in np.linspace(-1,1,13)] for h in (-.003,.003)] for t in np.linspace(0,1,33)])
        faces=[]
        def idx(i,h,j):return i*26+h*13+j
        def quad(a,b,c,d):faces.extend([[a,b,c],[a,c,d]])
        for i in range(32):
            for h in range(2):
                for j in range(12):quad(idx(i,h,j),idx(i+1,h,j),idx(i+1,h,j+1),idx(i,h,j+1))
            for j in (0,12):quad(idx(i,0,j),idx(i+1,0,j),idx(i+1,1,j),idx(i,1,j))
        for i in (0,32):
            for j in range(12):quad(idx(i,0,j),idx(i,0,j+1),idx(i,1,j+1),idx(i,1,j))
        mesh=trimesh.Trimesh(v.reshape(-1,3),faces,process=True);mesh.fix_normals();mesh.export(ASSETS/f'blade_{blade}.stl')
        for i in range(16):
            for j in range(3):
                trimesh.convex.convex_hull(v[i*2:i*2+3,:,j*4:j*4+5].reshape(-1,3)).export(ASSETS/f'blade_{blade}_{i}_{j}.stl')


def add_scene(root):
    if not (ASSETS/'blade_3.stl').exists():generate_assets()
    world,asset=root.find('worldbody'),root.find('asset')
    for name in ('apple','orange','bowl','banana','water_bottle','plate'):
        node=world.find(f"body[@name='{name}']")
        if node is not None:world.remove(node)
    ET.SubElement(asset,'material',name='cardinal_bronze',rgba='.63 .39 .14 1',specular='.65',shininess='.65')
    prop=ET.SubElement(world,'body',name='cardinal_propeller',pos=numbers(CENTER))
    def geom(body,name,**kw):return ET.SubElement(body,'geom',name=name,**{k:str(v) for k,v in kw.items()})
    for b in range(4):
        name=f'cardinal_blade_{b}'
        ET.SubElement(asset,'mesh',name=name,file=str(ASSETS/f'blade_{b}.stl'),smoothnormal='true')
        geom(prop,name+'_visual',type='mesh',mesh=name,material='cardinal_bronze',contype=0,conaffinity=0,group=2)
        for i in range(16):
            for j in range(3):
                n=f'{name}_{i}_{j}'
                ET.SubElement(asset,'mesh',name=n,file=str(ASSETS/f'blade_{b}_{i}_{j}.stl'))
                geom(prop,n,type='mesh',mesh=n,rgba='0 0 0 0',group=3,friction='1 .01 .001')
    geom(prop,'cardinal_hub',type='cylinder',size='.034 .038',euler=f'{np.pi/2} 0 0',material='cardinal_bronze')
    geom(prop,'cardinal_nose',type='ellipsoid',pos='0 .038 0',size='.033 .022 .033',material='cardinal_bronze')
    stand=ET.SubElement(world,'body',name='cardinal_stand',pos=numbers(CENTER))
    geom(stand,'cardinal_base',type='box',pos=numbers([0,-.07,TABLE_Z+.009-CENTER[2]]),size='.065 .06 .009',rgba='.17 .20 .23 1')
    height=CENTER[2]-TABLE_Z-.018
    geom(stand,'cardinal_post',type='box',pos=numbers([0,-.1,-height/2]),size=numbers([.015,.015,height/2]),rgba='.25 .29 .32 1')
    geom(stand,'cardinal_shaft',type='cylinder',size='.013 .061',pos='0 -.065 0',euler=f'{np.pi/2} 0 0',rgba='.6 .63 .65 1')
    for y in (-.095,-.073):geom(stand,'cardinal_bearing_'+str(y),type='cylinder',pos=f'0 {y} 0',size='.025 .007',euler=f'{np.pi/2} 0 0',rgba='.20 .23 .25 1')
    tool=ET.SubElement(world,'body',name='cardinal_tool',pos='.18 .02 .785',euler=f'{np.pi/2} 0 0')
    ET.SubElement(tool,'freejoint',name='cardinal_tool_free')
    geom(tool,'cardinal_handle',type='cylinder',size='.016 .035',mass='.07',rgba='.13 .16 .18 1',friction='1.5 .01 .001')
    geom(tool,'cardinal_neck',type='cylinder',pos='0 0 .048',size='.009 .013',mass='.015',rgba='.65 .68 .70 1')
    geom(tool,'cardinal_backing',type='cylinder',pos='0 0 .065',size='.026 .004',mass='.02',rgba='.12 .14 .16 1')
    geom(tool,'cardinal_pad',type='ellipsoid',pos=f'0 0 {PAD_Z}',size='.026 .026 .006',mass='.012',rgba='.17 .28 .25 1',friction='.30 .003 .0001',priority=1,solref='.01 1')
    # Inspection cameras only: existing three policy cameras are untouched.
    def camera(name,eye,target,fovy):
        z=np.asarray(eye)-target;z/=np.linalg.norm(z);x=np.cross([0,0,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
        ET.SubElement(world,'camera',name=name,pos=numbers(eye),xyaxes=numbers(np.r_[x,y]),fovy=str(fovy))
    camera('cardinal_front',[CENTER[0],.51,1.015],CENTER+[0,0,-.025],62)
    camera('cardinal_detail',[.31,.48,1.055],CENTER+[.065,0,0],37)
    root.find('visual/global').set('offwidth','1280');root.find('visual/global').set('offheight','720')
    return root


def build_xml():
    # Same B0 construction as cross_scenes.viscous_xml, with a repo-local source.
    root=ET.fromstring(yam_env.build_xml());source=ET.parse(ASSETS/'arm/source.xml').getroot()
    root.find('option').set('timestep','.001')
    world=root.find('worldbody');bases={s:world.find(f"body[@name='{s}_base']") for s in ('left','right')}
    for body in bases.values():world.remove(body)
    root.find('contact').clear();root.find('actuator').clear();ET.SubElement(root,'equality')
    default=copy.deepcopy(source.find('default'));default.set('class','viscous');root.find('default').append(default)
    for asset in source.find('asset'):
        asset=copy.deepcopy(asset)
        if asset.get('file'):asset.set('file',str(ASSETS/'arm'/asset.get('file')))
        root.find('asset').append(asset)
    for side in ('left','right'):
        names={n.get('name'):side+'_'+n.get('name') for n in source.iter() if n.get('name')}
        names.update({n:f'{side}_joint{i}' for i,n in enumerate(JOINTS,1)})
        names.update(gripper_left_joint=side+'_finger_-1',gripper_right_joint=side+'_finger_1',tool_center=side+'_tcp')
        def renamed(element):
            element=copy.deepcopy(element)
            for node in element.iter():
                for key in ('name','joint','joint1','joint2','body1','body2'):
                    if node.get(key):node.set(key,names[node.get(key)])
            return element
        arm=renamed(source.find("worldbody/body[@name='base_link']"))
        arm.set('pos',numbers([.22 if side=='left' else -.22,.24,.772]));arm.set('quat',bases[side].get('quat'));arm.set('childclass','viscous')
        arm.find(f".//site[@name='{side}_tcp']").set('rgba','0 0 0 0')
        eye,R=wrist_mount()
        ET.SubElement(arm.find(f".//body[@name='{side}_tool0']"),'camera',name=side,pos=numbers(eye),xyaxes=numbers(np.r_[R[:,0],R[:,1]]),fovy='77.2')
        world.append(arm)
        for section in ('contact','equality','actuator'):
            for element in source.find(section):root.find(section).append(renamed(element))
    # Task-local grip force: below the flange CAD-linkage ideal 160 N bound.
    # 40 N keeps the cylindrical handle from pivoting/slipping under scrub load.
    for side in ('left','right'):
        root.find(f"actuator/position[@name='{side}_gripper_left_joint_drive']").set('forcerange','-10 10' if side=='left' else '-40 40')
    return ET.tostring(add_scene(root),encoding='unicode')


class CardinalEnv(ViscousAppleEnv):
    def __init__(self):
        self.wrist='maker';self.sides=('left','right')
        self.model=mujoco.MjModel.from_xml_string(build_xml());self.data=mujoco.MjData(self.model)
        self.renderer=self.wrist_renderer=None;self.qadr={};self.vadr={};self.aids={}
        for s in self.sides:
            joints=[self.model.joint(f'{s}_joint{i}') for i in range(1,7)]
            self.qadr[s]=np.array([j.qposadr[0] for j in joints]);self.vadr[s]=np.array([j.dofadr[0] for j in joints])
            self.aids[s]=np.array([self.model.actuator(f'{s}_{n}_drive').id for n in JOINTS])
        self.reset()
