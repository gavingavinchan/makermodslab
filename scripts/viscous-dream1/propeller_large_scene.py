# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""960 mm propeller: unchanged arms/pad, explicitly separate bench/presentation.

Scale is applied to ALL blade meshes (visual and 192 collision cells) in MJCF.
The old cardinal module and its baseline dimensions/results are unchanged.
"""
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
import propeller_cardinal_scene as small
from cross_scenes import ViscousAppleEnv, JOINTS

HERE, ASSETS, PAD_Z = small.HERE, small.ASSETS, small.PAD_Z
SCALE = 3.
RADIUS = .480
CENTER = np.array([-.42, -.12, 1.29])
TABLE_Z = small.TABLE_Z
BASE_Z = 1.092
LEFT_T = .97
SCRUB_T = .80
SCRUB_AMPLITUDE = .05
LEFT_OPEN = .024
OUT = HERE / 'runs/propeller_large'
numbers = small.numbers


def blade_point(blade, t, u, h=0):
    return SCALE * small.blade_point(blade, t, u, h / SCALE)


def surface(t, u=0):
    point, normal = small.surface(t, u)
    return CENTER + SCALE * (point-small.CENTER), normal


def camera(world, name, eye, target, fovy):
    old = world.find(f"camera[@name='{name}']")
    if old is not None:
        world.remove(old)
    z = np.asarray(eye, dtype=float)-target
    z /= np.linalg.norm(z)
    x = np.cross([0, 0, 1], z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    ET.SubElement(world, 'camera', name=name, pos=numbers(eye),
                  xyaxes=numbers(np.r_[x, y]), fovy=str(fovy))


def geom(parent, name, **kw):
    return ET.SubElement(parent, 'geom', name=name, **{k: str(v) for k, v in kw.items()})


def build_xml(underwater=False):
    root = ET.fromstring(small.build_xml())
    world, asset = root.find('worldbody'), root.find('asset')
    world.remove(world.find("body[@name='center_camera_stand']"))
    quality=root.find('visual/quality')
    if quality is None:quality=ET.SubElement(root.find('visual'),'quality')
    quality.set('numslices','64');quality.set('numstacks','32');quality.set('shadowsize','4096');quality.set('offsamples','4')
    for mesh in asset.findall('mesh'):
        if mesh.get('name', '').startswith('cardinal_blade_'):
            mesh.set('scale', '3 3 3')
    prop = world.find("body[@name='cardinal_propeller']")
    prop.set('pos', numbers(CENTER))
    for name in ('cardinal_hub', 'cardinal_nose'):
        g = prop.find(f"geom[@name='{name}']")
        for attr in ('pos', 'size'):
            if g.get(attr):g.set(attr, numbers(np.fromstring(g.get(attr), sep=' ')*SCALE))
    stand = world.find("body[@name='cardinal_stand']")
    stand.set('pos', numbers(CENTER))
    stand.find("geom[@name='cardinal_base']").set('pos', numbers([0, -.15, TABLE_Z+.012-CENTER[2]]))
    stand.find("geom[@name='cardinal_base']").set('size', '.11 .10 .012')
    height = CENTER[2]-TABLE_Z-.024
    post = stand.find("geom[@name='cardinal_post']")
    post.set('pos', numbers([0, -.20, -height/2]))
    post.set('size', numbers([.028, .028, height/2]))
    shaft = stand.find("geom[@name='cardinal_shaft']")
    shaft.set('pos', '0 -.15 0');shaft.set('size', '.030 .12')
    for i, y in enumerate((-.20, -.14)):
        g = stand.findall("geom")[3+i]
        g.set('pos', f'0 {y} 0');g.set('size', '.045 .012')
    for side, x in [('left', .22), ('right', -.15)]:
        world.find(f"body[@name='{side}_base_link']").set('pos', numbers([x, .24, BASE_Z]))
        geom(world, side+'_temporary_riser', type='box', pos=numbers([x, .24, (BASE_Z+TABLE_Z)/2]),
             size=numbers([.09, .10, (BASE_Z-TABLE_Z)/2]), rgba='.23 .27 .29 1')
        geom(world, side+'_riser_foot', type='box', pos=numbers([x, .24, TABLE_Z+.009]),
             size='.12 .13 .009', rgba='.16 .19 .21 1')
    # Relocated overhead camera retains the complete standard policy image pipeline.
    camera(world, 'top', [-.14, 1.28, 2.35], [-.24, -.04, 1.22], 62)
    camera(world, 'cardinal_front', [-.27, 1.85, 1.72], [-.27, -.04, 1.25], 48)
    camera(world, 'cardinal_detail', [.31, .55, 1.50], [-.01, -.10, 1.31], 47)
    if underwater:
        underwater_style(root)
    return ET.tostring(root, encoding='unicode')


def underwater_style(root):
    """Rendering context only. Never used to run physics or claim wet dynamics."""
    world, asset, visual = root.find('worldbody'), root.find('asset'), root.find('visual')
    # Keep task/robot transforms identical. Replace visible tabletop fixtures with
    # a subsea service frame; extra geometry is visual-only and has no joints.
    for g in list(world.findall('geom')):
        world.remove(g)
    world.remove(world.find("body[@name='cardinal_stand']"))
    for light in list(world.findall('light')):world.remove(light)
    sky = asset.find("texture[@type='skybox']")
    if sky is not None:
        sky.set('rgb1', '.015 .10 .13');sky.set('rgb2', '.004 .025 .045')
        sky.set('mark', 'none')
    for tag, kw in [('map',dict(fogstart='1.3',fogend='7')),('rgba',dict(fog='.025 .14 .18 1'))]:
        node=visual.find(tag)
        if node is None:node=ET.SubElement(visual,tag)
        for k,v in kw.items():node.set(k,v)
    head=visual.find('headlight')
    if head is None:head=ET.SubElement(visual,'headlight')
    head.set('ambient','.14 .23 .25');head.set('diffuse','.16 .22 .25');head.set('specular','.1 .15 .18')
    def vgeom(name, **kw):return geom(world,name,contype=0,conaffinity=0,group=2,**kw)
    vgeom('seabed',type='plane',pos='0 0 -.13',size='12 12 .1',rgba='.12 .22 .21 1')
    # Broad rounded stern above and behind the shaft, with an exposed shaft boss.
    vgeom('vessel_stern',type='ellipsoid',pos=numbers(CENTER+[0,-1.35,1.05]),size='2.1 1.3 .93',rgba='.13 .22 .24 1')
    vgeom('stern_keel',type='ellipsoid',pos=numbers(CENTER+[0,-.93,.35]),size='.32 .9 .52',rgba='.11 .17 .18 1')
    vgeom('shaft_boss',type='ellipsoid',pos=numbers(CENTER+[0,-.64,0]),size='.12 .44 .14',rgba='.22 .30 .30 1')
    vgeom('submerged_shaft',type='cylinder',pos=numbers(CENTER+[0,-.34,0]),size='.041 .28',euler=f'{np.pi/2} 0 0',rgba='.48 .53 .49 1')
    for x in (.22,-.15):
        vgeom('mount_'+str(x),type='box',pos=numbers([x,.24,BASE_Z-.025]),size='.105 .12 .025',rgba='.14 .20 .21 1')
        vgeom('frame_leg_'+str(x),type='capsule',fromto=numbers([x,.28,.06,x,.28,BASE_Z-.05]),size='.025',rgba='.24 .33 .34 1')
        vgeom('frame_skid_'+str(x),type='capsule',fromto=numbers([x,-.02,.04,x,.65,.04]),size='.035',rgba='.13 .19 .21 1')
    vgeom('frame_crossbar',type='capsule',fromto=numbers([-.30,.32,BASE_Z-.07,.30,.32,BASE_Z-.07]),size='.025',rgba='.26 .34 .35 1')
    for x in (-.3,.3):
        vgeom('work_lamp_'+str(x),type='cylinder',pos=numbers([x,.29,BASE_Z+.05]),size='.037 .03',euler=f'{np.pi/2} 0 0',rgba='.6 .85 .9 1')
        ET.SubElement(world,'light',pos=numbers([x,.34,BASE_Z+.25]),dir=numbers([-.2,-1,.15]),directional='false',diffuse='.75 .90 .92',specular='.45 .55 .58',cutoff='65',exponent='2',castshadow='false')
    ET.SubElement(world,'light',pos='-2 1 5',dir='.3 -.3 -1',directional='true',diffuse='.35 .64 .70',specular='.35 .5 .5',castshadow='false')
    # Deterministic fine suspended particles and sparse seabed stones.
    rng=np.random.default_rng(41)
    for i in range(180):
        pos=rng.uniform([-2,-1.4,.1],[1.6,2.4,3.0])
        vgeom(f'water_particle_{i}',type='sphere',pos=numbers(pos),size=str(rng.uniform(.001,.003)),rgba='.38 .61 .62 .33')
    for i in range(30):
        pos=rng.uniform([-3,-3,-.1],[3,2,-.08]);size=rng.uniform([.02,.02,.01],[.10,.08,.04])
        vgeom(f'seabed_stone_{i}',type='ellipsoid',pos=numbers(pos),size=numbers(size),rgba='.16 .23 .21 1')
    camera(world,'underwater_hero',[.88,1.72,1.95],[-.30,-.16,1.37],42)
    camera(world,'underwater_wide',[1.45,2.70,2.04],[-.40,-.25,1.42],47)
    camera(world,'underwater_detail',[.32,.56,1.48],[-.025,-.11,1.32],43)


class LargeEnv(ViscousAppleEnv):
    def __init__(self, underwater=False):
        self.wrist='maker';self.sides=('left','right')
        self.model=mujoco.MjModel.from_xml_string(build_xml(underwater))
        self.data=mujoco.MjData(self.model)
        self.renderer=self.wrist_renderer=None;self.qadr={};self.vadr={};self.aids={}
        for s in self.sides:
            joints=[self.model.joint(f'{s}_joint{i}') for i in range(1,7)]
            self.qadr[s]=np.array([j.qposadr[0] for j in joints]);self.vadr[s]=np.array([j.dofadr[0] for j in joints])
            self.aids[s]=np.array([self.model.actuator(f'{s}_{n}_drive').id for n in JOINTS])
        self.reset()
