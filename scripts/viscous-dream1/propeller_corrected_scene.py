# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Conventional helical screw rotor; the large bench is otherwise unchanged.

Visual design reference: Sole FA1628 four-blade photograph, cross-checked with
France Helices FH4-65 and the Palumbo Malta four-blade shipyard photograph.
This is an illustrative parametric design, not a reverse-engineered product.
All four blades use EXACT rotations of one section family, never reflections.
The same sampled section grid supplies the visible surface and collision cells.
"""
import copy
import json
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import trimesh
import propeller_large_scene as large
from cross_scenes import ViscousAppleEnv, JOINTS

HERE, CENTER, PAD_Z = large.HERE, large.CENTER, large.PAD_Z
RADIUS, ROOT_RADIUS = .480, .085
PITCH = .754  # one revolution of the mean helicoid, metres; P/D = 0.7854
ASSETS = HERE / 'assets/propeller_corrected'
OUT = HERE / 'runs/propeller_corrected'
LEFT_T, SCRUB_T, SCRUB_AMPLITUDE, LEFT_OPEN = .97, .80, .05, .024
NR, NU = 48, 16
T_GRID = np.sin(np.linspace(0, np.pi/2, NR+1))
T_GRID[-1] = 1.
U_GRID = -np.cos(np.linspace(0, np.pi, NU+1))
numbers = large.numbers


def section(t):
    """Smooth broad outer chord with an elliptical, zero-chord rounded tip."""
    r = ROOT_RADIUS + (RADIUS-ROOT_RADIUS)*t
    half_chord = (.055 + .270*t - .055*t*t)*np.sqrt(max(0., 1-t**4))
    skew = .12*np.sin(np.pi*t)  # 6.9 deg maximum, tips on cardinal axes
    rake = .015*t
    half_thickness = (.012*(1-t)+.002)*np.sqrt(max(0., 1-t**4))
    return r, half_chord, skew, rake, half_thickness


def blade_point(blade, t, u, h=0):
    """Point on the +Y face for h=+1, -Y face for h=-1, mean for h=0.

    t is root-to-tip, u is chord coordinate. Sections lie on cylinders about
    the shaft: axial advance is -P*theta/(2*pi), giving coherent handedness
    and the appropriate decreasing pitch angle atan(P/(2*pi*r)).
    """
    r, w, skew, rake, thick = section(t)
    theta = skew + u*w/r
    camber = .0035*np.sin(np.pi*t)*(1-u*u)
    y = rake - PITCH/(2*np.pi)*theta + camber + h*thick*np.sqrt(max(0., 1-u*u))
    angle = blade*np.pi/2 + theta
    return np.array([r*np.cos(angle), y, r*np.sin(angle)])


def surface(t, u=0):
    p = blade_point(0,t,u,1)
    eps=1e-6
    dt=(blade_point(0,t+eps,u,1)-blade_point(0,t-eps,u,1))/(2*eps)
    du=(blade_point(0,t,u+eps,1)-blade_point(0,t,u-eps,1))/(2*eps)
    n=np.cross(du,dt);n/=np.linalg.norm(n)
    assert n[1]>0
    return CENTER+p,n


def grid(blade=0):
    return np.array([[[blade_point(blade,t,u,h) for u in U_GRID] for h in (-1,1)] for t in T_GRID])


def collision_cells(vertices):
    for i in range(NR):
        for j in range(NU):
            yield i,j,trimesh.convex.convex_hull(vertices[i:i+2,:,j:j+2].reshape(-1,3))


def blade_mesh(vertices):
    # Use the actual exterior triangles of the convex collision cells. The
    # shared cell walls are vertical planes (each x/z grid point has two y
    # heights), so the joined exterior has exactly the same contact surface.
    pieces=[]
    for i,j,cell in collision_cells(vertices):
        exposed=np.abs(cell.face_normals[:,1])>1e-8
        if i==0:
            radius=np.linalg.norm(cell.triangles[:,:,[0,2]],axis=2)
            exposed |= np.all(np.isclose(radius,ROOT_RADIUS,atol=1e-9),axis=1)
        pieces.append(trimesh.Trimesh(cell.vertices,cell.faces[exposed],process=False))
    mesh=trimesh.util.concatenate(pieces)
    mesh.merge_vertices();mesh.update_faces(mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices();mesh.fix_normals()
    return mesh


def generate_assets():
    ASSETS.mkdir(parents=True,exist_ok=True)
    g=grid();base=blade_mesh(g)
    assert base.is_watertight and base.is_winding_consistent and base.volume>0
    cells=list(collision_cells(g))
    for b in range(4):
        a=b*np.pi/2;c,s=np.cos(a),np.sin(a)
        rotation=np.array([[c,0,-s],[0,1,0],[s,0,c]])
        m=base.copy();m.vertices=base.vertices@rotation.T
        m.export(ASSETS/f'blade_{b}.stl')
        for i,j,cell in cells:
            m=cell.copy();m.vertices=cell.vertices@rotation.T
            m.export(ASSETS/f'blade_{b}_{i}_{j}.stl')
    (ASSETS/'profile.json').write_text(json.dumps(dict(profile='sole-inspired-helical-v1',
        diameter_m=2*RADIUS,root_radius_m=ROOT_RADIUS,pitch_m=PITCH,
        radial_cells=NR,chord_cells=NU,source='propeller_corrected_scene.py',
        reference='https://soleadvance.com/Fotos/FA1628_001.jpg'),indent=2)+'\n')


def build_xml():
    if not (ASSETS/'profile.json').exists():generate_assets()
    root=ET.fromstring(large.build_xml())
    asset=root.find('asset');prop=root.find("worldbody/body[@name='cardinal_propeller']")
    for node in list(asset):
        if node.get('name','').startswith('cardinal_blade_'):asset.remove(node)
    for node in list(prop):
        if node.get('name','').startswith('cardinal_blade_'):prop.remove(node)
    for b in range(4):
        name=f'cardinal_blade_{b}'
        ET.SubElement(asset,'mesh',name=name,file=str(ASSETS/f'blade_{b}.stl'),smoothnormal='true')
        large.geom(prop,name+'_visual',type='mesh',mesh=name,material='cardinal_bronze',contype=0,conaffinity=0,group=2)
        for i in range(NR):
            for j in range(NU):
                name=f'cardinal_blade_{b}_{i}_{j}'
                ET.SubElement(asset,'mesh',name=name,file=str(ASSETS/f'blade_{b}_{i}_{j}.stl'))
                large.geom(prop,name,type='mesh',mesh=name,rgba='0 0 0 0',group=3,friction='1 .01 .001')
    return ET.tostring(root,encoding='unicode')


def inspection_xml():
    """Separate propeller-only diagnostic model, never the dynamics scene."""
    root=ET.fromstring(build_xml());world=root.find('worldbody')
    prop=copy.deepcopy(world.find("body[@name='cardinal_propeller']"))
    lights=[copy.deepcopy(x) for x in world.findall('light')]
    world.clear();world.append(prop)
    for light in lights:world.append(light)
    for tag in ('actuator','equality','contact','sensor','keyframe'):
        node=root.find(tag)
        if node is not None:root.remove(node)
    for name,offset,fovy in [('head_on',[0,2.0,0],33),('oblique',[1.15,1.7,.65],34),('side',[1.9,.48,.15],34)]:
        large.camera(world,'inspection_'+name,CENTER+offset,CENTER,fovy)
    return ET.tostring(root,encoding='unicode')


class CorrectedEnv(ViscousAppleEnv):
    def __init__(self):
        self.wrist='maker';self.sides=('left','right')
        self.model=mujoco.MjModel.from_xml_string(build_xml());self.data=mujoco.MjData(self.model)
        self.renderer=self.wrist_renderer=None;self.qadr={};self.vadr={};self.aids={}
        for s in self.sides:
            joints=[self.model.joint(f'{s}_joint{i}') for i in range(1,7)]
            self.qadr[s]=np.array([j.qposadr[0] for j in joints]);self.vadr[s]=np.array([j.dofadr[0] for j in joints])
            self.aids[s]=np.array([self.model.actuator(f'{s}_{n}_drive').id for n in JOINTS])
        self.reset()
