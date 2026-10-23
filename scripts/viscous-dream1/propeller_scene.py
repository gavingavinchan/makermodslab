"""Bench propeller geometry and row measurements; no policy or network code."""
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import mujoco
import trimesh

HERE = Path(__file__).resolve().parent
ASSETS = HERE / 'assets/propeller'
CENTER = np.array([.025, -.16, .945])
SPONGE = np.array([.19, .055, .785])
BRONZE = '.62 .36 .105 1'


def blade_vertices(blade):
    # Rounded swept paddle, with pitch decreasing from 38 to 17 degrees.
    a = blade * 2 * np.pi / 3 + .20
    radial = np.array([np.cos(a), 0, np.sin(a)])
    tangent = np.array([-np.sin(a), 0, np.cos(a)])
    vertices = []
    for t in np.linspace(0, 1, 13):
        r = .020 + .083 * t
        width = .012 + .026 * np.sin(np.pi * t) ** .65
        if t == 1: width = .002
        pitch = np.deg2rad(38 - 21 * t)
        sweep = .014 * t ** 1.3
        for h in (-.002, .002):
            for u in np.linspace(-1, 1, 7):
                vertices.append(radial * r + tangent * (sweep + u * width * np.cos(pitch))
                                + np.array([0, u * width * np.sin(pitch) + h, 0]))
    return np.array(vertices).reshape(13, 2, 7, 3)


def generate_assets():
    ASSETS.mkdir(parents=True, exist_ok=True)
    blades = []
    for b in range(3):
        v = blade_vertices(b)
        faces = []
        index = lambda i, h, j: i * 14 + h * 7 + j
        for i in range(12):
            for h in range(2):
                for j in range(6):
                    q = [index(i,h,j),index(i+1,h,j),index(i+1,h,j+1),index(i,h,j+1)]
                    faces.extend([[q[0],q[1],q[2]],[q[0],q[2],q[3]]])
            for j in (0,6):
                q = [index(i,0,j),index(i+1,0,j),index(i+1,1,j),index(i,1,j)]
                faces.extend([[q[0],q[1],q[2]],[q[0],q[2],q[3]]])
        for i in (0,12):
            for j in range(6):
                q=[index(i,0,j),index(i,0,j+1),index(i,1,j+1),index(i,1,j)]
                faces.extend([[q[0],q[1],q[2]],[q[0],q[2],q[3]]])
        mesh=trimesh.Trimesh(v.reshape(-1,3),faces,process=True)
        mesh.fix_normals(); blades.append(mesh)
        for k in range(6):
            part=trimesh.convex.convex_hull(v[k*2:k*2+3].reshape(-1,3))
            part.export(ASSETS/f'blade_{b}_part_{k}.stl')
    mesh=trimesh.util.concatenate(blades)
    mesh.export(ASSETS/'blades.stl')
    return mesh


def add_scene(root):
    if not (ASSETS/'blades.stl').exists(): generate_assets()
    world,asset=root.find('worldbody'),root.find('asset')
    for name in ('apple','orange','bowl','banana','water_bottle','plate'):
        node=world.find(f"body[@name='{name}']")
        if node is not None: world.remove(node)
    ET.SubElement(asset,'material',name='bronze',rgba=BRONZE,specular='.65',shininess='.6')
    ET.SubElement(asset,'mesh',name='propeller_visual',file=str(ASSETS/'blades.stl'),smoothnormal='true')
    prop=ET.SubElement(world,'body',name='propeller',pos=' '.join(map(str,CENTER)))
    # No joint: fixed to world/table, including the stand; no dynamics can rotate the hub.
    ET.SubElement(prop,'geom',name='propeller_blades_visual',type='mesh',mesh='propeller_visual',material='bronze',contype='0',conaffinity='0',group='2')
    for b in range(3):
        for k in range(6):
            name=f'propeller_blade_{b}_{k}'
            ET.SubElement(asset,'mesh',name=name,file=str(ASSETS/f'blade_{b}_part_{k}.stl'))
            ET.SubElement(prop,'geom',name=name,type='mesh',mesh=name,rgba='0 0 0 0',group='3')
    ET.SubElement(prop,'geom',name='propeller_hub',type='cylinder',size='.025 .030',euler=f'{np.pi/2} 0 0',material='bronze')
    ET.SubElement(prop,'geom',name='propeller_hub_nose',type='ellipsoid',pos='0 .030 0',size='.024 .013 .024',material='bronze')
    stand=ET.SubElement(world,'body',name='propeller_stand',pos=' '.join(map(str,CENTER)))
    ET.SubElement(stand,'geom',name='propeller_stand_base',type='box',pos='0 -.045 -.177',size='.055 .045 .008',rgba='.18 .21 .23 1')
    ET.SubElement(stand,'geom',name='propeller_stand_post',type='box',pos='0 -.065 -.090',size='.012 .012 .085',rgba='.25 .28 .30 1')
    ET.SubElement(stand,'geom',name='propeller_stand_shaft',type='cylinder',pos='0 -.045 0',size='.010 .035',euler=f'{np.pi/2} 0 0',rgba='.45 .48 .50 1')
    sponge=ET.SubElement(world,'body',name='sponge',pos=' '.join(map(str,SPONGE)))
    ET.SubElement(sponge,'freejoint',name='sponge_free')
    ET.SubElement(sponge,'geom',name='sponge_foam',type='box',size='.045 .030 .017',mass='.025',rgba='.98 .80 .045 1',friction='1.6 .01 .001')
    ET.SubElement(sponge,'geom',name='sponge_scour',type='box',pos='0 0 .020',size='.045 .030 .003',mass='.005',rgba='.075 .28 .09 1',friction='1.8 .01 .001')
    # Small shallow pores, visual only, so the foam reads as a sponge at wrist resolution.
    rng=np.random.default_rng(123)
    for i in range(38):
        x,yy=rng.uniform([-.042,-.027],[.042,.027])
        ET.SubElement(sponge,'geom',name=f'sponge_pore_{i}',type='sphere',pos=f'{x} {yy} .023',size='.0013',rgba='.12 .35 .10 1',contype='0',conaffinity='0',mass='0')
    return root


def yam_env():
    from dream_examples.yam_mujoco.yam_env import YamEnv, build_xml
    class Env(YamEnv):
        def __init__(self):
            self.model=mujoco.MjModel.from_xml_string(ET.tostring(add_scene(ET.fromstring(build_xml())),encoding='unicode'))
            self.data=mujoco.MjData(self.model);self.renderer=None
            self.sides=('left','right');self.single=self.parked=self.copy_right=self.fixed_right=self.ignore_right=False
            self.reset()
        def step(self,left,right):
            for s,a in [('left',left),('right',right)]:
                a=np.asarray(a,float)
                assert a.shape==(7,) and np.isfinite(a).all() and 0<=a[6]<=1
                for i,q in enumerate(a[:6],1):
                    self.data.actuator(f'{s}_joint{i}').ctrl[0]=np.clip(q,*self.model.joint(f'{s}_joint{i}').range)
                for sign in (-1,1): self.data.actuator(f'{s}_finger_{sign}').ctrl[0]=a[6]*.0475
            for _ in range(25):
                mujoco.mj_step(self.model,self.data)
                if getattr(self,'row_observer',None): self.row_observer()
            mujoco.mj_forward(self.model,self.data)
            return self.state()
    return Env()


def closest_surface(points, triangles, normals):
    points=np.asarray(points).reshape(-1,3)
    result=[]
    for p in points:
        q=trimesh.triangles.closest_point(triangles,np.repeat(p[None],len(triangles),axis=0))
        i=np.argmin(np.sum((q-p)**2,axis=1))
        result.append((float(np.linalg.norm(q[i]-p)),q[i],normals[i]))
    return result


class Reversal:
    """One-centimetre hysteresis along a fixed local tangent per contact bout."""
    def __init__(self): self.reset()
    def reset(self): self.axis=None;self.origin=None;self.extreme=0.;self.direction=0;self.qualified=False
    def add(self,p,normal):
        if self.origin is None: self.origin=p.copy();self.last=p.copy();return 0,0.
        delta=p-self.last;self.last=p.copy();delta-=normal*np.dot(delta,normal)
        length=float(np.linalg.norm(delta))
        if self.axis is None:
            d=p-self.origin;d-=normal*np.dot(d,normal)
            if np.linalg.norm(d)<.001:return 0,length
            self.axis=d/np.linalg.norm(d)
        x=float(np.dot(p-self.origin,self.axis))
        if self.direction==0:
            if abs(x)>.01:self.direction=1 if x>0 else -1;self.extreme=x;self.qualified=True
            return 0,length
        if (x-self.extreme)*self.direction>0:self.extreme=x;return 0,length
        if (self.extreme-x)*self.direction>.01:
            self.direction*=-1;self.extreme=x
            return 1,length
        return 0,length


class Measurements:
    def __init__(self,env):
        self.env=env;m=env.model
        self.prop={i for i in range(m.ngeom) if m.body(m.geom_bodyid[i]).name=='propeller' and (m.geom_contype[i] or m.geom_conaffinity[i])}
        self.tool={m.geom(n).id for n in ('sponge_foam','sponge_scour')}
        self.fingers={s:[] for s in env.sides}
        for i in range(m.ngeom):
            name=m.body(m.geom_bodyid[i]).name or ''
            for s in env.sides:
                if name.startswith(s+'_') and any(k in name for k in ('gripper_left','gripper_right','tip_left','tip_right')) and (m.geom_contype[i] or m.geom_conaffinity[i]):self.fingers[s].append(i)
        self.mesh=trimesh.load(ASSETS/'blades.stl',force='mesh')
        self.triangles=self.mesh.triangles+CENTER;self.normals=self.mesh.face_normals
        # Requested-point distance includes the visible hub, with a tessellated smooth nose.
        hub=trimesh.creation.cylinder(radius=.025,height=.060,sections=64)
        hub.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]))
        nose=trimesh.creation.icosphere(subdivisions=3);nose.vertices*=np.array([.024,.013,.024]);nose.apply_translation([0,.030,0])
        whole=trimesh.util.concatenate([self.mesh,hub,nose])
        self.all_triangles=whole.triangles+CENTER;self.all_normals=whole.face_normals
        self.tip_local={}
        self.tip_body_local={}
        for side in env.sides:
            approach=env.data.site(side+'_tcp').xmat.reshape(3,3)[:,2]
            names=(side+'_visual_043',side+'_visual_042') if hasattr(env,'qadr') else None
            if names:
                self.tip_local[side]=[]
                for name in names:
                    g=m.geom(name).id;mid=m.geom_dataid[g]
                    v=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]].copy()
                    projection=(v@env.data.geom_xmat[g].reshape(3,3).T)@approach
                    tip=v[projection>projection.max()-.0001].mean(0)
                    self.tip_local[side].append((g,tip))
            else:
                self.tip_body_local[side]=[]
                for suffix in ('_tip_left','_tip_right'):
                    body=m.body(side+suffix).id;cloud=[]
                    for g in self.fingers[side]:
                        if m.geom_bodyid[g]!=body:continue
                        mid=m.geom_dataid[g]
                        if mid<0:continue
                        v=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]]
                        cloud.append(v@env.data.geom_xmat[g].reshape(3,3).T+env.data.geom_xpos[g])
                    cloud=np.concatenate(cloud);projection=cloud@approach
                    tip=cloud[projection>projection.max()-.0001].mean(0)
                    local=env.data.xmat[body].reshape(3,3).T@(tip-env.data.xpos[body])
                    self.tip_body_local[side].append((body,local))
        self.start_z=float(env.data.body('sponge').xpos[2]);self.track={k:Reversal() for k in ('tool','left','right')}
        self.prev_tool=env.data.body('sponge').xpos.copy()
        self.gripper=set().union(*self.fingers.values())
        for i in range(m.ngeom):
            name=m.body(m.geom_bodyid[i]).name or ''
            if any(name==s+suffix for s in env.sides for suffix in ('_wrist_roll_link','_gripper')) and (m.geom_contype[i] or m.geom_conaffinity[i]):self.gripper.add(i)
        self.row_contacts=set();env.row_observer=self.contacts
    def contacts(self):
        for con in self.env.data.contact:
            a,b=int(con.geom1),int(con.geom2)
            if a in self.prop or b in self.prop:
                other=b if a in self.prop else a
                if other in self.tool:self.row_contacts.add('tool')
                if other in self.gripper:self.row_contacts.add('gripper')
    def finger_points(self,s):
        e=self.env
        if hasattr(e,'qadr'):
            return np.array([e.data.geom_xpos[g]+e.data.geom_xmat[g].reshape(3,3)@p for g,p in self.tip_local[s]])
        return np.array([e.data.xpos[b]+e.data.xmat[b].reshape(3,3)@p for b,p in self.tip_body_local[s]])
    def between(self,s):
        a,b=self.finger_points(s);p=self.env.data.body('sponge').xpos
        axis=b-a;length=np.linalg.norm(axis)
        if length<.001:return False
        u=np.dot(p-a,axis)/length**2
        return bool(-.15<=u<=1.15 and np.linalg.norm(p-(a+u*axis))<.055)
    def distance(self,ids):
        e=self.env;best=.2;pair=np.zeros(6);bestpair=None
        for a in ids:
            # Coarse body distance avoids unnecessary convex mesh queries far away.
            if np.linalg.norm(e.data.geom_xpos[a]-CENTER)>.32:continue
            for b in self.prop:
                d=mujoco.mj_geomDistance(e.model,e.data,a,b,best,pair)
                if d<best:best=float(d);bestpair=pair.copy()
        return best,bestpair
    def requested(self,requested):
        e=self.env;answer={}
        for s,p in requested.items():
            base=e.data.body(s+('_base_link' if hasattr(e,'qadr') else '_base'))
            world=base.xpos+base.xmat.reshape(3,3)@p
            dist=closest_surface([world],self.all_triangles,self.all_normals)[0][0]
            local=(world-e.data.body('sponge').xpos)@e.data.body('sponge').xmat.reshape(3,3)
            sponge=float(np.linalg.norm(np.maximum(np.abs(local-[0,0,.003])-[.045,.030,.020],0)))
            answer[s]={'propeller_m':dist,'sponge_m':sponge}
        return answer
    def row(self,call,index,requested_dist):
        e=self.env;p=e.data.body('sponge').xpos.copy()
        between={s:self.between(s) for s in e.sides}
        grasped=p[2]>self.start_z+.03 and any(between.values())
        td,tp=self.distance(self.tool)
        row=dict(call=call,row=index,sponge_xyz=p.tolist(),sponge_lift_m=float(p[2]-self.start_z),between_fingers=between,tool_grasped=bool(grasped),tool_propeller_contact='tool' in self.row_contacts,gripper_propeller_contact='gripper' in self.row_contacts,requested_hand=requested_dist,tool_surface_gap_m=td,scrub_reversals=0,path_in_contact_m=0.,path_within_1cm_m=0.,effectors={})
        # Use sponge when held, or when it actually touches; otherwise bare finger motion.
        for key in ('tool',*e.sides):
            if key=='tool':point=p;gap=td;pair=tp;eligible=grasped or 'tool' in self.row_contacts
            else:
                pts=self.finger_points(key);point=pts.mean(0);gap,pair=self.distance(self.fingers[key]);eligible=not between[key]
            # Surface normal comes from the nearest triangle on the actual twisted blade.
            near=False
            if eligible and gap<=.01:
                surface=point if pair is None else pair[3:]
                blade_distance,nearest,normal=closest_surface([surface],self.triangles,self.normals)[0]
                near=blade_distance<=.012
            if near:rev,path=self.track[key].add(point,normal)
            else:self.track[key].reset();rev,path=0,0.
            row['effectors'][key]=dict(near_blade=bool(near),gap_m=gap,reversals=rev,path_m=path)
            row['scrub_reversals']+=rev;row['path_within_1cm_m']+=path
            physical=row['tool_propeller_contact'] if key=='tool' else row['gripper_propeller_contact']
            if physical:row['path_in_contact_m']+=path
        self.row_contacts.clear()
        return row


class ViewsRecorder:
    """Actual top and left wrist pixels, side by side, at the existing video cadence."""
    def __init__(self,env):
        self.frames=[];self.milestones=[0]
        self.renderer=type('NoRenderer',(),{'close':lambda self:None})()
    def grab(self,env,top):
        images=env.images()
        self.frames.append(np.concatenate([top,images['left']],axis=1))
