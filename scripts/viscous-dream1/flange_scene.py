"""Fixed pipe-flange fixture, real clearance contacts and an ideal thread manifold.
No policy, network or hardware access. All lengths are metres.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import trimesh
from shapely.geometry import Point, Polygon

HERE = Path(__file__).resolve().parent
ASSETS = HERE/'assets/flange'
CENTER = np.array([0., -.16, .965])
HOLE = CENTER + [0., .125, 0.]
BOLT = np.array([.19, .065, .787])
NUT = np.array([-.15, .065, .774])
PITCH = .0035
LENGTH = .120
NUT_HALF = .013
PROMPT = 'insert the bolt through the flange holes and screw the nut onto the other end'


def numbers(v): return ' '.join(f'{x:.9g}' for x in np.asarray(v).ravel())


def extrude(poly, length):
    m = trimesh.creation.extrude_polygon(poly, length, engine='earcut')
    m.apply_translation([0,0,-length/2]);m.fix_normals()
    return m


def generate_pipe_obj():
    n=96;verts=[];normals=[];faces=[]
    for radius,inner in [(.0603,False),(.0525,True)]:
     start=len(verts)
     for z in (-.095,.095):
      for i in range(n):
       a=2*np.pi*i/n;verts.append([radius*np.cos(a),radius*np.sin(a),z]);normals.append([(-1 if inner else 1)*np.cos(a),(-1 if inner else 1)*np.sin(a),0])
     for i in range(n):
      j=(i+1)%n
      for f in ([i,j,n+j],[i,n+j,n+i]):faces.append([start+k for k in (f[::-1] if inner else f)])
    for z,sign in [(-.095,-1),(.095,1)]:
     start=len(verts)
     for radius in (.0603,.0525):
      for i in range(n):
       a=2*np.pi*i/n;verts.append([radius*np.cos(a),radius*np.sin(a),z]);normals.append([0,0,sign])
     for i in range(n):
      j=(i+1)%n
      for f in ([i,j,n+j],[i,n+j,n+i]):faces.append([start+k for k in (f if sign==1 else f[::-1])])
    p=ASSETS/'pipe.obj'
    p.write_text('\n'.join(['# Split normals for smooth steel tube, sharp annular rims']+['v '+' '.join(map(str,v)) for v in verts]+['vn '+' '.join(map(str,v)) for v in normals]+['f '+' '.join(f'{i+1}//{i+1}' for i in f) for f in faces])+'\n')


def generate_assets():
    """Triangulate the perforated face, then merge triangles into convex prisms.
    Every collision prism stays OUTSIDE all five holes. A nonconvex visual mesh
    alone would be convexified by MuJoCo and incorrectly close the holes.
    """
    ASSETS.mkdir(parents=True,exist_ok=True)
    circle=lambda radius:Point(0,0).buffer(radius,quad_segs=24)
    flange=circle(.165).difference(circle(.0525))
    for a in np.arange(4)*np.pi/2:
        flange=flange.difference(Point(.125*np.cos(a),.125*np.sin(a)).buffer(.017,quad_segs=12))
    extrude(flange,.036).export(ASSETS/'flange.stl')
    v,f=trimesh.creation.triangulate_polygon(flange,engine='earcut')
    cells=[Polygon(v[t]) for t in f]
    # Greedy convex merging reduces static contact pieces without bridging holes.
    changed=True
    while changed:
        changed=False
        for i in range(len(cells)):
            for j in range(i+1,len(cells)):
                u=cells[i].union(cells[j])
                if u.geom_type=='Polygon' and abs(u.convex_hull.area-u.area)<1e-12:
                    cells[i]=u;cells.pop(j);changed=True;break
            if changed:break
    for old in ASSETS.glob('flange_cell_*.stl'):old.unlink()
    for i,p in enumerate(cells):extrude(p,.036).export(ASSETS/f'flange_cell_{i:03d}.stl')
    generate_pipe_obj()
    # Hexagonal nut with real open bore and flat-shaded wrench faces.
    r=.046/np.sqrt(3)
    hexagon=Polygon(np.c_[r*np.cos(np.arange(6)*np.pi/3),r*np.sin(np.arange(6)*np.pi/3)])
    nut=hexagon.difference(Point(0,0).buffer(.0155,quad_segs=12))
    extrude(nut,.026).export(ASSETS/'nut.stl')
    # Six trapezoids each split into small convex sectors: physical through bore.
    for i in range(48):
        a,b=2*np.pi*np.array([i,i+1])/48
        def radius(t):
            normal=(np.floor(t/(np.pi/3))+.5)*np.pi/3
            return .023/np.cos(t-normal)
        p=Polygon([[.0155*np.cos(a),.0155*np.sin(a)],[radius(a)*np.cos(a),radius(a)*np.sin(a)],
                   [radius(b)*np.cos(b),radius(b)*np.sin(b)],[.0155*np.cos(b),.0155*np.sin(b)]])
        extrude(p,.026).export(ASSETS/f'nut_cell_{i:02d}.stl')
    # Flat hexagonal wrench faces; simplified unchamfered head.
    m=trimesh.creation.cylinder(radius=r,height=.020,sections=6)
    m.apply_translation([0,0,-.010]);m.export(ASSETS/'head.stl')
    return len(cells)


def add_scene(root):
    if not (ASSETS/'flange.stl').exists():generate_assets()
    if not (ASSETS/'pipe.obj').exists():generate_pipe_obj()
    root.find('size').set('memory','512M')
    world,asset=root.find('worldbody'),root.find('asset')
    for name in ('apple','orange','bowl','banana','water_bottle','plate'):
        node=world.find(f"body[@name='{name}']")
        if node is not None:world.remove(node)
    for name,rgba,spec,shine in [('flange_blue','.25 .38 .48 1','.55','.55'),('pipe_steel','.43 .48 .51 1','.7','.65'),('zinc','.72 .76 .79 1','.85','.8'),('stand_steel','.13 .16 .18 1','.35','.3')]:
        ET.SubElement(asset,'material',name=name,rgba=rgba,specular=spec,shininess=shine)
    def mesh(name,path,smooth=False):
        ET.SubElement(asset,'mesh',name=name,file=str(path),smoothnormal=str(smooth).lower())
    mesh('flange_visual',ASSETS/'flange.stl');mesh('pipe_visual',ASSETS/'pipe.obj')
    cells=sorted(ASSETS.glob('flange_cell_*.stl'))
    for i,p in enumerate(cells):mesh(f'flange_cell_{i}',p)
    def geom(body,name,**kw):
        return ET.SubElement(body,'geom',name=name,**{k:str(v) for k,v in kw.items()})
    # Mesh z axes are rotated onto world x. Bodies have no joints: world fixed.
    for sign in (-1,1):
        body=ET.SubElement(world,'body',name=f'flange_{sign}',pos=numbers(CENTER+[sign*.019,0,0]),euler=f'0 {np.pi/2} 0')
        geom(body,f'flange_visual_{sign}',type='mesh',mesh='flange_visual',material='flange_blue',contype=0,conaffinity=0,group=2)
        for i in range(len(cells)):
            geom(body,f'flange_contact_{sign}_{i}',type='mesh',mesh=f'flange_cell_{i}',rgba='0 0 0 0',group=3,margin='.0001')
        pipe=ET.SubElement(world,'body',name=f'pipe_{sign}',pos=numbers(CENTER+[sign*.131,0,0]),euler=f'0 {np.pi/2} 0')
        geom(pipe,f'pipe_visual_{sign}',type='mesh',mesh='pipe_visual',material='pipe_steel',contype=0,conaffinity=0,group=2)
        # Convex annular sectors leave the pipe bore open, too.
        for k in range(24):
            a,b=2*np.pi*np.array([k,k+1])/24
            verts=[[r*np.cos(t),r*np.sin(t),z] for z in (-.095,.095) for t in (a,b) for r in (.0525,.0603)]
            name=f'pipe_sector_{sign}_{k}'
            ET.SubElement(asset,'mesh',name=name,vertex=numbers(verts))
            geom(pipe,name,type='mesh',mesh=name,rgba='0 0 0 0',group=3)
        stand=ET.SubElement(world,'body',name=f'pipe_stand_{sign}',pos=numbers(CENTER+[sign*.16,0,0]))
        geom(stand,f'stand_base_{sign}',type='box',pos='0 0 -.197',size='.055 .075 .008',material='stand_steel')
        geom(stand,f'stand_post_{sign}',type='box',pos='0 0 -.137',size='.016 .028 .055',material='stand_steel')
        for sy in (-1,1):
            geom(stand,f'vblock_{sign}_{sy}',type='box',pos=f'0 {sy*.047} -.069',size='.045 .012 .030',euler=f'{-sy*.60} 0 0',material='stand_steel')
    # A 2 mm sealing gap separates the fixed mating faces; bolt holes remain clear.
    bolt=ET.SubElement(world,'body',name='bolt',pos=numbers(BOLT),euler=f'0 {-np.pi/2} 0')
    ET.SubElement(bolt,'freejoint',name='bolt_free')
    mesh('bolt_head',ASSETS/'head.stl')
    geom(bolt,'bolt_head',type='mesh',mesh='bolt_head',mass='.13',material='zinc',friction='1.5 .02 .002')
    geom(bolt,'bolt_shank',type='cylinder',pos='0 0 .057',size='.015 .057',mass='.50',material='zinc',friction='.5 .01 .001')
    geom(bolt,'bolt_tip',type='cylinder',pos='0 0 .117',size='.0138 .003',mass='.02',material='zinc')
    # Fine thread crests are visual only. The shank is the contact envelope.
    for k in range(20):
        geom(bolt,f'bolt_thread_{k}',type='cylinder',pos=f'0 0 {.047+k*PITCH}',size='.01525 .00040',mass=0,material='pipe_steel',contype=0,conaffinity=0)
    nut=ET.SubElement(world,'body',name='nut',pos=numbers(NUT))
    ET.SubElement(nut,'freejoint',name='nut_free')
    mesh('nut_visual',ASSETS/'nut.stl')
    geom(nut,'nut_visual',type='mesh',mesh='nut_visual',mass=0,material='zinc',contype=0,conaffinity=0,group=2)
    for i in range(48):
        name=f'nut_cell_{i}';mesh(name,ASSETS/f'nut_cell_{i:02d}.stl')
        geom(nut,name,type='mesh',mesh=name,mass=str(.18/48),rgba='0 0 0 0',group=3,friction='1.5 .02 .002')
    # Inspection cameras only: top and both policy wrist cameras remain untouched.
    def camera(name,eye,target,fovy):
        z=np.asarray(eye)-target;z=z/np.linalg.norm(z);x=np.cross([0,0,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
        ET.SubElement(world,'camera',name=name,pos=numbers(eye),xyaxes=numbers(np.r_[x,y]),fovy=str(fovy))
    camera('flange_overview',[.82,-.64,1.46],np.array([0,-.04,.88]),48)
    camera('flange_detail',[.35,.50,1.45],np.array([.0,-.025,.89]),45)
    camera('flange_plan',[.0,-.01,1.85],np.array([0,-.10,.88]),48)
    root.find('visual/global').set('offwidth','1280');root.find('visual/global').set('offheight','720')
    return root


class ThreadModel:
    """Ideal, zero-backlash screw constraint by per-1ms free-joint projection.

    Entry is gated by <3 mm radial error, <10 deg axis error, and the nut's
    leading face within 3 mm of the bolt tip. No bolt pose is ever corrected.
    Spin comes from unconstrained dynamics (e.g. gripper contact torque). Only
    swing, radial displacement and axial displacement are projected; pitch is
    3.5 mm/rev. Reverse beyond the start, or accumulated 3 mm outward attempted
    axial slip beyond the engaged length plus 3 mm, releases the ideal thread. This is a feasibility surrogate, not
    a thread/contact/torque or cross-threading simulator.
    """
    def __init__(self,env):
        self.env=env;self.engaged=False;self.turns=0.;self.advance=0.;self.events=[];self.pull=0.;self.last_time=-1
    def step(self):
        e=self.env;d=e.data;m=e.model
        b,n=d.body('bolt'),d.body('nut');R=b.xmat.reshape(3,3);N=n.xmat.reshape(3,3)
        local=R.T@(n.xpos-b.xpos);relative=R.T@N
        flip=self.flip if self.engaged else np.diag([1.,-1.,-1.]) if relative[2,2]<0 else np.eye(3)
        relative=relative@flip
        angle=np.degrees(np.arccos(np.clip(abs(relative[2,2]),-1,1)))
        theta=float(np.arctan2(relative[1,0],relative[0,0]))
        if not self.engaged:
            if np.linalg.norm(local[:2])<.003 and angle<10 and abs(local[2]-(LENGTH+NUT_HALF))<.003:
                self.engaged=True;self.start=local[2];self.theta=theta;self.total=0.;self.turns=self.advance=0.;self.pull=0.;self.flip=flip;self.events.append(dict(time=d.time,event='engage'))
            return
        previous_z=self.start-self.advance
        delta=np.arctan2(np.sin(theta-self.theta),np.cos(theta-self.theta));self.total+=delta;self.theta=theta
        self.turns=self.total/(2*np.pi);self.advance=PITCH*self.turns
        z=self.start-self.advance
        self.pull=max(0.,self.pull+float(local[2]-previous_z))
        if self.turns<-.10 or self.pull>max(self.advance,0.)+.003 or z<LENGTH-.075:
            self.engaged=False;self.events.append(dict(time=d.time,event='release',turns=self.turns,pull_m=self.pull));return
        j=m.joint('nut_free');qa,va=j.qposadr[0],j.dofadr[0]
        d.qpos[qa:qa+3]=b.xpos+R@np.array([0,0,z])
        aligned=R@np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])@self.flip
        mujoco.mju_mat2Quat(d.qpos[qa+3:qa+7],aligned.ravel())
        # Free-joint angular velocities are local, translational velocities world.
        bj=m.joint('bolt_free');bv=bj.dofadr[0]
        bolt_omega=R@d.qvel[bv+3:bv+6]
        nut_omega=N@d.qvel[va+3:va+6]
        spin=float(np.dot(nut_omega-bolt_omega,R[:,2]))
        d.qvel[va:va+3]=d.qvel[bv:bv+3]+np.cross(bolt_omega,R[:,2]*z)-R[:,2]*PITCH*spin/(2*np.pi)
        d.qvel[va+3:va+6]=aligned.T@(bolt_omega+R[:,2]*spin)
        mujoco.mj_forward(m,d)


class Measurements:
    def __init__(self,env):
        self.env=env;self.contacts_seen=set();self.start={n:env.data.body(n).xpos[2] for n in ('bolt','nut')}
        self.previous=env.row_observer;env.row_observer=self.contacts
        m=env.model
        self.fingers={s:{i for i in range(m.ngeom) if (m.geom_contype[i] or m.geom_conaffinity[i]) and m.body(m.geom_bodyid[i]).name.startswith(s+'_') and 'gripper_' in m.body(m.geom_bodyid[i]).name} for s in env.sides}
        self.tip_local={}
        for side in env.sides:
            self.tip_local[side]=[]
            approach=env.data.site(side+'_tcp').xmat.reshape(3,3)[:,2]
            for name in (side+'_visual_043',side+'_visual_042'):
                g=m.geom(name).id;mid=m.geom_dataid[g]
                v=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]].copy()
                projection=(v@env.data.geom_xmat[g].reshape(3,3).T)@approach
                tip=v[projection>projection.max()-.0001].mean(0)
                self.tip_local[side].append((g,tip))
        self.fixture={i for i in range(m.ngeom) if m.body(m.geom_bodyid[i]).name.startswith(('flange_','pipe_'))}
        self.objects={n:{i for i in range(m.ngeom) if m.geom_bodyid[i]==m.body(n).id} for n in ('bolt','nut')}
    def contacts(self):
        self.previous()
        for c in self.env.data.contact:
            a,b=int(c.geom1),int(c.geom2)
            for n,ids in self.objects.items():
                other=b if a in ids else a if b in ids else None
                if other is None:continue
                if other in self.fixture:self.contacts_seen.add(n+'_fixture')
                for s,fs in self.fingers.items():
                    if other in fs:self.contacts_seen.add(n+'_'+s)
            for s,fs in self.fingers.items():
                if (a in fs and b in self.fixture) or (b in fs and a in self.fixture):self.contacts_seen.add('gripper_fixture')
                if (a in fs or b in fs) and any('table' in (self.env.model.geom(g).name or '') for g in (a,b)):self.contacts_seen.add('finger_table')
    def requested(self,requested):
        e=self.env;result={}
        for s,p in requested.items():
            base=e.data.body(s+'_base_link');world=base.xpos+base.xmat.reshape(3,3)@p
            result[s]={n+'_m':float(np.linalg.norm(world-e.data.body(n).xpos)) for n in ('bolt','nut')}
            result[s]['hole_m']=float(np.linalg.norm(world-HOLE))
        return result
    def row(self,call,index,requested_dist):
        e=self.env;b=e.data.body('bolt');axis=b.xmat.reshape(3,3)[:,2];tip=b.xpos+LENGTH*axis
        radial=np.linalg.norm((tip-HOLE)[1:]);aligned=np.dot(axis,[-1,0,0])>np.cos(np.deg2rad(10))
        depth=float(np.clip(.037-tip[0],0,.074)) if radial<.002 and aligned else 0.
        grasp={}
        for n in ('bolt','nut'):
            between={}
            for s in e.sides:
                pts=[e.data.geom_xpos[g]+e.data.geom_xmat[g].reshape(3,3)@p for g,p in self.tip_local[s]]
                a,bp=pts;axis=bp-a;length=np.linalg.norm(axis)
                p=e.data.body(n).xpos.copy()
                if n=='bolt':p-=.010*e.data.body('bolt').xmat.reshape(3,3)[:,2]
                u=np.dot(p-a,axis)/max(length**2,1e-9)
                between[s]=bool(-.15<=u<=1.15 and np.linalg.norm(p-(a+u*axis))<.045)
            grasp[n]=dict(lift_m=float(e.data.body(n).xpos[2]-self.start[n]),between_fingers=between,
                          grasped=bool(e.data.body(n).xpos[2]>self.start[n]+.03 and any(n+'_'+s in self.contacts_seen for s in e.sides) and any(between.values())))
        t=e.thread
        row=dict(call=call,row=index,bolt_xyz=b.xpos.tolist(),nut_xyz=e.data.body('nut').xpos.tolist(),bolt_inserted_depth_m=depth,bolt_through_both=depth>=.0739,nut_engaged=t.engaged,nut_turns=t.turns,nut_advance_m=t.advance,grasp=grasp,tool_grasped=any(g['grasped'] for g in grasp.values()),tool_fixture_contact=any(n+'_fixture' in self.contacts_seen for n in ('bolt','nut')),gripper_fixture_contact='gripper_fixture' in self.contacts_seen,finger_table_contact='finger_table' in self.contacts_seen,contacts=sorted(self.contacts_seen),requested_hand=requested_dist)
        self.contacts_seen.clear();return row
