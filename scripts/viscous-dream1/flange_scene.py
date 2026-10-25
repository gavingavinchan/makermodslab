"""Fixed pipe-flange fixture, real clearance contacts and an ideal thread manifold.
No policy, network or hardware access. All lengths are metres.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent
ASSETS = HERE/'assets/flange'
CENTER = np.array([0., -.24, .965])
HOLE = CENTER + [0., .125*np.cos(np.pi/8), -.125*np.sin(np.pi/8)]
BOLT = np.array([.12, -.04, .787])
NUT = np.array([-.07, -.04, .774])
PITCH = .0035
LENGTH = .120
NUT_HALF = .013
PROMPT = 'insert the bolt through the flange holes and screw the nut onto the other end'


def numbers(v): return ' '.join(f'{x:.9g}' for x in np.asarray(v).ravel())


def generate_assets():
    """Regenerate v2 visuals/textures and convex contacts, entirely locally."""
    from flange_assets_v2 import generate
    generate()
    return len(list((ASSETS/'v2_collision').glob('flange_*.stl')))


def add_scene(root):
    if not (ASSETS/'v2.version').exists():
        from flange_assets_v2 import generate
        generate()
    root.find('size').set('memory','512M')
    world,asset=root.find('worldbody'),root.find('asset')
    for name in ('apple','orange','bowl','banana','water_bottle','plate'):
        node=world.find(f"body[@name='{name}']")
        if node is not None:world.remove(node)
    # Realistic coating palette; explicit mesh UVs, light wear, satin highlights.
    for name,texture,spec,shine,reflect in [
        ('flange_epoxy','epoxy_grey','.45','.32','.03'),
        ('pipe_coating','pipe_fbe','.24','.22','0'),
        ('pipe_steel','steel_worn','.7','.55','.06'),
        ('zinc','ptfe_blue','.50','.42','.02')]:
        ET.SubElement(asset,'texture',name=name+'_texture',type='2d',file=str(ASSETS/(texture+'.png')))
        ET.SubElement(asset,'material',name=name,texture=name+'_texture',texuniform='false',rgba='1 1 1 1',specular=spec,shininess=shine,reflectance=reflect)
    for name,rgba in [('stand_steel','.13 .16 .18 1'),('gasket_steel','.19 .22 .23 1'),('weld_steel','.28 .31 .30 1'),('marking','.85 .68 .20 1')]:
        ET.SubElement(asset,'material',name=name,rgba=rgba,specular='.45',shininess='.35')
    def mesh(name,path,smooth=False,**kw):
        ET.SubElement(asset,'mesh',name=name,file=str(path),smoothnormal=str(smooth).lower(),**kw)
    for name,file in [('flange_visual','flange_v2.obj'),('pipe_visual','pipe_v2.obj'),('coating_visual','coating_v2.obj'),('hub_visual','hub_v2.obj'),('weld_visual','weld_v2.obj'),('raised_visual','raised_face_v2.obj'),('gasket_visual','gasket_v2.obj'),('nut_chamfered','nut_v2.obj')]:
        mesh(name,ASSETS/file,**({'scale':'1 1 .9166666667'} if name=='flange_visual' else {}))
    cells=sorted((ASSETS/'v2_collision').glob('flange_*.stl'))
    for i,p in enumerate(cells):mesh(f'flange_cell_{i}',p)
    def geom(body,name,**kw):
        return ET.SubElement(body,'geom',name=name,**{k:str(v) for k,v in kw.items()})
    def visual(body,name,meshname,material,**kw):
        return geom(body,name,type='mesh',mesh=meshname,material=material,mass=0,contype=0,conaffinity=0,group=2,**kw)
    # Preserve the task frame and 74 mm flange stack; extend only outboard.
    for sign in (-1,1):
        body=ET.SubElement(world,'body',name=f'flange_{sign}',pos=numbers(CENTER+[sign*.0205,0,0]),euler=f'0 {np.pi/2} 0')
        visual(body,f'flange_visual_{sign}','flange_visual','flange_epoxy')
        for i in range(len(cells)):
            geom(body,f'flange_contact_{sign}_{i}',type='mesh',mesh=f'flange_cell_{i}',rgba='0 0 0 0',group=3,margin='.0001')
        hub=ET.SubElement(world,'body',name=f'pipe_hub_{sign}',pos=numbers(CENTER+[sign*.037,0,0]),euler=f'0 {sign*np.pi/2} 0')
        visual(hub,f'hub_visual_{sign}','hub_visual','flange_epoxy')
        # The taper is a physical annulus, decomposed without capping the bore.
        for k in range(32):
            a,b=2*np.pi*np.array([k,k+1])/32
            verts=[[r*np.cos(t),r*np.sin(t),z] for z,ro in ((0,.092),(.008,.091),(.018,.086),(.070,.063),(.080,.0603)) for t in (a,b) for r in (.0525,ro)]
            name=f'pipe_hub_sector_{sign}_{k}';ET.SubElement(asset,'mesh',name=name,vertex=numbers(verts))
            geom(hub,name,type='mesh',mesh=name,rgba='0 0 0 0',group=3)
        pipe=ET.SubElement(world,'body',name=f'pipe_{sign}',pos=numbers(CENTER+[sign*.417,0,0]),euler=f'0 {sign*np.pi/2} 0')
        visual(pipe,f'pipe_visual_{sign}','pipe_visual','pipe_steel')
        visual(pipe,f'coating_visual_{sign}','coating_visual','pipe_coating')
        visual(pipe,f'weld_visual_{sign}','weld_visual','weld_steel',pos='0 0 -.300')
        for k in range(32):
            a,b=2*np.pi*np.array([k,k+1])/32
            # Metal wall and coating both participate in contact; bore stays open.
            verts=[[r*np.cos(t),r*np.sin(t),z] for z in (-.300,.300) for t in (a,b) for r in (.0525,.0633)]
            name=f'pipe_sector_{sign}_{k}';ET.SubElement(asset,'mesh',name=name,vertex=numbers(verts))
            geom(pipe,name,type='mesh',mesh=name,rgba='0 0 0 0',group=3)
        # Two grounded saddle stands per section, away from the insertion corridor.
        for station in (.28,.55):
            key=f'{sign}_{int(station*100)}'
            stand=ET.SubElement(world,'body',name=f'pipe_stand_{key}',pos=numbers(CENTER+[sign*station,0,0]))
            geom(stand,f'stand_base_{key}',type='box',pos='0 0 -.197',size='.045 .075 .008',material='stand_steel')
            geom(stand,f'stand_post_{key}',type='box',pos='0 0 -.137',size='.014 .028 .055',material='stand_steel')
            for sy in (-1,1):
                geom(stand,f'vblock_{key}_{sy}',type='box',pos=f'0 {sy*.047} -.069',size='.037 .012 .030',euler=f'{-sy*.60} 0 0',material='stand_steel')
        # A machined raised face with a dark ring gasket visible at its outer edge.
        raised=ET.SubElement(world,'body',name=f'flange_raised_{sign}',pos=numbers(CENTER+[sign*.002867,0,0]),euler=f'0 {-sign*np.pi/2} 0')
        name=f'raised_scaled_{sign}';mesh(name,ASSETS/'raised_face_v2.obj',scale='1 1 1.133333333')
        visual(raised,f'raised_face_{sign}',name,'pipe_steel')
    gasket=ET.SubElement(world,'body',name='flange_gasket',pos=numbers(CENTER),euler=f'0 {np.pi/2} 0')
    mesh('gasket_scaled',ASSETS/'gasket_v2.obj',scale='1.06122449 1.06122449 .6')
    visual(gasket,'gasket','gasket_scaled','gasket_steel')
    # Seven M30 studs with two heavy hex nuts each. The lower arm-side hole stays empty.
    for k in range(8):
        angle=k*np.pi/4+np.pi/8
        if k==1:continue
        xyz=CENTER+[0,.125*np.sin(angle),-.125*np.cos(angle)]
        stud=ET.SubElement(world,'body',name=f'flange_stud_{k}',pos=numbers(xyz),euler=f'0 {np.pi/2} 0')
        geom(stud,f'stud_{k}',type='cylinder',size='.015 .078',material='zinc')
        for sign in (-1,1):
            visual(stud,f'stud_nut_{k}_{sign}','nut_chamfered','zinc',pos=f'0 0 {sign*.050}')
            # Convex sectors preserve the physical hex and bore of each installed nut.
            for j in range(48):
                name=f'installed_nut_cell_{j}'
                if asset.find(f"mesh[@name='{name}']") is None:mesh(name,ASSETS/f'nut_cell_{j:02d}.stl')
                geom(stud,f'stud_nut_contact_{k}_{sign}_{j}',type='mesh',mesh=name,pos=f'0 0 {sign*.050}',rgba='0 0 0 0',group=3)
        for j in range(44):
            z=-.076+j*PITCH
            geom(stud,f'stud_thread_{k}_{j}',type='cylinder',pos=f'0 0 {z}',size='.01525 .00035',material='pipe_steel',contype=0,conaffinity=0)
    # 33 mm rings, 8 mm root gap, two 3.4 mm raised faces and a 1.2 mm gasket.
    # The outer contact faces remain at x +/-37 mm; the task stack stays 74 mm.
    bolt=ET.SubElement(world,'body',name='bolt',pos=numbers(BOLT),euler=f'0 {-np.pi/2} 0')
    ET.SubElement(bolt,'freejoint',name='bolt_free')
    mesh('bolt_head',ASSETS/'head.stl')
    mesh('bolt_head_chamfered',ASSETS/'head_v2.obj')
    visual(bolt,'bolt_head_chamfered','bolt_head_chamfered','zinc')
    geom(bolt,'bolt_head',type='mesh',mesh='bolt_head',mass='.13',rgba='0 0 0 0',group=3,friction='1.5 .02 .002')
    geom(bolt,'bolt_shank',type='cylinder',pos='0 0 .057',size='.015 .057',mass='.50',material='zinc',friction='.5 .01 .001')
    geom(bolt,'bolt_tip',type='cylinder',pos='0 0 .117',size='.0138 .003',mass='.02',material='zinc')
    # Fine thread crests are visual only. The shank is the contact envelope.
    for k in range(20):
        geom(bolt,f'bolt_thread_{k}',type='cylinder',pos=f'0 0 {.047+k*PITCH}',size='.01525 .00040',mass=0,material='pipe_steel',contype=0,conaffinity=0)
    nut=ET.SubElement(world,'body',name='nut',pos=numbers(NUT))
    ET.SubElement(nut,'freejoint',name='nut_free')
    mesh('nut_visual',ASSETS/'nut_v2.obj')
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
        self.event_cursor=0
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
            bolt=e.data.body('bolt');tip=bolt.xpos+LENGTH*bolt.xmat.reshape(3,3)[:,2]
            result[s]['bolt_tip_m']=float(np.linalg.norm(world-tip))
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
                          grasped=bool(n in getattr(getattr(e,'grasps',None),'active',{}).values() or (e.data.body(n).xpos[2]>self.start[n]+.03 and any(n+'_'+s in self.contacts_seen for s in e.sides) and any(between.values()))))
        t=e.thread
        row=dict(call=call,row=index,bolt_xyz=b.xpos.tolist(),nut_xyz=e.data.body('nut').xpos.tolist(),bolt_inserted_depth_m=depth,bolt_through_both=depth>=.0739,nut_engaged=t.engaged,nut_turns=t.turns,nut_advance_m=t.advance,grasp=grasp,tool_grasped=any(g['grasped'] for g in grasp.values()),tool_fixture_contact=any(n+'_fixture' in self.contacts_seen for n in ('bolt','nut')),gripper_fixture_contact='gripper_fixture' in self.contacts_seen,finger_table_contact='finger_table' in self.contacts_seen,contacts=sorted(self.contacts_seen),requested_hand=requested_dist)
        events=e.grasps.events[self.event_cursor:] if hasattr(e,'grasps') else []
        self.event_cursor+=len(events)
        row['grasp_events']=events
        row['grasp_attached']=dict(e.grasps.active) if hasattr(e,'grasps') else {}
        self.contacts_seen.clear();return row
