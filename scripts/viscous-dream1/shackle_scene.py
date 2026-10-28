"""Real-scale glued shackle station; task-only contact grasp and ideal screw.
No network or inference. Task-only lighting; original station cameras unchanged.
"""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from flange_scene import numbers
from flange_holding import Grasps as FlangeGrasps

HERE=Path(__file__).resolve().parent
ASSETS=HERE/'assets/shackle'
OUT=HERE/'runs/shackle_scene'
TABLE=.76
CENTER=np.array([.245,.1425,TABLE+.04924987])
PIN=np.array([.305,.040,TABLE+.00015])
# Left base is (.305,.240); its rest-pose tool projects along -Y.
# The bow extends along mesh -X. Rotate both glued parts together so -X
# points radially away from that base, leaving the ears on the near side.
FIXTURE_YAW=float(np.arctan2(.0975,.060))
FIXTURE_QUAT=np.array([np.cos(FIXTURE_YAW/2),0,0,np.sin(FIXTURE_YAW/2)])
PITCH=.0025
UNDERHEAD=.07985
TOP=CENTER[2]+.039925
MOUTH=CENTER[2]-.020574
SEAT=CENTER[2]-.039925
PROMPT='pick up the pin and screw it into the shackle'
CONFIG=dict(tcp='pad',arms='parked',rows=48,region='off',wrist_r='normal')

def camera(world,name,eye,target,fovy):
    z=np.array(eye)-target;z/=np.linalg.norm(z);x=np.cross([0,0,1],z);x/=np.linalg.norm(x)
    ET.SubElement(world,'camera',name=name,pos=numbers(eye),xyaxes=numbers(np.r_[x,np.cross(z,x)]),fovy=str(fovy))

def add_scene(root):
    w,a=root.find('worldbody'),root.find('asset')
    # Task-only key from the inspection side: move the shadow terminator off
    # the visible bow. Resolution alone leaves serrated self-shadow boundaries.
    # Keep the original light colours, fill, materials, meshes and all cameras.
    key=w.find('light')
    key.set('pos','.8 -.7 2.0');key.set('dir','-.6 .8 -1.2')
    key.set('castshadow','true')
    visual=root.find('visual')
    quality=visual.find('quality')
    if quality is None:quality=ET.SubElement(visual,'quality')
    quality.set('shadowsize','8192')
    mapping=visual.find('map')
    if mapping is None:mapping=ET.SubElement(visual,'map')
    mapping.set('shadowscale','.8')
    root.find('size').set('memory','512M')
    for n in ('apple','orange','bowl','banana','water_bottle','plate'):
        b=w.find(f"body[@name='{n}']")
        if b is not None:w.remove(b)
    ET.SubElement(a,'material',name='shackle_stainless',rgba='.63 .67 .70 1',specular='.7',shininess='.55',reflectance='.06')
    ET.SubElement(a,'material',name='holder_pla',rgba='.12 .29 .39 1',specular='.08',shininess='.05')
    for n in ('body','pin','holder_sim'):
        ET.SubElement(a,'mesh',name='shackle_'+n,file=str(ASSETS/f'shackle_{n}.obj'),smoothnormal='false')
    def visual(b,n,mat):
        ET.SubElement(b,'geom',name=n+'_visual',type='mesh',mesh=n,material=mat,contype='0',conaffinity='0',mass='0',group='2')
    body=ET.SubElement(w,'body',name='shackle',pos=numbers(CENTER),quat=numbers(FIXTURE_QUAT))
    visual(body,'shackle_body','shackle_stainless')
    for i,p in enumerate(sorted((ASSETS/'collision').glob('*.stl'))):
        n=f'shackle_cell_{i}';ET.SubElement(a,'mesh',name=n,file=str(p))
        ET.SubElement(body,'geom',name=n,type='mesh',mesh=n,rgba='0 0 0 0',group='3',margin='0',friction='.5 .01 .001')
    holder=ET.SubElement(w,'body',name='shackle_holder',pos=numbers(CENTER),quat=numbers(FIXTURE_QUAT))
    visual(holder,'shackle_holder_sim','holder_pla')
    # Static mesh's convex envelope is sufficient: the entire holder is >68 mm
    # from the insertion axis. Both body trees have no joints: welded to world.
    ET.SubElement(holder,'geom',name='holder_contact',type='mesh',mesh='shackle_holder_sim',rgba='0 0 0 0',group='3')
    pin=ET.SubElement(w,'body',name='pin',pos=numbers(PIN))
    ET.SubElement(pin,'freejoint',name='pin_free')
    # Mesh-derived stainless mass/inertia, not convex-hull density.
    import trimesh
    mesh=trimesh.load(ASSETS/'shackle_pin.stl',force='mesh');I=mesh.moment_inertia*7900
    ET.SubElement(pin,'inertial',pos=numbers(mesh.center_mass),mass=str(mesh.volume*7900),fullinertia=numbers([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
    visual(pin,'shackle_pin','shackle_stainless')
    for n,z,r,h in [('thread',.011115,.010,.011115),('shank',.05104,.009845,.02881),('collar',.08185,.01384,.002)]:
        ET.SubElement(pin,'geom',name='pin_'+n,type='cylinder',pos=f'0 0 {z}',size=f'{r} {h}',rgba='0 0 0 0',group='3',friction='1.5 .02 .002')
    # Tab collision preserves silhouette flats; cross-hole is visual only.
    ET.SubElement(pin,'geom',name='pin_tab',type='box',pos='0 0 .094',size='.012 .004595 .010',rgba='0 0 0 0',group='3',friction='1.5 .02 .002')
    camera(w,'shackle_overview',[.85,-.72,1.23],np.array([.16,.08,.83]),42)
    camera(w,'shackle_plan',[.181,.101,1.65],np.array([.18,.10,.80]),35)
    camera(w,'shackle_detail',[.44,-.13,1.02],np.array([.20,.18,TABLE+.04924987+.048]),28)
    root.find('visual/global').set('offwidth','1280');root.find('visual/global').set('offheight','720')
    return root

def configure(root):
    ET.SubElement(root.find('equality'),'weld',name='shackle_seat_stop',body1='world',body2='pin',active='false',solref='.003 1',solimp='.98 .999 .0001')
    for s in ('left','right'):
        for drive in root.findall('actuator/position'):
            if drive.get('joint') in {f'{s}_joint{i}' for i in range(1,7)}:drive.set('forcerange','-25 25')
        root.find(f"actuator/position[@name='{s}_gripper_left_joint_drive']").set('forcerange','-160 160')
        ET.SubElement(root.find('equality'),'weld',name=f'grasp_{s}_pin',body1=s+'_wrist_roll_link',body2='pin',active='false',solref='.002 1')

class Grasps(FlangeGrasps):
    def __init__(self,e):
        self.e=e;self.enabled=True;self.events=[];self.active={};m=e.model
        self.jaws={s:[{g for g in range(m.ngeom) if m.geom_bodyid[g]==m.joint(f'{s}_finger_{k}').bodyid[0] and (m.geom_contype[g] or m.geom_conaffinity[g])} for k in (-1,1)] for s in e.sides}
        self.parts={'pin':{g for g in range(m.ngeom) if m.geom_bodyid[g]==m.body('pin').id}}

class ThreadModel:
    """Flange-style 1 ms ideal manifold, gated on actual tip, axis and phase.
    Physical free rotation supplies spin. Right-hand spin -Z advances downward.
    No controller or grasp state drives entry, advance or release. Unengaged pin
    is never projected. Entry detects crossing of the phase-correct mouth plane between ticks.
    """
    def __init__(self,e):
        self.e=e;self.engaged=False;self.turns=0.;self.advance=0.;self.events=[];self.pull=0.;self.spin=0.;self.seat_latched=False
    def step(self):
        e=self.e;d=e.data;m=e.model;b=d.body('pin');R=b.xmat.reshape(3,3);p=b.xpos.copy()
        theta=float(np.arctan2(R[1,0],R[0,0]));radial=float(np.linalg.norm(p[:2]-CENTER[:2]));aligned=R[2,2]>np.cos(np.deg2rad(10))
        if not self.engaged:
            # Continuous crossing detection prevents a falling pin from stepping
            # over the narrow, phase-correct catch plane in a 1 ms integration.
            # Only the tapped mouth is eligible; the upper ear is plain contact.
            previous=getattr(self,'previous_tip',p[2]);self.previous_tip=p[2]
            phase_z=SEAT+PITCH*theta/(2*np.pi)
            candidates=[phase_z+k*PITCH for k in range(int((MOUTH-phase_z)/PITCH)-2,int((MOUTH-phase_z)/PITCH)+3)]
            catch=[z for z in candidates if abs(z-MOUTH)<.0012 and min(previous,p[2])-.00002<=z<=max(previous,p[2])+.00002]
            if aligned and radial<.003 and catch:
                z=min(catch,key=lambda z:abs(z-p[2]))
                self.engaged=True;self.theta=theta;self.total=0.;self.start=z;self.turns=self.advance=0.;self.pull=0.;self.last_advance=0.
                self.events.append(dict(time=float(d.time),event='engage',phase_error_deg=0.,radial_m=radial,substep_axial_correction_m=z-p[2]))
            return
        j=m.joint('pin_free');qa,va=j.qposadr[0],j.dofadr[0]
        seat_eq=m.equality('shackle_seat_stop').id
        if self.seat_latched:
            reverse=np.arctan2(np.sin(theta-self.theta),np.cos(theta-self.theta))
            if reverse>np.deg2rad(.5):
                d.eq_active[seat_eq]=0;self.seat_latched=False
                self.events.append(dict(time=float(d.time),event='unseat'))
            else:self.spin=0.;return
        # Same passive angular/axial momentum projection and light thread drag
        # as the flange Astra harness: .020 Nm breakaway, .015 Nm sliding,
        # .002 Nm s/rad damping. These are assumptions, not measured friction.
        dt=m.opt.timestep;h=PITCH/(2*np.pi);mass=float(m.body('pin').mass[0])
        axis=np.array([0.,0.,1.]);principal=d.ximat[b.id].reshape(3,3).T@axis
        inertia=float(np.sum(m.body('pin').inertia*principal**2));effective=inertia+mass*h*h
        free_spin=float((R@d.qvel[va+3:va+6])[2]);axial=float(d.qvel[va+2])
        momentum=inertia*free_spin+mass*h*axial
        if self.spin==0. and abs(momentum)<=.020*dt:spin=0.
        else:spin=float(np.sign(momentum)*max(0.,abs(momentum)-.015*dt)/(effective+.002*dt))
        self.spin=spin;self.theta+=dt*spin;self.total-=dt*spin
        self.turns=self.total/(2*np.pi);self.advance=PITCH*self.turns
        z=self.start-self.advance
        self.pull=max(0.,float(p[2]-(self.start-self.advance)))
        if self.turns<-.1 or p[2]>self.start+.003:
            self.engaged=False;self.previous_tip=p[2];self.events.append(dict(time=float(d.time),event='release',turns=self.turns,pull_m=self.pull));return
        if z<=SEAT+.00002 and spin<0:
            self.seat_latched=True;self.total=2*np.pi*(self.start-SEAT)/PITCH
            self.theta=2*np.pi*(SEAT-SEAT)/PITCH
            self.turns=self.total/(2*np.pi);self.advance=PITCH*self.turns;z=SEAT;self.spin=spin=0.
            m.eq_data[seat_eq,3:6]=[CENTER[0],CENTER[1],SEAT]
            m.eq_data[seat_eq,6:10]=[1,0,0,0];d.eq_active[seat_eq]=1
            self.events.append(dict(time=float(d.time),event='seat_stop',turns=self.turns))
        theta=self.theta;c,s=np.cos(theta),np.sin(theta)
        d.qpos[qa:qa+3]=[CENTER[0],CENTER[1],z]
        aligned_R=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        mujoco.mju_mat2Quat(d.qpos[qa+3:qa+7],aligned_R.ravel())
        d.qvel[va:va+3]=[0,0,h*spin];d.qvel[va+3:va+6]=[0,0,spin]
        mujoco.mj_forward(m,d)

class Measurements:
    def __init__(self,e):
        self.env=e;self.previous=e.row_observer;e.row_observer=self.contacts;self.seen=set();self.cursor=0
        self.peak={s:np.zeros(6) for s in e.sides};self.total={s:np.zeros(6) for s in e.sides}
        self.exposure={v:{s:np.zeros(6) for s in e.sides} for v in (5,14)}
    def contacts(self):
        self.previous();e=self.env
        for s in e.sides:
            t=np.abs(e.data.actuator_force[e.aids[s]]);self.peak[s]=np.maximum(self.peak[s],t);self.total[s]=np.maximum(self.total[s],t)
            for v in (5,14):self.exposure[v][s]+=(t>v)*e.model.opt.timestep
        for c in e.data.contact:
            names=[e.model.body(e.model.geom_bodyid[g]).name for g in (c.geom1,c.geom2)]
            if 'pin' in names or any('finger' in n or 'gripper' in n for n in names):self.seen.add('/'.join(sorted(names)))
    def requested(self,requested):return {}
    def row(self,call,index,requested_dist=None):
        e=self.env;b=e.data.body('pin');p=b.xpos;R=b.xmat.reshape(3,3)
        radial=float(np.linalg.norm(p[:2]-CENTER[:2]));aligned=R[2,2]>np.cos(np.deg2rad(10));gap=float(p[2]+UNDERHEAD*R[2,2]-TOP)
        events=e.grasps.events[self.cursor:];self.cursor=len(e.grasps.events)
        row=dict(call=call,row=index,time=float(e.data.time),pin_xyz=p.tolist(),pin_quat_wxyz=b.xquat.tolist(),pin_axis=R[:,2].tolist(),pin_radial_error_m=radial,
            inserted_depth_m=float(max(0,TOP-p[2])) if aligned and radial<.001 else 0.,inserted=bool(aligned and radial<.001 and p[2]<TOP-.019351),engaged=e.thread.engaged,turns=e.thread.turns,gap_to_seated_m=gap,
            seated=bool(e.thread.engaged and radial<.0005 and aligned and e.thread.seat_latched and abs(gap)<.0005),grasp_attached=dict(e.grasps.active),grasp_events=events,contacts=sorted(self.seen),
            arm_torque_peak_abs_Nm={s:t.tolist() for s,t in self.peak.items()})
        for s in e.sides:self.peak[s][:]=0
        self.seen.clear();return row
    def torque_summary(self):return dict(arm_torque_max_abs_Nm={s:t.tolist() for s,t in self.total.items()},arm_time_above_s={str(v):{s:t.tolist() for s,t in ss.items()} for v,ss in self.exposure.items()})
