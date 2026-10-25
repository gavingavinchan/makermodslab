"""Flange-only holding. 160 N is an ideal CAD-linkage estimate, not calibration.

Crank r=30 mm, rod L=42 mm, slider-line offset e=16 mm, derived from
CAD bearing centres (parts 20,26/27,29/30). For slider z = r sin(t) -
sqrt(L²-(e-r cos(t))²), virtual work gives Q=torque/|dz/dt|.
Over the CAD-connected branch and 0..24 mm per-jaw opening, max |dz/dt|
is 30.57 mm/rad: 5/.03057 = 163.56 N generalized, rounded down to 160 N.
The symmetric-slider equality splits Q approximately equally across jaws.
No measured transmission efficiency, physical gap or hardware claim.

Optional fallback: actual bilateral jaw/part contact while commanded closed
activates a MuJoCo weld at its current relative pose. Opening command >.65
releases immediately at the next 1 ms observer tick. No proximity-only attach,
object teleport, arm torque increase or fixture movement. Events are durable
in episode row logs. This surrogate eliminates local grasp slip, not arm load.
"""
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

FORCE_N=160.
RELEASE=.65

def configure(root):
    eq=root.find('equality')
    for side in ('left','right'):
        root.find(f"actuator/position[@name='{side}_gripper_left_joint_drive']").set('forcerange',f'-{FORCE_N:g} {FORCE_N:g}')
        for part in ('bolt','nut'):
            ET.SubElement(eq,'weld',name=f'grasp_{side}_{part}',body1=side+'_wrist_roll_link',body2=part,active='false',solref='.002 1')
    ET.SubElement(eq,'weld',name='flange_bolt_fixture',body1='bolt',active='false',solref='.002 1')

class Grasps:
    def __init__(self,e,enabled=False):
        self.e=e;self.enabled=enabled;self.events=[];self.active={}
        m=e.model
        self.jaws={s:[{g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name==s+'_'+jaw and (m.geom_contype[g] or m.geom_conaffinity[g])} for jaw in ('gripper_left','gripper_right')] for s in e.sides}
        self.parts={p:{g for g in range(m.ngeom) if m.geom_bodyid[g]==m.body(p).id} for p in ('bolt','nut')}
    def command(self,s):
        return getattr(self.e, 'gripper_command', {}).get(s, float(self.e.data.actuator(s+'_gripper_left_joint_drive').ctrl[0]/.0524125))
    def attach(self,s,p):
        e=self.e;m,d=e.model,e.data
        hand=d.body(s+'_wrist_roll_link');obj=d.body(p)
        R=hand.xmat.reshape(3,3);rel=R.T@(obj.xpos-hand.xpos);rot=R.T@obj.xmat.reshape(3,3)
        k=m.equality(f'grasp_{s}_{p}').id
        m.eq_data[k,3:6]=rel;mujoco.mju_mat2Quat(m.eq_data[k,6:10],rot.ravel());d.eq_active[k]=1
        self.active[s]=p;self.events.append(dict(time=float(d.time),event='attach',side=s,part=p,relative_xyz=rel.tolist(),relative_quat=m.eq_data[k,6:10].tolist(),bilateral_contact=True))
    def step(self):
        if not self.enabled:return
        e=self.e;m,d=e.model,e.data
        for s,p in list(self.active.items()):
            if self.command(s)>RELEASE:
                d.eq_active[m.equality(f'grasp_{s}_{p}').id]=0;del self.active[s]
                self.events.append(dict(time=float(d.time),event='release',side=s,part=p,command=self.command(s)))
        for s in e.sides:
            opening=sum(d.joint(f'{s}_finger_{k}').qpos[0] for k in (-1,1))/(2*.0524125)
            if s in self.active or self.command(s)>.45 or self.command(s)>=opening-.001:continue
            for p,parts in self.parts.items():
                if p in self.active.values():continue
                touched=set()
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2)
                    for j,ids in enumerate(self.jaws[s]):
                        if (a in parts and b in ids) or (b in parts and a in ids):touched.add(j)
                if len(touched)==2:self.attach(s,p);break
