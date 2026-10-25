"""Flange-only YAM comparator: stock vision/cameras/servo, matched holding model.
Stock YAM arm force bounds are 40 Nm; retain them, as in the propeller control.
Each independent jaw receives 80 N, matching Viscous's 160 N generalized drive.
"""
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from flange_scene import add_scene,ThreadModel
from flange_holding import Grasps


def yam_env():
    from dream_examples.yam_mujoco.yam_env import YamEnv,build_xml
    class Env(YamEnv):
        def __init__(self):
            root=add_scene(ET.fromstring(build_xml()))
            root.find('size').set('memory','1536M')
            eq=root.find('equality')
            if eq is None:eq=ET.SubElement(root,'equality')
            self.hand_bodies={}
            for s in ('left','right'):
                site=root.find(f".//site[@name='{s}_tcp']")
                parent=next(b for b in root.iter('body') if site in list(b))
                self.hand_bodies[s]=parent.get('name')
                for sign in (-1,1):root.find(f"actuator/position[@name='{s}_finger_{sign}']").set('forcerange','-80 80')
                for part in ('bolt','nut'):
                    ET.SubElement(eq,'weld',name=f'grasp_{s}_{part}',body1=parent.get('name'),body2=part,active='false',solref='.002 1')
            ET.SubElement(eq,'weld',name='flange_bolt_fixture',body1='world',body2='bolt',active='false',solref='.002 1')
            self.model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
            self.data=mujoco.MjData(self.model);self.renderer=None
            self.sides=('left','right');self.single=self.parked=self.copy_right=self.fixed_right=self.ignore_right=False
            self.rig='yam';self.base_suffix='_base';self.grip_open=.0475
            self.qadr={s:np.array([self.model.joint(f'{s}_joint{i}').qposadr[0] for i in range(1,7)]) for s in self.sides}
            self.aids={s:np.array([self.model.actuator(f'{s}_joint{i}').id for i in range(1,7)]) for s in self.sides}
            self.reset();self.grasps=Grasps(self,True);self.thread=ThreadModel(self)
            def observe():self.grasps.step();self.thread.step()
            self.row_observer=observe
        def step(self,left,right):
            self.gripper_command={s:float(a[6]) for s,a in [('left',left),('right',right)]}
            for s,a in [('left',left),('right',right)]:
                a=np.asarray(a,float)
                assert a.shape==(7,) and np.isfinite(a).all() and 0<=a[6]<=1
                for i,q in enumerate(a[:6],1):self.data.actuator(f'{s}_joint{i}').ctrl[0]=np.clip(q,*self.model.joint(f'{s}_joint{i}').range)
                for sign in (-1,1):self.data.actuator(f'{s}_finger_{sign}').ctrl[0]=a[6]*self.grip_open
            for _ in range(25):
                mujoco.mj_step(self.model,self.data)
                self.row_observer()
            mujoco.mj_forward(self.model,self.data)
            return self.state()
    return Env()
