"""Parked Maker-camera configuration with per-tick task observers."""
import mujoco
import numpy as np
from sweep_scene import SweepEnv,PARK
from cross_scenes import OPEN
from shackle_scene import CONFIG,Grasps,ThreadModel
class ShackleEnv(SweepEnv):
    def __init__(self,config=None):
        super().__init__('shackle',CONFIG)
        self.data.qpos[self.qadr['left']]=PARK[:6]
        self.data.ctrl[self.aids['left']]=PARK[:6]
        mujoco.mj_forward(self.model,self.data)
        self.grasps=Grasps(self);self.thread=ThreadModel(self)
        def observer():
            mujoco.mj_kinematics(self.model,self.data)
            self.grasps.step();self.thread.step()
        self.row_observer=observer
    def step(self,left,right):
        a=np.asarray(left,float)
        if a.shape!=(7,) or not np.isfinite(a).all() or not 0<=a[6]<=1:raise ValueError('Bad left action')
        m,d=self.model,self.data
        lim=np.array([m.joint(f'left_joint{i}').range for i in range(1,7)])
        q=np.clip(a[:6],lim[:,0],lim[:,1]);self.gripper_command={'left':float(a[6]),'right':1.}
        for _ in range(50):
            for s,target in [('left',q),('right',PARK[:6])]:
                aids=self.aids[s];d.ctrl[aids]=target+d.qfrc_bias[self.vadr[s]]/m.actuator_gainprm[aids,0]
                f=m.actuator(s+'_gripper_left_joint_drive').id
                d.ctrl[f]=(a[6] if s=='left' else 1)*OPEN
            mujoco.mj_step(m,d)
            self.parked_max_solver_deviation=max(self.parked_max_solver_deviation,float(np.max(np.abs(d.qpos[self.park_qadr]-self.park_values))))
            d.qpos[self.park_qadr]=self.park_values;d.qvel[self.park_vadr]=0
            self.row_observer()
        mujoco.mj_forward(m,d);return self.state()
