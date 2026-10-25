from pathlib import Path
import os,json,xml.etree.ElementTree as E
os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
ROOT=Path(__file__).resolve().parents[1]
U=E.parse(ROOT/'assets/kinematics.urdf').getroot(); J=list(U.findall('joint')); LINKS={x.get('name'):x for x in U.findall('link')}
f=lambda v:' '.join(str(float(x)) for x in v)
def vec(e,attr,default='0 0 0'):return np.fromstring(e.get(attr,default),sep=' ')
def origin(e):
 t=np.eye(4)
 if e is not None:t[:3,3]=vec(e,'xyz');t[:3,:3]=R.from_euler('xyz',vec(e,'rpy')).as_matrix()
 return t
JT={j.get('name'):origin(j.find('origin')) for j in J}
NAMES={s:[s+'_'+n for n in ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_yaw','wrist_roll']] for s in ['left','right']}
JD={j.get('name'):j for j in J}
def fk(side,q):
 vals=dict(zip(NAMES[side],q)); frames={'rov_base':np.eye(4)}
 for j in J:
  n=j.get('name'); p=j.find('parent').get('link');c=j.find('child').get('link');mot=np.eye(4)
  if n in vals:mot[:3,:3]=R.from_rotvec(vec(j.find('axis'),'xyz')*vals[n]).as_matrix()
  frames[c]=frames[p]@JT[n]@mot
 return frames[side+'_tool0']
def inverse(side,pos,rot,seed=None):
 lo=np.array([float(JD[n].find('limit').get('lower')) for n in NAMES[side]]); hi=np.array([float(JD[n].find('limit').get('upper')) for n in NAMES[side]])
 def res(q):
  t=fk(side,q);return np.r_[5*(t[:3,3]-pos),R.from_matrix(rot.T@t[:3,:3]).as_rotvec()]
 sol=least_squares(res,np.clip(seed if seed is not None else (lo+hi)/2,lo+1e-8,hi-1e-8),bounds=(lo,hi),max_nfev=300,xtol=1e-10,ftol=1e-10,gtol=1e-10)
 return sol.x,np.linalg.norm(sol.fun)
