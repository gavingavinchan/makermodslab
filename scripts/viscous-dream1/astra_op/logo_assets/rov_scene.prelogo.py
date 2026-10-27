"""Native, fixed BlueROV physics scene. Source URDF and task assets are read only."""
import copy,json,sys
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
URDF=Path.home()/'Documents/viscous-rov-model/model/viscous_rov.urdf'
CACHE=ROOT/'astra_op/underwater/native_assets'
REF=np.array(json.loads((URDF.parent/'manifest.json').read_text())['reference_arm_pose_rad'])
JN=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_yaw','wrist_roll')
# World is the level ROV frame translated vertically to put the keel on sand.
HEIGHT=1.005
# backup2 translates the whole level vehicle 20 mm to ROV-left. Fixture and
# seabed stay fixed; native mounts, meshes and joint/actuator limits stay native.
PARKING={'original':np.zeros(3),'backup2':np.array([0.,.020,0.])}
# Fixture transform: its pipe axis is transverse; the free nut faces right.
YAW=np.pi/3
FROT=R.from_euler('z',YAW).as_matrix()
# Selected by reach_check.py; task hole is in the open space ahead of both mounts.
HOLE_TARGET=np.array([.55,-.015,.94])
from flange_scene import HOLE
OFFSET=HOLE_TARGET-FROT@HOLE
f=lambda v:' '.join(f'{x:.12g}' for x in np.asarray(v).ravel())
def quat(r):return f(R.from_matrix(r).as_quat()[[3,0,1,2]])
def origin(el):
 return dict(pos=el.get('xyz','0 0 0'),quat=quat(R.from_euler('xyz',np.fromstring(el.get('rpy','0 0 0'),sep=' ')).as_matrix())) if el is not None else dict(pos='0 0 0')
def mesh(asset,path,name):
 CACHE.mkdir(parents=True,exist_ok=True);out=CACHE/(path.stem+'.obj')
 if not out.exists():
  import trimesh
  trimesh.load(path,process=False).export(out,file_type='obj',digits=12)
 E.SubElement(asset,'mesh',name=name,file=str(out))
def camera(parent,name,eye,target,fovy):
 z=np.array(eye)-target;z/=np.linalg.norm(z);x=np.cross([0,0,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
 return E.SubElement(parent,'camera',name=name,pos=f(eye),xyaxes=f(np.r_[x,y]),fovy=str(fovy))
def xml(source,bolt_free_spin=False,parking='original'):
 shift=PARKING[parking]
 root=E.fromstring(source);w=root.find('worldbody');a=root.find('asset');u=E.parse(URDF).getroot()
 oldarms={s:w.find(f"body[@name='{s}_base_link']") for s in ('left','right')}
 cams={s:copy.deepcopy(oldarms[s].find(f".//camera[@name='{s}']")) for s in oldarms}
 for b in oldarms.values():w.remove(b)
 # Remove room, table and top camera. Preserve every fixture body and material.
 for el in list(w):
  if el.tag in ('geom','light','camera'):w.remove(el)
 for b in list(w.findall('body')):
  if b.get('name') not in ('nut','bolt') and not b.get('name','').startswith(('pipe_','flange_')):w.remove(b)
 for b in w.findall('body'):
  p=np.fromstring(b.get('pos','0 0 0'),sep=' ');b.set('pos',f(FROT@p+OFFSET))
  rot=R.from_euler('xyz',np.fromstring(b.get('euler','0 0 0'),sep=' ')).as_matrix() if b.get('euler') else R.from_quat(np.fromstring(b.get('quat','1 0 0 0'),sep=' ')[[1,2,3,0]]).as_matrix()
  b.attrib.pop('euler',None);b.set('quat',quat(FROT@rot))
 links={l.get('name'):l for l in u.findall('link')};joints={j.find('child').get('link'):j for j in u.findall('joint')}
 def add(name,parent):
  j=joints.get(name);b=E.SubElement(parent,'body',name=name,**origin(j.find('origin') if j is not None else None))
  if name=='rov_base':b.set('pos',f(np.array([0,0,HEIGHT])+shift))
  if name in ('left_base_link','right_base_link'):b.set('childclass','viscous')
  inert=links[name].find('inertial')
  if inert is not None:
   i=inert.find('inertia');kw=origin(inert.find('origin'));kw.pop('quat',None)
   E.SubElement(b,'inertial',**kw,mass=inert.find('mass').get('value'),fullinertia=' '.join(i.get(k) for k in ('ixx','iyy','izz','ixy','ixz','iyz')))
  side=name.split('_')[0]
  if j is not None and j.get('type') in ('revolute','prismatic'):
   suffix=j.get('name')[len(side)+1:];jn=f'{side}_joint{JN.index(suffix)+1}' if suffix in JN else side+('_finger_-1' if suffix=='gripper_left_joint' else '_finger_1')
   lim=j.find('limit');E.SubElement(b,'joint',name=jn,type='slide' if j.get('type')=='prismatic' else 'hinge',axis=j.find('axis').get('xyz'),range=lim.get('lower')+' '+lim.get('upper'))
  for kind in ('visual','collision'):
   for k,v in enumerate(links[name].findall(kind)):
    path=URDF.parent/v.find('geometry/mesh').get('filename');mn='rov_'+v.get('name');mesh(a,path,mn)
    gn='native_'+v.get('name') if kind=='visual' else 'solid_'+v.get('name')
    # Measurements uses these two named distal jaw visuals for tip coordinates.
    if kind=='visual' and name.endswith(('gripper_left','gripper_right')) and k==0:gn=side+('_visual_043' if name.endswith('gripper_left') else '_visual_042')
    color=v.find('material/color');rgba=color.get('rgba') if color is not None else '.5 .5 .5 1'
    if kind=='collision' and side not in ('left','right'):
     # A single convex hull seals the open chassis. V-HACD keeps its openings.
     paths=sorted(CACHE.glob(path.stem+'_hull*.obj'))
     if not paths:
      import trimesh
      t=trimesh.load(path,process=True)
      parts=[t] if t.is_convex else t.convex_decomposition(maxConvexHulls=48,resolution=200000,maxRecursionDepth=10)
      for hi,hull in enumerate(parts):hull.export(CACHE/(path.stem+f'_hull{hi}.obj'))
      paths=sorted(CACHE.glob(path.stem+'_hull*.obj'))
     for hi,hp in enumerate(paths):
      import trimesh
      ht=trimesh.load(hp,process=False);extent=np.ptp(ht.vertices,axis=0)
      if min(extent)<1e-6 or np.linalg.matrix_rank(ht.vertices-ht.vertices.mean(0),tol=1e-7)<3:
       # Degenerate decomposition slivers receive finite, conservative thickness.
       E.SubElement(b,'geom',name=gn+f'_hull{hi}',type='box',pos=f(ht.bounds.mean(0)),size=f(np.maximum(extent/2,.0002)),rgba='0 0 0 0',group='3')
      else:
       hn=mn+f'_hull{hi}';E.SubElement(a,'mesh',name=hn,file=str(hp))
       E.SubElement(b,'geom',name=gn+f'_hull{hi}',type='mesh',mesh=hn,rgba='0 0 0 0',group='3',friction='1.5 .02 .002',**origin(v.find('origin')))
    else:
     E.SubElement(b,'geom',name=gn,type='mesh',mesh=mn,rgba=rgba if kind=='visual' else '0 0 0 0',group=('4' if name=='rov_base' and v.get('name')=='rov_base_51' else '2') if kind=='visual' else '3',contype='0' if kind=='visual' else '1',conaffinity='0' if kind=='visual' else '1',friction='.6 .005 .0001' if side in ('left','right') else '1.5 .02 .002',**origin(v.find('origin')))
  if name.endswith('tool0'):
   h=E.SubElement(b,'body',name=side+'_canonical_hand',quat='0 0 0 1' if side=='right' else '1 0 0 0')
   E.SubElement(h,'site',name=side+'_tcp',size='.003',rgba='0 0 0 0');h.append(cams[side])
  for child,jc in joints.items():
   if jc.find('parent').get('link')==name:add(child,b)
  return b
 vehicle=add('rov_base',w)
 # Old exclusions refer to old geom names. Keep only rigid adjacent-link exclusions.
 contact=root.find('contact');contact.clear()
 for side in oldarms:
  for el in [('base_link','yaw_link'),('yaw_link','upper_arm_link'),('upper_arm_link','forearm_link'),('forearm_link','wrist_pitch_link'),('wrist_pitch_link','wrist_yaw_link'),('wrist_yaw_link','wrist_roll_link'),('wrist_roll_link','gripper_left'),('wrist_roll_link','gripper_right')]:
   E.SubElement(contact,'exclude',body1=side+'_'+el[0],body2=side+'_'+el[1])
  E.SubElement(contact,'exclude',body1='rov_base',body2=side+'_base_link')
  E.SubElement(contact,'exclude',body1='rov_base',body2=side+'_yaw_link')
 # Collision meshes in the URDF include frame, foam, enclosures and thrusters.
 from PIL import Image
 rng=np.random.default_rng(720);tex=CACHE/'sand.png'
 if not tex.exists():Image.fromarray(np.clip(np.array([155,145,112])+np.random.default_rng(720).normal(0,7,(512,512,1)),0,255).astype('uint8')).save(tex)
 E.SubElement(a,'texture',name='sand_tex',type='2d',file=str(tex));E.SubElement(a,'material',name='sand_mat',texture='sand_tex',texrepeat='10 10',rgba='.65 .72 .62 1',specular='.05')
 E.SubElement(w,'geom',name='seabed',type='plane',pos='0 0 .76',size='12 12 .1',material='sand_mat',friction='1.2 .02 .002')
 # The work was raised 23 mm relative to the old tabletop. Low, flat sand
 # footings ground the unchanged saddle stands without lifting the ROV keel.
 rise=float(OFFSET[2])
 if rise>0:
  verts=[[x,y,z] for z,sx,sy in [(0,.10,.13),(rise,.065,.095)] for x in [-sx,sx] for y in [-sy,sy]]
  E.SubElement(a,'mesh',name='sand_footing',vertex=f(verts))
  for stand in w.findall('body'):
   if stand.get('name','').startswith('pipe_stand_'):
    p=np.fromstring(stand.get('pos'),sep=' ');p[2]=.76
    E.SubElement(w,'geom',name='sand_footing_'+stand.get('name'),type='mesh',mesh='sand_footing',pos=f(p),material='sand_mat',friction='1.2 .02 .002')
 visual=root.find('visual')
 for tag in ('map','rgba','headlight','quality'):
  n=visual.find(tag)
  if n is not None:visual.remove(n)
 stat=root.find('statistic')
 if stat is None:stat=E.SubElement(root,'statistic')
 stat.set('extent','1')
 E.SubElement(visual,'map',fogstart='.65',fogend='4.5',znear='.003',zfar='25');E.SubElement(visual,'rgba',fog='.025 .115 .145 1',haze='.02 .10 .13 1')
 E.SubElement(visual,'headlight',ambient='.24 .32 .33',diffuse='.4 .5 .52',specular='.3 .3 .3');E.SubElement(visual,'quality',shadowsize='2048',offsamples='4')
 for n in list(a.findall('texture')):
  if n.get('type')=='skybox':a.remove(n)
 E.SubElement(a,'texture',name='water',type='skybox',builtin='gradient',rgb1='.012 .055 .075',rgb2='.035 .13 .16',width='512',height='2048')
 E.SubElement(w,'light',pos='0 0 3',dir='0 0 -1',diffuse='.72 .85 .82',attenuation='1 .15 .05',castshadow='true')
 E.SubElement(vehicle,'light',pos='.20 0 -.03',dir='1 0 -.3',diffuse='.65 .75 .72',attenuation='1 .3 .3',castshadow='false')
 camera(vehicle,'rov',[.175,.0023,.1077],HOLE_TARGET-[0,0,HEIGHT]-shift,100)
 camera(w,'wide',[1.2,-1.25,1.5],np.array([.2,0,1.05]),55)
 if bolt_free_spin:
  camera(w,'flange_detail',HOLE_TARGET+.35*FROT[:,1]-.10*FROT[:,0]+[0,0,.15],HOLE_TARGET,45)
 else:
  camera(w,'flange_detail',[.28,-.25,1.35],HOLE_TARGET-.04*FROT[:,0],40)
 for i in range(75):
  p=rng.uniform([-1,-1,.82],[2,1,2.2])
  if np.linalg.norm(p-HOLE_TARGET)<.3:continue
  E.SubElement(w,'geom',name=f'snow_{i}',type='sphere',pos=f(p),size='.001',rgba='.43 .66 .65 .3',contype='0',conaffinity='0')
 used={g.get('mesh') for g in root.iter('geom') if g.get('mesh')}
 for el in list(a.findall('mesh')):
  if el.get('name') not in used:a.remove(el)
 from harness_env import seat_constraint
 seat_constraint(root)
 if bolt_free_spin:
  from bolt_spin import constraints
  constraints(root)
 return E.tostring(root,encoding='unicode')
def make_env(bolt_free_spin=False,parking='original'):
 import sweep_scene
 original=sweep_scene.viscous_xml
 try:
  sweep_scene.viscous_xml=lambda task:xml(original(task),bolt_free_spin,parking)
  e=sweep_scene.SweepEnv('flange',dict(tcp='pad',arms='both',rows=48,region='off',wrist_r='normal',flange_attach=True))
 finally:sweep_scene.viscous_xml=original
 e.rov=True;e.rov_parking=parking;e.flange_hole=HOLE_TARGET.copy();e.flange_face=HOLE_TARGET+FROT@np.array([-.037,0,0])
 return e

class NativeSolver:
 """Coordinate adapter around the unchanged installed native ClientIK solver."""
 def __init__(self,solver,e):
  from dataclasses import replace
  self.raw=solver;self.sides=solver.sides;self._groups=solver._groups
  self.sign=np.array([[-1,1,1,1,-1,-1] if s=='right' else [1]*6 for s in self.sides])
  for group in solver._groups:
   side=solver.sides[group.arms.start];idx=self.sides.index(side);limits=np.array([e.model.joint(f'{side}_joint{i}').range for i in range(1,7)])
   orig=limits*self.sign[idx,:,None]+REF[:,None];lo=np.min(orig,axis=1)+np.radians(.25);hi=np.max(orig,axis=1)-np.radians(.25)
   group.arm=replace(group.arm,lo=group.to_pyroki(lo),hi=group.to_pyroki(hi))
  solver.side_lo=np.stack([g.to_spec(g.arm.lo) for g in solver._side_groups]);solver.side_hi=np.stack([g.to_spec(g.arm.hi) for g in solver._side_groups])
  self.side_lo=np.stack([np.array([e.model.joint(f'{s}_joint{i}').range[0] for i in range(1,7)])+np.radians(.25) for s in self.sides])
  self.side_hi=np.stack([np.array([e.model.joint(f'{s}_joint{i}').range[1] for i in range(1,7)])-np.radians(.25) for s in self.sides])
  self.side_rest=(np.asarray(solver.side_rest)-REF)*self.sign
 def fk(self,q,s):return self.raw.fk(np.asarray(q)*self.sign[self.sides.index(s)]+REF,s)
 def solve(self,targets,q):
  from types import SimpleNamespace
  res=self.raw.solve(targets,np.asarray(q)*self.sign+REF)
  return SimpleNamespace(q_solution=(np.asarray(res.q_solution)-REF)*self.sign[:,None,:])

def initialize(e,args):
 from flange_start import local_ik,put
 from flange_ik_check import orientation
 from flange_scene import LENGTH
 from thread_model import RightHandThread,FrictionThread
 if args.condition!='G3':raise ValueError('ROV scene currently requires G3')
 ik=local_ik(e);records={}
 # Right hand is tip-side, approach along ROV-left, away from the solid frame.
 for s,p,axis in [('right',HOLE_TARGET-.18*FROT[:,0]+np.array([0,0,.006]),FROT[:,0]),('left',np.array([.5,.15,.96]),np.array([.62,-.6,-.35]))]:
  jaw=FROT@np.array([0,np.cos(np.radians(240)),np.sin(np.radians(240))]) if s=='right' else [0,0,-1]
  rot=orientation(axis,jaw);seed=np.array([-.069966,-.46242,2.0819,-1.1179,.76065,1.1744]) if s=='left' else np.array([-.1,.6,-1.33,1.75,1.15,-1.5708])
  q,res=ik.fit(s,p,rot,seed=seed,starts=0)
  if np.linalg.norm(res[:3])>.001:raise ValueError(f'{s} ROV start unreachable: {res}')
  e.data.qpos[e.qadr[s]]=q;e.data.ctrl[e.aids[s]]=q
  for sign in (-1,1):e.data.joint(f'{s}_finger_{sign}').qpos[0]=.0227 if s=='right' else .0524125
  mujoco.mj_forward(e.model,e.data)
  records[s]=dict(q=q.tolist(),target=p.tolist(),R=rot.tolist())
 h=e.data.site('right_tcp');H=h.xmat.reshape(3,3).copy()
 put(e,'nut',h.xpos+.040*H[:,2],H@R.from_euler('z',30,degrees=True).as_matrix())
 B=FROT@orientation([-1,0,0],[0,1,0]);put(e,'bolt',HOLE_TARGET+FROT@np.array([.05,0,0]),B)
 k=e.model.equality('flange_bolt_fixture').id;e.model.eq_data[k,3:6]=e.data.body('bolt').xpos;e.model.eq_data[k,6:10]=e.data.body('bolt').xquat;e.data.eq_active[k]=1
 e.start_info=dict(start='inserted',seed=args.seed,hands=records,bolt_fixture_held=True,g3_extra_insertion_m=.03,g3_grasp='flats',native_rov=True)
 if getattr(e,'rov_parking','original')!='original':
  e.start_info.update(rov_parking=e.rov_parking,rov_translation_m=PARKING[e.rov_parking].tolist(),rov_pitch_roll_deg=[0.,0.])
 e.thread=RightHandThread(e) if args.thread_hand=='right' else FrictionThread(e,'legacy')
 if getattr(args,'bolt_free_spin',False):
  from bolt_spin import initialize as spin_initialize
  spin_initialize(e)
 return 'inserted'

from flange_scene import Measurements,LENGTH
class ROVMeasurements(Measurements):
 def row(self,*args):
  row=super().row(*args);b=self.env.data.body('bolt');axis=b.xmat.reshape(3,3)[:,2];tip=b.xpos+LENGTH*axis
  delta=FROT.T@(tip-HOLE_TARGET);aligned=np.dot(axis,-FROT[:,0])>np.cos(np.radians(10))
  depth=float(np.clip(.037-delta[0],0,.074)) if np.linalg.norm(delta[1:])<.002 and aligned else 0.
  row.update(bolt_inserted_depth_m=depth,bolt_through_both=depth>=.0739,seat_torque_Nm=self.env.thread.seat_torque_nm)
  return row


def camera_options(name):
 """Keep the native CAD optical dome visible externally, exclude it from its
 internal camera's optical RGB/depth path. All collision hulls stay solid."""
 option=mujoco.MjvOption();option.geomgroup[4]=0 if name=='rov' else 1
 return option
