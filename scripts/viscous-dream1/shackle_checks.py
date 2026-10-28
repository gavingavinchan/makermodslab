"""Local contact/thread/legacy certification. No server, client or ledger."""
import os,sys,json,hashlib,ast
from pathlib import Path
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'runs/shackle_scene'
# Refuse remote connects/accounting, even accidental reads, in this check.
def audit(event,args):
 if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and Path(os.fsdecode(args[0])).name in ('ledger.json','ledger.lock','runner.lock'):raise AssertionError('protected accounting')
 if event=='socket.connect' and isinstance(args[1],tuple):raise AssertionError('network forbidden')
sys.addaudithook(audit)
from shackle_scene import CENTER,SEAT,MOUTH,PITCH,TOP,ThreadModel,Measurements
from shackle_env import ShackleEnv
from cross_scenes import viscous_xml

def put(e,z,angle=0,xy=None,tilt=0):
 from scipy.spatial.transform import Rotation
 j=e.model.joint('pin_free');q,v=j.qposadr[0],j.dofadr[0]
 e.data.qpos[q:q+3]=np.r_[CENTER[:2] if xy is None else xy,z]
 R=Rotation.from_rotvec([0,tilt,0]).as_matrix()@Rotation.from_rotvec([0,0,angle]).as_matrix()
 mujoco.mju_mat2Quat(e.data.qpos[q+3:q+7],R.ravel());e.data.qvel[v:v+6]=0;mujoco.mj_forward(e.model,e.data)

def run():
 old=json.loads((OUT/'validation/legacy_xml_before.json').read_text());now={t:hashlib.sha256(viscous_xml(t).encode()).hexdigest() for t in old};assert old==now
 # The entire original server module is identical after removing only the
 # early, __main__-only shackle dispatch. Existing imported functions/AST match.
 before=ast.parse((OUT/'source_before/server.py').read_text());after=ast.parse((ROOT/'astra_op/server.py').read_text())
 after.body=[n for n in after.body if not (isinstance(n,ast.If) and isinstance(n.test,ast.Compare) and ast.unparse(n.test)=="__name__ == '__main__'" and any(isinstance(x,ast.Import) and any(y.asname=='_shackle_sys' for y in x.names) for x in n.body))]
 assert ast.dump(before)==ast.dump(after)
 e=ShackleEnv();j=e.model.joint('pin_free');v=j.dofadr[0];t=e.thread
 assert e.model.body('pin').mass[0]>.240 and e.model.body('pin').mass[0]<.242
 assert e.model.body('shackle').jntnum[0]==0 and e.model.body('shackle_holder').jntnum[0]==0
 # Free insertion through the clear ear is never corrected/attached/threaded.
 put(e,TOP-.010);beforeq=e.data.qpos.copy();t.step();np.testing.assert_array_equal(beforeq,e.data.qpos);assert not t.engaged
 # Catch planes use exactly the shared visual phase at mouth, with swept entry.
 phase=SEAT+round((MOUTH-SEAT)/PITCH)*PITCH
 put(e,phase+.0001);t.step();put(e,phase-.0001);t.step();assert t.engaged
 t.step();assert abs(e.data.body('pin').xpos[2]-phase)<1e-8
 # No angular drift without drive. Friction stops free coasting, with exact pitch.
 start=t.start;t.spin=-20.;e.data.qvel[v+5]=-20.;e.data.qvel[v+2]=-20*PITCH/(2*np.pi)
 t.step();assert t.turns>0 and abs(e.data.body('pin').xpos[2]-(start-PITCH*t.turns))<1e-12
 for _ in range(200):t.step()
 still=t.turns
 for _ in range(100):t.step()
 assert abs(t.turns-still)<1e-12
 # Pull-off releases, and off-axis / >10deg tip entries cannot engage.
 put(e,t.start+.004);t.step();assert not t.engaged
 for xy,tilt in ((CENTER[:2]+[.004,0],0),(CENTER[:2],np.deg2rad(12))):
  e.thread=t=ThreadModel(e);put(e,phase+.0001,xy=xy,tilt=tilt);t.step();put(e,phase-.0001,xy=xy,tilt=tilt);t.step();assert not t.engaged
 # Actual mesh-cell clearance: centerline thread proxy passes upper/lower ears.
 contacts=[]
 for z in np.linspace(TOP+.005,SEAT+.002,60):
  put(e,z)
  hits=[float(c.dist) for c in e.data.contact if set(e.model.body(e.model.geom_bodyid[g]).name for g in (c.geom1,c.geom2))=={'pin','shackle'}]
  assert not [d for d in hits if d<-.00005],(z,hits)
  contacts.extend(hits)
 # Same parked state even in response to adversarial right actions.
 q=e.state();q['right'][:]=0;e.step(q['left'],q['right']);np.testing.assert_array_equal(e.data.qpos[e.park_qadr],e.park_values)
 e.close()
 # Profile/parser checks only; deliberately do NOT instantiate Episode.
 import shackle_astra as sa
 args=sa.parse_args(['--profile','shackle','--episode','review','--output',str(OUT/'astra/review')]);assert args.thread_hand=='right' and args.condition=='T'
 assert '2.5/360000.' in __import__('inspect').getsource(sa.Episode.screw)
 result=dict(legacy_xml_byte_identical=now,legacy_server_ast_identical=True,pin_stainless_mass_kg=.24073158,fixed_fixture=True,clear_ear_real_contact=True,continuous_phase_entry=True,right_hand_pitch=True,passive_thread_no_coasting=True,pull_release=True,off_axis_rejected=True,right_actions_ignored=True,astra_parser_only=True,astra_episode_instantiated=False,inference_calls=0,ledger_access=False)
 (OUT/'validation/task_checks.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':run()
