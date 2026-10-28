"""Run original no-call checks with outputs redirected and real accounting blocked.
Only disposable synthetic accounting state is allowed (state.json/state.lock).
Original check and simulation source files are never edited.
"""
import os,sys,json,importlib,shutil,subprocess,traceback,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'runs/shackle_scene/validation/legacy';OUT.mkdir(parents=True,exist_ok=True)
sys.dont_write_bytecode=True;sys.path.insert(0,str(ROOT))
os.environ['JAX_COMPILATION_CACHE_DIR']=str(ROOT/'.cache/jax');os.environ['XDG_CACHE_HOME']=str(ROOT/'.cache');os.environ['DREAM_LOG_DIR']=str(OUT/'dream_logs')
def audit(event,args):
 if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
  p=Path(os.fsdecode(args[0])).resolve()
  if p.name in ('ledger.json','ledger.lock','runner.lock'):raise AssertionError('protected accounting/lock access')
  flags=args[2] if len(args)>2 else 0
  if flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC) and not any(p.is_relative_to(x) for x in (ROOT/'.cache',OUT)) and str(p)!='/dev/null':raise AssertionError('write outside authorized directories: '+str(p))
 if event=='socket.connect' and isinstance(args[1],tuple):raise AssertionError('network forbidden')
sys.addaudithook(audit)
import sweep,yam_baseline as y,dream.models as models
(OUT/'tmp').mkdir(exist_ok=True);tempfile.tempdir=str(OUT/'tmp')
models._record=lambda checkpoint,api_key=None:models.Model(checkpoint,'public',checkpoint,'yam',True,None)
y.settings=lambda:{'DREAM_API_KEY':'local-check-no-network'}
original_atomic=sweep.atomic
def atomic(path,obj):
 p=Path(path)
 if not p.resolve().is_relative_to(OUT):p=OUT/'redirected'/p.relative_to(ROOT)
 return original_atomic(p,obj)
sweep.atomic=atomic
old=sweep.Ledger.__init__
def ledger(self,root=None,cap=None):
 # Synthetic state only. Never open any ledger file, including the real one.
 root=Path(root) if root is not None else OUT/'inert'
 if not root.resolve().is_relative_to(OUT):root=OUT/'synthetic'/root.name
 self.path=root/'state.json';self.lock_path=root/'state.lock'
 self.cap=cap if cap is not None else (json.loads(self.path.read_text())['cap'] if self.path.exists() else 900)
sweep.Ledger.__init__=ledger
sweep.ROOT=OUT/'sweep'

def run(name):
 if name=='baseline_child':
  import runpy
  sys.argv=[str(ROOT/'yam_baseline.py'),*sys.argv[2:]]
  runpy.run_path(str(ROOT/'yam_baseline.py'),run_name='__main__');return
 if name=='sweep_baseline_check':
  original_run=subprocess.run
  def guarded(command,*args,**kwargs):
   if isinstance(command,list) and len(command)>1 and str(command[1]).endswith('yam_baseline.py'):
    command=[command[0],__file__,'baseline_child',*command[2:]]
   return original_run(command,*args,**kwargs)
  subprocess.run=guarded
  source=(ROOT/'sweep_baseline_check.py').read_text().replace("OUT = HERE / 'runs/sweep/validation/baseline_modes'", "OUT = Path("+repr(str(OUT/'baseline_modes'))+")")
  exec(compile(source,str(ROOT/'sweep_baseline_check.py'),'exec'),dict(__file__=str(ROOT/'sweep_baseline_check.py'),__name__='__main__'));return
 module=importlib.import_module(name)
 if name in ('flange_check','flange_holding_check','flange_pregrasp_check','flange_runner_check'):module.HERE=OUT
 if name=='flange_dream2_check':module.OUT=OUT/'dream2'
 if name=='flange_clearance_check':module.OUT=OUT/'redirected/runs/flange_v2/validation'
 if name in ('onearm_check','onearm_isolate_check'):
  dest=sweep.ROOT/('onearm_validation' if name=='onearm_check' else 'isolate_validation');dest.mkdir(parents=True,exist_ok=True)
  original=ROOT/'runs/sweep'/dest.name
  for p in original.glob('*before*'):
   if p.is_file():shutil.copyfile(p,dest/p.name)
  module.OUT=dest
 if name=='sweep_check':module.check()
 else:module.check()

if __name__=='__main__':
 if len(sys.argv)>1:run(sys.argv[1])
 else:
  names=['flange_check','flange_holding_check','flange_ik_check','flange_clearance_check','flange_pregrasp_check','flange_runner_check','sweep_check','sweep_runner_check','onearm_check','onearm_isolate_check','propeller_ik_check','flange_dream2_check','sweep_baseline_check']
  results=[]
  for name in names:
   print('START',name,flush=True)
   with (OUT/(name+'.log')).open('w') as f:r=subprocess.run([sys.executable,__file__,name],stdout=f,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',MUJOCO_GL='egl'))
   results.append(dict(check=name,exit_code=r.returncode));(OUT/'results.json').write_text(json.dumps(results,indent=2));print('DONE',name,r.returncode,flush=True)
