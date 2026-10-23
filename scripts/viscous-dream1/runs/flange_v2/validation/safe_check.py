"""Run unmodified no-call suites, blocking the real ledger and remote sockets.
Synthetic accounting fixtures stay available; default Ledger objects are inert.
"""
import sys,runpy,os
from pathlib import Path
ROOT=Path.cwd();actual=(ROOT/'runs/sweep/ledger.json').resolve()
def audit(event,args):
    if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
        p=Path(os.fsdecode(args[0])).resolve()
        if p==actual or p==actual.with_suffix('.lock'):
            raise AssertionError('Real ledger access forbidden')
    if event=='socket.connect':
        address=args[1]
        if isinstance(address,tuple) and address[0] not in ('127.0.0.1','::1','localhost'):
            raise AssertionError('Remote network forbidden in local checks')
sys.addaudithook(audit)
import sweep,yam_baseline as y
import dream.models as models
models._record=lambda checkpoint,api_key=None:models.Model(checkpoint,'public',checkpoint,'yam',True,None)
# Child baseline processes receive the same guards and local metadata.
import subprocess
original_run=subprocess.run
def guarded_run(command,*args,**kwargs):
    if isinstance(command,list) and len(command)>1 and str(command[0])==sys.executable and str(command[1]).endswith('.py') and Path(command[1]).name!='safe_check.py':
        command=[command[0],str(ROOT/'runs/flange_v2/validation/safe_check.py'),*command[1:]]
    return original_run(command,*args,**kwargs)
subprocess.run=guarded_run
old=sweep.Ledger.__init__
def local_ledger(self,root=sweep.ROOT,cap=None):
    if Path(root).resolve()==sweep.ROOT.resolve():
        self.path=actual;self.lock_path=actual.with_suffix('.lock');self.cap=cap or 900
    else:old(self,root,cap)
sweep.Ledger.__init__=local_ledger
y.settings=lambda:{'DREAM_API_KEY':'local-check-no-network'}
script=sys.argv[1];sys.argv=sys.argv[1:]
runpy.run_path(str(ROOT/script),run_name='__main__')
