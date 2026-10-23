"""Run existing zero-inference suites; preserve their own assertions and fixtures."""
import os, subprocess, sys, json
from pathlib import Path
root=Path.cwd();out=root/'runs/flange/validation'
checks=[['sweep.py','--check'],['sweep_runner_check.py'],['sweep_baseline_check.py'],['onearm_check.py'],['onearm_isolate_check.py'],['propeller_ik_check.py'],['propeller_check.py']]
results=[]
for args in checks:
 name=Path(args[0]).stem
 print('START',name,flush=True)
 with (out/(name+'.console.log')).open('w') as f:
  p=subprocess.run([sys.executable,*args],cwd=root,env=os.environ,stdout=f,stderr=subprocess.STDOUT)
 results.append(dict(check=args,exit_code=p.returncode))
 (out/'legacy_checks.json').write_text(json.dumps(results,indent=2))
 print('DONE',name,p.returncode,flush=True)
