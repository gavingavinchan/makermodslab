"""Execute only the sweep local stub, with inference/accounting blocked."""
import os,sys,runpy,shutil,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'runs/shackle_scene'
def audit(event,args):
 if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and Path(os.fsdecode(args[0])).name in ('ledger.json','ledger.lock','runner.lock'):raise AssertionError('protected ledger/runner lock')
 if event=='socket.connect' and isinstance(args[1],tuple):raise AssertionError('remote network forbidden')
sys.addaudithook(audit)
import dream
def forbidden(*a,**k):raise AssertionError('Dream client construction forbidden')
dream.DreamClient=forbidden
shutil.rmtree(OUT/'dry',ignore_errors=True)
sys.argv=['sweep.py','--task','shackle','--episodes','1','--max-calls','2']
runpy.run_path(str(ROOT/'sweep.py'),run_name='__main__')
r=json.loads((OUT/'dry/ep_00/log.json').read_text());assert r['executed_rows']==96 and r['calls_used']==0 and r['stub_calls']==2 and not r['success']
assert len(r['row_measurements'])==96
(OUT/'validation/dry_checks.json').write_text(json.dumps(dict(passed=True,stub_calls=2,rows=96,inference_calls=0,client_construction_forbidden=True,accounting_access_forbidden=True,remote_network_forbidden=True),indent=2))
print('PASS: sweep --task shackle, two 48-row stub chunks, no client/network/accounting access')
