import sys
sys.path.insert(0,'.')
import json,re,csv,hashlib,ast,subprocess
from pathlib import Path
import numpy as np
import imageio_ffmpeg
import sweep
root=Path.cwd();out=root/'runs/flange_dream2';v=out/'validation'
audit=json.loads((v/'episode_audit.json').read_text());assert len(audit['episodes'])==12
assert audit['task_requests']==dict(billed=360,pool_warming=0,uncertain=0)
assert audit['ledger']==dict(cap=2212,budget_used=1986,confirmed_billed=1986,uncertain=0)
for n in ('legacy_checks.json','flange_local_checks.json'):
 checks=json.loads((v/n).read_text());assert all(c['exit_code']==0 for c in checks)
replays=json.loads((v/'real_replay_checks.json').read_text());assert {r['rig'] for r in replays}=={'viscous','yam'} and all(r['exact_row_and_torque_replay'] for r in replays)
script=json.loads((out/'scripted/log.json').read_text());rows=script['row_measurements']
assert not script['success'] and len(rows)==650
for s in ('left','right'):
 np.testing.assert_allclose(script['arm_torque_max_abs_Nm'][s],np.max([r['arm_torque_peak_abs_Nm'][s] for r in rows],axis=0),atol=1e-12)
 for rating in ('5','14'):
  np.testing.assert_allclose(script['arm_time_above_s'][rating][s],np.sum([r['arm_time_above_s'][rating][s] for r in rows],axis=0),atol=1e-8)
frames=imageio_ffmpeg.read_frames(str(out/'scripted/run.mp4'));metadata=next(frames);frames.close()
assert metadata['codec']=='h264' and metadata['pix_fmt'].startswith('yuv420p') and metadata['source_size']==(1920,400)
p=subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-i',str(out/'scripted/run.mp4'),'-f','null','-'],capture_output=True,text=True);assert p.returncode==0,p.stderr
report=root/'runs/flange_dream2_report.md';links=re.findall(r'\]\(([^)]+)\)',report.read_text());assert links and all((report.parent/link).exists() for link in links)
assert len(list(csv.DictReader((out/'episode_torques.csv').open())))==156
protected=json.loads((v/'protected.json').read_text())
for n,digest in protected['protected_sha256'].items():assert hashlib.sha256((root/n).read_bytes()).hexdigest()==digest
before=(out/'source_before/flange_scene.py').read_text();after=(root/'flange_scene.py').read_text()
def part(source,name):return ast.get_source_segment(source,next(n for n in ast.parse(source).body if getattr(n,'name',None)==name))
assert all(part(before,n)==part(after,n) for n in ('add_scene','ThreadModel'))
for f in ('sweep_scene.py','flange_recording.py'):assert (root/f).read_bytes()==(out/'source_before'/f).read_bytes()
e=sweep.SweepEnv('flange',sweep.DEFAULT)
assert all((e.model.actuator_forcerange[e.aids[s]]==[-25,25]).all() for s in e.sides);e.close()
changed=[n for n in ('flange_holding.py','flange_start.py','flange_scene.py','flange_pregrasp_demo.py','sweep.py','SWEEP_README.md') if (root/n).read_bytes()!=(out/'source_before'/n).read_bytes()]
result=dict(all_checks_passed=True,completed_policy_episodes=12,billed_calls=360,ledger_used=1986,ledger_cap=2212,report_links_checked=len(links),csv_joint_rows=156,scripted_rows=650,scripted_video=metadata,protected_jsons_unchanged=True,fixture_camera_mass_builder_unchanged=True,thread_model_unchanged=True,source_files_changed=changed,original_trial_check_artifacts_restored=True,report_sha256=hashlib.sha256(report.read_bytes()).hexdigest())
sweep.atomic(v/'final_check.json',result);print(json.dumps(result,indent=1))
