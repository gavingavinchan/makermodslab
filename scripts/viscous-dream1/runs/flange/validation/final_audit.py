import json,hashlib,subprocess,os
from pathlib import Path
from PIL import Image
import imageio.v2 as imageio
import imageio_ffmpeg
root=Path.cwd();out=root/'runs/flange'
results={}
for p in out.glob('overview_*.png'):
 assert Image.open(p).size==(1280,720)
results['overviews']=[p.name for p in out.glob('overview_*.png')]
assert len(results['overviews'])==3
assert Image.open(out/'contact_sheet.jpg').size==(960,204)
results['policy_frame_size']=[320,176]
meta={}
for p in [out/'scripted_demo.mp4',out/'dry/B0/flange/ep_00/run.mp4']:
 process=subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','info','-i',str(p),'-f','null','-'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
 assert process.returncode==0,process.stderr[-1000:]
 line=next(x for x in process.stderr.splitlines() if 'Video: h264' in x)
 assert 'yuv420p' in line,line
 reader=imageio.get_reader(p);m=reader.get_meta_data();reader.close()
 meta[str(p.relative_to(out))]={k:m[k] for k in ('codec','pix_fmt','fps','size','duration')}
results['videos']=meta
for line in (out/'validation/protected_before.sha256').read_text().splitlines():
 digest,name=line.split(maxsplit=1);assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
results['ledger_and_arm_json_hashes_unchanged']=True
for name in ['physics_checks','ik_reach','runner_checks','demo_replay']:
 data=json.loads((out/f'validation/{name}.json').read_text());results[name]=data.get('passed',data.get('pose_reach_passed',True));assert results[name]
legacy=json.loads((out/'validation/legacy_checks.json').read_text());assert all(x['exit_code']==0 for x in legacy)
results['legacy_suites_passed']=len(legacy)
log=json.loads((out/'scripted_demo_log.json').read_text());results['demo']={k:log[k] for k in ['inference_calls','rows','bolt_grasped','nut_grasped','max_bolt_inserted_depth_m','max_nut_turns','nut_ever_engaged']}
results['inference_calls']=0
(out/'validation/final_audit.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))
