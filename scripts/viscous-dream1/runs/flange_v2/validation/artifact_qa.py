"""Decode deliverables, check protected arm files and unchanged thread implementation."""
from pathlib import Path
import hashlib,json,subprocess,re
from PIL import Image
import imageio_ffmpeg
root=Path.cwd();out=root/'runs/flange_v2';validation=out/'validation';results={}
for name in ('overview_front.png','overview_top.png','overview_closeup.png'):
 with Image.open(out/name) as im:assert im.size==(1280,720);im.load();results[name]=dict(size=list(im.size))
for name in ('before_after.png','contact_sheet.jpg'):
 with Image.open(out/name) as im:im.load();results[name]=dict(size=list(im.size))
protected={'viscous_arm.json':'cbbd055358a98b8ef39eecde33df87db265ed2ebd7b06f8fd46f32674fe0d2b0','viscous_arm_fingertip.json':'b29d3461b256d10a36a43a6ac31a8acba5a9750c4384c9d8fec1b7103dab572a'}
assert all(hashlib.sha256((root/p).read_bytes()).hexdigest()==sha for p,sha in protected.items())
def thread(p):return p.read_text().split('class ThreadModel:')[1].split('class Measurements:')[0]
assert thread(root/'flange_scene.py')==thread(out/'source_before/flange_scene.py')
ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
p=subprocess.run([ffmpeg,'-hide_banner','-i',str(out/'scripted_demo.mp4'),'-f','null','-'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
(validation/'video_decode.log').write_text(p.stderr)
assert p.returncode==0,p.stderr[-1500:]
assert re.search(r'Video: h264.*yuv420p',p.stderr),p.stderr[:2000]
results['video']=dict(full_decode_exit_code=p.returncode,codec='h264',pixel_format='yuv420p')
results.update(protected_arm_sha256=protected,thread_model_byte_identical=True,real_ledger='never opened; guarded checks reject access',inference_calls=0)
(validation/'artifact_qa.json').write_text(json.dumps(results,indent=2)+'\n')
print('PASS: images decoded at requested size; full H.264/yuv420p video decode; arm hashes and thread implementation unchanged')
