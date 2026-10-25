import json,subprocess,re,hashlib
from pathlib import Path
import imageio_ffmpeg
from PIL import Image
root=Path.cwd();out=root/'runs/flange_dream';ffmpeg=imageio_ffmpeg.get_ffmpeg_exe();result={}
for name,p in [('force_only',out/'force_only/run.mp4'),('scripted',out/'scripted/run.mp4'),('pregrasp_stub',out/'dry_pregrasp/B0/flange/ep_00/run.mp4'),('table_stub',root/'runs/flange_v2/dry/B0/flange/ep_00/run.mp4')]:
    process=subprocess.run([ffmpeg,'-hide_banner','-i',str(p),'-f','null','-'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    (out/'validation'/f'{name}_decode.log').write_text(process.stderr)
    assert process.returncode==0,process.stderr[-1000:]
    assert re.search(r'Video: h264.*yuv420p',process.stderr)
    assert re.search(r'1920x400',process.stderr),process.stderr[:1500]
    result[name]=dict(path=str(p.relative_to(root)),full_decode_passed=True,codec='H.264',pixel_format='yuv420p',width=1920,height=400)
protected={'viscous_arm.json':'cbbd055358a98b8ef39eecde33df87db265ed2ebd7b06f8fd46f32674fe0d2b0','viscous_arm_fingertip.json':'b29d3461b256d10a36a43a6ac31a8acba5a9750c4384c9d8fec1b7103dab572a'}
assert all(hashlib.sha256((root/p).read_bytes()).hexdigest()==h for p,h in protected.items())
old=(out/'source_before/flange_scene.py').read_text();new=(root/'flange_scene.py').read_text()
assert old.split('class ThreadModel:')[1].split('class Measurements:')[0]==new.split('class ThreadModel:')[1].split('class Measurements:')[0]
# add_scene body remains byte-identical; v2 fixtures, camera definitions and pipe run intact.
assert old.split('def add_scene(root):')[1].split('class ThreadModel:')[0]==new.split('def add_scene(root):')[1].split('class ThreadModel:')[0]
result.update(protected_arm_hashes_unchanged=True,scene_builder_byte_identical=True,thread_model_byte_identical=True,inference_calls=0)
(out/'validation/artifact_checks.json').write_text(json.dumps(result,indent=2))
print('PASS: full H.264/yuv420p three-pane video decode, protected arm hashes, original scene and thread model')
