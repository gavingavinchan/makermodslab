# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Encode, fully decode and inventory final media; frames stay outside the repo."""
from pathlib import Path
import argparse, hashlib, json, subprocess, shutil
from PIL import Image, ImageDraw, ImageStat

HERE = Path(__file__).resolve().parent
OUT = HERE / 'results'
p = argparse.ArgumentParser()
p.add_argument('--scratch', type=Path, required=True)
a = p.parse_args()
motion = json.loads((OUT / 'motion.json').read_text())
expected = motion['end']
scene_sha = hashlib.sha256((OUT / 'final_scene.blend').read_bytes()).hexdigest()
audit = json.loads((OUT / 'geometry_audit.json').read_text())
validation = json.loads((OUT / 'report.json').read_text())
assert audit['passed'] and audit['samples'] == 1343 and audit['scene_sha256'] == scene_sha
assert validation['core_checks_passed'] and validation['final_scene_sha256'] == scene_sha
assert validation['mask_outside_exported_nominal_union_pixels'] == 0
visibility=json.loads((OUT/'camera_visibility.json').read_text())
assert visibility['passed'] and visibility['samples']==expected and visibility['scene_sha256']==scene_sha
assert visibility['minimum_context_body_bbox_frame_margin']>.02 and visibility['minimum_native_arm_bbox_frame_margin']>.02
stability=json.loads((OUT/'canister_stability.json').read_text())
assert stability['passed'] and stability['scene_sha256']==scene_sha
assert stability['frame_range']==[73,expected] and stability['samples']==expected-72
if 'invariance_verification' in audit:
    proof_path=OUT/audit['invariance_verification'];proof=json.loads(proof_path.read_text())
    assert proof['passed'] and proof['scene_sha256']==scene_sha
    assert hashlib.sha256(proof_path.read_bytes()).hexdigest()==audit['invariance_verification_sha256']
render_inputs=json.loads((a.scratch/'render_inputs.json').read_text())
assert render_inputs['scene_sha256']==scene_sha
assert render_inputs['motion_sha256']==hashlib.sha256((OUT/'motion.json').read_bytes()).hexdigest()
assert render_inputs['frames']==expected and render_inputs['fps']==motion['fps']

frames = [a.scratch / f'frame_{f:04d}.png' for f in range(1, expected + 1)]
assert all(f.is_file() for f in frames), 'Incomplete rendered sequence'
visual_stats=[]
for f in frames:
    with Image.open(f) as im:
        assert im.size == (1280, 720), (f, im.size)
        im.verify()
    with Image.open(f) as im:
        stat=ImageStat.Stat(im.convert('L').resize((64,36)))
        assert stat.mean[0]>2 and stat.stddev[0]>8,('Black or nearly uniform frame',f)
        visual_stats.append((stat.mean[0],stat.stddev[0]))
movie = OUT / 'cleaning_preview.mp4'
subprocess.run(['ffmpeg', '-y', '-v', 'error', '-framerate', str(motion['fps']),
                '-start_number', '1', '-i', str(a.scratch / 'frame_%04d.png'),
                '-frames:v', str(expected), '-c:v', 'libx264', '-crf', '18',
                '-preset', 'medium', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                '-an', str(movie)], check=True)
probe = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-count_frames',
    '-select_streams', 'v:0', '-show_entries',
    'stream=codec_name,width,height,pix_fmt,r_frame_rate,nb_read_frames,duration:format=duration,size',
    '-of', 'json', str(movie)]))
st = probe['streams'][0]
assert int(st['nb_read_frames']) == expected
assert st['width'] == 1280 and st['height'] == 720
assert st['r_frame_rate'] == '24/1'
assert abs(float(probe['format']['duration']) - expected / 24) < .001
# Independent full decoder pass, including all packets, with failure on errors.
decode = subprocess.run(['ffmpeg', '-v', 'error', '-xerror', '-i', str(movie),
                         '-map', '0:v:0', '-f', 'null', '-'], capture_output=True, text=True)
assert decode.returncode == 0 and not decode.stderr.strip(), decode.stderr
sheet = Image.new('RGB', (1280, 3 * 264), '#10242b')
draw = ImageDraw.Draw(sheet)
selected = [1, 80, 150, 240, 350, 450, 530, 590, 672]
for i, f in enumerate(selected):
    im = Image.open(frames[f - 1]).convert('RGB').resize((426, 240))
    x, y = (i % 3) * 426, (i // 3) * 264
    sheet.paste(im, (x, y))
    draw.text((x + 8, y + 244), f'{(f - 1) / 24:.2f}s | frame {f}', fill='#e4eeee')
sheet.save(OUT / 'contact_sheet.jpg', quality=93)
selection=json.loads((OUT / 'evidence_selection.json').read_text())
for key in ['outermost_native_pad_frame','largest_source_to_native_deviation_frame','peak_joint_step_frame']:
    shutil.copy2(frames[selection[key]-1],OUT / f'evidence_{key}.png')
report = dict(passed=True, png_frames_verified=expected, full_decode_exit_code=decode.returncode,
              full_decode_errors=decode.stderr, ffprobe=probe,
              video_sha256=hashlib.sha256(movie.read_bytes()).hexdigest(),
              scene_sha256=hashlib.sha256((OUT / 'final_scene.blend').read_bytes()).hexdigest(),
              frame_selection=selected, render=dict(engine='Cycles', samples=32,
              denoising=True, device='NVIDIA RTX 5070 Ti OptiX', fps=24, duration_s=28),
              scope='Frame dimensions/PNG validity, complete H.264 decode, stream duration and count; visual QA remains separate')
report.update(render_inputs=render_inputs,minimum_frame_luma_mean=min(x[0] for x in visual_stats),minimum_frame_luma_stddev=min(x[1] for x in visual_stats),black_or_uniform_frame_check_passed=True)
report['camera_visibility_verified_frames']=visibility['samples']
report['canister_stability_verified_working_frames']=stability['samples']
(OUT / 'media_verification.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
