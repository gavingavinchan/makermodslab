# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Render ONLY a passing physical run, at real time using unchanged cameras.

qpos assignment here is offline rendering of recorded physics, not simulation
control. The physics entrypoint contains no trajectory-time pose setters.
"""
import json

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from propeller_wide_demo import OUT, digest, model_fingerprint
from propeller_corrected_scene import CorrectedEnv


def main():
    check = json.loads((OUT/'check.json').read_text())
    assert check['target_passed'], 'Do not render rejected candidates'
    replay = np.load(OUT/'physics_replay.npz')
    rows = [json.loads(line) for line in (OUT/'trajectory.jsonl').read_text().splitlines()]
    e = CorrectedEnv()
    assert model_fingerprint(e.model) == check['scene_invariants']['compiled_numeric_model_sha256']
    renderer = mujoco.Renderer(e.model, width=1280, height=720)
    small = mujoco.Renderer(e.model, width=640, height=360)
    font = ImageFont.truetype('DejaVuSans.ttf', 22)
    big = ImageFont.truetype('DejaVuSans.ttf', 27)
    fps = 10
    writers = {name:imageio.get_writer(OUT/f'{name}.mp4', fps=fps, codec='libx264', pixelformat='yuv420p', macro_block_size=1, quality=8)
               for name in ('contact_detail', 'diagnostic')}
    snapshots = {39:'start', 279:'mid', 519:'end', 109:'outer_reversal', 229:'inner_reversal'}
    span = check['actual_pad_radial_span_m']*1000
    def state(i):
        e.data.qpos[:] = replay['qpos'][i]
        e.data.qvel[:] = replay['qvel'][i]
        e.data.time = replay['time'][i]
        mujoco.mj_forward(e.model, e.data)
    try:
        for i in range(1, len(rows), 2):
            state(i)
            renderer.update_scene(e.data, camera='cardinal_detail')
            detail = renderer.render().copy()
            renderer.update_scene(e.data, camera='cardinal_front')
            front = renderer.render().copy()
            policy = e.images()
            if i in snapshots:
                label = snapshots[i]
                Image.fromarray(detail).save(OUT/f'{label}_detail.png')
                Image.fromarray(front).save(OUT/f'{label}_front.png')
                if label in ('start','mid','end'):
                    for key, pixels in policy.items(): Image.fromarray(pixels).save(OUT/f'{label}_{key}.png')
            # Keep all simulation pixels unmodified; captions occupy added margins.
            clean = Image.new('RGB', (1280, 800), '#15222a')
            clean.paste(Image.fromarray(detail), (0, 0))
            d = ImageDraw.Draw(clean)
            d.text((14, 727), f'ACTUAL PAD SWEEP {span:.1f} mm / {check["baseline_ratio"]:.2f}x baseline | t={e.data.time:04.1f}s | REAL TIME', font=big, fill='white')
            d.text((14, 765), 'Scripted physics from initialized pregrasp | Fixed original camera | Free tool, contact holds', font=font, fill='#ffd28a')
            writers['contact_detail'].append_data(np.asarray(clean))
            frame = Image.new('RGB', (1920, 1140), '#15222a')
            frame.paste(Image.fromarray(front), (0, 0))
            small.update_scene(e.data, camera='cardinal_detail')
            frame.paste(Image.fromarray(small.render().copy()), (1280, 0))
            d = ImageDraw.Draw(frame)
            lines = ['SCRIPTED PHYSICS / REAL TIME', f'Actual radial span: {span:.1f} mm', 'Baseline: 31.506 mm', f'Span ratio: {check["baseline_ratio"]:.2f}x',
                     f'Simultaneous contact: {100*check["scrub_contact_fraction"]["simultaneous"]:.3f}%', 'Left + right holds: 100%', 'Left arm: at 5 Nm cap throughout',
                     'Free tool; no weld or external force', 'Initialized pregrasp; fixed shaft', 'No policy / hardware / abrasion claim']
            for j, line in enumerate(lines): d.text((1294, 373+j*32), line, font=font, fill='white' if j<6 else '#ffd28a')
            r = rows[i]
            d.text((12, 724), f'{r["stage"]} | t={e.data.time:.2f}s | pad contact {r["contact_substeps"]["pad"]}/50 physics substeps', font=font, fill='white')
            for x, key in enumerate(('top','left','right')):
                assert policy[key].shape == (360,640,3)
                frame.paste(Image.fromarray(policy[key]), (x*640, 760))
            for x, name in enumerate(('OVERHEAD', 'LEFT WRIST', 'RIGHT WRIST')):
                d.text((x*640+12, 1120), name+' / original 640x360 pixels', fill='white')
            writers['diagnostic'].append_data(np.asarray(frame))
        release = json.loads((OUT/'release_trace.json').read_text())
        e.data.qpos[:] = release[-1]['qpos']; e.data.qvel[:] = 0
        mujoco.mj_forward(e.model, e.data)
        renderer.update_scene(e.data, camera='cardinal_front')
        Image.fromarray(renderer.render()).save(OUT/'released_front.png')
    finally:
        for writer in writers.values(): writer.close()
        renderer.close(); small.close(); e.close()
    report = {}
    for name, shape in [('contact_detail',(800,1280,3)), ('diagnostic',(1140,1920,3))]:
        reader = imageio.get_reader(OUT/f'{name}.mp4')
        metadata = reader.get_meta_data(); count = 0
        for frame in reader:
            assert frame.shape == shape
            count += 1
        reader.close()
        assert count == len(rows)//2 and metadata['fps'] == fps
        assert abs(count/fps-check['duration_s']) < 1e-6
        report[name] = dict(frames_fully_decoded=count, fps=fps, duration_s=count/fps, frame_shape=shape,
                            fixed_original_camera=True, real_time=True, sha256=digest(OUT/f'{name}.mp4'))
    report['script_sha256'] = digest(__import__('pathlib').Path(__file__))
    (OUT/'media_check.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
