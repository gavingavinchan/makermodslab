# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Render the exact saved physical trajectory with underwater presentation.

No physics steps are executed: qpos/qvel are replayed, including the free tool.
These pose writes are explicitly replay, never used as feasibility evidence.
"""
import argparse
import hashlib
import json
import numpy as np
import mujoco
import imageio.v2 as imageio
from PIL import Image, ImageDraw
from propeller_large_scene import LargeEnv, OUT


def run(stills_only=False):
    out=OUT/'underwater';out.mkdir(parents=True,exist_ok=True)
    path=OUT/'physics_replay.npz';trajectory=np.load(path)
    e=LargeEnv(underwater=True);bench=LargeEnv()
    assert e.model.nq==bench.model.nq and e.model.nv==bench.model.nv
    assert [e.model.joint(i).name for i in range(e.model.njnt)]==[bench.model.joint(i).name for i in range(bench.model.njnt)]
    renderer=mujoco.Renderer(e.model,width=1280,height=720)
    def state(i):
        for env in (e,bench):
            env.data.qpos[:]=trajectory['qpos'][i];env.data.qvel[:]=trajectory['qvel'][i]
            env.data.time=float(trajectory['time'][i]);mujoco.mj_forward(env.model,env.data)
        for body in ('cardinal_tool','cardinal_propeller','left_tool0','right_tool0','left_base_link','right_base_link'):
            np.testing.assert_allclose(e.data.body(body).xpos,bench.data.body(body).xpos,atol=1e-12,rtol=0)
            np.testing.assert_allclose(e.data.body(body).xmat,bench.data.body(body).xmat,atol=1e-12,rtol=0)
    def frame(camera):
        renderer.update_scene(e.data,camera=camera)
        return renderer.render().copy()
    state(100)
    for camera in ('underwater_hero','underwater_wide','underwater_detail'):
        Image.fromarray(frame(camera)).save(out/(camera+'.png'))
    count=0
    if not stills_only:
        with imageio.get_writer(out/'underwater_preview.mp4',fps=10,codec='libx264',pixelformat='yuv420p',macro_block_size=1,quality=8) as writer:
            for i in range(0,len(trajectory['time']),2):
                state(i)
                camera='underwater_hero' if i<100 or i>=240 else 'underwater_detail'
                im=Image.fromarray(frame(camera));draw=ImageDraw.Draw(im)
                draw.rectangle((0,686,1280,720),fill='#061e28')
                draw.text((16,697),f'960 mm propeller | SAME PHYSICS TRAJECTORY / underwater visual replay | t={e.data.time:.1f}s | no wet dynamics or Dream result',fill='#c5e4e3')
                writer.append_data(np.asarray(im));count+=1
    report=dict(source='../physics_replay.npz',source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                visual_replay_only=True,underwater_dynamics=False,dream_calls=0,
                same_robot_and_free_tool_transforms_verified=True,frames=count,fps=10,
                duration_s=count/10,render_size=[1280,720],table_removed=True,
                fixture='Fixed submerged service frame; not a floating ROV',
                scene='Procedural stern, shaft, bronze propeller, seabed, work lights, blue-green fog and particles')
    (out/'replay_check.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));renderer.close();e.close();bench.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--stills-only',action='store_true')
    run(parser.parse_args().stills_only)
