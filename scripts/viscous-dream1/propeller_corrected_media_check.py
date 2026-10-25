# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Fully decode the corrected bench clip and hash its supporting evidence."""
import hashlib
import json
import imageio.v2 as imageio
import numpy as np
from propeller_corrected_scene import OUT


def main():
    path=OUT/'scripted_feasibility.mp4'
    reader=imageio.get_reader(path);metadata=reader.get_meta_data();count=0
    for frame in reader:
        assert frame.shape==(940,1600,3)
        count+=1
        if count in (1,35,60,65,120):imageio.imwrite(OUT/f'motion_sample_{count}.jpg',frame)
    reader.close()
    assert count==150 and metadata['fps']==10
    replay=np.load(OUT/'physics_replay.npz');rows=[json.loads(line) for line in (OUT/'trajectory.jsonl').read_text().splitlines()]
    assert len(rows)==300 and replay['qpos'].shape[0]==300
    assert np.all(np.diff(replay['time'])>0)
    assert np.isfinite(replay['qpos']).all() and np.isfinite(replay['qvel']).all()
    for row,t in zip(rows,replay['time']):assert abs(row['time_s']-t)<1e-9
    release=json.loads((OUT/'release_trace.json').read_text())
    assert len(release)==30 and release[0]['time_s']>replay['time'][-1]
    report=dict(frames_decoded=count,fps=metadata['fps'],duration_s=count/metadata['fps'],size=metadata['size'],
        replay_rows=300,release_rows=30,finite_and_monotonic_physics_states=True,
        sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [path,OUT/'physics_replay.npz',OUT/'trajectory.jsonl',OUT/'release_trace.json',OUT/'geometry_check.json',OUT/'check.json']})
    (OUT/'media_check.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
