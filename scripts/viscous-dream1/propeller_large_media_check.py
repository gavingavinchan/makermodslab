# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Decode every frame of both final previews and record artifact hashes."""
import hashlib,json
import imageio.v2 as imageio
from propeller_large_scene import OUT
def main():
    report={}
    for path in [OUT/'scripted_feasibility.mp4',OUT/'underwater/underwater_preview.mp4']:
        reader=imageio.get_reader(path);meta=reader.get_meta_data();count=0
        for frame in reader:
            assert frame.ndim==3 and frame.shape[2]==3
            count+=1
            if count in (1,60,120):imageio.imwrite(OUT/f'{path.stem}_sample_{count}.jpg',frame)
        reader.close()
        assert count==150,(path,count)
        report[str(path.relative_to(OUT))]=dict(frames_decoded=count,fps=meta['fps'],duration_s=count/meta['fps'],size=meta['size'],sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    (OUT/'media_check.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
