# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""OptiX render; large full-frame sequence stays in explicit scratch storage."""
from pathlib import Path
import argparse,hashlib,json,time,bpy
HERE=Path(__file__).parent;OUT=HERE/'results'
p=argparse.ArgumentParser();p.add_argument('--frames',nargs='*',type=int);p.add_argument('--scratch',type=Path,default=Path('/tmp/viscous-cinematic-20261024-frames'));p.add_argument('--comparison',action='store_true');p.add_argument('--samples',type=int,default=32);p.add_argument('--last-frame',type=int);a=p.parse_args()
bpy.ops.wm.open_mainfile(filepath=str(OUT/'final_scene.blend'));s=bpy.context.scene
prefs=bpy.context.preferences.addons['cycles'].preferences;prefs.compute_device_type='OPTIX';prefs.get_devices()
for d in prefs.devices:d.use=d.type=='OPTIX'
assert any(d.type=='OPTIX' for d in prefs.devices),'No OptiX device'
s.cycles.device='GPU';s.cycles.samples=a.samples;s.render.use_persistent_data=True
if a.comparison:
    for m in list(s.timeline_markers):s.timeline_markers.remove(m)
    s.camera=bpy.data.objects['Camera / fixed dirty clean comparison']
frames=a.frames or range(1,min(s.frame_end,a.last_frame or s.frame_end)+1);a.scratch.mkdir(exist_ok=True,parents=True)
if not a.frames:
    marker=a.scratch/'render_inputs.json'
    inputs=dict(scene_sha256=hashlib.sha256((OUT/'final_scene.blend').read_bytes()).hexdigest(),motion_sha256=hashlib.sha256((OUT/'motion.json').read_bytes()).hexdigest(),samples=a.samples,fps=s.render.fps,frames=s.frame_end,width=s.render.resolution_x,height=s.render.resolution_y)
    if marker.exists():assert json.loads(marker.read_text())==inputs,'Cached frames belong to different render inputs; use an empty scratch directory'
    else:
        assert not any(a.scratch.glob('frame_*.png')),'Unidentified cached frames; use an empty scratch directory'
        marker.write_text(json.dumps(inputs,indent=2)+'\n')
for f in frames:
    path=OUT/f'{"comparison" if a.comparison else "preview"}_{f:04d}.png' if a.frames else a.scratch/f'frame_{f:04d}.png'
    if path.exists() and not a.frames:continue
    start=time.monotonic();s.frame_set(f);s.render.filepath=str(path);bpy.ops.render.render(write_still=True);print(f'FRAME_COMPLETE {f} seconds={time.monotonic()-start:.2f}',flush=True)
