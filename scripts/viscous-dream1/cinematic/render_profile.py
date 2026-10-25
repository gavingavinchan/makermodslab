# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Render an eye-level profile so vehicle horizontality is visible directly."""
from pathlib import Path
import hashlib,json
import bpy,numpy as np
from mathutils import Vector
HERE=Path(__file__).parent;OUT=HERE/'results'
bpy.ops.wm.open_mainfile(filepath=str(OUT/'final_scene.blend'))
s=bpy.context.scene;s.timeline_markers.clear();s.frame_set(350)
M=np.array(bpy.data.objects['rov_base'].matrix_world)
assert np.linalg.norm(M[:3,2]-[0,0,1])<1e-12
target=Vector((.57,.24,.10));pos=target+Vector(M[:3,1])*3
d=bpy.data.cameras.new('Camera / eye-level horizontal profile');d.type='ORTHO';d.ortho_scale=2.3;d.clip_start=.01
cam=bpy.data.objects.new('Camera / eye-level horizontal profile',d);s.collection.objects.link(cam);cam.location=pos;cam.rotation_euler=(target-pos).to_track_quat('-Z','Y').to_euler();s.camera=cam
prefs=bpy.context.preferences.addons['cycles'].preferences;prefs.compute_device_type='OPTIX';prefs.get_devices()
for device in prefs.devices:device.use=device.type=='OPTIX'
s.cycles.device='GPU';s.cycles.samples=32;s.render.filepath=str(OUT/'horizontal_profile.png');bpy.ops.render.render(write_still=True)
report=dict(source_scene_sha256=hashlib.sha256((OUT/'final_scene.blend').read_bytes()).hexdigest(),vehicle_up_axis=M[:3,2].tolist(),camera_position=list(pos),camera_target=list(target),orthographic_horizontal_span_m=d.ortho_scale,scope='Eye-level still from the unchanged packed scene; native vehicle remains level. The source blend is never saved by this helper.')
(OUT/'profile_view.json').write_text(json.dumps(report,indent=2)+'\n')
print('HORIZONTAL_PROFILE_COMPLETE',flush=True)
