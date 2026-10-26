# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Independently measure the saved native rig through all stripe transitions."""
from pathlib import Path
import hashlib, json, sys
import bpy
import numpy as np
from scipy.signal import find_peaks

HERE=Path(__file__).parent; OUT=HERE/'results'
sys.path.insert(0,str(HERE/'vendor'))
from bimanual_common import fk,NAMES
from surface import surface

motion=json.loads((OUT/'motion.json').read_text())
bpy.ops.wm.open_mainfile(filepath=str(OUT/'final_scene.blend'))
scene=bpy.context.scene
begin,end=motion['contact_frames']
frames=np.arange(begin,end+.001,.25)
points=[];joints=[];agreement=0;surface_error=0;axis_error=0
record_frames=np.array([r['frame'] for r in motion['frames']])
for f in frames:
    scene.frame_set(int(f),subframe=float(f%1));bpy.context.view_layer.update()
    q=[bpy.data.objects[n+'_origin']['q'] for n in NAMES['right']]
    actual=np.array(bpy.data.objects['right_tool0'].matrix_world)
    expected=np.array(bpy.data.objects['rov_base'].matrix_world)@fk('right',q)
    agreement=max(agreement,float(abs(actual-expected).max()))
    pad=actual[:3,3]+actual[:3,2]*.101
    t=float(np.interp(f,record_frames,[r['t'] for r in motion['frames']]))
    u=float(np.interp(f,record_frames,[r['u'] for r in motion['frames']]))
    target,normal=surface(t,u)
    surface_error=max(surface_error,float(np.linalg.norm(pad-target)))
    axis_error=max(axis_error,float(np.linalg.norm(actual[:3,2]+normal)))
    points.append(pad);joints.append(q)
points=np.array(points);joints=np.array(joints)
dt=.25/motion['fps'];speed=np.linalg.norm(np.diff(points,axis=0),axis=1)/dt
radius=np.hypot(points[:,0],points[:,2])
outward,_=find_peaks(radius,prominence=.02)
inward,_=find_peaks(-radius,prominence=.02)
reversals=sorted([int(i) for i in outward]+[int(i) for i in inward])
transition=[]
for i in reversals:
    window=speed[max(0,i-48):min(len(speed),i+48)]
    transition.append(dict(frame=float(frames[i]),time_s=float((frames[i]-1)/24),minimum_speed_m_s=float(window.min()),maximum_speed_m_s=float(window.max())))
report=dict(passed=bool(speed.min()>.003 and len(reversals)==3 and agreement<1e-6 and surface_error<.00025 and axis_error<.001),
            scene_sha256=hashlib.sha256((OUT/'final_scene.blend').read_bytes()).hexdigest(),
            method='Evaluate actual saved Blender native driver rig every quarter frame (96 Hz) throughout active scrubbing, with independent FK agreement',
            samples=len(frames),active_frame_range=[begin,end],
            minimum_pad_speed_m_s=float(speed.min()),median_pad_speed_m_s=float(np.median(speed)),maximum_pad_speed_m_s=float(speed.max()),
            dwell_intervals_below_1mm_s=int(np.count_nonzero(speed<.001)),
            radial_span_m=float(np.ptp(radius)),radial_reversals=transition,
            peak_joint_speed_rad_s=float(abs(np.diff(joints,axis=0)).max()/dt),
            max_FK_Blender_matrix_error=agreement,
            maximum_pad_to_interpolated_analytic_surface_error_m=surface_error,
            maximum_pad_axis_vs_analytic_normal_vector_error=axis_error,
            scope='Authored continuous cinematic retarget, not a new observed dynamics run; initial and final presentation holds excluded')
(OUT/'motion_continuity.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2),flush=True)
assert report['passed']
