# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""A consistent side view keeps the mounted arms and blade readable together."""
import bpy
from mathutils import Vector

TARGET=(.50,.16,.20)
WIDE=(-.05,2.60,.42)
WORK=(-.05,2.00,.35)
LENSES=(38.,34.)
CAMERA_TARGETS={'Camera / cleaning overview':TARGET}


def create_camera(scene,name,poses):
    data=bpy.data.cameras.new(name);data.clip_start=.01;data.clip_end=100;data.dof.use_dof=False
    obj=bpy.data.objects.new(name,data);scene.collection.objects.link(obj);previous=None
    for frame,pos,lens in poses:
        obj.location=pos;q=(Vector(TARGET)-obj.location).to_track_quat('-Z','Y')
        angles=q.to_euler('XYZ',previous) if previous is not None else q.to_euler('XYZ');previous=angles.copy()
        obj.rotation_euler=angles;data.lens=lens
        obj.keyframe_insert('location',frame=frame);obj.keyframe_insert('rotation_euler',frame=frame);data.keyframe_insert('lens',frame=frame)
    return obj


def build(scene):
    poses=[]
    for f in range(1,scene.frame_end+1):
        if f<=73:a=(f-1)/72
        else:a=1.
        a=a*a*(3-2*a)
        pos=tuple((1-a)*x+a*y for x,y in zip(WIDE,WORK));lens=(1-a)*LENSES[0]+a*LENSES[1]
        poses.append((f,pos,lens))
    cam=create_camera(scene,'Camera / cleaning overview',poses)
    create_camera(scene,'Camera / fixed dirty clean comparison',[(1,WORK,LENSES[1])])
    scene.camera=cam
    marker=scene.timeline_markers.new('Continuous view / full mounted arms and working blade',frame=1);marker.camera=cam
    scene['camera_intent']='Single above-contact side view; no rear-body or under-ROV cuts. Gentle opening push, stable active scrub view, held ending; full working blade and both arm mounts remain in frame.'
