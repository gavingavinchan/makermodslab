# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Establish the installation, then give the cleaning action the visual priority."""
import bpy
from mathutils import Vector

CONTEXT_TARGET=(.50,.16,.15)
WORK_TARGET=(.50,.12,.10)
CONTEXT_POSITION=(-.08,2.70,.40)
WORK_POSITION=(-.05,1.80,.30)
CONTEXT_LENS=36.
WORK_LENS=48.
WORK_START=73
CAMERA_TARGETS={'CAM_Context_ThreeQuarter':CONTEXT_TARGET,'CAM_Cleaning_TaskMedium':WORK_TARGET}


def create_camera(scene,name,position,target,lens,focus=False):
    data=bpy.data.cameras.new(name);data.clip_start=.01;data.clip_end=100
    data.sensor_width=36;data.lens=lens;data.show_composition_thirds=True
    obj=bpy.data.objects.new(name,data);scene.collection.objects.link(obj)
    obj.location=position;obj.rotation_euler=(Vector(target)-obj.location).to_track_quat('-Z','Y').to_euler()
    aim=bpy.data.objects.new('Focus / '+name,None);scene.collection.objects.link(aim);aim.location=target
    data.dof.use_dof=focus
    if focus:data.dof.focus_object=aim;data.dof.aperture_fstop=16
    return obj


def build(scene):
    context=create_camera(scene,'CAM_Context_ThreeQuarter',CONTEXT_POSITION,CONTEXT_TARGET,CONTEXT_LENS)
    work=create_camera(scene,'CAM_Cleaning_TaskMedium',WORK_POSITION,WORK_TARGET,WORK_LENS,focus=True)
    create_camera(scene,'Camera / fixed dirty clean comparison',WORK_POSITION,WORK_TARGET,WORK_LENS,focus=True)
    for frame,cam in [(1,context),(WORK_START,work)]:
        marker=scene.timeline_markers.new(cam.name,frame=frame);marker.camera=cam
    scene.camera=context
    scene['camera_intent']='Three-second establishing view, then a 48mm task medium with both arm mounts and whole working blade. Same viewing side, no underbody cut, no orbit or zoom during scrubbing. Task scale takes priority over fitting the complete vehicle.'
