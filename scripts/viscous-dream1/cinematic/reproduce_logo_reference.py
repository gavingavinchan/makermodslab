# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Reproduce the source mounted extended badge view before scene integration."""
from pathlib import Path
import json,bpy,gzip
from mathutils import Matrix,Vector
HERE=Path(__file__).resolve().parent;A=HERE/'assets/logo_badges';OUT=HERE/'results/logo_check'
bpy.ops.wm.read_factory_settings(use_empty=True);s=bpy.context.scene
s.render.engine='CYCLES';s.cycles.samples=12;s.cycles.use_denoising=True
s.render.resolution_x=900;s.render.resolution_y=600;s.render.resolution_percentage=100
s.world=bpy.data.worlds.new('Reference');s.world.color=(.6,.6,.6);s.view_settings.view_transform='Standard'
materials={}
def obj(spec,T=None):
 key=tuple(spec['rgba'])
 if key not in materials:
  m=bpy.data.materials.new(str(key));m.diffuse_color=key;m.use_nodes=True;b=m.node_tree.nodes['Principled BSDF'];b.inputs['Base Color'].default_value=key;b.inputs['Roughness'].default_value=.44;materials[key]=m
 me=bpy.data.meshes.new(spec['name']);me.from_pydata(spec['vertices'],[],spec['faces']);me.update();o=bpy.data.objects.new(spec['name'],me);s.collection.objects.link(o);o.data.materials.append(materials[key])
 if T is not None:o.matrix_world=T
for spec in json.loads(gzip.decompress((A/'reference_arm.json.gz').read_bytes()).decode()):obj(spec)
for spec in json.loads((A/'parts.json').read_text()):
 side=spec['side'];x=79 if spec['kind']=='nameplate' else 33.94112;plane=29.5 if spec['kind']=='nameplate' else 37.5
 obj(spec,Matrix(((-side,0,0,x-160),(0,0,side,side*plane),(0,1,0,58.78775+111),(0,0,0,1))))
def aim(o,p):o.rotation_euler=(Vector(p)-o.location).to_track_quat('-Z','Y').to_euler()
for name,pos,power,size in [('Key',(-70,-100,420),2700000,210),('Fill',(180,60,340),1500000,170)]:
 d=bpy.data.lights.new(name,'AREA');d.energy=power;d.shape='DISK';d.size=size;o=bpy.data.objects.new(name,d);s.collection.objects.link(o);o.location=pos;aim(o,(-105,0,170))
d=bpy.data.cameras.new('Reference camera');d.type='ORTHO';d.ortho_scale=140;d.clip_end=3000;o=bpy.data.objects.new('Reference camera',d);s.collection.objects.link(o);s.camera=o
for name,pos in [('extended',(-85,460,230)),('bearing',(-115,-460,230))]:
 o.location=pos;aim(o,(-100,0,169));s.render.filepath=str(OUT/f'reproduced_mounted_{name}.png');bpy.ops.render.render(write_still=True)
print('REFERENCE_COMPLETE',flush=True)
