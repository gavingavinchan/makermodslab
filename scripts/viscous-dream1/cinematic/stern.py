# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Existing KRISO KCS hull/rudder CAD adapted around the approved rotor.

The official SIMMAN IGES surfaces replace the earlier illustrative loft.
The approved rotor, native vehicle and metric tool remain unchanged.
"""
import math,bpy,numpy as np
from mathutils import Vector

def material(name,color,metal=0,rough=.4):
    m=bpy.data.materials.new(name);m.use_nodes=True
    b=m.node_tree.nodes['Principled BSDF'];b.inputs['Base Color'].default_value=(*color,1);b.inputs['Metallic'].default_value=metal;b.inputs['Roughness'].default_value=rough
    return m

def mesh(name,verts,faces,mat):
    m=bpy.data.meshes.new(name);m.from_pydata(verts,[],faces);m.update()
    o=bpy.data.objects.new(name,m);bpy.context.scene.collection.objects.link(o);m.materials.append(mat)
    for p in m.polygons:p.use_smooth=True
    return o

def cylinder(name,radius,depth,mat,loc=(0,0,0),axis=(0,0,1)):
    bpy.ops.mesh.primitive_cylinder_add(vertices=64,radius=radius,depth=depth,location=loc)
    o=bpy.context.object;o.name=name;o.rotation_euler=Vector(axis).to_track_quat('Z','Y').to_euler();o.data.materials.append(mat)
    b=o.modifiers.new('Rounded machining edges','BEVEL');b.width=.0012;b.segments=3
    for p in o.data.polygons:p.use_smooth=True
    return o

def tube(name,points,radius,mat):
    c=bpy.data.curves.new(name,'CURVE');c.dimensions='3D';c.bevel_depth=radius;c.bevel_resolution=3
    s=c.splines.new('BEZIER');s.bezier_points.add(len(points)-1)
    for p,co in zip(s.bezier_points,points):p.co=co;p.handle_left_type=p.handle_right_type='AUTO'
    o=bpy.data.objects.new(name,c);bpy.context.scene.collection.objects.link(o);c.materials.append(mat);return o

def lathe_y(name,stations,mat):
    verts=[(r*math.cos(a),y,z+r*math.sin(a)) for y,r,z in stations for a in np.linspace(0,math.tau,65)[:-1]]
    faces=[(i*64+j,i*64+(j+1)%64,(i+1)*64+(j+1)%64,(i+1)*64+j) for i in range(len(stations)-1) for j in range(64)]
    faces.extend([tuple(range(63,-1,-1)),tuple((len(stations)-1)*64+j for j in range(64))])
    return mesh(name,verts,faces,mat)

def build():
    hull=material('Hull | aged red oxide antifouling',(.18,.025,.016),.15,.72)
    nt=hull.node_tree;n=nt.nodes;l=nt.links;b=n['Principled BSDF']
    geom=n.new('ShaderNodeNewGeometry');tex=n.new('ShaderNodeTexNoise');tex.inputs['Scale'].default_value=3;tex.inputs['Detail'].default_value=4;l.new(geom.outputs['Position'],tex.inputs['Vector'])
    ramp=n.new('ShaderNodeValToRGB');ramp.color_ramp.elements[0].color=(.13,.018,.010,1);ramp.color_ramp.elements[1].color=(.22,.035,.025,1);l.new(tex.outputs['Fac'],ramp.inputs[0]);l.new(ramp.outputs[0],b.inputs['Base Color'])
    fine=n.new('ShaderNodeTexNoise');fine.inputs['Scale'].default_value=200;fine.inputs['Detail'].default_value=2;l.new(geom.outputs['Position'],fine.inputs['Vector'])
    bump=n.new('ShaderNodeBump');bump.inputs['Strength'].default_value=.18;bump.inputs['Distance'].default_value=.0008;l.new(fine.outputs['Fac'],bump.inputs['Height']);l.new(bump.outputs[0],b.inputs['Normal'])
    seam=material('Hull | weld and seam oxide',(.035,.028,.019),.3,.8)
    bronze=material('Propeller hub | aged bronze',(.18,.12,.045),.78,.38)
    shaft=material('Shaft | machined stainless',(.22,.27,.28),.8,.28)
    from ship_cad import build_ship
    h,rud=build_ship(hull)
    # The adapted KCS shaft boss ends 0.349 m ahead of the approved propeller.
    cylinder('Ship / single coaxial shaft',.047,.38,shaft,(0,-.15,0),(0,1,0))
    lathe_y('Propeller / cast bronze hub',[(-.105,.074,0),(-.08,.090,0),(.055,.089,0),(.098,.067,0),(.128,.034,0),(.134,.006,0)],bronze)
    cylinder('Propeller / shaft locknut',.036,.025,bronze,(0,.128,0),(0,1,0))
    for a in np.linspace(0,math.tau,7)[:-1]:cylinder('Propeller / hub fastener',.007,.008,shaft,(.059*math.cos(a),.085,.059*math.sin(a)),(0,1,0))
    return {'hull':h,'rudder':rud}
