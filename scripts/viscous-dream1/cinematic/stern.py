# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Reference-informed closed single-screw stern around the approved rotor.

Sources: SMH Marine drydocking-stern-propeller-anonymized-01.webp and
propulsion-stern-anonymized.webp, plus Palumbo Malta 14.JPG. Original modeled
geometry; photographs inform stern fullness, overhang and rudder proportions.
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
    # Fair transverse sections continue into a broad transom. The centreline
    # run rises over the screw while the bilges stay full, as in the references.
    # No flat underside disk or detached ellipsoid substitutes for the hull.
    from scipy.interpolate import PchipInterpolator
    control=np.array([(-8,1.9,-.72),(-4,1.85,-.72),(-2.2,1.72,-.65),(-1.4,1.55,-.55),(-.9,1.40,-.35),(-.55,1.29,-.05),(-.3,1.23,.30),(0,1.20,.64),(.5,1.12,.73),(1.0,1.02,.87),(1.5,.92,1.08),(1.85,.86,1.28)])
    ys=np.linspace(control[0,0],control[-1,0],112)
    widths=PchipInterpolator(control[:,0],control[:,1])(ys)
    bottoms=PchipInterpolator(control[:,0],control[:,2])(ys)
    stations=list(zip(ys,widths,bottoms))
    angles=np.linspace(-math.pi/2,math.pi/2,65)
    verts=[]
    def cross_height(y,z,a):return z+1.15*(1-math.cos(a))**(1.45-.35*np.clip((y+2)/3.85,0,1))
    for y,w,z in stations:
        verts.extend((w*math.sin(a),y,cross_height(y,z,a)) for a in angles)
        verts.extend(((w,y,2.65),(-w,y,2.65)))
    count=67
    faces=[(i*count+j,(i+1)*count+j,(i+1)*count+(j+1)%count,i*count+(j+1)%count) for i in range(len(stations)-1) for j in range(count)]
    faces.extend([tuple(range(count-1,-1,-1)),tuple((len(stations)-1)*count+j for j in range(count))])
    h=mesh('Ship / closed faired stern and shaft aperture',verts,faces,hull)
    for p in h.data.polygons[-2:]:p.use_smooth=False
    for a in [-1.22,-.86,-.48,.48,.86,1.22]:
        tube('Ship / longitudinal plating seam',[(w*math.sin(a),y,cross_height(y,z,a)-.0005) for y,w,z in stations],.0010,seam)
    for y,w,z in control[1:-1:2]:
        tube('Ship / transverse plating seam',[(w*math.sin(a),y,cross_height(y,z,a)-.0005) for a in angles],.0008,seam)
    lathe_y('Ship / faired stern shaft boss',[(-1.25,.22,.02),(-.90,.18,0),(-.55,.14,0),(-.30,.105,0),(-.18,.073,0)],hull)
    cylinder('Ship / single coaxial shaft',.047,.38,shaft,(0,-.15,0),(0,1,0))
    lathe_y('Propeller / cast bronze hub',[(-.105,.074,0),(-.08,.090,0),(.055,.089,0),(.098,.067,0),(.128,.034,0),(.134,.006,0)],bronze)
    cylinder('Propeller / shaft locknut',.036,.025,bronze,(0,.128,0),(0,1,0))
    for a in np.linspace(0,math.tau,7)[:-1]:cylinder('Propeller / hub fastener',.007,.008,shaft,(.059*math.cos(a),.085,.059*math.sin(a)),(0,1,0))
    # One airfoil rudder aft of the rotor, supported from the hull by its stock.
    # y is fore/aft, x is foil thickness; symmetric NACA-like closed section.
    vv=[];span=[(-.54,.45,.73),(.72,.42,.73)]
    us=np.linspace(0,1,40)
    contour=[(u,s) for s,seq in [(1,us),(-1,us[::-1])] for u in seq]
    for z,leading,chord in span:
        for u,sgn in contour:
            thick=5*.13*chord*(.2969*math.sqrt(u)-.126*u-.3516*u*u+.2843*u**3-.1036*u**4)
            vv.append((sgn*thick,leading+chord*u,z))
    count=len(contour);ff=[(i,(i+1)%count,(i+1)%count+count,i+count) for i in range(count)]+[tuple(range(count-1,-1,-1)),tuple(range(count,2*count))]
    rud=mesh('Ship / single aft rudder foil',vv,ff,hull)
    bevel=rud.modifiers.new('Rudder edge radii','BEVEL');bevel.width=.004;bevel.segments=3
    cylinder('Ship / rudder stock',.053,.30,shaft,(0,.67,.77))
    cylinder('Ship / stock gland housing',.085,.12,hull,(0,.67,.82))
    for z in [-.40,-.12,.16,.44]:
        tube('Ship / rudder horizontal weld',[(.044,.50,z),(.047,.62,z),(.038,.81,z),(.017,1.06,z)],.0015,seam)
    zinc=material('Rudder | sacrificial zinc anodes',(.24,.27,.26),.55,.72)
    for side in [-1,1]:
        for z in [-.35,.10,.52]:
            bpy.ops.mesh.primitive_cube_add(size=1,location=(side*.047,.57,z))
            block=bpy.context.object;block.name='Ship / rudder zinc anode';block.dimensions=(.018,.065,.11)
            bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
            block.data.materials.append(zinc);edge=block.modifiers.new('Cast anode edges','BEVEL');edge.width=.004;edge.segments=3
    return {'hull':h,'rudder':rud}
