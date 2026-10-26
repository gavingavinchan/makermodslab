# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Build packed, editable native-ROV presentation. All geometry remains metric."""
from pathlib import Path
import argparse,hashlib,json,math,random
import bpy,numpy as np
from mathutils import Matrix,Vector
from surface import blade_point,section,BENCH_CENTER
from stern import build as build_stern,material,cylinder,tube
HERE=Path(__file__).parent;OUT=HERE/'results'
p=argparse.ArgumentParser();p.add_argument('--preview',type=int,default=350);p.add_argument('--no-render',action='store_true');a=p.parse_args()
motion=json.loads((OUT/'motion.json').read_text())
bpy.ops.wm.open_mainfile(filepath=str(HERE/'assets/native_rov.blend'))
scene=bpy.context.scene
for o in list(scene.objects):
    if o.type in {'CAMERA','LIGHT'}:bpy.data.objects.remove(o,do_unlink=True)
for marker in list(scene.timeline_markers):scene.timeline_markers.remove(marker)
rov=bpy.data.objects['rov_base'];rov.matrix_world=Matrix(motion['rov_transform'])
scene.render.fps=motion['fps'];scene.frame_start=1;scene.frame_end=motion['end']
for r in motion['frames']:
    for name,q in r['joints'].items():
        ob=bpy.data.objects[name+'_origin'];ob['q']=q;ob.keyframe_insert('["q"]',frame=r['frame'])
for ob in scene.objects:
    if 'rpm' in ob:ob['rpm']=36 if 'left' in ob.name else -36
build_stern()
with bpy.data.libraries.load(str(HERE/'tool/polisher.blend'),link=False) as (src,dst):dst.collections=['CleaningTool']
scene.collection.children.link(dst.collections[0])
tool=bpy.data.objects['CleaningToolRoot'];tool.parent=bpy.data.objects['right_tool0'];tool.matrix_basis=Matrix.Translation((0,0,.020))
rotor=bpy.data.objects['ToolRotor']
for f,angle in [(1,0),(72,0),(90,12),(600,12+510*.8),(640,12+510*.8+14),(672,12+510*.8+14)]:
    rotor.rotation_euler.z=angle;rotor.keyframe_insert('rotation_euler',frame=f)
# Persistent dirt/copper shader: first contact-gated nominal footprint frame.
m=material('Blade | algae removed only beneath accumulated pad',(.06,.085,.025),.10,.86)
n=m.node_tree.nodes;l=m.node_tree.links;bs=n['Principled BSDF'];tex=n.new('ShaderNodeTexNoise');tex.inputs['Scale'].default_value=48;tex.inputs['Detail'].default_value=4;tex.inputs['Roughness'].default_value=.75
ramp=n.new('ShaderNodeValToRGB');ramp.color_ramp.elements[0].position=.20;ramp.color_ramp.elements[0].color=(.009,.024,.009,1);ramp.color_ramp.elements[1].position=.79;ramp.color_ramp.elements[1].color=(.115,.16,.038,1);l.new(tex.outputs['Fac'],ramp.inputs[0])
attr=n.new('ShaderNodeAttribute');attr.attribute_name='clean_at'
clock=n.new('ShaderNodeValue');clock.name='Cinematic frame'
for f in [1,motion['end']]:clock.outputs[0].default_value=f;clock.outputs[0].keyframe_insert('default_value',frame=f)
sub=n.new('ShaderNodeMath');sub.operation='SUBTRACT';l.new(clock.outputs[0],sub.inputs[0]);l.new(attr.outputs['Fac'],sub.inputs[1])
reveal=n.new('ShaderNodeMapRange');reveal.inputs['From Min'].default_value=0;reveal.inputs['From Max'].default_value=2;l.new(sub.outputs[0],reveal.inputs[0])
mix=n.new('ShaderNodeMixRGB');mix.inputs[2].default_value=(.58,.245,.085,1);l.new(reveal.outputs[0],mix.inputs[0]);l.new(ramp.outputs[0],mix.inputs[1]);l.new(mix.outputs[0],bs.inputs['Base Color'])
for socket,v0,v1 in [('Roughness',.88,.27),('Metallic',.08,.87)]:
    nn=n.new('ShaderNodeMapRange');nn.inputs['To Min'].default_value=v0;nn.inputs['To Max'].default_value=v1;l.new(reveal.outputs[0],nn.inputs[0]);l.new(nn.outputs[0],bs.inputs[socket])
fine=n.new('ShaderNodeTexNoise');fine.inputs['Scale'].default_value=370;fine.inputs['Detail'].default_value=2
bump=n.new('ShaderNodeBump');bump.inputs['Distance'].default_value=.00065;l.new(fine.outputs['Fac'],bump.inputs['Height']);l.new(bump.outputs[0],bs.inputs['Normal'])
strength=n.new('ShaderNodeMapRange');strength.inputs['To Min'].default_value=.48;strength.inputs['To Max'].default_value=.025;l.new(reveal.outputs[0],strength.inputs[0]);l.new(strength.outputs[0],bump.inputs['Strength'])
mask={};clean_area=0
for bi in range(4):
    path=HERE.parent/'assets/propeller_corrected'/f'blade_{bi}.stl'
    bpy.ops.wm.stl_import(filepath=str(path));o=bpy.context.object;o.name=f'Propeller / exact corrected blade {bi}';o.data.materials.clear();o.data.materials.append(m)
    if bi==0:
        mod=o.modifiers.new('Linear refinement for contact mask','SUBSURF');mod.subdivision_type='SIMPLE';mod.levels=1
        bpy.ops.object.modifier_apply(modifier=mod.name)
    for poly in o.data.polygons:poly.use_smooth=True
    count=len(o.data.vertices);times=np.full(count,100000,dtype=np.float32)
    if bi==0:
        xyz=np.empty(count*3,dtype=np.float32);o.data.vertices.foreach_get('co',xyz);xyz=xyz.reshape(-1,3)
        normals=np.empty(count*3,dtype=np.float32);o.data.vertices.foreach_get('normal',normals);normals=normals.reshape(-1,3)
        for rec in motion['frames']:
            if rec['contact_duty']<.999999:continue
            normal=np.array(rec['normal']);delta=xyz-rec['pad'];plane=delta@normal
            tang2=np.einsum('ij,ij->i',delta,delta)-plane**2
            hit=(times>rec['frame'])&(tang2<=.026**2)&(abs(plane)<.003)&(normals@normal>.2)
            times[hit]=rec['frame']
        for poly in o.data.polygons:clean_area+=poly.area*float(np.mean(times[list(poly.vertices)]<10000))
        np.savez_compressed(OUT/'mask_evidence.npz',positions=xyz,normals=normals,clean_at=times)
        # A high-resolution first-contact texture removes coarse triangle-edge
        # stepping without smoothing, enlarging, or replacing the approved mesh.
        width,height=1024,512
        tt,uu=np.meshgrid((np.arange(width)+.5)/width,2*(np.arange(height)+.5)/height-1)
        rr=.085+.395*tt;ww=(.055+.270*tt-.055*tt*tt)*np.sqrt(1-tt**4)
        theta=.12*np.sin(np.pi*tt)+uu*ww/rr
        yy=.015*tt-.754/(2*np.pi)*theta+.0035*np.sin(np.pi*tt)*(1-uu*uu)+(.012*(1-tt)+.002)*np.sqrt(1-tt**4)*np.sqrt(1-uu*uu)
        points=np.stack((rr*np.cos(theta),yy,rr*np.sin(theta)),axis=-1).reshape(-1,3)
        tex_times=np.full(len(points),10000,dtype=np.float32)
        for rec in motion['frames']:
            if rec['contact_duty']<.999999:continue
            delta=points-rec['pad'];plane=delta@np.array(rec['normal']);tang2=np.einsum('ij,ij->i',delta,delta)-plane*plane
            hit=(tex_times>rec['frame'])&(tang2<=.026**2)&(abs(plane)<.003)
            tex_times[hit]=rec['frame']
        allowed_path=HERE/'assets/allowed_nominal_mask.npz'
        if allowed_path.exists():
            allowed=np.load(allowed_path)['allowed'].ravel()
        else:
            from scipy.spatial import cKDTree
            source_path=HERE.parent/'runs/propeller_edge/coverage_mask.npz'
            source_mask=np.load(source_path)
            nearest=cKDTree(source_mask['surface_xyz_m']-BENCH_CENTER).query(points,k=4,workers=2)[1]
            allowed=np.all(np.isfinite(source_mask['nominal_first_touch_time_s'][nearest]),axis=1)
            np.savez_compressed(allowed_path,allowed=allowed.reshape(height,width))
            (HERE/'assets/allowed_mask_provenance.json').write_text(json.dumps(dict(source='runs/propeller_edge/coverage_mask.npz',sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),method='Conservative four-nearest surface-sample intersection with exported nominal cumulative footprint',shape=[height,width]),indent=2))
        tex_times[~allowed]=10000
        grid=points.reshape(height,width,3)
        area=np.linalg.norm(np.cross(np.gradient(grid,axis=1),np.gradient(grid,axis=0)),axis=2)
        clean_area=float(area[tex_times.reshape(height,width)<10000].sum())
        img=bpy.data.images.new('Exact accumulated contact time',width=width,height=height,float_buffer=True)
        img.colorspace_settings.name='Non-Color'
        rgba=np.ones((len(points),4),dtype=np.float32);rgba[:,:3]=tex_times[:,None]/10000
        img.pixels.foreach_set(rgba.ravel());img.filepath_raw=str(OUT/'contact_time.exr');img.file_format='OPEN_EXR';img.save();img.pack();img.filepath='//contact_time.exr'
        workmat=m.copy();workmat.name='Blade 0 | exact high-resolution footprint clock';o.data.materials[0]=workmat
        wn=workmat.node_tree.nodes;wl=workmat.node_tree.links;image_node=wn.new('ShaderNodeTexImage');image_node.image=img;image_node.interpolation='Linear';image_node.extension='EXTEND'
        mult=wn.new('ShaderNodeMath');mult.operation='MULTIPLY';mult.inputs[1].default_value=10000;wl.new(image_node.outputs['Color'],mult.inputs[0])
        for node in wn:
            if node.bl_idname=='ShaderNodeMath' and node.operation=='SUBTRACT':wl.new(mult.outputs[0],node.inputs[1])
        uv=o.data.uv_layers.new(name='Corrected face t,u')
        for poly in o.data.polygons:
            for li in poly.loop_indices:
                co=o.data.vertices[o.data.loops[li].vertex_index].co
                if poly.normal.y<=0:uv.data[li].uv=(0,0);continue
                rad=math.hypot(co.x,co.z);t=min(.999999,max(0,(rad-.085)/.395));r,w,skew,_,_=section(t)
                u=(math.atan2(co.z,co.x)-skew)*r/max(1e-9,w);uv.data[li].uv=(t,(u+1)/2)
        np.savez_compressed(OUT/'texture_mask_evidence.npz',points=points,clean_at=tex_times,shape=[height,width])

    attr=o.data.attributes.new('clean_at','FLOAT','POINT');attr.data.foreach_set('value',times)
    mask[str(bi)]=dict(vertices=count,clean_vertices=int(sum(times<10000)),source_stl_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
(OUT/'mask_report.json').write_text(json.dumps(dict(blades=mask,unique_clean_area_m2=clean_area,rule='First whole-frame simultaneous-contact-gated actual native-FK pad footprint: tangent radius <=26mm, normal distance <3mm, front-facing normal; no back-face/global fade',transition_frames=2,exported_nominal_footprint_intersection=True),indent=2))
# Tether and robot work lights are attached to whole-vehicle coordinates.
M=Matrix(motion['rov_transform']);dark=material('Tether | black polyurethane',(.006,.012,.013),0,.6)
start=M@Vector((-.29,.03,.07));tube('ROV / slack trailing tether',[start,start+Vector((.1,.4,-.15)),start+Vector((.25,1.1,.15)),start+Vector((-.2,2.5,.7)),start+Vector((-1,5,2.5))],.004,dark)

def light(name,position,target,power,color,size,spot=False):
    d=bpy.data.lights.new(name,'SPOT' if spot else 'AREA');d.energy=power;d.color=color
    if spot:d.spot_size=.95;d.spot_blend=.5;d.shadow_soft_size=.025
    else:d.shape='DISK';d.size=size
    ob=bpy.data.objects.new(name,d);scene.collection.objects.link(ob);ob.location=position;ob.rotation_euler=(Vector(target)-ob.location).to_track_quat('-Z','Y').to_euler();return ob
light('Ocean / broad surface light',(1,-1,4),(0,0,0),500,(.30,.70,.81),5)
light('Water / aft diffuse',(1,3,1),(.25,0,.1),180,(.52,.76,.85),3)
light('Hull / port rim',(-3,-1,.5),(0,0,.5),220,(.17,.48,.56),3)
light('ROV / metal key',(1.3,.5,1.2),(.5,.3,.1),85,(.78,.88,1),1.5)
light('Copper / warm reflected work light',(.3,.8,-.2),(.35,0,.05),14,(1,.77,.51),.4)
for y in [-.19,.19]:light('ROV / work beam',M@Vector((.27,y,.11)),(.35,0,.03),8,(.69,.84,1),.04,True)
scene.world=bpy.data.worlds.new('Deep blue-green open water');scene.world.use_nodes=True
scene.world.node_tree.nodes['Background'].inputs[0].default_value=(.023,.069,.083,1);scene.world.node_tree.nodes['Background'].inputs[1].default_value=.45
volume=bpy.data.materials.new('Water | subtle extinction');volume.use_nodes=True;nn=volume.node_tree.nodes;nn.clear();out=nn.new('ShaderNodeOutputMaterial');v=nn.new('ShaderNodeVolumePrincipled');v.inputs['Density'].default_value=.025;v.inputs['Color'].default_value=(.15,.40,.43,1);v.inputs['Anisotropy'].default_value=.25;volume.node_tree.links.new(v.outputs['Volume'],out.inputs['Volume'])
bpy.ops.mesh.primitive_cube_add(size=1,location=(0,0,2));water=bpy.context.object;water.name='Open water / no seabed';water.scale=(30,35,24);water.data.materials.append(volume)
# Discrete small suspended particles, sparse enough to preserve engineering detail.
rng=random.Random(940);pm=material('Water / suspended particles',(.16,.24,.19),0,.9)
vv=[];ff=[]
for i in range(160):
    x,y,z=rng.uniform(-2,2),rng.uniform(-.3,3),rng.uniform(-1,1.7);r=rng.uniform(.0003,.0012);k=len(vv)
    vv.extend([(x+r,y,z),(x-r,y,z),(x,y+r,z),(x,y,z+r)]);ff.extend([(k,k+2,k+3),(k+1,k+3,k+2),(k,k+3,k+1),(k,k+1,k+2)])
from stern import mesh
mesh('Open water / suspended particulate',vv,ff,pm)

from camera_sequence import build as build_cameras
build_cameras(scene)

for action in bpy.data.actions:
    for layer in action.layers:
        for strip in layer.strips:
            for bag in strip.channelbags:
                for curve in bag.fcurves:
                    for k in curve.keyframe_points:k.interpolation='LINEAR'
scene.render.engine='CYCLES';scene.cycles.samples=32;scene.cycles.use_denoising=True;scene.cycles.max_bounces=6;scene.cycles.volume_bounces=1
prefs=bpy.context.preferences.addons['cycles'].preferences;prefs.compute_device_type='OPTIX';prefs.get_devices()
for d in prefs.devices:d.use=d.type=='OPTIX'
scene.cycles.device='GPU';scene.render.use_persistent_data=True;scene.render.resolution_x=1280;scene.render.resolution_y=720;scene.render.resolution_percentage=100;scene.render.image_settings.file_format='PNG';scene.render.film_transparent=False
scene.view_settings.view_transform='AgX';scene.view_settings.look='AgX - Medium High Contrast';scene.view_settings.exposure=.25
scene['provenance']='2026-09-18 original BlueROV2 Heavy with Viscous arms; native mounts unchanged; four-blade 960mm corrected propeller'
scene['validation_scope']=motion['physics_claim'];scene['mask_scope']='Only contact-gated actual native-FK footprints reveal copper. Fouling thickness visual only.'
scene.render.filepath='//preview.png'
scene.frame_set(a.preview);bpy.ops.file.pack_all();bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'final_scene.blend'),compress=True)
if not a.no_render:
    scene.render.filepath=str(OUT/f'preview_{a.preview:04d}.png');bpy.ops.render.render(write_still=True)
print('SCENE_COMPLETE',flush=True)
