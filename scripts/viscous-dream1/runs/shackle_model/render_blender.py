"""Local Blender stills only. No scene simulation; all output stays here."""
from pathlib import Path
import json
import math
import bpy
from mathutils import Vector, Matrix

bpy.ops.wm.read_factory_settings(use_empty=True)
RUN=Path(__file__).resolve().parent
ROOT=RUN.parents[1]
data=json.loads((RUN/'render_geometry.json').read_text())
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
scene=bpy.context.scene
scene.use_nodes=False;scene.render.use_compositing=False
scene.render.engine='CYCLES'
scene.cycles.device='CPU';scene.cycles.samples=32
scene.cycles.use_denoising=True
scene.eevee.use_gtao=True;scene.eevee.gtao_distance=25;scene.eevee.gtao_factor=1.2
scene.eevee.taa_render_samples=128
scene.render.resolution_x=1000;scene.render.resolution_y=1000;scene.render.resolution_percentage=100
scene.render.image_settings.file_format='OPEN_EXR'
scene.render.image_settings.color_depth='32'
scene.world=bpy.data.worlds.new('studio world')
scene.world.use_nodes=True
scene.world.node_tree.nodes['Background'].inputs['Color'].default_value=(.3,.3,.3,1)
scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value=.5
scene.view_settings.view_transform='Standard'
scene.view_settings.look='None'
scene.view_settings.exposure=0;scene.view_settings.gamma=1

def material(name,color,metallic=0,roughness=.4):
    mat=bpy.data.materials.new(name);mat.use_nodes=True
    p=mat.node_tree.nodes.get('Principled BSDF');p.inputs['Base Color'].default_value=(*color,1)
    p.inputs['Metallic'].default_value=metallic;p.inputs['Roughness'].default_value=roughness
    return mat

steel=material('satin stainless',(.52,.56,.60),.78,.24)
floor_mat=material('neutral matte',(.24,.26,.29),0,.8)
objects={}
for name,g in data['meshes'].items():
    mesh=bpy.data.meshes.new(name);mesh.from_pydata(g['vertices'],[],g['faces']);mesh.update()
    ob=bpy.data.objects.new(name,mesh);scene.collection.objects.link(ob);ob.data.materials.append(steel)
    for f in mesh.polygons:f.use_smooth=True
    mesh.use_auto_smooth=True;mesh.auto_smooth_angle=math.radians(38)
    objects[name]=ob

bpy.ops.mesh.primitive_plane_add(size=2000,location=(0,0,-21))
floor=bpy.context.object;floor.name='render ground';floor.data.materials.append(floor_mat)
for name,pos,power,size in [('key',(-120,-170,240),1500000,180),('fill',(120,100,220),1000000,150),('rim',(-160,140,130),1200000,120)]:
    light=bpy.data.lights.new(name,'AREA');light.energy=power;light.shape='DISK';light.size=size
    ob=bpy.data.objects.new(name,light);scene.collection.objects.link(ob);ob.location=pos
    ob.rotation_euler=(Vector((-35,0,20))-ob.location).to_track_quat('-Z','Y').to_euler()
camera=bpy.data.cameras.new('comparison camera');cam=bpy.data.objects.new('comparison camera',camera)
scene.collection.objects.link(cam);scene.camera=cam;camera.type='ORTHO';camera.lens=55
camera.clip_end=10000

def render(name,which,rotation,eye,target,scale,ground):
    for n,ob in objects.items():
        ob.hide_render=n not in which;ob.matrix_world=Matrix.Identity(4)
        if n=='shackle_pin':ob.location=data['seated_translation_mm']
        ob.matrix_world=rotation@ob.matrix_world
    floor.hide_render=ground is None
    if ground is not None:floor.location.z=ground
    cam.location=eye;cam.rotation_euler=(Vector(target)-cam.location).to_track_quat('-Z','Y').to_euler()
    camera.ortho_scale=scale
    scene.render.filepath=str(RUN/(name+'.exr'));bpy.ops.render.render(write_still=True)

identity=Matrix.Identity(4)
render('assembled',['shackle_body','shackle_pin'],identity,(150,-280,160),(-36,0,7),177,-50)
# Exploded view is a raised pin in its actual insertion orientation.
data['seated_translation_mm'][2]+=92
render('exploded',['shackle_body','shackle_pin'],identity,(140,-290,165),(-28,0,50),255,-50)
data['seated_translation_mm'][2]-=92
render('render_m289',['shackle_body'],Matrix.Rotation(math.pi/2,4,'X'),(85,-210,270),(-40,0,0),157,-20.1)
render('render_m296',['shackle_body','shackle_pin'],Matrix.Rotation(-math.pi/2,4,'X'),(-10,-120,360),(-40,8,0),160,-20.1)
# m288: standalone pin tip exactly on the table, cross-hole facing camera.
data['seated_translation_mm']=[0,0,0]
render('render_m288',['shackle_pin'],identity,(40,-290,132),(0,0,53),142,0)
# Annotatable orthographic reference, body in requested task frame.
render('frame_reference',['shackle_body','shackle_pin'],identity,(0,-400,0),(-38,0,10),170,None)
