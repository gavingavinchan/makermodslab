# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Add source-CAD handed PLA badges, without changing presentation or motion.

Run with the cinematic bpy Python. With no arguments, update native_rov.blend and
final_scene.blend in place. apply() is also called by build_scene.py. All source
triangles and registration evidence are vendored in assets/logo_badges.
"""
from pathlib import Path
import argparse,json,shutil
import bpy
from mathutils import Matrix
HERE=Path(__file__).resolve().parent
ASSETS=HERE/'assets/logo_badges'
COLLECTION='LOGO_BADGES'

def srgb(v):
    return v/12.92 if v<=.04045 else ((v+.055)/1.055)**2.4

def mount(spec,arm):
    """Print mm -> positive-determinant native link metres.

    Right ROV geometry is reflected in Y. Select its outward-side handed part
    directly instead of propagating that reflection into the text/glyph mesh.
    """
    reg=json.loads((ASSETS/'registration.json').read_text())['arms'][arm]
    t=reg['source_preview_arm_to_link_mm']
    side=spec['side'];nameplate=spec['kind']=='nameplate'
    x=(79 if nameplate else 33.94112)-160+t[0][3]
    z=58.78775+111+t[2][3]
    y=side*(29.5 if nameplate else 37.5)+t[1][3]
    return Matrix(((-side,0,0,x*.001),(0,0,side,y*.001),(0,1,0,z*.001),(0,0,0,1)))

def apply():
    if COLLECTION in bpy.data.collections:
        old=bpy.data.collections[COLLECTION]
        for o in list(old.all_objects):
            mesh=o.data;bpy.data.objects.remove(o,do_unlink=True)
            if mesh is not None and mesh.users==0:bpy.data.meshes.remove(mesh)
        bpy.data.collections.remove(old)
    col=bpy.data.collections.new(COLLECTION);bpy.context.scene.collection.children.link(col)
    mats={}
    for label,color in [('blue',tuple(srgb(x/255) for x in (45,107,255))),('white',(1,1,1))]:
        name=f'BADGE | {label} PLA';m=bpy.data.materials.get(name) or bpy.data.materials.new(name)
        m.use_nodes=True;m.diffuse_color=(*color,1)
        b=m.node_tree.nodes.get('Principled BSDF');b.inputs['Base Color'].default_value=(*color,1)
        b.inputs['Metallic'].default_value=0;b.inputs['Roughness'].default_value=.62
        b.inputs['Coat Weight'].default_value=0;b.inputs['Specular IOR Level'].default_value=.28
        mats[label]=m
    for arm in ('left','right'):
        parent=bpy.data.objects[arm+'_forearm_link']
        assert parent.matrix_world.determinant()>0,'Unexpected reflected link object'
        for spec in json.loads((ASSETS/'parts.json').read_text()):
            name='BADGE | '+arm+' | '+spec['name']
            me=bpy.data.meshes.new(name);me.from_pydata([[x*.001 for x in p] for p in spec['vertices']],[],spec['faces']);me.update()
            ob=bpy.data.objects.new(name,me);col.objects.link(ob)
            ob.parent=parent;ob.matrix_parent_inverse=Matrix.Identity(4);ob.matrix_basis=mount(spec,arm)
            color='white' if spec['name'].endswith('_white') else 'blue';me.materials.append(mats[color])
            ob['source_part']=spec['part'];ob['outward_side']=spec['side'];ob['print_units']='mm; geometry converted to metres';ob['mount']='source mounted-reference rigid transform registered to forearm bracket'
            assert ob.matrix_basis.determinant()>0
    return col

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('files',nargs='*',type=Path);a=p.parse_args()
    for path in a.files or [HERE/'assets/native_rov.blend',HERE/'results/final_scene.blend']:
        path=path.resolve();backup=path.with_suffix('.prelogo.blend')
        if not backup.exists():shutil.copy2(path,backup)
        bpy.ops.wm.open_mainfile(filepath=str(path));apply();bpy.ops.file.pack_all()
        bpy.ops.wm.save_as_mainfile(filepath=str(path),compress=True)
        print('BADGES_SAVED',path,flush=True)
if __name__=='__main__':main()
