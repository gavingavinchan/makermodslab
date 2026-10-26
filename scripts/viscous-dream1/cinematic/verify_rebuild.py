# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Rebuild an isolated repo-local package and compare its actual scene."""
from pathlib import Path
import argparse, hashlib, json, os, shutil, subprocess, sys
import bpy
import numpy as np
from native_appearance import TARGETS

HERE=Path(__file__).parent
p=argparse.ArgumentParser();p.add_argument('--scratch',type=Path,required=True);a=p.parse_args()
if a.scratch.exists():raise FileExistsError(a.scratch)
dst=a.scratch/'cinematic';dst.parent.mkdir(parents=True)
shutil.copytree(HERE,dst,ignore=shutil.ignore_patterns('results','__pycache__','.cache','*.blend1','*.log'))
(dst/'results').mkdir()
deps=a.scratch/'assets/propeller_corrected';deps.mkdir(parents=True)
for i in range(4):shutil.copy2(HERE.parent/f'assets/propeller_corrected/blade_{i}.stl',deps)
env=dict(os.environ,OPENBLAS_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
for script,extra in [('plan_motion.py',[]),('build_scene.py',['--no-render'])]:
    with (a.scratch/(script+'.log')).open('w') as log:
        subprocess.run([sys.executable,str(dst/script),*extra],cwd=dst,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
checks={}
for name in ('motion.json','texture_mask_evidence.npz','mask_report.json'):
    selected=(HERE/'results'/name).read_bytes();rebuilt=(dst/'results'/name).read_bytes()
    checks[name]=dict(identical=selected==rebuilt,selected_sha256=hashlib.sha256(selected).hexdigest(),rebuilt_sha256=hashlib.sha256(rebuilt).hexdigest())
frames=[1,25,48,49,120,121,350,600,601,624,625,672]
def snapshot(path):
    bpy.ops.wm.open_mainfile(filepath=str(path));scene=bpy.context.scene;geometry={};poses={}
    for o in scene.objects:
        if o.type=='MESH':
            xyz=np.empty(len(o.data.vertices)*3,dtype=np.float32);o.data.vertices.foreach_get('co',xyz)
            indices=np.concatenate([np.array(f.vertices,dtype=np.int32) for f in o.data.polygons])
            geometry[o.name]=hashlib.sha256(xyz.tobytes()+indices.tobytes()).hexdigest()
    for f in frames:
        scene.frame_set(f);bpy.context.view_layer.update()
        poses[f]={o.name:np.array(o.matrix_world) for o in scene.objects}
        poses[f].update({o.name+' / lens':np.array([o.data.lens]) for o in scene.objects if o.type=='CAMERA'})
    appearance={}
    for name in TARGETS:
        mat=bpy.data.objects[name].active_material;bs=mat.node_tree.nodes['Opaque anodized aluminium']
        appearance[name]=dict(material=mat.name,base_color=list(bs.inputs['Base Color'].default_value),metallic=bs.inputs['Metallic'].default_value,roughness=bs.inputs['Roughness'].default_value,alpha=bs.inputs['Alpha'].default_value,transmission=bs.inputs['Transmission Weight'].default_value)
    return geometry,poses,appearance
sg,sp,sm=snapshot(HERE/'results/final_scene.blend');rg,rp,rm=snapshot(dst/'results/final_scene.blend')
same_names=all(sp[f].keys()==rp[f].keys() for f in frames)
error=max(float(np.max(abs(sp[f][n]-rp[f][n]))) for f in frames for n in sp[f]) if same_names else float('inf')
report=dict(passed=all(v['identical'] for v in checks.values()) and sg==rg and error<1e-6 and sm==rm,method='Self-contained package copied to isolated Linux scratch with no sibling source checkout; native IK plan and no-render scene rebuilt',checks=checks,scene_mesh_geometry_identical=sg==rg,scene_object_names_identical=same_names,scene_all_object_world_transform_and_lens_max_error=error,canister_materials_identical=sm==rm,canister_materials=sm,frames_compared=frames,selected_scene_sha256=hashlib.sha256((HERE/'results/final_scene.blend').read_bytes()).hexdigest())
(HERE/'results/rebuild_verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2),flush=True)
assert report['passed']
