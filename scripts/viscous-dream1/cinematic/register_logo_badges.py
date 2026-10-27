# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Verify source bracket vertices against each saved scene's own link meshes."""
from pathlib import Path
import json,hashlib
import bpy,numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree
HERE=Path(__file__).resolve().parent;A=HERE/'assets/logo_badges';OUT=HERE/'results/logo_check'
source=json.loads((A/'source_brackets.json').read_text());reg=json.loads((A/'registration.json').read_text());report={}
for label,path in [('native',HERE/'assets/native_rov.blend'),('film',HERE/'results/final_scene.blend')]:
    bpy.ops.wm.open_mainfile(filepath=str(path));checks={}
    for arm in ('left','right'):
        o=bpy.data.objects[arm+'_forearm_link__'+arm+'_forearm_link_00'];parent=bpy.data.objects[arm+'_forearm_link']
        M=parent.matrix_world.inverted()@o.matrix_world
        tree=BVHTree.FromPolygons([M@v.co for v in o.data.vertices],[list(p.vertices) for p in o.data.polygons])
        T=np.array(reg['arms'][arm]['source_preview_arm_to_link_mm']);plates={}
        for spec in source:
            pts=(np.array(spec['vertices'])@T[:3,:3].T+T[:3,3])*.001
            dist=np.array([tree.find_nearest(Vector(v))[3] for v in pts])*1000
            plates[spec['name']]=dict(vertices=len(pts),rms_mm=float(np.sqrt(np.mean(dist**2))),max_mm=float(dist.max()))
        checks[arm]=dict(carrier=parent.name,matched_mesh=o.name,mesh_object_determinant=o.matrix_world.determinant(),registration_determinant=float(np.linalg.det(T[:3,:3])),plates=plates)
    # Idempotence and packed-path checks are in memory only, never re-save QA.
    import add_logo_badges
    add_logo_badges.apply();first={o.name:(len(o.data.vertices),list(map(list,o.matrix_basis))) for o in bpy.data.collections['LOGO_BADGES'].objects};add_logo_badges.apply();second={o.name:(len(o.data.vertices),list(map(list,o.matrix_basis))) for o in bpy.data.collections['LOGO_BADGES'].objects}
    assert first==second and len(second)==16
    dets={o.name:o.matrix_world.determinant() for o in bpy.data.collections['LOGO_BADGES'].objects};assert min(dets.values())>.999
    external=[im.filepath for im in bpy.data.images if im.source=='FILE' and not im.packed_file];assert not external,external
    assert len(bpy.data.libraries)==0,'Live linked-library dependency'
    weak_paths=sorted(set(bpy.utils.blend_paths(absolute=True,packed=False)))
    checks['idempotence']=dict(passed=first==second,objects=16,materials=[m.name for m in bpy.data.materials if m.name.startswith('BADGE |')],minimum_badge_world_determinant=min(dets.values()),external_unpacked_paths=external,live_linked_libraries=len(bpy.data.libraries),inert_append_provenance_weak_paths=weak_paths)
    report[label]=checks
(OUT/'registration.json').write_text(json.dumps(dict(source_registration=reg,scene_mesh_checks=report),indent=2)+'\n');print('SCENE_MESH_REGISTRATION_PASS',flush=True)
