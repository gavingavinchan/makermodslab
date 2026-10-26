# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Approved presentation-only replacement for the transparent acrylic housings."""
from pathlib import Path
import json
import bpy

TARGETS={'rov_base__rov_base_09':'CAD_9','rov_base__rov_base_41':'CAD_41'}
NAME='ROV | Satin black anodized aluminium canisters'


def apply():
    mat=bpy.data.materials.new(NAME);mat.use_nodes=True
    nodes=mat.node_tree.nodes;nodes.clear()
    bs=nodes.new('ShaderNodeBsdfPrincipled');bs.name='Opaque anodized aluminium'
    bs.inputs['Base Color'].default_value=(.018,.023,.028,1)
    bs.inputs['Metallic'].default_value=.95
    bs.inputs['Roughness'].default_value=.38
    bs.inputs['Anisotropic'].default_value=.22
    bs.inputs['Transmission Weight'].default_value=0
    bs.inputs['Alpha'].default_value=1
    out=nodes.new('ShaderNodeOutputMaterial');mat.node_tree.links.new(bs.outputs['BSDF'],out.inputs['Surface'])
    changed=[]
    for obj in bpy.context.scene.objects:
        if obj.type!='MESH':continue
        for slot in obj.material_slots:
            if slot.material and slot.material.name in TARGETS.values():
                assert TARGETS.get(obj.name)==slot.material.name,('Unexpected canister source user',obj.name)
                slot.material=mat;changed.append(obj.name)
    assert sorted(changed)==sorted(TARGETS),changed
    report=dict(changed_objects=changed,source_materials=TARGETS,replacement_material=NAME,alpha=1.,transmission=0.,metallic=.95,roughness=.38,geometry_unchanged=True,scope='Gavin requested replacing acrylic canisters with opaque black anodized aluminium; original source CAD asset remains untouched')
    (Path(__file__).parent/'results/appearance_report.json').write_text(json.dumps(report,indent=2)+'\n')
