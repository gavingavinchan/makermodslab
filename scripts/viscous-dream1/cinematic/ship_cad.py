# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Load tessellated official KCS CAD; mirror its half hull and adapt uniformly.

The KCS hull is a geometric basis for this presentation, not a claim to depict
the original full-scale KCS installation: its five-blade rotor is replaced by
the user's approved four-blade 960 mm rotor. See assets/ship/provenance.json.
"""
from pathlib import Path
import json
import bpy

HERE = Path(__file__).parent
SCALE = .96 / 7.9 / 1000  # full-scale mm -> presentation metres
PROPELLER_X_MM = .0175 * 230000
PROPELLER_Z_MM = 10800 - .02913 * 230000


def cad_mesh(filename, name, material, mirror=False):
    data = json.loads((HERE/'assets/ship'/filename).read_text())
    verts = [(y*SCALE, (PROPELLER_X_MM-x)*SCALE, (z-PROPELLER_Z_MM)*SCALE)
             for x,y,z in data['vertices']]
    faces = [tuple(f) for f in data['faces']]
    normals = [(y,-x,z) for x,y,z in data['normals']]
    if mirror:
        offset = len(verts)
        verts += [(-x,y,z) for x,y,z in verts]
        faces += [tuple(i+offset for i in reversed(f)) for f in faces.copy()]
        normals += [(-x,y,z) for x,y,z in normals.copy()]
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    mesh.materials.append(material)
    for poly in mesh.polygons: poly.use_smooth = True
    # CAD patches tessellate independently. Preserve their analytic surface
    # normals rather than introducing shading bands at unmatched edge samples.
    mesh.normals_split_custom_set_from_vertices(normals)
    obj['source'] = filename
    obj['uniform_scale_m_per_source_mm'] = SCALE
    obj['source_propeller_center_mm'] = [PROPELLER_X_MM, 0, PROPELLER_Z_MM]
    return obj


def build_ship(material):
    hull = cad_mesh('KCS_hull_SVA.json', 'Ship / imported KCS hull CAD', material, mirror=True)
    rudder = cad_mesh('KCS_full-scale-rudder.json', 'Ship / imported KCS horn rudder CAD', material)
    return hull, rudder
