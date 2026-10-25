# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Read-only scene check of the replacement tool at the native +20 mm seat.

Runs in its own Linux Blender Python process. Writes only tool/native_fit_check.json;
never saves or modifies the coordinator's scene file. Not an all-frame audit.
"""
import hashlib
import json
import argparse

import bpy
import numpy as np
from mathutils import Matrix
from mathutils.bvhtree import BVHTree

from build_polisher import HERE


def fingerprint(obj):
    return hashlib.sha256(repr(([tuple(v.co) for v in obj.data.vertices],
                               [tuple(p.vertices) for p in obj.data.polygons])).encode()).hexdigest()


def geometry(obj, transform):
    evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    mesh = evaluated.to_mesh()
    vertices = np.array([tuple(v.co) for v in mesh.vertices])
    matrix = transform @ np.array(obj.matrix_world)
    vertices = vertices @ matrix[:3, :3].T + matrix[:3, 3]
    faces = [tuple(p.vertices) for p in mesh.polygons]
    evaluated.to_mesh_clear()
    return vertices, faces


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sweep", action="store_true")
    args = parser.parse_args()
    bpy.ops.wm.open_mainfile(filepath=str(HERE.parent / "results/final_scene.blend"))
    bpy.context.scene.frame_set(350)
    hand = bpy.data.objects["right_tool0"]
    inv = np.linalg.inv(np.array(hand.matrix_world))
    arms = [o for o in bpy.context.scene.objects if o.type == "MESH"
            and o.name.startswith(("right_gripper_", "right_wrist_"))]
    native = {o.name: geometry(o, inv) for o in arms}
    native_bvh = {n: BVHTree.FromPolygons(v, f) for n, (v, f) in native.items()}
    old = bpy.data.collections["CleaningTool"]
    before = {o.name: fingerprint(o) for o in old.all_objects if o.type == "MESH"}
    for obj in list(old.all_objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.collections.remove(old)
    with bpy.data.libraries.load(str(HERE / "polisher.blend"), link=False) as (_, dst):
        dst.collections = ["CleaningTool"]
    bpy.context.scene.collection.children.link(dst.collections[0])
    root = bpy.data.objects["CleaningToolRoot"]
    root.matrix_world = hand.matrix_world @ Matrix.Translation((0, 0, .020))
    bpy.context.view_layer.update()
    meshes = [o for o in dst.collections[0].all_objects if o.type == "MESH"]
    if args.sweep:
        rows = []
        for factor in np.arange(1, .39, -.05):
            issues = []
            for obj in meshes:
                if not obj.name.startswith(("ToolRearSeal", "ToolRearCap", "ToolRearIndexRing")):
                    continue
                vertices, faces = geometry(obj, inv)
                vertices[:, :2] *= factor
                tree = BVHTree.FromPolygons(vertices, faces)
                for name, native_tree in native_bvh.items():
                    count = len(tree.overlap(native_tree))
                    if count:
                        issues.append(dict(tool=obj.name, native=name, triangle_pairs=count))
            rows.append(dict(radial_scale=float(factor), issues=issues))
            print(round(factor, 3), sum(i["triangle_pairs"] for i in issues), flush=True)
        report = dict(source_blend_sha256=hashlib.sha256((HERE/"polisher.blend").read_bytes()).hexdigest(),
                      source_rear_radius_m=max(float(np.linalg.norm(geometry(o, inv)[0][:, :2], axis=1).max())
                                               for o in meshes if o.name.startswith("ToolRear")),
                      frame=350, axial_seat_m=.020, rows=rows)
        (HERE/"rear_radius_fit.json").write_text(json.dumps(report, indent=2)+"\n")
        return
    changed = [o.name for o in meshes if before.get(o.name) != fingerprint(o)]
    assert all(n.startswith("ToolRear") for n in changed), changed
    issues = []
    for obj in meshes:
        if obj.name == "ToolGraspSleeve":
            continue
        vertices, faces = geometry(obj, inv)
        tree = None
        for name, (v, _) in native.items():
            if np.any(vertices.max(0) < v.min(0)) or np.any(v.max(0) < vertices.min(0)):
                continue
            if tree is None:
                tree = BVHTree.FromPolygons(vertices, faces)
            count = len(tree.overlap(native_bvh[name]))
            if count:
                issues.append(dict(tool=obj.name, native=name, triangle_pairs=count))
    result = dict(passed=not issues, frame=350, native_tool0_axial_seat_m=.020,
                  replacement_blend_sha256=hashlib.sha256((HERE/"polisher.blend").read_bytes()).hexdigest(),
                  changed_meshes=sorted(changed), unchanged_other_meshes=True,
                  tested_tool_meshes=len(meshes)-1, native_hand_meshes=len(arms), issues=issues,
                  method="Native visible mesh BVH triangles, replacement appended in memory; sleeve contact excluded",
                  limits="One frame only, all-frame audit belongs to scene coordinator; scene file never saved")
    (HERE/"native_fit_check.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))
    assert result["passed"], issues


if __name__ == "__main__":
    main()
