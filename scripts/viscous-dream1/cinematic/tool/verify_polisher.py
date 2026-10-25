# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Focused saved-file and embedding checks; run with existing Linux bpy Python."""

import hashlib
import json

import bpy

from build_polisher import HERE, audit, create_tool


def signature(objects):
    h = hashlib.sha256()
    for obj in sorted(objects, key=lambda o: o.name):
        h.update(obj.name.encode())
        if obj.type == "MESH":
            for vertex in obj.data.vertices:
                h.update(str(tuple(vertex.co)).encode())
            for face in obj.data.polygons:
                h.update(str(tuple(face.vertices)).encode())
    return h.hexdigest()


def main():
    bpy.ops.wm.open_mainfile(filepath=str(HERE/"polisher.blend"))
    collection = bpy.data.collections["CleaningTool"]
    asset = dict(collection=collection, root=bpy.data.objects["CleaningToolRoot"],
                 grip_anchor=bpy.data.objects["ToolGripAnchor"],
                 pad_contact_anchor=bpy.data.objects["ToolPadContactAnchor"],
                 rotor=bpy.data.objects["ToolRotor"])
    check = audit(asset)
    assert len(bpy.data.collections) == 1
    assert set(bpy.context.scene.objects) == set(collection.objects)
    assert all(o.type in {"MESH", "EMPTY"} for o in collection.objects)
    root = asset["root"]
    assert root.parent is None
    assert all(o == root or o.parent in {root, asset["rotor"]} for o in collection.objects)
    assert bpy.data.objects["ToolAbrasivePad"].parent == asset["rotor"]
    assert bpy.data.objects["ToolGraspSleeve"].parent == root
    original = signature(collection.objects)
    try:
        create_tool()
    except ValueError:
        pass
    else:
        raise AssertionError("create_tool must refuse duplicate canonical names")
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sentinel = bpy.data.objects.new("ExistingSceneObject", None)
    bpy.context.scene.collection.objects.link(sentinel)
    sentinel.location = (1, 2, 3)
    rebuilt = create_tool()
    assert signature(rebuilt["collection"].objects) == original
    assert tuple(sentinel.location) == (1, 2, 3)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    with bpy.data.libraries.load(str(HERE/"polisher.blend"), link=False) as (src, dst):
        assert "CleaningTool" in src.collections
        dst.collections = ["CleaningTool"]
    bpy.context.scene.collection.children.link(dst.collections[0])
    assert signature(dst.collections[0].objects) == original
    # Blender may retain source-library bookkeeping after a native append;
    # the appended objects and all their render data must nevertheless be local.
    for obj in dst.collections[0].objects:
        assert obj.library is None
        if obj.type == "MESH":
            assert obj.data.library is None
            assert all(mat.library is None for mat in obj.data.materials)
    previews = {}
    for name in ("hero", "pad", "profile"):
        p = HERE/f"preview_{name}.png"
        img = bpy.data.images.load(str(p))
        assert tuple(img.size) == (1280, 720)
        previews[p.name] = dict(size=list(img.size), sha256=hashlib.sha256(p.read_bytes()).hexdigest())
        bpy.data.images.remove(img)
    result = dict(passed=True, blender=bpy.app.version_string,
                  saved_geometry_checks_passed=check["passed"],
                  clean_asset_no_camera_lights=True, native_append_passed=True,
                  deterministic_rebuild_geometry_sha256=original,
                  create_tool_preserves_scene=True, canonical_name_collision_refused=True,
                  hierarchy_passed=True, previews=previews)
    (HERE/"saved_asset_check.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
