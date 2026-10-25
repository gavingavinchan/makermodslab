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
"""Original, procedural cinematic polisher. All coordinates are metres.

Import in Blender and call create_tool(); append the CleaningTool collection
from polisher.blend alternatively. No scene reset, lights, physics or file writes
occur on import/create_tool. The CLI creates a clean asset and optional previews.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

import bpy
from mathutils import Vector

HERE = Path(__file__).resolve().parent
ROOT_NAME = "CleaningToolRoot"
COLLECTION_NAME = "CleaningTool"


def material(name, color, metallic, roughness, grain=0, bump=0):
    mat = bpy.data.materials.new("Tool_" + name)
    mat.diffuse_color = (*color, 1)
    mat.use_nodes = True
    nodes, links = mat.node_tree.nodes, mat.node_tree.links
    bsdf = nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*color, 1)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    if grain:
        coords = nodes.new("ShaderNodeTexCoord")
        noise = nodes.new("ShaderNodeTexNoise")
        noise.inputs["Scale"].default_value = grain
        noise.inputs["Detail"].default_value = 2.0
        links.new(coords.outputs["Object"], noise.inputs["Vector"])
        ramp = nodes.new("ShaderNodeValToRGB")
        ramp.color_ramp.elements[0].position = .15
        ramp.color_ramp.elements[0].color = (*(c * .68 for c in color), 1)
        ramp.color_ramp.elements[1].position = .85
        ramp.color_ramp.elements[1].color = (*(min(c * 1.25, 1) for c in color), 1)
        if name == "AbrasiveMineral":
            ramp.color_ramp.elements[0].position = .34
            ramp.color_ramp.elements[0].color = (*(c * .30 for c in color), 1)
            ramp.color_ramp.elements[1].position = .66
            ramp.color_ramp.elements[1].color = (*(min(c * 1.8, 1) for c in color), 1)
        links.new(noise.outputs["Fac"], ramp.inputs["Fac"])
        links.new(ramp.outputs["Color"], bsdf.inputs["Base Color"])
        if bump:
            bn = nodes.new("ShaderNodeBump")
            bn.inputs["Strength"].default_value = .24
            bn.inputs["Distance"].default_value = bump
            links.new(noise.outputs["Fac"], bn.inputs["Height"])
            links.new(bn.outputs["Normal"], bsdf.inputs["Normal"])
    return mat


class Builder:
    def __init__(self, collection):
        self.collection = collection
        self.root = self.empty(ROOT_NAME)

    def empty(self, name, parent=None, location=(0, 0, 0)):
        obj = bpy.data.objects.new(name, None)
        self.collection.objects.link(obj)
        obj.parent = parent
        obj.location = location
        obj.empty_display_type = "ARROWS"
        obj.empty_display_size = .009
        return obj

    def mesh(self, name, verts, faces, mat, parent=None, smooth=True):
        mesh = bpy.data.meshes.new(name + "Mesh")
        mesh.from_pydata(verts, [], faces)
        mesh.update()
        obj = bpy.data.objects.new(name, mesh)
        self.collection.objects.link(obj)
        obj.parent = self.root if parent is None else parent
        obj.data.materials.append(mat)
        for p in mesh.polygons:
            p.use_smooth = smooth
        return obj

    def lathe(self, name, profile, mat, parent=None, segments=128):
        # Ordered (z, radius) profile, bottom to top. Bevels are explicit geometry.
        verts = [(r * math.cos(2 * math.pi * i / segments),
                  r * math.sin(2 * math.pi * i / segments), z)
                 for z, r in profile for i in range(segments)]
        faces = []
        for row in range(len(profile) - 1):
            for i in range(segments):
                j = (i + 1) % segments
                a, b = row * segments, (row + 1) * segments
                faces.append((a + i, a + j, b + j, b + i))
        faces += [tuple(reversed(range(segments))),
                  tuple((len(profile) - 1) * segments + i for i in range(segments))]
        obj = self.mesh(name, verts, faces, mat, parent)
        obj.data.polygons[-1].use_smooth = False
        obj.data.polygons[-2].use_smooth = False
        return obj

    def shell(self, name, rings, mat):
        # Rounded-square sections, transitioning into circular grip and spindle.
        n = 128
        verts = []
        for z, rx, ry, power in rings:
            for i in range(n):
                a = 2 * math.pi * i / n
                c, s = math.cos(a), math.sin(a)
                verts.append((rx * math.copysign(abs(c) ** (2 / power), c),
                              ry * math.copysign(abs(s) ** (2 / power), s), z))
        faces = [(k*n+i, k*n+(i+1)%n, (k+1)*n+(i+1)%n, (k+1)*n+i)
                 for k in range(len(rings)-1) for i in range(n)]
        faces += [tuple(reversed(range(n))), tuple((len(rings)-1)*n+i for i in range(n))]
        return self.mesh(name, verts, faces, mat)

    def screw(self, name, xyz, normal, steel, dark):
        # Actual recessed six-lobed drive: annular rim, socket walls and floor.
        n = 72
        rings = [(.00215, -.00045), (.00215, .0001), (.00195, .00035),
                 (.00103, .00035), (.00103, -.00005)]
        verts = []
        for ri, (rad, z) in enumerate(rings):
            for i in range(n):
                a = i * 2 * math.pi / n
                r = rad * (1 + .16 * math.cos(6*a)) if ri >= 3 else rad
                verts.append((r * math.cos(a), r * math.sin(a), z))
        faces = [(k*n+i, k*n+(i+1)%n, (k+1)*n+(i+1)%n, (k+1)*n+i)
                 for k in range(4) for i in range(n)]
        faces.append(tuple(4*n+i for i in range(n)))
        obj = self.mesh(name, verts, faces, steel)
        obj.data.materials.append(dark)
        obj.data.polygons[-1].material_index = 1
        obj.location = xyz
        obj.rotation_euler = Vector(normal).to_track_quat("Z", "Y").to_euler()
        return obj


def create_tool(collection=None):
    """Create a single canonical tool without changing the caller's scene.

    collection may be an empty Blender collection named CleaningTool; otherwise
    it is created and linked to the active scene. Existing canonical objects are
    refused, never replaced. Returns root, grip_anchor, pad_contact_anchor, rotor,
    collection. Parent/transform root to mount; rotate rotor.rotation_euler.z.
    """
    for name in (ROOT_NAME, "ToolGripAnchor", "ToolPadContactAnchor", "ToolRotor"):
        if name in bpy.data.objects:
            raise ValueError(f"Canonical tool object already exists: {name}")
    if collection is None:
        if COLLECTION_NAME in bpy.data.collections:
            raise ValueError("CleaningTool collection already exists")
        collection = bpy.data.collections.new(COLLECTION_NAME)
        bpy.context.scene.collection.children.link(collection)
    elif collection.name != COLLECTION_NAME or len(collection.objects):
        raise ValueError("Pass an empty collection named CleaningTool")
    b = Builder(collection)
    rubber = material("GraspRubber", (.032, .039, .043), 0, .60, 3800, .00004)
    blue = material("SatinMarineBlue", (.018, .115, .165), .68, .29, 3900, .000008)
    steel = material("MachinedStainless", (.43, .48, .51), .95, .24, 9500, .000004)
    black = material("BlackPolymer", (.014, .020, .025), .05, .36, 6000, .000012)
    dark = material("Recess", (.006, .009, .012), .20, .40)
    amber = material("AmberAnodizedIndex", (.84, .32, .035), .62, .30)
    foam = material("CompliantBacking", (.041, .047, .049), 0, .76, 3100, .00008)
    grit = material("AbrasiveMineral", (.22, .12, .065), .08, .88, 3400, .00015)
    grip = b.empty("ToolGripAnchor", b.root)
    pad_anchor = b.empty("ToolPadContactAnchor", b.root, (0, 0, .081))
    rotor = b.empty("ToolRotor", b.root)
    rotor["spin_axis"] = "local +Z; radians; origin remains handle centre"
    b.root["asset_version"] = "1.1"
    b.root["units"] = "metres"
    b.root["physics_validated"] = False
    b.root["baseline_mass_kg_not_assigned"] = .117
    b.root["grip_diameter_m"] = .032
    b.root["grip_nominal_length_m"] = .070
    b.root["pad_contact_z_m"] = .081

    # Entire original grip length remains inside the original 32 mm envelope.
    b.lathe("ToolGraspSleeve", [(-.035, .0154), (-.0346, .01585),
            (-.034, .016), (.034, .016), (.0346, .01585), (.035, .0154)], rubber)
    # Housing starts beyond the original jaws' cylindrical handle.
    b.shell("ToolMotorHousing", [(.035, .0154, .0154, 2),
            (.036, .0162, .0159, 2.3), (.0385, .019, .017, 3.2),
            (.040, .0197, .0175, 3.5), (.048, .0197, .0175, 3.5),
            (.050, .0187, .0167, 3.2), (.053, .012, .012, 2),
            (.0534, .0115, .0115, 2)], blue)
    b.shell("ToolHousingGasket", [(.0391, .0194, .0173, 3.5),
            (.03955, .01965, .01748, 3.5)], dark)
    # Side inspection covers and fasteners make a manufactured, sealed assembly.
    for sign in (-1, 1):
        b.screw(f"ToolNoseFastener_{sign}_A", (sign*.013, -.01625, .043),
                (0, -1, 0), steel, dark)
        b.screw(f"ToolNoseFastener_{sign}_B", (sign*.013, .01625, .043),
                (0, 1, 0), steel, dark)
    b.lathe("ToolNoseSeal", [(.0528, .0116), (.054, .0116)], dark)
    b.lathe("ToolSpindleBearing", [(.0538, .0107), (.0542, .011),
            (.0567, .011), (.0572, .0105)], steel)
    b.lathe("ToolCollet", [(.0572, .0086), (.0576, .009),
            (.0623, .009), (.0627, .0086)], black, rotor, segments=12)
    b.lathe("ToolSpindle", [(.0625, .0055), (.0658, .0055)], steel, rotor)
    b.lathe("ToolDiscHub", [(.064, .006), (.0644, .009), (.0658, .014),
            (.067, .020), (.0682, .0248), (.0692, .0255),
            (.0705, .0255), (.071, .025)], black, rotor)
    b.lathe("ToolDiscIndexRing", [(.06945, .0255), (.06965, .02565),
            (.0700, .02565), (.07015, .0255)], amber, rotor)
    b.lathe("ToolPadCushion", [(.0707, .0250), (.0712, .0258),
            (.072, .026), (.0765, .026), (.077, .0258)], foam, rotor)
    b.lathe("ToolAbrasivePad", [(.0766, .0258), (.0773, .026),
            (.0792, .026), (.0802, .0256), (.0808, .0248),
            (.081, .0240)], grit, rotor)
    # Native jaws sweep around the rear package at the scene's +20 mm seat.
    # R14 still intersects CAD finger backs; the measured fit needs R10.85.
    b.lathe("ToolRearSeal", [(-.036, .0106), (-.035, .0106)], dark)
    b.lathe("ToolRearCap", [(-.043, .0105), (-.0425, .0107),
            (-.041, .0107), (-.040, .01076), (-.0365, .01076),
            (-.036, .01053)], blue)
    b.lathe("ToolRearIndexRing", [(-.0376, .01076), (-.0375, .01084),
            (-.0369, .01084), (-.0368, .01076)], amber)
    for i in range(3):
        a = 2*math.pi*i/3
        fastener = b.screw(f"ToolRearFastener_{i}",
                (.0092*math.cos(a), .0092*math.sin(a), -.0431), (0, 0, -1), steel, dark)
        for vertex in fastener.data.vertices:
            vertex.co.x *= .5
            vertex.co.y *= .5
    b.lathe("ToolCableGlandNut", [(-.049, .007), (-.0485, .008),
            (-.043, .008), (-.0425, .007)], steel, segments=6)
    b.lathe("ToolCableStrainRelief", [(-.062, .0035), (-.061, .0041),
            (-.051, .0055), (-.049, .0055)], rubber)
    for i in range(5):
        z = -.060 + i*.002
        r = .0044 + i*.00023
        b.lathe(f"ToolGlandRib_{i}", [(z-.0003, r-.0002), (z, r),
                (z+.00045, r), (z+.0007, r-.0002)], rubber)
    b.lathe("ToolConnectorSocket", [(-.064, .0036), (-.062, .0036)], black)
    b.lathe("ToolConnectorMatingFace", [(-.0641, .0028), (-.064, .0028)], dark)
    for i in range(4):
        a = i*math.pi/2
        pin = b.lathe(f"ToolConnectorContact_{i}", [(-.06412, .00035),
                (-.0641, .00035)], steel, segments=24)
        pin.location.x, pin.location.y = .00145*math.cos(a), .00145*math.sin(a)
    bpy.context.view_layer.update()
    return dict(root=b.root, grip_anchor=grip, pad_contact_anchor=pad_anchor,
                rotor=rotor, collection=collection)


def bounds(obj):
    return [
        [min((obj.matrix_world @ v.co)[k] for v in obj.data.vertices) for k in range(3)],
        [max((obj.matrix_world @ v.co)[k] for v in obj.data.vertices) for k in range(3)]]


def audit(asset):
    objects = list(asset["collection"].objects)
    meshes = [o for o in objects if o.type == "MESH"]
    bb = {o.name: bounds(o) for o in meshes}
    lo = [min(b[0][k] for b in bb.values()) for k in range(3)]
    hi = [max(b[1][k] for b in bb.values()) for k in range(3)]
    sleeve = bb["ToolGraspSleeve"]
    pad = bb["ToolAbrasivePad"]
    clear = []
    for o in meshes:
        if o.name == "ToolGraspSleeve":
            continue
        # Every triangle/face wholly outside central +/-34 mm grip band.
        if bb[o.name][0][2] < .034 and bb[o.name][1][2] > -.034:
            clear.append(o.name)
    assert not clear, clear
    rear = [o for o in meshes if o.name.startswith("ToolRear")]
    rear_radius = max(math.hypot(*(o.matrix_world @ v.co)[:2])
                      for o in rear for v in o.data.vertices)
    assert rear_radius <= .01085 + 1e-8
    assert abs(sleeve[1][0]-sleeve[0][0]-.032) < 1e-7
    assert abs(sleeve[1][2]-sleeve[0][2]-.070) < 1e-7
    assert abs(pad[1][0]-pad[0][0]-.052) < 1e-7
    assert abs(pad[1][2]-.081) < 1e-7
    for key, z in (("root", 0), ("grip_anchor", 0), ("pad_contact_anchor", .081)):
        o = asset[key]
        assert (o.location-Vector((0, 0, z))).length < 1e-8
        assert all(abs(v) < 1e-8 for v in o.rotation_euler)
        assert tuple(o.scale) == (1, 1, 1)
    assert all(o.rigid_body is None for o in objects)
    assert not bpy.data.libraries
    assert not [i for i in bpy.data.images if i.source == "FILE" and not i.packed_file]
    return dict(passed=True, units="metres", bounds_m=dict(min=lo, max=hi,
                size=[hi[k]-lo[k] for k in range(3)]), mesh_bounds_m=bb,
                mesh_count=len(meshes), object_count=len(objects),
                vertices=sum(len(o.data.vertices) for o in meshes),
                clear_grip_band_z_m=[-.034, .034], grip_band_intruders=clear,
                rear_package_max_radius_m=rear_radius,
                external_libraries=0, external_images=0, rigid_bodies=0)


def write_manifest(asset, checks, out):
    mats = []
    used = sorted({m for o in asset["collection"].objects if o.type == "MESH"
                   for m in o.data.materials}, key=lambda m: m.name)
    for m in used:
        p = m.node_tree.nodes.get("Principled BSDF")
        mats.append(dict(name=m.name, base_color_linear=list(m.diffuse_color),
                         metallic=p.inputs["Metallic"].default_value,
                         roughness=p.inputs["Roughness"].default_value,
                         textures="embedded procedural nodes; no images"))
    manifest = dict(schema_version=1, asset_version="1.1",
        blend="scripts/viscous-dream1/cinematic/tool/polisher.blend",
        generator="scripts/viscous-dream1/cinematic/tool/build_polisher.py",
        blender_version=bpy.app.version_string, collection=COLLECTION_NAME,
        root=ROOT_NAME, units="metres", forward_axis="+Z from hand toward pad",
        anchors={"ToolGripAnchor": dict(position=[0,0,0], rotation_euler=[0,0,0]),
                 "ToolPadContactAnchor": dict(position=[0,0,.081], rotation_euler=[0,0,0])},
        rotor=dict(object="ToolRotor", parent=ROOT_NAME, axis="local +Z", pivot=[0,0,0]),
        dimensions=dict(grip_diameter_m=.032, grip_nominal_length_m=.070,
                        grip_straight_length_m=.068, pad_diameter_m=.052,
                        rear_package_max_radius_m=checks["rear_package_max_radius_m"],
                        baseline_pad_centre_z_m=.075, baseline_pad_half_thickness_m=.006,
                        abrasive_contact_z_m=.081),
        mounting=dict(target="native right_tool0 via scene-owned anchor transform",
                      bench_grasp_tcp_offset_along_approach_m=.020,
                      offset_baked_into_asset=False, native_rov_mounts_modified=False),
        bounds_m=checks["bounds_m"], materials=mats,
        physics=dict(baseline_mass_kg=.117, visual_asset_mass_kg=None,
                     mass_and_inertia_validated=False, collision_validated=False,
                     warning="Artistic housing, connector and flat abrasive face differ from baseline physics. No rigid-body/collision model supplied."),
        license=dict(spdx="Apache-2.0", copyright="Copyright 2026 MakerMods. All rights reserved.",
                     provenance="Original procedural geometry and shaders; no external textures, CAD or commercial branding."),
        previews=["preview_hero.png", "preview_pad.png", "preview_profile.png"],
        api="create_tool(collection=None) returns dict(root, grip_anchor, pad_contact_anchor, rotor, collection)")
    (out/"asset_manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    (out/"verification.json").write_text(json.dumps(checks, indent=2)+"\n")


def render_previews(out):
    scene = bpy.context.scene
    studio = bpy.data.collections.new("PreviewStudio")
    scene.collection.children.link(studio)
    def point(obj, target):
        obj.rotation_euler = (Vector(target)-obj.location).to_track_quat("-Z", "Y").to_euler()
    for name, loc, power, size, color in [
        ("Key", (.13,-.16,.18), 10, .15, (1,.91,.80)),
        ("Fill", (-.13,-.06,.05), 5, .12, (.66,.83,1)),
        ("Rim", (.05,.12,.14), 14, .10, (.75,.89,1)),
        ("FrontSoft", (0,-.2,-.10), 3, .16, (1,1,1))]:
        data = bpy.data.lights.new(name, "AREA")
        data.energy, data.shape, data.size, data.color = power*.085, "DISK", size, color
        obj = bpy.data.objects.new(name, data)
        studio.objects.link(obj)
        obj.location = loc
        point(obj, (0,0,.01))
    world = bpy.data.worlds.new("ToolStudioWorld")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (.075,.10,.14,1)
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = .4
    scene.world = world
    data = bpy.data.cameras.new("ToolPreviewCamera")
    camera = bpy.data.objects.new("ToolPreviewCamera", data)
    studio.objects.link(camera)
    scene.camera = camera
    data.type = "ORTHO"
    data.clip_start = .001
    data.clip_end = 10
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = 48
    scene.cycles.use_denoising = True
    scene.cycles.seed = 37
    scene.cycles.max_bounces = 5
    scene.render.threads_mode = "FIXED"
    scene.render.threads = 6
    scene.render.resolution_x, scene.render.resolution_y = 1280, 720
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.view_settings.view_transform = "AgX"
    scene.render.film_transparent = False
    for name, location, target, scale, roll in [
        ("hero", (.15,-.26,.13), (0,0,.008), .300, -.33),
        ("pad", (-.15,-.22,.27), (0,0,.023), .25, .25),
        ("profile", (0,-.32,.02), (0,0,.008), .285, -.52)]:
        camera.location = location
        point(camera, target)
        camera.rotation_euler.rotate_axis("Z", roll)
        data.ortho_scale = scale
        scene.render.filepath = str(out / f"preview_{name}.png")
        bpy.ops.render.render(write_still=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1
    asset = create_tool()
    checks = audit(asset)
    write_manifest(asset, checks, HERE)
    bpy.context.view_layer.objects.active = asset["root"]
    asset["root"].select_set(True)
    bpy.ops.file.pack_all()
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(HERE/"polisher.blend"), compress=True)
    # Saving precedes studio creation: appending the asset cannot import a camera/light.
    if args.render:
        render_previews(HERE)
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
              [Path(__file__), HERE/"polisher.blend", HERE/"asset_manifest.json"]}
    (HERE/"sha256.json").write_text(json.dumps(hashes, indent=2)+"\n")
    print(json.dumps(dict(asset="polisher.blend", audit=checks["passed"], bounds=checks["bounds_m"])))


if __name__ == "__main__":
    main()
