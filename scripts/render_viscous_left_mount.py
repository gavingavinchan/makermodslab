"""Blender-only reference drawings; use the existing repo-local native CAD rig.

blender -b --python-exit-code 1 --python scripts/render_viscous_left_mount.py
No hardware, no saved .blend changes. Arrows are world-Z rotations, not CAD q.
"""

import math
from pathlib import Path

import bpy
from mathutils import Vector

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "configs/viscous/viscous_01/evidence/left-mount-2026-11-05"
bpy.ops.wm.open_mainfile(filepath=str(REPO / "scripts/viscous-dream1/cinematic/assets/native_rov.blend"))
scene = bpy.context.scene
scene.frame_set(1)
for obj in scene.objects:
    if "q" in obj:
        obj["q"] = 0.0
    if "rpm" in obj:
        obj["rpm"] = 0.0
bpy.context.view_layer.update()


def material(name, color):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1)
    mat.use_nodes = True
    shader = mat.node_tree.nodes.new("ShaderNodeEmission")
    shader.inputs["Color"].default_value = (*color, 1)
    mat.node_tree.links.new(shader.outputs[0], mat.node_tree.nodes.get("Material Output").inputs["Surface"])
    return mat


orange = material("Unit 1 annotation", (0.95, 0.23, 0.025))
teal = material("Waterproof annotation", (0.01, 0.40, 0.42))
ink = material("Annotation ink", (0.035, 0.045, 0.06))


def line(name, points, mat, thickness=0.003):
    curve = bpy.data.curves.new(name, "CURVE")
    curve.dimensions = "3D"
    curve.bevel_depth = thickness
    curve.bevel_resolution = 2
    poly = curve.splines.new("POLY")
    poly.points.add(len(points) - 1)
    for target, point in zip(poly.points, points, strict=True):
        target.co = (*point, 1)
    obj = bpy.data.objects.new(name, curve)
    scene.collection.objects.link(obj)
    obj.data.materials.append(mat)


for side, start, mat in (("left", 90, orange), ("right", -94.48, teal)):
    center = Vector((0.165721504063, 0.246525 if side == "left" else -0.246525, 0.07))
    angles = [math.radians(start * (1 - i / 60)) for i in range(61)]
    points = [center + Vector((0.09 * math.cos(a), 0.09 * math.sin(a), 0)) for a in angles]
    line(side + " J1 world rotation", points, mat)
    tangent = (points[-1] - points[-2]).normalized()
    normal = Vector((-tangent.y, tangent.x, 0))
    line(
        side + " arrowhead",
        [
            points[-1] - tangent * 0.022 + normal * 0.012,
            points[-1],
            points[-1] - tangent * 0.022 - normal * 0.012,
        ],
        mat,
    )
    # Inferred old table heading: dotted, never presented as measured CAD indexing.
    direction = Vector((math.cos(angles[0]), math.sin(angles[0]), 0))
    for i in range(4):
        line(
            side + " inferred table heading",
            [center + direction * (i * 0.025), center + direction * (i * 0.025 + 0.013)],
            mat,
            0.0015,
        )

scene.render.engine = "CYCLES"
scene.cycles.device = "CPU"
scene.cycles.samples = 20
scene.cycles.use_denoising = True
scene.render.resolution_x = 1200
scene.render.resolution_y = 900
scene.render.resolution_percentage = 100
scene.render.film_transparent = False
scene.world.use_nodes = True
background = scene.world.node_tree.nodes.get("Background")
background.inputs["Color"].default_value = (0.92, 0.94, 0.96, 1)
background.inputs["Strength"].default_value = 0.8
scene.view_settings.view_transform = "Standard"
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_mode = "RGB"
scene.render.image_settings.compression = 75

camera_data = bpy.data.cameras.new("Mount review camera")
camera = bpy.data.objects.new("Mount review camera", camera_data)
scene.collection.objects.link(camera)
scene.camera = camera
camera_data.type = "ORTHO"
camera_data.ortho_scale = 1.52


def caption(body, xy, size, mat):
    curve = bpy.data.curves.new("Caption", "FONT")
    curve.body = body
    curve.size = size
    obj = bpy.data.objects.new("Caption", curve)
    scene.collection.objects.link(obj)
    obj.parent = camera
    obj.location = (xy[0], xy[1], -1)
    obj.data.materials.append(mat)


caption("LEFT MOUNT / UNIT 1 - OFFLINE CANDIDATE", (-0.70, 0.51), 0.032, ink)
caption("ROV +X forward, +Y left, +Z up | CAD folded articulation shown", (-0.70, 0.465), 0.022, ink)
caption(
    "LEFT: unit 1\n90 deg CW / expected motor +90 deg\nConditional on physical base indexing",
    (-0.70, -0.425),
    0.023,
    orange,
)
caption(
    "RIGHT: waterproof (CAD mirror proxy)\n94.48 deg CCW / recorded motor -94.48 deg\nArrows show motion from old table neutral",
    (0.02, -0.425),
    0.021,
    teal,
)
caption(
    "Dashed rays = inferred table headings. CAD does not verify installed motor zeros or hull clearance.",
    (-0.70, -0.545),
    0.018,
    ink,
)

OUT.mkdir(parents=True, exist_ok=True)
for name, position in (("top-down", (0.16, 0, 3)), ("three-quarter", (1.35, -1.4, 1.2))):
    camera.location = position
    target = Vector((0.16, 0, -0.14 if name == "three-quarter" else -0.03))
    camera.rotation_euler = (target - camera.location).to_track_quat("-Z", "Y").to_euler()
    if name == "top-down":
        camera.rotation_euler = (0, 0, -math.pi / 2)
    scene.render.filepath = str(OUT / f"{name}.png")
    bpy.ops.render.render(write_still=True)
