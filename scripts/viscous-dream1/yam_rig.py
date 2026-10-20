"""Local YAM station composition. Installed Dream assets are read-only."""

import copy
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from dream_examples.yam_mujoco import yam_env


def build_model(scene, wrist_pos, wrist_target, wrist_fovy, look_at):
    """Keep Viscous dynamics and base-local joint frames; transplant the station scenery."""
    root = ET.parse(scene).getroot()
    root.find("compiler").set("meshdir", str((scene.parent / "../model/meshes").resolve()))
    world = root.find("worldbody")
    # Read Dream's actual scene builder, including calibrated_camera, rather than approximating its fovy.
    station = ET.fromstring(yam_env.build_xml())
    root.remove(root.find("visual"))
    visual = copy.deepcopy(station.find("visual"))
    visual.find("global").set("offwidth", "1280")
    visual.find("global").set("offheight", "720")
    root.append(visual)
    for element in station.find("asset"):
        if element.get("type") == "skybox" or element.get("name") == "oak":
            root.find("asset").append(copy.deepcopy(element))
    for element in list(world):
        if element.tag in ("light", "geom") or element.get("name") == "cup":
            world.remove(element)
    for element in station.find("worldbody"):
        if element.tag == "light" or element.tag == "geom" or element.get("name") in (
            "top", "center_camera_stand"
        ):
            element = copy.deepcopy(element)
            if element.get("name") == "top":
                element.set("name", "overhead")
            if element.get("name") == "table":
                element.set("name", "floor")  # contact support name used by the existing success check
            world.append(element)
    left = world.find("body[@name='base_link']")
    left.find(".//site[@name='tool_center']").set("rgba", "0 0 0 0")
    ET.SubElement(
        left.find(".//body[@name='tool0']"), "camera", name="wrist",
        pos=yam_env.numbers(wrist_pos),
        xyaxes=yam_env.numbers(look_at(wrist_pos, wrist_target, up=(0, -1, 0))),
        fovy=str(wrist_fovy),
    )
    right = copy.deepcopy(left)
    # Mesh references remain shared; all body/joint/geom/site/camera names are distinct.
    for element in right.iter():
        if "name" in element.attrib:
            element.set("name", "right_" + element.get("name"))
    world.append(right)
    for exclude in list(root.find("contact")):
        duplicate = copy.deepcopy(exclude)
        for key in ("body1", "body2"):
            duplicate.set(key, "right_" + duplicate.get(key))
        root.find("contact").append(duplicate)
    # The right arm has real qpos entries, but is parked kinematically by Sim at each physics step.
    for side, body in (("left", left), ("right", right)):
        calibration = yam_env.CALIBRATION["arms"][side]
        body.set("pos", yam_env.numbers(
            yam_env.WORLD_ROTATION @ calibration["position"] + yam_env.WORLD_OFFSET
        ))
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, np.array(calibration["quaternion"]))
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, (yam_env.WORLD_ROTATION @ rotation.reshape(3, 3)).ravel())
        body.set("quat", yam_env.numbers(quat))
    # Base-local task offsets: forward +x, inward -y. Both are inside 0.30 m horizontal reach.
    base = np.array(yam_env.CALIBRATION["arms"]["left"]["position"])
    cube = world.find("body[@name='cube']")
    cube.set("pos", yam_env.numbers(
        yam_env.WORLD_ROTATION @ (base + [0.20, -0.07, 0.0101]) + yam_env.WORLD_OFFSET
    ))
    cube.find("geom").set("rgba", ".95 .95 .95 1")
    square = ET.SubElement(world, "body", name="place_square", pos=yam_env.numbers(
        yam_env.WORLD_ROTATION @ (base + [0.23, -0.15, 0]) + yam_env.WORLD_OFFSET
    ))
    for axis in range(2):
        for sign in (-1, 1):
            pos, size = [0, 0, .0001], [.0225, .0225, .0001]
            pos[axis], size[axis] = sign * (.0225 - .002), .002
            ET.SubElement(square, "geom", name=f"square_{axis}_{sign}", type="box",
                          pos=yam_env.numbers(pos), size=yam_env.numbers(size),
                          rgba=".025 .025 .025 1", contype="0", conaffinity="0")
    eye, target = (.95, 1.15, 1.65), (0, .02, .92)
    ET.SubElement(world, "camera", name="third_person", pos=yam_env.numbers(eye),
                  xyaxes=yam_env.numbers(look_at(eye, target)), fovy="50")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def comparison(sim, out):
    """Instantiate the unmodified reference environment; render the full 640x480 optical view."""
    from PIL import Image, ImageDraw

    env = yam_env.YamEnv()
    try:
        with mujoco.Renderer(env.model, height=480, width=640) as renderer:
            renderer.update_scene(env.data, camera="top")
            reference = renderer.render().copy()
        Image.fromarray(reference).save(out / "reference_yam_overhead.png")
        canvas = Image.new("RGB", (1280, 508), "white")
        canvas.paste(Image.fromarray(reference), (0, 28))
        canvas.paste(Image.fromarray(sim.render("overhead")), (640, 28))
        draw = ImageDraw.Draw(canvas)
        draw.text((12, 8), "Dream YamEnv (reset, open grippers)", fill="black")
        draw.text((652, 8), "Viscous YAM rig (requested start pose)", fill="black")
        canvas.save(out / "compare_overhead_yam_vs_viscous.png")
    finally:
        env.close()
