"""Local scene variants for the arm/task 2x2; installed Dream assets stay read-only."""

import copy
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from dream_examples.yam_mujoco import yam_env

HERE = Path(__file__).resolve().parent
SCENE = Path.home() / "Documents/viscous-arm-model/demo/pick_place_scene.xml"
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll")
OPEN = 0.0524125


def cube_xml():
    root = ET.fromstring(yam_env.build_xml())
    world = root.find("worldbody")
    apple, bowl = world.find("body[@name='apple']"), world.find("body[@name='bowl']")
    cube = ET.SubElement(world, "body", name="cube", pos=apple.get("pos"))
    ET.SubElement(cube, "freejoint", name="cube_free")
    ET.SubElement(cube, "geom", name="cube_geom", type="box", size=".01 .01 .01",
                  mass=".01", rgba=".95 .95 .95 1")
    square = ET.SubElement(world, "body", name="place_square", pos=bowl.get("pos"))
    for axis in range(2):
        for sign in (-1, 1):
            pos, size = [0, 0, .0001], [.0225, .0225, .0001]
            pos[axis], size[axis] = sign * .0205, .002
            ET.SubElement(square, "geom", name=f"square_{axis}_{sign}", type="box",
                          pos=yam_env.numbers(pos), size=yam_env.numbers(size),
                          rgba=".025 .025 .025 1", contype="0", conaffinity="0")
    world.remove(apple)
    world.remove(bowl)
    return ET.tostring(root, encoding="unicode")


class CubeYamEnv(yam_env.YamEnv):
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_string(cube_xml())
        self.data = mujoco.MjData(self.model)
        self.renderer = None
        self.reset()


def viscous_xml():
    # Start with the actual Dream scene so all object builders, transforms, lighting,
    # contact defaults, and calibrated top-camera parameters are retained exactly.
    root = ET.fromstring(yam_env.build_xml())
    source = ET.parse(SCENE).getroot()
    root.find("option").set("timestep", ".001")
    world = root.find("worldbody")
    bases = {side: world.find(f"body[@name='{side}_base']") for side in ("left", "right")}
    for side in bases:
        world.remove(bases[side])
    root.find("contact").clear()
    root.find("actuator").clear()
    equality = ET.SubElement(root, "equality")
    defaults = copy.deepcopy(source.find("default"))
    defaults.set("class", "viscous")
    root.find("default").append(defaults)
    for asset in source.find("asset"):
        asset = copy.deepcopy(asset)
        if asset.get("file"):
            asset.set("file", str((SCENE.parent / "../model/meshes" / asset.get("file")).resolve()))
        root.find("asset").append(asset)

    for side in ("left", "right"):
        names = {element.get("name"): f"{side}_{element.get('name')}"
                 for element in source.iter() if element.get("name")}
        names.update({name: f"{side}_joint{i}" for i, name in enumerate(JOINTS, 1)})
        names.update(gripper_left_joint=f"{side}_finger_-1", gripper_right_joint=f"{side}_finger_1",
                     tool_center=f"{side}_tcp")

        def renamed(element):
            element = copy.deepcopy(element)
            for node in element.iter():
                for key in ("name", "joint", "joint1", "joint2", "body1", "body2"):
                    if node.get(key):
                        node.set(key, names[node.get(key)])
            return element

        arm = renamed(source.find("worldbody/body[@name='base_link']"))
        arm.set("pos", bases[side].get("pos"))
        arm.set("quat", bases[side].get("quat"))
        arm.set("childclass", "viscous")
        arm.find(f".//site[@name='{side}_tcp']").set("rgba", "0 0 0 0")
        # Same camera mount as the existing Viscous yam rig; image pipeline is YamEnv's.
        eye, target = np.array([0, .055, -.09]), np.array([0, .025, .08])
        z = eye - target
        z /= np.linalg.norm(z)
        x = np.cross([0, -1, 0], z)
        x /= np.linalg.norm(x)
        ET.SubElement(arm.find(f".//body[@name='{side}_tool0']"), "camera", name=side,
                      pos=yam_env.numbers(eye), xyaxes=yam_env.numbers(np.r_[x, np.cross(z, x)]), fovy="90")
        world.append(arm)
        for section in ("contact", "equality", "actuator"):
            for element in source.find(section):
                root.find(section).append(renamed(element))
    return ET.tostring(root, encoding="unicode")


class ViscousAppleEnv(yam_env.YamEnv):
    """Both Viscous arms servo their own 20 Hz rows, without per-row speed clipping."""

    def __init__(self):
        self.model = mujoco.MjModel.from_xml_string(viscous_xml())
        self.data = mujoco.MjData(self.model)
        self.renderer = None
        self.qadr, self.vadr, self.aids = {}, {}, {}
        for side in ("left", "right"):
            joints = [self.model.joint(f"{side}_joint{i}") for i in range(1, 7)]
            self.qadr[side] = np.array([j.qposadr[0] for j in joints])
            self.vadr[side] = np.array([j.dofadr[0] for j in joints])
            self.aids[side] = np.array([self.model.actuator(f"{side}_{n}_drive").id for n in JOINTS])
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        for side in ("left", "right"):
            for sign in (-1, 1):
                self.data.joint(f"{side}_finger_{sign}").qpos[0] = OPEN
            self.data.actuator(f"{side}_gripper_left_joint_drive").ctrl[0] = OPEN
        mujoco.mj_forward(self.model, self.data)
        return self.state()

    def state(self):
        return {side: np.r_[self.data.qpos[self.qadr[side]], np.clip(sum(
            self.data.joint(f"{side}_finger_{sign}").qpos[0] for sign in (-1, 1)) / (2 * OPEN), 0, 1)]
                for side in ("left", "right")}

    def step(self, left, right):
        commands = {}
        for side, action in (("left", left), ("right", right)):
            action = np.asarray(action, float)
            if action.shape != (7,) or not np.isfinite(action).all() or not 0 <= action[6] <= 1:
                raise ValueError("Expected seven finite commands with normalized gripper.")
            limits = np.array([self.model.joint(f"{side}_joint{i}").range for i in range(1, 7)])
            commands[side] = np.r_[np.clip(action[:6], limits[:, 0], limits[:, 1]), action[6] * OPEN]
        for _ in range(50):
            for side, target in commands.items():
                aids = self.aids[side]
                self.data.ctrl[aids] = target[:6] + self.data.qfrc_bias[self.vadr[side]] / self.model.actuator_gainprm[aids, 0]
                finger = self.model.actuator(f"{side}_gripper_left_joint_drive").id
                dofs = [self.model.joint(f"{side}_finger_{s}").dofadr[0] for s in (-1, 1)]
                self.data.ctrl[finger] = target[6] + sum(self.data.qfrc_bias[dofs]) / self.model.actuator_gainprm[finger, 0]
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        return self.state()


def success(env, task):
    """Conservative geometry + rest check; frame inspection remains the reported judgement."""
    name = "cube" if task == "cube" else task
    body = env.model.body(name)
    p = env.data.body(name).xpos
    joint = env.model.joint(f"{name}_free")
    speed = np.linalg.norm(env.data.qvel[joint.dofadr[0]:joint.dofadr[0] + 6])
    if task == "cube":
        corners = np.array([[x, y, z] for x in (-.01, .01) for y in (-.01, .01) for z in (-.01, .01)])
        corners = corners @ env.data.xmat[body.id].reshape(3, 3).T + p
        centered = corners - env.data.body("place_square").xpos
        return bool(np.all(np.abs(centered[:, :2]) <= .0185) and abs(p[2] - .77) < .002 and speed < .05)
    return bool(np.linalg.norm(p[:2] - [0, -.04]) < .04 and .78 < p[2] < .83 and speed < .05)


def preview_comparison(env, out):
    from PIL import Image, ImageDraw

    baseline = Image.open(HERE / "runs/yam_vision_live1/frame_00.png").crop((640, 0, 1280, 480))
    current = Image.new("RGB", (640, 480))
    current.paste(Image.fromarray(env.images()["top"]), (0, 60))
    canvas = Image.new("RGB", (1280, 508), "white")
    canvas.paste(baseline, (0, 28))
    canvas.paste(current, (640, 28))
    draw = ImageDraw.Draw(canvas)
    draw.text((12, 8), "YAM vision live1 frame_00: top camera", fill="black")
    draw.text((652, 8), "Viscous / Dream apple scene: top camera", fill="black")
    canvas.save(out / "compare_top_yam_vs_viscous.png")

