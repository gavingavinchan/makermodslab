"""Dream's built-in Maker/Metal kinematics with local copies of vendor CAD.

Joint coordinates are Dream's own coordinates. Maker wrist yaw is deliberately
the negative of the vendor coordinate; no joint zero offsets are applied.
"""
import copy
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from dream_ik.specs import MAKERS_ARM, METAL_ARM
from dream_examples.yam_mujoco import yam_env
from cross_scenes import cube_xml

HERE = Path(__file__).resolve().parent
SPECS = {"makers": MAKERS_ARM, "metal": METAL_ARM}


def numbers(x):
    return " ".join(f"{v:.16g}" for v in np.ravel(x))


def pose(element):
    origin = element.find("origin")
    if origin is None:
        return {}
    r = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_quat()
    return {"pos": origin.get("xyz", "0 0 0"), "quat": numbers(r[[3, 0, 1, 2]])}


def camera_mount():
    """The calibrated YAM wrist optical frame expressed relative to its TCP."""
    ref = yam_env.YamEnv()
    tcp = ref.data.site("left_tcp")
    cam = ref.data.cam("left")
    r = tcp.xmat.reshape(3, 3)
    result = (r.T @ (cam.xpos - tcp.xpos), r.T @ cam.xmat.reshape(3, 3))
    ref.close()
    return result


def build_xml(arm, task="apple"):
    spec = SPECS[arm]
    folder = HERE / "assets/supported_arms" / ("maker-urdf" if arm == "makers" else "metal-urdf")
    vendor = ET.parse(folder / ("robot.urdf" if arm == "makers" else "metal_with_gripper.urdf")).getroot()
    chain = ET.parse(spec.urdf).getroot()
    # CAD origins agree; use the vendor origins, Dream axes/soft limits, and
    # Dream's additional fixed root. This makes the mapping explicit.
    for j in vendor.findall("joint"):
        dj = chain.find(f"joint[@name='{j.get('name')}']")
        if dj is not None:
            for tag in ("origin", "parent", "child"):
                if j.find(tag) is not None and dj.find(tag) is not None:
                    assert j.find(tag).attrib == dj.find(tag).attrib, (arm, j.get("name"), tag)
            for tag in ("axis", "limit"):
                if dj.find(tag) is not None:
                    j.remove(j.find(tag))
                    j.append(copy.deepcopy(dj.find(tag)))
    if arm == "makers":
        vendor.append(copy.deepcopy(chain.find("joint[@name='ik_base_joint']")))
        vendor.append(ET.Element("link", name="ik_base"))
    root = ET.fromstring(cube_xml(task) if task in ("cube", "box") else yam_env.build_xml())
    root.find("option").set("timestep", ".001")
    world, asset, actuators = root.find("worldbody"), root.find("asset"), root.find("actuator")
    actuators.clear()
    root.find("contact").clear()
    links = {l.get("name"): l for l in vendor.findall("link")}
    children = {}
    for j in vendor.findall("joint"):
        children.setdefault(j.find("parent").get("link"), []).append(j)
    root_link = next(n for n in links if not any(j.find("child").get("link") == n for j in vendor.findall("joint")))
    # Start from YAM's TCP-relative calibration, then move to the palm side
    # perpendicular to jaw travel: Maker's longer CAD jaws obscure YAM's mount.
    mount_p, mount_r = wrist_mount()
    tool = spec.tools["tcp"]
    tool_r = np.asarray(tool.rotation)
    cp = np.asarray(tool.offset) + tool_r @ mount_p
    cq = Rotation.from_matrix(tool_r @ mount_r).as_quat()[[3, 0, 1, 2]]
    meshes = {}
    for side in ("left", "right"):
        old = world.find(f"body[@name='{side}_base']")
        base = ET.SubElement(world, "body", **old.attrib)
        world.remove(old)

        def add_link(parent, name, joint=None):
            body = ET.SubElement(parent, "body", name=f"{side}_{name}", **(pose(joint) if joint is not None else {}))
            link = links[name]
            inertial = link.find("inertial")
            if inertial is not None:
                inertia = inertial.find("inertia")
                # All supplied inertia origins have identity rotation.
                assert np.allclose(np.fromstring(inertial.find("origin").get("rpy", "0 0 0"), sep=" "), 0)
                ET.SubElement(body, "inertial", pos=inertial.find("origin").get("xyz"),
                              mass=inertial.find("mass").get("value"),
                              fullinertia=" ".join(inertia.get(k) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")))
            if joint is not None and joint.get("type") != "fixed":
                jname = joint.get("name")
                index = spec.joints.index(jname) + 1 if jname in spec.joints else None
                mjname = f"{side}_joint{index}" if index else f"{side}_{jname}"
                limit = joint.find("limit")
                ET.SubElement(body, "joint", name=mjname, type="hinge" if index else "slide",
                              axis=joint.find("axis").get("xyz"), limited="true",
                              range=f"{limit.get('lower')} {limit.get('upper')}",
                              damping="4" if index else "2", armature=".02" if index else ".001")
                ET.SubElement(actuators, "position", name=mjname, joint=mjname,
                              kp="800" if index else "500", kv="50" if index else "5",
                              forcelimited="true", forcerange="-100 100" if index else "-20 20")
            for kind in ("visual", "collision"):
                for i, geom in enumerate(link.findall(kind)):
                    mesh = geom.find("geometry/mesh")
                    filename = mesh.get("filename")
                    path = folder / ("meshes/" + filename.split("/")[-1])
                    key = (str(path), mesh.get("scale", "1 1 1"))
                    if key not in meshes:
                        meshes[key] = f"{arm}_mesh{len(meshes)}"
                        ET.SubElement(asset, "mesh", name=meshes[key], file=str(path), scale=key[1])
                    color = geom.find("material/color")
                    rgba = color.get("rgba") if color is not None else ".75 .77 .79 1"
                    ET.SubElement(body, "geom", name=f"{side}_{name}_{kind}{i}", type="mesh", mesh=meshes[key],
                                  **pose(geom), rgba=rgba if kind == "visual" else "0 0 0 0",
                                  contype="0" if kind == "visual" else "1",
                                  conaffinity="0" if kind == "visual" else "1", group="2" if kind == "visual" else "3",
                                  mass="0", condim="4")
            if name == spec.flange:
                tq = Rotation.from_matrix(tool_r).as_quat()[[3, 0, 1, 2]]
                ET.SubElement(body, "site", name=f"{side}_tcp", pos=numbers(tool.offset), quat=numbers(tq), size=".003", rgba="0 0 0 0")
                ET.SubElement(body, "camera", name=side, pos=numbers(cp), quat=numbers(cq), fovy="90")
            for child in children.get(name, []):
                add_link(body, child.find("child").get("link"), child)

        add_link(base, root_link)
        # CAD bearing surfaces overlap; explicit adjacent exclusions are needed
        # especially for a dynamic first link against a welded static base.
        for j in vendor.findall("joint"):
            ET.SubElement(root.find("contact"), "exclude",
                          body1=f"{side}_{j.find('parent').get('link')}",
                          body2=f"{side}_{j.find('child').get('link')}")
        jaws = ("gripper_left", "gripper_right") if arm == "makers" else ("link7", "link8")
        ET.SubElement(root.find("contact"), "exclude", body1=f"{side}_{jaws[0]}", body2=f"{side}_{jaws[1]}")
    return ET.tostring(root, encoding="unicode")


def wrist_mount():
    eye, target = np.array([0., -.07, -.12]), np.array([0., 0., .04])
    z = eye - target
    z /= np.linalg.norm(z)
    x = np.cross([0., -1., 0.], z)
    x /= np.linalg.norm(x)
    return eye, np.column_stack([x, np.cross(z, x), z])


class SupportedArmEnv(yam_env.YamEnv):
    def __init__(self, arm="makers", task="apple"):
        self.arm, self.spec = arm, SPECS[arm]
        self.model = mujoco.MjModel.from_xml_string(build_xml(arm, task))
        self.data = mujoco.MjData(self.model)
        self.renderer = None
        self.qadr, self.vadr, self.aids = {}, {}, {}
        self.fingers = ("gripper_left_joint", "gripper_right_joint") if arm == "makers" else ("joint7", "joint8")
        self.open = np.array([.0524125, .0524125]) if arm == "makers" else np.array([-.05, .05])
        for side in ("left", "right"):
            names = [f"{side}_joint{i}" for i in range(1, 7)] + [f"{side}_{f}" for f in self.fingers]
            self.qadr[side] = np.array([self.model.joint(n).qposadr[0] for n in names])
            self.vadr[side] = np.array([self.model.joint(n).dofadr[0] for n in names])
            self.aids[side] = np.array([self.model.actuator(n).id for n in names])
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        for side in ("left", "right"):
            target = np.r_[self.spec.rest_pose, self.open]
            self.data.qpos[self.qadr[side]] = target
            self.data.ctrl[self.aids[side]] = target
        mujoco.mj_forward(self.model, self.data)
        return self.state()

    def state(self):
        return {s: np.r_[self.data.qpos[self.qadr[s][:6]],
                         np.clip(np.mean(self.data.qpos[self.qadr[s][6:]] / self.open), 0, 1)]
                for s in ("left", "right")}

    def step(self, left, right):
        targets = {}
        for side, action in (("left", left), ("right", right)):
            action = np.asarray(action, float)
            if action.shape != (7,) or not np.isfinite(action).all() or not 0 <= action[6] <= 1:
                raise ValueError("Expected six finite arm joints and grip 0..1")
            limits = np.array([self.model.joint(f"{side}_joint{i}").range for i in range(1, 7)])
            targets[side] = np.r_[np.clip(action[:6], limits[:, 0], limits[:, 1]), action[6] * self.open]
        for _ in range(50):
            for side in targets:
                ids = self.aids[side]
                self.data.ctrl[ids] = targets[side] + self.data.qfrc_bias[self.vadr[side]] / self.model.actuator_gainprm[ids, 0]
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        return self.state()
