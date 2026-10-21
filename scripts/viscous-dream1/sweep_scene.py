"""Local sweep-only scene options; legacy baseline remains unchanged."""
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import yam_baseline as y
import mujoco
import numpy as np
from cross_scenes import ViscousAppleEnv, viscous_xml, JOINTS, OPEN

HERE = Path(__file__).resolve().parent
PARK = np.r_[json.loads((HERE / 'viscous_arm.json').read_text())['rest'], 1.]


def fingertip_cad():
    env = ViscousAppleEnv()
    try:
        env.data.qpos[:] = 0
        mujoco.mj_forward(env.model, env.data)
        spec = json.loads((HERE / 'viscous_arm.json').read_text())
        flange = env.data.body('left_wrist_roll_link')
        approach = np.array(spec['approach']); approach /= np.linalg.norm(approach)
        tips = []
        for name in ('left_visual_043', 'left_visual_042'):
            geom = env.model.geom(name); mid = geom.dataid[0]
            vertices = env.model.mesh_vert[env.model.mesh_vertadr[mid]:env.model.mesh_vertadr[mid] + env.model.mesh_vertnum[mid]]
            vertices = ((vertices @ env.data.geom(name).xmat.reshape(3, 3).T + env.data.geom(name).xpos) - flange.xpos) @ flange.xmat.reshape(3, 3)
            projection = vertices @ approach
            face = vertices[projection > projection.max() - .0001]
            tips.append((face.min(0) + face.max(0)) / 2)
        spec['tcp'] = np.mean(tips, axis=0).tolist()
        return spec, tips
    finally:
        env.close()


class SweepEnv(ViscousAppleEnv):
    def __init__(self, task, config):
        self.wrist = 'maker'
        self.single = config['arms'] == 'single'
        self.parked = config['arms'] == 'parked'
        self.ignore_right = self.single or self.parked
        self.copy_right = config.get('wrist_r', 'normal') == 'copy'
        if self.copy_right and not self.single:
            raise ValueError('wrist_r=copy requires arms=single')
        self.parked_max_joint_deviation = 0.
        self.parked_max_solver_deviation = 0.
        self.sides = ('left',) if self.single else ('left', 'right')
        root = ET.fromstring(viscous_xml(task))
        if self.single:
            root.find('worldbody').remove(root.find("worldbody/body[@name='right_base_link']"))
            for section in ('contact', 'equality', 'actuator'):
                for node in list(root.find(section)):
                    if any(v.startswith('right_') for v in node.attrib.values()):
                        root.find(section).remove(node)
        if self.parked:
            # Ideal position hold: solver constraints react to contacts, with projection
            # after each 1 ms step to remove finite constraint-solver residuals.
            equality = root.find('equality')
            for node in list(equality):
                if any(v.startswith('right_') for v in node.attrib.values()):
                    equality.remove(node)
            for name, value in [(f'right_joint{i}', q) for i, q in enumerate(PARK[:6], 1)] + [(f'right_finger_{s}', OPEN) for s in (-1, 1)]:
                ET.SubElement(equality, 'joint', name='park_' + name, joint1=name,
                              polycoef=f'{value} 0 0 0 0', solref='.002 1')
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        self.data = mujoco.MjData(self.model)
        self.renderer = self.wrist_renderer = None
        self.qadr, self.vadr, self.aids = {}, {}, {}
        for side in self.sides:
            joints = [self.model.joint(f'{side}_joint{i}') for i in range(1, 7)]
            self.qadr[side] = np.array([j.qposadr[0] for j in joints])
            self.vadr[side] = np.array([j.dofadr[0] for j in joints])
            self.aids[side] = np.array([self.model.actuator(f'{side}_{n}_drive').id for n in JOINTS])
        if self.parked:
            joints = [self.model.joint(f'right_joint{i}') for i in range(1, 7)] + [self.model.joint(f'right_finger_{s}') for s in (-1, 1)]
            self.park_qadr = np.array([j.qposadr[0] for j in joints])
            self.park_vadr = np.array([j.dofadr[0] for j in joints])
            self.park_values = np.r_[PARK[:6], OPEN, OPEN]
        if config['tcp'] == 'fingertip':
            spec = json.loads((HERE / 'viscous_arm_fingertip.json').read_text())
            # Site is a child of tool0; preserve its axes and the camera mounting.
            mujoco.mj_forward(self.model, self.data)
            for side in self.sides:
                flange, tool = self.data.body(side + '_wrist_roll_link'), self.data.body(side + '_tool0')
                point = flange.xpos + flange.xmat.reshape(3, 3) @ np.array(spec['tcp'])
                self.model.site(side + '_tcp').pos[:] = tool.xmat.reshape(3, 3).T @ (point - tool.xpos)
                self.model.site_sameframe[self.model.site(side + '_tcp').id] = 0
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        for side in self.sides:
            for sign in (-1, 1):
                self.data.joint(f'{side}_finger_{sign}').qpos[0] = OPEN
            self.data.actuator(f'{side}_gripper_left_joint_drive').ctrl[0] = OPEN
        if self.parked:
            self.data.qpos[self.park_qadr] = self.park_values
            self.data.ctrl[self.aids['right']] = PARK[:6]
        mujoco.mj_forward(self.model, self.data)
        return self.state()

    def state(self):
        result = {s: np.r_[self.data.qpos[self.qadr[s]], np.clip(sum(self.data.joint(f'{s}_finger_{k}').qpos[0] for k in (-1, 1)) / (2 * OPEN), 0, 1)] for s in self.sides}
        if self.single:
            result['right'] = PARK.copy()
        return result

    def images(self):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=480, width=640)
            self.wrist_renderer = mujoco.Renderer(self.model, height=360, width=640)
        self.renderer.update_scene(self.data, camera='top')
        result = {'top': self.renderer.render()[60:420].copy()}
        for side in self.sides:
            self.wrist_renderer.update_scene(self.data, camera=side)
            result[side] = self.wrist_renderer.render().copy()
        if self.single:
            result['right'] = result['left'].copy() if self.copy_right else np.zeros_like(result['left'])
        return result

    def step(self, left, right):
        if not self.ignore_right:
            return super().step(left, right)
        action = np.asarray(left, float)
        if action.shape != (7,) or not np.isfinite(action).all() or not 0 <= action[6] <= 1:
            raise ValueError('Bad left action')
        limits = np.array([self.model.joint(f'left_joint{i}').range for i in range(1, 7)])
        target = np.r_[np.clip(action[:6], limits[:, 0], limits[:, 1]), action[6] * OPEN]
        aids = self.aids['left']
        finger = self.model.actuator('left_gripper_left_joint_drive').id
        dofs = [self.model.joint(f'left_finger_{s}').dofadr[0] for s in (-1, 1)]
        for _ in range(50):
            self.data.ctrl[aids] = target[:6] + self.data.qfrc_bias[self.vadr['left']] / self.model.actuator_gainprm[aids, 0]
            self.data.ctrl[finger] = target[6] + sum(self.data.qfrc_bias[dofs]) / self.model.actuator_gainprm[finger, 0]
            if self.parked:
                right_aids = self.aids['right']
                self.data.ctrl[right_aids] = PARK[:6] + self.data.qfrc_bias[self.vadr['right']] / self.model.actuator_gainprm[right_aids, 0]
                self.data.actuator('right_gripper_left_joint_drive').ctrl[0] = OPEN
            mujoco.mj_step(self.model, self.data)
            if self.parked:
                deviation = np.max(np.abs(self.data.qpos[self.park_qadr] - self.park_values))
                self.parked_max_solver_deviation = max(self.parked_max_solver_deviation, float(deviation))
                self.data.qpos[self.park_qadr] = self.park_values
                self.data.qvel[self.park_vadr] = 0
                self.parked_max_joint_deviation = max(self.parked_max_joint_deviation, float(np.max(np.abs(self.data.qpos[self.park_qadr] - self.park_values))))
                mujoco.mj_forward(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        return self.state()


def table_region(env):
    # World box x [-.08,.40], y [-.32,.16], z [.785,1.06].
    base = env.data.body('left_base_link')
    corners = np.array([[x, yy, z] for x in (-.08, .40) for yy in (-.32, .16) for z in (.785, 1.06)])
    local = (corners - base.xpos) @ base.xmat.reshape(3, 3)
    return {'left': {'lo': local.min(0).tolist(), 'hi': local.max(0).tolist()}}
