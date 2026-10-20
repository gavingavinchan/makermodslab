"""Offline checks of station calibration, parked state and geometric task reachability."""
import base64
import io
import json
from pathlib import Path
from types import SimpleNamespace

import viscous_dream_sim as v
import mujoco
import numpy as np
from PIL import Image
from scipy.optimize import least_squares
from yam_rig import yam_env


def main():
    args = SimpleNamespace(rig='yam', start='neutral', wrist_pos=(0, .055, -.09),
                           wrist_target=(0, .025, .08), wrist_fovy=90)
    sim = v.Sim(args)
    reference = yam_env.YamEnv()
    out = {}
    try:
        a, b = sim.m.camera('overhead').id, reference.model.camera('top').id
        for field in ('pos', 'quat', 'intrinsic', 'sensorsize', 'resolution', 'fovy'):
            actual = getattr(sim.m, 'cam_' + field)[a]
            expected = getattr(reference.model, 'cam_' + field)[b]
            np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=0)
        out['camera_matches_YamEnv'] = True
        out['bases'] = {}
        for side, name in [('left', 'base_link'), ('right', 'right_base_link')]:
            body = sim.m.body(name)
            expected = yam_env.WORLD_ROTATION @ yam_env.CALIBRATION['arms'][side]['position'] + yam_env.WORLD_OFFSET
            np.testing.assert_allclose(body.pos, expected, atol=1e-12)
            np.testing.assert_allclose(sim.d.xmat[body.id].reshape(3, 3), yam_env.WORLD_ROTATION, atol=1e-12)
            out['bases'][side] = {'pos': body.pos.tolist(), 'quat_wxyz': body.quat.tolist()}
        right_initial = sim.d.xpos[sim.m.body('right_tool0').id].copy()
        for _ in range(20):
            sim.run_row(sim.start, sim.start_grip)
        np.testing.assert_array_equal(sim.right_state()[0], np.zeros(6))
        assert sim.right_state()[1] == 0
        np.testing.assert_allclose(sim.d.xpos[sim.m.body('right_tool0').id], right_initial, atol=1e-12)
        out['parked_right_exact_after_1s'] = True
        out['policy_images'] = {}
        for role in ('overhead', 'wrist_l', 'wrist_r'):
            im = Image.open(io.BytesIO(base64.b64decode(sim.policy_image(role))))
            assert im.size == (320, 176)
            assert np.asarray(im).std() > 1
            out['policy_images'][role] = list(im.size)
        # Independent bounded FK solve: tool position + downward approach, unconstrained jaw yaw.
        tool = sim.m.body('tool0').id
        results = {}
        for name in ('cube', 'place_square'):
            target = sim.d.xpos[sim.m.body(name).id].copy()
            if name == 'place_square':
                target[2] += .01
            def residual(q):
                sim.d.qpos[sim.qadr] = q
                mujoco.mj_forward(sim.m, sim.d)
                return np.r_[sim.d.xpos[tool] - target,
                             .1 * (sim.d.xmat[tool].reshape(3,3)[:,2] - [0,0,-1])]
            best = None
            for yaw in (0, -.4, -.8):
                seed = v.REST.copy()
                seed[0] = yaw
                result = least_squares(residual, seed, bounds=(sim.lo, sim.hi),
                                       max_nfev=150, gtol=1e-11, ftol=1e-11, xtol=1e-11)
                if best is None or np.linalg.norm(result.fun) < np.linalg.norm(best.fun):
                    best = result
            error = residual(best.x)
            assert np.linalg.norm(error[:3]) < .001, (name, error)
            assert np.linalg.norm(error[3:]) < .001, (name, error)
            results[name] = {'world_target_m': target.tolist(), 'joints_rad': best.x.tolist(),
                             'position_error_m': float(np.linalg.norm(error[:3])),
                             'approach_error': float(np.linalg.norm(error[3:]) / .1)}
        out['geometric_reach_downward_tool'] = results
        out['reach_scope'] = 'Bounded static FK solution; not a collision-free trajectory or grasp success test.'
        sim.d.qpos[sim.qadr] = sim.start
        sim.d.qvel[:] = 0
        cube_qadr = sim.m.joint('cube_free').qposadr[0]
        sim.d.qpos[cube_qadr:cube_qadr+3] = sim.d.xpos[sim.square] + [0, 0, .0101]
        sim.d.qpos[cube_qadr+3:cube_qadr+7] = [1, 0, 0, 0]
        mujoco.mj_forward(sim.m, sim.d)
        for _ in range(10):
            sim.run_row(sim.start, sim.start_grip)
        assert sim.cube_in_square()
        sim.d.qpos[cube_qadr] += .06
        mujoco.mj_forward(sim.m, sim.d)
        assert not sim.cube_in_square()
        out['raised_table_success_check_inside_and_outside'] = True
        Path('runs/yamrig_preview/validation.json').write_text(json.dumps(out, indent=2) + '\n')
        print(json.dumps(out, indent=2))
    finally:
        reference.close()
        sim.big.close()
        sim.renderer.close()


if __name__ == '__main__':
    main()
