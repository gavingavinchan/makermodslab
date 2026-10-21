"""Offline live-run metrics, including measured joint FK and table interference."""
import yam_baseline
import json
import sys
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from dream_ik import decode
from cross_scenes import ViscousAppleEnv
from analyse_supported_runs import analyse


def main(path):
    result = analyse(path)
    rows = json.loads((path / 'trajectory.json').read_text())
    chunks = [json.loads(x) for x in (path / 'chunks.jsonl').read_text().splitlines()]
    task = json.loads((path / 'configuration.json').read_text())['task']
    poses = {(c['call'], s): decode.head_poses(np.array(c['per_key'][f'action.{s}_eef_xyz']),
                                              np.array(c['per_key'][f'action.{s}_eef_rot6']))
             for c in chunks for s in ('left', 'right')}
    env = ViscousAppleEnv(task)
    measured = {s: [] for s in ('left', 'right')}
    try:
        for row in rows:
            for s in measured:
                env.data.qpos[env.qadr[s]] = row['arm_joints'][s]
            mujoco.mj_kinematics(env.model, env.data)
            for s in measured:
                base = env.data.body(s + '_base_link')
                tcp = env.data.site(s + '_tcp')
                r = base.xmat.reshape(3, 3).T
                xyz = r @ (tcp.xpos - base.xpos)
                rot = r @ tcp.xmat.reshape(3, 3)
                target = poses[row['call'], s][row['row']]
                measured[s].append([float(np.linalg.norm(xyz - target[:3, 3])),
                                    float(np.degrees(Rotation.from_matrix(target[:3, :3].T @ rot).magnitude()))])
        result['measured_tracking'] = {s: {'position_m_median_p95': np.percentile(np.array(v)[:, 0], [50, 95]).tolist(),
                                          'rotation_deg_median_p95': np.percentile(np.array(v)[:, 1], [50, 95]).tolist()}
                                       for s, v in measured.items()}
        result['limit_notices'] = [dict(call=c['call'], notices=c['limit_notices']) for c in json.loads((path / 'log.json').read_text()) if c['limit_notices']]
        (path / 'analysis.json').write_text(json.dumps(result, indent=2))
        print(json.dumps({k: v for k, v in result.items() if k not in ('per_call',)}, indent=2))
    finally:
        env.close()

if __name__ == '__main__':
    for arg in sys.argv[1:]:
        main(Path(arg))
