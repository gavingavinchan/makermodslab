"""Hold test: the heaviest clear-of-table left-arm pose in a box run's chunks.jsonl, held 2 s
under B0 (5 Nm), t14 (14 Nm) and buoy (5 Nm, 90% gravcomp). No inference.
Run from viscous_sim/: python box_torque_hold.py runs/sweep/B0/box/ep_00/chunks.jsonl
"""
import json, sys, numpy as np, mujoco
sys.path.insert(0, '.')
from sweep import parse_config
from sweep_scene import SweepEnv
rows = []
for line in open(sys.argv[1]):
    rows += json.loads(line)['execution']['left']
env = SweepEnv('box', parse_config('B0'))
best, bq = -1, None
for q in rows[::5]:
    env.data.qpos[env.qadr['left']] = q[:6]; mujoco.mj_forward(env.model, env.data)
    g = abs(env.data.qfrc_bias[env.vadr['left'][1]])
    clear = env.data.site('left_tcp').xpos[2] > .76 + .06 and not any(
        'left_' in (env.model.geom(c.geom1).name + env.model.geom(c.geom2).name) for c in env.data.contact[:env.data.ncon])
    if clear and g > best: best, bq = g, np.array(q)
env.close()
env.data.qpos[env.qadr['left']] = bq[:6]; mujoco.mj_forward(env.model, env.data); print('tcp', env.data.site('left_tcp').xpos.round(3), 'ncon', env.data.ncon)
print(f'pose: shoulder_lift gravity {best:.2f} Nm, q={np.round(bq[:6],3).tolist()}'); env.close()
for name in ['B0', 't14', 'buoy']:
    env = SweepEnv('box', parse_config(name))
    env.data.qpos[env.qadr['left']] = bq[:6]; env.data.qpos[env.qadr['right']] = env.state()['right'][:6]
    mujoco.mj_forward(env.model, env.data)
    tcp0 = env.data.site('left_tcp').xpos.copy()
    for _ in range(40): env.step(np.r_[bq[:6], 1.], env.state()['right'] if not env.parked else None)  # 2 s
    sag = env.data.site('left_tcp').xpos - tcp0
    t = env.torque_summary()
    print(f"{name:5s} limit {t['limit_nm']:>4} buoy {t['buoyancy']:.1f}: tcp moved {np.linalg.norm(sag)*1000:5.1f} mm (dz {sag[2]*1000:+.1f}), shoulder at limit {t['shoulder_lift_at_limit_s']:.2f}/{t['sim_s']:.2f} s, peak {t['peak_nm']['shoulder_lift']:.2f} Nm")
    env.close()
