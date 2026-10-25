# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Wide scripted servo scrub; imports the unchanged corrected physics model.

Only initialization sets poses. IK uses separate scratch MjData, never the live
physics data. Every 1 ms contact sample and actual pad position is retained.
Rendering is a separate, explicitly labelled replay of saved physical states.
"""
import argparse
import copy
import hashlib
import json

import mujoco
import numpy as np
from scipy.signal import find_peaks
from scipy.spatial import ConvexHull
from scipy.spatial.distance import pdist

from flange_start import local_ik
from flange_ik_check import orientation
from propeller_corrected_scene import CorrectedEnv, CENTER, HERE, PAD_Z, surface, LEFT_T, LEFT_OPEN, build_xml, section, ROOT_RADIUS, RADIUS
from propeller_corrected_demo import Audit
from propeller_edge_geometry import clearance_summary

OUT = HERE / 'runs/propeller_edge'


from propeller_wide_demo import digest, model_fingerprint, scratch_ik, setup, motion_metrics


class SubstepAudit(Audit):
    def __init__(self, e):
        super().__init__(e)
        self.samples = []
        self.stage = 0
        self.previous_geoms = None
        self.left_geoms = [i for i in range(e.model.ngeom) if self.bodies[i].startswith('left_') and e.model.geom_contype[i]]
        self.tool_geoms = [i for i in range(e.model.ngeom) if (self.bodies[i]=='cardinal_tool' or self.bodies[i].startswith(('right_gripper_left','right_gripper_right'))) and e.model.geom_contype[i]]
    def tick(self):
        before = self.counts.copy()
        super().tick()
        e = self.e
        assert not np.any(e.data.xfrc_applied) and not np.any(e.data.qfrc_applied)
        clearance = min(mujoco.mj_geomDistance(e.model,e.data,a,b,.2,None) for a in self.left_geoms for b in self.tool_geoms)
        geoms = self.left_geoms+self.tool_geoms
        positions=e.data.geom_xpos[geoms].copy(); rotations=e.data.geom_xmat[geoms].copy().reshape(-1,3,3)
        motion_bound=0.
        if self.previous_geoms is not None:
            pos0,rot0=self.previous_geoms
            angles=np.arccos(np.clip((np.einsum('nij,nij->n',rotations,rot0)-1)/2,-1,1))
            bounds=np.linalg.norm(positions-pos0,axis=1)+e.model.geom_rbound[geoms]*angles
            motion_bound=float(bounds[:len(self.left_geoms)].max()+bounds[len(self.left_geoms):].max())
        self.previous_geoms=positions,rotations
        self.samples.append([e.data.time, self.stage, *e.data.geom('cardinal_pad').xpos,
                             *[self.counts[k]-before[k] for k in self.counts],
                             *e.data.geom('cardinal_pad').xmat, clearance, motion_bound])


def target(args, seconds):
    if not args.lanes:
        return (args.centre+args.amplitude*np.sin(2*np.pi*seconds/args.period), args.u, 0)
    lanes = [float(v) for v in args.lanes.split(',')]
    # Start in the middle of the first lane as in the proven wide pregrasp.
    # First pass travels outward then inward; later lanes alternate full strokes.
    half=args.period/2
    first=half*1.5
    if seconds<=first:
        return args.centre+args.amplitude*np.sin(2*np.pi*seconds/args.period),lanes[0],0
    elapsed=seconds-first
    previous=-1.
    for index in range(1,len(lanes)):
        if elapsed<=args.shift_s:
            blend=(1-np.cos(np.pi*elapsed/args.shift_s))/2
            return args.centre+previous*args.amplitude,lanes[index-1]+blend*(lanes[index]-lanes[index-1]),index
        elapsed-=args.shift_s
        if elapsed<=half:
            return args.centre+previous*args.amplitude*np.cos(np.pi*elapsed/half),lanes[index],index
        elapsed-=half
        previous=-previous
    return args.centre+previous*args.amplitude,lanes[-1],len(lanes)-1



def run(args):
    out = OUT / args.name if args.name else OUT
    out.mkdir(parents=True, exist_ok=True)
    assert not (out/'check.json').exists(), 'Preserve prior runs; choose a new --name'
    assert 0 < args.centre-args.amplitude < args.centre+args.amplitude < .99
    assert abs(args.u)+abs(args.u_amplitude) < .6
    assert args.seconds > 0 and args.period > 0
    e = CorrectedEnv()
    before = model_fingerprint(e.model)
    xml = build_xml()
    reference = mujoco.MjModel.from_xml_string(xml)
    assert before == model_fingerprint(reference)
    t0,u0,_ = target(args,0)
    poses, q = setup(e, t0,u0,np.deg2rad(args.roll))
    commands = []
    ik = scratch_ik(e); audit = SubstepAudit(e); e.row_observer = audit.tick
    states, velocities, times = [], [], []
    def step(stage, error=None):
        e.step(q['left'], q['right'])
        row = audit.row(stage, q, error)
        row['target_tu_lane'] = list(commands[-1]) if commands else [t0,u0,0]
        states.append(e.data.qpos.copy()); velocities.append(e.data.qvel.copy()); times.append(e.data.time)
    for i in range(40):
        for side, opening in [('left', LEFT_OPEN), ('right', .017)]:
            q[side][6] = opening/.0524125*max(0., 1-(i+1)/12)
        step('settle and close pregrasp')
    audit.stage = 1
    for i in range(round(args.seconds*20)):
        t,u,lane = target(args,i/20)
        commands.append((float(t),float(u),lane))
        p, n = surface(t, u); rot = orientation(-n, [np.cos(np.deg2rad(args.roll)), 0, np.sin(np.deg2rad(args.roll))])
        gap = .006 - args.load*min(1., i/20)
        saved = e.data.qpos.copy()
        fit, res = ik.fit('right', p+(PAD_Z+.020+gap)*n, rot, seed=q['right'][:6], starts=0)
        np.testing.assert_array_equal(saved, e.data.qpos)
        q['right'] = np.r_[fit, 0.]
        step('scripted scrub', dict(position_m=float(np.linalg.norm(res[:3])), orientation_deg=float(np.degrees(np.linalg.norm(res[3:])/.15))))
    audit.stage = 2
    for _ in range(20): step('finish hold')
    np.savez_compressed(out/'physics_replay.npz', qpos=states, qvel=velocities, time=times)
    np.savez_compressed(out/'substeps.npz', samples=audit.samples,
                        columns=['time_s','stage','pad_x','pad_y','pad_z','pad','left_bilateral','right_bilateral','simultaneous',*[f'pad_R{j}' for j in range(9)],'left_clearance_m','pair_motion_bound_m'])
    rows = audit.rows; scrub = [r for r in rows if r['stage'] == 'scripted scrub']
    metrics = motion_metrics(audit.samples, args.centre, args.u)
    baseline = json.loads((HERE/'runs/propeller_corrected/check.json').read_text())
    metrics.update(task='propeller_edge', config=vars(args), duration_s=float(e.data.time), poses=poses,
                   baseline_actual_radial_span_m=baseline['actual_pad_radial_span_m'],
                   baseline_check_sha256=digest(HERE/'runs/propeller_corrected/check.json'),
                   motion_source_sha256=digest(HERE/'propeller_edge_demo.py'),
                   substep_sample_convention='mj_step contact/geom positions at start of 1 ms integration; timestamp is integration end',
                   max_intended_penetration_m=max(r['max_intended_penetration_m'] for r in rows),
                   max_unwanted_penetration_m=max(r['max_unwanted_penetration_m'] for r in rows),
                   unexpected_contact_pairs=audit.unexpected, intended_pair_max_penetration_m=audit.intended_pairs,
                   max_pad_normal_N=max(r['pad_peak_normal_N'] for r in rows),
                   max_scrub_ik_error_m=max(r['ik_error']['position_m'] for r in scrub),
                   max_scrub_ik_angle_deg=max(r['ik_error']['orientation_deg'] for r in scrub),
                   left_edge_radial_range_m=[min(r['left_contact_radial_range_m'][0] for r in scrub if r['left_contact_radial_range_m']), max(r['left_contact_radial_range_m'][1] for r in scrub if r['left_contact_radial_range_m'])],
                   arm_force_limit_fraction={side:sum(r['arm_force_limit_substeps'][side] for r in scrub)/(args.seconds*1000) for side in e.sides},
                   peak_arm_force_by_side_Nm={side:max(r['arm_peak_force_Nm'][side] for r in rows) for side in e.sides},
                   inference_calls=0, billed_calls=0, simulation_only=True, initialized_pregrasp=True, fixed_shaft=True,
                   live_ik_state_unchanged=True, no_external_forces=True, free_tool=True)
    metrics['baseline_ratio'] = metrics['actual_pad_radial_span_m']/metrics['baseline_actual_radial_span_m']
    e.row_observer = None
    z = float(e.data.body('cardinal_tool').xpos[2]); q['right'][6] = 1.
    release = []
    for _ in range(30):
        e.step(q['left'], q['right'])
        release.append(dict(time_s=float(e.data.time), tool_xyz=e.data.body('cardinal_tool').xpos.tolist(), qpos=e.data.qpos.tolist()))
    (out/'release_trace.json').write_text(json.dumps(release, indent=2)+'\n')
    metrics['release_tool_drop_m'] = z-float(e.data.body('cardinal_tool').xpos[2])
    assert before == model_fingerprint(e.model)
    assert xml == build_xml()
    assert not np.any(e.model.eq_type == mujoco.mjtEq.mjEQ_WELD)
    metrics['scene_invariants'] = dict(entire_corrected_xml_equal=True, xml_sha256=hashlib.sha256(xml.encode()).hexdigest(),
                                      compiled_numeric_model_sha256=before, compiled_model_identical_before_after=True, no_welds=True,
                                      corrected_sources_sha256={name:digest(HERE/name) for name in ('propeller_corrected_scene.py','propeller_corrected_demo.py','propeller_corrected_check.py','cross_scenes.py')})
    metrics['clearance'] = clearance_summary(np.asarray(audit.samples))
    metrics['target_passed'] = bool(metrics['actual_pad_radial_span_m']>=.1 and metrics['clearance']['safety_passed'] and
                                    all(s['simultaneous_contact_fraction']>=.99 and s['endpoints_in_contact'] for s in metrics['complete_strokes']) and metrics['scrub_contact_fraction']['simultaneous']>=.99 and metrics['max_intended_penetration_m']<.001 and
                                    not metrics['unexpected_contact_pairs'] and metrics['max_scrub_ik_error_m']<.0001 and
                                    metrics['left_edge_radial_range_m'][0]>.432 and metrics['release_tool_drop_m']>.05)
    (out/'check.json').write_text(json.dumps(metrics, indent=2)+'\n')
    with (out/'trajectory.jsonl').open('w') as f:
        for row in rows: f.write(json.dumps(row)+'\n')
    print(json.dumps({k:metrics[k] for k in ('config','target_passed','actual_pad_radial_span_m','scrub_contact_fraction','max_intended_penetration_m','max_unwanted_penetration_m','max_scrub_ik_error_m','clearance')}, indent=2), flush=True)
    e.close()
    return metrics


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--name', default='')
    p.add_argument('--lanes', default='.10,.19,.28')
    p.add_argument('--shift-s',type=float,default=1.5)
    p.add_argument('--centre', type=float, default=.746)
    p.add_argument('--amplitude', type=float, default=.186)
    p.add_argument('--u', type=float, default=0.)
    p.add_argument('--u-amplitude', type=float, default=0.)
    p.add_argument('--roll', type=float, default=90.)
    p.add_argument('--load', type=float, default=.003)
    p.add_argument('--seconds', type=float, default=24.)
    p.add_argument('--period', type=float, default=12.)
    run(p.parse_args())
