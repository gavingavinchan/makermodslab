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

OUT = HERE / 'runs/propeller_wide'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def model_fingerprint(model):
    """All exposed numeric compiled arrays, plus names and scalar options."""
    h = hashlib.sha256()
    for name in sorted(dir(model)):
        value = getattr(model, name)
        if isinstance(value, np.ndarray):
            h.update(name.encode()); h.update(value.tobytes())
    for name in sorted(dir(model.opt)):
        value = getattr(model.opt, name)
        if isinstance(value, (np.ndarray, int, float)):
            h.update(name.encode()); h.update(np.asarray(value).tobytes())
    return h.hexdigest()


def scratch_ik(e):
    scratch = copy.copy(e)
    scratch.data = mujoco.MjData(e.model)
    mujoco.mj_copyData(scratch.data, e.model, e.data)
    return local_ik(scratch)


def setup(e, t, u, roll=0.):
    ik = scratch_ik(e)
    p, n = surface(LEFT_T)
    leftR = orientation([-1, -.3, 0], -n)
    leftp = p - .024 * leftR[:, 2] - .003 * n
    p, n = surface(t, u)
    rightR = orientation(-n, [np.cos(roll), 0, np.sin(roll)])
    rightp = p + (PAD_Z + .020 + .006) * n
    poses, q = {}, {}
    for side, point, rot, opening in [('left', leftp, leftR, LEFT_OPEN), ('right', rightp, rightR, .017)]:
        fit, res = ik.fit(side, point, rot, starts=20)
        assert np.linalg.norm(res[:3]) < .002 and np.linalg.norm(res[3:]) / .15 < np.deg2rad(2)
        e.data.qpos[e.qadr[side]] = fit
        e.data.ctrl[e.aids[side]] = fit
        for sign in (-1, 1):
            e.data.joint(f'{side}_finger_{sign}').qpos[0] = opening
        mujoco.mj_forward(e.model, e.data)
        q[side] = np.r_[fit, 0.]
        poses[side] = dict(target=point.tolist(), R=rot.tolist(), q=fit.tolist(), error_m=float(np.linalg.norm(res[:3])))
    hand = e.data.site('right_tcp'); rot = hand.xmat.reshape(3, 3)
    a = e.model.joint('cardinal_tool_free').qposadr[0]
    e.data.qpos[a:a+3] = hand.xpos + .020 * rot[:, 2]
    mujoco.mju_mat2Quat(e.data.qpos[a+3:a+7], rot.ravel())
    e.data.qvel[:] = 0
    mujoco.mj_forward(e.model, e.data)
    return poses, q


class SubstepAudit(Audit):
    def __init__(self, e):
        super().__init__(e)
        self.samples = []
        self.stage = 0
    def tick(self):
        before = self.counts.copy()
        super().tick()
        assert not np.any(self.e.data.xfrc_applied) and not np.any(self.e.data.qfrc_applied)
        self.samples.append([self.e.data.time, self.stage, *self.e.data.geom('cardinal_pad').xpos,
                             *[self.counts[k]-before[k] for k in self.counts]])


def motion_metrics(samples, centre_t, centre_u):
    samples = np.asarray(samples)
    s = samples[samples[:, 1] == 1]
    xyz = s[:, 2:5]; valid = s[:, 8].astype(bool)
    contact = xyz[valid]
    p, n = surface(centre_t, centre_u)
    radial = surface(centre_t+1e-5, centre_u)[0] - surface(centre_t-1e-5, centre_u)[0]
    radial /= np.linalg.norm(radial)
    chord = np.cross(radial, n); chord /= np.linalg.norm(chord)
    projected = np.column_stack([(contact-p) @ radial, (contact-p) @ chord])
    radii = np.linalg.norm((xyz-CENTER)[:, [0, 2]], axis=1)
    # Actual surface parameters inferred from cylindrical x/z, not commanded t/u.
    actual_t = (radii-ROOT_RADIUS)/(RADIUS-ROOT_RADIUS)
    actual_u = []
    for pos, t in zip(xyz, actual_t):
        r, width, skew, _, _ = section(float(t))
        actual_u.append((np.arctan2(pos[2]-CENTER[2], pos[0]-CENTER[0])-skew)*r/width)
    # 10 mm prominence rejects servo chatter; extrema are actual measured positions.
    hi = find_peaks(radii, prominence=.01, distance=1000)[0]
    lo = find_peaks(-radii, prominence=.01, distance=1000)[0]
    extrema = sorted([*hi, *lo])
    strokes = []
    for a, b in zip(extrema, extrema[1:]):
        delta = xyz[b]-xyz[a]
        strokes.append(dict(start_s=float(s[a, 0]), end_s=float(s[b, 0]), radial_amplitude_m=float(abs(radii[b]-radii[a])),
                            endpoint_distance_m=float(np.linalg.norm(delta)), projected_tangent_amplitude_m=float(np.hypot(delta@radial, delta@chord)),
                            simultaneous_contact_fraction=float(np.mean(valid[a:b+1])), endpoints_in_contact=bool(valid[a] and valid[b])))
    segments = np.split(np.arange(len(valid)), np.where(np.diff(valid.astype(int)) != 0)[0]+1)
    contact_segments = [v for v in segments if valid[v[0]]]
    missing_segments = [v for v in segments if not valid[v[0]]]
    d = np.diff(xyz, axis=0)
    normals = np.array([surface(float(t), float(u))[1] for t, u in zip(actual_t, actual_u)])
    tangential = d - np.sum(d * normals[:-1], axis=1)[:, None] * normals[:-1]
    continuous = valid[:-1] & valid[1:]
    hull = ConvexHull(contact)
    full_span = float(np.max(pdist(contact[hull.vertices])))
    return dict(actual_pad_radial_span_m=float(np.ptp(radii[valid])), pad_motion_xyz_range_m=np.ptp(contact, axis=0).tolist(),
                actual_full_3d_span_m=full_span, projected_radial_tangent_span_m=float(np.ptp(projected[:, 0])),
                projected_chordwise_span_m=float(np.ptp(projected[:, 1])),
                actual_t_range=[float(min(actual_t)), float(max(actual_t))], actual_u_range=[float(min(actual_u)), float(max(actual_u))],
                actual_continuous_contact_tangential_path_m=float(np.linalg.norm(tangential[continuous], axis=1).sum()),
                radial_reversals_10mm=len(extrema), complete_strokes=strokes,
                longest_contact_gap_s=max((len(v)*.001 for v in missing_segments), default=0),
                uninterrupted_contact_radial_spans_m=[float(np.ptp(radii[v])) for v in contact_segments],
                minimum_complete_stroke_m=min((v['radial_amplitude_m'] for v in strokes), default=0),
                scrub_contact_fraction={key:float(np.mean(s[:, i])) for i,key in enumerate(('pad','left_bilateral','right_bilateral','simultaneous'), 5)})


def run(args):
    out = OUT / args.name if args.name else OUT
    out.mkdir(parents=True, exist_ok=True)
    assert 0 < args.centre-args.amplitude < args.centre+args.amplitude < .9
    assert abs(args.u)+abs(args.u_amplitude) < .6
    assert args.seconds > 0 and args.period > 0
    e = CorrectedEnv()
    before = model_fingerprint(e.model)
    xml = build_xml()
    reference = mujoco.MjModel.from_xml_string(xml)
    assert before == model_fingerprint(reference)
    poses, q = setup(e, args.centre, args.u, np.deg2rad(args.roll))
    ik = scratch_ik(e); audit = SubstepAudit(e); e.row_observer = audit.tick
    states, velocities, times = [], [], []
    def step(stage, error=None):
        e.step(q['left'], q['right'])
        audit.row(stage, q, error)
        states.append(e.data.qpos.copy()); velocities.append(e.data.qvel.copy()); times.append(e.data.time)
    for i in range(40):
        for side, opening in [('left', LEFT_OPEN), ('right', .017)]:
            q[side][6] = opening/.0524125*max(0., 1-(i+1)/12)
        step('settle and close pregrasp')
    audit.stage = 1
    for i in range(round(args.seconds*20)):
        t = args.centre + args.amplitude*np.sin(2*np.pi*i/(args.period*20))
        u = args.u + args.u_amplitude*np.sin(2*np.pi*i/(args.period*20))
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
                        columns=['time_s','stage','pad_x','pad_y','pad_z','pad','left_bilateral','right_bilateral','simultaneous'])
    rows = audit.rows; scrub = [r for r in rows if r['stage'] == 'scripted scrub']
    metrics = motion_metrics(audit.samples, args.centre, args.u)
    baseline = json.loads((HERE/'runs/propeller_corrected/check.json').read_text())
    metrics.update(task='propeller_wide', config=vars(args), duration_s=float(e.data.time), poses=poses,
                   baseline_actual_radial_span_m=baseline['actual_pad_radial_span_m'],
                   baseline_check_sha256=digest(HERE/'runs/propeller_corrected/check.json'),
                   motion_source_sha256=digest(HERE/'propeller_wide_demo.py'),
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
    metrics['target_passed'] = bool(metrics['minimum_complete_stroke_m']>=.1 and len(metrics['complete_strokes'])>=2 and
                                    all(s['simultaneous_contact_fraction']>=.99 and s['endpoints_in_contact'] for s in metrics['complete_strokes']) and metrics['scrub_contact_fraction']['simultaneous']>=.99 and metrics['max_intended_penetration_m']<.001 and
                                    not metrics['unexpected_contact_pairs'] and metrics['max_scrub_ik_error_m']<.0001 and
                                    metrics['left_edge_radial_range_m'][0]>.432 and metrics['release_tool_drop_m']>.05)
    (out/'check.json').write_text(json.dumps(metrics, indent=2)+'\n')
    with (out/'trajectory.jsonl').open('w') as f:
        for row in rows: f.write(json.dumps(row)+'\n')
    print(json.dumps(metrics, indent=2), flush=True)
    e.close()
    return metrics


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--name', default='')
    p.add_argument('--centre', type=float, default=.64)
    p.add_argument('--amplitude', type=float, default=.19)
    p.add_argument('--u', type=float, default=0.)
    p.add_argument('--u-amplitude', type=float, default=0.)
    p.add_argument('--roll', type=float, default=90.)
    p.add_argument('--load', type=float, default=.003)
    p.add_argument('--seconds', type=float, default=24.)
    p.add_argument('--period', type=float, default=12.)
    run(p.parse_args())
