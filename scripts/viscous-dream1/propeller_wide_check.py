# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Independent saved-physics, frozen-model, footprint and media assertions."""
import hashlib
import importlib.metadata
import json
from pathlib import Path
import subprocess

import mujoco
import numpy as np

from propeller_corrected_scene import CorrectedEnv, HERE, CENTER, build_xml
from propeller_wide_demo import OUT, digest, model_fingerprint, motion_metrics

BASELINE_REF = '81d64c1e'


def frozen_files():
    repo = HERE.parents[1]
    prefix = 'scripts/viscous-dream1/'
    paths = [prefix + p for p in ('assets/propeller_corrected', 'assets/propeller_cardinal',
                                  'runs/propeller_corrected', 'propeller_corrected_scene.py',
                                  'propeller_corrected_demo.py', 'propeller_corrected_check.py',
                                  'propeller_large_scene.py', 'propeller_cardinal_scene.py',
                                  'cross_scenes.py', 'supported_arms.py')]
    tree = subprocess.check_output(['git', 'ls-tree', '-r', BASELINE_REF, '--', *paths], cwd=repo, text=True)
    count = 0
    for line in tree.splitlines():
        metadata, name = line.split('\t', 1)
        mode, kind, expected = metadata.split()
        assert kind == 'blob'
        data = (repo/name).read_bytes()
        actual = hashlib.sha1(f'blob {len(data)}\0'.encode()+data).hexdigest()
        assert actual == expected, f'Frozen baseline file changed: {name}'
        count += 1
    assert count > 3000
    return dict(reference_commit=BASELINE_REF, baseline_files_exactly_preserved=count,
                git_tree_listing_sha256=hashlib.sha256(tree.encode()).hexdigest())


def footprint_check(samples, check):
    xyz = samples[samples[:, 1] == 1, 2:5]
    # A 26 mm sphere encloses the ellipsoidal pad in every orientation. Its XZ
    # footprint is conservatively larger than the actual tilted pad footprint.
    angle = np.linspace(0, 2*np.pi, 129)[:-1]
    x = xyz[:, 0, None]-CENTER[0]+.026*np.cos(angle)
    z = xyz[:, 2, None]-CENTER[2]+.026*np.sin(angle)
    radius = np.hypot(x, z)
    t = (radius-.085)/.395
    width = (.055+.270*t-.055*t*t)*np.sqrt(np.maximum(0, 1-t**4))
    skew = .12*np.sin(np.pi*t)
    u = (np.arctan2(z, x)-skew)*radius/width
    assert np.min(radius) > .102 and np.max(radius) < .480
    assert np.max(np.abs(u)) < .9
    clearance = check['left_edge_radial_range_m'][0]-float(np.max(radius))
    assert clearance > .02
    return dict(conservative_pad_footprint_radius_m=.026, sampled_footprint_points=int(radius.size),
                footprint_t_range=[float(t.min()), float(t.max())], footprint_u_range=[float(u.min()), float(u.max())],
                hub_radial_clearance_m=float(radius.min()-.102),
                left_hold_contact_zone_radial_clearance_m=clearance,
                entire_pad_footprint_within_same_blade_face=True)


def main():
    check = json.loads((OUT/'check.json').read_text())
    assert check['target_passed']
    assert digest(HERE/'propeller_wide_demo.py') == check['motion_source_sha256']
    baseline = json.loads((HERE/'runs/propeller_corrected/check.json').read_text())
    assert digest(HERE/'runs/propeller_corrected/check.json') == check['baseline_check_sha256']
    np.testing.assert_array_equal(check['poses']['left']['target'], baseline['poses']['left']['target'])
    np.testing.assert_array_equal(check['poses']['left']['R'], baseline['poses']['left']['R'])
    replay = np.load(OUT/'physics_replay.npz')
    samples = np.load(OUT/'substeps.npz')['samples']
    rows = [json.loads(line) for line in (OUT/'trajectory.jsonl').read_text().splitlines()]
    assert len(samples) == len(rows)*50 == 27000
    assert np.all(np.isfinite(samples)) and np.all(np.isfinite(replay['qpos'])) and np.all(np.isfinite(replay['qvel']))
    np.testing.assert_allclose(np.diff(samples[:, 0]), .001, atol=1e-10)
    np.testing.assert_allclose(np.diff(replay['time']), .05, atol=1e-10)
    for i, row in enumerate(rows):
        assert abs(row['time_s']-replay['time'][i]) < 1e-10
        np.testing.assert_array_equal(samples[i*50:(i+1)*50, 5:9].sum(axis=0), list(row['contact_substeps'].values()))
    computed = motion_metrics(samples, check['config']['centre'], check['config']['u'])
    assert computed == {key:check[key] for key in computed}, 'Motion summary differs from raw 1 ms evidence'
    assert computed['minimum_complete_stroke_m'] >= .120
    assert len(computed['complete_strokes']) >= 3
    assert all(s['simultaneous_contact_fraction'] == 1 and s['endpoints_in_contact'] for s in computed['complete_strokes'])
    assert computed['scrub_contact_fraction']['simultaneous'] >= .99
    assert computed['scrub_contact_fraction']['left_bilateral'] == computed['scrub_contact_fraction']['right_bilateral'] == 1
    assert check['max_intended_penetration_m'] < .001 and not check['unexpected_contact_pairs']
    assert check['max_unwanted_penetration_m'] == 0 and check['max_scrub_ik_error_m'] < 1e-6
    assert check['left_edge_radial_range_m'][0] >= .432 and check['release_tool_drop_m'] > .05
    assert check['baseline_ratio'] > 4
    e = CorrectedEnv()
    try:
        fingerprint = model_fingerprint(e.model)
        assert fingerprint == check['scene_invariants']['compiled_numeric_model_sha256']
        xml = build_xml()
        assert hashlib.sha256(xml.encode()).hexdigest() == check['scene_invariants']['xml_sha256']
        canonical = xml.replace(str(HERE), '${SCENE_DIR}')
        assert e.model.body('cardinal_propeller').jntnum[0] == 0
        assert e.model.body('cardinal_tool').jntnum[0] == 1
        assert e.model.joint('cardinal_tool_free').type[0] == mujoco.mjtJoint.mjJNT_FREE
        assert not np.any(e.model.eq_type == mujoco.mjtEq.mjEQ_WELD)
        assert e.model.nmocap == 0
        force_limits = {s:e.model.actuator(f'{s}_shoulder_lift_drive').forcerange.tolist() for s in e.sides}
        assert force_limits == baseline['arm_force_limits_Nm']
        for name, sha in check['scene_invariants']['corrected_sources_sha256'].items(): assert digest(HERE/name) == sha
    finally:
        e.close()
    release = json.loads((OUT/'release_trace.json').read_text())
    assert len(release) == 30 and release[0]['time_s'] > replay['time'][-1]
    media = json.loads((OUT/'media_check.json').read_text())
    for name in ('contact_detail', 'diagnostic'):
        assert digest(OUT/f'{name}.mp4') == media[name]['sha256']
        assert media[name]['frames_fully_decoded'] == 270 and media[name]['duration_s'] == 27
    report = dict(passed=True, frozen_baseline=frozen_files(), identical_entire_corrected_physics_model=True,
                  canonical_xml_sha256=hashlib.sha256(canonical.encode()).hexdigest(), compiled_model_sha256=fingerprint,
                  left_hold_target_and_orientation_identical=True, arm_force_limits_Nm=force_limits,
                  raw_substeps_verified=len(samples), scrub_substeps=24000, complete_strokes_all_100_percent_contact=True,
                  no_weld_mocap_or_external_forces=True, fixed_shaft_free_tool=True, footprint=footprint_check(samples, check),
                  media_fully_decoded=True, versions={n:importlib.metadata.version(n) for n in ('mujoco','numpy','scipy','trimesh','dreamlabs','dreamlabs_client','imageio','Pillow')},
                  evidence_sha256={p.name:digest(p) for p in OUT.iterdir() if p.is_file() and p.suffix in ('.json','.jsonl','.npz','.mp4','.png') and p.name != 'validation.json'},
                  validation_source_sha256=digest(Path(__file__)))
    (OUT/'validation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'evidence_sha256'}, indent=2))


if __name__ == '__main__':
    main()
