# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Validate raw edge-motion evidence and frozen source/physics invariants."""
import hashlib
import json
import subprocess
import numpy as np
import mujoco
from propeller_corrected_scene import CorrectedEnv, HERE, build_xml
from propeller_edge_demo import OUT
from propeller_edge_geometry import parameters, clearance_summary, point_boundary_distance, boundary
from propeller_wide_demo import digest, model_fingerprint, motion_metrics
from propeller_wide_check import frozen_files


def geometry_tests():
    from propeller_corrected_scene import surface
    for t,u in ((.2,-.5),(.7,.2),(.94,.25)):
        p,_=surface(t,u);np.testing.assert_allclose(parameters(p),[t,u],atol=1e-12)
    np.testing.assert_allclose(point_boundary_distance(boundary()),0,atol=1e-12)
    from types import SimpleNamespace
    from propeller_edge_demo import target
    args=SimpleNamespace(lanes='.13,.17,.21,.25',centre=.738,amplitude=.198,period=12.,shift_s=1.5)
    for time,t,u in ((0,.738,.13),(3,.936,.13),(4.5,.936,.17),(10.5,.54,.17),(12,.54,.21),(18,.936,.21),(19.5,.936,.25),(25.5,.54,.25)):
        np.testing.assert_allclose(target(args,time)[:2],[t,u],atol=1e-12)
    path=np.array([target(args,time)[:2] for time in np.linspace(0,25.5,25501)])
    assert np.max(np.linalg.norm(np.diff(path,axis=0),axis=1))<.0002
    assert np.all(np.diff(path[:,1])>=-1e-12)


def main():
    geometry_tests()
    check=json.loads((OUT/'check.json').read_text());assert check['target_passed']
    s=np.load(OUT/'substeps.npz')['samples'];replay=np.load(OUT/'physics_replay.npz')
    rows=[json.loads(line) for line in (OUT/'trajectory.jsonl').read_text().splitlines()]
    plan=json.loads((OUT/'coverage_plan.json').read_text());mask=np.load(OUT/'coverage_mask.npz')
    assert len(s)==len(rows)*50 and len(plan['frames'])==len(rows)
    assert np.isfinite(s).all() and np.isfinite(replay['qpos']).all()
    np.testing.assert_allclose(np.diff(s[:,0]),.001,atol=1e-10)
    np.testing.assert_allclose(np.diff(replay['time']),.05,atol=1e-10)
    for i,(row,frame) in enumerate(zip(rows,plan['frames'])):
        count=s[i*50:(i+1)*50,5:9].sum(axis=0)
        np.testing.assert_array_equal(count,list(row['contact_substeps'].values()))
        np.testing.assert_array_equal(count/50,list(frame['contact_duty'].values()))
        np.testing.assert_array_equal(frame['pad_center_world_m'],s[(i+1)*50-1,2:5])
    computed=motion_metrics(s,check['config']['centre'],check['config']['u'])
    assert computed=={k:check[k] for k in computed}
    assert clearance_summary(s)==check['clearance']
    assert computed['scrub_contact_fraction']['left_bilateral']==computed['scrub_contact_fraction']['right_bilateral']==1
    assert computed['scrub_contact_fraction']['simultaneous']>=.99
    assert not check['unexpected_contact_pairs'] and check['max_intended_penetration_m']<.001
    assert check['max_scrub_ik_error_m']<.0001
    assert check['clearance']['safety_passed']
    assert np.all(np.diff(plan['coverage']['cumulative_area_by_frame_m2'])>=0)
    assert plan['coverage']['blade_front_nominal_coverage_fraction']<.5
    assert plan['coverage']['all_oriented_footprints_inside_blade']
    np.testing.assert_allclose(mask['area_m2'][np.isfinite(mask['nominal_first_touch_time_s'])].sum(),plan['coverage']['unique_contact_gated_nominal_footprint_area_m2'])
    assert abs(plan['coverage']['area_refinement_difference_m2'])<.0001
    e=CorrectedEnv()
    assert model_fingerprint(e.model)==check['scene_invariants']['compiled_numeric_model_sha256']
    assert e.model.nmocap==0 and not np.any(e.model.eq_type==mujoco.mjtEq.mjEQ_WELD)
    assert e.model.body('cardinal_propeller').jntnum[0]==0
    assert e.model.joint('cardinal_tool_free').type[0]==mujoco.mjtJoint.mjJNT_FREE
    assert all(np.array_equal(e.model.actuator(f'{side}_shoulder_lift_drive').forcerange,[-5,5]) for side in e.sides)
    original=json.loads((HERE/'runs/propeller_corrected/check.json').read_text())
    np.testing.assert_array_equal(check['poses']['left']['target'],original['poses']['left']['target'])
    np.testing.assert_array_equal(check['poses']['left']['R'],original['poses']['left']['R'])
    canonical=build_xml().replace(str(HERE),'${SCENE_DIR}')
    frozen=frozen_files()
    # Current wide baseline is frozen as well, including prior results.
    prefix='scripts/viscous-dream1/'
    paths=[prefix+'propeller_wide_demo.py',prefix+'propeller_wide_media.py',prefix+'propeller_wide_check.py',prefix+'runs/propeller_wide']
    tree=subprocess.check_output(['git','ls-tree','-r','288f86ef','--',*paths],cwd=HERE.parents[1],text=True)
    for line in tree.splitlines():
        metadata,name=line.split('\t');expected=metadata.split()[2];data=(HERE.parents[1]/name).read_bytes()
        assert hashlib.sha1(f'blob {len(data)}\0'.encode()+data).hexdigest()==expected,name
    media=json.loads((OUT/'media_check.json').read_text())
    for name in ('contact_detail','diagnostic'):
        assert media[name]['frames_fully_decoded']==len(rows)//2 and digest(OUT/f'{name}.mp4')==media[name]['sha256']
    assert digest(HERE/'propeller_edge_demo.py')==check['motion_source_sha256']
    report=dict(passed=True,raw_substeps=len(s),frames=len(rows),frozen_corrected=frozen,frozen_wide_files=len(tree.splitlines()),
        canonical_xml_sha256=hashlib.sha256(canonical.encode()).hexdigest(),compiled_model_sha256=model_fingerprint(e.model),
        clearance=check['clearance'],all_media_fully_decoded=True,geometry_unit_tests_passed=True,
        coverage_area_refinement_difference_m2=plan['coverage']['area_refinement_difference_m2'],
        evidence_sha256={p.name:digest(p) for p in OUT.iterdir() if p.is_file() and p.name!='validation.json'},
        source_sha256={p.name:digest(p) for p in HERE.glob('propeller_edge_*.py')})
    (OUT/'validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if not k.endswith('sha256')},indent=2));e.close()

if __name__=='__main__':main()
