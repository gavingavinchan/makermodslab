# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Compare unique overlap against an instrumented unchanged wide-motion rerun."""
import json
import numpy as np
from propeller_corrected_scene import surface
from propeller_edge_demo import OUT
from propeller_edge_geometry import parameters, point_boundary_distance


def extra_coverage(out,metrics):
    data=np.load(out/'coverage_mask.npz');points=data['surface_xyz_m']
    actual=points[np.isfinite(data['first_touch_time_s'])]
    _,n=surface(.7,.2);radial=surface(.70001,.2)[0]-surface(.69999,.2)[0];radial/=np.linalg.norm(radial)
    chord=np.cross(radial,n);chord/=np.linalg.norm(chord)
    metrics['global_geometric_touch_chordwise_width_m']=float(np.ptp(actual@chord))
    metrics['global_geometric_touch_radial_width_m']=float(np.ptp(actual@radial))
    metrics['geometric_touch_closest_rim_sample_m']=min(float(point_boundary_distance(p[:,[0,2]]).min()) for p in np.array_split(actual,20) if len(p))
    (out/'coverage_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')


def main():
    plan=json.loads((OUT/'coverage_plan.json').read_text())
    baseline=json.loads((OUT/'candidates/single_lane_baseline/coverage_plan.json').read_text())
    a,b=plan['coverage'],baseline['coverage']
    extra_coverage(OUT,a);extra_coverage(OUT/'candidates/single_lane_baseline',b)
    lanes=plan['lanes'];t=.74
    points=np.array([surface(t,u)[0] for u in lanes])
    rows=[json.loads(line) for line in (OUT/'trajectory.jsonl').read_text().splitlines()]
    samples=np.load(OUT/'substeps.npz')['samples'];tu=parameters(samples[:,2:5])
    row_lanes=np.repeat([r['target_tu_lane'][2] for r in rows],50)
    actual=[]
    for i in range(len(lanes)):
        selected=(samples[:,1]==1)&(row_lanes==i)&(abs(tu[:,0]-t)<.004)
        lane_samples=(samples[:,1]==1)&(row_lanes==i)
        actual.append(dict(lane=i,commanded_u=lanes[i],actual_radial_span_m=float(np.ptp(tu[lane_samples,0])*.395),samples_at_shared_t_band=int(selected.sum()),
                           actual_u_mean_at_t074=float(tu[selected,1].mean()) if selected.any() else None))
    compare=dict(baseline='instrumented same-model rerun of wide .64 + .19 sine, u=0,24s scrub',
        selected_scrub_s=sum(f['stage']=='scripted scrub' for f in plan['frames'])/20,
        baseline_scrub_s=24,
        actual_geometric_touch_area_ratio=a['refined_1mm_geometric_touch_area_m2']/b['refined_1mm_geometric_touch_area_m2'],
        actual_geometric_touch_area_gain_m2=a['refined_1mm_geometric_touch_area_m2']-b['refined_1mm_geometric_touch_area_m2'],
        nominal_footprint_area_ratio=a['refined_1mm_area_m2']/b['refined_1mm_area_m2'],
        nominal_footprint_area_gain_m2=a['refined_1mm_area_m2']-b['refined_1mm_area_m2'],
        selected_geometric_touch_area_m2=a['refined_1mm_geometric_touch_area_m2'],baseline_geometric_touch_area_m2=b['refined_1mm_geometric_touch_area_m2'],
        selected_nominal_footprint_area_m2=a['refined_1mm_area_m2'],baseline_nominal_footprint_area_m2=b['refined_1mm_area_m2'],
        selected_actual_touch_width_m=a['global_geometric_touch_chordwise_width_m'],baseline_actual_touch_width_m=b['global_geometric_touch_chordwise_width_m'],
        selected_global_coverage_width_m=a['global_footprint_chordwise_width_m'],baseline_global_coverage_width_m=b['global_footprint_chordwise_width_m'],
        lane_offsets_u=lanes,commanded_adjacent_lane_spacing_at_t074_m=np.linalg.norm(np.diff(points,axis=0),axis=1).tolist(),
        commanded_first_to_last_lane_offset_at_t074_m=float(np.linalg.norm(points[-1]-points[0])),
        observed_lanes=actual,
        note='Global width includes blade curvature and path orientation; shared-t lane spacing isolates the lateral shift. Areas are union areas, never accumulated overlapping footprints. Neither estimates abrasion.')
    plan['single_lane_comparison']=compare
    (OUT/'coverage_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    (OUT/'coverage_comparison.json').write_text(json.dumps(compare,indent=2)+'\n')
    print(json.dumps(compare,indent=2))

if __name__=='__main__':main()
