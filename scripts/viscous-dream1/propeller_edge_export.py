# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Export observed poses plus contact-gated nominal footprint, never abrasion.

Actual rigid ellipsoid contact consists of solver points: those do not establish
52 mm of uniform loaded contact. The swept nominal disk is a presentation mask
and an upper footprint estimate, not measured area of removed fouling.
"""
import argparse
import json
import numpy as np
import trimesh
from scipy.spatial import cKDTree
import shapely
from propeller_corrected_scene import ASSETS, CENTER, HERE, surface
from propeller_edge_geometry import parameters, boundary
from propeller_wide_demo import digest

OUT=HERE/'runs/propeller_edge'


def footprint_polygons(samples, count=128):
    angle=np.arange(count)*2*np.pi/count
    circle=.026*np.column_stack((np.cos(angle),np.sin(angle)))
    rot=samples[:,9:18].reshape(-1,3,3)
    points=samples[:,None,[2,4]]+np.einsum('nij,kj->nki',rot[:,[0,2],:2],circle)
    return shapely.polygons(points)


def coverage(samples, edge_size):
    mesh=trimesh.load(ASSETS/'blade_0.stl',force='mesh')
    triangles=mesh.triangles[mesh.face_normals[:,1]>1e-8]
    base=trimesh.Trimesh(triangles.reshape(-1,3),np.arange(len(triangles)*3).reshape(-1,3),process=False)
    v,f=trimesh.remesh.subdivide_to_size(base.vertices,base.faces,max_edge=edge_size,max_iter=12)
    tri=v[f]; points=tri.mean(axis=1)+CENTER
    areas=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)/2
    tree=cKDTree(points[:,[0,2]])
    first=np.full(len(points),np.inf)
    actual_first=np.full(len(points),np.inf)
    eligible=samples[(samples[:,1]==1)&(samples[:,8]==1)]
    for s in eligible:
        ids=np.asarray(tree.query_ball_point(s[[2,4]],.026),dtype=int)
        if not len(ids):continue
        rot=s[9:18].reshape(3,3)
        a=rot[[0,2],:2]
        delta=points[ids][:,[0,2]]-s[[2,4]]
        det=a[0,0]*a[1,1]-a[0,1]*a[1,0]
        xy=np.column_stack(((a[1,1]*delta[:,0]-a[0,1]*delta[:,1])/det,(-a[1,0]*delta[:,0]+a[0,0]*delta[:,1])/det))
        hit=ids[(np.sum(xy*xy,axis=1)<=.026**2)&np.isinf(first[ids])]
        first[hit]=s[0]
        local=(points[ids]-s[2:5])@rot
        actual=ids[(np.sum((local/[.026,.026,.006])**2,axis=1)<=1)&np.isinf(actual_first[ids])]
        actual_first[actual]=s[0]
    hit=np.isfinite(first)
    _,n=surface(.7,.2)
    radial=surface(.70001,.2)[0]-surface(.69999,.2)[0];radial/=np.linalg.norm(radial)
    chord=np.cross(radial,n);chord/=np.linalg.norm(chord)
    return dict(points=points,areas=areas,first=first,actual_first=actual_first),dict(
        surface_triangle_max_edge_m=edge_size,surface_samples=len(points),
        blade_front_area_m2=float(areas.sum()),
        unique_geometric_touch_area_m2=float(areas[np.isfinite(actual_first)].sum()),
        geometric_touch_fraction=float(areas[np.isfinite(actual_first)].sum()/areas.sum()),
        geometric_touch_definition='Area quadrature on collision-matched front triangles whose centroids enter actual pad ellipsoid at a simultaneous-contact 1ms substep. Contact-overlap estimate, not abrasion. Submillimetre contact patch discretization may miss smaller touches.',unique_contact_gated_nominal_footprint_area_m2=float(areas[hit].sum()),
        blade_front_nominal_coverage_fraction=float(areas[hit].sum()/areas.sum()),
        global_footprint_chordwise_width_m=float(np.ptp(points[hit]@chord)),
        global_footprint_radial_width_m=float(np.ptp(points[hit]@radial)),
        interpretation='Contact-gated swept nominal 52 mm disk on actual blade triangles; upper footprint estimate, not force-bearing contact area or removed fouling. No abrasion model.')


def main():
    p=argparse.ArgumentParser();p.add_argument('--name',default='');args=p.parse_args()
    out=OUT/args.name if args.name else OUT
    check=json.loads((out/'check.json').read_text());assert check['target_passed']
    s=np.load(out/'substeps.npz')['samples']
    rows=[json.loads(line) for line in (out/'trajectory.jsonl').read_text().splitlines()]
    frames=[]
    for i,row in enumerate(rows):
        sample=s[(i+1)*50-1];t,u=parameters(sample[2:5]);_,n=surface(t,u)
        ct,cu,lane=row['target_tu_lane']
        frames.append(dict(frame_index=i,time_s=float(sample[0]),stage=row['stage'],lane_index=lane,
            commanded_t=ct,commanded_u=cu,actual_t=float(t),actual_u=float(u),
            pad_center_world_m=sample[2:5].tolist(),normal_world=n.tolist(),pad_rotation_world=sample[9:18].reshape(3,3).tolist(),
            contact_duty={k:v/50 for k,v in row['contact_substeps'].items()}))
    data,metrics=coverage(s,.0015)
    _,fine=coverage(s,.001)
    metrics['area_refinement_difference_m2']=fine['unique_contact_gated_nominal_footprint_area_m2']-metrics['unique_contact_gated_nominal_footprint_area_m2']
    metrics['refined_1mm_area_m2']=fine['unique_contact_gated_nominal_footprint_area_m2']
    metrics['refined_1mm_geometric_touch_area_m2']=fine['unique_geometric_touch_area_m2']
    metrics['geometric_touch_area_refinement_difference_m2']=fine['unique_geometric_touch_area_m2']-metrics['unique_geometric_touch_area_m2']
    scrub=s[s[:,1]==1];polygons=footprint_polygons(scrub)
    rim=shapely.Polygon(boundary())
    margin=np.asarray(shapely.distance(polygons,rim.boundary))
    inside=np.asarray(shapely.covers(rim,polygons))
    metrics['actual_oriented_disk_minimum_sampled_rim_clearance_m']=float(margin.min())
    metrics['all_oriented_footprints_inside_blade']=bool(inside.all())
    metrics['perimeter_polygon_error_bound_m']=float(.026*(1-np.cos(np.pi/128)))
    metrics['cumulative_geometric_touch_area_by_frame_m2']=[float(data['areas'][data['actual_first']<=r['time_s']].sum()) for r in rows]
    metrics['cumulative_area_by_frame_m2']=[float(data['areas'][data['first']<=r['time_s']].sum()) for r in rows]
    np.savez_compressed(out/'coverage_mask.npz',surface_xyz_m=data['points'],surface_tu=parameters(data['points']),
                        area_m2=data['areas'],first_touch_time_s=data['actual_first'],nominal_first_touch_time_s=data['first'])
    plan=dict(schema='propeller-edge-coverage-v1',fps=20,units='metres_seconds',source='observed scripted MuJoCo bench physics',
        surface=dict(identifier='sole-inspired-helical-v1',blade_id=0,face='+Y',diameter_m=.96,
            source='scripts/viscous-dream1/propeller_corrected_scene.py',surface_function='surface(t,u)',
            mesh='scripts/viscous-dream1/assets/propeller_corrected/blade_0.stl',mesh_sha256=digest(ASSETS/'blade_0.stl'),
            center_world_m=CENTER.tolist(),coordinates='t: root-to-tip [0,1]; u: chord [-1,1]; blade0 world +X = screen-left from +Y'),
        pad=dict(diameter_m=.052,ellipsoid_half_thickness_m=.006,nominal_disk_radius_m=.026,
                 center_offset_from_tool_body_m=[0,0,.075],face_offset_from_tool_body_m=[0,0,.081]),
        left_hold=dict(t=.97,u=0,**check['poses']['left']),
        lanes=[float(v) for v in check['config']['lanes'].split(',')] if check['config']['lanes'] else [check['config']['u']],
        contact_gate='frames[i].contact_duty.simultaneous; fraction of preceding 50 one-millisecond substeps; only stage scripted scrub contributes mask',
        timestamp_convention=check['substep_sample_convention'],
        retargeting='These world poses are validated only on the unchanged fixed bench. Retargeting to BlueROV mounts is kinematic presentation, not validated wet dynamics.',
        cleaning='Coverage is a contact-gated nominal pad footprint estimate. Render only cumulative footprint; no whole-blade clean or material-removal claim.',
        clearance=check['clearance'],coverage=metrics,coverage_mask='coverage_mask.npz: surface_xyz_m, surface_tu, area_m2, first_touch_time_s (geometric overlap), nominal_first_touch_time_s (nominal whole-disk sweep)',frames=frames,
        evidence_sha256={n:digest(out/n) for n in ('substeps.npz','physics_replay.npz','check.json','trajectory.jsonl')})
    (out/'coverage_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    (out/'coverage_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    from propeller_edge_map import render
    render(out)
    print(json.dumps({k:v for k,v in metrics.items() if not k.startswith('cumulative_')},indent=2))

if __name__=='__main__':main()
