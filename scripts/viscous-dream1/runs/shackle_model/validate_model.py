"""Static mesh measurements only. Does not import or run MuJoCo."""
from pathlib import Path
import importlib.util
import json
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from shapely.geometry import MultiPoint, Point

RUN=Path(__file__).resolve().parent;ROOT=RUN.parents[1]
spec=importlib.util.spec_from_file_location('shackle_assets',ROOT/'shackle_assets.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s)
A=ROOT/'assets/shackle'
body=trimesh.load_mesh(A/'shackle_body.stl');pin=trimesh.load_mesh(A/'shackle_pin.stl')
for mesh in [body,pin]:
    assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume>0
    assert len(mesh.split())==1
manifest=json.loads((A/'collision_manifest.json').read_text())
minimum=float('inf')
for p in manifest['parts']:
    mesh=trimesh.load_mesh(A/p['file'])
    assert mesh.is_watertight and mesh.is_convex and mesh.volume>0,p['file']
    minimum=min(minimum,MultiPoint(mesh.vertices[:,:2]*1000).convex_hull.distance(Point(0,0)))
assert minimum >= (s.THREAD_MAJOR_DIAMETER+s.COLLISION_DIAMETRAL_CLEARANCE)/2 - 2e-5
shank=pin.section([0,0,1],[0,0,(s.PIN_THREAD_LENGTH+s.PIN_UNDERHEAD_LENGTH)/2000])
assert shank is not None
shank_d=float(np.ptp(shank.vertices[:,0])*1000)
pv=pin.vertices*1000;bv=body.vertices*1000
tab_base=s.PIN_UNDERHEAD_LENGTH+s.COLLAR_THICKNESS
hole_center=tab_base+s.TAB_HOLE_HEIGHT
cross_section=pin.section([0,1,0],[0,0,0])
hole_loops=[loop*1000 for loop in cross_section.discrete
            if np.ptp(loop[:,0])*1000 < s.TAB_CROSS_HOLE_DIAMETER+1
            and abs(loop[:,2].mean()*1000-hole_center)<s.TAB_CROSS_HOLE_DIAMETER]
assert len(hole_loops)==1,'cross-hole loop not isolated'
cross_d=float(np.ptp(hole_loops[0][:,0]))
tab_t=float(np.ptp(pv[pv[:,2]>tab_base+1,1]))
collar=pin.section([0,0,1],[0,0,(s.PIN_UNDERHEAD_LENGTH+s.COLLAR_THICKNESS/2)/1000])
collar_d=float(np.ptp(collar.vertices[:,0])*1000)
radial=np.linalg.norm(pv[:,:2],axis=1)
thread_end=float(pv[(abs(radial-s.THREAD_MAJOR_DIAMETER/2)<1e-3)&(pv[:,2]<s.PIN_UNDERHEAD_LENGTH/2),2].max())
face_points=bv[(bv[:,0]>=0)&(np.linalg.norm(bv[:,:2],axis=1)<s.EAR_OUTER_DIAMETER/2-s.EAR_EDGE_ROUND+.01)]
ear_span=float(np.ptp(face_points[:,2]))
crown_section=body.section([1,0,0],[(s.BOW_CROWN_CENTER_X-s.BOW_CROWN_RADIUS)/1000,0,0])
rod_d=float(np.ptp(crown_section.vertices[:,1])*1000)
# Read joint limits from existing MJCF and pad geometry at the JSON TCP; no model load.
xml=ET.parse(ROOT/'assets/propeller_cardinal/arm/source.xml').getroot()
limits={n:[float(v) for v in xml.find(f'.//joint[@name="{n}"]').get('range').split()] for n in ['gripper_left_joint','gripper_right_joint']}
travel=sum(v[1] for v in limits.values())*1000
config=json.loads((ROOT/'viscous_arm.json').read_text())
urdf=ROOT/Path(config['urdf']);meshdir=urdf.resolve().parent/'meshes'
pad_slices=[]
for n in [42,43]:
    m=trimesh.load_mesh(meshdir/f'part_{n:03d}.stl')
    section=m.section([1,0,0],[config['tcp'][0],0,0])
    pad_slices.append([float(section.vertices[:,1].min()*1000),float(section.vertices[:,1].max()*1000)])
closed_gap=pad_slices[1][0]-pad_slices[0][1]
grip={'source_mjcf':'assets/propeller_cardinal/arm/source.xml','source_config':'viscous_arm.json',
      'joint_ranges_m':limits,'nominal_maximum_opening_mm':travel,
      'pad_closed_gap_at_tcp_mm':closed_gap,'pad_maximum_opening_at_tcp_mm':travel+closed_gap,
      'method':'read XML prismatic limits; section original CAD pad meshes 042/043 at JSON TCP x; no simulation',
      'features_mm':{'pin_shank':shank_d,'thread_major':s.THREAD_MAJOR_DIAMETER,'collar':s.COLLAR_DIAMETER,
                     'tab_thickness':s.TAB_THICKNESS,'tab_width':s.TAB_WIDTH,'bow_stock':s.BOW_ROD_DIAMETER,
                     'ear_boss':s.EAR_OUTER_DIAMETER,'ear_thickness':s.EAR_THICKNESS,
                     'body_maximum_bow_width':float(body.extents[2]*1000),'body_overall_length':float(body.extents[0]*1000)}}
(RUN/'grip_check.json').write_text(json.dumps(grip,indent=2)+'\n')
measurements=[
 ('m283',7.92,cross_d,'Cross hole: X extent of inner closed loop in pin mesh section Y=0.'),
 ('m284',27.68,collar_d,'Collar OD: X extent of mesh section halfway through collar thickness.'),
 ('m285',9.19,tab_t,'Flat tab thickness: Y extent of mesh vertices above the collar.'),
 ('m286',24.05,float(pin.bounds[1,2]*1000-s.PIN_UNDERHEAD_LENGTH-s.COLLAR_THICKNESS),'Tab height above collar top; underside-to-tip alternative is 28.05 mm.'),
 ('m287',22.23,thread_end,'Maximum Z of full-major-radius thread vertices below mid-shank, measured from tip Z=0.'),
 ('m290',79.85,ear_span,'Z extent of forward annular seating-face mesh vertices, excluding bow.'),
 ('m291',19.41,rod_d,'Y extent of mesh section at the leftmost crown centerline station.'),
 ('m292',40.07,float(body.extents[1]*1000),'Boss OD: body Y extent; axis Z.'),
 ('m293',19.69,shank_d,'Exposed shank diameter from mesh section Z halfway along plain shank, X extent.'),
 ('m294',118.71,float(body.extents[0]*1000),'Full body X bounding box: crown to forward ear edge.'),
 ('m295',98.50,float(body.extents[2]*1000),'Maximum body Z bounding box across bow, not assembled pin/head.')]
out={'reloaded_exported_meshes_valid':True,'convex_parts':len(manifest['parts']),
     'minimum_collision_axis_distance_mm_after_stl_reload':minimum,
     'guaranteed_collision_passage_diameter_mm':minimum*2,
     'measured_shank_section_mm':shank_d,
     'measurements':[{'photo':i,'photo_mm':p,'model_mm':m,'difference_mm':m-p,'method':method} for i,p,m,method in measurements]}
(RUN/'static_checks.json').write_text(json.dumps(out,indent=2)+'\n')
lines=['# Dimension check','',
 'All 14 originals m283–m296 were opened before modeling. Displays m283, m286 and m287 were read after mentally rotating 180°; m295 was also saved rotated for inspection. Values below are mm. Differences are model minus display, conditional on the interpretation; matching a chosen input is not independent validation. Mesh extents/sections use reloaded metre STLs converted to mm; small differences are polygon/float32 error.','',
 '| Photo | Display | Model | Δ | Measurement on model | Photo interpretation and confidence |',
 '|---|---:|---:|---:|---|---|']
calls={
 'm283':'7.92. Inside jaws are in the cross hole. Disagree with tab thickness; moderate/high confidence in feature, moderate in contact.',
 'm284':'27.68. Outside jaws span the collar/head envelope. Collar OD is the working interpretation; tab width cannot be separated confidently. Moderate.',
 'm285':'9.19. Outside jaws clamp the narrow edge of the tab, not its cross hole. High.',
 'm286':'24.05. Axis-wise tab height, collar top to tip. Left contact is partly hidden; if it is under the collar, model is 4.00 too long for that reading. Moderate/low.',
 'm287':'22.23. Jaws run from threaded tip to runout. Disagree with shank diameter. High.',
 'm290':'79.85. Outside ear span is provisional: the right jaw may land on the collar lip. If it includes collar thickness, true ear span would be 75.85 with the current 4 mm collar. Moderate/low.',
 'm291':'19.41. Bow rod diameter at crown. High.',
 'm292':'40.07. Ear boss OD viewed along pin axis; matches jaw locations. High.',
 'm293':'19.69. Jaws straddle the exposed pin shank between ears (see inspect_m293.png). Disagree with ear thickness or bow diameter. Moderate/high; remeasure loose pin.',
 'm294':'118.71. Crown outside to forward ear outside, not inside length. High.',
 'm295':'98.50, final digit reads 0 after 90° rotation. Bow maximum outside width, high confidence in reading, moderate in tangent alignment.'}
for i,p,m,method in measurements:lines.append(f'| {i} | {p:.2f} | {m:.4f} | {m-p:+.4f} | {method} | {calls[i]} |')
lines += ['', 'The nominal 7/8-inch interpretation would make m293 22.225 mm, a +2.535 mm discrepancy; m287 does not measure diameter. The model therefore uses the observed 19.69 mm shank and a provisional nominal 20 mm thread. Ear thickness is **not measured**: the 19.351 mm value follows the table gap and provisional outside span.', '',
 'Extra static checks: both visual STLs are connected, closed and outward oriented; all 212 collision STLs are convex and closed. The reloaded collision set guarantees a circular passage of at least '+f'{2*minimum:.5f} mm'+'. STL has no smooth-shading attribute: companion OBJ files contain explicit split corner normals; Final CPU renders interpolate smooth corner normals with a 38° crease threshold.', '',
 'Body origin is at the pin-axis midpoint; clear ear +Z, tapped ear -Z, bow -X. Pin origin is its flat threaded tip; +Z points toward the head. Seated pin translation is (0, 0, -39.925) mm. A minor-diameter visual bore and major-diameter plain pin necessarily overlap at the tapped ear: this is the requested thread simplification. Collision geometry uses the enlarged bore instead.']
(RUN/'dimension_check.md').write_text('\n'.join(lines)+'\n')
print(json.dumps(out,indent=2));print(json.dumps(grip,indent=2))
