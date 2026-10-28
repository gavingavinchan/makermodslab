"""Deterministic local shackle authoring, following flange_assets_v2.py.

All geometry is authored in mm, exported in metres. No simulator, network,
model service or hardware imports. Run with ~/dreamtrial/.venv/bin/python.
STL stores face normals only; companion OBJs preserve split/smooth normals.
"""
from pathlib import Path
import ast
import json
import numpy as np
import trimesh
from shapely.geometry import Polygon

# Edit a single line here, then regenerate. ESTIMATE means not directly measured.
BOW_ROD_DIAMETER = 19.41  # MEASURED m291, outside jaws across bow stock.
BODY_OUTER_LENGTH = 118.71  # MEASURED m294, crown to forward edge of ear.
BOW_OUTER_WIDTH = 98.50  # MEASURED m295, rotated display reads 98.50.
EAR_OUTER_DIAMETER = 40.07  # MEASURED m292, ear boss viewed along pin axis.
EAR_OUTER_SPAN = 79.85  # PROVISIONAL m290, outside ear faces; collar contact obscured.
EAR_INNER_GAP = 1.62 * 25.4  # ESTIMATE Suncor S0119-0020 dimension C; remeasure.
EAR_THICKNESS = (EAR_OUTER_SPAN - EAR_INNER_GAP) / 2  # ESTIMATE derived symmetric ears; not m293.
EAR_EDGE_ROUND = 5.0  # ESTIMATE radial roll-in from boss OD to flat face; elliptical barrel profile, m289/m296.
CLEAR_BORE_DIAMETER = 20.60  # ESTIMATE fit allowance for nominal 20 mm stainless pin; no inside reading.
THREAD_MAJOR_DIAMETER = 20.0  # ESTIMATE nominal M20 from m293 19.69 shank, Suncor matching diameters.
THREAD_PITCH = 2.50  # ESTIMATE m286/m287/m288 crest counting; M20x2.5 favored, not identified.
TAPPED_BORE_DIAMETER = THREAD_MAJOR_DIAMETER - 1.082532 * THREAD_PITCH  # ESTIMATE basic metric internal minor diameter; plain visual bore.
BORE_MOUTH_CHAMFER = 0.60  # ESTIMATE m289 bore lips; radial and axial extent.
PIN_SHANK_DIAMETER = 19.69  # PROVISIONAL MEASURED m293, jaws straddle exposed pin shank.
PIN_UNDERHEAD_LENGTH = EAR_OUTER_SPAN  # ESTIMATE flush seated tip, m293; m290 span assumption.
PIN_THREAD_LENGTH = 22.23  # MEASURED m287, outside jaws along thread from tip to runout.
PIN_TIP_CHAMFER = 0.70  # ESTIMATE m286/m288 lead-in, leaves flat standing end.
COLLAR_DIAMETER = 27.68  # PROVISIONAL MEASURED m284, jaws span collar; tab width similar.
COLLAR_THICKNESS = 4.0  # ESTIMATE m288 silhouette, not independently measured.
COLLAR_EDGE_ROUND = 0.80  # ESTIMATE m286/m288 rounded turned/forged edge.
TAB_HEIGHT = 24.05  # PROVISIONAL MEASURED m286, collar top to tab tip; contact partly hidden.
TAB_WIDTH = COLLAR_DIAMETER  # ESTIMATE m284/m288 tab roughly as wide as collar; no separate reading.
TAB_THICKNESS = 9.19  # MEASURED m285, outside jaws across narrow tab edge.
TAB_CROSS_HOLE_DIAMETER = 7.92  # MEASURED m283, inside jaws in cross hole; upside-down display.
TAB_HOLE_HEIGHT = 13.0  # ESTIMATE m286/m288 cross-hole center above collar top.
TAB_BASE_WIDTH_FRACTION = 0.68  # ESTIMATE m288 forged tab taper relative to maximum width.
TAB_SHOULDER_HEIGHT_FRACTION = 0.75  # ESTIMATE m288 transition from tapered sides to round crown.
BOW_NECK_LENGTH = 22.0  # ESTIMATE m289/m296 ear-to-curve transition, from pin axis along -X.
BOW_BLEND_CONTROL_LENGTH = 13.0  # ESTIMATE m289/m296 planform spline handles; shape only.
COLLISION_DIAMETRAL_CLEARANCE = 0.60  # ESTIMATE deliberate proxy clearance over thread major, both bores.
COLLISION_NECK_END_X = -max(CLEAR_BORE_DIAMETER, TAPPED_BORE_DIAMETER, THREAD_MAJOR_DIAMETER + COLLISION_DIAMETRAL_CLEARANCE) / 2 - 0.70  # ESTIMATE conservative cutback avoids pin-axis obstruction.
PIN_PLAIN_SHANK_LENGTH = PIN_UNDERHEAD_LENGTH - PIN_THREAD_LENGTH  # ESTIMATE derived plain axial length, m287 plus provisional seating span.
PIN_TOTAL_LENGTH = PIN_UNDERHEAD_LENGTH + COLLAR_THICKNESS + TAB_HEIGHT  # ESTIMATE derived tip to tab tip.
BODY_INNER_LENGTH = BODY_OUTER_LENGTH - EAR_OUTER_DIAMETER/2 - BOW_ROD_DIAMETER - PIN_SHANK_DIAMETER/2  # ESTIMATE derived crown-inner to pin tangent, m294/m292/m291/m293.
BOW_INNER_WIDTH = BOW_OUTER_WIDTH - 2*BOW_ROD_DIAMETER  # ESTIMATE derived maximum inside bow width, m295/m291.
EAR_CENTER_OFFSET = (EAR_INNER_GAP + EAR_THICKNESS)/2  # ESTIMATE derived ear center offset from pin midpoint.
BOW_CROWN_RADIUS = (BOW_OUTER_WIDTH - BOW_ROD_DIAMETER)/2  # ESTIMATE circular centerline crown radius from m295/m291.
BOW_CROWN_CENTER_X = -(BODY_OUTER_LENGTH-EAR_OUTER_DIAMETER/2-BOW_ROD_DIAMETER/2) + BOW_CROWN_RADIUS  # ESTIMATE derived circular crown center along X.
VISUAL_SECTIONS = 128  # Meshing resolution, dimensionless.
TUBE_SECTIONS = 40  # Meshing resolution, dimensionless.
CROWN_SEGMENTS = 64  # Meshing resolution, dimensionless (even: includes maximum width).
BLEND_SEGMENTS = 20  # Meshing resolution, dimensionless.
COLLISION_EAR_SECTORS = 48  # Convex-cell resolution; tangent inner faces preserve circular clearance.

ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / 'assets/shackle'
RUN = ROOT / 'runs/shackle_model'


def cylinder(diameter, low, high, sections=VISUAL_SECTIONS):
    mesh = trimesh.creation.cylinder(radius=diameter/2, height=high-low, sections=sections)
    mesh.apply_translation([0, 0, (low+high)/2])
    return mesh


def lathe(profile, z=0):
    mesh = trimesh.creation.revolve(np.asarray(profile), sections=VISUAL_SECTIONS)
    mesh.apply_translation([0, 0, z])
    mesh.fix_normals()
    return mesh


def ear_profile():
    r, h, e = EAR_OUTER_DIAMETER/2, EAR_THICKNESS/2, EAR_EDGE_ROUND
    # Elliptical barrel roll, rather than an unmeasured straight cylindrical rim.
    points = [(0, -h), (r-e, -h)]
    points += [(r-e+e*np.cos(a), h*np.sin(a)) for a in np.linspace(-np.pi/2, np.pi/2, 33)[1:]]
    points += [(0, h)]
    return points


def bow_path():
    r = (BOW_OUTER_WIDTH-BOW_ROD_DIAMETER)/2
    cx = -(BODY_OUTER_LENGTH-EAR_OUTER_DIAMETER/2-BOW_ROD_DIAMETER/2) + r
    ez = (EAR_INNER_GAP+EAR_THICKNESS)/2
    # Upper branch travels from ear to crown; semicircle returns to lower branch.
    neck_x = np.linspace(COLLISION_NECK_END_X, -BOW_NECK_LENGTH, 7)
    neck = np.c_[neck_x, np.zeros(len(neck_x)), np.full(len(neck_x), ez)]
    controls = np.array([[-BOW_NECK_LENGTH, ez], [-BOW_NECK_LENGTH-BOW_BLEND_CONTROL_LENGTH, ez],
                         [cx+BOW_BLEND_CONTROL_LENGTH, r], [cx, r]])
    t = np.linspace(0, 1, BLEND_SEGMENTS+1)[1:]
    blend = ((1-t)**3)[:, None]*controls[0] + (3*(1-t)**2*t)[:, None]*controls[1] + (3*(1-t)*t*t)[:, None]*controls[2] + (t**3)[:, None]*controls[3]
    top = np.vstack([neck, np.c_[blend[:,0], np.zeros(len(t)), blend[:,1]]])
    a = np.linspace(np.pi/2, 3*np.pi/2, CROWN_SEGMENTS+1)[1:]
    crown = np.c_[cx+r*np.cos(a), np.zeros(len(a)), r*np.sin(a)]
    bottom = top[-2::-1].copy(); bottom[:,2] *= -1
    return np.vstack([top, crown, bottom])


def tube_rings(path):
    tangent = np.gradient(path, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:,None]
    across = np.cross(tangent, [0, 1, 0])
    angles = np.arange(TUBE_SECTIONS)*2*np.pi/TUBE_SECTIONS
    return path[:,None,:] + BOW_ROD_DIAMETER/2*(np.cos(angles)[None,:,None]*np.array([0,1,0]) + np.sin(angles)[None,:,None]*across[:,None,:])


def sweep(rings):
    v = rings.reshape(-1,3).tolist(); f=[]; n = rings.shape[1]
    for j in range(len(rings)-1):
        for i in range(n):
            a=j*n+i; b=j*n+(i+1)%n; c=b+n; d=a+n
            f.extend([[a,b,c],[a,c,d]])
    for j in (0,len(rings)-1):
        center=len(v);v.append(rings[j].mean(axis=0).tolist())
        for i in range(n):f.append([center,j*n+i,j*n+(i+1)%n])
    mesh=trimesh.Trimesh(v,f,process=True);mesh.fix_normals();return mesh


def bore(diameter, z):
    h = EAR_THICKNESS/2; r=diameter/2; c=BORE_MOUTH_CHAMFER
    return lathe([(0,-h-c),(r+c,-h-c),(r+c,-h),(r,-h+c),
                  (r,h-c),(r+c,h),(r+c,h+c),(0,h+c)], z)


def body_mesh():
    ez=(EAR_INNER_GAP+EAR_THICKNESS)/2
    rings=tube_rings(bow_path())
    # Neck sweep stops just short of holes; bosses bridge its closed ends.
    solid=trimesh.boolean.union([sweep(rings),lathe(ear_profile(),ez),lathe(ear_profile(),-ez)],engine='manifold')
    return trimesh.boolean.difference([solid,bore(CLEAR_BORE_DIAMETER,ez),bore(TAPPED_BORE_DIAMETER,-ez)],engine='manifold')


def pin_mesh():
    t=PIN_TIP_CHAMFER; r=THREAD_MAJOR_DIAMETER/2; s=PIN_SHANK_DIAMETER/2
    shaft=lathe([(0,0),(r-t,0),(r,t),(r,PIN_THREAD_LENGTH),
                 (s,PIN_THREAD_LENGTH),(s,PIN_UNDERHEAD_LENGTH+COLLAR_THICKNESS/2),(0,PIN_UNDERHEAD_LENGTH+COLLAR_THICKNESS/2)])
    r=COLLAR_DIAMETER/2; e=COLLAR_EDGE_ROUND; h=COLLAR_THICKNESS
    collar=lathe([(0,0),(r-e,0),(r,e),(r,h-e),(r-e,h),(0,h)],PIN_UNDERHEAD_LENGTH)
    w=TAB_WIDTH; h=TAB_HEIGHT; shoulder=TAB_SHOULDER_HEIGHT_FRACTION*h
    outline=[[-w*TAB_BASE_WIDTH_FRACTION/2,-COLLAR_THICKNESS/2],[-w/2,shoulder]]
    outline += [[w/2*np.cos(a),shoulder+(h-shoulder)*np.sin(a)] for a in np.linspace(np.pi,0,65)[1:]]
    outline += [[w*TAB_BASE_WIDTH_FRACTION/2,-COLLAR_THICKNESS/2]]
    tab=trimesh.creation.extrude_polygon(Polygon(outline),TAB_THICKNESS,engine='earcut')
    # Extruded polygon coordinates (x, tab height, thickness) -> (x,y,z).
    v=tab.vertices.copy();tab.vertices=np.c_[v[:,0],v[:,2]-TAB_THICKNESS/2,v[:,1]+PIN_UNDERHEAD_LENGTH+COLLAR_THICKNESS]
    tab.fix_normals()
    hole=cylinder(TAB_CROSS_HOLE_DIAMETER,-TAB_THICKNESS,TAB_THICKNESS)
    hole.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]))
    hole.apply_translation([0,0,PIN_UNDERHEAD_LENGTH+COLLAR_THICKNESS+TAB_HOLE_HEIGHT])
    solid=trimesh.boolean.union([shaft,collar,tab],engine='manifold')
    return trimesh.boolean.difference([solid,hole],engine='manifold')


def collision_cells():
    rings=tube_rings(bow_path())
    for i in range(len(rings)-1):
        yield f'bow_{i:03d}',trimesh.convex.convex_hull(np.vstack([rings[i],rings[i+1]]))
    ez=(EAR_INNER_GAP+EAR_THICKNESS)/2
    for label,z,visual_d in [('clear',ez,CLEAR_BORE_DIAMETER),('tapped',-ez,TAPPED_BORE_DIAMETER)]:
        n=COLLISION_EAR_SECTORS
        radius=max(visual_d,THREAD_MAJOR_DIAMETER+COLLISION_DIAMETRAL_CLEARANCE)/2
        # A chord on a circumscribed polygon is tangent to the requested bore.
        inner=radius/np.cos(np.pi/n)
        for i in range(n):
            a,b=2*np.pi*np.array([i,i+1])/n
            pts=[]
            for angle in (a,b):
                pts.extend([[rad*np.cos(angle),rad*np.sin(angle),height+z] for rad,height in ear_profile() if rad>inner])
                pts.extend([[inner*np.cos(angle),inner*np.sin(angle),sign*EAR_THICKNESS/2+z] for sign in (-1,1)])
            yield f'ear_{label}_{i:03d}',trimesh.convex.convex_hull(pts)


def save_smooth_obj(mesh,path,crease_degrees=38):
    """Explicit per-corner normals: smooth curved facets, crisp machined edges."""
    mesh.vertex_normals  # cache once
    incident=mesh.vertex_faces; normals=mesh.face_normals
    threshold=np.cos(np.deg2rad(crease_degrees))
    lines=['# metres; procedural shackle; split corner normals']
    lines += ['v '+' '.join(f'{x:.10g}' for x in v) for v in mesh.vertices]
    for fi,face in enumerate(mesh.faces):
        for vi in face:
            faces=incident[vi];faces=faces[faces>=0]
            near=faces[normals[faces]@normals[fi]>=threshold]
            normal=np.sum(normals[near]*mesh.area_faces[near,None],axis=0)
            normal/=np.linalg.norm(normal)
            lines.append('vn '+' '.join(f'{x:.9g}' for x in normal))
    for i,face in enumerate(mesh.faces):lines.append('f '+' '.join(f'{v+1}//{i*3+j+1}' for j,v in enumerate(face)))
    path.write_text('\n'.join(lines)+'\n')


def parameters():
    source=Path(__file__).read_text();lines=source.splitlines(); out={}
    for node in ast.parse(source).body:
        if not isinstance(node,ast.Assign) or len(node.targets)!=1 or not isinstance(node.targets[0],ast.Name):continue
        name=node.targets[0].id
        if not name.isupper() or not isinstance(globals().get(name),(float,int)):continue
        comment=lines[node.lineno-1].partition('#')[2].strip()
        value=globals()[name]
        unit='dimensionless' if name.endswith(('SECTIONS','SEGMENTS','SECTORS','FRACTION')) else 'mm'
        out[name]={'value':value,'unit':unit,'source':comment,
                   'status':'ESTIMATE' if 'ESTIMATE' in comment else 'PROVISIONAL_MEASURED' if 'PROVISIONAL' in comment else 'MEASURED' if 'MEASURED' in comment else 'MESH_SETTING'}
    return out


def generate():
    ASSETS.mkdir(parents=True,exist_ok=True);RUN.mkdir(parents=True,exist_ok=True)
    collision=ASSETS/'collision';collision.mkdir(exist_ok=True)
    meshes={'shackle_body':body_mesh(),'shackle_pin':pin_mesh()};checks={}
    for name,mm in meshes.items():
        mm.fix_normals()
        assert mm.is_watertight and mm.is_winding_consistent and mm.volume>0,(name,'invalid mesh')
        assert len(mm.split())==1,(name,'disconnected mesh')
        m=mm.copy();m.apply_scale(.001);m.export(ASSETS/(name+'.stl'));save_smooth_obj(m,ASSETS/(name+'.obj'))
        checks[name]={'watertight':bool(mm.is_watertight),'consistent_winding':bool(mm.is_winding_consistent),
                      'connected_components':len(mm.split()),'bounds_mm':mm.bounds.tolist(),'volume_mm3':mm.volume,
                      'vertices':len(mm.vertices),'triangles':len(mm.faces)}
    cells=[];min_radial=np.inf
    for name,mm in collision_cells():
        assert mm.is_watertight and mm.is_convex and mm.volume>0,name
        # Exact radial closest distance of each convex cell in XY (not just vertices).
        hull=Polygon(mm.vertices[:,:2]).convex_hull
        from shapely.geometry import Point
        distance=hull.distance(Point(0,0));min_radial=min(min_radial,distance)
        cells.append({'file':f'collision/{name}.stl','convex':True,'min_axis_distance_mm':distance})
        mm.apply_scale(.001);mm.export(collision/(name+'.stl'))
    requested_radius=(THREAD_MAJOR_DIAMETER+COLLISION_DIAMETRAL_CLEARANCE)/2
    assert min_radial>=requested_radius-1e-6,(min_radial,requested_radius)
    checks['collision']={'count':len(cells),'all_convex_watertight':True,'minimum_axis_clear_radius_mm':min_radial,
                         'diametral_clearance_over_thread_major_mm':2*min_radial-THREAD_MAJOR_DIAMETER,
                         'radial_clearance_over_thread_major_mm':min_radial-THREAD_MAJOR_DIAMETER/2}
    (ASSETS/'collision_manifest.json').write_text(json.dumps({'units':'metres','frame':'body frame; preserve mesh vertices and offsets; one geom per file',
        'purpose':'both bores enlarged for plain-major-cylinder pin insertion; tapped thread contact intentionally omitted', 'parts':cells},indent=2)+'\n')
    p={'dimensions':parameters(),'frame':{'parameter_units':'mm','mesh_units':'m','body':'origin pin-axis midpoint; clear +Z, tapped -Z, bow -X; bow plane XZ',
          'pin':'origin threaded tip; +Z toward head; tab broad face XZ, cross-hole axis Y',
          'seated_pin_translation_m':[0,0,(EAR_OUTER_SPAN/2-PIN_UNDERHEAD_LENGTH)*.001]},
       'sources':{'photos':'../../shackle/photos; all 14 photos m283–m296 inspected',
                  'suncor':'https://suncorstainless.com/wp-content/uploads/2022/02/Forged-Shackles-1-17-22-1.pdf',
                  'crosby':'https://kitocrosby.com/wp-content/uploads/2026/04/01_Shackles_IMP_24.pdf'},
       'notes':['Provisional 19–20 mm stainless pattern, manufacturer not identified.',
                'STL is faceted geometry with outward face normals; smooth split-normal OBJ companions supplied.',
                'Plain major-cylinder thread overlaps visual minor bore when seated; collision tapped bore has positive clearance.',
                'No simulation or billed inference calls.']}
    (ASSETS/'shackle_params.json').write_text(json.dumps(p,indent=2)+'\n')
    (RUN/'mesh_validation.json').write_text(json.dumps(checks,indent=2)+'\n')
    # Blender reads these local arrays directly; no STL import add-on dependency.
    (RUN/'render_geometry.json').write_text(json.dumps({'meshes':{n:{'vertices':m.vertices.tolist(),'faces':m.faces.tolist()} for n,m in meshes.items()},
        'seated_translation_mm':[0,0,EAR_OUTER_SPAN/2-PIN_UNDERHEAD_LENGTH]},separators=(',',':')))
    print(json.dumps(checks,indent=2))


if __name__=='__main__':generate()
