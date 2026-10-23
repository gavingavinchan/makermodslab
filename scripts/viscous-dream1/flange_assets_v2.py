"""Deterministic, local mesh/texture authoring for the subsea fixture.
OBJ normals are explicit: turned surfaces smooth, machined faces/hexes crisp.
Visual detail is independent of convex clearance/contact geometry.
"""
from pathlib import Path
import numpy as np
import trimesh
from PIL import Image, ImageFilter
from shapely.geometry import Polygon, Point

ASSETS=Path(__file__).resolve().parent/'assets/flange'

class Obj:
    def __init__(self): self.v=[];self.n=[];self.uv=[];self.f=[]
    def triangle(self,points,normals,uv=None):
        start=len(self.v)
        self.v.extend(points);self.n.extend(normals)
        self.uv.extend(uv if uv is not None else [[p[0]/.34+.5,p[1]/.34+.5] for p in points])
        self.f.append([start+i+1 for i in range(3)])
    def quad(self,p,n,uv=None):
        for inds in ((0,1,2),(0,2,3)):
            self.triangle([p[i] for i in inds],[n[i] for i in inds],None if uv is None else [uv[i] for i in inds])
    def cap(self,poly,z,sign):
        v,f=trimesh.creation.triangulate_polygon(poly,engine='earcut')
        for face in f:
            ids=face if sign>0 else face[::-1]
            self.triangle([[*v[i],z] for i in ids],[[0,0,sign]]*3)
    def save(self,name):
        lines=['# Local procedural asset: explicit split normals and UVs']
        lines += ['v '+' '.join(f'{c:.8g}' for c in p) for p in self.v]
        lines += ['vt '+' '.join(f'{c:.8g}' for c in p) for p in self.uv]
        lines += ['vn '+' '.join(f'{c:.8g}' for c in p) for p in self.n]
        lines += ['f '+' '.join(f'{i}/{i}/{i}' for i in f) for f in self.f]
        (ASSETS/name).write_text('\n'.join(lines)+'\n')

def circle(r,n=128,center=(0,0)):
    a=np.arange(n)*2*np.pi/n
    return np.c_[r*np.cos(a)+center[0],r*np.sin(a)+center[1]]

def lathe(name,profile,n=128):
    """Closed radial cross-section [(radius,z),...] revolved around local z."""
    out=Obj()
    for (r0,z0),(r1,z1) in zip(profile,profile[1:]+profile[:1]):
        dr,dz=r1-r0,z1-z0;normal=np.array([dz,-dr]);normal/=np.linalg.norm(normal)
        for i in range(n):
            a,b=np.array([i,i+1])*2*np.pi/n
            p=[[r0*np.cos(a),r0*np.sin(a),z0],[r0*np.cos(b),r0*np.sin(b),z0],
               [r1*np.cos(b),r1*np.sin(b),z1],[r1*np.cos(a),r1*np.sin(a),z1]]
            ns=[[normal[0]*np.cos(t),normal[0]*np.sin(t),normal[1]] for t in (a,b,b,a)]
            # Cylindrical UVs follow the coating along the pipe, without poles.
            out.quad(p,ns,[[i/n,z0],[(i+1)/n,z0],[(i+1)/n,z1],[i/n,z1]])
    out.save(name)

def chamfered_flange():
    out=Obj();centres=[(.125*np.cos(a),.125*np.sin(a)) for a in np.arange(8)*np.pi/4+np.pi/8]
    for z,sign in ((-.018,-1),(.018,1)):
        cap=Polygon(circle(.1635),[circle(.054,96),*[circle(.0185,48,c) for c in centres]])
        out.cap(cap,z,sign)
    for radius,center,inner,n in [(.165,(0,0),False,128),(.0525,(0,0),True,96),*[(.017,c,True,48) for c in centres]]:
        edge=radius+(.0015 if inner else -.0015)
        rings=[(edge,-.018),(radius,-.0165),(radius,.0165),(edge,.018)]
        for (r0,z0),(r1,z1) in zip(rings,rings[1:]):
            nr,nz=z1-z0,-(r1-r0);norm=np.hypot(nr,nz);nr/=norm;nz/=norm
            if inner:nr=-nr;nz=-nz
            for i in range(n):
                a,b=np.array([i,i+1])*2*np.pi/n
                p=[[center[0]+r*np.cos(t),center[1]+r*np.sin(t),z] for r,z,t in ((r0,z0,a),(r0,z0,b),(r1,z1,b),(r1,z1,a))]
                normals=[[nr*np.cos(t),nr*np.sin(t),nz] for t in (a,b,b,a)]
                if inner:p=p[::-1];normals=normals[::-1]
                out.quad(p,normals)
    out.save('flange_v2.obj')

def hex_part(name,half,bore=0,offset=0):
    # Trim each vertical corner, then bevel both ends. AF remains exactly 46 mm.
    out=Obj();r=.046/np.sqrt(3);hexagon=circle(r,6)
    perimeter=[]
    for i,v in enumerate(hexagon):
        perimeter.extend([v*.94+hexagon[(i-1)%6]*.06,v*.94+hexagon[(i+1)%6]*.06])
    perimeter=np.asarray(perimeter);rings=[(perimeter*.91,-half),(perimeter,-half+.002),(perimeter,half-.002),(perimeter*.91,half)]
    for (pa,za),(pb,zb) in zip(rings,rings[1:]):
        for i in range(12):
            j=(i+1)%12
            p=np.array([[*pa[i],za+offset],[*pa[j],za+offset],[*pb[j],zb+offset],[*pb[i],zb+offset]])
            normal=np.cross(p[1]-p[0],p[2]-p[0]);normal/=np.linalg.norm(normal)
            out.quad(p.tolist(),[normal.tolist()]*4)
    for z,sign in ((-half,-1),(half,1)):
        out.cap(Polygon(perimeter*.91,[circle(bore+.001,64)] if bore else []),z+offset,sign)
    if bore:
        for (r0,z0),(r1,z1) in zip([(bore+.001,-half),(bore,-half+.001),(bore,half-.001)],[(bore,-half+.001),(bore,half-.001),(bore+.001,half)]):
            nr,nz=-(z1-z0),r1-r0;length=np.hypot(nr,nz);nr/=length;nz/=length
            for i in range(64):
                a,b=np.array([i,i+1])*2*np.pi/64
                p=[[r*np.cos(t),r*np.sin(t),z+offset] for r,z,t in ((r0,z0,a),(r1,z1,a),(r1,z1,b),(r0,z0,b))]
                out.quad(p,[[nr*np.cos(t),nr*np.sin(t),nz] for t in (a,a,b,b)])
    out.save(name)

def textures():
    rng=np.random.default_rng(20261023);size=1024
    def noise(sigma):
        return np.asarray(Image.fromarray(rng.integers(0,256,(size,size),dtype=np.uint8)).filter(ImageFilter.GaussianBlur(sigma)),float)-127.5
    fine=noise(.5);broad=noise(12);slow=noise(40)
    for name,base in [('epoxy_grey',(153,159,154)),('pipe_fbe',(90,47,34)),('steel_worn',(114,121,120)),('ptfe_blue',(26,91,143))]:
        rgb=np.zeros((size,size,3))+base
        rgb += (fine*.075+broad*.55+slow*1.2)[...,None]
        if name in ('epoxy_grey','pipe_fbe'):
            # Sparse scratches/chips, with exposed primer/steel; no dirt on bores.
            im=Image.fromarray(np.uint8(np.clip(rgb,0,255)))
            from PIL import ImageDraw
            d=ImageDraw.Draw(im)
            for _ in range(110):
                x,y=rng.integers(0,size,2);length=int(rng.integers(2,16))
                d.line((x,y,x+length,y+int(rng.integers(-2,3))),fill=(92,83,67) if name=='epoxy_grey' else (115,100,78),width=1)
        else:im=Image.fromarray(np.uint8(np.clip(rgb,0,255)))
        im.save(ASSETS/(name+'.png'))

def generate():
    ASSETS.mkdir(parents=True,exist_ok=True)
    # Preserve the loose-part contact envelopes and inertial assignments from v1.
    # Recreate their assets only if absent; visual chamfers do not alter grasp physics.
    for i in range(48):
        path=ASSETS/f'nut_cell_{i:02d}.stl'
        if path.exists():continue
        a,b=2*np.pi*np.array([i,i+1])/48
        def radius(t):return .023/np.cos(t-(np.floor(t/(np.pi/3))+.5)*np.pi/3)
        polygon=Polygon([[.0155*np.cos(a),.0155*np.sin(a)],[radius(a)*np.cos(a),radius(a)*np.sin(a)],[radius(b)*np.cos(b),radius(b)*np.sin(b)],[.0155*np.cos(b),.0155*np.sin(b)]])
        mesh=trimesh.creation.extrude_polygon(polygon,.026,engine='earcut');mesh.apply_translation([0,0,-.013]);mesh.export(path)
    if not (ASSETS/'head.stl').exists():
        mesh=trimesh.creation.cylinder(radius=.046/np.sqrt(3),height=.020,sections=6);mesh.apply_translation([0,0,-.010]);mesh.export(ASSETS/'head.stl')
    chamfered_flange();hex_part('nut_v2.obj',.013,.0155);hex_part('head_v2.obj',.010,offset=-.010)
    # Pipe metal extends .6 m; 3 mm coating stops 50 mm beyond the hub weld.
    lathe('pipe_v2.obj',[(.0603,-.300),(.0603,.299),(.0593,.300),(.0525,.300),(.0525,-.300)])
    lathe('coating_v2.obj',[(.0630,-.249),(.0633,-.246),(.0633,.299),(.0623,.300),(.0603,.300),(.0603,-.249)])
    lathe('hub_v2.obj',[(.092,0),(.091,.008),(.086,.018),(.063,.070),(.0603,.080),(.0525,.080),(.0525,0)])
    lathe('weld_v2.obj',[(.0603,-.004),(.0625,-.001),(.063,0),(.0625,.001),(.0603,.004),(.060,.004),(.060,-.004)])
    lathe('raised_face_v2.obj',[(.099,-.001),(.100,0),(.100,.001),(.099,.002),(.0525,.002),(.0525,-.001)])
    lathe('gasket_v2.obj',[(.098,-.001),(.098,.001),(.0525,.001),(.0525,-.001)])
    # Lightweight, open-bore convex collision cells; all eight holes stay open.
    flange=Polygon(circle(.165,64),[circle(.0525,48),*[circle(.017,24,(.125*np.cos(a),.125*np.sin(a))) for a in np.arange(8)*np.pi/4+np.pi/8]])
    vertices,faces=trimesh.creation.triangulate_polygon(flange,engine='earcut')
    directory=ASSETS/'v2_collision';directory.mkdir(exist_ok=True)
    for old in directory.glob('*.stl'):old.unlink()
    for i,f in enumerate(faces):
        if Polygon(vertices[f]).area<1e-12:continue
        mesh=trimesh.creation.extrude_polygon(Polygon(vertices[f]),.033,engine='earcut');mesh.apply_translation([0,0,-.0165]);mesh.export(directory/f'flange_{i:03d}.stl')
    textures()
    (ASSETS/'v2.version').write_text('2\n')
    print('Generated',len(faces),'convex flange cells and smooth/chamfered visual meshes',flush=True)

if __name__=='__main__':generate()
