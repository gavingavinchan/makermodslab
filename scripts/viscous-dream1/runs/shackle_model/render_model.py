"""Deterministic CPU renderer for exact meshes; no graphics context.

Installed Blender returned zero RGB renders. This fallback uses a depth
buffer, smooth corner normals, orthographic cameras and projected shadows.
It renders geometry; there are no generated-image or inference calls.
"""
from pathlib import Path
import json
import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFilter

RUN=Path(__file__).resolve().parent
data=json.loads((RUN/'render_geometry.json').read_text())
SIZE=1400

def unit(v):
    v=np.asarray(v,dtype=float);return v/np.linalg.norm(v,axis=-1,keepdims=True)

def corners(mesh):
    normals=mesh.face_normals;inc=mesh.vertex_faces;out=np.zeros((len(mesh.faces),3,3))
    for fi,f in enumerate(mesh.faces):
        for k,vi in enumerate(f):
            ids=inc[vi];ids=ids[ids>=0];ids=ids[normals[ids]@normals[fi]>=np.cos(np.deg2rad(38))]
            out[fi,k]=unit(np.sum(normals[ids]*mesh.area_faces[ids,None],axis=0))
    return out

meshes={}
for name,g in data['meshes'].items():
    m=trimesh.Trimesh(g['vertices'],g['faces'],process=False)
    meshes[name]=(m,corners(m))

def rotation_x(a):
    c,s=np.cos(a),np.sin(a);return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def render(name,which,rotation,eye,target,scale,ground,pin_offset=None,pin_spin=0):
    forward=unit(np.array(eye)-target);right=unit(np.cross([0,0,1],forward));up=unit(np.cross(forward,right))
    basis=np.array([right,up,forward]);target=np.asarray(target)
    depth=np.full((SIZE,SIZE),-np.inf);normals=np.zeros((SIZE,SIZE,3))
    shadow=Image.new('L',(SIZE,SIZE));draw=ImageDraw.Draw(shadow)
    def project(v):
        p=(v-target)@basis.T;p[:,0]=SIZE/2+p[:,0]*SIZE/scale;p[:,1]=SIZE/2-p[:,1]*SIZE/scale
        return p
    for mesh_name in which:
        mesh,ns=meshes[mesh_name];v=mesh.vertices.copy();n=ns.copy()
        if mesh_name=='shackle_pin':
            c,s=np.cos(pin_spin),np.sin(pin_spin);spin=np.array([[c,-s,0],[s,c,0],[0,0,1]])
            v=v@spin.T;n=n@spin.T
            v+=np.asarray(pin_offset if pin_offset is not None else data['seated_translation_mm'])
        v=v@rotation.T;n=n@rotation.T;p=project(v)
        if ground is not None:
            light=unit([-1,-1,2]);sv=v-(v[:,2:3]-ground)/light[2]*light
            sp=project(sv)
            for f in mesh.faces:draw.polygon([tuple(q) for q in sp[f,:2]],fill=65)
        for fi,f in enumerate(mesh.faces):
            pp=p[f];nn=n[fi]
            face_n=np.cross(v[f[1]]-v[f[0]],v[f[2]]-v[f[0]])
            if np.dot(face_n,forward)<=0:continue
            xmin=max(0,int(np.floor(pp[:,0].min())));xmax=min(SIZE-1,int(np.ceil(pp[:,0].max())))
            ymin=max(0,int(np.floor(pp[:,1].min())));ymax=min(SIZE-1,int(np.ceil(pp[:,1].max())))
            if xmin>xmax or ymin>ymax:continue
            den=(pp[1,1]-pp[2,1])*(pp[0,0]-pp[2,0])+(pp[2,0]-pp[1,0])*(pp[0,1]-pp[2,1])
            if abs(den)<1e-10:continue
            yy,xx=np.mgrid[ymin:ymax+1,xmin:xmax+1];xx=xx+.5;yy=yy+.5
            a=((pp[1,1]-pp[2,1])*(xx-pp[2,0])+(pp[2,0]-pp[1,0])*(yy-pp[2,1]))/den
            b=((pp[2,1]-pp[0,1])*(xx-pp[2,0])+(pp[0,0]-pp[2,0])*(yy-pp[2,1]))/den;c=1-a-b
            z=a*pp[0,2]+b*pp[1,2]+c*pp[2,2]
            crop=depth[ymin:ymax+1,xmin:xmax+1];mask=(a>=-1e-8)&(b>=-1e-8)&(c>=-1e-8)&(z>crop)
            if not mask.any():continue
            crop[mask]=z[mask]
            ns_crop=normals[ymin:ymax+1,xmin:xmax+1]
            ns_crop[mask]=(a[...,None]*nn[0]+b[...,None]*nn[1]+c[...,None]*nn[2])[mask]
    mask=np.isfinite(depth);n=unit(normals[mask])
    light1=unit([-.5,-.7,1]);light2=unit([.8,.2,.5]);light3=unit([-.4,.8,.4])
    shade=.19+.34*np.maximum(n@light1,0)+.21*np.maximum(n@light2,0)+.18*np.maximum(n@light3,0)
    spec=.38*np.maximum(n@unit(light1+forward),0)**20+.25*np.maximum(n@unit(light2+forward),0)**35
    fresnel=.10*(1-np.clip(n@forward,0,1))**3
    steel=(shade+spec+fresnel)[:,None]*np.array([.89,.93,.97])
    bg=np.full((SIZE,SIZE,3),.79)
    if ground is not None:bg-=np.asarray(shadow.filter(ImageFilter.GaussianBlur(9)))[:,:,None]/255*.38
    bg[mask]=np.clip(steel,0,1)
    image=Image.fromarray(np.uint8(np.clip(bg,0,1)*255)).resize((1000,1000),Image.Resampling.LANCZOS)
    image.save(RUN/(name+'.png'));print(name,flush=True)

I=np.eye(3)
render('assembled',['shackle_body','shackle_pin'],I,(150,-280,160),(-36,0,7),177,-50)
render('exploded',['shackle_body','shackle_pin'],I,(140,-290,165),(-28,0,50),255,-50,
       np.array(data['seated_translation_mm'])+[0,0,92])
render('render_m289',['shackle_body'],rotation_x(np.pi/2),(85,-210,270),(-40,0,0),157,-20.1)
render('render_m296',['shackle_body','shackle_pin'],rotation_x(-np.pi/2),(-10,-120,360),(-40,8,0),160,-20.1,pin_spin=np.pi/2)
render('render_m288',['shackle_pin'],I,(40,-290,132),(0,0,53),142,0,[0,0,0])
render('frame_reference',['shackle_body','shackle_pin'],I,(0,-400,0),(-38,0,10),170,None)
