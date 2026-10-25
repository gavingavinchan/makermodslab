import json,numpy as np
p=json.load(open('/home/gavin/Documents/viscous-arm-model/inspection/parts.json'))
a=np.array(p[20]['cylinders'][1]['p']); axis=np.array(p[20]['cylinders'][1]['axis']);axis/=np.linalg.norm(axis)
b=np.mean([p[i]['center_mm'] for i in (26,27)],axis=0);s=np.mean([p[i]['center_mm'] for i in (29,30)],axis=0)
b-=axis*np.dot(b-a,axis);s-=axis*np.dot(s-a,axis)
u=np.array([0.,0,1]);v=np.cross(axis,u)
r=np.linalg.norm(b-a);L=np.linalg.norm(s-b);e=np.dot(s-a,v);theta=np.arctan2(np.dot(b-a,u),np.dot(b-a,v))
t=np.linspace(-np.pi,np.pi,200001);X=r*np.cos(t);Z=r*np.sin(t);valid=L**2-(e-X)**2>0
z=Z-np.sqrt(np.maximum(0,L**2-(e-X)**2));q=np.dot(s-a,u)-z
jac=r*np.cos(t)+(e-X)*r*np.sin(t)/np.sqrt(np.maximum(1e-10,L**2-(e-X)**2))
# CAD-connected branch near closed angle, jaw travel needed for a 46 mm hex.
valid &= (q>=0)&(q<=24)&(t>theta-1.6)&(t<theta+.001)
print(dict(crank_radius_mm=r,rod_length_mm=L,slider_offset_mm=e,closed_angle_rad=theta,closed_slider_mm=np.dot(s-a,u),max_jac_mm=float(np.max(abs(jac[valid]))),generalized_N=float(5000/np.max(abs(jac[valid])))))
