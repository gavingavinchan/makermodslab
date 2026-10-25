# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Exact corrected profile, isolated from bench/MuJoCo dependencies.

Equations copied from ../propeller_corrected_scene.py. Coordinates here are
relative to the fixed shaft centre, with no scale or shape change.
"""
import numpy as np
RADIUS, ROOT_RADIUS, PITCH = .480, .085, .754
BENCH_CENTER = np.array([-.42,-.12,1.29])
def section(t):
    r=ROOT_RADIUS+(RADIUS-ROOT_RADIUS)*t
    w=(.055+.270*t-.055*t*t)*np.sqrt(max(0.,1-t**4))
    return r,w,.12*np.sin(np.pi*t),.015*t,(.012*(1-t)+.002)*np.sqrt(max(0.,1-t**4))
def blade_point(blade,t,u,h=0):
    r,w,skew,rake,thick=section(t);theta=skew+u*w/r
    y=rake-PITCH/(2*np.pi)*theta+.0035*np.sin(np.pi*t)*(1-u*u)+h*thick*np.sqrt(max(0.,1-u*u))
    angle=blade*np.pi/2+theta
    return np.array([r*np.cos(angle),y,r*np.sin(angle)])
def surface(t,u=0):
    p=blade_point(0,t,u,1);eps=1e-6
    dt=blade_point(0,t+eps,u,1)-blade_point(0,t-eps,u,1)
    du=blade_point(0,t,u+eps,1)-blade_point(0,t,u-eps,1)
    n=np.cross(du,dt);n/=np.linalg.norm(n)
    return p,n

def orientation(z,x):
    z=np.array(z,dtype=float);z/=np.linalg.norm(z)
    x=np.array(x,dtype=float);x-=x.dot(z)*z;x/=np.linalg.norm(x)
    return np.column_stack((x,np.cross(z,x),z))
