# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Re-tessellate vendored SIMMAN IGES files using cadquery-ocp 7.9.3.1.1.

Optional asset preparation only; rebuilding the scene uses the supplied JSON
and does not require OpenCascade. Analytic surface normals survive patch seams.
"""
from pathlib import Path
from OCP.IGESControl import IGESControl_Reader
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.BRepLib import BRepLib_ToolTriangulatedShape
from OCP.TopLoc import TopLoc_Location
import json
root=Path(__file__).parent/'assets/ship'
for name in ['KCS_hull_SVA','KCS_full-scale-rudder']:
 reader=IGESControl_Reader();status=reader.ReadFile(str(root/f'{name}.igs'));print('read',name,status,flush=True);reader.TransferRoots();shape=reader.OneShape()
 box=Bnd_Box();BRepBndLib.Add_s(shape,box);bounds=box.Get();print('bounds',name,bounds,flush=True)
 mesher=BRepMesh_IncrementalMesh(shape,20,False,.12,True);mesher.Perform()
 exp=TopExp_Explorer(shape,TopAbs_FACE);verts=[];faces=[];parts=[];normals=[]
 while exp.More():
  face=TopoDS.Face_s(exp.Current());loc=TopLoc_Location();tri=BRep_Tool.Triangulation_s(face,loc)
  if tri is not None:
   BRepLib_ToolTriangulatedShape.ComputeNormals_s(face,tri)
   offset=len(verts)
   for i in range(1,tri.NbNodes()+1):
    v=tri.Node(i).Transformed(loc.Transformation());verts.append([v.X(),v.Y(),v.Z()])
    n=tri.Normal(i).Transformed(loc.Transformation());sgn=-1 if face.Orientation()==TopAbs_REVERSED else 1;normals.append([sgn*n.X(),sgn*n.Y(),sgn*n.Z()])
   first=len(faces)
   for i in range(1,tri.NbTriangles()+1):
    indices=list(tri.Triangle(i).Get());
    if face.Orientation()==TopAbs_REVERSED:indices.reverse()
    faces.append([offset+j-1 for j in indices])
   parts.append([first,len(faces)])
  exp.Next()
 out=dict(name=name,vertices=verts,normals=normals,faces=faces,face_parts=parts,bounds=list(bounds),tessellation=dict(linear_deflection_source_units=20,angular_deflection_rad=.12))
 (root/f'{name}.json').write_text(json.dumps(out,separators=(',',':')))
 print('mesh',len(verts),len(faces),'parts',len(parts),flush=True)
