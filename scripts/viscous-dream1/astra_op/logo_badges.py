# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Visual-only, handed Viscous nameplates and Oct 9 extended V badges.

Meshes retain the source print frame in metres. Link registration is shared with
cinematic/assets/logo_badges/registration.json; reflection never reaches text.
No body, collision, inertial, joint or controller is added or modified.
"""
from pathlib import Path
import json
import xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
ASSETS=Path(__file__).resolve().parent/'logo_assets'

def add(root):
    asset=root.find('asset')
    for el in list(asset):
        if el.get('name','').startswith('badge_'):asset.remove(el)
    for b in root.iter('body'):
        for g in list(b.findall('geom')):
            if g.get('name','').startswith('badge_'):b.remove(g)
    for color,rgba in [('blue','0.176470588235 0.419607843137 1 1'),('white','1 1 1 1')]:
        E.SubElement(asset,'material',name='badge_'+color,rgba=rgba,specular='.14',shininess='.08',reflectance='0',metallic='0',roughness='.62')
    registration=json.loads((ASSETS/'registration.json').read_text())['arms']
    for arm in ('left','right'):
        body=root.find(f".//body[@name='{arm}_forearm_link']")
        t=np.array(registration[arm]['source_preview_arm_to_link_mm'])[:3,3]
        for side,label in [(1,'PosY_output'),(-1,'NegY_bearing')]:
            rot=np.array([[-side,0,0],[0,0,side],[0,1,0]])
            assert np.linalg.det(rot)>0
            q=Rotation.from_matrix(rot).as_quat()[[3,0,1,2]]
            for kind,x,plane in [('nameplate',79,29.5),('v_extended',33.94112,37.5)]:
                pos=np.array([x-160+t[0],side*plane+t[1],58.78775+111+t[2]])*.001
                for color in ('blue','white'):
                    part=f'{kind}_{label}_{color}';mesh='badge_'+part
                    if asset.find(f"mesh[@name='{mesh}']") is None:
                        E.SubElement(asset,'mesh',name=mesh,file=str(ASSETS/(part+'.obj')))
                    E.SubElement(body,'geom',name=f'badge_{arm}_{part}',type='mesh',mesh=mesh,material='badge_'+color,
                                 pos=' '.join(f'{x:.16g}' for x in pos),quat=' '.join(f'{x:.16g}' for x in q),
                                 contype='0',conaffinity='0',mass='0',density='0',group='2')
    return root
