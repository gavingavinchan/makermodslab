# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Raster diagnostic of measured coverage using existing Pillow dependencies."""
import argparse
import json
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageFilter
from propeller_edge_demo import OUT
from propeller_edge_geometry import boundary


def render(out):
    data=np.load(out/'coverage_mask.npz');metrics=json.loads((out/'coverage_metrics.json').read_text())
    points=data['surface_xyz_m'][:,[0,2]];rim=boundary();low=rim.min(axis=0);high=rim.max(axis=0)
    scale=min(1400/(high[0]-low[0]),680/(high[1]-low[1]))
    def project(p):
        return np.column_stack((1500-(p[:,0]-low[0])*scale,780-(p[:,1]-low[1])*scale)).astype(int)
    img=Image.new('RGB',(1600,950),'#172328');draw=ImageDraw.Draw(img)
    draw.polygon([tuple(p) for p in project(rim)],fill='#45684b')
    pixels=project(points)
    for key,color in (('nominal_first_touch_time_s','#939c85'),('first_touch_time_s','#c4874d')):
        selected=np.isfinite(data[key]);arr=np.zeros((950,1600),dtype=np.uint8)
        p=pixels[selected];arr[p[:,1],p[:,0]]=255
        mask=Image.fromarray(arr).filter(ImageFilter.MaxFilter(3))
        img.paste(color,(0,0),mask)
    s=np.load(out/'substeps.npz')['samples'];s=s[s[:,1]==1]
    draw=ImageDraw.Draw(img);draw.line([tuple(p) for p in project(s[::10][:,[2,4]])],fill='white',width=2)
    font=ImageFont.truetype('DejaVuSans.ttf',23)
    texts=['CONTACT-GATED COVERAGE / unchanged corrected blade 0 / no abrasion model',
        f'Actual ellipsoid overlap: {metrics["refined_1mm_geometric_touch_area_m2"]*1e4:.1f} cm2 (copper)',
        f'Nominal 52 mm disk sweep: {metrics["refined_1mm_area_m2"]*1e4:.1f} cm2 (grey)',
        'Green remains untouched; white is observed pad-centre path. Front-view world XZ projection.',
        'Raster colors illustrate quadrature results; use coverage_mask.npz for exact sample timestamps.']
    for i,text in enumerate(texts):draw.text((35,25+i*34 if i<3 else 840+(i-3)*38),text,font=font,fill='white')
    img.save(out/'coverage_map.png')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--name',default='');args=p.parse_args();render(OUT/args.name if args.name else OUT)
