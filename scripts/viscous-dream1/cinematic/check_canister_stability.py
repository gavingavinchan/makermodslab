# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Measure temporal luminance jumps on visible, static-view canister interiors.

ROI centres were independently ray-probed against the saved scene: the upper
region hits rov_base__rov_base_09, the lower region rov_base__rov_base_41.
This is a localized flashing check, not an all-pixel temporal-render certificate.
"""
from pathlib import Path
import argparse,hashlib,json,statistics
from PIL import Image,ImageStat,ImageChops,ImageDraw

HERE=Path(__file__).parent;OUT=HERE/'results'
p=argparse.ArgumentParser();p.add_argument('--scratch',type=Path,required=True);p.add_argument('--last-frame',type=int,default=672);a=p.parse_args()
regions={'upper_canister':(145,165,190,195),'battery_canister':(95,360,135,390)}
values={k:[] for k in regions};differences={k:[] for k in regions};previous={};worst_pixel95={k:0 for k in regions}
for f in range(73,a.last_frame+1):
 with Image.open(a.scratch/f'frame_{f:04d}.png') as im:
  assert im.size==(1280,720)
  for name,box in regions.items():
   crop=im.crop(box).convert('L');values[name].append(ImageStat.Stat(crop).mean[0])
   if name in previous:
    diff=ImageChops.difference(crop,previous[name]);differences[name].append(ImageStat.Stat(diff).mean[0]);hist=diff.histogram();total=sum(hist);cumulative=0
    for level,count in enumerate(hist):
     cumulative+=count
     if cumulative>=total*.95:worst_pixel95[name]=max(worst_pixel95[name],level);break
   previous[name]=crop.copy()
metrics={name:dict(roi_pixels=regions[name],mean_luma_range=[min(v),max(v)],temporal_mean_luma_stddev=statistics.pstdev(v),maximum_frame_mean_luma_step=max(abs(b-a) for a,b in zip(v,v[1:])),maximum_mean_absolute_pixel_difference=max(differences[name]),maximum_95th_percentile_pixel_step=worst_pixel95[name]) for name,v in values.items()}
passed=all(x['maximum_frame_mean_luma_step']<8 and x['maximum_95th_percentile_pixel_step']<24 for x in metrics.values())
report=dict(passed=passed,scene_sha256=hashlib.sha256((OUT/'final_scene.blend').read_bytes()).hexdigest(),frame_range=[73,a.last_frame],samples=a.last_frame-72,regions=metrics,limits='Static working-camera canister interiors only; moving reflections, clamps, scene cuts and other pixels are outside this localized flashing test. Material alpha/transmission checks are separate.')
(OUT/'canister_stability.json').write_text(json.dumps(report,indent=2)+'\n')
selected=[73,(73+a.last_frame)//2,a.last_frame];sheet=Image.new('RGB',(960,382),'#14232a');draw=ImageDraw.Draw(sheet)
for i,f in enumerate(selected):
 with Image.open(a.scratch/f'frame_{f:04d}.png') as im:sheet.paste(im.crop((40,95,360,450)),(i*320,0))
 draw.text((i*320+8,360),f'Frame {f} | {(f-1)/24:.2f}s',fill='white')
sheet.save(OUT/'canister_stability.jpg',quality=94)
print(json.dumps(report,indent=2),flush=True);assert passed
