"""One-command local rebuild of threaded visuals and all review deliverables."""
from pathlib import Path
import json
import os
import sys

os.environ.setdefault('MUJOCO_GL','egl')
sys.dont_write_bytecode=True
RUN=Path(__file__).resolve().parent; ROOT=RUN.parents[1]
sys.path.insert(0,str(ROOT))
import shackle_assets as s
import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from render_threads import render
from validate_threads import validate


def finish():
    checks=json.loads((RUN/'thread_validation.json').read_text())
    reader=imageio.get_reader(str(RUN/'screw_in.mp4'))
    try:
        meta=reader.get_meta_data()
        assert meta['size']==(1280,720) and meta['fps']==30 and abs(meta['duration']-8)<.01
        assert meta['codec']=='h264' and meta['pix_fmt'].startswith('yuv420p')
        meta['nframes']=reader.count_frames()
        assert meta['nframes']==240
        for frame in [0,100,150,239]:
            rgb=reader.get_data(frame)
            assert rgb.shape==(720,1280,3) and rgb.std()>15
        checks['video_encoding']=meta
    finally:reader.close()
    names=[('pin_thread_closeup','Pin: truncated crests, lead-in and runout'),
           ('tapped_ear_below','Tapped ear: helical bore viewed from below'),
           ('pin_halfway','Halfway engaged: phase follows actual pitch'),
           ('pin_seated','Seated: clear ear above, tapped ear below')]
    sheet=Image.new('RGB',(2560,1536),'#1b2835');draw=ImageDraw.Draw(sheet)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',26)
    for i,(name,label) in enumerate(names):
        im=Image.open(RUN/(name+'.png')).convert('RGB')
        assert im.size==(1280,720) and np.asarray(im).std()>15
        x=(i%2)*1280;y=(i//2)*768
        sheet.paste(im,(x,y+48));draw.text((x+20,y+10),label,font=font,fill='white')
    sheet.save(RUN/'review_contact_sheet.png')
    (RUN/'thread_validation.json').write_text(json.dumps(checks,indent=2,allow_nan=False)+'\n')
    report=f'''# Shackle visual threads — MuJoCo check

Default `assets/shackle/shackle_body.obj/.stl` and `shackle_pin.obj/.stl` now have matching, single-start right-hand helices. Original visuals are retained byte-for-byte as `*_plain.obj/.stl`. Clear ear remains a plain bore. No collision geometry was regenerated: SHA-256 matches for all 212 convex cells and their manifest.

Current provisional inputs: **{s.THREAD_MAJOR_DIAMETER:g} mm major diameter, {s.THREAD_PITCH:g} mm pitch, {s.PIN_THREAD_LENGTH:g} mm pin thread length**. Change `THREAD_MAJOR_DIAMETER`, `THREAD_PITCH`, lengths, or `THREAD_START_ANGLE` in `shackle_assets.py`; both mating surfaces derive from them. The 60° axial profile has P/8 truncated external crests, truncated roots at the ISO external minor diameter, a chamfer starting below the root cylinder, and a smooth {s.THREAD_RUNOUT_PITCHES:g}-pitch runout. Female crests use the basic ISO internal minor, plus {s.THREAD_RADIAL_CLEARANCE:g} mm radial running clearance. Both female mouths have 45° countersinks beyond the thread major radius.

| Visual mesh | Triangles | Watertight / connected |
|---|---:|---|
| Body | {checks['body']['faces']:,} | Yes / one component |
| Pin | {checks['pin']['faces']:,} | Yes / one component |

Both are below 150k faces. Helical edges follow profile breakpoints with {s.THREAD_ANGULAR_SECTIONS} angular segments per revolution. OBJ exports independent per-corner normals with a 38° crease threshold: circular surfaces shade smoothly, while 60° crest/flank transitions remain sharp. Normals are calculated in millimetres and verified finite/unit after import. MuJoCo {checks['mujoco_version']} preserves every OBJ normal index (pin: {checks['mujoco_normals']['pin']['obj_normals']:,} normals); STL instead loads {checks['mujoco_normals']['pin']['stl_normals']:,} generated vertex normals. Independent OBJ normal indices suffice; positional vertex duplication is unnecessary. Use OBJ with `smoothnormal="false"`. [Same-lighting OBJ/STL comparison](normals_obj_vs_stl.png).

The new standalone [MJCF](shackle_threads_check.xml) uses two contact-disabled visual meshes and mocap poses, upright with clear ear +Z, tapped ear −Z. Lighting/headlight and quality values are copied from `yam_env.build_xml()`, inherited by the flange scene; stainless specular/shininess/reflectance use flange `pipe_steel` values. All object pixels in the [four stills](review_contact_sheet.png), [comparison](compare_m288.png) and [video](screw_in.mp4) come from MuJoCo/EGL. The video is 1280×720, H.264/yuv420p, 30 fps, 240 frames / 8 seconds. Main view stays three-quarter; the inset renders the same pose at the tapped ear. Motion is entirely kinematic with no physics steps.

**Phase convention:** pin local +Z points from tip toward head; body seated tip is `(0,0,{(s.EAR_OUTER_SPAN/2-s.PIN_UNDERHEAD_LENGTH)*.001:g})` m, spin zero. Crest phase is `q = (z_body-z_seated)/pitch - (azimuth-start_angle)/(2*pi)`, with crests at integer q. `THREAD_START_ANGLE = {s.THREAD_START_ANGLE:g}` radians changes the shared start phase. For remaining axial travel Δ, use `z_tip=z_seated+Δ` and `spin_about_+Z=2*pi*Δ/pitch` with an unwrapped angle. **Pin turned clockwise by θ, viewed from head toward tip, advances downward by θ/(2*pi)*pitch.** These relations and units are recorded in `assets/shackle/shackle_params.json`.

Validation: zero tapped-ear intersection for seven actual exported-solid poses (including fractional turns); the deliberately wrong half-turn control intersects by {checks['wrong_half_turn_positive_control_intersection_mm3']:.2f} mm³. Dense sampling across 23 engagement positions preserves at least {s.THREAD_RADIAL_CLEARANCE:g} mm radial gap. A second 20.2 mm / 2.54 mm / 0.731 rad parameter set also builds watertight helices and preserves the gap. Every engaged video frame satisfies the screw relation, maximum residual {checks['animation']['maximum_engaged_constraint_error_m']:.2g} m. [Full checks](thread_validation.json).

Remaining limitations: dimensions and standard await the real measurements; clean uniform stainless omits the photo's scratches and forging marks. The inherited bow neck reaches 0.0295 mm above the clear ear's seating plane, producing 0.023 mm³ collar-only overlap when fully seated; there is no thread overlap. Frozen collision bores retain their original 20.60 mm passage, so a larger future diameter needs a separate clearance review. Physically incompatible shank/thread/ear-length edits will be caught by engagement validation.

Rebuild assets, MJCF, stills, video, comparison, checks and this report in one command from the repo:

```bash
PYTHONDONTWRITEBYTECODE=1 MUJOCO_GL=egl ~/dreamtrial/.venv/bin/python runs/shackle_threads/rebuild.py
```

No billed calls, episodes, hardware, holder/task scene, or server operations. Writes are confined to `shackle_assets.py`, `assets/shackle/` and `runs/shackle_threads/`.
'''
    (RUN/'shackle_threads_report.md').write_text(report)
    print('Verified encoding; wrote report and review_contact_sheet.png',flush=True)


if __name__=='__main__':
    s.generate()
    render()
    validate()
    finish()
