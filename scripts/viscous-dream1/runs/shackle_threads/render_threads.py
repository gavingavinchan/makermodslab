"""Standalone local MuJoCo/EGL inspection and kinematic screw animation.

No scene builders, inference clients, robot modules, servers or physics steps.
Run from the repository with the project venv and MUJOCO_GL=egl.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

os.environ.setdefault('MUJOCO_GL', 'egl')
ROOT = Path(__file__).resolve().parents[2]
RUN = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
import shackle_assets as s
import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps


def numbers(v):
    return ' '.join(f'{x:.12g}' for x in np.asarray(v).ravel())


def add_camera(world, name, eye, target, fovy):
    eye, target = np.array(eye), np.array(target)
    z = eye-target; z /= np.linalg.norm(z)
    x = np.cross([0., 0., 1.], z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    ET.SubElement(world, 'camera', name=name, pos=numbers(eye),
                  xyaxes=numbers(np.r_[x, y]), fovy=str(fovy))


def write_scene():
    root = ET.Element('mujoco', model='Shackle helical visual inspection')
    ET.SubElement(root, 'compiler', angle='radian', meshdir='../../assets/shackle')
    ET.SubElement(root, 'option', gravity='0 0 0')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='1280', offheight='720')
    ET.SubElement(visual, 'quality', shadowsize='4096', offsamples='4')
    ET.SubElement(visual, 'map', znear='.001', zfar='8')
    # Exact values from yam_env.build_xml(), inherited by flange_scene.add_scene.
    ET.SubElement(visual, 'headlight', ambient='.25 .25 .25', diffuse='.35 .35 .35', specular='.1 .1 .1')
    ET.SubElement(root, 'statistic', center='-.025 0 .035', extent='.3')
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'texture', name='background', type='skybox', builtin='gradient',
                  rgb1='.16 .20 .25', rgb2='.055 .075 .10', width='256', height='1024')
    # pipe_steel's flange material response, with neutral stainless base colour.
    ET.SubElement(asset, 'material', name='stainless', rgba='.58 .61 .64 1',
                  specular='.7', shininess='.55', reflectance='.06')
    for label in ('body', 'pin'):
        ET.SubElement(asset, 'mesh', name=f'shackle_{label}',
                      file=f'shackle_{label}.obj', smoothnormal='false')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='-.8 .2 2.7', dir='.3 -.1 -1',
                  diffuse='.55 .53 .50', ambient='.25 .25 .25')
    ET.SubElement(world, 'light', pos='1 -.6 2', dir='-.4 .2 -1',
                  diffuse='.4 .44 .48', castshadow='false')
    for label in ('body', 'pin'):
        pos = [0, 0, 0] if label=='body' else [0, 0, (s.EAR_OUTER_SPAN/2-s.PIN_UNDERHEAD_LENGTH)*.001]
        body = ET.SubElement(world, 'body', name=f'shackle_{label}', mocap='true', pos=numbers(pos))
        ET.SubElement(body, 'geom', name=f'{label}_visual', type='mesh', mesh=f'shackle_{label}',
                      material='stainless', contype='0', conaffinity='0', group='2')
    add_camera(world, 'three_quarter', [.19, -.335, .165], [.055, 0., .055], 34)
    add_camera(world, 'pin_thread_closeup', [.042, -.080, .027], [0., 0., .011], 23)
    add_camera(world, 'tapped_ear_below', [.021, -.026, -.102], [-.001, 0., -.031], 39)
    add_camera(world, 'halfway_detail', [.065, -.125, .023], [-.005, 0., -.009], 33)
    add_camera(world, 'engagement_closeup', [.045, -.085, .010], [0., 0., -.021], 29)
    add_camera(world, 'seated_detail', [.13, -.26, .13], [-.030, 0., .012], 29)
    add_camera(world, 'm288', [.016, -.31, .112], [0., 0., .052], 25)
    ET.indent(root, space='  ')
    path = RUN/'shackle_threads_check.xml'
    path.write_text(ET.tostring(root, encoding='unicode')+'\n')
    return path


def spin_quat(angle):
    return [np.cos(angle/2), 0, 0, np.sin(angle/2)]


def set_pose(model, data, remaining=0., pin_only=False, body_only=False, loose=False):
    bodyid = model.body_mocapid[model.body('shackle_body').id]
    pinid = model.body_mocapid[model.body('shackle_pin').id]
    seated = (s.EAR_OUTER_SPAN/2-s.PIN_UNDERHEAD_LENGTH)*.001
    data.mocap_pos[bodyid] = [10 if pin_only else 0, 0, 0]
    data.mocap_pos[pinid] = [10 if body_only else 0, 0, 0 if loose else seated+remaining]
    data.mocap_quat[pinid] = spin_quat(0 if loose else 2*np.pi*remaining/(s.THREAD_PITCH*.001))
    mujoco.mj_forward(model, data)


def capture(renderer, model, data, camera):
    renderer.update_scene(data, camera=camera)
    return renderer.render().copy()


def comparisons():
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 27)
    small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 21)
    photo = Image.open(next((ROOT/'shackle/photos').glob('*m288.jpg'))).convert('RGB').crop((350,250,610,895))
    model = Image.open(RUN/'render_m288.png').crop((475,55,810,705))
    canvas = Image.new('RGB', (1800,1110), '#e8edf1'); d = ImageDraw.Draw(canvas)
    d.rectangle((0,0,1800,86), fill='#202b38')
    d.text((30,24), 'm288 — original photo crop', font=font, fill='white')
    d.text((930,24), 'Threaded pin — MuJoCo / EGL', font=font, fill='white')
    for im,x in [(photo,25),(model,925)]:
        fit = ImageOps.contain(im,(850,900),Image.Resampling.LANCZOS)
        canvas.paste(fit,(x+(850-fit.width)//2,100+(900-fit.height)//2))
    d.line((900,86,900,1010), fill='#bec4ca', width=2)
    d.text((30,1025),f'Right-hand 60° thread, provisional {s.THREAD_MAJOR_DIAMETER:g} mm major / {s.THREAD_PITCH:g} mm pitch. Real geometry, visual only.',font=small,fill='#263548')
    d.text((30,1064),'Photo and model scaled independently; approximate viewpoint, no perspective calibration.',font=small,fill='#536172')
    canvas.save(RUN/'compare_m288.png')


def animation_pose(t):
    """Insertion is linear until engagement; every subsequent frame obeys lead."""
    seated = (s.EAR_OUTER_SPAN/2-s.PIN_UNDERHEAD_LENGTH)*.001
    initial_tip = s.EAR_OUTER_SPAN*.0005 + .012
    engagement_tip = -s.EAR_INNER_GAP*.0005 + .0008
    engagement_remaining = engagement_tip-seated
    if t < .35:
        return initial_tip-seated, 'approach'
    if t < 2.15:
        u = (t-.35)/1.8; u = u*u*(3-2*u)
        tip = initial_tip+(engagement_tip-initial_tip)*u
        return tip-seated, 'pass through clear ear'
    if t < 7.55:
        u = (t-2.15)/5.4
        return engagement_remaining*(1-u), 'screw into tapped ear'
    return 0., 'seated'


def render(video=True):
    path = write_scene()
    model = mujoco.MjModel.from_xml_path(str(path)); data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=720, width=1280)
    try:
        for name, cam, opts in [
            ('pin_thread_closeup','pin_thread_closeup',dict(pin_only=True,loose=True)),
            ('tapped_ear_below','tapped_ear_below',dict(body_only=True)),
            ('pin_halfway','halfway_detail',dict(remaining=s.EAR_THICKNESS*.0005)),
            ('pin_seated','seated_detail',{}),
            ('render_m288','m288',dict(pin_only=True,loose=True)),
        ]:
            set_pose(model,data,**opts)
            Image.fromarray(capture(renderer,model,data,cam)).save(RUN/(name+'.png'))
            print('Rendered',name,flush=True)
        comparisons()
        if video:
            rows=[]; fps=30
            writer=imageio.get_writer(str(RUN/'screw_in.mp4'),fps=fps,codec='libx264',
                pixelformat='yuv420p',macro_block_size=1,ffmpeg_params=['-crf','18','-preset','medium'])
            try:
                for i in range(8*fps):
                    t=i/fps; remaining,stage=animation_pose(t)
                    set_pose(model,data,remaining=remaining)
                    # During free passage hold the engagement orientation.
                    if t<2.15:
                        engage,_=animation_pose(2.15)
                        pid=model.body_mocapid[model.body('shackle_pin').id]
                        data.mocap_quat[pid]=spin_quat(2*np.pi*engage/(s.THREAD_PITCH*.001))
                        mujoco.mj_forward(model,data)
                    frame=capture(renderer,model,data,'three_quarter')
                    # The main view remains three-quarter. A MuJoCo-rendered
                    # inset keeps the actual engagement clearly inspectable.
                    inset=capture(renderer,model,data,'engagement_closeup')
                    inset=Image.fromarray(inset).resize((600,338),Image.Resampling.LANCZOS)
                    im=Image.fromarray(frame);im.paste(inset,(658,352))
                    draw=ImageDraw.Draw(im)
                    draw.rectangle((656,350,1260,692),outline='#8c9ba8',width=2)
                    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',19)
                    draw.text((22,22),'MuJoCo  |  right-hand helical thread',font=font,fill='#edf1f5')
                    draw.text((22,51),f'{stage}  |  pitch {s.THREAD_PITCH:g} mm/rev',font=font,fill='#cbd6df')
                    draw.text((666,325),'Engagement detail — same pose, MuJoCo',font=font,fill='#edf1f5')
                    writer.append_data(np.asarray(im))
                    rows.append({'frame':i,'time_s':t,'stage':stage,'remaining_m':remaining,
                                 'tip_z_m':float(data.mocap_pos[model.body_mocapid[model.body('shackle_pin').id],2]),
                                 'spin_about_plus_z_rad':2*np.pi*(engage if t<2.15 else remaining)/(s.THREAD_PITCH*.001)})
                    if i in (0,60,100,150,225,239): im.save(RUN/f'video_frame_{i:03d}.png')
                    if i%30==0:print('Video frame',i,'/ 240',flush=True)
            finally:writer.close()
            (RUN/'animation_poses.json').write_text(json.dumps(rows,indent=2)+'\n')
    finally:renderer.close()
    print('MuJoCo',mujoco.__version__,'done',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--stills-only',action='store_true')
    args=parser.parse_args();render(video=not args.stills_only)
