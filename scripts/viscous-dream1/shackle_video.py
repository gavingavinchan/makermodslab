"""Offline presentation render of the recorded S001 episode. No integration or inference.

Run with MUJOCO_GL=egl PYTHONDONTWRITEBYTECODE=1 and the dreamtrial venv.
All generated files and temporary files live in the episode's video directory.
"""
import os
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'runs/shackle_astra/shackle_S001_20261028_001/video'
OUT.mkdir(exist_ok=True)
os.environ['TMPDIR'] = str(OUT)
os.environ['MUJOCO_GL'] = 'egl'
sys.dont_write_bytecode = True
import argparse
import hashlib
import json
import re
import socket
import subprocess
import textwrap
import time
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont, ImageOps
import imageio_ffmpeg
from cross_scenes import viscous_xml
from shackle_scene import camera, CENTER

# Fail closed if any dependency ever attempts remote communication or integration.
def forbidden(*args, **kwargs):
    raise RuntimeError('Offline render: networking and physics stepping are forbidden')
socket.socket.connect = forbidden
socket.create_connection = forbidden
mujoco.mj_step = forbidden
mujoco.mj_step1 = forbidden
mujoco.mj_step2 = forbidden

EP = OUT.parent
Z = np.load(EP / 'video_states.npz')
Q, T = Z['qpos'], Z['time']
ROWS = [json.loads(s) for s in (EP / 'episode.jsonl').read_text().splitlines()]
TIMELINE_COMMANDS = {}
for line in (EP/'timeline.txt').read_text().splitlines():
    match=re.match(r'\s*([\d.]+) #(\d+) (start|command_end|command_result) (.*?) \|',line)
    if match: TIMELINE_COMMANDS[int(match[2])]=match[4]
assert all(r['command']==TIMELINE_COMMANDS[r['command_number']] for r in ROWS)
RT = np.array([r['time'] for r in ROWS])
END = float(T[-1])
FF = imageio_ffmpeg.get_ffmpeg_exe()
BG, PANEL, FG, MUTED, GOLD, GREEN = '#101820', '#192530', '#f1f0e9', '#aabac6', '#f1bd6b', '#99dbb3'
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans'
def font(n, bold=False):
    return ImageFont.truetype(FONT + ('-Bold' if bold else '') + '.ttf', n)
def row(t):
    return ROWS[max(0, np.searchsorted(RT, t, side='right') - 1)]
def state_index(t):
    i = int(np.searchsorted(T, t))
    if i and (i == len(T) or t - T[i-1] < T[i] - t): i -= 1
    return min(i, len(T)-1)
def phase(t):
    if t >= 915.19: return 'SEATED', 'Collar seated. Gripper clears for verification.'
    if t >= 408: return 'SCREWING', 'Clockwise turns with release / reset / regrasp.'
    if t >= 393.906: return 'THREAD START', 'The lower thread catches; a short turn checks advance.'
    if t >= 376: return 'INSERT', 'Small downward steps through the clear upper ear.'
    if t >= 350: return 'ALIGN', 'Measure the ears and bring the pin onto their axis.'
    if t >= 310: return 'CARRY / ALIGN', 'Lift the pin and refine the grip and approach.'
    if t >= 90.4: return 'RECOVER / CARRY', 'Recover the dropped pin and change its orientation.'
    if t >= 60.1: return 'DROP / RECOVERY', 'First drop: locate the pin and pick it back up.'
    if t >= 6: return 'CARRY / ALIGN', 'Carry the pin; adjust approach and grip.'
    return 'PICK', 'Locate the pin using cameras and depth; close the gripper.'

class Replay:
    def __init__(self, shadows=False):
        xml = ET.fromstring(viscous_xml('shackle'))
        v = xml.find('visual')
        v.find('global').set('offwidth', '1920')
        v.find('global').set('offheight', '1080')
        v.find('quality').set('offsamples', '8')
        # Existing v2 key, 8192 map and .8 shadow scale are preserved.
        w = xml.find('worldbody')
        camera(w, 'film_wide', [.88,-.83,1.25], np.array([.29,.05,.92]), 30)
        camera(w, 'film_detail', [.44,-.13,1.02], CENTER.copy(), 26)
        self.m = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding='unicode'))
        self.d = mujoco.MjData(self.m)
        assert self.m.nq == Q.shape[1]
        self.r = mujoco.Renderer(self.m, width=1280, height=960)
        actual_samples=int(self.r._mjr_context.offSamples)
        assert actual_samples>=4
        (OUT/'msaa.json').write_text(json.dumps(dict(requested=8,actual=actual_samples),indent=2))
        self.wide = self.m.camera('film_wide').id
        self.detail = self.m.camera('film_detail').id
        self.shadows = shadows
        self.offset = np.array([.02,-.28,.08])
        self.last = None
    def set(self, i):
        self.d.qpos[:] = Q[i]
        self.d.time = T[i]
        # Forward kinematics only. No constraints, controller, observer, or integration.
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)
        mujoco.mj_camlight(self.m, self.d)
        np.testing.assert_array_equal(self.d.qpos, Q[i])
    def view(self, i, mode='detail'):
        self.set(i)
        if mode == 'detail':
            pin = self.d.body('pin').xpos.copy() + [0,0,.045]
            fixed = np.array([CENTER[0], CENTER[1], .815])
            sep = np.linalg.norm(pin-fixed)
            weight = np.clip((sep-.045)/.10,0,1)*.7
            target = (1-weight)*fixed + weight*pin
            scale = 1 + max(0,sep-.055)*4
            eye = target + self.offset*scale
            z = eye-target; z /= np.linalg.norm(z)
            x = np.cross([0,0,1],z); x /= np.linalg.norm(x)
            rot = np.stack([x,np.cross(z,x),z],axis=1)
            self.m.cam_pos[self.detail] = eye
            mujoco.mju_mat2Quat(self.m.cam_quat[self.detail],rot.ravel())
            mujoco.mj_kinematics(self.m,self.d)
            mujoco.mj_camlight(self.m,self.d)
        self.r.update_scene(self.d,camera=self.detail if mode=='detail' else self.wide)
        if mode == 'detail' and not self.shadows:
            self.r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        return Image.fromarray(self.r.render().copy())
    def close(self): self.r.close()

class Writer:
    def __init__(self, name, size):
        self.name=name; self.frames=0; self.mapping=[]
        self.log=(OUT/(name+'.encode.log')).open('w')
        self.p=subprocess.Popen([FF,'-v','error','-y','-f','rawvideo','-pix_fmt','rgb24',
            '-s',f'{size[0]}x{size[1]}','-r','30','-i','-','-an','-c:v','libx264',
            '-preset','medium','-crf','17','-threads','4','-pix_fmt','yuv420p',
            '-movflags','+faststart',str(OUT/name)],stdin=subprocess.PIPE,stderr=self.log)
    def add(self, im, i=None, speed=None, kind='replay'):
        self.p.stdin.write(np.asarray(im,dtype=np.uint8).tobytes())
        self.mapping.append([self.frames,i,None if i is None else float(T[i]),speed,kind])
        self.frames+=1
    def close(self):
        self.p.stdin.close(); assert self.p.wait()==0
        self.log.close()
        (OUT/(self.name+'.frames.json')).write_text(json.dumps(dict(
            columns=['output_frame','saved_state_index','simulation_s','labelled_speed','kind'],
            frames=self.mapping),separators=(',',':')))

def frame(wide,detail,t,speed='10×',vertical=False,headline=None,body=None,hold=False):
    ph,desc=phase(t); rr=row(t)
    if headline: ph=headline
    if body: desc=body
    col=GREEN if t>=915.19 else GOLD
    drops=sum(t>=s for s in [60.1,393.85])
    if vertical:
        im=Image.new('RGB',(1080,1920),BG); d=ImageDraw.Draw(im)
        txt(d,(48,40),'ASTRA',44,FG,True)
        txt(d,(48,102),'Fully autonomous · simulation',28,MUTED)
        pill(d,(810,44,1032,102),speed,size=27)
        # Recompose both views for portrait; no crop of the landscape artwork.
        im.paste(fit(wide,(984,554)),(48,165))
        im.paste(fit(detail,(984,738)),(48,743))
        pill(d,(68,185,408,237),'ARM / SHACKLE',size=22)
        pill(d,(68,763,468,815),'EARS / THREADS',size=22)
        txt(d,(48,1518),ph,38,col,True)
        wrap(d,desc,(48,1584),984,31)
        d.line((48,1730,1032,1730),fill='#364651',width=2)
        txt(d,(48,1754),f'{rr["turns"]:.2f} turns   ·   {drops} drops',30,col,True)
        txt(d,(48,1812),f'Recorded time {t:06.2f} s',25,MUTED)
        txt(d,(48,1860),'Selected moments · one continuous episode',23,MUTED)
    else:
        im=Image.new('RGB',(1920,1080),BG); d=ImageDraw.Draw(im)
        txt(d,(40,32),'ASTRA',46,FG,True)
        txt(d,(250,49),'Fully autonomous · simulation',28,MUTED)
        pill(d,(1600,32,1880,96),speed+'  REPLAY' if not hold else 'FINAL HOLD',size=26)
        im.paste(fit(wide,(728,546)),(40,132))
        im.paste(fit(detail,(1088,816)),(792,132))
        pill(d,(60,152,398,204),'ARM / SHACKLE',size=22)
        pill(d,(812,152,1204,204),'EARS / THREADS',size=22)
        d.rounded_rectangle((40,702,768,948),radius=16,fill=PANEL)
        txt(d,(64,720),ph,32,col,True)
        if headline:
            wrap(d,desc,(64,774),680,27)
        else:
            txt(d,(64,774),'CURRENT ROBOT COMMAND',18,MUTED,True)
            wrap(d,TIMELINE_COMMANDS[rr['command_number']],(64,810),675,25)
        txt(d,(64,902),f'{rr["turns"]:.2f} turns   ·   {drops} drops',24,col,True)
        txt(d,(40,983),f'Recorded time  {t:06.2f} s / {END:.2f} s',25,MUTED)
        txt(d,(792,983),'Cameras + depth only  ·  Zero human input',25,MUTED)
        d.line((40,1040,1880,1040),fill='#364651',width=4)
        d.line((40,1040,40+1840*t/END,1040),fill=col,width=4)
    return im

def full():
    p=Replay(); w=Writer('shackle_autonomous_10x.mp4',(1920,1080)); start=time.monotonic()
    n=int(np.ceil((END-T[0])/10*30))+1
    for f in range(n):
        t=min(T[0]+f*10/30,END); i=state_index(t)
        wide=p.view(i,'wide'); detail=p.view(i)
        im=frame(wide,detail,float(T[i]))
        w.add(im,i,10)
        if f in (0,180,1182,1350,n-1): im.save(OUT/f'full_sample_{f:04d}.jpg',quality=95)
        if f%150==0: print(f'full {f}/{n}, elapsed {time.monotonic()-start:.1f}s',flush=True)
    w.close(); p.close()

# Chronological editorial cuts. Each source interval retains its recorded motion.
BEATS=[
    (0.1,12.1,4,'Pick up the pin','One task: put the pin through the ears and screw it home.'),
    (56,68,3,'A drop. Then a recovery.','The pin slips. Astra locates it again through its cameras.'),
    (84,96,4,'Pick it back up','No reset. The same episode continues.'),
    (310,340,10,'Carry and align','A new grip brings the pin back above the shackle.'),
    (376,400,4,'Through the upper ear','A second drop on release; the lower thread catches. Astra regrips.'),
    (400,520,20,'The thread advances','Clockwise turns draw the pin into the lower ear.'),
    (520,730,42,'Turn. Release. Regrip.','The robot repeats its screw strokes and checks progress.'),
    (730,916,31,'Approaching the seat','Shorter turns bring the collar down onto the upper ear.'),
    (932,END,5,'Seated and verified','The gripper releases. Astra checks the collar with its cameras.'),
]

def card(vertical=False,end=False):
    wh=(1080,1920) if vertical else (1920,1080)
    im=Image.new('RGB',wh,BG);d=ImageDraw.Draw(im)
    x=72 if vertical else 128; width=wh[0]-2*x; y=310 if vertical else 196
    txt(d,(x,y),'GPT-6 ASTRA',30,GOLD,True);y+=80
    title='Pin seated.' if end else 'An AI. A robot.\nOne shackle pin.'
    for line in title.split('\n'):
        txt(d,(x,y),line,78 if vertical else 86,FG,True);y+=112
    y+=38
    if end:
        lines=[('Attempt 2',GREEN),('Learned from its own notes after attempt 1 failed.',FG),
               ('8 turns · ~55 min wall time',GOLD)]
    else:
        lines=[('Fully autonomous · simulation',GOLD),
               ('Decisions from cameras and depth only. Zero human input.',FG)]
    for s,c in lines:
        y+=wrap(d,s,(x,y),width,38 if vertical else 40,c,spacing=16)+28
    d.line((x,wh[1]-230,wh[0]-x,wh[1]-230),fill='#364651',width=2)
    wrap(d,'One recorded episode · no resets or edited robot actions' if end else
         'Selected moments from one continuous episode. Every speed-up is labelled.',
         (x,wh[1]-190),width,27,MUTED)
    return im

def social():
    p=Replay(); a=Writer('shackle_social.mp4',(1920,1080));b=Writer('shackle_social_vertical.mp4',(1080,1920))
    for v,w in [(False,a),(True,b)]:
        im=card(v);im.save(OUT/('social_title_vertical.png' if v else 'social_title.png'))
        for _ in range(90): w.add(im,kind='title card')
    for k,(s,e,speed,head,body) in enumerate(BEATS):
        n=round((e-s)/speed*30)
        for f in range(n):
            i=state_index(min(s+f*speed/30,e));t=float(T[i]);wide=p.view(i,'wide');detail=p.view(i)
            for v,w in [(False,a),(True,b)]:
                im=frame(wide,detail,t,f'{speed}×',v,head,body)
                w.add(im,i,speed)
                if f==n//2:im.save(OUT/f'social_beat_{k}_{"vertical" if v else "wide"}.jpg',quality=95)
        print(f'social beat {k+1}/{len(BEATS)} complete',flush=True)
    i=len(T)-1;wide=p.view(i,'wide');detail=p.view(i)
    for v,w in [(False,a),(True,b)]:
        im=frame(wide,detail,END,'FINAL HOLD',v,'Pin seated.','8.04 turns. Collar seated on the upper ear.',True)
        for _ in range(75):w.add(im,i,0,'final hold')
        end=card(v,True);end.save(OUT/('social_end_vertical.png' if v else 'social_end.png'))
        for _ in range(150):w.add(end,kind='end card')
        w.close()
    p.close()

def stills():
    p=Replay();p.r.close();p.r=mujoco.Renderer(p.m,width=1920,height=1080)
    hero=p.view(len(T)-1);hero.save(OUT/'hero_seated.png')
    # Before/after compares the approach with the final seating, same macro angle.
    before=p.view(state_index(380))
    im=Image.new('RGB',(1920,1080),BG);d=ImageDraw.Draw(im)
    txt(d,(48,38),'ASTRA  /  FROM ALIGNMENT TO SEATING',42,FG,True)
    txt(d,(48,100),'Fully autonomous · simulation · recorded episode states',26,MUTED)
    # Render 4:3 views for the two panels so neither ear is cropped.
    p.r.close();p.r=mujoco.Renderer(p.m,width=1280,height=960)
    for x,t,label in [(48,380,'BEFORE  /  0 TURNS'),(984,END,'AFTER  /  8.04 TURNS')]:
        im.paste(fit(p.view(state_index(t)),(888,666)),(x,180))
        txt(d,(x,876),label,32,GOLD if t==380 else GREEN,True)
        txt(d,(x,932),f'Recorded time {t:.2f} s',25,MUTED)
    txt(d,(48,1014),'No altered poses. No re-simulation.',24,MUTED)
    im.save(OUT/'before_after.png');p.close()
    (OUT/'stills.frames.json').write_text(json.dumps(dict(
        hero_seated=dict(saved_state_index=len(T)-1,simulation_s=END),
        before_after=[dict(saved_state_index=state_index(t),simulation_s=float(T[state_index(t)]))
                      for t in [380,END]]),indent=2))

def hashes():
    paths=[p for p in EP.iterdir() if p.is_file()]
    paths += [EP.parent/(EP.name+'_astra.log'),EP.parent/(EP.name+'_astra_final.md')]
    return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}

def verify():
    results=[]
    for name in ['shackle_autonomous_10x.mp4','shackle_social.mp4','shackle_social_vertical.mp4']:
        path=OUT/name
        metadata_reader=imageio_ffmpeg.read_frames(str(path))
        metadata=next(metadata_reader);metadata_reader.close()
        expected=(1080,1920) if 'vertical' in name else (1920,1080)
        assert metadata['size']==expected and metadata['fps']==30 and metadata['codec']=='h264'
        # Decode every frame and count via ffmpeg's progress output.
        proc=subprocess.run([FF,'-v','error','-i',str(path),'-f','null','-',
                             '-progress','pipe:1'],capture_output=True,text=True)
        assert proc.returncode==0,proc.stderr
        progress=dict(l.split('=',1) for l in proc.stdout.splitlines() if '=' in l)
        mapping=json.loads((OUT/(name+'.frames.json')).read_text())['frames']
        assert int(progress['frame'])==len(mapping)
        replay=[r for r in mapping if r[1] is not None]
        assert all(r[2]==T[r[1]] for r in replay)
        assert all(a[1]<=b[1] for a,b in zip(replay,replay[1:]))
        assert replay[-1][1]==len(T)-1
        if 'autonomous' in name:
            assert replay[0][1]==0
            assert len(replay)==len(mapping)
            assert all(r[3]==10 for r in replay)
        results.append(dict(file=name,frames=len(mapping),duration_s=len(mapping)/30,
                            size=list(metadata['size']),fps=metadata['fps'],codec=metadata['codec'],
                            full_decode='passed',chronological_saved_states=True,
                            sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    assert hashes()==json.loads((OUT/'source_hashes.json').read_text())
    (OUT/'validation.json').write_text(json.dumps(dict(videos=results,original_files_unchanged=True,
        model_calls=0,physics_steps=0,network_connections=0,state_shape=list(Q.shape),
        saved_time_range_s=[float(T[0]),END],render_settings=dict(
        mujoco=mujoco.__version__,backend='EGL',offsamples_requested=8,
        offsamples_actual=json.loads((OUT/'msaa.json').read_text())['actual'],
        shadow_map=8192,shadow_scale=.8,wide_shadows=True,detail_shadows=False,
        detail_resolution=[1280,960],hero_resolution=[1920,1080],output_fps=30)),indent=2))
    print(json.dumps(results,indent=2),flush=True)

def fit(im, wh): return ImageOps.fit(im,wh,method=Image.Resampling.LANCZOS)
def txt(d, xy, s, size=28, color=FG, bold=False): d.text(xy,s,font=font(size,bold),fill=color)
def pill(d, box, s, color=GOLD, size=24):
    d.rounded_rectangle(box, radius=12, fill=PANEL)
    txt(d,(box[0]+18,box[1]+10),s,size,color,True)
def wrap(d, s, xy, width, size=26, color=FG, bold=False, spacing=10):
    words=s.split(); lines=[]; line=''
    for word in words:
        test=(line+' '+word).strip()
        if d.textlength(test,font=font(size,bold))>width and line: lines.append(line);line=word
        else: line=test
    if line: lines.append(line)
    for n,l in enumerate(lines):txt(d,(xy[0],xy[1]+n*(size+spacing)),l,size,color,bold)
    return len(lines)*(size+spacing)

def preview():
    p=Replay()
    sheet=Image.new('RGB',(1920,1440),BG)
    for n,t in enumerate([0,60.2,90,380,450,END]):
        im=p.view(state_index(t));im.save(OUT/f'qa_{t:.1f}.png')
        tile=fit(im,(640,480));ImageDraw.Draw(tile).text((12,12),f'{t:.1f}s',font=font(28,True),fill=GOLD)
        sheet.paste(tile,((n%3)*640,(n//3)*480))
    sheet.save(OUT/'camera_preview.jpg',quality=95)
    p.view(state_index(END),'wide').save(OUT/'wide_preview.png')
    for n,offset in enumerate([[-.05,-.28,.115],[-.15,-.23,.10],[.02,-.28,.08]]):
        p.offset=np.array(offset)
        p.view(state_index(END)).save(OUT/f'hero_angle_{n}.png')
    p.close()

if __name__ == '__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['preview','all','full','social','stills','verify'])
    args=ap.parse_args()
    if args.mode=='preview':preview()
    if args.mode=='all':
        (OUT/'source_hashes.json').write_text(json.dumps(hashes(),indent=2))
        stills();full();social();verify()
    if args.mode=='full':full()
    if args.mode=='social':social()
    if args.mode=='stills':stills()
    if args.mode=='verify':verify()
