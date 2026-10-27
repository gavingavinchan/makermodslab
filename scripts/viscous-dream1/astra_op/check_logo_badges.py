# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""No-inference, no-video badge physics and episode-pose still checks."""
from pathlib import Path
import sys,json,hashlib,argparse,shutil,os,tempfile
import numpy as np
import mujoco
import server,rov_scene,logo_badges,sweep_scene,harness_env
from PIL import Image,ImageDraw
ROOT=server.ROOT;OUT=ROOT/'runs/astra_flange/logo_check';OUT.mkdir(exist_ok=True)
# The inherited server normally selects its own subdirectory; honor this task's
# explicit simulation TMPDIR after importing it, without editing that server.
(ROOT/'.cache/tmp').mkdir(parents=True,exist_ok=True)
os.environ['TMPDIR']=str(ROOT/'.cache/tmp');tempfile.tempdir=os.environ['TMPDIR']
SOURCE=ROOT/'runs/astra_flange/astra_rov_backup2_g3_20261026_001'
SHA=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def baseline():
    p=ROOT/'astra_op/logo_assets/rov_scene.prelogo.py'
    ns={'__file__':str(ROOT/'astra_op/rov_scene.py'),'__name__':'logo_baseline'}
    exec(compile(p.read_text(),str(p),'exec'),ns)
    return ns

def physics():
    old=baseline();source=sweep_scene.viscous_xml('flange');checks=[]
    for profile,spin,parking in [('astra_rov',False,'original'),('astra_rov_backup',True,'original'),('astra_rov_backup2',True,'backup2')]:
        a=old['xml'](source,spin,parking);b=rov_scene.xml(source,spin,parking)
        ma=mujoco.MjModel.from_xml_string(a);mb=mujoco.MjModel.from_xml_string(b)
        import xml.etree.ElementTree as E
        stripped=E.fromstring(b)
        for parent in stripped.iter():
            for child in list(parent):
                if child.get('name','').startswith('badge_'):parent.remove(child)
        unchanged_xml=E.tostring(stripped,encoding='unicode')==a
        mismatches=[]
        for name in dir(ma):
            if name.startswith(('body_','jnt_','dof_','actuator_','eq_','pair_','exclude_','tendon_')) and name not in ('body_geomadr','body_geomnum','body_bvhadr','body_bvhnum','body_bvhadr','body_bvhnum'):
                x,y=getattr(ma,name),getattr(mb,name)
                if isinstance(x,np.ndarray) and not np.array_equal(x,y):mismatches.append(name)
        # Compare each original geom by name: inserting visual geoms shifts IDs.
        for i in range(ma.ngeom):
            j=mb.geom(ma.geom(i).name).id
            for name in ['geom_type','geom_contype','geom_conaffinity','geom_condim','geom_bodyid','geom_dataid','geom_size','geom_pos','geom_quat','geom_friction','geom_solmix','geom_solref','geom_solimp','geom_margin','geom_gap','geom_priority','geom_fluid']:
                if not np.array_equal(getattr(ma,name)[i],getattr(mb,name)[j]):mismatches.append(ma.geom(i).name+':'+name)
        badges=[i for i in range(mb.ngeom) if mb.geom(i).name.startswith('badge_')]
        assert len(badges)==16 and np.all(mb.geom_contype[badges]==0) and np.all(mb.geom_conaffinity[badges]==0) and np.all(mb.geom_group[badges]==2)
        check=dict(profile=profile,nbody_before=ma.nbody,nbody_after=mb.nbody,ngeom_before=ma.ngeom,ngeom_after=mb.ngeom,added_visual_geoms=len(badges),old_xml_identical_after_removing_badge_elements=unchanged_xml,compiled_mismatches=mismatches,baseline_xml_sha256=hashlib.sha256(a.encode()).hexdigest())
        check['passed']=unchanged_xml and ma.nbody==mb.nbody and not mismatches;checks.append(check);print('PHYSICS',check,flush=True)
    # Tabletop never calls the ROV XML hook. Prove XML and initialization parity
    # with the hook available versus replaced by a function that would fail.
    a=harness_env.make_env('table');orig=logo_badges.add
    args=server.parse_args(['--profile','astra','--condition','G3','--episode','badge_table_check','--output',str(OUT/'unused')]);server.initialize(a,args)
    def forbidden(root):raise AssertionError('tabletop invoked ROV badge hook')
    try:
        logo_badges.add=forbidden;b=harness_env.make_env('table');server.initialize(b,args)
    finally:logo_badges.add=orig
    table=dict(ngeom_equal=a.model.ngeom==b.model.ngeom,nbody_equal=a.model.nbody==b.model.nbody,qpos_equal=np.array_equal(a.data.qpos,b.data.qpos),ctrl_equal=np.array_equal(a.data.ctrl,b.data.ctrl),body_mass_equal=np.array_equal(a.model.body_mass,b.model.body_mass),no_badges=not any(a.model.geom(i).name.startswith('badge_') for i in range(a.model.ngeom)))
    a.close();b.close()
    report=dict(passed=all(c['passed'] for c in checks) and all(table.values()),profiles=checks,tabletop_astra=table)
    (OUT/'physics.json').write_text(json.dumps(report,indent=2)+'\n');assert report['passed']

class ReplayComplete(BaseException):pass
class NoVideo:
    def append_data(self,*a,**k):raise AssertionError('Video render/export forbidden in badge QA')
    def close(self):pass

def replay():
    """Replay actual CLI commands through the first 2,000 source log rows.

    Video capture/export and RGB look output are disabled; point depth stills
    remain real. The simulation/controller/IK/measurement paths are unchanged.
    """
    old=baseline();current_xml=rov_scene.xml
    rows=[json.loads(l) for l in (SOURCE/'episode.jsonl').open()][:2000]
    commands=[r['argv'] for r in rows if r['kind']=='command_start']
    original_record=server.Episode.record;original_image=server.Episode.image
    server.Episode.video=lambda self:None
    server.Episode.write_workspace=lambda self:None
    server.imageio.get_writer=lambda *a,**k:NoVideo()
    server.Episode.image=lambda self,camera,depth=False:original_image(self,camera,True) if depth else np.zeros((540,960,3),np.uint8)
    arrays=[];meta=[]
    for label,xml in [('before',old['xml']),('after',current_xml)]:
        rov_scene.xml=xml;states=[];times=[];kinds=[]
        def record(self,kind,**extra):
            original_record(self,kind,**extra)
            states.append(np.r_[self.d.qpos.copy(),self.d.qvel.copy(),self.d.ctrl.copy()]);times.append(float(self.d.time));kinds.append(kind)
            if len(states)>=2000:raise ReplayComplete()
        server.Episode.record=record
        temp=OUT/('replay_'+label);ws=OUT/'replay_workspace';server.WSROOT=ws
        if temp.exists():shutil.rmtree(temp)
        if (ws/label).exists():shutil.rmtree(ws/label)
        args=server.parse_args(['--scene','rov','--profile','astra_rov_backup2','--condition','G3','--seed','20261024','--episode',label,'--output',str(temp),'--video-export','capture'])
        ep=None
        try:
            ep=server.Episode(args)
            for cmd in commands:
                print('REPLAY',label,len(states),cmd,flush=True);ep.execute(cmd)
        except ReplayComplete:pass
        finally:
            if ep:
                ep.log.close()
                for name in ('render','toprender','videorender'):
                    r=getattr(ep,name,None)
                    if r is not None:r.close()
                ep.writer.close();ep.e.close()
        arr=np.array(states);arrays.append(arr);meta.append(dict(label=label,rows=len(states),last_time_s=times[-1],state_sha256=hashlib.sha256(arr.tobytes()).hexdigest(),times_sha256=hashlib.sha256(np.array(times).tobytes()).hexdigest(),kinds_sha256=hashlib.sha256(json.dumps(kinds).encode()).hexdigest()))
        shutil.rmtree(temp);shutil.rmtree(ws/label)
    rov_scene.xml=current_xml;server.Episode.record=original_record
    same=np.array_equal(*arrays)
    report=dict(passed=same and len(arrays[0])==2000,source_episode=SOURCE.name,source_episode_sha256=SHA(SOURCE/'episode.jsonl'),command_count=len(commands),compared_rows=2000,before_after=meta,bit_identical_qpos_qvel_ctrl=same,max_state_difference=float(abs(arrays[0]-arrays[1]).max()),commands=commands,method='Actual Episode.execute CLI replay, original 1ms controller/physics/IK/thread/grasp paths. Abort after 2000 log rows. No video capture, render or export. RGB look output replaced with blank scratch stills; point depth uses real renderer. Logged point results are not substituted into recorded numeric commands.')
    (OUT/'replay.json').write_text(json.dumps(report,indent=2)+'\n');print('REPLAY_RESULT',report['passed'],flush=True);assert report['passed']

def stills():
    e=rov_scene.make_env(True,'backup2');m,d=e.model,e.data
    cap=np.load(SOURCE/'video_states.npz');rows=[json.loads(l) for l in (SOURCE/'episode.jsonl').open()]
    seated=next(r for r in rows if r.get('nut_seated') and r.get('left_holds_bolt'))
    # The captured qpos arrays are exact episode states; no simulation steps.
    frames=[('start',0),('seated',int(np.argmin(abs(cap['time']-seated['time'])))),('interference_27s',269)]
    renderer=mujoco.Renderer(m,width=960,height=640);manifest=[]
    for label,index in frames:
        d.qpos[:]=cap['qpos'][index];d.time=cap['time'][index];mujoco.mj_forward(m,d)
        renderer.update_scene(d,camera='rov',scene_option=rov_scene.camera_options('rov'));Image.fromarray(renderer.render()).save(OUT/f'{label}_rov.png')
        manifest.append(dict(pose=label,index=index,time_s=float(d.time)))
        for arm in ('left','right'):
            body=d.body(arm+'_forearm_link');R=body.xmat.reshape(3,3)
            for side,sl in [(1,'PosY'),(-1,'NegY')]:
                target=body.xpos+R@np.array([.057,side*.035,.05879]);back=R@np.array([0,side,0]);up=R@np.array([0,0,1]);right=np.cross(up,back)
                camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE;camera.lookat[:]=target;camera.distance=.23
                # Use MjvGLCamera to retain the forearm's actual local up vector.
                renderer.update_scene(d,camera=camera,scene_option=rov_scene.camera_options('wide'))
                for glcam in renderer.scene.camera:
                    glcam.pos[:]=target+back*.23;glcam.forward[:]=-back;glcam.up[:]=up;glcam.frustum_near=.001;glcam.frustum_far=30
                    glcam.frustum_bottom=-.00031;glcam.frustum_top=.00031;glcam.frustum_center=0
                Image.fromarray(renderer.render()).save(OUT/f'{label}_{arm}_{sl}.png')
    renderer.close();e.close();(OUT/'stills.json').write_text(json.dumps(dict(source_states_sha256=SHA(SOURCE/'video_states.npz'),poses=manifest,physics_steps=0,temporary_external_cameras=True),indent=2)+'\n')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['physics','replay','stills']);a=p.parse_args();globals()[a.mode]()
