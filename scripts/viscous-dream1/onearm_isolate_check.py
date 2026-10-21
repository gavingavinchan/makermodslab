"""No inference: exact legacy comparisons, camera geometry, inputs, hold and task budget."""
import base64
import hashlib
import io
import json
import numpy as np
import mujoco
from PIL import Image, ImageDraw
import onearm_check
import onearm_isolate_sweep as runner
import sweep

OUT = sweep.ROOT / 'isolate_validation'


def decoded(window):
    return np.array(Image.open(io.BytesIO(base64.b64decode(window[-1]))).convert('RGB'))


def check():
    current = onearm_check.legacy_snapshot()
    before = json.loads((OUT/'legacy_before.json').read_text())
    for a,b in zip(before,current):
        assert {k:v for k,v in a.items() if k!='images'} == {k:v for k,v in b.items() if k!='images'}
    sweep.atomic(OUT/'legacy_after.json',current)
    previous = json.loads((OUT/'previous_modes_before.json').read_text())
    legacy_pixels={}
    for name in ('single_copy','parked'):
        c=sweep.parse_config(name)
        env,_,_,images,_=sweep.initialize('apple',c,20261020)
        try:
            actual={'qpos':env.data.qpos.tolist(),'qvel':env.data.qvel.tolist(),'ctrl':env.data.ctrl.tolist(),'state':{s:q.tolist() for s,q in env.state().items()},'policy':sweep.policy(c,env)}
            assert actual==previous[name]
            with np.load(OUT/(name+'_before.npz')) as prior:
                legacy_pixels[name]={}
                for k in images:
                    d=np.abs(images[k].astype(int)-prior[k].astype(int))
                    assert d.max()<=1
                    legacy_pixels[name][k]={'max':int(d.max()),'changed_pixels':int(np.any(d,axis=2).sum())}
        finally: env.close()
    environments={}
    try:
        for name in ('parked','single_fixedcam','parked_blind','single'):
            environments[name]=sweep.initialize('apple',sweep.parse_config(name),20261020)
        p,pw,_,pi,_=environments['parked']
        f,fw,_,fi,_=environments['single_fixedcam']
        b,bw,_,bi,_=environments['parked_blind']
        s,_,_,si,_=environments['single']
        for name in runner.NAMES:
            assert sweep.config_name(sweep.parse_config(name))==name
        np.testing.assert_array_equal(f.state()['right'],p.state()['right'])
        np.testing.assert_array_equal(b.state()['right'],p.state()['right'])
        assert not any((f.model.body(i).name or '').startswith('right_') for i in range(f.model.nbody))
        assert f.model.cam_bodyid[f.model.camera('right').id]==0
        for attr in ('cam_xpos','cam_xmat'):
            np.testing.assert_allclose(getattr(f.data,attr)[f.model.camera('right').id],getattr(p.data,attr)[p.model.camera('right').id],atol=2e-15,rtol=0)
        assert f.model.camera('right').fovy==p.model.camera('right').fovy
        assert fi['right'].shape==pi['right'].shape==(360,640,3)
        assert fi['right'].any() and not np.array_equal(fi['left'],fi['right'])
        for attr in ('qpos','qvel','ctrl'):
            np.testing.assert_array_equal(getattr(b.data,attr),getattr(p.data,attr))
            np.testing.assert_array_equal(getattr(f.data,attr),getattr(s.data,attr))
        for key in ('top','left'):
            assert np.abs(bi[key].astype(int)-pi[key].astype(int)).max()<=1
            assert np.abs(fi[key].astype(int)-si[key].astype(int)).max()<=1
        assert not bi['right'].any()
        assert all(not decoded([frame]).any() for frame in bw['right'])
        for env in (f,b):
            fake={'q_l_chunk':np.repeat(env.state()['left'][None],48,axis=0),'q_r_chunk':np.full((48,7),.4)}
            _,right,_=sweep.prepared_rows(env,fake,48)
            np.testing.assert_array_equal(right,np.repeat(sweep.PARK[None],48,axis=0))
        pose=f.data.cam_xpos[f.model.camera('right').id].copy()
        rotation=f.data.cam_xmat[f.model.camera('right').id].copy()
        for _ in range(48):
            for env in (p,f,b,s): env.step(env.state()['left'],np.full(7,.4))
        for attr in ('qpos','qvel','ctrl'):
            np.testing.assert_array_equal(getattr(b.data,attr),getattr(p.data,attr))
            np.testing.assert_array_equal(getattr(f.data,attr),getattr(s.data,attr))
        np.testing.assert_array_equal(pose,f.data.cam_xpos[f.model.camera('right').id])
        np.testing.assert_array_equal(rotation,f.data.cam_xmat[f.model.camera('right').id])
        # Side-by-side exact policy JPEGs and absolute pixel difference (no scaling).
        a,z=decoded(pw['right']),decoded(fw['right'])
        d=np.abs(a.astype(int)-z.astype(int)).astype(np.uint8)
        height,width=a.shape[:2]
        canvas=Image.new('RGB',(width*3,height+64),'white'); draw=ImageDraw.Draw(canvas)
        for i,(label,img) in enumerate([('parked wrist_r',a),('single_fixedcam wrist_r',z),('Absolute RGB difference (unscaled)',d)]):
            canvas.paste(Image.fromarray(img),(width*i,28));draw.text((width*i+8,8),label,fill='black')
        stats=dict(mean_absolute_rgb=float(d.mean()),max_absolute_rgb=int(d.max()),changed_pixels=int(np.any(d,axis=2).sum()),pixels=height*width,width=width,height=height)
        draw.text((8,height+39),f'Call 1 / seed 20261020; policy JPEGs. MAE={d.mean():.4f}, max={d.max()}, changed pixels={stats["changed_pixels"]}/{height*width}',fill='black')
        canvas.save(OUT/'parked_fixedcam_wrist_r.png')
        raw=np.abs(pi['right'].astype(int)-fi['right'].astype(int))
        camera=dict(position_m=pose.tolist(),rotation_matrix=rotation.reshape(3,3).tolist(),fovy_deg=float(f.model.cam_fovy[f.model.camera('right').id]),width=640,height=360,aspect=640/360)
        pixel_diff=dict(policy_jpeg=stats,raw=dict(mean_absolute_rgb=float(raw.mean()),max_absolute_rgb=int(raw.max()),changed_pixels=int(np.any(raw,axis=2).sum())))
    finally:
        for env,*_ in environments.values(): env.close()
    for name in runner.NAMES:
        sweep.episode('apple',sweep.parse_config(name),0,False,sweep.Ledger(),dry_root=sweep.ROOT/'dry')
        log=json.loads((sweep.ROOT/'dry'/name/'apple/ep_00/log.json').read_text())
        assert log['dry_executed_rows']==864
        if name=='parked_blind': assert log['parked_max_joint_deviation_rad']==0
    # Exercise the extra limit inside the original ledger's locked atomic update.
    fixture=OUT/'budget_fixture'
    fixture.mkdir(exist_ok=True)
    ledger=runner.IsolateLedger(fixture,cap=1312)
    requests=[dict(id=i+1,episode='old',status='billed') for i in range(1248)]
    sweep.atomic(ledger.path,dict(cap=1312,requests=requests))
    r=ledger.reserve('test',18)
    try: ledger.reserve('test',18)
    except sweep.BudgetEnd: pass
    else: raise AssertionError('Task limit not enforced')
    ledger.finish(r['id'],'pool_warming')
    ledger.reserve('test',18)
    assert runner.IsolateLedger(fixture).read()['budget_used']==1249
    protected=json.loads((OUT/'protected_before.json').read_text())
    assert protected=={n:hashlib.sha256((sweep.HERE/n).read_bytes()).hexdigest() for n in protected}
    sweep.atomic(OUT/'checks.json',dict(inference_calls=0,legacy_physics_policy_variants=len(current)+2,legacy_pixels=legacy_pixels,camera=camera,pixel_difference=pixel_diff,protected_sha256=protected,task_limit_and_warming_refund_passed=True,new_modes_dry_rows=864))
    print('PASS: 34 legacy physics/policy variants; fixed camera pose/FOV/aspect; paired physics; right state/actions; blind JPEG windows; full dry episodes; hard 220-call limit. Zero inference.',flush=True)


if __name__=='__main__': check()
