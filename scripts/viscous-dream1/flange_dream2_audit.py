"""Audit all completed real flange episodes and make video review contact sheets."""
from pathlib import Path
import json,subprocess,csv,hashlib
import numpy as np
import imageio.v2 as imageio
import imageio_ffmpeg
from PIL import Image,ImageDraw
import sweep

OUT=sweep.HERE/'runs/flange_dream2'
def run():
    evidence=OUT/'validation';rows=[];torque=[]
    ledger=json.loads((sweep.ROOT/'ledger.json').read_text());requests=[r for r in ledger['requests'] if r['id']>1626]
    assert ledger['cap']==2212 and sum(r['status']!='pool_warming' for r in requests)<=400
    scripted=json.loads((OUT/'scripted/log.json').read_text())
    for s in ('left','right'):
        for i in range(6):torque.append(dict(episode='scripted',arm=s,joint=i+1,max_abs_Nm=scripted['arm_torque_max_abs_Nm'][s][i],above_5_s=scripted['arm_time_above_s']['5'][s][i],above_14_s=scripted['arm_time_above_s']['14'][s][i]))
    for p in sorted((OUT/'episodes').rglob('log.json')):
        e=json.loads(p.read_text())
        if not e['finished']:continue
        measurements=e['row_measurements'];folder=p.parent
        chunks=[json.loads(l) for l in (folder/'chunks.jsonl').read_text().splitlines()]
        assert len(chunks)<=30 and e['executed_rows']==sum(c['executed_rows'] for c in chunks)==len(measurements)
        eid='/'.join(folder.relative_to(OUT/'episodes').parts)
        accounting=[r for r in requests if r['episode']==eid]
        assert e['calls_used']==sum(r['status']!='pool_warming' for r in accounting)
        assert len(chunks)==sum(r['status']=='billed' for r in accounting)
        if e['success']:
            assert e['confirmed_success_call']==e['first_success_call']+1
            assert chunks[-1]['call']==e['confirmed_success_call']
        for s in ('left','right'):
            np.testing.assert_allclose(e['arm_torque_max_abs_Nm'][s],np.max([r['arm_torque_peak_abs_Nm'][s] for r in measurements],axis=0),atol=1e-12)
            assert max(e['arm_torque_max_abs_Nm'][s])<= (40 if e['rig']=='yam' else 25)
            for rating in ('5','14'):
                np.testing.assert_allclose(e['arm_time_above_s'][rating][s],np.sum([r['arm_time_above_s'][rating][s] for r in measurements],axis=0),atol=1e-8)
            for i in range(6):torque.append(dict(episode=eid,arm=s,joint=i+1,max_abs_Nm=e['arm_torque_max_abs_Nm'][s][i],above_5_s=e['arm_time_above_s']['5'][s][i],above_14_s=e['arm_time_above_s']['14'][s][i]))
        frames=imageio_ffmpeg.read_frames(str(folder/'run.mp4'));m=next(frames);frames.close()
        assert m['codec']=='h264' and m['pix_fmt'].startswith('yuv420p') and m['source_size']==(1920,400)
        decoded=subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-i',str(folder/'run.mp4'),'-f','null','-'],capture_output=True,text=True)
        assert decoded.returncode==0,decoded.stderr
        review=evidence/('review_'+eid.replace('/','_')+'.jpg')
        if not review.exists():
            reader=imageio.get_reader(folder/'run.mp4');count=reader.count_frames();indices=np.linspace(0,count-1,8).astype(int)
            canvas=Image.new('RGB',(960,8*220),'#14212b');draw=ImageDraw.Draw(canvas)
            for k,i in enumerate(indices):
                canvas.paste(Image.fromarray(reader.get_data(int(i))).resize((960,200)),(0,k*220))
                draw.text((8,k*220+201),f'{eid} | video t={i/5:.1f}s',fill='white')
            reader.close();canvas.save(review,quality=90)
        rows.append(dict(episode=eid,calls=e['calls_used'],rows=e['executed_rows'],video=m,full_decode_passed=True,review=str(review.relative_to(sweep.HERE))))
    if torque:
        with (OUT/'episode_torques.csv').open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(torque[0]));w.writeheader();w.writerows(torque)
    result=dict(episodes=rows,ledger={k:ledger[k] for k in ('cap','budget_used','confirmed_billed','uncertain')},
        task_requests=dict(billed=sum(r['status']=='billed' for r in requests),pool_warming=sum(r['status']=='pool_warming' for r in requests),uncertain=sum(r['status']=='reserved' for r in requests)),
        protected_sha256={n:hashlib.sha256((sweep.HERE/n).read_bytes()).hexdigest() for n in ('viscous_arm.json','viscous_arm_fingertip.json')})
    sweep.atomic(evidence/'episode_audit.json',result)
    print(json.dumps(dict(completed_episodes=len(rows),**result['task_requests']),indent=1))
if __name__=='__main__':run()
