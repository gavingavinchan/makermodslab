"""Replay the saved force-only diagnostic into the required three-pane recording."""
import sys,json
sys.path.insert(0,'.')
import numpy as np,imageio.v2 as imageio
import sweep
from flange_recording import ViewsRecorder
out=sweep.HERE/'runs/flange_dream/force_only';out.mkdir(exist_ok=True)
log=json.loads((sweep.HERE/'runs/flange_dream/validation/force_only_demo.json').read_text())
e,*_=sweep.initialize('flange',dict(sweep.DEFAULT,flange_attach=False),20261023);rec=ViewsRecorder(e)
writer=imageio.get_writer(out/'run.mp4',fps=10,codec='libx264',pixelformat='yuv420p',macro_block_size=1,quality=8)
error=0.
try:
    with (out/'chunks.jsonl').open('w') as f:
        for i,row in enumerate(log['row_measurements']):
            cmd=row['command'];e.step(cmd['left'],cmd['right'])
            error=max(error,float(np.linalg.norm(e.data.body('bolt').xpos-row['bolt_xyz'])))
            f.write(json.dumps(dict(row=i,execution=cmd))+'\n')
            if i%2==0:
                rec.grab(e,e.images()['top']);writer.append_data(rec.frames[-1]);rec.frames.clear()
    assert error<1e-8,error
    log['replay_max_bolt_position_error_m']=error;sweep.atomic(out/'log.json',log)
    print('PASS: force-only exact replay, three-pane H.264 video; max bolt error',error)
finally:writer.close();rec.renderer.close();e.close()
