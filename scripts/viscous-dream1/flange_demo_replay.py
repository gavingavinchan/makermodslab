"""Re-render the recorded arm commands; no IK, inference or object pose changes."""
import json
import numpy as np
import mujoco
import imageio.v2 as imageio
from PIL import Image,ImageDraw,ImageFont
import sweep
from flange_scene import HERE,Measurements


def replay():
    out=HERE/'runs/flange_v2';log=json.loads((out/'scripted_demo_log.json').read_text())
    e,*_=sweep.initialize('flange',sweep.DEFAULT,20261023);o=Measurements(e)
    r=mujoco.Renderer(e.model,width=640,height=360)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
    writer=imageio.get_writer(out/'scripted_demo.mp4',fps=10,codec='libx264',pixelformat='yuv420p',macro_block_size=1,quality=8)
    errors=[];samples=[]
    try:
        for i,row in enumerate(log['row_measurements']):
            e.step(row['command']['left'],row['command']['right']);sample=o.row(0,i,{})
            errors.append(max(np.max(np.abs(np.array(sample[n+'_xyz'])-row[n+'_xyz'])) for n in ('bolt','nut')))
            if i%2:continue
            top=e.images()['top'];r.update_scene(e.data,camera='flange_detail');close=r.render().copy()
            im=Image.new('RGB',(1280,400),'#14212b');im.paste(Image.fromarray(top),(0,0));im.paste(Image.fromarray(close),(640,0))
            label=row['stage'];dr=ImageDraw.Draw(im)
            dr.text((12,363),f'{label} | {e.data.time:.1f}s | inserted={sample["bolt_inserted_depth_m"]*1000:.0f} mm | turns={sample["nut_turns"]:.2f}',font=font,fill='white')
            writer.append_data(np.asarray(im))
            if i in (0,80,160,240,360,480):samples.append(im.copy())
        canvas=Image.new('RGB',(1280,200*((len(samples)+1)//2)))
        for i,im in enumerate(samples):canvas.paste(im.resize((640,200)),((i%2)*640,(i//2)*200))
        canvas.save(out/'validation/demo_review.jpg',quality=93)
        sweep.atomic(out/'validation/demo_replay.json',dict(inference_calls=0,object_pose_overrides=False,max_object_replay_error_m=float(max(errors)),rows=len(errors),passed=bool(max(errors)<1e-8)))
        assert max(errors)<1e-8,max(errors)
    finally:writer.close();r.close();e.close()

if __name__=='__main__':replay()
