"""Scene-only inspection renders and exact encoded policy input sheet."""
import mujoco
from PIL import Image, ImageDraw, ImageFont
import sweep
from flange_scene import HERE


def render():
    out=HERE/'runs/flange_v2';out.mkdir(parents=True,exist_ok=True)
    e,w,*_=sweep.initialize('flange',sweep.DEFAULT,20261023)
    r=mujoco.Renderer(e.model,height=720,width=1280)
    try:
        sweep.input_sheet(w,HERE/'runs/flange_v2/contact_sheet.jpg')
        for name,cam in [('front','flange_overview'),('top','flange_plan'),('closeup','flange_detail')]:
            r.update_scene(e.data,camera=cam);Image.fromarray(r.render()).save(HERE/f'runs/flange_v2/overview_{name}.png')
    finally:r.close();e.close()
    # Same camera, dimensions and framing: label outside the unchanged renders.
    canvas=Image.new('RGB',(2560,768),'#15202a');draw=ImageDraw.Draw(canvas)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',23)
    for i,(folder,label) in enumerate([('flange','BEFORE: short four-hole flange fixture'),('flange_v2','AFTER: coated pipework, weld necks, eight-hole stud joint')]):
        canvas.paste(Image.open(HERE/f'runs/{folder}/overview_closeup.png'),(1280*i,48))
        draw.text((1280*i+18,12),label,font=font,fill='white')
    canvas.save(out/'before_after.png')

if __name__=='__main__':render()
