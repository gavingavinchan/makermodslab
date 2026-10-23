"""Scene-only inspection renders and exact encoded policy input sheet."""
import mujoco
from PIL import Image
import sweep
from flange_scene import HERE


def render():
    e,w,*_=sweep.initialize('flange',sweep.DEFAULT,20261023)
    r=mujoco.Renderer(e.model,height=720,width=1280)
    try:
        sweep.input_sheet(w,HERE/'runs/flange/contact_sheet.jpg')
        for name,cam in [('front','flange_overview'),('top','flange_plan'),('closeup','flange_detail')]:
            r.update_scene(e.data,camera=cam);Image.fromarray(r.render()).save(HERE/f'runs/flange/overview_{name}.png')
    finally:r.close();e.close()

if __name__=='__main__':render()
