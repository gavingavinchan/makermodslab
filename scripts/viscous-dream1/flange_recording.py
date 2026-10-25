"""Three-pane flange recording; scene and policy cameras stay unchanged."""
import numpy as np
import mujoco
from PIL import Image,ImageDraw

class ViewsRecorder:
    def __init__(self,env):
        self.frames=[];self.milestones=[0]
        self.renderer=mujoco.Renderer(env.model,width=640,height=360)
    def grab(self,env,top):
        views=env.images();self.renderer.update_scene(env.data,camera='flange_detail')
        detail=Image.fromarray(self.renderer.render()).crop((100,30,500,255)).resize((640,360),Image.Resampling.LANCZOS)
        canvas=Image.new('RGB',(1920,400),'#14212b');draw=ImageDraw.Draw(canvas)
        for x,pixels,label in [(0,Image.fromarray(top),'TOP'),(640,detail,'FLANGE JOINT'),(1280,Image.fromarray(views['left']),'LEFT WRIST')]:
            canvas.paste(pixels,(x,0));draw.text((x+8,7),label,fill='#ffcf70')
        thread=env.thread
        draw.text((12,370),f't={env.data.time:.1f}s | engaged={thread.engaged} | nut turns={thread.turns:.3f} | advance={thread.advance*1000:.2f}mm | grasp welds={env.grasps.active}',fill='white')
        self.frames.append(np.asarray(canvas))
