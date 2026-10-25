import sys
sys.path.insert(0,'.')
import sweep,numpy as np,mujoco,json
from flange_start import setup
from flange_scene import Measurements
E=sweep.SweepEnv('flange',sweep.DEFAULT);setup(E,'pregrasp');measure=Measurements(E)
state={s:np.r_[q[:6],0.] for s,q in E.state().items()};rows=[]
for i in range(100):
 E.step(state['left'],state['right']);r=measure.row(0,i,{})
 r['tcp']={s:E.data.site(s+'_tcp').xpos.tolist() for s in E.sides};r['err']={s:(E.state()[s][:6]-state[s][:6]).tolist() for s in E.sides};r['force']={s:E.data.actuator_force[E.aids[s]].tolist() for s in E.sides};rows.append(r)
print('start',E.start_info);print('end',rows[-1]);print('events',E.grasps.events)
json.dump(rows,open('runs/flange_dream/validation/hold_check.json','w'),indent=1);E.close()
