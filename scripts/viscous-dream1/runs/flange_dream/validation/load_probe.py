import sys
sys.path.insert(0,'.')
import sweep,numpy as np,mujoco,json
from flange_start import local_ik
from flange_ik_check import orientation
from flange_scene import HOLE
E=sweep.SweepEnv('flange',sweep.DEFAULT);ik=local_ik(E);out=[]
for side,sg in [('left',1),('right',-1)]:
 for final in (False,True):
  for deg in (30,60,90,120,210,240,270,300):
   axis=np.array([-sg,0.,0.]) if final else np.array([-sg*np.cos(np.pi/6),-.5,0.])
   a=np.deg2rad(deg);R=orientation(axis,[0,np.cos(a),np.sin(a)])
   p=HOLE+([.118,0,0] if sg==1 else [-.081,0,0]) if final else HOLE+[sg*.16,.06,.05]-.038*axis
   for attempt in range(12):
    seed=ik.rng.uniform(ik.limits[side][:,0],ik.limits[side][:,1]);q,res=ik.fit(side,p,R,seed,starts=0)
    if np.linalg.norm(res[:3])>.001 or np.linalg.norm(res[3:])/.15>.02:continue
    E.data.qpos[E.qadr[side]]=q;mujoco.mj_forward(E.model,E.data)
    body=E.data.body(side+'_wrist_roll_link');com=p+((.038+float(E.model.body('bolt').ipos[2])) if sg==1 else (.028+float(E.model.body('nut').ipos[2])))*axis
    jp=np.zeros((3,E.model.nv));jr=np.zeros_like(jp);mujoco.mj_jac(E.model,E.data,jp,jr,com,body.id)
    own=E.data.qfrc_bias[E.vadr[side]].copy();payload=jp[2,E.vadr[side]]*(.65 if sg==1 else .18)*9.81
    tau=own+payload
    c=ik.contacts(side,q,.44);c=[c for c in c if c['dist_m']<-.0003 and not any(n in ('bolt','nut') for n in c['bodies'])]
    r=dict(side=side,final=final,roll=deg,q=q.tolist(),p=p.tolist(),R=R.tolist(),tau=tau.tolist(),own_gravity_tau=own.tolist(),payload_tau=payload.tolist(),max_tau=float(max(abs(tau))),contacts=len(c))
    out.append(r)
   ok=[r for r in out if r['side']==side and r['final']==final and r['roll']==deg and not r['contacts']]
   print(side,final,deg,'best',min([r['max_tau'] for r in ok],default=None),flush=True)
json.dump(out,open('runs/flange_dream/validation/load_probe.json','w'),indent=1);E.close()
