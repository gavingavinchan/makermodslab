"""Shackle adapter for sweep.py; durable chunks and the existing call ledger/cap.
The default invocation is a local hold stub. No client is constructed in dry mode.
"""
import json,os,time,warnings
from collections import deque
import numpy as np,mujoco
import sweep as sw
from shackle_scene import CONFIG,OUT,PROMPT,Measurements
from shackle_env import ShackleEnv

class Recorder:
 def __init__(self,e):self.frames=[];self.milestones=[];self.renderer=mujoco.Renderer(e.model,width=640,height=360)
 def grab(self,e,top):
  self.renderer.update_scene(e.data,camera='shackle_detail');self.frames.append(np.concatenate([top,self.renderer.render().copy()],axis=1))

def initialize(seed):
 e=ShackleEnv();jitter=np.random.default_rng(seed).uniform(-.01,.01,2);e.data.joint('pin_free').qpos[:2]+=jitter;mujoco.mj_forward(e.model,e.data)
 windows={k:deque(maxlen=sw.y.FRAMES) for k in sw.y.VIDEO_ROLES};steps=0
 while any(len(w)<sw.y.FRAMES for w in windows.values()):
  st=e.state();e.step(st['left'],st['right'])
  if steps%sw.y.FRAME_EVERY_STEPS==0:images=sw.y.capture(e,windows)
  steps+=1
 return e,windows,steps,images,jitter

def episode(index=0,live=False,ledger=None,call_cap=30):
 seed=20261028+index;directory=OUT/('dream' if live else 'dry')/f'ep_{index:02d}';directory.mkdir(parents=True,exist_ok=True)
 path=directory/'log.json';chunks_path=directory/'chunks.jsonl'
 prior=json.loads(path.read_text()) if path.exists() else {}
 if prior.get('finished') and (directory/'run.mp4').exists():return prior
 if live and ledger is None:raise ValueError('Live review launch requires the existing sweep Ledger')
 eid=f'parked/shackle/ep_{index:02d}';e,windows,steps,images,jitter=initialize(seed);measure=Measurements(e);rec=Recorder(e)
 sw.input_sheet(windows,directory/'contact_sheet.jpg')
 if index==0:sw.input_sheet(windows,OUT/'contact_sheet.jpg')
 rec.grab(e,images['top']);client=None;commanded=None;consecutive=0
 metrics=dict(task='shackle',instruction=PROMPT,configuration=CONFIG,seed=seed,jitter_xy_m=jitter.tolist(),call_cap=call_cap,calls_used=0,stub_calls=0,executed_rows=0,success=False,inserted=False,engaged=False,max_turns=0.,max_inserted_depth_m=0.,row_measurements=[],finished=False,dry_run=not live,policy=sw.policy(CONFIG,e))
 def execute(chunk):
  nonlocal steps,images,commanded,consecutive
  for i,(l,r) in enumerate(zip(chunk['execution']['left'],chunk['execution']['right'])):
   e.step(l,r);row=measure.row(chunk['call'],i);metrics['row_measurements'].append(row);metrics['executed_rows']+=1
   metrics['inserted']|=row['inserted'];metrics['engaged']|=row['engaged'];metrics['max_turns']=max(metrics['max_turns'],row['turns']);metrics['max_inserted_depth_m']=max(metrics['max_inserted_depth_m'],row['inserted_depth_m'])
   commanded=dict(q_l_chunk=l,q_r_chunk=r)
   if steps%sw.y.FRAME_EVERY_STEPS==0:
    images=sw.y.capture(e,windows);rec.grab(e,images['top'])
    if client:sw.y.feed(sw.y.CustomGripFeed(client),e,windows,PROMPT,sw.y.BETA_MODEL,steps)
   steps+=1
  consecutive=consecutive+1 if row['seated'] else 0;metrics['success']=consecutive>=2;metrics.update(measure.torque_summary());metrics['calls_used']=ledger.episode_calls(eid) if live else 0;sw.atomic(path,metrics)
 try:
  saved=[]
  if chunks_path.exists():
   for line in chunks_path.read_text().splitlines():
    try:saved.append(json.loads(line))
    except json.JSONDecodeError:break
   # Discard a possible incomplete crash tail before appending.
   chunks_path.write_text(''.join(json.dumps(c)+'\n' for c in saved))
  for c in saved:execute(c)
  if live:
   import dream
   from hand_path_tools import IKTrace,plain
   client=dream.DreamClient(api_key=sw.y.settings().get('DREAM_API_KEY',''));client.policy(sw.policy(CONFIG,e));trace=IKTrace(client._policy._ik,'shackle')
   sw.y.feed(sw.y.CustomGripFeed(client),e,windows,PROMPT,sw.y.BETA_MODEL,steps,every=0)
  call=len(saved)
  while call<call_cap and not metrics['success']:
   if live:
    if ledger.episode_calls(eid)>=call_cap:break
    request=ledger.reserve(eid,call_cap)
    try:reply=client.get_action(commanded=commanded)
    except dream.DreamServerError as error:
     if error.code=='DA-2000' and error.reason=='pool_warming':ledger.finish(request['id'],'pool_warming');time.sleep(min(error.retry_after_s or 10,60));continue
     raise
    ledger.finish(request['id'],'billed');l,r,notices=sw.prepared_rows(e,reply,CONFIG['rows']);call=request['call']
    chunk=dict(call=call,ledger_id=request['id'],anchor=trace.anchor,per_key=plain(reply['per_key']),client_ik=trace.context,execution=plain(dict(left=l,right=r)),notices=notices)
   else:
    call+=1;state=e.state();chunk=dict(call=call,execution={s:np.repeat(state[s][None],CONFIG['rows'],axis=0).tolist() for s in e.sides},notices=['local stub; no inference'])
   with chunks_path.open('a') as f:f.write(json.dumps(chunk)+'\n');f.flush();os.fsync(f.fileno())
   execute(chunk)
  metrics.update(finished=True,stub_calls=0 if live else call,termination='success_two_calls' if metrics['success'] else 'episode_cap' if live else 'local_stub_cap',parked_max_joint_deviation_rad=e.parked_max_joint_deviation)
  sw.atomic(path,metrics);return metrics
 finally:
  if client:client.close()
  import imageio.v2 as imageio
  imageio.mimsave(directory/'run.mp4',rec.frames,fps=5,codec='libx264',pixelformat='yuv420p',macro_block_size=1)
  rec.renderer.close();e.close()
