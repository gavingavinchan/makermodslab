"""Separate shackle profile; shared CLI/motion methods, no legacy profile changes.
No server smoke during the running A/B. Host-side review launch only.
"""
import sys,os,json,time,math,argparse,socket,signal,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'astra_op'))
import server as base
from server import CommandStopped
import numpy as np,mujoco,imageio.v2 as imageio
from PIL import Image,ImageDraw
from scipy.spatial.transform import Rotation
from shackle_scene import Measurements,OUT
from shackle_env import ShackleEnv
from flange_ik_check import LocalIK
CAMERAS=base.CAMERAS
under=base.under
OUTROOT=OUT/'astra'
WSROOT=OUT/'workspaces'
SCREW_TORQUE_RISE_NM=base.SCREW_TORQUE_RISE_NM
SCREW_SNUG_SECONDS=base.SCREW_SNUG_SECONDS
class Episode(base.Episode):
    def __init__(self,args):
        self.args=args;self.out=under(args.output,OUTROOT);self.ws=under(WSROOT/args.episode,WSROOT)
        if len(os.fsencode(str(self.out/'robot.sock')))>=108:raise ValueError('Unix socket path too long; choose a shorter output directory')
        if self.out.exists() and any(self.out.iterdir()):raise ValueError('output must be empty')
        if self.ws.exists() and any(self.ws.iterdir()):raise ValueError('workspace must be empty')
        self.out.mkdir(parents=True,exist_ok=True);(self.ws/'obs').mkdir(parents=True,exist_ok=True)
        self.log=(self.out/'episode.jsonl').open('w',buffering=1)
        self.started=time.monotonic();self.count=0;self.tick=0;self.ended=False;self.reason=None;self.command='initialization'
        self.last_result=None;self.motion=None;self.screw_state=None
        self.cameras={'rov':'rov','wrist_l':'left','wrist_r':'right'} if args.scene=='rov' else CAMERAS
        self.e=ShackleEnv()
        self.m,self.d=self.e.model,self.e.data
        assert abs(self.m.opt.timestep-.001)<1e-12
        for side in self.e.sides:
            np.testing.assert_array_equal(self.m.actuator_forcerange[self.e.aids[side]],np.tile([-25,25],(6,1)))
            np.testing.assert_array_equal(self.m.actuator(side+'_gripper_left_joint_drive').forcerange,[-160,160])
        start='table'
        self.start_label='shackle | table start | left active / right parked'
        rng=np.random.default_rng(args.seed)
        self.d.joint('pin_free').qpos[:2]+=rng.uniform(-.01,.01,2)
        mujoco.mj_forward(self.m,self.d)
        self.q={s:q.copy() for s,q in self.e.state().items()}
        if start!='table':
            for s in self.e.sides:self.q[s][6]=0
        self.correction={s:np.zeros(6) for s in self.e.sides}
        self.grip_integral={s:0. for s in self.e.sides}
        self.grip_latch={s:None for s in self.e.sides}
        if args.condition=='G3' and args.g3_grasp=='flats':self.grip_latch['right']=.0225
        self.previous_reference={s:self.q[s][:6].copy() for s in self.e.sides}
        self.arm_stiffness={s:1. for s in self.e.sides}
        self.measure=Measurements(self.e)
        self.ik=LocalIK(self.e);self.sol=self.ik.solver
        import jax.numpy as jnp
        for group in self.sol._groups:
            group.w["pos"]=jnp.asarray(500.)
            group.w["ori"]=jnp.asarray(100.)
        self.scratch=mujoco.MjData(self.m)
        self.render=mujoco.Renderer(self.m,width=960,height=540)
        self.toprender=mujoco.Renderer(self.m,width=960,height=720)
        self.videorender=mujoco.Renderer(self.m,width=640,height=360)
        self.video_states=[];self.video_export=None
        self.capture_video=args.scene=='rov' or args.video_export=='capture'
        self.writer=imageio.get_writer(self.out/'run.mp4',fps=10,codec='libx264',pixelformat='yuv420p',macro_block_size=1,quality=7,ffmpeg_log_level='error')
        self.grasp_cursor=0;self.thread_cursor=0;self.forces={s:np.zeros(2) for s in self.e.sides};self.force_window=[]
        previous=self.e.row_observer
        def observer():
            # mj_step integrates qpos after computing xpos/xmat. Refresh transforms
            # so the unchanged thread observer sees this tick's physical rotation.
            # This computes kinematics only; it does not integrate or set any qpos.
            mujoco.mj_kinematics(self.m,self.d)
            self.d.qpos[self.e.park_qadr]=self.e.park_values
            self.d.qvel[self.e.park_vadr]=0
            previous();self.tick+=1
            self.check_motion()
            forces=self.jaw_forces();self.force_window.append(forces)
            self.force_window=self.force_window[-100:]
            if self.tick%50==0:self.record('physics')
            if self.tick%100==0:self.video()
            if time.monotonic()-self.started>args.wall_seconds:raise TimeoutError('wall-clock cap')
        self.e.row_observer=observer
        self.record('start',profile='shackle',seed=args.seed,command_cap=args.command_cap,wall_seconds=args.wall_seconds)
        self.hold(1.)
        self.write_workspace()
    def record(self,kind,**extra):
        row=self.measure.row(0,self.tick,{})
        row.update(kind=kind,command=self.command,command_number=self.count,
            qactual={s:q.tolist() for s,q in self.e.state().items()},
            qreference={s:q.tolist() for s,q in self.q.items()},
            thread_events=self.e.thread.events[self.thread_cursor:],**extra)
        self.thread_cursor=len(self.e.thread.events)
        self.log.write(json.dumps(row,allow_nan=False)+'\n')
    def video(self):
        self.videorender.update_scene(self.d,camera='shackle_detail')
        top=self.image('top');detail=self.videorender.render().copy()
        canvas=Image.new('RGB',(1280,400),'#15212a');canvas.paste(Image.fromarray(top).resize((640,360)),(0,0));canvas.paste(Image.fromarray(detail),(640,0))
        ImageDraw.Draw(canvas).text((8,367),f'{self.command} | shackle | hidden inspection video',fill='white')
        self.writer.append_data(np.asarray(canvas))
    def write_workspace(self):
        template=(ROOT/'SHACKLE_README_OPERATOR.md').read_text()
        calibration={c:self.calibration(c) for c in self.cameras}
        (self.ws/'README.md').write_text(template+'\nCamera calibration at start; look refreshes moving extrinsics:\n```json\n'+json.dumps(calibration,indent=2)+'\n```\n')
        wrapper=(ROOT/'astra_op/robot.py').read_text().replace("'__SOCKET__'",repr(str(self.out/'robot.sock')))
        (self.ws/'robot').write_text(wrapper);(self.ws/'robot').chmod(0o755)
    def _execute(self,argv):
        if len(argv)>1 and argv[0] in ('move','nudge','turn','grip','screw') and argv[1]=='right':
            return dict(ok=False,error='Right arm is parked; use the left arm.')
        return super()._execute(argv)
    def screw(self,s,degrees,rate):
        p0,R0=self.pose(s);axis=R0[:,2].copy()
        state=dict(side=s,turned=0.,cycles=0,tracking=False,axis=axis,previous_rotation=R0,phase='acquire',
            baseline=None,baseline_samples=[],samples=[],roll_speed=0.,roll_torque=0.,snug_s=0.,max_progress=0.)
        self.screw_state=state
        # A backup wrench can elastically deflect with the arm. For this
        # profile, compare the two hands' encoder/FK roll when the opposite
        # gripper has a contact latch. No object identity or thread state is
        # available to the controller; an ungripped opposite hand is ignored.
        other='left' if s=='right' else 'right'
        if getattr(self.args,'bolt_free_spin',False) and self.q[other][6]<=.45 and self.grip_latch[other] is not None:
            state.update(backup_side=other,backup_turned=0.,previous_backup_rotation=self.pose(other)[1])
            # A short finishing call may start already at the seat. Retain
            # the last measured moving baseline just as successive strokes
            # within one screw call do; fresh moving samples still replace it.
            state['baseline']=getattr(self,'last_backup_baseline',{}).get(s)
        reason='completed';detail='Requested rotation completed.';commanded=0.;roll_reference=0.;stroke_left=30.
        feed_sign=1. if self.args.thread_hand=='right' else -1.
        direction=(1. if degrees>=0 else -1.)*feed_sign
        state['direction']=direction
        try:
            found=self.set_grip(s,0.) if self.q[s][6]<=.65 else False
            if not found:
                reason='lost_grasp';detail='Start with the part gripped at bilateral contact.'
            else:
                while abs(degrees)-direction*state['turned']>.1:
                    # Wrist roll sign from encoder/FK only, independent of parts.
                    q=self.d.qpos[self.e.qadr[s]].copy()
                    self.scratch.qpos[self.e.qadr[s]]=q
                    self.scratch.qpos[self.e.qadr[s][5]]+=.001
                    mujoco.mj_kinematics(self.m,self.scratch)
                    _,R=self.pose(s)
                    derivative=Rotation.from_matrix(self.scratch.site(s+'_tcp').xmat.reshape(3,3)@R.T).as_rotvec()@axis/.001
                    sign=direction*np.sign(derivative)
                    lo,hi=self.m.joint(s+'_joint6').range
                    available=min(stroke_left,np.degrees((hi-q[5] if sign>0 else q[5]-lo)) - 10.)
                    if available<8.:
                        state['tracking']=False;state['phase']='open';self.motion=None
                        opening=self.proprio()['arms'][s]['opening_mm']
                        g=min(1.,max(.7,(opening+8.)/104.825))
                        self.set_grip(s,g)
                        opened=self.proprio()['arms'][s]
                        release_ok=opened['opening_mm']>=g*104.825-1.5 and max(opened['jaw_load_N'])<=.5
                        self.record('screw_release',cycle=state['cycles']+1,sensed=opened,release_ok=release_ok)
                        if not release_ok:
                            reason='loaded_on_open';detail='Release did not reach the requested opening with unloaded jaws.';break
                        # Keep the roll sweep local to the operator's starting pose.
                        # A sixty-degree reset also preserves the hex-flat phase.
                        q6=float(self.d.qpos[self.e.qadr[s][5]])
                        reset_room=np.degrees(q6-lo if sign>0 else hi-q6)-10.
                        if reset_room<8.:
                            reason='stall';detail='No safe wrist reset travel remains.';break
                        reset=-direction*min(60.,reset_room)
                        p,R=self.pose(s)
                        target=p0+axis*(commanded*feed_sign*2.5/360000.)
                        state['phase']='reset'
                        self.move(s,target,Rotation.from_rotvec(axis*np.radians(roll_reference+reset)).as_matrix()@R0,cartesian=True,rate_deg_s=180.)
                        roll_reference+=reset
                        self.motion=None;state['phase']='close';found=self.set_grip(s,0.)
                        state['cycles']+=1;stroke_left=abs(reset)
                        sensed=self.proprio()['arms'][s]
                        self.record('screw_regrasp',cycle=state['cycles'],sensed=sensed)
                        if not found:
                            reason='lost_grasp';detail='Reclose found no bilateral part contact.';break
                        continue
                    chunk=direction*min(60.,available,abs(degrees)-direction*state['turned'])
                    p,R=self.pose(s)
                    target=p0+axis*((commanded+chunk)*feed_sign*2.5/360000.)
                    state['previous_rotation']=R;state['tracking']=True;state['phase']='turn'
                    if state.get('backup_side') is not None:state['previous_backup_rotation']=self.pose(other)[1]
                    state.update(stroke_start=state['turned'],samples=[],baseline_samples=[],snug_s=0.)
                    self.arm_stiffness[s]=20. if rate>25 else 1.
                    result=self.move(s,target,Rotation.from_rotvec(axis*np.radians(roll_reference+chunk)).as_matrix()@R0,cartesian=True,rate_deg_s=rate)
                    roll_reference+=chunk
                    # A rate is an upper bound: permit slower actuator-limited
                    # tracking until the normal no-progress guard stops it.
                    while True:
                        pos_error,angle_error=self.motion_error()
                        if pos_error<=.003 and angle_error<=np.radians(3.):break
                        self.hold(.05)
                    result=self.motion_result()
                    state['tracking']=False
                    commanded+=chunk;stroke_left-=abs(chunk)
        except CommandStopped as ex:
            reason='timed_out' if ex.timed_out else 'stall';detail=str(ex)
        finally:
            state['tracking']=False
            if state.get('backup_side') is not None and state['baseline'] is not None:
                if not hasattr(self,'last_backup_baseline'):self.last_backup_baseline={}
                self.last_backup_baseline[s]=state['baseline']
            if reason!='completed':self.stop_and_hold()
            final=self.proprio()['arms'][s]
            result=dict(self.motion_result() if self.motion else {})
            result.update(degrees_turned=state['turned']*feed_sign,regrasp_cycles=state['cycles'],stop_reason=reason,
                message=detail,stall_at_start=reason=='stall' and state['max_progress']<1.,
                free_running_torque_Nm=state['baseline'],snug_torque_rise_Nm=SCREW_TORQUE_RISE_NM,snug_sustain_s=SCREW_SNUG_SECONDS,
                stop_phase=state['phase'],stalled=reason=='stall',timed_out=reason=='timed_out',final_pose=final)
            self.screw_state=None
        return result


def parse_args(argv=None):
    p=argparse.ArgumentParser();p.add_argument('--profile',choices=['shackle'],default='shackle');p.add_argument('--scene',choices=['table'],default='table')
    p.add_argument('--condition',choices=['T'],default='T');p.add_argument('--episode',required=True);p.add_argument('--output',required=True);p.add_argument('--seed',type=int,default=20261028)
    p.add_argument('--command-cap',type=int,default=1000);p.add_argument('--wall-seconds',type=float,default=21600.)
    a=p.parse_args(argv)
    if '/' in a.episode or a.episode in ('.','..'):p.error('episode must be one directory name')
    if a.command_cap<1 or not np.isfinite(a.wall_seconds) or a.wall_seconds<=0:p.error('positive caps required')
    a.thread_hand='right';a.bolt_free_spin=False;a.g3_grasp='stock';a.g3_seat_mm=0.;a.video_export='render'
    return a

def main():
    a=parse_args()
    # Preserve the original audited server loop, using only this adapter's roots.
    base.Episode=Episode;base.parse_args=lambda:a
    def guard():
        def audit(event,args):
            if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
                path=Path(os.fsdecode(args[0]))
                if path.name in ('ledger.json','ledger.lock','runner.lock'):raise RuntimeError('protected accounting')
                flags=args[2] if len(args)>2 else 0
                if flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC) and str(path)!='/dev/null' and not path.resolve().is_relative_to(ROOT):raise RuntimeError('write outside task root')
            if event=='socket.connect' and isinstance(args[1],tuple):raise RuntimeError('network disabled')
        sys.addaudithook(audit)
        import dream
        dream.DreamClient=lambda *a,**k:(_ for _ in ()).throw(RuntimeError('inference forbidden in robot server'))
    base.guard=guard;base.main()
if __name__=='__main__':main()
