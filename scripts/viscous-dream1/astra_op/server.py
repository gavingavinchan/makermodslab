#!/usr/bin/env python3
"""Local flange episode. Agent protocol intentionally excludes task state."""
# Additive dispatch: legacy imports, profiles and server loop are unchanged.
if __name__ == '__main__':
    import sys as _shackle_sys
    if any(a == '--profile=shackle' or (a == '--profile' and i+1 < len(_shackle_sys.argv) and _shackle_sys.argv[i+1] == 'shackle') for i,a in enumerate(_shackle_sys.argv)):
        from pathlib import Path as _ShacklePath
        _shackle_sys.path.insert(0,str(_ShacklePath(__file__).resolve().parents[1]))
        from shackle_astra import main as _shackle_main
        _shackle_main()
        raise SystemExit
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
_base=__import__('pathlib').Path(__file__).resolve().parent
os.environ['JAX_COMPILATION_CACHE_DIR']=str(_base/'.cache/jax')
os.environ['XDG_CACHE_HOME']=str(_base/'.cache')
os.environ['DREAM_LOG_DIR']=str(_base/'.cache/dream_logs')
os.environ['TMPDIR']=str(_base/'.cache/tmp')
import sys, json, socket, time, argparse, math, shutil, signal, tempfile
(_base/".cache/tmp").mkdir(parents=True,exist_ok=True)
tempfile.tempdir=str(_base/".cache/tmp")
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.dont_write_bytecode=True
import numpy as np
import mujoco
import imageio.v2 as imageio
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation, Slerp
from scipy.optimize import least_squares
from sweep_scene import SweepEnv
from profiles import initialize, resolve_profile, seating, configure_astra_ik
from flange_scene import Measurements
from flange_ik_check import LocalIK

CAMERAS={'top':'top','wrist_l':'left','wrist_r':'right'}
WSROOT=ROOT.parent/'astra_ws'
OUTROOT=ROOT/'runs/astra_flange'
DT=.05
COMMAND_SECONDS=90.
SCREW_SECONDS=300.
SCREW_TORQUE_RISE_NM=1.5
SCREW_SNUG_SECONDS=.3
SCREW_STOP_SPEED_DEG_S=1.
PROGRESS_SECONDS=10.

class CommandStopped(Exception):
    def __init__(self,reason,timed_out=False):
        super().__init__(reason);self.timed_out=timed_out

def fractions(count):
    # Match linspace's interpolation without allocating a trajectory-sized array.
    # An arbitrarily small, valid turn rate must time out rather than exhaust RAM.
    step=1./count
    for i in range(1,count+1):yield 1. if i==count else i*step

def under(path,root):
    path=Path(path).resolve()
    if not path.is_relative_to(root.resolve()):raise ValueError(f'path must be under {root}')
    return path

def guard():
    """No API client, remote network, ledger, or shared runner lock in this process."""
    def audit(event,args):
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
            p=Path(os.fsdecode(args[0]))
            if p.name in ('ledger.json','ledger.lock','runner.lock'):raise RuntimeError('protected accounting/runner file')
            flags=args[2] if len(args)>2 else 0
            if flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC):
                resolved=p.resolve()
                if str(resolved)!='/dev/null' and not any(resolved.is_relative_to(root) for root in (ROOT/'astra_op',OUTROOT,WSROOT)):
                    raise RuntimeError('write outside harness directories')
        if event=='socket.connect' and isinstance(args[1],tuple):raise RuntimeError('network disabled')
    sys.addaudithook(audit)
    import dream
    def forbidden(*a,**k):raise RuntimeError('Dream clients forbidden')
    dream.DreamClient=forbidden

class Episode:
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
        from harness_env import make_env
        self.e=make_env(args.scene,bolt_free_spin=args.bolt_free_spin,
                        parking='backup2' if args.profile=='astra_rov_backup2' else 'original')
        self.m,self.d=self.e.model,self.e.data
        assert abs(self.m.opt.timestep-.001)<1e-12
        for side in self.e.sides:
            np.testing.assert_array_equal(self.m.actuator_forcerange[self.e.aids[side]],np.tile([-25,25],(6,1)))
            np.testing.assert_array_equal(self.m.actuator(side+'_gripper_left_joint_drive').forcerange,[-160,160])
        start=initialize(self.e,args)
        self.start_label=f'{args.condition} {args.profile} {args.thread_hand} {args.g3_grasp} seat+{args.g3_seat_mm:g}mm'
        if start=='table':
            rng=np.random.default_rng(args.seed)
            for part in ('bolt','nut'):self.d.joint(part+'_free').qpos[:2]+=rng.uniform(-.02,.02,2)
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
        if args.scene=='rov':
            from rov_scene import ROVMeasurements
            self.measure=ROVMeasurements(self.e)
        else:self.measure=Measurements(self.e)
        self.ik=LocalIK(self.e);self.sol=self.ik.solver
        if args.scene=='rov':
            from rov_scene import NativeSolver
            self.sol=NativeSolver(self.sol,self.e)
        elif args.profile=='astra':configure_astra_ik(self.e,self.sol)
        import jax.numpy as jnp
        for group in self.sol._groups:
            group.w["pos"]=jnp.asarray(500.)
            group.w["ori"]=jnp.asarray(100.)
        self.scratch=mujoco.MjData(self.m)
        self.render=mujoco.Renderer(self.m,width=960,height=540)
        self.toprender=mujoco.Renderer(self.m,width=960,height=720)
        self.videorender=mujoco.Renderer(self.m,width=640,height=360) if args.scene=='rov' else None
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
            previous();self.tick+=1
            self.check_motion()
            forces=self.jaw_forces();self.force_window.append(forces)
            self.force_window=self.force_window[-100:]
            if self.tick%50==0:self.record('physics')
            if self.tick%100==0:self.video()
            if time.monotonic()-self.started>args.wall_seconds:raise TimeoutError('wall-clock cap')
        self.e.row_observer=observer
        self.record('start',condition=args.condition,seed=args.seed,scene=args.scene,physics='native fixed BlueROV and flange harness' if args.scene=='rov' else 'flange trial 2 with harness-selected thread hand',profile=args.profile,g3_grasp=args.g3_grasp,g3_seat_mm=args.g3_seat_mm,command_cap=args.command_cap,wall_seconds=args.wall_seconds,ik_joint_margin_deg=.25 if args.scene=='rov' or args.profile=='astra' else 2.,start_info=getattr(self.e,'start_info',{}))
        self.hold(1.)
        self.write_workspace()
    def pose(self,s):
        h=self.d.site(s+'_tcp');return h.xpos.copy(),h.xmat.reshape(3,3).copy()
    def jaw_forces(self):
        result={s:np.zeros(2) for s in self.e.sides};force=np.zeros(6)
        for i,c in enumerate(self.d.contact):
            mujoco.mj_contactForce(self.m,self.d,i,force)
            for s in self.e.sides:
                for j,ids in enumerate(self.e.grasps.jaws[s]):
                    # Sensed jaw load, not object identity or active-weld bookkeeping.
                    if (int(c.geom1) in ids) != (int(c.geom2) in ids):result[s][j]+=max(0.,force[0])
        return result
    def proprio(self):
        result={}
        for s in self.e.sides:
            p,R=self.pose(s);forces=np.mean([f[s] for f in self.force_window],axis=0) if self.force_window else np.zeros(2)
            result[s]=dict(tool_m=p.tolist(),approach=R[:,2].tolist(),jaw=R[:,0].tolist(),opening_mm=float(sum(self.d.joint(f'{s}_finger_{k}').qpos[0] for k in (-1,1))*1000),joints_rad=self.d.qpos[self.e.qadr[s]].tolist(),grasp='holding' if min(forces)>.5 else 'empty',jaw_load_N=forces.tolist())
        return dict(step=self.tick,sim_seconds=float(self.d.time),commands=self.count,command_cap=self.args.command_cap,wall_remaining_s=max(0.,self.args.wall_seconds-(time.monotonic()-self.started)),arms=result)
    def record(self,kind,**extra):
        row=self.measure.row(0,self.tick,{})
        row.update(kind=kind,step=self.tick,time=float(self.d.time),command=self.command,command_number=self.count,
            qactual={s:q.tolist() for s,q in self.e.state().items()},qreference={s:q.tolist() for s,q in self.q.items()},
            torques_Nm={s:self.d.actuator_force[self.e.aids[s]].tolist() for s in self.e.sides},
            jaw_drive_N={s:float(self.d.actuator_force[self.m.actuator(s+'_gripper_left_joint_drive').id]) for s in self.e.sides},
            jaw_integral_m=dict(self.grip_integral),
            tool={s:self.pose(s)[0].tolist() for s in self.e.sides},
            object_quat={part:self.d.body(part).xquat.tolist() for part in ('bolt','nut')},
            contact_pairs=[dict(geoms=[self.m.geom(int(g)).name for g in (c.geom1,c.geom2)],distance_m=float(c.dist)) for c in self.d.contact],
            thread_hand=self.args.thread_hand,thread_events=self.e.thread.events[self.thread_cursor:],**seating(self.e),**extra)
        state=getattr(self,'screw_state',None)
        if state is not None:
            row['screw_controller']=dict(phase=state['phase'],tracking=state['tracking'],degrees_turned=state['turned'],regrasp_cycles=state['cycles'],
                baseline_torque_Nm=state['baseline'],roll_torque_Nm=state['roll_torque'],roll_speed_deg_s=state['roll_speed'],snug_s=state['snug_s'])
            if state.get('backup_side') is not None:
                row['screw_controller'].update(speed_reference='relative hand roll (encoder/FK)',backup_hand=state['backup_side'],backup_degrees_turned=state['backup_turned'])
        self.thread_cursor=len(self.e.thread.events)
        if self.args.bolt_free_spin:
            row.update(bolt_turns=self.e.thread.bolt_turns,nut_world_turns=self.e.thread.nut_world_turns,
                       left_holds_bolt=self.e.grasps.active.get('left')=='bolt',
                       bearing_torque_Nm=self.e.thread.bearing_torque_nm,
                       thread_friction_Nm=self.e.thread.friction_nm,thread_spin_rad_s=self.e.thread.spin,
                       bolt_spin_rad_s=float(self.d.qvel[self.m.joint('bolt_free').dofadr[0]+5]))
        self.log.write(json.dumps(row,allow_nan=False)+'\n')
    def image(self,camera,depth=False):
        r=self.toprender if camera=='top' else self.render
        if depth:r.enable_depth_rendering()
        else:r.disable_depth_rendering()
        name=self.cameras.get(camera,camera);option=None
        if self.args.scene=='rov':
            from rov_scene import camera_options
            option=camera_options(name)
        r.update_scene(self.d,camera=name,scene_option=option);im=r.render().copy()
        if depth:r.disable_depth_rendering()
        return im[90:630] if camera=='top' else im
    def video(self):
        caption=f'{self.start_label} | engaged={self.e.thread.engaged} turns={self.e.thread.turns:.4f} advance={self.e.thread.advance*1000:.2f} mm seated={seating(self.e)["nut_seated"]} | grasps={self.e.grasps.active}'
        data=self.d;command=self.command
        if self.capture_video and self.video_export is None:
            # Exact render states at 10 Hz. Export after commands, so movie GPU
            # cost cannot consume the robot's 300 second screw watchdog.
            self.video_states.append((self.d.qpos.copy(),float(self.d.time),command,caption))
            return
        if self.video_export is not None:
            data,command,caption=self.video_export
        if self.args.scene=='rov':
            # Render at the actual delivered pane size; observations stay 960x540.
            ims=[]
            for camera in (('wide','flange_detail','right','left') if self.args.bolt_free_spin else ('wide','flange_detail','right')):
                from rov_scene import camera_options
                self.videorender.update_scene(data,camera=camera,scene_option=camera_options(camera))
                ims.append(self.videorender.render().copy())
        else:
            ims=[self.image('top'),self.image('flange_detail'),self.image('wrist_r')]
            ims[1]=np.asarray(Image.fromarray(ims[1]).crop((150,45,750,383)).resize((960,540)))
        frame=Image.new('RGB',(640*len(ims),400),'#14212b');dr=ImageDraw.Draw(frame)
        for i,(im,name) in enumerate(zip(ims,['ROV EXTERNAL' if self.args.scene=='rov' else 'TOP','FLANGE JOINT (inspection)','RIGHT WRIST','LEFT WRIST'])):
            frame.paste(Image.fromarray(im).resize((640,360)),(i*640,0));dr.text((i*640+8,7),name,fill='#ffcf70')
        dr.text((8,366),f't={data.time:.2f}s | {command[:100]}',fill='white')
        dr.text((8,383),caption,fill='white')
        self.writer.append_data(np.asarray(frame))
    def step(self):
        # Original 1 ms integration/arm servos, with an interface-side jaw PI
        # controller so a requested opening is reached under contact load. The
        # existing actuator force caps and all constraints remain unchanged.
        e=self.e;m=self.m;d=self.d
        velocity={s:(self.q[s][:6]-self.previous_reference[s])/DT for s in e.sides}
        e.gripper_command={s:float(self.q[s][6]) for s in e.sides}
        for _ in range(50):
            for s in e.sides:
                aids=e.aids[s]
                gain=np.ones(6);gain[5]=self.arm_stiffness[s];kp=m.actuator_gainprm[aids,0];kv=-m.actuator_biasprm[aids,2]
                error=self.q[s][:6]-d.qpos[e.qadr[s]]
                extra=(gain-1)*error
                if gain[5]>1:extra[5]+=(np.sqrt(gain[5])*kv[5]*velocity[s][5]-(np.sqrt(gain[5])-1)*kv[5]*d.qvel[e.vadr[s][5]])/kp[5]
                d.ctrl[aids]=self.q[s][:6]+self.correction[s]+extra+d.qfrc_bias[e.vadr[s]]/kp
                finger=m.actuator(s+'_gripper_left_joint_drive').id
                joints=[m.joint(f'{s}_finger_{sign}') for sign in (-1,1)]
                actual=np.mean([d.qpos[j.qposadr[0]] for j in joints]);target=self.q[s][6]*.0524125
                # A real gripper stops closing at current/contact, rather than
                # driving its pads deeply into an immobilized grasped part.
                if self.q[s][6]>.65:self.grip_latch[s]=None
                elif self.q[s][6]<=.45:
                    sensed=self.force_window[-1][s] if self.force_window else np.zeros(2)
                    if self.grip_latch[s] is None and min(sensed)>1.:
                        self.grip_latch[s]=max(0.,float(actual)-.001)
                    if self.grip_latch[s] is not None:target=max(target,self.grip_latch[s])
                if self.q[s][6]>.65:self.grip_integral[s]=float(np.clip(self.grip_integral[s]+10*(target-actual)*m.opt.timestep,0,.07))
                else:self.grip_integral[s]=0.
                d.ctrl[finger]=target+self.grip_integral[s]+sum(d.qfrc_bias[j.dofadr[0]] for j in joints)/m.actuator_gainprm[finger,0]
            mujoco.mj_step(m,d);e.row_observer()
        mujoco.mj_forward(m,d)
        for s in e.sides:
            error=self.q[s][:6]-d.qpos[e.qadr[s]]
            torque=d.actuator_force[e.aids[s]]
            error=np.where((np.abs(torque)>24.9)&(torque*error>0),0.,error)
            bias_limit=20./m.actuator_gainprm[e.aids[s],0]
            self.correction[s]=np.clip(self.correction[s]+.2*error,-bias_limit,bias_limit)
        self.previous_reference={s:self.q[s][:6].copy() for s in e.sides}
    def hold(self,seconds):
        for _ in range(max(1,math.ceil(seconds/DT))):self.step()
    def stop_and_hold(self):
        # Discard trajectory, integral bias and velocity feed-forward. Never set
        # physical joint/object state; the next command starts at measured joints.
        for s in self.e.sides:
            self.q[s][:6]=self.d.qpos[self.e.qadr[s]]
            self.previous_reference[s]=self.q[s][:6].copy()
            self.correction[s][:]=0.;self.arm_stiffness[s]=1.
            aids=self.e.aids[s]
            self.d.ctrl[aids]=self.q[s][:6]+self.d.qfrc_bias[self.e.vadr[s]]/self.m.actuator_gainprm[aids,0]
        mujoco.mj_forward(self.m,self.d)
    def motion_error(self):
        m=self.motion;p,R=self.pose(m['side'])
        return float(np.linalg.norm(p-m['target'])),float(Rotation.from_matrix(m['rotation'].T@R).magnitude())
    def check_motion(self):
        state=getattr(self,'screw_state',None)
        if state is not None and state['tracking']:
            _,R=self.pose(state['side'])
            delta=Rotation.from_matrix(R@state['previous_rotation'].T).as_rotvec()
            state['turned']+=float(np.degrees(delta@state['axis']))
            state['previous_rotation']=R
            sensed_roll=state['turned']
            if state.get('backup_side') is not None:
                _,backup_R=self.pose(state['backup_side'])
                backup_delta=Rotation.from_matrix(backup_R@state['previous_backup_rotation'].T).as_rotvec()
                state['backup_turned']+=float(np.degrees(backup_delta@state['axis']))
                state['previous_backup_rotation']=backup_R
                sensed_roll-=state['backup_turned']
            state['max_progress']=max(state['max_progress'],state['direction']*state['turned'])
            # Encoder/FK and actuator torque only. Learn the load while actually
            # rotating in the first 15 degrees of each stroke; retain the last
            # moving baseline if a subsequent stroke starts already blocked.
            now=float(self.d.time)
            state['samples'].append((now,sensed_roll,abs(float(self.d.actuator_force[self.e.aids[state['side']][5]]))))
            while len(state['samples'])>1 and state['samples'][1][0]<=now-.1:
                state['samples'].pop(0)
            samples=state['samples'];elapsed=now-samples[0][0]
            if elapsed>=.099:
                speed=state['direction']*(sensed_roll-samples[0][1])/elapsed
                torque=float(np.mean([sample[2] for sample in samples]))
                state['roll_speed']=speed;state['roll_torque']=torque
                progress=state['direction']*(state['turned']-state['stroke_start'])
                if 3.<progress<15. and speed>3.:
                    state['baseline_samples'].append(torque)
                    state['baseline']=float(np.median(state['baseline_samples']))
                high=state['baseline'] is not None and torque>=state['baseline']+SCREW_TORQUE_RISE_NM
                stopped=abs(speed)<SCREW_STOP_SPEED_DEG_S
                state['snug_s']=state['snug_s']+self.m.opt.timestep if high and stopped else 0.
                if state['snug_s']>=SCREW_SNUG_SECONDS:
                    raise CommandStopped('Roll stopped below 1 degree/s with wrist torque at least 1.5 Nm above the free-running baseline for 0.3 simulated seconds.')
        m=self.motion
        if m is None:return
        s=m['side'];sat=np.abs(self.d.actuator_force[self.e.aids[s]])>=24.9
        m['saturation_s']=np.where(sat,m['saturation_s']+self.m.opt.timestep,0.)
        m['saturated'].update(np.flatnonzero(sat).tolist())
        if np.max(m['saturation_s'])>=.5:raise CommandStopped('Arm torque saturated for 0.5 simulated seconds.')
        if self.tick%50:return
        pos,ang=self.motion_error();metric=pos+.05*ang
        if self.d.time-m['window_time']>=1.-1e-9:
            # Rotation counts as progress for a turn. A smooth joint path can
            # initially curve away from its target: only call that a stall when
            # the measured joints also lag their reference significantly.
            lag=float(np.max(np.abs(self.q[s][:6]-self.d.qpos[self.e.qadr[s]])))
            if (pos>.002 or ang>np.radians(3)) and m['window_error']-metric<.001 and lag>.05:
                raise CommandStopped('Less than 1 mm of target progress in 1 simulated second (rotation weighted at 50 mm/rad).')
            m['window_time']=float(self.d.time);m['window_error']=metric
    def motion_result(self):
        m=self.motion;p,R=self.pose(m['side']);pos,ang=self.motion_error()
        result=dict(reached_tool_m=p.tolist(),approach=R[:,2].tolist(),jaw=R[:,0].tolist(),residual_m=pos,
            orientation_residual_deg=float(np.degrees(ang)),ik_residual_m=m.get('ik_residual_m'),
            ik_orientation_residual_deg=m.get('ik_orientation_residual_deg'),ik_hit_limit=m.get('ik_hit_limit',False),
            saturated_joints=[self.m.joint(self.m.actuator_trnid[self.e.aids[m['side']][j],0]).name for j in sorted(m['saturated'])])
        if result['ik_residual_m'] is not None and (result['ik_residual_m']>.003 or result['ik_orientation_residual_deg']>3):
            result['message']='IK could not reach this target in the requested orientation within tolerance. Choose a closer target or change the approach/orientation; check IK residuals and joint limits.'
        elif result['ik_hit_limit']:
            result['message']='IK is at or near a joint limit; choose a closer target or change the approach/orientation.'
        return result
    def calibration(self,c):
        name=self.cameras[c];k=self.m.camera(name).id;f=(720 if c=='top' else 540)/(2*np.tan(np.radians(self.m.cam_fovy[k])/2))
        fx=fy=float(f);cx=479.5;cy=269.5
        sensor=self.m.cam_sensorsize[k];intrinsic=self.m.cam_intrinsic[k]
        if np.all(sensor>0):
            height=720 if c=='top' else 540
            fx=float(960*intrinsic[0]/sensor[0]);fy=float(height*intrinsic[1]/sensor[1])
            cx-=float(960*intrinsic[2]/sensor[0]);cy-=float(height*intrinsic[3]/sensor[1])
        return dict(width=960,height=540,fx=fx,fy=fy,cx=cx,cy=cy,position_world_m=self.d.cam_xpos[k].tolist(),R_world_from_camera=self.d.cam_xmat[k].reshape(3,3).tolist())
    def write_workspace(self):
        template=(ROOT/'astra_op/README_OPERATOR.md').read_text()
        begin='<!-- BACKUP PROFILE -->';end='<!-- END BACKUP PROFILE -->'
        a,b=template.split(begin);backup,c=b.split(end)
        template=a+(backup if self.args.bolt_free_spin else '')+c
        if self.args.scene=='rov':
            template=template.replace('(default 90, maximum 180)', '(default 180 in this ROV episode, maximum 180)')
            template=template.replace('You control two simulated Viscous arms, left and right.', 'You control a fixed, level BlueROV2 Heavy with two simulated Viscous arms, left and right, on its native mounts.')
            template=template.replace("World coordinates are metres: +Z is up, +X runs along the pipes toward the left arm's side, +Y points from the fixture toward the arm bases. The origin is the table scene's world origin.", 'World coordinates are metres in a fixed frame aligned with the level ROV: +X is forward out through the dome toward the work, +Y is toward the vehicle’s left, and +Z is up. The origin is directly below the ROV frame origin at the world zero-height datum; the camera calibration and hand poses use this same frame.')
            template=template.replace('`top`','`rov`').replace('_top.png','_rov.png').replace('obs/top.png','obs/rov.png')
            template=template.replace('Images retain the original observation camera poses and framing: top is the central 540 rows of a 960×720 render; wrist images use 960×540. The intrinsics below include the top crop.', 'The rov camera is inside the front dome, looking forward and down at the work. All three cameras render 960×540, with no crop.')
        if self.args.profile=='trial2':template=template.replace('Astra profile episode limits','Trial-2 profile episode limits')
        template=template.replace('1000 commands and 21600 seconds (6 hours)',f'{self.args.command_cap} commands and {self.args.wall_seconds:g} seconds')
        if self.args.thread_hand=='legacy':
            template=template.replace('For the right-hand 3.5 mm thread','For this legacy 3.5 mm thread')
            template=template.replace('`./robot turn right 30`','`./robot turn right -30`').replace('`./robot turn right 60 --feed-mm 0.583333`','`./robot turn right -60 --feed-mm 0.583333`')
            template=template.replace('`degrees * 3.5 / 360`','`-degrees * 3.5 / 360`')
            template=template.replace('With approach pointing from the bolt-tip side toward the flange, positive `turn` is clockwise when viewed from that side and positive feed moves toward the flange.','This legacy episode advances with negative turn and positive feed for a tip-side approach.')
            template=template.replace('The specified thread is right-hand with 3.5 mm pitch. With approach pointing from the bolt-tip side toward the flange, use positive `turn` values with positive feed to advance; negative turn and negative feed back the nut off.','This episode reproduces the legacy winding model with 3.5 mm pitch: for a tip-side approach, negative turn with positive feed advances, and positive turn with negative feed backs off.')
        calibration={c:self.calibration(c) for c in self.cameras}
        mounts={c:dict(body=self.m.body(self.m.cam_bodyid[self.m.camera(name).id]).name,position_m=self.m.cam_pos[self.m.camera(name).id].tolist(),quaternion_wxyz=self.m.cam_quat[self.m.camera(name).id].tolist()) for c,name in self.cameras.items()}
        (self.ws/'README.md').write_text(template+'\nCamera calibration at episode start (wrist extrinsics move with the hand; each `look` reports current calibration):\n```json\n'+json.dumps(dict(cameras=calibration,mounts=mounts),indent=2)+'\n```\n')
        wrapper=(ROOT/'astra_op/robot.py').read_text().replace("'__SOCKET__'",repr(str(self.out/'robot.sock')))
        (self.ws/'robot').write_text(wrapper);(self.ws/'robot').chmod(0o755)
    def solve(self,s,p,R,seed):
        # Native client IK is authoritative. FK-only local refinement seeds it when
        # the trajectory regularizer leaves a small residual; no live-state edits.
        sol=self.sol;idx=sol.sides.index(s);q0=np.array([self.q[x][:6] for x in sol.sides]);q0[idx]=seed
        b=self.d.body(s+'_base_link');B=b.xmat.reshape(3,3);target=np.eye(4);target[:3,3]=B.T@(p-b.xpos);target[:3,:3]=B.T@R
        targets=np.stack([np.repeat(sol.fk(q0[i:i+1],x),48,axis=0) for i,x in enumerate(sol.sides)])
        targets[idx]=target
        def residual(candidate):
            self.scratch.qpos[:]=self.d.qpos
            self.scratch.qpos[self.e.qadr[s]]=candidate
            mujoco.mj_kinematics(self.m,self.scratch)
            h=self.scratch.site(s+'_tcp')
            return np.r_[h.xpos-p,.15*Rotation.from_matrix(R.T@h.xmat.reshape(3,3)).as_rotvec()]
        lim=np.stack([sol.side_lo[idx],sol.side_hi[idx]],axis=1)
        fit=least_squares(residual,np.clip(seed,lim[:,0]+1e-7,lim[:,1]-1e-7),bounds=(lim[:,0],lim[:,1]),max_nfev=60)
        if np.linalg.norm(fit.fun[:3])>.0005 or np.linalg.norm(fit.fun[3:])/.15>.01:
            rng=np.random.default_rng(23)
            fits=[fit]
            for guess in [sol.side_rest[idx],*rng.uniform(lim[:,0],lim[:,1],size=(10,6))]:
                f=least_squares(residual,np.clip(guess,lim[:,0]+1e-7,lim[:,1]-1e-7),bounds=(lim[:,0],lim[:,1]),max_nfev=80)
                fits.append(f)
            precise=[f for f in fits if np.linalg.norm(f.fun)<.0002]
            fit=min(precise,key=lambda f:np.linalg.norm(f.x-seed)) if precise else min(fits,key=lambda f:np.linalg.norm(f.fun))
        q0[idx]=fit.x
        result=sol.solve(targets,q0);q=np.asarray(result.q_solution[idx,-1]).copy()
        fk=sol.fk(q[None],s)[0];err=np.r_[fk[:3,3]-target[:3,3],.15*Rotation.from_matrix(target[:3,:3].T@fk[:3,:3]).as_rotvec()]
        return q,err
    def move(self,s,p,R,cartesian=False,rate_deg_s=None):
        p=np.asarray(p,float);p0,R0=self.pose(s)
        self.motion=dict(side=s,target=p.copy(),rotation=R.copy(),saturation_s=np.zeros(6),saturated=set(),
            window_time=float(self.d.time),window_error=float(np.linalg.norm(p-p0)+.05*Rotation.from_matrix(R.T@R0).magnitude()))
        angle=Rotation.from_matrix(R@R0.T).magnitude()
        open_hand=self.q[s][6]>.65
        angular_speed=np.radians(rate_deg_s if rate_deg_s is not None else 180 if open_hand else 25)
        joint_speed=3.5 if open_hand else max(.5,min(3.5,1.15*angular_speed)) if rate_deg_s is not None else .5
        linear_speed=.04
        fast_turn=rate_deg_s is not None and rate_deg_s>25
        smooth_factor=1.5 if fast_turn else 2.25
        duration=max(.4,smooth_factor*np.linalg.norm(p-p0)/linear_speed,smooth_factor*angle/angular_speed)
        if not cartesian:
            seed=self.q[s][:6].copy();goal,err=self.solve(s,p,R,seed)
            limited=bool(np.any(np.minimum(goal-self.sol.side_lo[self.sol.sides.index(s)],self.sol.side_hi[self.sol.sides.index(s)]-goal)<.005) or np.linalg.norm(err[:3])>.003 or np.linalg.norm(err[3:])/.15>np.radians(3))
            self.motion.update(ik_residual_m=float(np.linalg.norm(err[:3])),ik_orientation_residual_deg=float(np.degrees(np.linalg.norm(err[3:])/.15)),ik_hit_limit=limited)
            # Bound speed along the actual FK path, which can curve in world space.
            sol=self.sol;prev=sol.fk(seed[None],s)[0];length=0.;rotation=0.
            for t in np.linspace(0,1,101)[1:]:
                cur=sol.fk((seed+t*(goal-seed))[None],s)[0]
                length=max(length,float(np.linalg.norm(cur[:3,3]-prev[:3,3]))*100)
                rotation=max(rotation,float(Rotation.from_matrix(cur[:3,:3]@prev[:3,:3].T).magnitude())*100);prev=cur
            seconds=max(.4,1.5*length/linear_speed,1.5*rotation/angular_speed,1.5*np.max(np.abs(goal-seed))/joint_speed)
            for t in fractions(math.ceil(seconds/DT)):
                self.q[s][:6]=seed+(t*t*(3-2*t))*(goal-seed);self.step()
            self.hold(.05 if open_hand else .8)
            return self.motion_result()
        sl=Slerp([0,1],Rotation.from_matrix(np.stack([R0,R])))
        seed=self.q[s][:6].copy();worst=0.;limited=False
        # IK waypoints at <=2 mm / 2 degrees. Joint trajectories remain bounded
        # even on an unreachable Cartesian path; there is no branch teleport.
        count=max(2,math.ceil(duration/.15))
        for u in fractions(count):
            smooth=u*u*(3-2*u);pt=p0+smooth*(p-p0);rot=sl(smooth).as_matrix()
            q,err=self.solve(s,pt,rot,seed);worst=max(worst,float(np.linalg.norm(err[:3])))
            limited|=bool(np.any(np.minimum(q-self.ik.limits[s][:,0],self.ik.limits[s][:,1]-q)<.02) or np.linalg.norm(err[:3])>.003 or np.linalg.norm(err[3:])/.15>np.radians(3))
            self.motion.update(ik_residual_m=worst,ik_orientation_residual_deg=max(self.motion.get('ik_orientation_residual_deg',0.),float(np.degrees(np.linalg.norm(err[3:])/.15))),ik_hit_limit=limited)
            n=max(1,math.ceil((duration/count)/DT),math.ceil((1. if fast_turn else 1.5)*float(np.max(np.abs(q-seed)))/(joint_speed*DT)))
            for v in fractions(n):
                self.q[s][:6]=seed+(v if fast_turn else v*v*(3-2*v))*(q-seed);self.step()
            seed=q
        self.hold(.05 if open_hand or fast_turn else .6)
        return self.motion_result()
    def set_grip(self,s,g):
        # Shared contact-stop/preload and release controller used by grip/screw.
        if g<=.45:self.grip_latch[s]=None
        old=self.q[s][6]
        for t in np.linspace(0,1,11)[1:]:
            next_opening=old+t*(g-old)
            if self.q[s][6]<=.65<next_opening:
                # The held part's external load disappears at the release gate.
                # Carrying its learned arm bias into free space kicks the hand
                # away from its reference, especially with a tilted grasp.
                self.correction[s][:]=0.
            self.q[s][6]=next_opening;self.step()
        self.hold(.4)
        if g>.65:
            for _ in range(60):
                opening=self.proprio()['arms'][s]['opening_mm']/1000
                if abs(opening-g*.104825)<.001:break
                self.step()
            self.hold(.15)
        # A fresh bilateral contact stop plus a nonzero gap is the part
        # detection. One pad can unload after a rigid grasp settles.
        return self.grip_latch[s] is not None and self.proprio()['arms'][s]['opening_mm']>5.

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
                        target=p0+axis*(commanded*feed_sign*3.5/360000.)
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
                    target=p0+axis*((commanded+chunk)*feed_sign*3.5/360000.)
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

    def execute(self,argv,progress=lambda _:None):
        if argv==['last']:
            self.record('last_result')
            return dict(ok=True,previous_command=self.last_result)
        self.motion=None;started=time.monotonic()
        limit=SCREW_SECONDS if argv and argv[0]=='screw' else COMMAND_SECONDS
        old_handler=signal.getsignal(signal.SIGALRM)
        def heartbeat(*_):
            elapsed=time.monotonic()-started
            if elapsed>=limit:raise CommandStopped(f'Command reached the {limit:g} s wall-time limit.',timed_out=True)
            progress(dict(progress=True,command_number=self.count,command=self.command,elapsed_wall_s=round(elapsed,1),sim_seconds=float(self.d.time),message='Command is alive; safe to wait.'))
            signal.setitimer(signal.ITIMER_REAL,min(PROGRESS_SECONDS,max(.001,limit-(time.monotonic()-started))))
        signal.signal(signal.SIGALRM,heartbeat)
        signal.setitimer(signal.ITIMER_REAL,min(PROGRESS_SECONDS,limit))
        try:
            try:result=self._execute(argv)
            except CommandStopped as ex:
                signal.setitimer(signal.ITIMER_REAL,0)
                self.stop_and_hold()
                result=dict(ok=True,stalled=not ex.timed_out,timed_out=ex.timed_out,stop_reason=str(ex),holding_current_pose=True,
                    **(self.motion_result() if self.motion else self.proprio()))
                self.record('command_end',response=result)
        finally:
            signal.setitimer(signal.ITIMER_REAL,0);signal.signal(signal.SIGALRM,old_handler);self.motion=None
        result.update(elapsed_wall_s=time.monotonic()-started)
        self.last_result=dict(command_number=self.count,argv=list(argv),result=result)
        self.record('command_result',previous_command=self.last_result)
        return result
    def _execute(self,argv):
        self.count+=1;self.command=' '.join(argv)
        self.record('command_start',argv=argv)
        try:
            if not argv:raise ValueError('command required')
            op=argv[0];a=argv[1:]
            if op in ('look','status'):
                if a:raise ValueError('no arguments expected')
                self.hold(.1);result=self.proprio()
                if op=='look':
                    files={}
                    for c in self.cameras:
                        filename=f'{self.tick:08d}_{c}.png';Image.fromarray(self.image(c)).save(self.ws/'obs'/filename)
                        shutil.copyfile(self.ws/'obs'/filename,self.ws/'obs'/(c+'.png'));files[c]='obs/'+filename
                    result.update(images=files,cameras={c:self.calibration(c) for c in self.cameras})
            elif op=='point':
                if len(a)!=3 or a[0] not in self.cameras:raise ValueError('point '+ '|'.join(self.cameras)+' u v')
                c=a[0];u,v=map(int,a[1:])
                if not(0<=u<960 and 0<=v<540):raise ValueError('pixel outside 960x540')
                self.hold(.05);depth=self.image(c,True);z=float(depth[v,u]);cal=self.calibration(c)
                if not np.isfinite(z) or z>=self.m.vis.map.zfar*self.m.stat.extent*.99:raise ValueError('no visible depth at pixel')
                point=np.array(cal['position_world_m'])+np.array(cal['R_world_from_camera'])@np.array([(u-cal['cx'])*z/cal['fx'],-(v-cal['cy'])*z/cal['fy'],-z])
                result=dict(camera=c,pixel=[u,v],world_m=point.tolist(),depth_m=z,step=self.tick)
                direction=point-np.array(cal['position_world_m']);direction/=np.linalg.norm(direction)
                geom=np.array([-1],dtype=np.int32)
                distance=mujoco.mj_ray(self.m,self.d,np.array(cal['position_world_m']),direction,np.array([1,1,1,0,int(self.args.scene=='rov' and c!='rov'),0],dtype=np.uint8),1,-1,geom)
                truth=np.array(cal['position_world_m'])+direction*distance
                self.record('depth_validation',ray_world_m=truth.tolist(),depth_world_m=point.tolist(),error_m=float(np.linalg.norm(truth-point)),geom_id=int(geom[0]))
            elif op in ('move','nudge','turn','grip','screw'):
                if not a or a[0] not in self.e.sides:raise ValueError('arm must be left or right')
                s=a[0];values=a[1:];p,R=self.pose(s)
                if op!='turn':self.arm_stiffness[s]=1.
                if op=='grip':
                    if len(values)!=1:raise ValueError('grip arm open|close|0..1')
                    g={'open':1.,'close':0.}.get(values[0])
                    if g is None:g=float(values[0])
                    if not np.isfinite(g) or not 0<=g<=1:raise ValueError('grip must be 0..1')
                    self.set_grip(s,g)
                    result=self.proprio()
                elif op=='screw':
                    if len(values) not in (1,3) or (len(values)==3 and values[1]!='--rate-deg-s'):
                        raise ValueError('screw arm degrees [--rate-deg-s R]')
                    degrees=float(values[0]);rate=float(values[2]) if len(values)==3 else (180. if self.args.scene=='rov' else 90.)
                    if not np.isfinite(degrees):raise ValueError('finite degrees required')
                    if not np.isfinite(rate) or not 0<rate<=180:raise ValueError('screw rate must be >0 and <=180 degrees/s')
                    result=self.screw(s,degrees,rate)
                elif op=='turn':
                    if not values or len(values)%2!=1:raise ValueError('turn arm degrees [--feed-mm N] [--rate-deg-s N]')
                    opts={}
                    for flag,value in zip(values[1::2],values[2::2]):
                        if flag not in ('--feed-mm','--rate-deg-s') or flag in opts:raise ValueError('unknown or duplicate turn option')
                        opts[flag]=float(value)
                    degrees=float(values[0]);feed=opts.get('--feed-mm',0.);rate=opts.get('--rate-deg-s')
                    if not np.isfinite(degrees) or abs(degrees)>180:raise ValueError('turn bounded to +/-180 degrees; regrasp at wrist limit')
                    if not np.isfinite(feed) or abs(feed)>10:raise ValueError('feed must be within +/-10 mm')
                    if rate is not None and (not np.isfinite(rate) or not 0<rate<=180):raise ValueError('turn rate must be >0 and <=180 degrees/s')
                    self.arm_stiffness[s]=20. if rate is not None and rate>25 else 1.
                    p=p+R[:,2]*feed/1000
                    R=Rotation.from_rotvec(R[:,2]*np.radians(degrees)).as_matrix()@R
                    result=self.move(s,p,R,cartesian=True,rate_deg_s=rate)
                else:
                    if len(values) not in ((3,6) if op=='move' else (3,)):raise ValueError('move arm x y z [ax ay az] / nudge arm dx dy dz')
                    vals=np.array(values,float)
                    if not np.isfinite(vals).all():raise ValueError('finite values required')
                    target=vals[:3] if op=='move' else p+vals[:3]/1000
                    if np.linalg.norm(target-p)>1.5:raise ValueError('target too far; use metres')
                    if len(vals)==6:
                        axis=vals[3:];norm=np.linalg.norm(axis)
                        if norm<1e-8:raise ValueError('approach must be nonzero')
                        axis=axis/norm;cross=np.cross(R[:,2],axis);ang=np.arctan2(np.linalg.norm(cross),np.dot(R[:,2],axis))
                        if np.linalg.norm(cross)<1e-8:cross=R[:,0]
                        R=Rotation.from_rotvec(cross/np.linalg.norm(cross)*ang).as_matrix()@R
                    result=self.move(s,target,R,cartesian=(op=="nudge"))
            elif op=='done':
                if len(a)!=1 or '\n' in a[0] or '\r' in a[0] or len(a[0])>500:raise ValueError('done "one-line claim"')
                self.hold(.1);self.ended=True;self.reason='operator done';result=dict(ended=True,claim=a[0],step=self.tick)
            else:raise ValueError('unknown command')
            self.record('command_end',response=result)
            return dict(ok=True,**result)
        except (ValueError,OverflowError) as ex:
            self.hold(.05);self.record('command_error',error=str(ex));return dict(ok=False,error=str(ex))
    def close(self,reason):
        self.reason=reason
        if hasattr(self,'writer'):
            self.record('end',reason=reason,torque_summary=self.measure.torque_summary());self.video()
            if self.capture_video:
                np.savez_compressed(self.out/'video_states.npz',qpos=np.stack([x[0] for x in self.video_states]),time=np.array([x[1] for x in self.video_states]))
                data=mujoco.MjData(self.m)
                states=self.video_states if self.args.video_export=='render' else []
                for index,(qpos,t,command,caption) in enumerate(states):
                    data.qpos[:]=qpos;data.time=t;mujoco.mj_forward(self.m,data)
                    self.video_export=(data,command,caption);self.video()
                    if index%500==0:print(json.dumps(dict(video_export_frames=index,video_export_total=len(self.video_states))),flush=True)
                self.video_export=None
            self.writer.close();self.render.close();self.toprender.close()
            if self.videorender is not None:self.videorender.close()
        if hasattr(self,'e'):self.e.close()
        if hasattr(self,'log'):self.log.close()

def parse_args(argv=None):
    p=argparse.ArgumentParser();p.add_argument('--episode',required=True);p.add_argument('--condition',choices=['G3','G1','T'],default='G3');p.add_argument('--seed',type=int,default=20261024);p.add_argument('--output',required=True)
    p.add_argument('--profile',choices=['trial2','astra','astra_rov','astra_rov_backup','astra_rov_backup2'],default='trial2')
    p.add_argument('--bolt-free-spin',action=argparse.BooleanOptionalAction,default=None,help='axially retained bolt can turn in clearance holes (ROV G3 only)')
    p.add_argument('--scene',choices=['table','rov'],default=None)
    p.add_argument('--video-export',choices=['render','capture'],default='render',help='render movie, or retain exact states only for diagnostics')
    p.add_argument('--g3-seat-mm',type=float,default=None)
    p.add_argument('--thread-hand',choices=['right','legacy'],default=None)
    p.add_argument('--g3-grasp',choices=['flats','stock'],default=None)
    p.add_argument('--command-cap',type=int,default=None);p.add_argument('--wall-seconds',type=float,default=None)
    a=resolve_profile(p.parse_args(argv))
    if a.scene is None:a.scene='rov' if a.profile in ('astra_rov','astra_rov_backup','astra_rov_backup2') else 'table'
    if a.profile in ('astra_rov','astra_rov_backup','astra_rov_backup2') and a.scene!='rov':p.error('ROV profiles require --scene rov')
    if a.bolt_free_spin and (a.scene!='rov' or a.thread_hand!='right'):p.error('--bolt-free-spin requires ROV G3 with right-hand thread')
    if a.scene=='rov' and (a.condition!='G3' or a.g3_seat_mm!=30 or a.g3_grasp!='flats'):p.error('ROV requires G3, 30 mm extra insertion and flats grasp')
    if not np.isfinite(a.g3_seat_mm) or not 0<=a.g3_seat_mm<=40 or (a.g3_seat_mm and a.condition!='G3'):p.error('--g3-seat-mm requires G3 and a value in 0..40')
    if a.command_cap<1 or not np.isfinite(a.wall_seconds) or a.wall_seconds<=0:p.error('positive caps required')
    if '/' in a.episode or a.episode in ('.','..'):p.error('episode must be one directory name')
    return a

def serve_client(ep,conn):
    """One bounded request; transport failures never own the episode lifetime."""
    connected=True;problems=[]
    def problem(phase,ex):
        # Progress can run in SIGALRM while the main code is writing telemetry.
        # Defer logging to avoid re-entering the buffered episode log in a signal.
        problems.append(dict(phase=phase,error=type(ex).__name__,detail=str(ex)))
    def log_problems():
        for fields in problems:ep.record('client_problem',**fields)
    def send(result):
        nonlocal connected
        if not connected:return
        try:
            conn.settimeout(.2)
            conn.sendall((json.dumps(result,allow_nan=False)+'\n').encode())
        except OSError as ex:
            connected=False;problem('send',ex)
    try:
        conn.settimeout(5.);raw=b''
        while b'\n' not in raw:
            chunk=conn.recv(4096)
            if not chunk:raise ConnectionError('client disconnected before a complete request')
            raw+=chunk
            if len(raw)>8192:raise ValueError('request exceeds 8192 bytes')
        argv=json.loads(raw)
        if not isinstance(argv,list) or not all(isinstance(x,str) for x in argv):raise ValueError('expected argument list')
    except (OSError,ValueError,RecursionError) as ex:
        problem('receive',ex);send(dict(ok=False,error=str(ex)));log_problems();return
    result=ep.execute(argv,progress=send)
    send(result)
    log_problems()

def main():
    a=parse_args()
    guard();ep=None;sock=None
    def stop(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,stop)
    try:
        ep=Episode.__new__(Episode);ep.__init__(a);path=ep.out/'robot.sock'
        sock=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);sock.bind(str(path));sock.listen(4)
        print(json.dumps(dict(ready=True,workspace=str(ep.ws),socket=str(path))),flush=True)
        while not ep.ended:
            remaining=a.wall_seconds-(time.monotonic()-ep.started)
            if remaining<=0:ep.reason='wall-clock cap';break
            sock.settimeout(min(remaining,1.))
            try:conn,_=sock.accept()
            except socket.timeout:continue
            except (ConnectionAbortedError,ConnectionResetError) as ex:
                ep.record('client_problem',phase='accept',error=type(ex).__name__,detail=str(ex));continue
            with conn:
                serve_client(ep,conn)
            if ep.count>=a.command_cap:ep.reason='command cap';break
    except (TimeoutError,KeyboardInterrupt) as ex:
        if ep:ep.reason=str(ex) or 'interrupted'
    finally:
        if ep:ep.close(getattr(ep,'reason',None) or 'server stopped')
        if sock:sock.close();(ep.out/'robot.sock').unlink(missing_ok=True)

if __name__=='__main__':main()
