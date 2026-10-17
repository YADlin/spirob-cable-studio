"""Reproducible plans and isolated MuJoCo motion previews. No hardware I/O."""
import copy
import hashlib
import json
import math
import random
from pathlib import Path
import mujoco
import numpy as np
from spirob_analysis.model import SpiRobModel
from motion_profile import validate_profile


class PreviewCancelled(Exception):pass


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def model_hashes(xml):
    root=Path(xml).resolve().parent
    return {p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(root.rglob('*')) if p.is_file() and p.suffix.lower() in ('.xml','.stl','.obj','.msh','.png')}


def mixed_difference_range(profile,ranges,mean):
    """Full-D interval at the chosen mean, inside both requested cable ranges."""
    if not isinstance(mean,(int,float)) or not math.isfinite(mean):raise ValueError('Provide a finite difference-mode mean.')
    low=max(-profile['difference_mm'],2*(ranges[0][0]-mean),2*(mean-ranges[1][1]))
    high=min(profile['difference_mm'],2*(ranges[0][1]-mean),2*(mean-ranges[1][0]))
    if low>high:raise ValueError('This mean has no difference targets inside both cable ranges. Adjust the mean or ranges.')
    return [low,high]


def random_plan(profile,mode,ranges,speeds,poses,seed,mean=None,linked=True,difference_range=None):
    """Seeded measurement rows; expand_plan adds the final return separately.

    Mixed drafts start with a two-cable target, so later one-cable moves keep
    the other cable within the requested ranges. Every block of four covers
    all four modes; the remaining order and all targets/speeds are random.
    """
    p=validate_profile(profile)
    if isinstance(poses,bool) or int(poses)!=poses or not 2<=poses<=200:raise ValueError('Choose 2..200 poses.')
    poses=int(poses)
    if not isinstance(seed,int) or isinstance(seed,bool) or not 0<=seed<=2147483647:raise ValueError('Seed must be an integer 0..2147483647.')
    if len(speeds)!=2 or not all(math.isfinite(v) for v in speeds) or not .05<=speeds[0]<=speeds[1]<=p['max_speed_mm_s']:
        raise ValueError('Speed range must lie within the session speed ceiling.')
    if mode not in ('Mixed','Both cables','Difference'):raise ValueError('Choose Mixed, Both cables or Difference.')
    if len(ranges)!=(1 if mode=='Difference' else 2) or any(len(b)!=2 or not all(math.isfinite(v) for v in b) or b[0]>b[1] for b in ranges):
        raise ValueError('Provide ordered finite motion ranges.')
    from actuation import targets,independent_targets
    if mode=='Difference':
        for d in ranges[0]:targets(mean,d,speeds[0],p['rest_lengths_mm'],p)
    else:
        independent_targets([b[0] for b in ranges],[speeds[0]]*2,p['rest_lengths_mm'],p)
        independent_targets([b[1] for b in ranges],[speeds[1]]*2,p['rest_lengths_mm'],p)
    if mode=='Mixed':
        if poses<4:raise ValueError('Mixed campaigns need at least 4 measurement poses to include every motion mode.')
        allowed=mixed_difference_range(p,ranges,mean)
        if difference_range is None:difference_range=allowed
        if (len(difference_range)!=2 or not all(math.isfinite(v) for v in difference_range) or
            difference_range[0]>difference_range[1] or difference_range[0]<allowed[0]-1e-9 or difference_range[1]>allowed[1]+1e-9):
            raise ValueError(f'Mixed difference range must be within {allowed[0]:.3f}..{allowed[1]:.3f} mm at this mean, so both cables stay in their selected ranges.')
        # Normalize only floating-point roundoff at an otherwise valid bound.
        difference_range=[min(allowed[1],max(allowed[0],d)) for d in difference_range]
        for d in difference_range:targets(mean,d,speeds[0],p['rest_lengths_mm'],p)
    rng=random.Random(seed);rows=[]
    modes=[mode]*poses
    if mode=='Mixed':
        tail=['Difference','Cable 1','Cable 2'];rng.shuffle(tail);modes=['Both cables',*tail]
        while len(modes)<poses:
            block=['Difference','Cable 1','Cable 2','Both cables'];rng.shuffle(block);modes.extend(block)
    for i,current in enumerate(modes[:poses]):
        bands=[difference_range] if current=='Difference' and mode=='Mixed' else ranges
        values=[rng.uniform(*b) for b in bands];s1=rng.uniform(*speeds)
        s2=s1 if linked or current=='Difference' else rng.uniform(*speeds)
        rows.append(dict(mode=current,x=values[0],y=mean if current=='Difference' else values[1],s1=s1,s2=s2,
                         steps=1,label=f'Random {i+1} · seed {seed}'))
    return rows


def difference_sweep(profile,start,end,mean,speed,intervals):
    """Capture the first extreme, then each evenly spaced interval to the other."""
    from actuation import targets
    p=validate_profile(profile)
    if isinstance(intervals,bool) or int(intervals)!=intervals or not 1<=intervals<=100:
        raise ValueError('Choose 1..100 sweep intervals.')
    targets(mean,start,speed,p['rest_lengths_mm'],p);targets(mean,end,speed,p['rest_lengths_mm'],p)
    if start==end:raise ValueError('Choose two different sweep extremes.')
    return [dict(mode='Difference',x=d,y=mean,s1=speed,s2=speed,steps=n,label=label)
            for d,n,label in [(start,1,'Sweep start extreme'),(end,int(intervals),'Difference sweep')]]


def review_context(rows,dwell,profile,xml,parameters,d2,setup,return_speed_mm_s=1.,settle_s=2.):
    return dict(rows=copy.deepcopy(rows),dwell_s=dwell,profile=validate_profile(profile),model_hashes=model_hashes(xml),
                parameters=copy.deepcopy(parameters),d2=d2,setup=copy.deepcopy(setup),settle_s=settle_s,
                return_to_start=dict(speed_mm_s=return_speed_mm_s,on='normal completion only'))


def require_review(review,context,initial):
    if not review or review['fingerprint']!=digest(context):
        raise ValueError('Plan, model, mapping or motion settings changed. Simulate and approve the current plan first.')
    if max(abs(a-b) for a,b in zip(initial,review['plan']['initial_lengths_mm']))>.5:
        raise ValueError('Starting cable lengths differ from the preview by more than 0.5 mm. Re-preview from the current held pose.')
    return copy.deepcopy(review['plan'])


def simulate(plan,xml,parameters,d2,setup,cancel=None,progress=None):
    """Integrate the original model timestep, with speed-limited cable commands.

    Motor ramp/serial timing, backlash and load-dependent compliance are not
    modelled. The simulated initial pose is conditioned for five seconds.
    """
    model=SpiRobModel(xml);m,d=model.m,model.d
    model.set_stiffness_profile(parameters.get('stiffness_ratio',model.rk),parameters.get('stiffness_exponent',1.))
    model.scale(parameters['k2'],d2)
    rests=np.asarray(plan['rest_lengths_mm']);mapping=setup.get('mapping',[0,1]);signs=setup.get('signs',[1,1])
    if sorted(mapping)!=[0,1] or len(signs)!=2 or any(s not in (-1,1) for s in signs):raise ValueError('Invalid cable mapping.')
    def controls(lengths):
        return model.controls((np.asarray(lengths)-rests)[mapping]*signs,parameters.get('mean_offset_mm',0),parameters.get('difference_offset_mm',0))
    initial=np.asarray(plan['initial_lengths_mm'],dtype=float)
    controls(initial)
    for pose in plan['poses']:controls(pose['lengths_mm'])
    total=5+sum(p['travel_s']+p['settle_s']+p['dwell_s'] for p in plan['poses'])
    if total>3600:raise ValueError('Preview exceeds one hour of simulated motion. Increase speed or split the plan.')
    if not plan['poses']:raise ValueError('The plan contains no poses.')
    # Small control intervals preserve specified rates; frames are decimated,
    # never the dynamics. A one-hour preview stores at most about 6200 frames.
    batch=max(1,int(round(.01/m.opt.timestep)));period=batch*m.opt.timestep
    frame_period=max(.05,total/6000);next_frame=0.;frames=[];samples=[]
    tendon_ids=m.actuator_trnid[:,0];lo=np.full(2,np.inf);hi=-lo.copy();contacts=0
    checked_time=-1.;previous_warning=np.array(d.warning.number,copy=True)
    def advance(lengths,pose_number,record=True):
        nonlocal next_frame,checked_time,contacts
        if cancel is not None and cancel.is_set():raise PreviewCancelled('Simulation cancelled; no hardware motion was requested.')
        d.ctrl[:]=controls(lengths)
        before=d.time;mujoco.mj_step(m,d,nstep=batch)
        if d.time<before or not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all() or np.any(d.warning.number!=previous_warning):
            raise ValueError('MuJoCo reported a numerical warning. Review model/parameters and narrow the journey.')
        # Update derived positions at the final integrated state.
        mujoco.mj_forward(m,d)
        measured=d.ten_length[tendon_ids]*1000;lo[:]=np.minimum(lo,measured);hi[:]=np.maximum(hi,measured)
        contacts=max(contacts,int(d.ncon))
        if record and d.time>=next_frame:
            frames.append(dict(t_s=float(d.time-5),pose=pose_number,lengths_mm=np.asarray(lengths).tolist(),
                tendon_mm=measured.tolist(),curve_mm=(1000*np.vstack([model.base_point(),model.joint_curve()])).tolist()))
            next_frame=d.time+frame_period
        if progress and d.time-checked_time>=.5:
            checked_time=d.time;progress(min(1.,d.time/total),pose_number,len(plan['poses']))
    for _ in range(int(math.ceil(5/period))):advance(initial,0,False)
    previous=initial.copy()
    for pose in plan['poses']:
        target=np.asarray(pose['lengths_mm']);speed=np.asarray(pose['speeds_mm_s'])
        delta=target-previous;travel=float(np.max(np.abs(delta)/speed))
        for step in range(max(1,int(math.ceil(travel/period)))):
            lengths=previous+np.sign(delta)*np.minimum(np.abs(delta),speed*min(travel,(step+1)*period))
            advance(lengths,pose['number'])
        for _ in range(int(math.ceil((pose['settle_s']+pose['dwell_s'])/period))):advance(target,pose['number'])
        endpoint=dict(number=pose['number'],command_mm=target.tolist(),control_mm=(d.ctrl*1000).tolist(),
            role=pose.get('role','measurement'),label=pose.get('label',''),
            tendon_mm=(d.ten_length[tendon_ids]*1000).tolist(),max_joint_speed_rad_s=float(np.max(np.abs(d.qvel))),contacts=int(d.ncon))
        samples.append(endpoint);previous=target
    return dict(frames=frames,summary=dict(joints=m.njnt,neutral_tendon_mm=(model.neutral_lengths*1000).tolist(),
        tendon_min_mm=lo.tolist(),tendon_max_mm=hi.tolist(),max_contacts=contacts,poses=samples,
        simulation_seconds=float(d.time),mujoco_version=mujoco.__version__,timestep_s=float(m.opt.timestep),
        initialization='Five seconds at the initial commands; actual robot history and motor ramps can differ.',
        return_to_start=copy.deepcopy(plan.get('return_to_start')),
        meaning='Observed ten_length range for this journey, not a hardware travel rating or guaranteed reachable range.'))
