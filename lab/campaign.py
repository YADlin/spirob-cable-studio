"""Workstation campaign planning and nonblocking, acknowledged motion sequencing.

Uses the existing motor owner, limits, watchdog and Stop path. Never enables motors.
No image fitting or equilibrium solve runs in the acquisition state machine.
"""
import copy
import math
import time
from collections import deque
from actuation import targets, independent_targets
from analysis import inputs_at

MODES = ('Difference', 'Both cables', 'Cable 1', 'Cable 2')


def default_plan(rests=(220., 220.)):
    """Broad initial qualification, not the untested full travel envelope."""
    a,b=rests;mean=(a+b)/2;zero=a-b
    rows=[]
    def add(mode,x,y,s1=1.,s2=1.,steps=1,label=''):
        rows.append(dict(mode=mode,x=float(x),y=float(y),s1=s1,s2=s2,steps=steps,label=label))
    for d,n,label in [(0,1,'Initial reference'),(40,15,'Positive loading'),(-40,15,'Reverse through reference'),
                       (40,15,'Forward repeat'),(0,15,'Return reference')]:
        add('Difference',zero+d,mean,steps=n,label=label)
    for mode,x,y,label in [('Cable 1',a-20,b,'Cable 1 shorten'),('Cable 1',a+20,b,'Cable 1 extend'),
        ('Cable 1',a,b,'Reference after cable 1'),('Cable 2',a,b-20,'Cable 2 shorten'),
        ('Cable 2',a,b+20,'Cable 2 extend'),('Cable 2',a,b,'Reference after cable 2'),
        ('Both cables',a-10,b-10,'Common shortening'),('Both cables',a+10,b+10,'Common extension'),
        ('Both cables',a-15,b+10,'Unequal lengths'),('Both cables',a+10,b-15,'Reverse unequal lengths'),
        ('Both cables',a,b,'Final reference'),('Both cables',a,b,'Repeated final reference')]:
        add(mode,x,y,label=label)
    # Demonstrates separate speeds without confounding the main difference map.
    rows[-4]['s2']=.5;rows[-3]['s1']=.5
    return rows


def expand_plan(rows, start_lengths, rests, difference_limit, dwell_s=2., settle_s=2.):
    if not 1 <= len(rows) <= 200:raise ValueError('Use 1..200 waypoint rows.')
    if not math.isfinite(dwell_s) or not 0 <= dwell_s <= 30:raise ValueError('Dwell must be 0..30 seconds.')
    previous=list(map(float,start_lengths));poses=[];duration=0.
    for r,row in enumerate(rows):
        try:
            mode=row['mode'];x,y,s1,s2=[float(row[k]) for k in ('x','y','s1','s2')]
            n=int(row['steps'])
            if not all(math.isfinite(v) for v in (x,y,s1,s2)) or n!=float(row['steps']) or not 1<=n<=100:
                raise ValueError('Use finite values and 1..100 integer intervals.')
            if r==0 and n!=1:raise ValueError('First row is the initial capture: use 1 interval.')
            if mode=='Difference':
                end=targets(y,x,s1,rests,difference_limit);speeds=[s1,s1]
            elif mode in MODES[1:]:
                end=[x,y] if mode=='Both cables' else [x,previous[1]] if mode=='Cable 1' else [previous[0],y]
                speeds=[s1,s2]
                independent_targets(end,speeds,rests,difference_limit)
            else:raise ValueError('Unknown motion mode.')
            origin=previous[:]
            for j in range(1,n+1):
                point=[u+(v-u)*j/n for u,v in zip(origin,end)]
                independent_targets(point,speeds,rests,difference_limit)
                if mode=='Difference':
                    command=dict(action='move',mean_mm=sum(point)/2,difference_mm=point[0]-point[1],speed_mm_s=s1)
                    targets(command['mean_mm'],command['difference_mm'],s1,rests,difference_limit)
                elif mode in ('Cable 1','Cable 2'):
                    axis=int(mode[-1])-1
                    command=dict(action='axis',axis=axis,length_mm=point[axis],speed_mm_s=speeds[axis])
                else:command=dict(action='independent',lengths_mm=point,speeds_mm_s=speeds)
                travel=max(abs(v-u)/s for u,v,s in zip(previous,point,speeds))
                poses.append(dict(number=len(poses)+1,row=r+1,interval=j,intervals=n,label=str(row.get('label',''))[:120],
                    mode=mode,lengths_mm=point,speeds_mm_s=speeds,command=command,travel_s=travel,
                    settle_s=max(2.,settle_s),dwell_s=dwell_s))
                duration+=travel+max(2.,settle_s)+dwell_s+1.5
                previous=point
        except (ValueError,TypeError,KeyError) as exc:raise ValueError(f'Row {r+1}: {exc}') from exc
    if len(poses)>500:raise ValueError('Expanded campaign exceeds 500 captures. Reduce intervals or split the plan.')
    return dict(poses=poses,estimated_seconds=duration,reserved_seconds=duration*1.3+30,
                initial_lengths_mm=list(start_lengths),rest_lengths_mm=list(rests),difference_limit_mm=difference_limit)


def held_packet(packet, target, window, max_gap=.65, tolerance=.5):
    """Both encoders must bracket the frame, be HOLDING at target and be stable."""
    try:
        lengths,_=inputs_at(packet,packet['t_s'],max_gap)
        if max(abs(a-b) for a,b in zip(lengths,target))>tolerance:return False
        for axis in (0,1):
            rows=sorted((r for r in packet['motor'] if r['axis']==axis),key=lambda r:r['t_s'])
            first=max([i for i,r in enumerate(rows) if r['t_s']<=packet['t_s']-window],default=-1)
            last=next((i for i,r in enumerate(rows) if r['t_s']>=packet['t_s']),-1)
            if first<0 or last<0:return False
            rows=rows[first:last+1]
            if len(rows)<3 or any(r['state']!='HOLDING' or r.get('target_mm') is None or
                abs(r['target_mm']-target[axis])>tolerance or abs(r['length_mm']-target[axis])>tolerance for r in rows):return False
            if any(b['t_s']-a['t_s']>max_gap for a,b in zip(rows,rows[1:])):return False
            if max(r['length_mm'] for r in rows)-min(r['length_mm'] for r in rows)>.15:return False
        return True
    except (ValueError,KeyError,TypeError):return False


class CampaignRunner:
    """tick/event/captured are GUI-thread calls; disk capture is acknowledged later."""
    def __init__(self, plan, send, capture, changed=None, clock=time.monotonic):
        self.plan=copy.deepcopy(plan);self.send=send;self.capture=capture;self.changed=changed or (lambda *_:None);self.clock=clock
        self.state='idle';self.index=0;self.message='Ready';self.command_id=None;self.sent_t=None
        self.events=deque(maxlen=1500)

    @property
    def active(self):return self.state in ('next','ack','moving','saving')

    def update(self,state,message):
        self.state=state;self.message=message
        if state=='next':self.next_since=self.clock()
        self.events.append(dict(state=state,pose=self.index+1,message=message,monotonic_s=self.clock()))
        self.changed(state,message)

    def start(self):
        if self.state!='idle':raise ValueError('A campaign cannot restart or automatically resume. Start a new acquisition.')
        self.update('next','Starting campaign')

    def abort(self,reason,hold=True):
        if not self.active:return
        self.update('aborted',reason)
        if hold:
            try:self.send('hold')
            except Exception:pass  # Existing owner watchdog remains authoritative.

    def event(self,event):
        if not self.active:return
        if event.get('type') in ('closing','closed','startup_failed','transport_closed'):
            self.abort('Connection ended: '+event.get('message',event['type']));return
        if self.state=='ack' and event.get('id')==self.command_id:
            if event['type']=='rejected':self.abort('Command rejected: '+event.get('message',''));return
            if event['type']=='accepted':self.update('moving','Waiting for target arrival, stable encoders and dwell')

    def tick(self, remote, remote_age, item, max_age=1., max_gap=.65):
        if not self.active:return
        now=self.clock()
        if remote_age>1.5 or not item or item['displayed_age_s']>max_age:
            self.abort('Fresh camera and motor status lost. Completed poses retained.');return
        packet=item['packet']
        if self.plan.get('session') and (packet.get('session')!=self.plan['session'] or remote.get('session',self.plan['session'])!=self.plan['session']):
            self.abort('Camera and motor session identity changed.');return
        if (not packet.get('motor_active') or packet.get('state') not in ('STREAMING','RECORDING') or
            any(v is None or not -.1<=v<=max_age for v in packet.get('motor_age_s',[None,None]))):
            self.abort('Enabled, referenced encoder feedback lost.');return
        axes=remote.get('axes',[])
        if len(axes)!=2 or any(not a.get('enabled') or a.get('rest_count') is None or a.get('state') in ('FAULT','DISABLED') for a in axes):
            self.abort('Motor enable, reference or healthy state lost.');return
        if packet.get('rest_lengths_mm')!=self.plan['rest_lengths_mm']:
            self.abort('Rest lengths changed during acquisition.');return
        if self.state=='saving':return
        pose=self.plan['poses'][self.index]
        if self.state=='next':
            if now-self.next_since>5:
                self.abort('Motors did not become ready for the next command.');return
            if remote.get('phase')!='HOLDING' or remote.get('pending'):return
            self.sent_t=packet['t_s'];self.sent_wall=now
            self.deadline=now+2*pose['travel_s']+45+pose['settle_s']+pose['dwell_s']
            try:self.command_id=self.send(**pose['command'])
            except Exception as exc:self.abort('Could not send motion: '+str(exc));return
            self.update('ack',f'Pose {self.index+1}/{len(self.plan["poses"])}: awaiting command acknowledgement');return
        if self.state=='ack' and now-self.sent_wall>3:
            self.abort('Command acknowledgement timed out; motion is never retried.');return
        if now>self.deadline:
            self.abort('Target/settling timeout; this pose was not captured.');return
        window=pose['settle_s']+pose['dwell_s']
        if (self.state=='moving' and remote.get('phase')=='HOLDING' and not remote.get('pending') and
            packet['t_s']-window>self.sent_t and now-self.sent_wall>=window and
            held_packet(packet,pose['lengths_mm'],window,max_gap)):
            self.update('saving',f'Pose {self.index+1}: saving image and measured inputs')
            try:self.capture(self.index,item,pose)
            except Exception as exc:self.abort('Capture failed: '+str(exc))

    def captured(self,index,error=None):
        if not self.active or self.state!='saving' or index!=self.index:return
        if error:self.abort('Capture could not be saved: '+str(error));return
        self.index+=1
        if self.index==len(self.plan['poses']):self.update('complete','Journey complete — all poses retained')
        else:self.update('next','Saved; preparing the next waypoint')
