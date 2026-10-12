"""Campaign -> existing dispatcher -> original motor Rig with DemoDrive encoders.

Accelerated deterministic clock; never opens physical hardware or a network.
"""
import argparse,copy,sys,time
from pathlib import Path
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
p=argparse.ArgumentParser();p.add_argument('--motor-source',required=True);args=p.parse_args();sys.path.insert(0,str(Path(args.motor_source).resolve()))
from campaign import CampaignRunner,expand_plan,default_plan
from actuation import dispatch
from motion_limits import worker_limits
from spirob_cable import worker
from spirob_cable.config import RigSettings
from spirob_cable.drive import DemoDrive

class Recorder:
 path="simulated-campaign"
 def __init__(self):self.events=[];self.rows=[]
 def event(self,*args,**kwargs):self.events.append((args,kwargs))
 def sample(self,axis,row):
  self.rows.append(dict(axis=axis,t_s=row['read_end_s'],length_mm=row['absolute_mm'],target_mm=row['target_absolute_mm'],state=row['state']))
 def pair(self,*args,**kwargs):pass
now=[100.];cfg=RigSettings.demo();rec=Recorder();acks=[];captured=[];sent=[]
with patch('time.monotonic',lambda:now[0]),worker_limits([220,220],70):
 rig=worker.Rig(cfg,[DemoDrive(c,rec) for c in cfg.selected],rec)
 for a in rig.axes:
  position=a.drive.position();a.count=a.connection=a.rest=a.target=position;a.rest_length=220.;a.enabled=True;a.state='HOLDING'
  a.last_read=now[0];a.stable.extend([position]*3);a.position_mode=True;a.drive.enable()
 class Motor:
  referenced=True;pending=False
  @property
  def snapshot(self):return rig.snapshot()
  def submit(self,name,d):
   if name=='independent':rig.move_independent(d['lengths_mm'],d['speeds_mm_s'])
   elif name=='move':rig.move_axis(d['axis'],d['value_mm'],d['speed_mm_s'],d.get('absolute',False))
   elif name=='modes':rig.move_modes(d['mode'],d['mean_mm'],d['difference_mm'],d['speed_mm_s'])
   else:raise AssertionError(name)
 motor=Motor()
 def send(action,**values):
  sent.append((action,values));ident=len(sent)
  if action=='hold':rig.stop_all();return ident
  dispatch(dict(action=action,received_s=now[0],**values),motor,[220,220],True)
  acks.append(dict(type='accepted',id=ident));return ident
 plan=expand_plan(default_plan(),[220,220],[220,220],70)
 def capture(index,item,pose):
  axes=rig.snapshot()['axes'];assert all(a['state']=='HOLDING' for a in axes)
  assert np.allclose([a['absolute_mm'] for a in axes],pose['lengths_mm'],atol=.01)
  captured.append(index);runner.captured(index)
 runner=CampaignRunner(plan,send,capture,clock=lambda:now[0]);runner.start()
 for tick in range(22000):
  now[0]+=.1;rig.poll_all()
  for event in acks:runner.event(event)
  acks.clear();axes=rig.snapshot()['axes']
  remote=dict(axes=axes,pending=False,phase='HOLDING' if all(a['state']=='HOLDING' for a in axes) else 'MOVING_OR_REFERENCING')
  packet=dict(t_s=now[0]-.15,motor=[r for r in rec.rows if r['t_s']>now[0]-6],state='STREAMING',motor_active=True,motor_age_s=[0,0],rest_lengths_mm=[220,220])
  runner.tick(remote,0,dict(packet=packet,displayed_age_s=.15))
  if not runner.active:break
 assert runner.state=='complete',(runner.state,runner.message,len(captured))
 assert captured==list(range(73)) and len(sent)==73
 assert all(a['enabled'] and a['state']=='HOLDING' for a in rig.snapshot()['axes'])
 speeds=[v['requested_mm_s'] for e,v in rec.events if e[0]=='move_requested'];assert .5 in speeds and 1. in speeds
 rig.stop_all()
 print(f'PASS all 73 default poses through original DemoDrive/Rig and real command dispatcher; independent speeds, HOLDING + target capture, one acknowledged command per pose; simulated duration {now[0]-100:.1f} s')
