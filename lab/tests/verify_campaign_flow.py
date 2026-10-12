"""GUI-level autonomous collection -> automatic fit / manual queue; no hardware."""
import base64,copy,io,json,os,queue,sys,tempfile,threading,time
from pathlib import Path
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
from interface import QApplication,Window,recorded_packet
from analysis import Comparator,inputs_at
from PIL import Image
cfg=json.loads(Path('config.json').read_text());fit=json.loads(Path('calibration/pilot_fit.json').read_text());engine=Comparator(cfg,fit)
app=QApplication([])
pkt=recorded_packet(cfg,2);lengths,_=inputs_at(pkt,pkt['t_s'],cfg['live']['max_input_gap_s']);rgb=np.array(Image.open(io.BytesIO(base64.b64decode(pkt['image_b64']))).convert('RGB'))
class Link:
 def __init__(self,folder):self.log_dir=folder;self.events=queue.Queue();self.commands=[];self.cancel=threading.Event()
 def send(self,action,**values):
  self.commands.append((action,values));self.events.put(dict(type='accepted',id=len(self.commands)));return len(self.commands)
 def touch(self):pass
 def preview_received(self,age):pass
with tempfile.TemporaryDirectory() as folder:
 for mode in (0,1):
  w=Window(cfg,fit,{},False);w.timer.stop();link=Link(Path(folder)/str(mode));w.session=link
  w.remote=dict(phase='HOLDING',pending=False,rest_lengths_mm=[220,220],axes=[dict(enabled=True,rest_count=0,state='HOLDING') for _ in (0,1)],session_limit_s=1800,elapsed_s=10)
  clock=[10.];sequence=[0]
  def source():
   p=copy.deepcopy(pkt);sequence[0]+=1;p.update(t_s=clock[0],frame_index=sequence[0],motor_active=True,state='STREAMING',motor_age_s=[0,0],frame_age_s=0.)
   p['motor']=[dict(axis=a,t_s=float(t),length_mm=float(lengths[a]),target_mm=float(lengths[a]),state='HOLDING') for a in (0,1) for t in np.arange(clock[0]-6,clock[0]+.4,.2)]
   return dict(packet=p,rgb=rgb,observed=None,parameters=engine.p,d2=engine.d2,setup=engine.setup(),displayed_age_s=0.,recorded_demo=False,message='simulated held photograph',received=time.monotonic())
  w.manual_source=source;w.last_video=source();w.open_campaign();d=w.campaign_window
  d.mode.setCurrentIndex(mode);d.low.setValue(.5);d.high.setValue(2);d.count.setValue(3)
  rows=[dict(mode='Both cables',x=float(lengths[0]),y=float(lengths[1]),s1=1.,s2=1.,steps=1,label='held test')]*2
  d.set_rows(rows);d.start();assert d.runner and d.active,d.status.text();d.runner.clock=lambda:clock[0]
  end=time.monotonic()+30
  while d.active and time.monotonic()<end:
   while not link.events.empty():d.motor_event(link.events.get_nowait())
   w.remote_received=time.monotonic();clock[0]+=.25;w.last_video=source();d.tick();d.poll();app.processEvents();time.sleep(.01)
  assert d.runner.state=='complete',d.status.text()
  assert len(d.store.doc['poses'])==2 and len(link.commands)==2
  if mode==0:
   end=time.monotonic()+45
   while not d.store.doc.get('analysis') and time.monotonic()<end:
    app.processEvents();time.sleep(.01)
   assert d.store.doc.get('analysis'),d.status.text()
   assert d.store.doc['analysis']['complete'] and not d.store.doc['analysis']['per_pose']
  else:
   for _ in range(10):app.processEvents();time.sleep(.01)
   assert not d.busy and not d.store.doc.get('analysis') and all(p['observation'] is None for p in d.store.doc['poses'])
  # Simulate a process loss after both atomic files but before the index commit.
  from campaign_storage import CampaignStore
  d.store.doc['poses'].pop();d.store.doc['image_bytes']-=(d.store.root/'poses/0002/frame.jpg').stat().st_size
  d.store.doc['state']='saving';d.store.save()
  recovered=CampaignStore(d.store.root)
  assert len(recovered.doc['poses'])==2 and recovered.doc['state']=='interrupted' and recovered.doc['poses'][-1]['observation'] is None
  # Quiesce all campaign writers before existing Save / Discard touches run data.
  d.begin_shutdown();d.shutdown_wait();assert not d.isEnabled()
  root=d.store.root
  import shutil
  shutil.rmtree(root);d.poll();d.end_shutdown();assert d.store is None and not root.exists()
  w.session=None;d.close();w.close();app.processEvents()
  print('PASS GUI autonomous '+('automatic measurement + shared result' if mode==0 else 'manual queue without extraction')+'; shutdown waits for writers and discarded folders stay deleted',flush=True)
