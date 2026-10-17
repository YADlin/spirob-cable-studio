"""Campaign verification using recorded images, synthetic points and fake transport."""
import base64,copy,io,json,os,sys,tempfile,time,threading
from pathlib import Path
from unittest.mock import patch
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
from campaign import default_plan,expand_plan,held_packet,CampaignRunner
from campaign_storage import CampaignStore
from campaign_analysis import fit_campaign,add_per_pose,auto_measure,CampaignCancelled
from analysis import Comparator,inputs_at
from interface import Window,QApplication,recorded_packet
from spirob_analysis.fitting import predict
from PIL import Image
from matplotlib.backend_bases import MouseEvent
from PySide6.QtWidgets import QMessageBox
def fail_message(parent,title,message):raise AssertionError(title+': '+message)
QMessageBox.warning=fail_message
cfg=json.loads(Path('config.json').read_text());cfg['model']='model/spirob_robot.xml'
fit=json.loads(Path('calibration/pilot_fit.json').read_text());engine=Comparator(cfg,fit)
def row(mode='Difference',x=0,y=220,s1=1,s2=1,steps=1):return dict(mode=mode,x=x,y=y,s1=s1,s2=s2,steps=steps,label='test')
p=expand_plan(default_plan(),[220,220],[220,220],70);assert len(p['poses'])==74 and p['reserved_seconds']<1200
m=expand_plan([row(),row(x=50,steps=15),row(x=-40,steps=15)],[220,220],[220,220],70)
assert len(m['poses'])==32 and m['poses'][15]['lengths_mm']==[245,195] and m['poses'][-2]['lengths_mm']==[200,240] and m['poses'][-1]['lengths_mm']==[220,220]
ind=expand_plan([row('Cable 1',190,0,s1=5),row('Both cables',150,210,s1=1,s2=4),row('Cable 2',0,250)],[220,220],[220,220],70)
assert [p['lengths_mm'] for p in ind['poses']]==[[190,220],[150,210],[150,250],[220,220]] and ind['poses'][1]['command']['speeds_mm_s']==[1,4]
for rows in [[row(x=80)],[row('Both cables',150,291)],[row(s1=21)],[row(steps=2)],[row(),row(steps=1.5)],[row(),*[row(steps=100) for _ in range(6)]]]:
 try:expand_plan(rows,[220,220],[220,220],70)
 except ValueError:pass
 else:raise AssertionError('Invalid plan accepted')
print('PASS mixed waypoints, speeds, interpolation, 73 measurement poses plus return and limits',flush=True)
now=[0.];commands=[];captures=[]
remote=dict(phase='HOLDING',pending=False,axes=[dict(enabled=True,rest_count=0,state='HOLDING') for _ in (0,1)])
def item(t,lengths=(220.,220.),target=None):
 target=lengths if target is None else target
 return dict(displayed_age_s=0.,packet=dict(t_s=t,state='STREAMING',motor_active=True,motor_age_s=[0.,0.],rest_lengths_mm=[220.,220.],
  motor=[dict(axis=a,t_s=float(ti),state='HOLDING',length_mm=lengths[a],target_mm=target[a]) for a in (0,1) for ti in np.arange(t-5,t+.31,.2)]))
def sender(action,**v):commands.append((action,v));return len(commands)
plan=expand_plan([row(),row(x=10)],[220,220],[220,220],70,dwell_s=0)
r=CampaignRunner(plan,sender,lambda *v:captures.append(v),clock=lambda:now[0]);r.start();r.tick(remote,0,item(0));assert r.state=='ack'
now[0]=1;r.event(dict(type='accepted',id=999));r.tick(remote,0,item(1));assert not captures
r.event(dict(type='accepted',id=r.command_id));now[0]=3;r.tick(remote,0,item(3));assert len(captures)==1 and r.state=='saving'
r.tick(remote,0,item(4));assert len(captures)==1
r.captured(0);r.tick(remote,0,item(4));r.event(dict(type='accepted',id=r.command_id));now[0]=8
r.tick(remote,0,item(8));assert len(captures)==1
r.tick(remote,0,item(8,(225,215)));assert len(captures)==2;r.captured(1);assert r.state=='next'
r.tick(remote,0,item(9,(225,215)));r.event(dict(type='accepted',id=r.command_id));now[0]=12
r.tick(remote,0,item(12));assert len(captures)==3;r.captured(2);assert r.state=='complete'
for reason in ('stale','rejected','ack','write','stop','closing'):
 now[0]=0;commands.clear();captures.clear();runner=CampaignRunner(plan,sender,lambda *v:captures.append(v),clock=lambda:now[0]);runner.start();runner.tick(remote,0,item(0))
 if reason=='stale':runner.tick(remote,2,item(0))
 elif reason=='rejected':runner.event(dict(type='rejected',id=runner.command_id,message='injected'))
 elif reason=='ack':now[0]=4;runner.tick(remote,0,item(4))
 elif reason=='write':
  runner.event(dict(type='accepted',id=runner.command_id));now[0]=3;runner.tick(remote,0,item(3));runner.captured(0,'disk full')
 elif reason=='closing':runner.event(dict(type='closing',message='timeout'))
 else:runner.abort('Stop')
 assert runner.state=='aborted' and commands[-1][0]=='hold'
 before=len(commands);runner.tick(remote,0,item(10));assert len(commands)==before
print('PASS ack identity, correct target + stable hold, one capture per pose, stale/reject/ack/write/Stop/timeout aborts and no retry',flush=True)
app=QApplication.instance() or QApplication([])
def until(predicate,seconds=40):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  app.processEvents()
  if predicate():return
  time.sleep(.01)
 raise AssertionError('GUI timeout')
def recorded(step):
 packet=recorded_packet(cfg,step)
 return dict(packet=packet,rgb=np.array(Image.open(io.BytesIO(base64.b64decode(packet['image_b64']))).convert('RGB')),
  parameters=copy.deepcopy(engine.p),d2=engine.d2,setup=engine.setup(),observed=None,displayed_age_s=0.,recorded_demo=True,message='Recorded test image')
out=Path(os.environ.get('SPIROB_TEST_OUTPUT','/tmp/spirob-v080-verification'));out.mkdir(parents=True,exist_ok=True)
with tempfile.TemporaryDirectory() as td:
 plan=expand_plan([row(),row(x=20),row(x=-20)],[220,220],[220,220],70)
 store=CampaignStore.create(td,plan,[row()],recorded(2),cfg,fit,'manual')
 for i,step in enumerate((2,3,4,1)):store.capture(i,recorded(step),plan['poses'][i])
 store.state('complete','test');assert len(CampaignStore(store.root).doc['poses'])==4
 assert store.doc['image_bytes']==sum(len(base64.b64decode(recorded(s)['packet']['image_b64'])) for s in (2,3,4,1))
 assert not any('image_b64' in p.read_text() for p in store.root.rglob('snapshot.json'))
 for i,truth in enumerate((.5,1.,2.)):
  original=store.prepared(i);assert original['result']['manual_capture_ready']
  lengths,_=inputs_at(original['packet'],original['packet']['t_s'],cfg['live']['max_input_gap_s'])
  obs=dict(displacement_mm=(lengths-np.array(original['packet']['rest_lengths_mm'])).tolist(),landmark_ids=['j_002','j_005','j_010','j_015','j_020','tip'],
   landmark_mode='joints',tail_distance_px=[1,2,3,4,5,0],points_px=[[0,0]]*6)
  obs['points_px']=predict(engine.model,obs,dict(engine.p,k2=truth))[0].tolist();store.set_observation(i,obs,'synthetic physical test')
 result=fit_campaign(store,.5,2.,3);assert not result['per_pose'] and result['complete'] and result['shared_minimum']['cost_px2']>0
 result=add_per_pose(store);assert [p['minimum']['k2'] for p in result['poses']]==[.5,1.,2.]
 assert max(p['minimum']['cost_px2'] for p in result['poses'])<1e-12
 for i,p in enumerate(store.doc['poses'][:3]):
  obs=copy.deepcopy(p['observation']);obs['points_px']=predict(engine.model,obs,dict(engine.p,k2=1.))[0].tolist();store.set_observation(i,obs,'synthetic shared')
 result=fit_campaign(store,.5,2.,3);assert result['shared_minimum']['k2']==1 and result['shared_minimum']['cost_px2']<1e-12
 def bad_k(model,observation,parameters):
  if parameters['k2']==.5 and observation['displacement_mm'][0]<0:raise ValueError('injected failed equilibrium')
  return predict(model,observation,parameters)
 with patch('campaign_analysis.predict',bad_k):result=fit_campaign(store,.5,2.,3)
 assert any(p['errors'][0] for p in result['poses']) and result['shared_cost_px2'][0] is None
 store.doc['poses'][2]['observation']=None;store.save();result=fit_campaign(store,.5,2.,3);assert not result['complete'] and result['missing_points']==[3]
 store.exclude(1,True);assert 'analysis' not in store.doc
 result=fit_campaign(store,.5,2.,3);assert result['included_poses']==1 and result['excluded_poses']==[2,4]
 stop=threading.Event();stop.set()
 try:fit_campaign(store,.5,2.,3,cancel=stop)
 except CampaignCancelled:pass
 else:raise AssertionError('Cancel ignored')
 store.doc['budget_bytes']=store.doc['image_bytes']
 try:store.capture(4,recorded(4),plan['poses'][0])
 except ValueError:pass
 else:raise AssertionError('Image budget bypassed')
 assert len(store.doc['poses'])==4
 # Even an invalid equilibrium at the current k cannot block joint annotation.
 with patch.object(Comparator,'compare_still',side_effect=RuntimeError('injected equilibrium failure')):
  frozen=store.prepared(0)
 assert frozen['result']['manual_capture_ready'] and frozen['capture_error'] is None and frozen['result']['landmarks']['model_px'] is None
 from manual_analysis import complete_capture
 from landmarks import correspondences
 manual={n:[800+i*12,150+i*80] for i,n in enumerate(['j_002','j_005','j_010','j_015','j_020','tip'])}
 layout=frozen['result']['landmarks'];pairs=correspondences(None,layout,manual=manual,excluded=set(layout['ids'])-set(manual))
 annotated=complete_capture(frozen,pairs,'joints',True,allow_missing_model=True)
 assert annotated['observed']['points_px']==list(manual.values()) and annotated['result']['cost_px2'] is None
 print('PASS exact raw JPEG retention, frozen model, offline reopen, shared/separate synthetic minima, invalid cells, subset reporting, storage cap and cancellation',flush=True)
 w=Window(cfg,fit,{},False);w.show();w.open_campaign();d=w.campaign_window;d.load(store.root/'campaign.json')
 print('GUI editor prepare',flush=True);d.poses.selectRow(0);d.edit_selected();until(lambda:d.inspector is not None);editor=d.inspector
 editor.ax.set_xlim(400,1200);editor.ax.set_ylim(900,100);editor.canvas.draw();limits=(editor.ax.get_xlim(),editor.ax.get_ylim())
 editor.begin_placement();editor.table.selectRow(0);xy=editor.ax.transData.transform((875,120))
 editor.canvas.callbacks.process('button_press_event',MouseEvent('button_press_event',editor.canvas,*xy,button=1))
 assert editor.table.currentRow()==1 and (editor.ax.get_xlim(),editor.ax.get_ylim())==limits
 assert (store.path(store.doc['poses'][0]['path'])/'draft.json').exists()
 print('GUI draft reload',flush=True);editor.close();d.edit_selected();until(lambda:d.inspector is not editor);editor=d.inspector;assert 'j_002' in editor.manual
 layout=editor.source_item['result']['landmarks'];names=['j_002','j_005','j_010','j_015','j_020','tip']
 editor.manual={n:p for n,p in zip(layout['ids'],layout['model_px']) if 0<=p[0]<1920 and 0<=p[1]<1080};editor.excluded=set(layout['ids'])-set(editor.manual)
 assert len(editor.manual)>=4
 print('GUI save points',flush=True);editor.rebuild();editor.stationary.setChecked(True);assert editor.sweep_profile_button.isEnabled();editor.sweep_profile_button.click()
 until(lambda:d.store.doc['poses'][0].get('annotations') is not None);until(lambda:d.inspector is not editor);assert d.inspector.pose_index==2
 suggestion=d.inspector
 rng=np.random.default_rng(91);texture=rng.integers(0,256,(1080,1920,3),dtype=np.uint8)
 suggestion.source_item['rgb']=np.roll(texture,(2,3),(0,1))
 with patch.object(d.store,'load_item',return_value={'rgb':texture}):suggestion.suggest()
 assert len(suggestion.manual)>=4 and not suggestion.stationary.isChecked()
 prior={r['id']:r['physical_px'] for r in d.store.doc['poses'][0]['annotations']['rows'] if r['physical_px'] is not None}
 errors={n:float(np.linalg.norm(np.array(xy)-prior[n]-[3,2])) for n,xy in suggestion.manual.items()}
 assert all(e<.4 for e in errors.values()),errors
 d.inspector.close();d.store.doc['analysis']=fit_campaign(d.store,.5,2.,3);d.draw();d.individual();assert d.store.doc['analysis']['per_pose']
 for size in [(1040,650),(1280,720),(1366,768)]:
  w.resize(*size);d.resize(size[0]-30,size[1]-40);d.tabs.setCurrentIndex(2);app.processEvents();d.canvas.draw()
  for widget in (d.start_button,d.cancel_button,d.canvas,d.result_text):
   at=widget.mapTo(d,widget.rect().topLeft());assert at.x()>=0 and at.y()>=0 and at.x()+widget.width()<=d.width() and at.y()+widget.height()<=d.height()
  d.grab().save(str(out/f'campaign_results_{size[0]}.png'))
 d.tabs.setCurrentIndex(0);d.grab().save(str(out/'campaign_plan.png'));d.tabs.setCurrentIndex(1);d.grab().save(str(out/'campaign_poses.png'))
 d.close();w.close();app.processEvents()
 print('PASS native batch editor, saved drafts, auto-advance retaining zoom, Save & next, exclusion, result exports and layouts',flush=True)
print('PASS all v0.8.0 campaign checks',flush=True)
