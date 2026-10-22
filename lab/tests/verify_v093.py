"""E1 marker dots and corrected horizontal model; synthetic images, no hardware.

Run: QT_QPA_PLATFORM=offscreen python tests/verify_v093.py
"""
import base64,copy,io,json,os,sys,tempfile,threading,time
from pathlib import Path
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
import cv2,mujoco
from PIL import Image
from spirob_analysis.model import SpiRobModel
from dot_tracking import measure_dots,expected_spacing_px,LINK_IDS,DotTracker,detect_dots

XML='model/new_20261013/spirob_robot.xml'


def render(model,D,ppm=4178.,T=(1040.,100.),rot=0.,seed=0,drop=()):
    """Brown board, white tapered elements with dark edge notches, black centre dots."""
    rng=np.random.default_rng(seed)
    model.equilibrium(model.controls([D/2,-D/2]),k2=.1)
    J=np.vstack([model.base_point(),model.joint_curve()])
    c,s=np.cos(rot),np.sin(rot);J=ppm*(J@np.array([[c,-s],[s,c]]).T)+np.array(T)
    img=np.clip(np.full((1080,1920,3),(120,95,70),float)+rng.normal(0,6,(1080,1920,3)),0,255).astype(np.uint8)
    widths=np.linspace(110,26,len(J)-1)
    for k,(p,q) in enumerate(zip(J[:-1],J[1:])):
        cv2.line(img,tuple(np.round(p).astype(int)),tuple(np.round(q).astype(int)),(195,200,205),int(widths[k]))
        t=(q-p)/np.linalg.norm(q-p);n=np.array([-t[1],t[0]])
        for sg in (-1,1):cv2.circle(img,tuple(np.round(q+sg*n*widths[k]/2).astype(int)),3,(40,40,40),-1)
    centres=(J[1:-1]+J[2:])/2
    for i,p in enumerate(centres):
        if i not in drop:cv2.circle(img,tuple(np.round(p).astype(int)),4,(15,15,15),-1)
    for f in (.3,.62):cv2.circle(img,tuple(np.round(J[0]+f*(J[1]-J[0])).astype(int)),4,(15,15,15),-1)
    return cv2.GaussianBlur(img,(3,3),0),centres


# 1. Corrected model: horizontal plane, gravity along the hinge axes.
m=mujoco.MjModel.from_xml_path(XML);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
assert np.allclose(d.xanchor[0],[0,.23,.09],atol=1e-5) and np.allclose(d.xaxis[1],[0,0,-1],atol=1e-5)
assert abs(d.site("tip_site").xpos[2]-.09)<1e-5
a=SpiRobModel(XML);b=SpiRobModel(XML);b.m.opt.gravity[:]=0
for D in (20,70):
    x=a.equilibrium(a.controls([-D/2,D/2]),k2=.1)['curve_m'];y=b.equilibrium(b.controls([-D/2,D/2]),k2=.1)['curve_m']
    assert np.abs(x-y).max()<1e-6
print('PASS corrected XML: robot_base z=0.09 m, horizontal bending plane, gravity adds no in-plane torque',flush=True)

# 2. Dot measurement across the full ±70 mm range, missing dots and scale error.
M=SpiRobModel(XML);M.m.actuator_gainprm[:,0]=2.5e5;M.m.actuator_biasprm[:,1]=-2.5e5;M.kp[:]=2.5e5
M.set_stiffness_profile(.915);sp=expected_spacing_px(M,4178.)
for D in range(-70,71,20):
    for ppm,drop in ((4178.,()),(3800.,()),(4500.,()),(4178.,(8,13))):
        img,cent=render(M,D,ppm=ppm,seed=D+500,drop=drop)
        r=measure_dots(img,sp);got=dict(zip(r['landmark_ids'],r['points_px']))
        assert [LINK_IDS.index(n) for n in r['missing_links']]==sorted(drop),(D,ppm,r['missing_links'])
        assert max(np.linalg.norm(np.array(got[n])-cent[i]) for i,n in enumerate(LINK_IDS) if n in got)<1.5
assert r['landmark_mode']=='centres' and r['measurement_mode']=='marker_dots' and len(r['tail_distance_px'])==len(r['points_px'])
blank=np.full((1080,1920,3),120,np.uint8)
try:measure_dots(blank,sp)
except ValueError:pass
else:raise AssertionError('Empty image accepted')
t=DotTracker(sp);img,_=render(M,30);t.update(img);assert t.base_hint is not None and len(t.update(img)['landmark_ids'])==20
print('PASS marker dots: 20/20 elements over D = −70…+70 mm, two hidden dots skipped, ±9 % scale tolerance, sub-1.5 px',flush=True)

# 3. Campaign: dots fill missing observations, manual points are kept and compared.
from analysis import Comparator
from campaign import expand_plan
from campaign_storage import CampaignStore
from campaign_analysis import fit_campaign
from campaign_dots import measure_campaign_dots
from interface import Window,QApplication,recorded_packet
cfg=json.loads(Path('config.json').read_text());cfg['model']=XML
fit=json.loads(Path('calibration/new_model_initial.json').read_text());engine=Comparator(cfg,fit)
def row(x=0):return dict(mode='Difference',x=x,y=227.353,s1=1,s2=1,steps=1,label='dots')
def recorded(step,D):
    packet=recorded_packet(cfg,step);img,_=render(M,D,seed=step)
    buf=io.BytesIO();Image.fromarray(img).save(buf,'JPEG',quality=95);packet['image_b64']=base64.b64encode(buf.getvalue()).decode()
    return dict(packet=packet,rgb=img,parameters=copy.deepcopy(engine.p),d2=engine.d2,setup=engine.setup(),observed=None,
                displayed_age_s=0.,recorded_demo=True,message='synthetic dots')
app=QApplication.instance() or QApplication([])
with tempfile.TemporaryDirectory() as td:
    rests=cfg['motion_profile']['rest_lengths_mm']
    plan=expand_plan([row(),row(20),row(-20)],rests,rests,70)
    store=CampaignStore.create(td,plan,[row()],recorded(2,0),cfg,fit,'manual')
    for i,(step,D) in enumerate(((2,-30),(3,30),(4,50),(1,0))):store.capture(i,recorded(step,D),plan['poses'][i])
    store.state('complete','test')
    manual=dict(displacement_mm=[0.,0.],landmark_ids=['j_002','j_005','j_010','j_015','j_020','tip'],landmark_mode='joints',
                tail_distance_px=[1,2,3,4,5,0],points_px=[[1000,100+80*k] for k in range(6)])
    store.set_observation(0,manual,'Manual')
    rows=measure_campaign_dots(store)
    assert len(rows)==3 and all(r['links']==20 for r in rows),rows
    assert store.doc['poses'][0]['observation']['landmark_mode']=='joints' and rows[0]['result'].startswith('manual points kept')
    assert all(store.doc['poses'][i]['observation']['landmark_mode']=='centres' for i in (1,2)) and store.doc['poses'][3]['excluded']
    assert (store.root/'dots_summary.csv').exists() and all((store.path(p['path'])/'dots_overlay.jpg').exists() for p in store.doc['poses'] if not p['excluded'])
    res=fit_campaign(store,.05,2.,3);assert res['included_poses']==3
    stop=threading.Event();stop.set()
    try:measure_campaign_dots(store,cancel=stop)
    except Exception as exc:assert 'stopped' in str(exc)
    else:raise AssertionError('Cancel ignored')
    print('PASS campaign dots: unannotated poses measured, manual points kept and compared, overlays, shared fit runs',flush=True)
    # 4. GUI wiring: live method switch and campaign button.
    w=Window(cfg,fit,{},False);w.show()
    w.dots_box.setChecked(True);assert w.config['vision_method']=='dots'
    w.dots_box.setChecked(False);assert w.config['vision_method']=='silhouette'
    w.open_campaign();dlg=w.campaign_window;dlg.load(store.root/'campaign.json');dlg.dots_replace.setChecked(True)
    assert dlg.dots_button.isEnabled();dlg.dots_button.click()
    end=time.monotonic()+120
    while dlg.busy and time.monotonic()<end:app.processEvents();time.sleep(.02)
    assert not dlg.busy and 'Marker dots: 3/3' in dlg.status.text(),dlg.status.text()
    assert CampaignStore(store.root).doc['poses'][0]['observation']['landmark_mode']=='centres'
    dlg.close();w.close();app.processEvents()
    print('PASS GUI: marker-dot checkbox persists method; campaign button measures and optionally replaces points',flush=True)
print('PASS all v0.9.3 marker-dot checks',flush=True)
