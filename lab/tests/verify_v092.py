"""Campaign k₂ × stiffness-ratio surface; synthetic points, no hardware.

Run: QT_QPA_PLATFORM=offscreen python tests/verify_v092.py
"""
import base64,copy,io,json,os,sys,tempfile,threading,time
from pathlib import Path
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
from PIL import Image
from campaign import expand_plan
from campaign_storage import CampaignStore
from campaign_analysis import fit_campaign
from campaign_surface import fit_surface,surface_is_current,SurfaceCancelled
from analysis import Comparator,inputs_at
from interface import Window,QApplication,recorded_packet
from spirob_analysis.fitting import predict

cfg=json.loads(Path('config.json').read_text());cfg['model']='model/spirob_robot.xml'
fit=json.loads(Path('calibration/pilot_fit.json').read_text());engine=Comparator(cfg,fit)
def row(mode='Difference',x=0,y=220):return dict(mode=mode,x=x,y=y,s1=1,s2=1,steps=1,label='test')
def recorded(step):
    packet=recorded_packet(cfg,step)
    return dict(packet=packet,rgb=np.array(Image.open(io.BytesIO(base64.b64decode(packet['image_b64']))).convert('RGB')),
        parameters=copy.deepcopy(engine.p),d2=engine.d2,setup=engine.setup(),observed=None,displayed_age_s=0.,recorded_demo=True,message='test')
TRUTH=dict(k2=1.,stiffness_ratio=.95,stiffness_exponent=1.)
app=QApplication.instance() or QApplication([])
with tempfile.TemporaryDirectory() as td:
    plan=expand_plan([row(),row(x=20),row(x=-20)],[220,220],[220,220],70)
    store=CampaignStore.create(td,plan,[row()],recorded(2),cfg,fit,'manual')
    for i,step in enumerate((2,3,4,1)):store.capture(i,recorded(step),plan['poses'][i])
    store.state('complete','test')
    for i in range(3):
        original=store.prepared(i)
        lengths,_=inputs_at(original['packet'],original['packet']['t_s'],cfg['live']['max_input_gap_s'])
        obs=dict(displacement_mm=(lengths-np.array(original['packet']['rest_lengths_mm'])).tolist(),
                 landmark_ids=['j_002','j_005','j_010','j_015','j_020','tip'],landmark_mode='joints',
                 tail_distance_px=[1,2,3,4,5,0],points_px=[[0,0]]*6)
        obs['points_px']=predict(engine.model,obs,dict(engine.p,**TRUTH))[0].tolist();store.set_observation(i,obs,'synthetic')
    # Truth lies on the grid: k₂ ∈ {0.5,1,2}, r ∈ {0.9,0.95,1.0}.
    res=fit_surface(store,.5,2.,3,.9,1.,3)
    m=res['minimum'];assert abs(m['k2']-1)<1e-12 and abs(m['ratio']-.95)<1e-12 and m['cost_px2']<1e-12,m
    assert res['included_poses']==3 and res['valid_points']>=9 and not res['cable_kp_overridden']
    for name in ('fit_surface.json','fit_surface.csv'):assert (store.root/name).exists()
    rows=(store.root/'fit_surface.csv').read_text().strip().splitlines();assert len(rows)==1+len(res['k2'])*len(res['ratio'])
    assert all(p['own_minimum']['ratio']==.95 for p in res['poses']) and surface_is_current(store,res)
    assert 'analysis' not in store.doc or store.doc['analysis'].get('kind')=='campaign_fit'
    # The campaign's own (k₂, r) is always included as a grid point.
    base=res['baseline_exponential'];assert base is None or base['k2'] in res['k2']
    print('PASS synthetic (k₂, r) minimum recovered exactly; CSV/JSON written; fit_results untouched',flush=True)
    # kp override changes the model used, and is recorded.
    stiff=fit_surface(store,.5,2.,3,.9,1.,3,cable_kp=2.5e5)
    assert stiff['cable_kp_overridden'] and stiff['cable_kp_N_per_m']==2.5e5 and stiff['minimum']['cost_px2']>0
    for bad in (dict(r_lower=1.2,r_upper=1.1),dict(k_count=2),dict(cable_kp=1.)):
        args=dict(k_lower=.5,k_upper=2.,k_count=3,r_lower=.9,r_upper=1.,r_count=3);args.update(bad)
        try:fit_surface(store,**args)
        except ValueError:pass
        else:raise AssertionError(f'Invalid input accepted: {bad}')
    stop=threading.Event();stop.set()
    try:fit_surface(store,.5,2.,3,.9,1.,3,cancel=stop)
    except SurfaceCancelled:pass
    else:raise AssertionError('Cancel ignored')
    store.exclude(1,True);assert not surface_is_current(store,res)
    store.exclude(1,False)
    print('PASS kp override recorded, invalid ranges rejected, cancel, stale detection after exclusion',flush=True)
    # GUI: run the tab end to end and check layout.
    fit_surface(store,.5,2.,3,.9,1.,3)
    w=Window(cfg,fit,{},False);w.show();w.open_campaign();d=w.campaign_window;d.load(store.root/'campaign.json')
    assert d.surface and abs(d.surface['minimum']['ratio']-.95)<1e-12
    d.s_klow.setValue(.5);d.s_khigh.setValue(2.);d.s_kcount.setValue(3);d.s_rlow.setValue(.9);d.s_rhigh.setValue(1.);d.s_rcount.setValue(3)
    assert d.surface_button.isEnabled();d.surface_button.click()
    end=time.monotonic()+120
    while d.busy and time.monotonic()<end:app.processEvents();time.sleep(.02)
    assert not d.busy and 'Minimum: k₂ = 1' in d.surface_text.text() and (store.root/'fit_surface.png').exists(),d.surface_text.text()
    d.s_metric.setCurrentIndex(1);app.processEvents()
    from unittest.mock import patch
    with patch('campaign_window.QDesktopServices.openUrl'):
        d.overlay_button.click()
        end=time.monotonic()+120
        while d.busy and time.monotonic()<end:app.processEvents();time.sleep(.02)
    overlays=sorted(store.root.glob('surface_overlay_pose_*.png'));assert len(overlays)==3,d.status.text()
    scaled=fit_surface(store,.5,2.,3,.9,1.,3,pixels_per_m=4000.)
    lab=fit_surface(store,.5,2.,3,.9,1.,3,model_path='model/new_20261013/spirob_robot.xml')
    assert lab['model_source']=='external' and lab['model_xml'].endswith('new_20261013/spirob_robot.xml')
    try:fit_surface(store,.5,2.,3,.9,1.,3,model_path='model/does_not_exist.xml')
    except (ValueError,OSError):pass
    else:raise AssertionError('Missing model accepted')
    assert scaled['pixels_per_m_overridden'] and scaled['pixels_per_m']==4000. and scaled['campaign_pixels_per_m']!=4000.
    d.surface=scaled;d.draw_surface();assert 'scale 4000 px/m (overridden)' in d.surface_text.text()
    out=Path(os.environ.get('SPIROB_TEST_OUTPUT','/tmp/spirob-v092-verification'));out.mkdir(parents=True,exist_ok=True)
    for size in [(1040,650),(1366,768)]:
        d.resize(size[0]-30,size[1]-40);d.tabs.setCurrentIndex(3);app.processEvents();d.s_canvas.draw()
        for widget in (d.surface_button,d.surface_cancel,d.s_canvas,d.surface_text):
            at=widget.mapTo(d,widget.rect().topLeft());assert at.x()>=0 and at.y()>=0 and at.x()+widget.width()<=d.width() and at.y()+widget.height()<=d.height()
        d.grab().save(str(out/f'campaign_surface_{size[0]}.png'))
    d.close();w.close();app.processEvents()
    print('PASS k₂ × ratio tab: reload saved surface, sweep from GUI, plot/metric switch, layouts',flush=True)
print('PASS all v0.9.2 surface checks',flush=True)
