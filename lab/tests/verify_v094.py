"""Joint stiffness + cable transmission fit from a campaign; synthetic points, no hardware.

Run: QT_QPA_PLATFORM=offscreen python tests/verify_v094.py
"""
import copy,json,os,sys,tempfile,threading,time
from pathlib import Path
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
import mujoco
from analysis import Comparator
from campaign import expand_plan
from campaign_storage import CampaignStore
from campaign_surface import set_cable_kp
from joint_fit import fit_joints,export_xml,render_overlays,JointFitCancelled,effective_observation
from dot_tracking import LINK_IDS
from interface import Window,QApplication,recorded_packet
from spirob_analysis.fitting import predict
from spirob_analysis.model import SpiRobModel

XML='model/spirob_robot.xml'
cfg=json.loads(Path('config.json').read_text());cfg['model']=XML
fit=json.loads(Path('calibration/new_model_initial.json').read_text());engine=Comparator(cfg,fit)
def row(x=0):return dict(mode='Difference',x=x,y=227.353,s1=1,s2=1,steps=1,label='syn')
def recorded(step):
    return dict(packet=recorded_packet(cfg,step),parameters=copy.deepcopy(engine.p),d2=engine.d2,setup=engine.setup(),
                observed=None,displayed_age_s=0.,recorded_demo=True,message='synthetic')
truth_model=SpiRobModel(XML);set_cable_kp(truth_model,2.5e5)
TRUE_R,TRUE_G,TRUE_O=.9,np.array([.78,.88]),np.array([1.,-.5])
app=QApplication.instance() or QApplication([])
with tempfile.TemporaryDirectory() as td:
    rests=cfg['motion_profile']['rest_lengths_mm']
    Ds=[-40,-25,-10,10,25,40]   # moderate curls: no self-contact in the free-space solver
    plan=expand_plan([row(D) for D in Ds],rests,rests,70)
    store=CampaignStore.create(td,plan,[row()],recorded(2),cfg,fit,'manual')
    for i in range(len(Ds)):store.capture(i,recorded(1+i%4),plan['poses'][i])
    store.state('complete','test')
    params=dict(store.doc['fit']['candidate']['parameters'],stiffness_exponent=1.)
    for i,D in enumerate(Ds):
        enc=np.array([D/2,-D/2])
        # Truth follows the rig: the pulling cable delivers gain·travel+offset, the paid-out cable stays slack.
        o=dict(displacement_mm=enc.tolist(),landmark_ids=LINK_IDS,landmark_mode='centres',tail_distance_px=[0.]*20,points_px=[[0,0]]*20)
        o=effective_observation(o,dict(transmission=dict(gain=TRUE_G,offset_mm=TRUE_O,slack_antagonist=True)))
        pts=predict(truth_model,o,dict(params,k2=.18,stiffness_ratio=TRUE_R,difference_offset_mm=0.,mean_offset_mm=0.))[0]
        o.update(displacement_mm=enc.tolist(),points_px=pts.tolist());store.set_observation(i,o,'synthetic')
    # 1. Exponential profile + transmission recovers the truth exactly.
    loc=fit_joints(store,model_path=XML,profile='exponential',k2_reference=.18,max_evaluations=40,optimizer='least_squares')
    res=fit_joints(store,model_path=XML,profile='exponential',k2_reference=.18,max_evaluations=40,optimizer='cmaes',cma_evaluations=800)
    print(f"  local least squares alone: RMSE {loc['rmse_px']:.1f} px; CMA-ES + polish: {res['rmse_px']:.2f} px ({res['cma']['evaluations']} evaluations)",flush=True)
    t=res['transmission']
    assert abs(res['ratio']-TRUE_R)<2e-3 and np.allclose(t['gain'],TRUE_G,atol=5e-3) and np.allclose(t['offset_mm'],TRUE_O,atol=.1),(res['ratio'],t)
    assert res['rmse_px']<.5 and res['start_rmse_px']>5
    # Without transmission the same data cannot be matched by stiffness alone.
    no_t=fit_joints(store,model_path=XML,profile='exponential',fit_gain=False,max_evaluations=40,cma_evaluations=300)
    assert no_t['rmse_px']>3*res['rmse_px']+2
    print(f"PASS exponential + transmission: r {res['ratio']:.4f}, gains {np.round(t['gain'],3)}, offsets {np.round(t['offset_mm'],2)}; without gain RMSE {no_t['rmse_px']:.1f} px",flush=True)
    # 2. Independent joints: runs, improves, reports uncertainty, saves files, exports XML.
    ind=fit_joints(store,model_path=XML,profile='independent',start='uniform',smooth=.1,max_evaluations=15,optimizer='least_squares')
    assert len(ind['joints'])==20 and ind['rmse_px']<ind['start_rmse_px'] and all(np.isfinite(j['sigma_log']) for j in ind['joints'])
    for name in ('joint_fit.json','joint_fit.csv'):assert (store.root/name).exists()
    out=export_xml(ind,Path(XML).with_name('verify_joints_tmp.xml'))
    try:
        m=mujoco.MjModel.from_xml_path(str(out))
        assert np.allclose(m.jnt_stiffness[1:],[j['k_Nm_per_rad'] for j in ind['joints']],rtol=1e-9) and m.jnt_stiffness[0]==100
    finally:out.unlink()
    assert len(render_overlays(store,res))==len(Ds)
    stop=threading.Event();stop.set()
    try:fit_joints(store,model_path=XML,cancel=stop)
    except JointFitCancelled:pass
    else:raise AssertionError('Cancel ignored')
    print('PASS independent joints: 20 weights with σ, smoothness, joint_fit.json/csv, XML export (joint 1 untouched), overlays, cancel',flush=True)
    # 3. GUI tab.
    w=Window(cfg,fit,{},False);w.show();w.open_campaign();d=w.campaign_window;d.load(store.root/'campaign.json')
    assert d.joint_result is not None
    d.j_profile.setCurrentIndex(1);d.j_evals.setValue(30);d.joint_button.click()
    end=time.monotonic()+300
    while d.busy and time.monotonic()<end:app.processEvents();time.sleep(.02)
    assert not d.busy and 'exponential r = 0.9' in d.joint_text.text() and 'transmission gain 0.78' in d.joint_text.text(),d.joint_text.text()
    out=Path(os.environ.get('SPIROB_TEST_OUTPUT','/tmp/spirob-v094-verification'));out.mkdir(parents=True,exist_ok=True)
    d.resize(1336,728);d.tabs.setCurrentIndex(4);app.processEvents();d.j_canvas.draw();d.grab().save(str(out/'campaign_joints.png'))
    for widget in (d.joint_button,d.joint_export,d.j_canvas,d.joint_text):
        at=widget.mapTo(d,widget.rect().topLeft());assert at.x()>=0 and at.y()>=0 and at.x()+widget.width()<=d.width() and at.y()+widget.height()<=d.height()
    d.close();w.close();app.processEvents()
    print('PASS GUI: Joints + transmission tab fits, reloads and draws',flush=True)
# 4. Model check: the Lab XML matches the horizontal rig; a hanging (vertical) XML is refused.
from model_check import check_model,check_config
ok,lines=check_config();assert ok,lines
text=Path(XML).read_text().replace('<body name="robot_base" pos="0.0 0.0 0.09" euler="-1.5708 0 0">','<body name="robot_base" pos="0.0 0.0 0.09">')
bad=Path(XML).with_name('verify_vertical_tmp.xml');bad.write_text(text)
try:
    ok,lines=check_model(bad);assert not ok and any(l.startswith('FAIL all hinge axes vertical') for l in lines),lines
finally:bad.unlink()
print('PASS model check: horizontal plane at 90 mm accepted, vertical XML refused',flush=True)
print('PASS all v0.9.4 joint/transmission checks',flush=True)
