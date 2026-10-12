"""Manual correction, tracking failure, exact scoring and fixed-base scaling.

Uses synthetic image features, recorded frames and offscreen Qt. No hardware.
"""
import base64
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import cv2
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
from joint_tracking import manual_profile, manual_observation, ManualJointTracker
from landmarks import correspondences, scale_from_profile
from analysis import Comparator, point_cost
from interface import QApplication, Window, Preview, recorded_packet
from point_inspector import PointInspector
from spirob_analysis.measurement import measure
from PySide6.QtWidgets import QMessageBox

config = json.loads(Path('config.json').read_text())
fit = json.loads(Path('calibration/pilot_fit.json').read_text())
for name in ('model', 'session'): config[name] = str(ROOT/config[name])
out = Path('/tmp/spirob-v061-verification'); out.mkdir(exist_ok=True)
names = ['j_018', 'j_019', 'j_020', 'j_021', 'tip']
lengths = np.array([80., 70., 65., 60.])
def chain(angles, base=(250., 100.)):
    return np.vstack([base, base + np.cumsum(lengths[:, None] * np.c_[np.cos(angles), np.sin(angles)], axis=0)])
points = chain([1.5, 1.4, 1.3, 1.2])
rng = np.random.default_rng(22)
patches = [rng.integers(20, 240, (23, 23), dtype=np.uint8) for _ in names]
def frame(points):
    image = np.full((600, 800), 150, np.uint8)
    for point, patch in zip(points, patches):
        # Translate the same unique image patch to the known subpixel location.
        tile = np.full(image.shape, 150, np.uint8); tile[89:112, 89:112] = patch
        shifted = cv2.warpAffine(tile, np.float32([[1,0,point[0]-100],[0,1,point[1]-100]]),
            (800,600), flags=cv2.INTER_LINEAR, borderValue=150)
        mask = shifted != 150; image[mask] = shifted[mask]
    return np.repeat(image[:, :, None], 3, axis=2)
rgb = frame(points)
rows = [dict(id=n, included=True, physical_px=p.tolist()) for n,p in zip(names,points)]
# Deliberately wrong automatic curve: all clicks are >15 px away.
wrong = dict(curve_px=(points-[45,0]).tolist())
profile = manual_profile(rows,'joints',1,rgb,True,True)
assert np.array_equal(list(profile['calibration_points_px'].values()), points)
assert np.allclose(profile['link_lengths_px'], lengths)
observed = manual_observation(profile,points,config)
layout = dict(ids=names,tail_m=(np.r_[lengths[::-1].cumsum()[::-1],0]/4000).tolist(),
    pixels_per_m=4000,model_px=(points+[10,-4]).tolist())
paired = correspondences(observed,layout,profile=profile)
assert np.array_equal([r['physical_px'] for r in paired],points)
assert all(r['physical_px'] is None for r in correspondences(wrong,layout,profile=profile))
assert abs(scale_from_profile(profile,layout)-4000.) < 1e-8
ref = dict(rgb=rgb,packet=dict(session='synthetic',frame_index=1))
tracker = ManualJointTracker(profile,config,ref)
assert tracker.update(rgb,('synthetic',1))['joint_points_px'] == profile['calibration_points_px']
for i in range(1,6):
    truth = chain(np.array([1.5,1.4,1.3,1.2]) + np.arange(4)*.002*i, (250.+2*i,100.+i))
    result = tracker.update(frame(truth),('synthetic',i+1))
    actual = np.array(result['curve_px'])
    assert np.max(np.linalg.norm(actual-truth,axis=1)) < 1., (actual,truth)
    assert np.allclose(np.linalg.norm(np.diff(actual,axis=0),axis=1),lengths,atol=1e-6)
saved = json.loads(json.dumps(profile))
reloaded = ManualJointTracker(saved,config)
reacquired = reloaded.update(rgb,('restarted',1))
assert np.max(np.linalg.norm(np.array(reacquired['curve_px'])-points,axis=1)) < 1.5
blank = np.full_like(rgb,150)
for t, image, key in [(tracker,blank,('synthetic',99)),
                       (ManualJointTracker(saved,config),blank,('new',1)),
                       (ManualJointTracker(saved,config,ref),rgb[:500],('synthetic',2))]:
    try:t.update(image,key)
    except ValueError as exc:assert 'Tracking paused' in str(exc)
    else:raise AssertionError('Lost/absent joint evidence was accepted')
# Repeating image features are ambiguous even with high correlation. Two
# equally good matches within the restart search must not choose arbitrarily.
ambiguous = rgb.copy()
tile = cv2.imdecode(np.frombuffer(base64.b64decode(saved['templates_png'][names[0]]),np.uint8),cv2.IMREAD_GRAYSCALE)
for dx in (-43,43):
    x,y=np.round(points[0]+[dx,0]).astype(int)
    ambiguous[y-20:y+21,x-20:x+21]=tile[:,:,None]
try:ManualJointTracker(saved,config).update(ambiguous,('ambiguous',1))
except ValueError as exc:assert 'unambiguous image match' in str(exc)
else:raise AssertionError('Ambiguous saved-image matches were accepted')
try:tracker.update(rgb,('synthetic',100))
except ValueError:pass
else:raise AssertionError('Failed track silently replayed calibration')
print('PASS off-curve exact clicks; rigid image tracking; restart evidence; missing/occluded/resized image pauses')

packet = recorded_packet(config,2)
rgb, automatic = measure(base64.b64decode(packet['image_b64']),config)
engine = Comparator(config,fit); baseline = engine.compare_still(packet,automatic)
rows = correspondences(automatic,baseline['landmarks'])
first = next(r for r in rows if r['physical_px'] is not None)
first['physical_px'][0] += 35.  # A concrete cross-track correction, kept EXACT.
profile = manual_profile(rows,'joints',packet['frame_index'],rgb,True,True)
corrected = manual_observation(profile,[profile['calibration_points_px'][n] for n in profile['chain_ids']],config)
setup = engine.setup(); setup['profile'] = profile; engine.set_setup(setup)
result = engine.compare_still(packet,corrected)
assert result['settled']
assert result['observation']['points_px'][0] == first['physical_px']
assert result['cost_px2'] == point_cost(result['observation']['points_px'],result['predicted_points_px'])
assert engine.compare_still(packet,automatic)['cost_px2'] is None
assert engine.compare_still(packet,None)['predicted_curve_px']
preview = Preview(config,fit,demo=True); preview.engine = engine
reference = dict(rgb=rgb,packet=packet)
preview.configure_tracker(reference)
initial_tracker = preview.tracker
shifted = copy.deepcopy(setup); shifted['translation_x_px'] += 2
preview.requests.put(('setup',(shifted,None))); preview.handle_requests()
assert preview.tracker is initial_tracker
assert preview.tracker.update(rgb,(packet['session'],packet['frame_index']))['curve_px'] == corrected['curve_px']
print('PASS J uses exact corrected anchors; wrong automatic measurement refused; red model independent; alignment preserves image track')

app = QApplication.instance() or QApplication([])
def warning(*args):raise AssertionError('Unexpected UI warning: '+str(args[2]))
QMessageBox.warning = warning
def until(predicate, seconds=30):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        app.processEvents()
        if predicate():return
        time.sleep(.015)
    raise AssertionError('UI condition timed out')
window=Window(config,fit,None,True);window.show()
try:
    until(lambda:window.latest and window.latest.get('result'))
    window.inspect_points();inspector=window.inspector
    first=next(r for r in inspector.rows if r['physical_px'] is not None)
    clicked=[first['physical_px'][0]+35.,first['physical_px'][1]]
    inspector.manual[first['id']]=clicked;inspector.rebuild()
    window.scan_min.setValue(.4);window.scan_max.setValue(1.3);window.scan_count.setValue(5)
    inspector.sweep_profile_button.click();until(lambda:window.sweep_result is not None)
    snapshot=window.sweep_snapshot;profile=window.comparison_setup['profile']
    assert snapshot['observed']['joint_points_px'][first['id']]==clicked
    assert snapshot['result']['observation']['points_px'][0]==clicked
    assert snapshot['result']['landmark_rows'][next(i for i,r in enumerate(inspector.rows) if r['id']==first['id'])]['physical_px']==clicked
    assert window.sweep_result['observation']==snapshot['result']['observation']
    path=window.export_pose(out/'pose.png');data=json.loads(path.with_suffix('.json').read_text())
    assert data['result']['observation']['points_px'][0]==clicked
    assert data['sweep']['observation']==data['result']['observation']
    window.open_setup();dialog=window.setup_dialog
    base=[dialog.x.value(),dialog.y.value()]
    dialog.match_length();values=dialog.values()
    check=Comparator(config,fit);check.set_setup(values)
    assert np.allclose(check.joint_preview()['base_px'],base,atol=.01)
    assert abs(values['pixels_per_m']-scale_from_profile(profile,window.reference_layout)) < .001
    dialog.apply()
    until(lambda:window.latest and window.latest.get('result') and not window.pending_comparison_change)
    assert window.preview.tracker.profile['calibration_id']==profile['calibration_id']
    with tempfile.TemporaryDirectory() as tmp:
        os.chdir(tmp);window.save_preferences(True)
        settings=json.loads(Path('lab.local.json').read_text())
        restored=Comparator(dict(config,comparison_setup=settings['comparison_setup']),fit)
        assert restored.profile['calibration_points_px'][first['id']]==clicked
    os.chdir(ROOT)
    inspector.grab().save(str(out/'inspector.png'));window.grab().save(str(out/'window.png'))
    # No automatic curve and no motor feedback must still allow manual placement.
    no_curve=copy.deepcopy(snapshot);no_curve.update(observed=None,result=None,setup=window.comparison_setup,
        message='No measurement',landmarks=window.reference_layout)
    no_curve['landmarks']['model_px']=None
    seen=[]
    empty=PointInspector(lambda:no_curve,apply_callback=lambda *a,**kw:seen.append((a,kw)))
    assert empty.snapshot is not None and all(r['physical_px'] is None for r in empty.rows)
    empty.manual={n:profile['calibration_points_px'][n] for n in profile['chain_ids']}
    empty.rebuild();empty.apply_profile()
    assert seen and seen[0][0][0]['calibration_points_px'][first['id']]==clicked
    empty.close();dialog.close();inspector.close()
    print('PASS actual Apply/sweep/export buttons preserve >15 px correction; scale fixes base; saved calibration; placement without automatic curve')
finally:window.close();app.processEvents();os.chdir(ROOT)
print('All v0.6.1 checks passed. No hardware was operated.')
