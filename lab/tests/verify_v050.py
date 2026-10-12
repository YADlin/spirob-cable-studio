"""Checks for calibration, mapping, landmark transfer, sweep application and storage.

No real serial devices, cameras or SSH are used. Screenshots and exports go to
a caller-selected test output folder, never to normal experiment directories.
"""
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import queue
import threading
from unittest.mock import patch
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
from analysis import Comparator, point_cost
from landmarks import correspondences, placement_profile
from interface import QApplication, Window, recorded_packet
from spirob_analysis.measurement import measure
from PySide6.QtGui import QPalette
from PySide6.QtWidgets import QMessageBox

config=json.loads(Path('config.json').read_text());fit=json.loads(Path('calibration/pilot_fit.json').read_text())
engine=Comparator(config,fit)
packet=recorded_packet(config,2)
import base64
_,observation=measure(base64.b64decode(packet['image_b64']),config)
original=engine.compare_still(packet,observation)
# Different physical rest lengths must be subtracted BEFORE reordering axes.
rests={'rest_lengths_mm':[200.,230.]};lengths=np.array([203.,237.])
setup=engine.setup();setup.update(mapping=[1,0],signs=[1,-1]);engine.set_setup(setup)
expected=engine.model.controls([7.,-3.],engine.p['mean_offset_mm'],engine.p['difference_offset_mm'])
assert np.allclose(engine.controls(lengths,rests),expected)
assert np.allclose(packet['rest_lengths_mm'],[220,220]),'Mapping changed physical commands'
# Swapping channels and the corresponding physical inputs preserves the model input.
swapped=copy.deepcopy(packet)
for row in swapped['motor']:row['axis']=1-row['axis']
setup.update(signs=[1,1]);engine.set_setup(setup)
replayed=engine.compare_still(swapped,observation)
assert np.allclose(replayed['predicted_points_px'],original['predicted_points_px'])
assert np.allclose(replayed['tendons']['actual_mm'],engine.model.d.ten_length*1000)
assert np.allclose(replayed['tendons']['demanded_mm'],engine.model.d.ctrl*1000)
assert not np.allclose(replayed['tendons']['actual_mm'],replayed['tendons']['demanded_mm'])
# Alignment changes projection, not physical pose or camera measurements.
before_q=engine.model.d.qpos.copy();before_observation=copy.deepcopy(observation)
setup['translation_x_px']+=25;setup['translation_y_px']-=13;engine.set_setup(setup)
shifted=engine.compare_still(swapped,observation)
assert np.allclose(np.asarray(shifted['predicted_points_px'])-replayed['predicted_points_px'],[25,-13])
assert np.allclose(before_q,engine.model.d.qpos) and observation==before_observation
print('PASS mapping, asymmetric rest subtraction, sign, tendon telemetry and fixed projection')

engine=Comparator(config,fit);result=engine.compare_still(packet,observation)
rows=correspondences(observation,result['landmarks'])
profile=placement_profile(observation,rows,'joints',packet['frame_index'],(1080,1920,3),True)
setup=engine.setup();setup['profile']=profile;engine.set_setup(setup)
joint_result=engine.compare_still(packet,observation)
assert joint_result['observation']['landmark_ids']
predicted=engine.transform(engine.model.landmark_points(joint_result['observation']['landmark_ids']))
assert np.allclose(predicted,joint_result['predicted_points_px'])
assert point_cost(joint_result['observation']['points_px'],predicted)==joint_result['cost_px2']
# Moving the measured curve moves estimated physical landmarks; red model points never set them.
moving=copy.deepcopy(observation);moving['curve_px']=(np.asarray(moving['curve_px'])+[17,9]).tolist()
old=engine.landmark_rows(observation);new=engine.landmark_rows(moving)
for a,b in zip(old,new):
    if a['physical_px'] is not None:
        assert np.allclose(np.asarray(b['physical_px'])-a['physical_px'],[17,9])
bad=copy.deepcopy(rows);i=next(i for i,r in enumerate(bad) if r['physical_px'] is not None)
bad[i]['physical_px']=[0.,0.]
try:placement_profile(observation,bad,'joints',1,(1080,1920,3))
except ValueError:pass
else:raise AssertionError('Large cross-track manual edit was transferred')
scan=engine.scan(joint_result,7,.2,1.2)
assert all(.2<=k<=1.2 for k in scan['k2'])
valid=[c for c in scan['cost_px2'] if c is not None]
assert scan['minimum']['cost_px2']==min(valid)
assert scan['setup']['profile']==profile
try:engine.scan(joint_result,7,1.2,.2)
except ValueError:pass
else:raise AssertionError('Reversed interval accepted')
print('PASS reusable image-only joint spacing, exact joint objective and bounded frozen sweep')

app=QApplication([])
def until(predicate,seconds=20):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        app.processEvents()
        if predicate():return
        time.sleep(.02)
    raise AssertionError('UI condition timed out')

output=Path(os.environ.get('SPIROB_TEST_OUTPUT','/tmp/spirob-v050-verification'));output.mkdir(parents=True,exist_ok=True)
window=Window(config,fit,None,True);window.show()
try:
    until(lambda:window.latest and window.latest.get('result'))
    # Simulated base picker respects rotation/scale and j_001 rather than j_002.
    window.open_setup();dialog=window.setup_dialog
    dialog.x.setValue(900);dialog.y.setValue(80);dialog.rotation.setValue(12);dialog.scale.setValue(4200)
    draft=dialog.values();check=Comparator(config,fit);check.set_setup(draft)
    assert np.allclose(check.joint_preview()['base_px'],[900,80],atol=.02)
    dialog.close()
    # Sweep stays in RAM; selected minimum is explicitly applied and acknowledged.
    window.scan_min.setValue(.5);window.scan_max.setValue(1.2);window.scan_count.setValue(9)
    until(lambda:window.scan_button.isEnabled())
    window.scan_button.click();until(lambda:window.sweep_result is not None)
    best=window.sweep_result['minimum']['k2'];window.apply_scan_button.click()
    until(lambda:window.latest and window.latest.get('result') and abs(window.latest['result']['k2']-best)<1e-7)
    assert 'Applied in simulation' in window.applied_label.text()
    path=window.export_sweep(output/'sweep.png')
    exported=json.loads(path.with_suffix('.json').read_text())
    assert exported['minimum']==window.sweep_result['minimum'] and path.with_suffix('.csv').is_file()
    # Raw video must not overwrite the analytical status line.
    snapshot=copy.deepcopy(window.latest);window.refresh_summary(window.inspector_snapshot());text=window.status.text()
    raw=dict(snapshot,raw=True,result=None,observed=None,message='Live video; waiting for a current comparison')
    for _ in range(10):window.show_comparison(raw)
    assert window.status.text()==text
    window.show_comparison(snapshot)
    # Both palettes include dialogs and all palette groups; images retain source colours.
    with tempfile.TemporaryDirectory() as tmp:
        os.chdir(tmp)
        for name,foreground,background in [('Dark','#ffffff','#111111'),('Light','#000000','#ffffff')]:
            window.theme_box.setCurrentText(name);app.processEvents()
            palette=app.palette()
            for group in (QPalette.Active,QPalette.Inactive):
                assert palette.color(group,QPalette.WindowText).name()==foreground
                assert palette.color(group,QPalette.Window).name()==background
            window.grab().save(str(output/f'{name.lower()}.png'))
        preferences=json.loads(Path('lab.local.json').read_text());assert 'comparison_setup' not in preferences
    os.chdir(ROOT)
    # A setup change invalidates the old sweep for apply, while export stays available.
    setup=copy.deepcopy(window.comparison_setup);setup['translation_x_px']+=3
    window.apply_setup(setup);window.tick()
    assert not window.apply_scan_button.isEnabled() and window.save_scan_button.isEnabled()
    until(lambda:window.latest and window.latest.get('result'))
    window.inspect_points();app.processEvents()
    window.inspector.score_profile.setChecked(True);window.inspector.apply_profile()
    until(lambda:window.latest and window.latest.get('result') and window.latest['result']['observation'].get('landmark_ids'))
    assert window.preview.engine.profile['use_for_score']
    window.inspector.grab().save(str(output/'inspector.png'))
finally:
    window.close();app.processEvents();os.chdir(ROOT)

print('PASS base picker, stable status, themes, sweep selection/apply/export and profile integration; run retention is checked in verify_v051.py')
