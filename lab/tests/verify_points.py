"""Landmark geometry, frame-local corrections, and explicit-only exports."""
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import time
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
import numpy as np
from point_inspector import correspondences, PointInspector
from interface import QApplication,Window

observed={'curve_px':[[0,0],[300,0]]}
layout=dict(ids=['j_002','j_003','tip'],tail_m=[.4,.1,0],pixels_per_m=1000,model_px=[[0,3],[203,4],[300,0]])
rows=correspondences(observed,layout)
assert rows[0]['physical_px'] is None,'Out-of-view joint was clamped onto the visible backbone'
assert rows[1]['physical_px']==[200.,0.] and rows[1]['distance_px']==5
moved=correspondences(observed,layout,tip_offset_px=10,arc_scale=1.1,manual={'j_002':[30,40]},excluded=['j_003'])
assert moved[0]['source']=='manual' and moved[0]['physical_px']==[30,40]
assert np.allclose(moved[1]['physical_px'],[180,0]) and not moved[1]['included']
assert correspondences(observed,layout,'centres')[1]['physical_px']==[250.,0.]
config=json.loads(Path('config.json').read_text());fit=json.loads(Path('calibration/pilot_fit.json').read_text())
app=QApplication([]);window=Window(config,fit,None,True);window.show()
def until(predicate,seconds=10):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        app.processEvents()
        if predicate():return
        time.sleep(.02)
    raise AssertionError('Timed out')
until(lambda:window.latest and window.latest.get('result'))
model=window.preview.engine.model
assert len(window.latest['result']['landmarks']['ids'])==21
world=np.vstack([model.d.xanchor[1:],model.d.site('tip_site').xpos])
assert np.allclose(model.joint_curve(),np.c_[world[:,0]-model.reference_origin[0],model.reference_origin[1]-world[:,1]])
assert np.ptp(np.diff(model.joint_tail_m))>.0001,'Tapered joint spacing was flattened'
parameters=copy.deepcopy(window.preview.engine.p);result=copy.deepcopy(window.latest['result'])
with tempfile.TemporaryDirectory() as directory:
    previous=Path.cwd();os.chdir(directory)
    try:
        window.inspect_points();app.processEvents();inspector=window.inspector
        assert list(Path('.').iterdir())==[], 'Opening inspector wrote a file'
        assert len(inspector.rows)==21
        i=next(i for i,r in enumerate(inspector.rows) if r['physical_px'] is not None)
        inspector.table.selectRow(i);app.processEvents();inspector.x.setValue(inspector.x.value()+2)
        assert inspector.rows[i]['source']=='manual'
        assert window.preview.engine.p==parameters and window.latest['result']==result
        assert list(Path('.').iterdir())==[], 'Adjusting inspector wrote a file'
        path=inspector.export(Path(directory)/'explicit_save.png')
        assert path.is_file() and path.with_suffix('.csv').is_file()
        metadata=json.loads(path.with_suffix('.json').read_text())
        assert metadata['manual'] and metadata['frame_index']==window.latest['packet']['frame_index']
        inspector.capture();assert inspector.manual=={} and inspector.excluded==set()
        inspector.mode.setCurrentIndex(1);assert len(inspector.rows)==20
        inspector.mode.setCurrentIndex(0);app.processEvents()
        inspector.grab().save(str(ROOT/'preview_points.png'))
    finally:os.chdir(previous)
window.close();app.processEvents()
print('PASS: real anchors, tapered spacing, invisible points, per-frame corrections, unchanged fit, explicit-only PNG/CSV/JSON export')
