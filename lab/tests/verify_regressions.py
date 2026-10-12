"""October 3 failures: settled JSON, preview storage, transport/analysis isolation."""
import argparse
import copy
from collections import deque
import json
import os
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
p=argparse.ArgumentParser();p.add_argument('--capture-source',required=True);args=p.parse_args()
sys.path.insert(0,str(Path(args.capture_source).resolve()))
from analysis import Comparator
from connection import camera_get
from spirob_live.capture import LiveCameraService
from spirob_live.server import add_observer
from spirob_experiment.camera import SyntheticCamera,serve
from spirob_experiment.camera_client import CameraClient
config=json.loads(Path('config.json').read_text());fit=json.loads(Path('calibration/pilot_fit.json').read_text())

# This formerly returned numpy.bool_, which broke the live JSON log after 2 s.
model=Comparator(config,fit)
h=deque();point=np.zeros((3,2))
model.stable_shape(h,0.,point)
for t in np.arange(.2,4.5,.2):
    settled=model.stable_shape(h,float(t),point)
    json.dumps({'virtual_settled':settled,'settled':True and settled})
assert type(settled) is bool and settled
print('PASS: settled transition remains JSON-serializable after two seconds')

with tempfile.TemporaryDirectory() as folder:
    token='x'*40
    service=LiveCameraService(folder,token,lambda:SyntheticCamera(fps=30))
    server=add_observer(serve(service,'127.0.0.1',0),service)
    thread=threading.Thread(target=server.serve_forever);thread.start()
    url='http://127.0.0.1:'+str(server.server_port)
    client=CameraClient(url,token,timeout=2)
    root=Path(folder)/'TEST';root.mkdir()
    try:
        client.start('TEST','a'*32)
        deadline=time.monotonic()+3
        while client.status()['frame_count']<12 and time.monotonic()<deadline: time.sleep(.02)
        capture=service.capture
        assert list((capture.path/'frames').iterdir())==[]
        assert capture.recording()['bytes']==0
        assert 'image_b64' in camera_get(url,token,'/packet')
        context={'coordinator_monotonic_s':time.monotonic(), 'clock_before':client.clock(),
                 'trial':{'rest_lengths_mm':[220,220]}}
        client.call('/context',dict(client.identity,context=context))
        original_read=Path.read_text
        def guarded_read(path,*a,**kw):
            if path.name=='run.json': raise PermissionError('simulated Windows sharing violation')
            return original_read(path,*a,**kw)
        with patch.object(Path,'read_text',guarded_read):
            assert 'image_b64' in camera_get(url,token,'/packet')
        assert not (root/'run.json').exists()
        print('PASS: live preview writes zero JPEGs and does not read run.json')
        capture.record_limit=40000
        client.call('/record',dict(client.identity,enabled=True))
        deadline=time.monotonic()+3
        while capture.recording()['active'] and time.monotonic()<deadline:
            client.status();time.sleep(.02)
        rec=capture.recording();assert not rec['active'] and rec['bytes']<=40000 and rec['frames']>0
        before=client.status()['frame_count'];images=len(list((capture.path/'frames').glob('*.jpg')))
        time.sleep(.2)
        assert client.status()['frame_count']>before
        assert images==len(list((capture.path/'frames').glob('*.jpg')))
        assert capture.buffer_bytes<=capture.buffer_limit
        assert capture.status()['state']=='STREAMING'
        print('PASS: explicit recording obeys byte limit while preview keeps streaming')
        capture.record_limit=1000000;capture.record_seconds=.1
        capture.record_elapsed=0.
        client.call('/record',dict(client.identity,enabled=True));time.sleep(.25)
        assert not capture.recording()['active']
        print('PASS: elapsed recording cap also stops recording')
        assert client.stop()['thread_exited']
        # A delayed camera open must still cleanly observe a pending stop.
    finally:
        if service.capture: service.capture.stop()
        server.shutdown();thread.join(3);server.server_close()

# The real Preview object must deliver raw video while compare() is slow/fails.
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from interface import Preview,recorded_packet,QApplication
app=QApplication.instance() or QApplication([])
packet=recorded_packet(config,2);counter=[0];raw=[]
class Link:
    url='test';token='x'*40
link=Link()
def source(*a,**kw):
    q=copy.deepcopy(packet);counter[0]+=1;q['frame_index']=counter[0]
    q['t_s']=counter[0]/15;return q
class SlowComparator(Comparator):
    def compare(self,*a): time.sleep(.5);raise ValueError('injected slow analysis')
with patch('interface.camera_get',source),patch('interface.Comparator',SlowComparator),patch('interface.measure',lambda *a:(None,None)):
    preview=Preview(config,fit,link,False)
    preview.video.connect(lambda item:raw.append(item['packet']['frame_index']))
    preview.start();deadline=time.monotonic()+1.4
    while time.monotonic()<deadline:
        app.processEvents();time.sleep(.01)
    preview.stop.set();assert preview.wait(2500)
assert len(set(raw))>=10,len(raw)
print('PASS: slow/failing analysis does not stop live video delivery')
