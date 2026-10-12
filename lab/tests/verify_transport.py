"""Exercise the real camera HTTP routes and lease behavior using SyntheticCamera."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from connection import camera_get
from spirob_live.server import add_observer

parser=argparse.ArgumentParser();parser.add_argument('--capture-source', required=True)
args=parser.parse_args();sys.path.insert(0,str(Path(args.capture_source).resolve()))
from spirob_experiment.camera import SyntheticCamera,serve
from spirob_live.capture import LiveCameraService as CameraService
from spirob_experiment.camera_client import CameraClient
with tempfile.TemporaryDirectory() as directory:
    token='x'*40
    service=CameraService(directory,token,lambda:SyntheticCamera(fps=25))
    server=add_observer(serve(service,'127.0.0.1',0),service)
    thread=threading.Thread(target=server.serve_forever);thread.start()
    url='http://127.0.0.1:'+str(server.server_port)
    client=CameraClient(url,token)
    try:
        assert camera_get(url,token)['version']=='0.7.0'
        assert camera_get(url,token)['capture_state']=='IDLE'
        folder=Path(directory)/'CHECK';folder.mkdir()
        run={'state':'READY_DISABLED','coordinator_monotonic_s':time.monotonic(),
             'clock_before':client.clock(),'trial':{'rest_lengths_mm':[220,220]}}
        (folder/'run.json').write_text(json.dumps(run))
        client.start('CHECK','1'*32)
        deadline=time.monotonic()+3
        while client.status()['frame_count']<5 and time.monotonic()<deadline: time.sleep(.02)
        lease=service.capture.lease
        packet=camera_get(url,token,'/packet')
        assert 'image_b64' in packet and not packet['motor_active']
        assert service.capture.lease==lease, 'Observer renewed camera control lease'
        for credential,expected in [('wrong',401),(token,409)]:
            try:
                urlopen(Request(url+'/shutdown-service',b'{}',{'Authorization':'Bearer '+credential}),timeout=2)
            except HTTPError as exc: assert exc.code==expected
            else: raise AssertionError('Shutdown route accepted an invalid request')
        assert client.stop()['thread_exited']
        with urlopen(Request(url+'/shutdown-service',b'{}',{'Authorization':'Bearer '+token}),timeout=2) as response:
            assert response.status==200
        thread.join(3)
        assert not thread.is_alive()
        print('PASS: authenticated preview, disabled-motor preview, no lease renewal, active-session shutdown rejection, clean service exit')
    finally:
        if service.capture: service.capture.stop()
        if thread.is_alive(): server.shutdown();thread.join(3)
        server.server_close()
