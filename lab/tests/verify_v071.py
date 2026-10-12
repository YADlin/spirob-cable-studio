"""Independent view controls: rendered layers and camera/session ownership.

Only synthetic/recorded images and fake links are used; no hardware is opened.
"""
import copy
import io
import base64
import json
import os
from pathlib import Path
import queue
import sys
import threading
import time
from unittest.mock import patch
import numpy as np
from PIL import Image

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
from interface import QApplication,Window,CameraView,recorded_packet
from analysis import Comparator
from PySide6.QtGui import QImage

app=QApplication([])
config=json.loads(Path('config.json').read_text());fit=json.loads(Path('calibration/pilot_fit.json').read_text())
out=Path('/tmp/spirob-v071-verification');out.mkdir(exist_ok=True)

def until(predicate,seconds=8):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        app.processEvents()
        if predicate():return
        time.sleep(.02)
    raise AssertionError('Timed out waiting for window state')

def pixels(view):
    app.processEvents()
    image=view.grab().toImage().convertToFormat(QImage.Format_RGB888)
    return np.asarray(image.bits()).reshape(image.height(),image.bytesPerLine())[:,:image.width()*3].reshape(image.height(),image.width(),3).copy()

def colours(rgb):
    r,g,b=np.moveaxis(rgb.astype(int),-1,0)
    def near(colour):return np.all(np.abs(rgb.astype(int)-colour)<3,axis=2)
    return dict(red=int(((r>200)&(g<120)&(b<120)).sum()),
                green=int((near([0,173,142])|near([0,239,182])).sum()),
                camera=int(near([0,0,240]).sum()),
                connectors=int(((r>80)&(g>70)&(r>b*1.1)&(g>b))[125:190,100:120].sum()))

# Rendered pixels prove that all camera/model marks and connectors obey the
# separate controls, rather than only checking the state of the buttons.
view=CameraView();view.resize(640,360);view.show()
rgb=np.zeros((360,640,3),np.uint8);rgb[:,:,2]=240
view.set_frame(rgb,[[100,100],[450,150]],[[120,210],[460,240]],
    [dict(included=True,physical_px=[120,210],model_px=[100,100])],[100,100])
for camera,backbone in ((False,False),(True,False),(False,True),(True,True)):
    view.set_visibility(camera,backbone);c=colours(pixels(view))
    assert bool(c['camera'])==camera,(camera,backbone,c)
    assert bool(c['green'])==camera,(camera,backbone,c)
    assert bool(c['red'])==backbone,(camera,backbone,c)
    assert bool(c['connectors'])==(camera and backbone),(camera,backbone,c)
    view.grab().save(str(out/f'layers_camera_{int(camera)}_backbone_{int(backbone)}.png'))
view.close()
print('PASS all four rendered combinations: camera, physical points, red backbone, base and paired connectors')

class FakeSetup:
    created=[]
    def __init__(self,rig):
        self.events=queue.Queue();self.cancel=threading.Event();self.stopped=False
        self.created.append(self)
    def connect(self):pass
    def touch(self):pass
    def disconnect(self):self.stopped=True

class FakeRun:
    created=[]
    def __init__(self,rig,logs):
        self.events=queue.Queue();self.cancel=threading.Event();self.stopped=False
        self.received=[];self.commands=[];self.created.append(self)
    def connect(self):assert all(s.stopped for s in FakeSetup.created)
    def touch(self):pass
    def preview_received(self,age):self.received.append(age)
    def send(self,action,**values):self.commands.append((action,values))
    def disconnect(self):
        self.stopped=True
        return dict(path='SIMULATED',error=None,motor_shutdown=dict(worker_exited=True,disable_confirmed=[True,True]))

packet=recorded_packet(config,2)
rgb=np.asarray(Image.open(io.BytesIO(base64.b64decode(packet['image_b64']))).convert('RGB'))
engine=Comparator(config,fit)
result=engine.compare_still(packet,None)
item=dict(rgb=rgb,packet=packet,result=result,observed=None,parameters=dict(engine.p),d2=engine.d2,
    setup=engine.setup(),landmarks=result['landmarks'],message='Recorded test frame',
    displayed_age_s=0.,received=time.monotonic(),processing_s=0.)

with patch('interface.CameraSetupSession',FakeSetup),patch('interface.WorkstationSession',FakeRun):
    window=Window(config,fit,{},False);window.start_preview=lambda:None;window.show()
    try:
        assert not window.camera_toggle.isChecked() and window.backbone_toggle.isChecked()
        assert colours(pixels(window.camera))['red']>0
        assert window.camera.frame is None and window.latest is None
        assert window.manual_source() is None and not window.scan_button.isEnabled()
        window.backbone_toggle.click();assert not colours(pixels(window.camera))['red']
        window.backbone_toggle.click();assert colours(pixels(window.camera))['red']>0
        assert not FakeSetup.created and not FakeRun.created
        window.grab().save(str(out/'backbone_only_window.png'))

        # Camera alone opens a camera lease, never a motor run or timer.
        window.backbone_toggle.click();window.camera_toggle.click()
        until(lambda:window.setup_session and not window.setup_connecting)
        first=window.setup_session;assert len(FakeSetup.created)==1 and not FakeRun.created
        window.show_comparison(dict(item,setup_preview=True,received=time.monotonic()))
        window.tick();assert not window.camera.show_backbone and window.camera.show_camera
        window.backbone_toggle.click();assert window.setup_session is first and not first.stopped
        window.grab().save(str(out/'both_window.png'))
        window.camera_toggle.click();until(lambda:window.setup_session is None and not window.setup_stopping)
        assert first.stopped and window.backbone_toggle.isChecked()
        assert window.camera.frame is None and colours(pixels(window.camera))['red']>0
        assert window.last_video is None and window.manual_source() is None

        # Connect closes setup capture and preserves both independent choices.
        window.backbone_toggle.click();window.camera_toggle.click()
        until(lambda:window.setup_session and not window.setup_connecting)
        second=window.setup_session
        window.duration.setCurrentIndex(2)
        window.connect();until(lambda:window.session and not window.connecting)
        link=window.session
        assert second.stopped and window.setup_session is None
        assert window.camera_toggle.isChecked() and not window.backbone_toggle.isChecked()
        assert link.max_seconds==1800

        # During a run, view hiding must not close capture, disable freshness
        # checks, change parameters, or send a motor command.
        window.remote={'phase':'HOLDING'}
        window.show_comparison(dict(item,received=time.monotonic()));window.tick()
        window.camera_toggle.click();window.backbone_toggle.click()
        assert not window.camera.show_camera and window.camera.show_backbone
        before=len(link.received)
        window.show_comparison(dict(item,received=time.monotonic()));window.tick()
        assert len(link.received)>before and window.move_button.isEnabled()
        assert not link.stopped and not link.commands
        assert 'acquisition' in window.display_status.text()
        window.backbone_toggle.click();assert not window.camera.show_backbone
        window.show_comparison(dict(item,received=time.monotonic()-3))
        window.latest=dict(item,received=time.monotonic()-3);window.tick()
        assert not window.move_button.isEnabled() and window.hold_button.isEnabled()
        window.disconnect();until(lambda:window.session is None and not window.closing)
        assert link.stopped and not window.camera_toggle.isChecked() and not window.backbone_toggle.isChecked()
        assert window.manual_source() is None
        window.backbone_toggle.click();assert colours(pixels(window.camera))['red']>0
    finally:
        window.close();app.processEvents()

class FailingSetup(FakeSetup):
    def connect(self):raise RuntimeError('Injected camera open failure')

with patch('interface.CameraSetupSession',FailingSetup):
    window=Window(config,fit,{},False);window.start_preview=lambda:None;window.show()
    try:
        window.camera_toggle.click()
        until(lambda:not window.camera_toggle.isChecked() and not window.setup_stopping)
        assert window.backbone_toggle.isChecked() and colours(pixels(window.camera))['red']>0
        assert 'Injected camera open failure' in window.command_status.text()
        assert window.session is None and not window.scan_button.isEnabled()
    finally:window.close();app.processEvents()
print('PASS backbone without camera/SSH/timer; camera-only lease; independent transitions; hidden-view freshness; shutdown and camera-failure recovery')
print('All v0.7.1 checks passed. No physical hardware was operated.')
