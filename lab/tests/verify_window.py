"""Offscreen window checks. Fake transport records clicks; it cannot actuate motors."""
import copy
import json
import os
from pathlib import Path
import queue
import sys
import threading
import time
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
from interface import QApplication,Window
config=json.loads(Path('config.json').read_text());fit=json.loads(Path('calibration/pilot_fit.json').read_text())
app=QApplication([])
def until(predicate, seconds=8):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        app.processEvents()
        if predicate(): return
        time.sleep(.02)
    raise AssertionError('Window condition timed out')
window=Window(config,fit,None,True);window.show()
until(lambda:window.latest and window.latest.get('result'))
assert not window.move_button.isEnabled() and not window.arm_button.isEnabled()
first=window.latest['result']['cost_px2']
window.kbox.setValue(.4);window.apply_button.click()
until(lambda:window.latest and window.latest.get('result') and abs(window.latest['result']['k2']-.4)<1e-8)
assert abs(window.latest['result']['cost_px2']-first)>1
window.kbox.setValue(fit['candidate']['parameters']['k2']);window.apply_button.click()
until(lambda:window.latest and window.latest.get('result') and window.latest['result']['k2']>.8)
until(lambda:window.scan_button.isEnabled())
window.scan_button.click()
until(lambda:not window.scan_busy and len(window.scan_line.get_xdata())>1, 20)
assert window.preview.engine.p['k2']>.8
window.grab().save('preview.png')
item=copy.deepcopy(window.latest)
window.close();app.processEvents()

class FakeLink:
    def __init__(self):
        self.events=queue.Queue();self.cancel=threading.Event();self.commands=[]
    def touch(self): pass
    def preview_received(self, age): pass
    def send(self,action,**values): self.commands.append((action,values))
    def disconnect(self):
        return dict(path='SIMULATED',error=None,motor_shutdown=dict(worker_exited=True,disable_confirmed=[True,True]))
link=FakeLink()
live=Window(config,fit,{},False);live.show();live.session=link
live.remote={'phase':'READY_DISABLED'};live.at_rest.setChecked(True)
item.update(received=time.monotonic(),displayed_age_s=0.)
live.show_comparison(item);live.tick()
assert live.arm_button.isEnabled() and not live.move_button.isEnabled()
live.arm_button.click();assert link.commands[-1]==('arm',{'at_rest':True})
assert live.setup_enable.isEnabled()
live.setup_enable.click();assert link.commands[-1][0]=='setup_enable'
live.remote={'phase':'SETUP_HOLD'};live.tick()
assert live.reference_button.isEnabled() and all(b.isEnabled() for b in live.jog_buttons)
assert not live.mean_button.isEnabled() and not live.axis_buttons[0].isEnabled()
live.jog_buttons[0].click();assert link.commands[-1]==('jog',{'axis':0,'delta_mm':-.5,'speed_mm_s':.5})
live.reference_button.click();assert link.commands[-1]==('reference',{'at_rest':True})
live.remote={'phase':'HOLDING'};live.tick();assert live.move_button.isEnabled()
live.mean_button.click();assert link.commands[-1]==('mean',{'mean_mm':220.,'speed_mm_s':1.})
live.axis_targets[1].setValue(221);live.axis_buttons[1].click()
assert link.commands[-1]==('axis',{'axis':1,'length_mm':221.,'speed_mm_s':1.})
live.move_button.click();assert link.commands[-1]==('move',{'mean_mm':220.,'difference_mm':20.,'speed_mm_s':1.})
live.record_button.click();assert link.commands[-1]==('record',{'enabled':True})
live.hold_button.click();assert link.commands[-1][0]=='hold'
live.latest['received']-=5;live.tick()
assert not live.move_button.isEnabled() and not live.arm_button.isEnabled() and live.hold_button.isEnabled()
assert not live.mean_button.isEnabled() and not any(b.isEnabled() for b in live.jog_buttons)
live.disconnect_button.click();until(lambda:live.session is None)
assert 'release confirmed' in live.banner.text()
live.close();app.processEvents()
print('PASS: recorded example, parameter edit, frozen sweep, arm/move/hold command wiring, stale-data gating, disconnect acknowledgement')
