"""Native jog focus, stable layouts and actual Matplotlib navigation/clicks.

Recorded images and a fake command link only; no physical hardware is opened.
"""
import base64
import copy
import io
import json
import os
from pathlib import Path
import queue
import sys
import threading
import time
import numpy as np
from PIL import Image
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));os.chdir(ROOT)
from interface import QApplication,Window,recorded_packet
from point_inspector import PointInspector
from manual_analysis import prepare_capture
from analysis import Comparator
from PySide6.QtCore import Qt,QPoint,QRect,QSize
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QScrollArea
from matplotlib.backend_bases import MouseEvent

app=QApplication([]);config=json.loads(Path('config.json').read_text());fit=json.loads(Path('calibration/pilot_fit.json').read_text())
out=Path('/tmp/spirob-v072-verification');out.mkdir(exist_ok=True)
packet=recorded_packet(config,2)
rgb=np.asarray(Image.open(io.BytesIO(base64.b64decode(packet['image_b64']))).convert('RGB'))
engine=Comparator(config,fit);result=engine.compare_still(packet,None)
item=dict(rgb=rgb,packet=packet,result=result,observed=None,parameters=dict(engine.p),d2=engine.d2,
    setup=engine.setup(),landmarks=result['landmarks'],message='Recorded test frame',recorded_demo=True,
    displayed_age_s=0.,received=time.monotonic(),processing_s=0.)

class Link:
    def __init__(self):self.events=queue.Queue();self.cancel=threading.Event();self.commands=[]
    def touch(self):pass
    def preview_received(self,age):pass
    def send(self,action,**values):self.commands.append((action,values))
    def disconnect(self):return dict(path='SIMULATED',error=None,motor_shutdown=dict(worker_exited=True,disable_confirmed=[True,True]))

def process():
    app.processEvents();QTest.qWait(25);app.processEvents()

def rect(widget,ancestor):return QRect(widget.mapTo(ancestor,QPoint(0,0)),widget.size())

def inside(widget,window):
    assert widget.isVisible(),widget.objectName() or type(widget).__name__
    assert window.rect().contains(rect(widget,window)),(widget,rect(widget,window),window.rect())

window=Window(config,fit,{},False);window.session=Link();window.remote={'phase':'SETUP_HOLD'};window.show()
try:
    for width,height in ((1280,720),(1366,768),(1920,1080)):
        window.resize(width,height);window.motor_tabs.setCurrentWidget(window.setup_panel)
        window.show_comparison(dict(item,received=time.monotonic()));window.tick();process()
        assert window.size()==QSize(width,height),(window.size(),width,height)
        for widget in (window.camera,window.motor_tabs,window.hold_button,window.disconnect_button,*window.jog_buttons):inside(widget,window)
        # A Qt scroll area must never own the viewer or repeatedly used jogs.
        for widget in (window.camera,*window.jog_buttons):
            parent=widget.parentWidget()
            while parent:
                assert not isinstance(parent,QScrollArea)
                parent=parent.parentWidget()
        anchors={id(w):rect(w,window) for w in (window.camera,window.hold_button,*window.jog_buttons)}
        window.analysis_scroll.verticalScrollBar().setValue(window.analysis_scroll.verticalScrollBar().maximum())
        for i in range(8):
            window.remote={'phase':'SETUP_HOLD'}
            window.show_comparison(dict(item,received=time.monotonic()));window.tick();process()
            button=window.jog_buttons[i%4]
            scroll=window.analysis_scroll.verticalScrollBar().value()
            QTest.mouseClick(button,Qt.LeftButton);process()
            assert window.session.commands[-1]==('jog',dict(axis=(i%4)//2,delta_mm=.5 if i%2 else -.5,speed_mm_s=.5))
            window.remote={'phase':'MOVING'};window.tick();process()
            assert window.hold_button.hasFocus()
            assert window.motor_tabs.currentWidget() is window.setup_panel
            assert window.analysis_scroll.verticalScrollBar().value()==scroll
            assert all(rect(w,window)==anchors[id(w)] for w in (window.camera,window.hold_button,*window.jog_buttons))
            window.command_status.setText(('A changing command message, '*(1+i)))
        window.remote={'phase':'SETUP_HOLD'};window.show_comparison(dict(item,received=time.monotonic()));window.tick();process()
        window.grab().save(str(out/f'jog_{width}x{height}.png'))
        for tab in range(window.motor_tabs.count()):
            window.motor_tabs.setCurrentIndex(tab);process();inside(window.camera,window);inside(window.motor_tabs,window)
        window.motor_tabs.setCurrentIndex(0);window.motor_mode.setCurrentIndex(1);process()
        for widget in (*window.length_targets,*window.length_speeds,*window.length_buttons,window.length_move):inside(widget,window)
        window.analysis_tabs.setCurrentIndex(1);process();window.canvas.draw()
        for widget in (window.canvas,window.camera,window.hold_button,window.length_move):inside(widget,window)
        window.grab().save(str(out/f'plots_{width}x{height}.png'))
        window.analysis_tabs.setCurrentIndex(0);process()
    assert len(window.session.commands)==24
    print('PASS 24 native jog clicks at 1280×720, 1366×768 and 1920×1080: no scroll, no tab switch, stable control coordinates; Stop retains focus')
    window.resize(1366,768);window.command_status.setText('UI VERIFICATION — recorded image and simulated command link; no hardware')
    window.analysis_mode_box.setCurrentIndex(1);process();window.grab().save(str(out/'main_window.png'))

    captured=prepare_capture(item,config,fit)
    inspector=PointInspector(lambda:captured,window,manual_mode=True,manual_callback=lambda *args:None)
    inspector.resize(1366,768);inspector.show();process();inspector.canvas.draw()
    assert inspector.size()==QSize(1366,768),inspector.size()
    for widget in (inspector.canvas,inspector.table,inspector.x,inspector.y,inspector.sweep_profile_button,inspector.fit_button):inside(widget,inspector)

    def bounds():return np.array([inspector.ax.get_xlim(),inspector.ax.get_ylim()])
    def event(kind,x,y):
        pixel=inspector.ax.transData.transform((x,y))
        e=MouseEvent(kind,inspector.canvas,*pixel,button=1,**({'buttons':{1}} if kind=='motion_notify_event' else {}))
        inspector.canvas.callbacks.process(kind,e)
        return e
    initial=bounds()
    inspector.toolbar.zoom()
    event('button_press_event',740,100);event('motion_notify_event',1080,600);event('button_release_event',1080,600)
    inspector.canvas.draw();zoom=bounds()
    assert np.ptp(zoom[0])<400 and np.ptp(zoom[1])<600,zoom
    inspector.place_button.click();assert not inspector.toolbar.mode
    assert np.allclose(bounds(),zoom)
    for index,xy in enumerate(((900,220),(915,285),(925,360),(945,430))):
        inspector.table.selectRow(index);inspector.canvas.draw();assert np.allclose(bounds(),zoom)
        clicked=event('button_press_event',*xy);event('button_release_event',*xy)
        inspector.canvas.draw();assert np.allclose(bounds(),zoom)
        assert np.allclose(inspector.manual[inspector.rows[index]['id']],[clicked.xdata,clicked.ydata])
    inspector.x.setValue(inspector.x.value()+.2);process();assert np.allclose(bounds(),zoom)
    inspector.table.item(2,0).setCheckState(Qt.Unchecked);process();assert np.allclose(bounds(),zoom)
    inspector.table.item(2,0).setCheckState(Qt.Checked);process();assert np.allclose(bounds(),zoom)
    inspector.table.selectRow(18);inspector.table.verticalScrollBar().setValue(inspector.table.verticalScrollBar().maximum())
    table_scroll=inspector.table.verticalScrollBar().value()
    event('button_press_event',960,480);event('button_release_event',960,480);process()
    assert inspector.table.verticalScrollBar().value()==table_scroll
    assert np.allclose(bounds(),zoom)

    # Pan with the real toolbar, then return to placement without moving the view.
    inspector.toolbar.pan();inspector.canvas.draw()
    event('button_press_event',900,300);event('motion_notify_event',920,320);event('button_release_event',920,320)
    inspector.canvas.draw();panned=bounds();assert not np.allclose(panned,zoom)
    inspector.place_button.click();assert not inspector.toolbar.mode
    event('button_press_event',900,300);event('button_release_event',900,300);inspector.canvas.draw()
    assert np.allclose(bounds(),panned)
    inspector.stationary.setChecked(True);assert inspector.sweep_profile_button.isEnabled()
    inspector.grab().save(str(out/'inspector_zoomed.png'))

    # Home/history survives point redraws; Fit and new capture deliberately reset.
    inspector.toolbar.home();inspector.canvas.draw();assert np.allclose(bounds(),initial)
    inspector.ax.set_xlim(820,970);inspector.ax.set_ylim(450,150);inspector.canvas.draw()
    inspector.fit_button.click();inspector.canvas.draw();assert np.allclose(bounds(),initial)
    inspector.ax.set_xlim(820,970);inspector.ax.set_ylim(450,150)
    inspector.full_frame.setChecked(False);inspector.canvas.draw();assert not np.allclose(bounds(),[[820,970],[450,150]])
    inspector.full_frame.setChecked(True)
    inspector.ax.set_xlim(820,970);inspector.ax.set_ylim(450,150)
    inspector.capture();inspector.canvas.draw();assert np.allclose(bounds(),initial) and not inspector.manual
    for size in ((1280,720),(1920,1080)):
        inspector.resize(*size);process();assert inspector.size()==QSize(*size)
        for widget in (inspector.canvas,inspector.table,inspector.sweep_profile_button):inside(widget,inspector)
    inspector.close()
    print('PASS actual box zoom/pan, precise point clicks, row selection, numeric edits, inclusion and table scroll; Home/Fit/new-capture behavior; editor fits tested displays')
finally:
    window.session=None;window.close();app.processEvents()
print('All v0.7.2 checks passed. No physical hardware was operated.')
