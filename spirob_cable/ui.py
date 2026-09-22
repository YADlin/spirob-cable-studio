"""Qt widgets only. All serial access belongs to Worker."""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import queue
import signal
import sys
import time
from zipfile import ZipFile, ZIP_DEFLATED
from PySide6 import QtCore, QtGui, QtWidgets as Q
import pyqtgraph as pg
import pyqtgraph.exporters
from .config import RigSettings, lengths_from_modes, difference_bounds
from .model import Settings, Rejected, finite
from .rig import FRESH_SECONDS
from .worker import Worker
from .plots import Plots, COLORS
from .review import load_runs
from .analysis import plot_series, measured_circumference
from .theme import apply_theme

ROOT = Path(__file__).resolve().parent.parent


def label(text='', wrap=False):
    w=Q.QLabel(text); w.setWordWrap(wrap); return w


def button(text, fn, name=None):
    w=Q.QPushButton(text); w.clicked.connect(fn)
    if name: w.setObjectName(name)
    return w


def number(value=0, low=-70, high=70, decimals=3, suffix=''):
    w=Q.QDoubleSpinBox(); w.setRange(low,high); w.setDecimals(decimals); w.setSuffix(suffix)
    w.setValue(value); w.setKeyboardTracking(False); return w


def scroll(widget):
    w=Q.QScrollArea(); w.setWidgetResizable(True); w.setWidget(widget); return w


class Studio(Q.QMainWindow):
    def __init__(self, demo=False, config_path=ROOT/'config.json', log_root=ROOT/'logs'):
        apply_theme(Q.QApplication.instance())
        super().__init__(); self.demo=demo; self.config_path=Path(config_path); self.log_root=Path(log_root)
        self.worker=None; self.messages=queue.Queue(); self.pending=False; self.closing=False
        self.snapshot={}; self.active_config=None; self.session_path=None; self.last_draw=0
        self.review_runs=[]; self.notice_until=0
        try: cfg=RigSettings.load(self.config_path) if self.config_path.exists() else RigSettings()
        except Exception as exc: cfg=RigSettings(); self.load_error=str(exc)
        if demo: cfg=RigSettings.demo()
        self.cfg=cfg
        pg.setConfigOptions(foreground='#17364a',antialias=True)
        self.setWindowTitle('SpiRob Cable Studio 3.0'+(' — OFFLINE DEMO' if demo else ''))
        self.resize(1420,900); self.setMinimumSize(1060,720)
        self.build(); self.fill(cfg); self.refresh_controls()
        self.timer=QtCore.QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(50)
        self.escape=QtGui.QShortcut(QtGui.QKeySequence('Escape'),self); self.escape.activated.connect(lambda:self.command('stop'))
        if hasattr(self,'load_error'): self.notice(self.load_error)

    def build(self):
        root=Q.QWidget(); outer=Q.QVBoxLayout(root); outer.setContentsMargins(10,8,10,8); self.setCentralWidget(root)
        header=Q.QHBoxLayout()
        title=label('SPIROB / CABLE STUDIO'); title.setStyleSheet('font-size:19px;font-weight:700;')
        header.addWidget(title); header.addStretch()
        self.mode_label=label('OFFLINE DEMO' if self.demo else 'HARDWARE'); header.addWidget(self.mode_label)
        self.connect_button=button('Connect · torque off',self.connect_motor,'primary'); header.addWidget(self.connect_button)
        self.enable_rest_button=button('Enable at rest',self.enable_at_rest)
        self.enable_rest_button.setToolTip('Enable all selected motors, then record the present stationary configuration as rest using the entered physical rest lengths.')
        header.addWidget(self.enable_rest_button)
        self.release_all_button=button('Release all',lambda:self.release(None)); header.addWidget(self.release_all_button)
        self.stop_button=button('STOP ALL  Esc',lambda:self.command('stop'),'stop'); header.addWidget(self.stop_button)
        outer.addLayout(header)
        split=Q.QSplitter(QtCore.Qt.Orientation.Horizontal); split.setChildrenCollapsible(False); outer.addWidget(split,1)
        self.controls=Q.QTabWidget(); self.controls.setMinimumWidth(365); self.controls.setMaximumWidth(430)
        self.controls.addTab(self.build_manual(),'Jog')
        self.controls.addTab(self.build_pair(),'A / D')
        self.controls.addTab(self.build_setup(),'Setup')
        self.controls.addTab(self.build_experiments(),'Trials')
        self.controls.addTab(self.build_review_controls(),'Review')
        split.addWidget(self.controls)
        main=Q.QWidget(); layout=Q.QVBoxLayout(main); layout.setContentsMargins(12,0,0,0)
        self.readout=label('Connect to see live encoder displacement; no rest reference is required.',True)
        self.readout.setMinimumHeight(64); self.readout.setMaximumHeight(92)
        self.readout.setStyleSheet('background:white;border:1px solid #cfdae2;border-radius:5px;padding:8px;font-size:13px;')
        layout.addWidget(self.readout)
        toolbar=Q.QHBoxLayout()
        self.frame=Q.QComboBox(); self.frame.addItem('Since connection · mm','connection'); self.frame.addItem('From rest · mm','rest'); self.frame.addItem('Cable length · mm','absolute')
        self.frame.currentIndexChanged.connect(self.redraw); toolbar.addWidget(self.frame)
        self.freeze=Q.QCheckBox('Freeze view'); self.freeze.toggled.connect(lambda x:setattr(self.plots,'frozen',x)); toolbar.addWidget(self.freeze)
        self.error_check=Q.QCheckBox('Tracking error'); self.error_check.toggled.connect(lambda x:self.plots.error.setVisible(x)); toolbar.addWidget(self.error_check)
        toolbar.addStretch(); toolbar.addWidget(button('Auto range',lambda:self.plots.auto_range()))
        self.export_plot_button=button('Export plot',self.export_plot); toolbar.addWidget(self.export_plot_button)
        layout.addLayout(toolbar)
        self.views=Q.QStackedWidget()
        self.plots=Plots(); self.views.addWidget(self.plots)
        self.review_plot=pg.PlotWidget(background='white'); self.review_plot.addLegend(); self.review_plot.showGrid(x=True,y=True,alpha=.15)
        self.review_plot.setLabel('bottom','Session time',units='s'); self.review_plot.setLabel('left','Cable travel / length',units='mm')
        for name in ('left','bottom'): self.review_plot.getAxis(name).enableAutoSIPrefix(False)
        self.views.addWidget(self.review_plot); layout.addWidget(self.views,1)
        self.quality=label('Recording begins on connection. Setup displacement is visible immediately.',True); layout.addWidget(self.quality)
        self.notice_label=label('Choose IDs and calibration in Setup, then connect. Enable is sufficient for jogging.',True)
        self.notice_label.setMinimumHeight(35); layout.addWidget(self.notice_label)
        split.addWidget(main); split.setSizes([385,1035])
        self.activity=Q.QPlainTextEdit(); self.activity.setReadOnly(True); self.activity.setMaximumBlockCount(500)
        self.activity.setMaximumHeight(140)
        self.dock=Q.QDockWidget('Activity / diagnostics',self); self.dock.setWidget(self.activity)
        self.addDockWidget(QtCore.Qt.DockWidgetArea.BottomDockWidgetArea,self.dock); self.dock.hide()
        self.menuBar().addMenu('View').addAction(self.dock.toggleViewAction())

    def build_manual(self):
        page=Q.QWidget(); layout=Q.QVBoxLayout(page); self.manual=[]
        for i in range(2):
            box=Q.QGroupBox(f'Cable {i+1}'); g=Q.QGridLayout(box); w={'box':box}
            w['status']=label('Disconnected',True); g.addWidget(w['status'],0,0,1,2)
            w['enable']=button('Enable / hold here',lambda _=False,i=i:self.command('enable',{'indices':[i]}),'primary')
            w['release']=button('Release',lambda _=False,i=i:self.release(i))
            g.addWidget(w['enable'],1,0); g.addWidget(w['release'],1,1)
            w['step']=number(1,.01,2000,2,' mm'); w['speed']=number(5,.05,20,2,' mm/s')
            g.addWidget(label('Jog distance per click'),2,0); g.addWidget(label('Jog speed'),2,1)
            g.addWidget(w['step'],3,0); g.addWidget(w['speed'],3,1)
            w['minus']=button('− Take in',lambda _=False,i=i:self.jog(i,-1))
            w['plus']=button('+ Pay out',lambda _=False,i=i:self.jog(i,1))
            g.addWidget(w['minus'],4,0); g.addWidget(w['plus'],4,1)
            w['rest_length']=number(0,0,10000,2,' mm'); w['rest_length'].setSpecialValueText('Length unknown')
            w['rest_length'].setToolTip('Measured free cable length at the physical rest configuration. Required for absolute A/D control.')
            w['rest_length'].setPrefix('Rest L: ')
            w['rest']=button('Set rest here',lambda _=False,i=i:self.set_rest(i))
            g.addWidget(w['rest_length'],5,0); g.addWidget(w['rest'],5,1)
            w['target']=number(0,-70,70,2,' mm'); w['target_speed']=number(5,.05,20,2,' mm/s')
            g.addWidget(label('Target from rest'),6,0); g.addWidget(label('Target speed'),6,1)
            g.addWidget(w['target'],7,0); g.addWidget(w['target_speed'],7,1)
            w['move']=button('Move to target',lambda _=False,i=i:self.move_axis(i))
            w['home']=button('Return to rest',lambda _=False,i=i:self.move_axis(i,0))
            g.addWidget(w['move'],8,0); g.addWidget(w['home'],8,1)
            layout.addWidget(box); self.manual.append(w)
        layout.addWidget(label('Any numeric jog step is allowed within the displayed travel envelope. Each click is one finite move; wait for that cable to settle. The other cable can be jogged independently.',True))
        layout.addStretch(); return scroll(page)

    def build_pair(self):
        page=Q.QWidget(); layout=Q.QVBoxLayout(page)
        layout.addWidget(label('MEAN LENGTH + HALF-DIFFERENCE'))
        layout.addWidget(label('A = (L₁ + L₂) / 2\nD = (L₁ − L₂) / 2\nL₁ = A + D     L₂ = A − D',True))
        form=Q.QFormLayout()
        self.mean=number(.220,.001,10,4,' m'); self.mean.setSingleStep(.001)
        self.diff=number(0,-1000,1000,2,' mm'); self.diff.setSingleStep(1)
        self.pair_speed=number(5,.05,20,2,' mm/s')
        form.addRow('Mean A · metres',self.mean); form.addRow('Half-difference D',self.diff); form.addRow('Cable speed ceiling',self.pair_speed)
        layout.addLayout(form)
        self.pair_preview=label('',True); layout.addWidget(self.pair_preview)
        self.mean.valueChanged.connect(self.update_pair_preview); self.diff.valueChanged.connect(self.update_pair_preview)
        self.apply_mean=button('Apply A · keep current D',lambda:self.move_modes('mean'),'primary')
        self.apply_diff=button('Apply D · keep current A',lambda:self.move_modes('difference'),'primary')
        self.apply_both=button('Apply both entered values',lambda:self.move_modes('both'))
        self.use_current=button('Copy measured A and D into fields',self.copy_modes)
        for w in (self.apply_mean,self.apply_diff,self.apply_both,self.use_current): layout.addWidget(w)
        layout.addWidget(label('0.220 m = 220 mm. D is HALF the length difference: D = 10 mm means L₁ − L₂ = 20 mm.\n\nApply A preserves measured D; Apply D preserves measured A. Both motors must be stationary and referenced. Endpoints are checked together. Starts and feedback are sequential, not hardware synchronized.',True))
        layout.addStretch(); return scroll(page)

    def build_setup(self):
        page=Q.QWidget(); layout=Q.QVBoxLayout(page)
        self.axis_count=Q.QComboBox(); self.axis_count.addItems(['One cable','Two cables']); layout.addWidget(self.axis_count)
        self.import_button=button('Import previous config…',self.import_config); layout.addWidget(self.import_button)
        self.setup=[]
        for i in range(2):
            box=Q.QGroupBox(f'Cable {i+1} connection and geometry'); form=Q.QFormLayout(box); d={}
            d['port']=Q.QLineEdit('/dev/ttyUSB0'); d['device_id']=Q.QSpinBox(); d['device_id'].setRange(0,247); d['device_id'].setSpecialValueText('Enter ID')
            d['spool_diameter_mm']=number(27,.1,1000,3,' mm'); d['cable_diameter_mm']=number(.405,.001,20,3,' mm')
            d['diameter_basis']=Q.QComboBox()
            for name,value in [('Select diameter basis',None),('Bare core','core'),('Cable centreline','effective')]: d['diameter_basis'].addItem(name,value)
            d['payout_sign']=Q.QComboBox()
            for name,value in [('Select payout direction',None),('Counts + → payout',1),('Counts − → payout',-1)]: d['payout_sign'].addItem(name,value)
            d['calibrated_mm_per_rev']=number(0,0,1000,6); d['calibrated_mm_per_rev'].setSpecialValueText('Use geometry')
            d['setup_travel_mm']=number(70,.5,1000,1,' mm')
            for key,text in [('port','Serial port'),('device_id','Device ID'),('spool_diameter_mm','Spool diameter'),('cable_diameter_mm','Cable diameter'),('diameter_basis','Diameter basis'),('payout_sign','Payout direction'),('calibrated_mm_per_rev','Measured mm/rev'),('setup_travel_mm','Setup envelope ±')]: form.addRow(text,d[key])
            layout.addWidget(box); self.setup.append(d)
        layout.addWidget(label('100:1 gearbox · 133600 counts per spool revolution. Setup envelope is centred on the encoder position first read on connection; after Set rest, the ±70 mm rest window applies. Releasing/re-enabling does not reset the connection origin.',True))
        self.save_button=button('Save configuration',self.save_config,'primary'); layout.addWidget(self.save_button)
        cal=Q.QGroupBox('Measured-travel calibration'); form=Q.QFormLayout(cal)
        self.cal_axis=Q.QComboBox(); self.cal_axis.addItems(['Cable 1','Cable 2'])
        self.count_a=Q.QLineEdit(); self.count_b=Q.QLineEdit(); self.travel=number(0,0,1000,3,' mm')
        form.addRow('Cable',self.cal_axis); form.addRow('Start count A',self.count_a); form.addRow('End count B',self.count_b)
        self.capture_a=button('Capture A',lambda:self.capture(self.count_a)); self.capture_b=button('Capture B',lambda:self.capture(self.count_b))
        row=Q.QHBoxLayout(); row.addWidget(self.capture_a); row.addWidget(self.capture_b); form.addRow(row)
        form.addRow('Measured travel',self.travel); self.cal_result=label('',True); form.addRow(self.cal_result)
        form.addRow(button('Calculate mm/rev',self.calculate_calibration))
        self.apply_cal_button=button('Use result after disconnect',self.apply_calibration); form.addRow(self.apply_cal_button)
        self.calculated=None; layout.addWidget(cal)
        self.run_name=Q.QLineEdit('spirob-two-cable'); layout.addWidget(label('Run name')); layout.addWidget(self.run_name)
        layout.addStretch(); return scroll(page)

    def build_experiments(self):
        page=Q.QWidget(); layout=Q.QVBoxLayout(page)
        self.trial_kind=Q.QComboBox(); self.trial_kind.addItems(['Mean / difference','Cable 1 from rest','Cable 2 from rest'])
        self.trial_kind.currentIndexChanged.connect(self.reset_trial); layout.addWidget(self.trial_kind)
        self.trial_table=Q.QTableWidget(0,3); self.trial_table.horizontalHeader().setSectionResizeMode(Q.QHeaderView.ResizeMode.Stretch)
        self.trial_table.setMinimumHeight(220); layout.addWidget(self.trial_table)
        row=Q.QHBoxLayout(); self.add_row_button=button('Add',self.add_trial_row); row.addWidget(self.add_row_button)
        self.remove_row_button=button('Remove',self.remove_trial_row); row.addWidget(self.remove_row_button); layout.addLayout(row)
        form=Q.QFormLayout(); self.trial_speed=number(5,.05,20,2,' mm/s'); self.repeats=Q.QSpinBox(); self.repeats.setRange(1,20)
        form.addRow('Cable speed',self.trial_speed); form.addRow('Repetitions',self.repeats); layout.addLayout(form)
        self.start_trial_button=button('Start move–settle–hold trial',self.start_trial,'primary'); layout.addWidget(self.start_trial_button)
        layout.addWidget(button('Stop / abort',lambda:self.command('stop')))
        self.trial_status=label('All rows are validated before any movement.',True); layout.addWidget(self.trial_status)
        self.load_trial_button=button('Load procedure…',self.load_trial); self.save_trial_button=button('Save procedure…',self.save_trial)
        layout.addWidget(self.load_trial_button); layout.addWidget(self.save_trial_button)
        self.note=Q.QLineEdit(); self.note.setMaxLength(1000); self.note.setPlaceholderText('Observation / trial note')
        layout.addWidget(self.note); self.note_button=button('Record observation',self.annotate); layout.addWidget(self.note_button)
        layout.addWidget(label('Hold begins after all selected cables settle. Stop discards remaining steps. No automatic resume or return-to-rest is implied.',True))
        self.reset_trial(); layout.addStretch(); return scroll(page)

    def build_review_controls(self):
        page=Q.QWidget(); layout=Q.QVBoxLayout(page)
        self.open_run_button=button('Add recorded session…',self.open_run); layout.addWidget(self.open_run_button)
        self.clear_runs_button=button('Clear comparison',self.clear_runs); layout.addWidget(self.clear_runs_button)
        self.live_button=button('Back to live plot',lambda:self.views.setCurrentIndex(0)); layout.addWidget(self.live_button)
        self.export_run_button=button('Export current session ZIP…',self.export_run); layout.addWidget(self.export_run_button)
        self.review_summary=label('Open samples.csv. Each cable retains its own acquisition timestamps. Legacy single-cable logs are also supported.',True); layout.addWidget(self.review_summary)
        self.demo_fault_buttons=[]
        if self.demo:
            for i in range(2):
                b=button(f'Demo: lose cable {i+1} feedback',lambda _=False,i=i:self.command('demo_fault',{'axis':i}))
                self.demo_fault_buttons.append(b); layout.addWidget(b)
        layout.addStretch(); return scroll(page)

    def fill(self,cfg):
        self.cfg=cfg; self.axis_count.setCurrentIndex(cfg.active_axes-1)
        for i,a in enumerate(cfg.axes):
            for key,w in self.setup[i].items():
                value=getattr(a,key)
                if isinstance(w,Q.QComboBox): w.setCurrentIndex(max(0,w.findData(value)))
                elif isinstance(w,Q.QLineEdit): w.setText(value)
                else: w.setValue(value or 0)
            self.manual[i]['speed'].setValue(a.jog_speed)
            self.manual[i]['target_speed'].setValue(a.speed_mm_s)
            self.manual[i]['rest_length'].setValue(a.rest_length_mm or 0)
        self.refresh_controls()

    def settings(self):
        axes=[]
        for i,d in enumerate(self.setup):
            values={}
            for key,w in d.items():
                values[key]=w.currentData() if isinstance(w,Q.QComboBox) else w.text().strip() if isinstance(w,Q.QLineEdit) else w.value()
            values['device_id']=values['device_id'] or None
            values['calibrated_mm_per_rev']=values['calibrated_mm_per_rev'] or None
            values.update(speed_mm_s=self.manual[i]['target_speed'].value(),jog_speed_mm_s=self.manual[i]['speed'].value(),rest_length_mm=self.manual[i]['rest_length'].value() or None)
            axes.append(replace(self.cfg.axes[i],**values))
        cfg=RigSettings(tuple(axes),self.axis_count.currentIndex()+1); cfg.validate(); return cfg

    def save_config(self):
        try:
            cfg=self.settings()
            if self.demo: self.notice('Demo choices are session-only. Hardware configuration was not changed.'); return
            cfg.save(self.config_path); self.cfg=cfg; self.notice('Configuration saved, including both speed defaults.')
        except Exception as exc: self.notice(str(exc))

    def import_config(self):
        path,_=Q.QFileDialog.getOpenFileName(self,'Import working configuration',str(ROOT.parent),'JSON (*.json)')
        if path:
            try:
                cfg=RigSettings.load(path); self.fill(cfg)
                if not self.demo: cfg.save(self.config_path)
                self.notice('Imported. A single-cable file selects one cable; choose Two cables and enter the second ID to expand.')
            except Exception as exc: self.notice(str(exc))

    def confirm(self,title,text):
        return Q.QMessageBox.question(self,title,text)==Q.QMessageBox.StandardButton.Yes

    def connect_motor(self):
        if self.worker:
            if any(a.get('enabled') for a in self.snapshot.get('axes',[])) and not self.confirm('Disconnect','Disconnect releases both motors. Support the mechanism. Continue?'): return
            self.command('close'); return
        try:
            cfg=self.settings()
            if not self.demo: cfg.save(self.config_path)
            self.cfg=self.active_config=cfg; self.snapshot={}; self.plots.clear(); self.views.setCurrentIndex(0)
            self.worker=Worker(cfg,self.demo,self.log_root,self.messages,self.run_name.text()); self.worker.start()
            self.notice('Connecting selected IDs. Torque will be disabled; plots begin with the first feedback.')
        except Exception as exc: self.notice(str(exc))
        self.refresh_controls()

    def command(self,name,data=None):
        if not self.worker: return
        try:
            if self.pending and name not in ('stop','release_all','close'): raise Rejected('Wait for the pending command')
            self.worker.submit(name,data); self.pending=name not in ('stop','release_all')
            self.notice_until=0
        except Exception as exc: self.notice(str(exc))
        self.refresh_controls()

    def enable_at_rest(self):
        indices=list(range(self.active_config.active_axes))
        lengths={i:self.manual[i]['rest_length'].value() for i in indices}
        if any(x<=70 for x in lengths.values()): self.notice('Enter each measured physical rest length (>70 mm) in the Jog tab first.'); return
        if self.confirm('Enable at physical rest','Is the mechanism currently at its intended rest configuration, with the entered physical cable lengths? This enables holding and records rest after stable feedback.'):
            self.command('enable',{'indices':indices,'reference_lengths':lengths})

    def set_rest(self,i):
        length=self.manual[i]['rest_length'].value() or None
        if self.confirm('Set rest here',f'Record the stationary position of cable {i+1} as physical rest? This re-centres its ±70 mm window. Entered physical length: {length if length is not None else "unknown"} mm.'):
            self.command('rest',{'axis':i,'length_mm':length})

    def release(self,i):
        if self.confirm('Release torque','Support the mechanism. Release removes holding torque and clears the selected rest reference.'):
            self.command('release_all' if i is None else 'release',None if i is None else {'axis':i})

    def jog(self,i,sign):
        w=self.manual[i]; self.command('jog',{'axis':i,'delta_mm':sign*w['step'].value(),'speed_mm_s':w['speed'].value()})

    def move_axis(self,i,value=None):
        w=self.manual[i]; self.command('move',{'axis':i,'value_mm':w['target'].value() if value is None else value,'speed_mm_s':w['target_speed'].value()})

    def move_modes(self,mode):
        self.command('modes',{'mode':mode,'mean_mm':self.mean.value()*1000,'difference_mm':self.diff.value(),'speed_mm_s':self.pair_speed.value()})

    def copy_modes(self):
        c=self.snapshot.get('coordinates')
        if c: self.mean.setValue(c['mean_mm']/1000); self.diff.setValue(c['half_difference_mm'])

    def update_pair_preview(self,*_):
        if not hasattr(self,'pair_preview'): return
        a,d=self.mean.value()*1000,self.diff.value(); l1,l2=lengths_from_modes(a,d)
        text=f'Apply both → L₁ {l1:.2f} mm · L₂ {l2:.2f} mm'
        axes=self.snapshot.get('axes',[])
        if len(axes)==2 and all(x.get('rest_length_mm') is not None for x in axes):
            low,high=difference_bounds(a,*[x['rest_length_mm'] for x in axes])
            text+=f'\nFor this A, allowed D: {low:.2f} to {high:.2f} mm.' if low<=high else '\nThis mean is outside the joint travel window.'
            if not low<=d<=high: text+='\nEntered pair would be rejected; neither target is sent.'
        else: text+='\nSet both physical rest lengths to enable A/D control.'
        self.pair_preview.setText(text)

    def capture(self,w):
        i=self.cal_axis.currentIndex(); axes=self.snapshot.get('axes',[])
        if i<len(axes) and axes[i]['count'] is not None:
            w.setText(str(axes[i]['count'])); self.command('note',{'text':f'Calibration cable {i+1}: count {axes[i]["count"]}'})

    def calculate_calibration(self):
        try:
            self.calculated=(self.cal_axis.currentIndex(),measured_circumference(self.count_a.text(),self.count_b.text(),self.travel.value()))
            self.cal_result.setText(f'Cable {self.calculated[0]+1}: {self.calculated[1]:.6f} mm/rev. Compare payout/take-in before applying.')
        except Exception as exc: self.calculated=None; self.cal_result.setText(str(exc))
        self.refresh_controls()

    def apply_calibration(self):
        if not self.worker and self.calculated:
            i,value=self.calculated; self.setup[i]['calibrated_mm_per_rev'].setValue(value)
            self.notice('Calibration entered. Save configuration, reconnect and re-reference.')

    def reset_trial(self):
        if not hasattr(self,'trial_table'): return
        pair=self.trial_kind.currentIndex()==0
        self.trial_table.setColumnCount(3 if pair else 2)
        self.trial_table.setHorizontalHeaderLabels(['A (m)','D (mm)','Hold (s)'] if pair else ['From rest (mm)','Hold (s)'])
        self.trial_table.setRowCount(0)
        for value in (0,1,0): self.add_trial_row(False,[.220,value,1] if pair else [value,1])

    def add_trial_row(self,_=False,values=None):
        if self.trial_table.rowCount()>=100: return
        pair=self.trial_kind.currentIndex()==0
        values=values or ([.220,0,1] if pair else [0,1])
        row=self.trial_table.rowCount(); self.trial_table.insertRow(row)
        for j,val in enumerate(values):
            last=j==len(values)-1
            widget=number(val,0 if last else .001 if pair and j==0 else -1000,120 if last else 10 if pair and j==0 else 1000,4 if pair and j==0 else 2)
            self.trial_table.setCellWidget(row,j,widget)
        self.trial_table.setRowHeight(row,38)

    def remove_trial_row(self):
        if self.trial_table.currentRow()>=0: self.trial_table.removeRow(self.trial_table.currentRow())

    def trial_data(self):
        pair=self.trial_kind.currentIndex()==0
        rows=[[self.trial_table.cellWidget(i,j).value() for j in range(self.trial_table.columnCount())] for i in range(self.trial_table.rowCount())]
        if pair:
            for row in rows: row[0]*=1000
        return {'kind':['pair','cable1','cable2'][self.trial_kind.currentIndex()], 'rows':rows,
                'repeats':self.repeats.value(),'speed_mm_s':self.trial_speed.value()}

    def start_trial(self): self.command('experiment',self.trial_data())

    def save_trial(self):
        path,_=Q.QFileDialog.getSaveFileName(self,'Save procedure',str(ROOT/'procedure.json'),'JSON (*.json)')
        if path:
            try: Path(path).write_text(json.dumps(self.trial_data(),indent=2)+'\n')
            except Exception as exc: self.notice(str(exc))

    def load_trial(self):
        path,_=Q.QFileDialog.getOpenFileName(self,'Load procedure',str(ROOT),'JSON (*.json)')
        if path:
            try:
                data=json.loads(Path(path).read_text()); kind=['pair','cable1','cable2'].index(data['kind'])
                if not isinstance(data['rows'],list) or not 1<=len(data['rows'])<=100: raise Rejected('Invalid row count')
                # Validate all table values before replacing the editor contents.
                expected=3 if kind==0 else 2
                for row in data['rows']:
                    if len(row)!=expected: raise Rejected('Incorrect row shape')
                    for value in row: finite(value,'Procedure value')
                    if not 0<=row[-1]<=120: raise Rejected('Invalid hold time')
                    if kind==0 and not 1<=row[0]<=10000: raise Rejected('A is stored in mm; invalid mean')
                    if not -1000<=row[-2]<=1000: raise Rejected('Invalid target/difference')
                if type(data['repeats']) is not int or not 1<=data['repeats']<=20 or len(data['rows'])*data['repeats']>200: raise Rejected('Invalid repeat count')
                if not .05<=finite(data['speed_mm_s'],'Speed')<=20: raise Rejected('Invalid speed')
                self.trial_kind.setCurrentIndex(kind); self.trial_table.setRowCount(0)
                for row in data['rows']: self.add_trial_row(False,[row[0]/1000,*row[1:]] if kind==0 else row)
                self.repeats.setValue(data['repeats']); self.trial_speed.setValue(data['speed_mm_s'])
                self.notice('Procedure loaded for review. No motor commands sent.')
            except Exception as exc: self.notice(str(exc))

    def annotate(self):
        self.command('note',{'text':self.note.text()}); self.note.clear()

    def open_run(self):
        path,_=Q.QFileDialog.getOpenFileName(self,'Open recorded samples',str(self.log_root),'CSV (*.csv)')
        if path:
            try: self.load_review(path)
            except Exception as exc: self.notice(str(exc))

    def load_review(self,path):
        if any(a.get('enabled') for a in self.snapshot.get('axes',[])): raise Rejected('Release before offline review')
        if len(self.review_runs)>=4: raise Rejected('Compare up to four sessions; clear first')
        self.review_runs.append(load_runs(path,self.frame.currentData())); self.draw_review()

    def draw_review(self):
        self.review_plot.clear(); self.review_plot.plotItem.legend.clear(); summary=[]
        for n,runs in enumerate(self.review_runs):
            for k,(name,run) in enumerate(runs.items()):
                t,a,target=plot_series(run); color=pg.intColor(n*2+k,hues=8)
                mode='DEMO' if run.metadata.get('demo') else 'HW' if run.metadata.get('demo') is False else 'UNKNOWN'
                self.review_plot.plot(t,a,pen=pg.mkPen(color,width=2),name=f'{n+1} {name} [{mode}]',connect='finite')
                self.review_plot.plot(t,target,pen=pg.mkPen(color,style=QtCore.Qt.PenStyle.DashLine),connect='finite')
                m=run.metrics(); rmse=f'{m["rmse_mm"]:.4f}' if m['rmse_mm'] is not None else '—'
                summary.append(f'{n+1} {name}: {m["samples"]} samples · RMSE {rmse} mm')
        self.review_summary.setText('\n'.join(summary)+'\nRMSE includes moving transients, not just endpoint error.'); self.views.setCurrentIndex(1)

    def clear_runs(self): self.review_runs.clear(); self.draw_review()

    def export_plot(self):
        path,_=Q.QFileDialog.getSaveFileName(self,'Export displayed plot',str(ROOT/'plot.png'),'PNG (*.png)')
        if path:
            try: pg.exporters.ImageExporter((self.plots.graph if self.views.currentIndex()==0 else self.review_plot).plotItem).export(path)
            except Exception as exc: self.notice(str(exc))

    def export_run(self):
        if not self.session_path: return
        path,_=Q.QFileDialog.getSaveFileName(self,'Export session',str(ROOT/'session.zip'),'ZIP (*.zip)')
        if path:
            try:
                with ZipFile(path,'w',ZIP_DEFLATED) as z:
                    for name in ('session.json','samples.csv','pairs.csv','events.jsonl'):
                        p=self.session_path/name
                        if p.exists(): z.write(p,name)
                self.notice('Session exported with raw samples, pair timing, events and configuration.')
            except Exception as exc: self.notice(str(exc))

    def notice(self,text):
        self.notice_label.setText(text); self.notice_until=time.monotonic()+7
        self.activity.appendPlainText(time.strftime('%H:%M:%S')+'  '+text)

    def redraw(self,*_):
        if hasattr(self,'plots'): self.plots.draw(self.frame.currentData())

    def refresh_controls(self):
        if not hasattr(self,'setup'): return
        connected=self.worker is not None; axes=self.snapshot.get('axes',[])
        count=self.active_config.active_axes if connected and self.active_config else self.axis_count.currentIndex()+1
        fault=self.snapshot.get('faulted',False); running=self.snapshot.get('procedure',{}).get('active',False)
        pending_ref=self.snapshot.get('pending_reference',False)
        free=not self.pending and not self.closing and not fault and not running and not pending_ref
        enabled=any(a.get('enabled') for a in axes)
        fresh=[a.get('last_read') is not None and time.monotonic()-a['last_read']<FRESH_SECONDS for a in axes]
        for i,w in enumerate(self.manual):
            w['box'].setVisible(i<count)
            a=axes[i] if i<len(axes) else {}; idle=free and i<len(fresh) and fresh[i] and a.get('state')=='HOLDING'
            disabled=free and i<len(fresh) and fresh[i] and a.get('state')=='DISABLED'
            w['enable'].setEnabled(disabled)
            w['enable'].setText('Motor enabled' if a.get('enabled') else 'Enable / hold here')
            w['release'].setEnabled(connected and not self.closing and not running and not self.pending)
            for key in ('step','speed','target_speed','rest_length'): w[key].setEnabled(not connected or idle or disabled)
            for key in ('plus','minus','rest'): w[key].setEnabled(idle)
            for key in ('move','home','target'): w[key].setEnabled(idle and a.get('rest_count') is not None)
            if a:
                cfg=self.active_config.axes[i]; rest=a.get('rest_count') is not None
                coordinate = f'From rest: {a["offset_mm"]:+.3f} mm / ±70 mm' if rest else f'Setup: {a["connection_mm"]:+.3f} mm / ±{cfg.setup_travel_mm:g} from connection' if a.get('connection_mm') is not None else 'No valid encoder reading'
                try:
                    rpm=cfg.rpm_for_speed(w['speed'].value())
                    rate=f'Jog: {rpm} motor RPM / {rpm/100:.2f} output RPM'
                except Rejected as exc:
                    rate=str(exc)
                w['status'].setText(f'ID {cfg.device_id} · {a["state"]}\n'+coordinate+'\n'+rate)
            else: w['status'].setText('Disconnected. Connect, then Enable to jog.')
        self.connect_button.setText('Disconnect' if connected else 'Connect · torque off'); self.connect_button.setEnabled(not self.closing)
        self.enable_rest_button.setEnabled(free and len(axes)==count and all(fresh) and all(a['state']=='DISABLED' for a in axes))
        self.stop_button.setEnabled(connected and not self.closing); self.release_all_button.setEnabled(connected and not self.closing)
        both_idle=free and len(axes)==2 and all(fresh) and all(a['state']=='HOLDING' for a in axes) and self.snapshot.get('coordinates') is not None
        for w in (self.apply_mean,self.apply_diff,self.apply_both,self.use_current): w.setEnabled(bool(both_idle))
        for w in (self.mean,self.diff,self.pair_speed): w.setEnabled(not connected or bool(both_idle))
        self.axis_count.setEnabled(not connected); self.import_button.setEnabled(not connected)
        for d in self.setup:
            for w in d.values(): w.setEnabled(not connected)
        self.save_button.setEnabled(not connected); self.run_name.setEnabled(not connected)
        self.apply_cal_button.setEnabled(not connected and self.calculated is not None)
        cal_i=self.cal_axis.currentIndex(); cal_ok=free and cal_i<len(axes) and axes[cal_i]['state']=='HOLDING'
        self.capture_a.setEnabled(cal_ok); self.capture_b.setEnabled(cal_ok)
        offline=not enabled and not self.pending and not self.closing
        for w in (self.open_run_button,self.clear_runs_button,self.export_plot_button,self.load_trial_button,self.save_trial_button): w.setEnabled(offline)
        self.export_run_button.setEnabled(not connected and self.session_path is not None)
        for w in (self.trial_kind,self.trial_table,self.add_row_button,self.remove_row_button,self.trial_speed,self.repeats): w.setEnabled(not running and not self.pending)
        kind=self.trial_kind.currentIndex(); trial_ok=bool(both_idle) if kind==0 else free and kind-1<len(axes) and kind>0 and axes[kind-1]['state']=='HOLDING' and axes[kind-1]['rest_count'] is not None
        self.start_trial_button.setEnabled(bool(trial_ok)); self.note_button.setEnabled(connected and not self.pending and not fault)
        for i,b in enumerate(self.demo_fault_buttons): b.setEnabled(connected and i<len(axes) and free)
        self.update_pair_preview()

    def refresh(self):
        if self.worker: self.worker.heartbeat=time.monotonic()
        for _ in range(300):
            try: item=self.messages.get_nowait()
            except queue.Empty: break
            kind=item['kind']
            if kind=='sample': self.plots.append(item['data'])
            elif kind=='snapshot':
                self.snapshot=item; self.session_path=Path(item['log_path'])
            elif kind=='event':
                d=item['data']
                if d['event']!='write_requested': self.activity.appendPlainText(f'{d["t_s"]:.3f}s '+json.dumps({k:v for k,v in d.items() if k!='t_s'}))
            elif kind=='command_done': self.pending=False
            elif kind=='rejected': self.pending=False; self.notice(item['text'])
            elif kind=='closed':
                self.worker=None; self.pending=False; self.snapshot={}
                if not all(item['disable_confirmed']): self.notice('One or more disables UNCONFIRMED. Use the independent power disconnect.')
                else: self.notice('Disconnected. Plot retained for inspection; rest references cleared.')
        axes=self.snapshot.get('axes',[]); lines=[]
        for i,a in enumerate(axes):
            age=time.monotonic()-a['last_read'] if a.get('last_read') else math.inf
            if age>FRESH_SECONDS or self.snapshot.get('faulted'): lines.append(f'Cable {i+1}: FEEDBACK INVALID'); continue
            absolute=f'{a["absolute_mm"]:.3f} mm length' if a['absolute_mm'] is not None else 'physical length not referenced'
            lines.append(f'Cable {i+1}  {a["state"]}  ·  Δconnection {a["connection_mm"]:+.3f} mm  ·  {absolute}  ·  count {a["count"]}')
        c=self.snapshot.get('coordinates')
        if c: lines.append(f'Mean A {c["mean_mm"]/1000:.4f} m   |   Half-difference D {c["half_difference_mm"]:+.3f} mm   |   sequential-read skew {c["skew_s"]*1000:.0f} ms')
        if lines: self.readout.setText('\n'.join(lines))
        elif not self.worker: self.readout.setText('Disconnected · displayed traces are historical. On connection, displacement traces appear before enabling or setting rest.')
        if self.snapshot.get('faulted'):
            confirmed=[a['disable_confirmed'] for a in axes]
            self.notice_label.setText('FAULT: '+self.snapshot['fault_reason']+(' | DISABLE UNCONFIRMED — use power disconnect' if not all(confirmed) else ' | Both disables acknowledged'))
        elif time.monotonic()>self.notice_until:
            self.notice_label.setText('Rest capture pending: waiting for stable feedback.' if self.snapshot.get('pending_reference') else 'Jog needs Enable only. Set rest for referenced targets; physical lengths enable mean/difference control.')
        p=self.snapshot.get('procedure',{})
        if p: self.trial_status.setText(f'{p["phase"]} · step {p["step"]} of {p["total"]}')
        timing=[]
        for i,rows in enumerate(self.plots.records):
            if len(rows)>1:
                recent=list(rows)[-20:]; dt=recent[-1]['read_end_s']-recent[0]['read_end_s']
                if dt>0: timing.append(f'C{i+1}: {(len(recent)-1)/dt:.1f} samples/s')
        self.quality.setText(' · '.join(timing)+(' · Recording: '+str(self.session_path.name) if self.worker and self.session_path else ' · Historical display')+' · Host timestamps, sequential feedback')
        if time.monotonic()-self.last_draw>.1:
            self.redraw(); self.last_draw=time.monotonic()
        self.refresh_controls()
        if self.closing and not self.worker: self.timer.stop(); self.close()

    def closeEvent(self,event):
        if self.worker:
            if not self.closing and any(a.get('enabled') for a in self.snapshot.get('axes',[])) and not self.confirm('Quit','Quit releases both motors and clears references. Continue?'):
                event.ignore(); return
            self.closing=True; self.command('close'); event.ignore()
        else: self.timer.stop(); event.accept()


def main():
    parser=argparse.ArgumentParser(description='SpiRob one/two cable research workstation')
    parser.add_argument('--demo',action='store_true'); parser.add_argument('--config',type=Path,default=ROOT/'config.json')
    parser.add_argument('--import-config',type=Path); args=parser.parse_args()
    if args.import_config:
        if args.demo: parser.error('Import hardware settings separately from demo')
        cfg=RigSettings.load(args.import_config); cfg.save(args.config)
        print(f'Imported {cfg.active_axes} active cable(s). IDs: {[a.device_id for a in cfg.axes]}. No motor commands sent.')
    app=Q.QApplication(sys.argv[:1])
    window=Studio(args.demo,args.config); window.show()
    signal.signal(signal.SIGINT,lambda *_:window.close()); signal.signal(signal.SIGTERM,lambda *_:window.close())
    return app.exec()
