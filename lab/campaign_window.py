"""Campaign planning, acquisition progress, offline batch review and fit results."""
import copy
import json
import queue
import threading
import time
from pathlib import Path
import numpy as np
from PySide6.QtCore import Qt,QTimer,QUrl
from PySide6.QtGui import QDesktopServices,QPixmap,QIcon
from PySide6.QtWidgets import (QDialog,QWidget,QVBoxLayout,QHBoxLayout,QLabel,QPushButton,QTabWidget,
    QTableWidget,QTableWidgetItem,QHeaderView,QComboBox,QDoubleSpinBox,QSpinBox,QFileDialog,QCheckBox,QApplication)
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg,NavigationToolbar2QT
from campaign import MODES,default_plan,expand_plan,CampaignRunner
from campaign_storage import CampaignStore,atomic,encoded
from campaign_analysis import auto_measure,fit_campaign,add_per_pose,CampaignCancelled
from campaign_surface import fit_surface,surface_is_current,render_overlays,FORMULA as SURFACE_FORMULA


class CampaignWindow(QDialog):
    def __init__(self,host):
        super().__init__(host);self.host=host;self.store=None;self.runner=None;self.inspector=None
        self.quiescing=False;self.approved_review=None;self.simulation_dialog=None
        self.messages=queue.Queue();self.worker=None;self.capture_worker=None;self.cancel=threading.Event();self.busy=False
        self.setWindowTitle('SpiRob — campaigns');self.setMinimumSize(960,620)
        screen=QApplication.primaryScreen().availableGeometry();self.resize(min(1160,max(960,screen.width()-48)),min(730,max(620,screen.height()-64)))
        root=QVBoxLayout(self);self.status=QLabel('Draft or generate a plan → simulate → approve → Connect / enable at rest → Start campaign.')
        self.status.setWordWrap(True);self.status.setFixedHeight(50);root.addWidget(self.status)
        bar=QHBoxLayout();root.addLayout(bar)
        self.start_button=QPushButton('Start approved campaign');self.start_button.clicked.connect(self.start);bar.addWidget(self.start_button)
        self.stop_button=QPushButton('STOP — hold both');self.stop_button.clicked.connect(lambda:self.host.command('hold'));bar.addWidget(self.stop_button)
        self.open_button=QPushButton('Open captured campaign…');self.open_button.clicked.connect(self.open);bar.addWidget(self.open_button)
        self.folder_button=QPushButton('Open data folder');self.folder_button.clicked.connect(self.open_folder);bar.addWidget(self.folder_button)
        self.tabs=QTabWidget();root.addWidget(self.tabs,1)
        plan_page=QWidget();pl=QVBoxLayout(plan_page);self.tabs.addTab(plan_page,'Plan / preflight')
        self.plan_page=plan_page
        row=QHBoxLayout();pl.addLayout(row)
        self.mode=QComboBox();self.mode.addItems(['Automatic shared k₂ after journey','Collect now; place joints afterward']);row.addWidget(self.mode,1)
        self.dwell=self.number(2,0,30,1);row.addWidget(QLabel('Extra dwell [s]'));row.addWidget(self.dwell)
        cap=self.host.motion_profile['max_speed_mm_s']
        self.return_speed=self.number(min(1,cap),.05,cap,3);row.addWidget(QLabel('Return [mm/s]'));row.addWidget(self.return_speed)
        self.preflight_button=QPushButton('Preflight');self.preflight_button.clicked.connect(self.preflight);row.addWidget(self.preflight_button)
        note=QLabel('Difference: x = full D, y = mean. Both: x = L₁, y = L₂. Cable 1/2 changes only its own length. Speeds in mm/s.\nIntervals >1: capture each interpolated step from the preceding row. First row is one capture. A final return to starting lengths is always added; STOP/fault cancels it.')
        note.setWordWrap(True);pl.addWidget(note)
        self.table=QTableWidget(0,7);self.table.setHorizontalHeaderLabels(['Motion','x [mm]','y [mm]','Speed 1','Speed 2','Intervals','Label'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch);pl.addWidget(self.table,1)
        buttons=QHBoxLayout();pl.addLayout(buttons)
        for title,handler in [('Difference sweep…',self.generate_sweep),('Mixed / random…',self.generate_random),('Broad 73 poses',self.defaults),('Simulate / review',self.simulate_plan)]:
            b=QPushButton(title);b.clicked.connect(handler);buttons.addWidget(b)
        buttons=QHBoxLayout();pl.addLayout(buttons)
        for title,handler in [('Add row',self.add_row),('Remove row',self.remove_row),('Load plan…',self.load_plan),('Save plan…',self.save_plan)]:
            b=QPushButton(title);b.clicked.connect(handler);buttons.addWidget(b)
        self.preflight_text=QLabel('Preflight expands every row and checks the session motion envelope, storage budget and remaining time.');self.preflight_text.setWordWrap(True);self.preflight_text.setFixedHeight(65);pl.addWidget(self.preflight_text)
        poses_page=QWidget();pv=QVBoxLayout(poses_page);self.tabs.addTab(poses_page,'Captured poses / batch review')
        controls=QHBoxLayout();pv.addLayout(controls)
        self.edit_button=QPushButton('Place / review joints');self.edit_button.clicked.connect(self.edit_selected);controls.addWidget(self.edit_button)
        self.next_button=QPushButton('Review next unfinished');self.next_button.clicked.connect(self.next_unfinished);controls.addWidget(self.next_button)
        self.exclude_button=QPushButton('Exclude / include selected');self.exclude_button.clicked.connect(self.exclude);controls.addWidget(self.exclude_button)
        self.dots_button=QPushButton('Measure marker dots');self.dots_button.clicked.connect(self.measure_dots);controls.addWidget(self.dots_button)
        self.dots_replace=QCheckBox('Replace my points');self.dots_replace.setToolTip('Off: poses with manual points keep them; dots are saved beside them (dots.json) and compared in dots_summary.csv.');controls.addWidget(self.dots_replace)
        self.poses=QTableWidget(0,5);self.poses.setHorizontalHeaderLabels(['Pose / image','Label','State','L₁, L₂ targets','Details'])
        self.poses.setSelectionBehavior(QTableWidget.SelectRows);self.poses.setSelectionMode(QTableWidget.SingleSelection)
        self.poses.setEditTriggers(QTableWidget.NoEditTriggers);self.poses.setIconSize(__import__('PySide6.QtCore',fromlist=['QSize']).QSize(100,56))
        self.poses.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch);pv.addWidget(self.poses,1)
        self.poses.cellDoubleClicked.connect(lambda *_:self.edit_selected())
        review_note=QLabel('Frames and drafts are retained in this run. Save & next opens the next unfinished pose. The final return image is excluded from fitting by default; include it here if useful.\nPrevious-image optical-flow suggestions are drafts for your review; they never use MuJoCo as the physical measurement.')
        review_note.setWordWrap(True);pv.addWidget(review_note)
        result_page=QWidget();rv=QVBoxLayout(result_page);self.tabs.addTab(result_page,'Shared k₂ → per-pose k₂')
        knobs=QHBoxLayout();rv.addLayout(knobs)
        self.low=self.number(.01,.001,10,4);self.high=self.number(4,.001,10,4);self.count=QSpinBox();self.count.setRange(3,101);self.count.setValue(31)
        for title,widget in [('k₂ min',self.low),('max',self.high),('samples',self.count)]:knobs.addWidget(QLabel(title));knobs.addWidget(widget)
        self.fit_button=QPushButton('1. Fit shared k₂');self.fit_button.clicked.connect(lambda:self.analyse(False));knobs.addWidget(self.fit_button)
        self.individual_button=QPushButton('2. Fit each pose');self.individual_button.clicked.connect(self.individual);knobs.addWidget(self.individual_button)
        self.cancel_button=QPushButton('Cancel fit');self.cancel_button.clicked.connect(self.cancel.set);knobs.addWidget(self.cancel_button)
        self.result_text=QLabel('One common k₂ is evaluated across all included poses before separate k₂ values are reported.');self.result_text.setWordWrap(True);self.result_text.setFixedHeight(70);rv.addWidget(self.result_text)
        plotbar=QHBoxLayout();rv.addLayout(plotbar);plotbar.addWidget(QLabel('Per-pose horizontal axis:'))
        self.xaxis=QComboBox();self.xaxis.addItems(['Measured cable difference [mm]','Measured distal lateral offset [mm]','Pose order']);self.xaxis.currentIndexChanged.connect(self.draw);plotbar.addWidget(self.xaxis)
        self.figure=Figure(figsize=(10,4),layout='constrained');self.canvas=FigureCanvasQTAgg(self.figure)
        rv.addWidget(NavigationToolbar2QT(self.canvas,self));rv.addWidget(self.canvas,1)
        surface_page=QWidget();sv=QVBoxLayout(surface_page);self.tabs.addTab(surface_page,'k₂ × stiffness ratio')
        self.surface=None
        sk=QHBoxLayout();sv.addLayout(sk)
        self.s_klow=self.number(.03,.001,10,4);self.s_khigh=self.number(3,.001,10,4);self.s_kcount=QSpinBox();self.s_kcount.setRange(3,61);self.s_kcount.setValue(21)
        for title,widget in [('k₂ min',self.s_klow),('max',self.s_khigh),('samples',self.s_kcount)]:sk.addWidget(QLabel(title));sk.addWidget(widget)
        sk.addSpacing(16)
        self.s_rlow=self.number(.90,.5,1.5,3);self.s_rhigh=self.number(1.10,.5,1.5,3);self.s_rcount=QSpinBox();self.s_rcount.setRange(3,61);self.s_rcount.setValue(21)
        for title,widget in [('ratio r min',self.s_rlow),('max',self.s_rhigh),('samples',self.s_rcount)]:sk.addWidget(QLabel(title));sk.addWidget(widget)
        sk2=QHBoxLayout();sv.addLayout(sk2)
        self.s_kp_on=QCheckBox('Override cable kp [N/m]');self.s_kp=self.number(250000,10,1e7,0);self.s_kp.setEnabled(False)
        self.s_kp_on.toggled.connect(self.s_kp.setEnabled);sk2.addWidget(self.s_kp_on);sk2.addWidget(self.s_kp)
        self.s_ppm_on=QCheckBox('Override image scale [px/m]');self.s_ppm=self.number(4250,500,50000,1);self.s_ppm.setEnabled(False)
        self.s_ppm_on.toggled.connect(self.s_ppm.setEnabled);sk2.addWidget(self.s_ppm_on);sk2.addWidget(self.s_ppm)
        self.s_labmodel=QCheckBox('Use current Lab model');self.s_labmodel.setChecked(True)
        self.s_labmodel.setToolTip('Score the saved points with the model in config.json (e.g. the corrected horizontal XML) instead of the copy frozen inside this campaign.');sk2.addWidget(self.s_labmodel)
        sk2.addWidget(QLabel('Show'));self.s_metric=QComboBox();self.s_metric.addItems(['RMSE [px]','J [px²]']);self.s_metric.currentIndexChanged.connect(self.draw_surface);sk2.addWidget(self.s_metric)
        self.surface_button=QPushButton('3. Sweep k₂ × ratio');self.surface_button.clicked.connect(self.sweep_surface);sk2.addWidget(self.surface_button)
        self.surface_cancel=QPushButton('Cancel');self.surface_cancel.clicked.connect(self.cancel.set);sk2.addWidget(self.surface_cancel)
        self.overlay_button=QPushButton('Overlay on pose photos');self.overlay_button.clicked.connect(self.overlay_surface);sk2.addWidget(self.overlay_button);sk2.addStretch(1)
        self.surface_text=QLabel(SURFACE_FORMULA+'. Every included pose is scored at every (k₂, r); a grid point counts only if all poses converge without contact. '
            'Default kp is the model value. Saved as fit_surface.json/.csv/.png; fit_results.json is not changed.')
        self.surface_text.setWordWrap(True);self.surface_text.setFixedHeight(70);sv.addWidget(self.surface_text)
        self.s_figure=Figure(figsize=(10,4),layout='constrained');self.s_canvas=FigureCanvasQTAgg(self.s_figure)
        sv.addWidget(NavigationToolbar2QT(self.s_canvas,self));sv.addWidget(self.s_canvas,1)
        joints_page=QWidget();jv=QVBoxLayout(joints_page);self.tabs.addTab(joints_page,'Joints + transmission')
        self.joint_result=None
        jk=QHBoxLayout();jv.addLayout(jk)
        self.j_k2=self.number(.18,.001,10,5);jk.addWidget(QLabel('Reference k₂ [N m/rad]'));jk.addWidget(self.j_k2)
        self.j_kp_on=QCheckBox('Stiff cable, kp [N/m]');self.j_kp_on.setChecked(True);self.j_kp=self.number(250000,10,1e7,0)
        self.j_kp_on.toggled.connect(self.j_kp.setEnabled);jk.addWidget(self.j_kp_on);jk.addWidget(self.j_kp)
        self.j_smooth=self.number(0,0,100,2);jk.addWidget(QLabel('Smoothness'));jk.addWidget(self.j_smooth)
        self.j_start=QComboBox();self.j_start.addItems(['exponential','uniform','previous']);self.j_start.setToolTip('previous: start from this campaign\'s last joint fit (e.g. exponential + transmission, then free the joints).');jk.addWidget(QLabel('Start'));jk.addWidget(self.j_start)
        self.j_opt=QComboBox();self.j_opt.addItem('CMA-ES → least squares','cmaes');self.j_opt.addItem('Least squares only','least_squares')
        self.j_opt.setToolTip('CMA-ES: global, derivative-free search (as in PACE), robust to slack/taut switching; then a local least-squares polish.');jk.addWidget(self.j_opt)
        self.j_cma=QSpinBox();self.j_cma.setRange(100,20000);self.j_cma.setSingleStep(100);self.j_cma.setValue(1500);jk.addWidget(QLabel('CMA budget'));jk.addWidget(self.j_cma)
        self.j_evals=QSpinBox();self.j_evals.setRange(10,400);self.j_evals.setValue(40);jk.addWidget(QLabel('Polish iterations'));jk.addWidget(self.j_evals);jk.addStretch(1)
        jk2=QHBoxLayout();jv.addLayout(jk2)
        self.j_profile=QComboBox();self.j_profile.addItem('Independent joints','independent');self.j_profile.addItem('Exponential (one ratio)','exponential');jk2.addWidget(self.j_profile)
        self.j_gain=QCheckBox('Fit cable transmission gain');self.j_gain.setChecked(True)
        self.j_gain.setToolTip('Per cable: how much of the encoder-measured cable travel reaches the body (slack, stretch, guide friction, spool radius).');jk2.addWidget(self.j_gain)
        self.j_offset=QCheckBox('Fit cable offsets');self.j_offset.setChecked(True);jk2.addWidget(self.j_offset)
        self.j_slack=QCheckBox('Antagonist slack');self.j_slack.setChecked(True)
        self.j_slack.setToolTip('The lengthened cable is treated as slack, so the pulling cable alone sets the shape (as observed). Untick only with compliant cables: two taut inextensible cables lock the body.');jk2.addWidget(self.j_slack)
        self.j_labmodel=QCheckBox('Use current Lab model');self.j_labmodel.setChecked(True);jk2.addWidget(self.j_labmodel)
        self.joint_button=QPushButton('4. Fit each joint independently');self.joint_button.clicked.connect(self.fit_joints);jk2.addWidget(self.joint_button)
        self.joint_cancel=QPushButton('Cancel');self.joint_cancel.clicked.connect(self.cancel.set);jk2.addWidget(self.joint_cancel)
        self.joint_overlay=QPushButton('Overlay on pose photos');self.joint_overlay.clicked.connect(self.overlay_joints);jk2.addWidget(self.joint_overlay)
        self.joint_export=QPushButton('Export fitted XML…');self.joint_export.clicked.connect(self.export_joints);jk2.addWidget(self.joint_export);jk2.addStretch(1)
        self.joint_text=QLabel('Every free joint j_002…j_021 gets its own stiffness; joint 1 stays fixed. Shapes determine the ratios k_i/k₂; k₂ is held at the reference '
            '(absolute stiffness needs a known force). Saved as joint_fit.json/.csv; fit_results and fit_surface are unchanged.')
        self.joint_text.setWordWrap(True);self.joint_text.setFixedHeight(70);jv.addWidget(self.joint_text)
        self.j_figure=Figure(figsize=(10,4),layout='constrained');self.j_canvas=FigureCanvasQTAgg(self.j_figure)
        jv.addWidget(NavigationToolbar2QT(self.j_canvas,self));jv.addWidget(self.j_canvas,1)
        self.table.itemChanged.connect(self.invalidate_review);self.dwell.valueChanged.connect(self.invalidate_review)
        self.return_speed.valueChanged.connect(self.invalidate_review)
        self.defaults();self.timer=QTimer(self);self.timer.timeout.connect(self.poll);self.timer.start(100)

    @staticmethod
    def number(value,low,high,decimals):
        box=QDoubleSpinBox();box.setRange(low,high);box.setDecimals(decimals);box.setValue(value);box.setKeyboardTracking(False);return box

    @property
    def active(self):return bool(self.runner and self.runner.active)

    def set_status(self,text):self.status.setText(text);self.status.setToolTip(text)

    def defaults(self):
        rests=self.host.motion_profile['rest_lengths_mm']
        self.set_rows(default_plan(rests))

    def set_rows(self,rows):
        self.invalidate_review();self.table.setRowCount(0)
        for row in rows:self.add_row(row)

    def add_row(self,row=None):
        if not isinstance(row,dict):
            a,b=self.host.motion_profile['rest_lengths_mm'];row=dict(mode='Both cables',x=a,y=b,s1=min(1,self.host.motion_profile['max_speed_mm_s']),s2=min(1,self.host.motion_profile['max_speed_mm_s']),steps=1,label='')
        i=self.table.rowCount();self.table.insertRow(i);mode=QComboBox();mode.addItems(MODES);mode.setCurrentText(row['mode']);self.table.setCellWidget(i,0,mode);mode.currentIndexChanged.connect(self.invalidate_review)
        for j,key in enumerate(('x','y','s1','s2','steps','label'),1):self.table.setItem(i,j,QTableWidgetItem(str(row[key])))

    def remove_row(self):
        if self.table.currentRow()>=0:self.invalidate_review();self.table.removeRow(self.table.currentRow())

    def rows(self):
        result=[]
        for i in range(self.table.rowCount()):
            values={k:self.table.item(i,j).text() for j,k in enumerate(('x','y','s1','s2','steps','label'),1)}
            values.update(mode=self.table.cellWidget(i,0).currentText());result.append(values)
        return result

    def load_plan(self):
        path,_=QFileDialog.getOpenFileName(self,'Load campaign plan','', 'JSON (*.json)')
        if path:
            try:
                data=json.loads(Path(path).read_text());self.set_rows(data['rows']);self.dwell.setValue(data.get('dwell_s',2))
                self.return_speed.setValue(data.get('return_speed_mm_s',min(1,self.host.motion_profile['max_speed_mm_s'])))
            except Exception as exc:self.set_status('Plan could not be loaded: '+str(exc))

    def save_plan(self):
        path,_=QFileDialog.getSaveFileName(self,'Save campaign plan','campaign_plan.json','JSON (*.json)')
        if path:
            try:atomic(path,encoded(dict(rows=self.rows(),dwell_s=self.dwell.value(),return_speed_mm_s=self.return_speed.value())))
            except Exception as exc:self.set_status('Plan could not be saved: '+str(exc))

    def build_plan(self,require_live=False):
        from analysis import inputs_at
        item=self.host.manual_source();rests=self.host.motion_profile['rest_lengths_mm']
        initial=rests[:]
        if item and item['packet'].get('motor_active'):
            initial,_=inputs_at(item['packet'],item['packet']['t_s'],self.host.config['live']['max_input_gap_s']);initial=initial.tolist()
        if require_live and (self.host.demo or not self.host.session or self.host.closing or self.host.connecting or
            self.host.remote.get('phase')!='HOLDING' or self.host.remote.get('pending') or not item or self.host.pending_comparison_change):
            raise ValueError('Connect, enable/declare rest and wait for fresh HOLDING feedback before starting.')
        plan=expand_plan(self.rows(),initial,rests,self.host.motion_profile,self.dwell.value(),self.host.config['live']['settle_window_s'],
            return_speed_mm_s=self.return_speed.value())
        remaining=self.host.remote.get('session_limit_s',self.host.duration.currentData())-self.host.remote.get('elapsed_s',0)
        estimate=(len(item['packet']['image_b64'])*3/4 if item else 300000)*len(plan['poses'])
        text=f'{len(plan["poses"])-1} measurement poses + 1 return to ({initial[0]:.3f}, {initial[1]:.3f}) mm · estimate {plan["estimated_seconds"]/60:.1f} min; reserve {plan["reserved_seconds"]/60:.1f} min · remaining {remaining/60:.1f} min. Images ≈ {estimate/1048576:.1f} MiB; cap 256 MiB. '
        text+=('TIME TOO SHORT: select a longer session before connecting.' if remaining<plan['reserved_seconds'] else 'Time budget OK.')
        self.preflight_text.setText(text)
        if require_live and remaining<plan['reserved_seconds']:raise ValueError(text)
        if estimate>256*1048576:raise ValueError('Estimated images exceed 256 MiB. Split the campaign.')
        if require_live and (self.host.scan_busy or self.host.manual_capture_busy):raise ValueError('Wait for the current single-pose analysis to finish.')
        return plan,item

    def invalidate_review(self,*_):
        if self.approved_review:self.set_status('Plan changed. Simulate and approve the revised journey before starting.')
        self.approved_review=None

    def review_context(self):
        from campaign_preview import review_context
        return review_context(self.rows(),self.dwell.value(),self.host.motion_profile,self.host.config['model'],
            self.host.fit['candidate']['parameters'],self.host.current_d2,self.host.comparison_setup,
            self.return_speed.value(),self.host.config['live']['settle_window_s'])

    def generate_sweep(self):
        from campaign_preview_window import DifferenceSweep
        dialog=DifferenceSweep(self,self.host.motion_profile)
        if dialog.exec():
            self.set_rows(dialog.rows);self.set_status('Difference sweep ready. Simulate and review the sweep and final return, then approve.')

    def generate_random(self):
        from campaign_preview_window import RandomCampaign
        dialog=RandomCampaign(self,self.host.motion_profile)
        if dialog.exec():
            self.set_rows(dialog.rows);self.set_status('Random draft ready. Use Simulate / review, then explicitly approve the path.')

    def simulate_plan(self):
        if self.active or self.busy:return
        try:
            if self.host.pending_comparison_change:raise ValueError('Wait for the model parameter change to finish.')
            plan,_=self.build_plan()
            if self.simulation_dialog:
                self.simulation_dialog.cancel.set();self.simulation_dialog.close()
            from campaign_preview_window import CampaignPreview
            self.invalidate_review()
            self.simulation_dialog=CampaignPreview(self,plan,self.host.config['model'],self.review_context(),self.approve_preview)
            self.simulation_dialog.show()
        except Exception as exc:self.set_status('Preview: '+str(exc))

    def approve_preview(self,review):
        from campaign_preview import digest
        if review['fingerprint']!=digest(self.review_context()):
            self.set_status('Settings changed while simulating. Re-preview the current journey.');return
        self.approved_review=copy.deepcopy(review)
        self.set_status('Simulated journey approved. Start sends this exact plan after Connect, enable/reference, and a starting-pose check.')

    def preflight(self):
        try:
            plan,_=self.build_plan()
            # Reviewable expansion is saved only by explicit plan export/start;
            # show all endpoints in a separate table, without sending anything.
            dialog=QDialog(self);dialog.setWindowTitle('Expanded path — no motion');dialog.resize(850,580);layout=QVBoxLayout(dialog)
            table=QTableWidget(len(plan['poses']),6);table.setHorizontalHeaderLabels(['Pose','L₁ mm','L₂ mm','Speed₁','Speed₂','Label'])
            table.setEditTriggers(QTableWidget.NoEditTriggers);table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            for i,p in enumerate(plan['poses']):
                for j,v in enumerate([p['number'],*p['lengths_mm'],*p['speeds_mm_s'],p['label']]):table.setItem(i,j,QTableWidgetItem(f'{v:.3f}' if isinstance(v,float) else str(v)))
            layout.addWidget(table);label=QLabel(self.preflight_text.text());label.setWordWrap(True);layout.addWidget(label);dialog.show();self.path_dialog=dialog
        except Exception as exc:self.set_status('Preflight: '+str(exc))

    def start(self):
        if self.quiescing:return
        if self.active or self.busy:return
        try:
            live_plan,item=self.build_plan(True)
            from campaign_preview import require_review
            plan=require_review(self.approved_review,self.review_context(),live_plan['initial_lengths_mm'])
            plan['start_feedback_lengths_mm']=live_plan['initial_lengths_mm']
            plan['simulation_review']={k:copy.deepcopy(self.approved_review[k]) for k in ('fingerprint','context','summary')}
            if self.host.remote.get('session',item['packet']['session'])!=item['packet']['session']:
                raise ValueError('Camera and motor session identities do not match.')
            plan['session']=item['packet']['session']
            self.store=CampaignStore.create(Path(self.host.session.log_dir)/'campaigns',plan,self.rows(),item,self.host.config,self.host.fit,
                'automatic' if self.mode.currentIndex()==0 else 'manual')
            self.surface=None;self.draw_surface();self.joint_result=None;self.draw_joints()
            if self.mode.currentIndex()==1:self.host.analysis_mode_box.setCurrentIndex(1)
            self.runner=CampaignRunner(plan,self.host.session.send,self.capture,self.changed)
            self.poses.setRowCount(0);self.runner.start();self.refresh_poses()
            self.host.command_status.setText('Campaign running. Stop / Hold aborts the sequence; captured poses are retained.')
        except Exception as exc:self.set_status('Campaign not started: '+str(exc))

    def changed(self,state,message):
        self.set_status(message)
        self.host.campaign_status.setText(message)
        if self.store:
            try:self.store.state(state,message,self.runner.events if self.runner else [])
            except OSError as exc:
                if self.active:self.runner.abort('Campaign journal write failed: '+str(exc))
                return
        if state=='complete':
            self.tabs.setCurrentIndex(1);self.refresh_poses()
            if self.store.doc['mode']=='automatic':QTimer.singleShot(0,lambda:self.analyse(True))
            else:self.set_status('Journey complete; starting cable lengths reached. Images retained for joint placement, shared k₂, then per-pose k₂. The return image is excluded by default.')

    def capture(self,index,item,pose):
        store=self.store
        def work():
            try:store.capture(index,item,pose);self.messages.put(('captured',(index,None)))
            except Exception as exc:self.messages.put(('captured',(index,str(exc))))
        self.capture_worker=threading.Thread(target=work,daemon=True);self.capture_worker.start()

    def motor_event(self,event):
        if self.runner:self.runner.event(event)

    def tick(self):
        if self.active:self.runner.tick(self.host.remote,time.monotonic()-self.host.remote_received,self.host.manual_source(),
            self.host.config['live']['max_age_s'],self.host.config['live']['max_input_gap_s'])

    def abort(self,reason='Campaign interrupted'):
        if self.runner:self.runner.abort(reason)

    def begin_shutdown(self):
        self.quiescing=True;self.abort('Disconnected; completed campaign poses retained')
        self.cancel.set()
        if self.simulation_dialog:self.simulation_dialog.close()
        if self.inspector:self.inspector.close()
        self.setEnabled(False)

    def end_shutdown(self):
        self.quiescing=False;self.busy=False;self.runner=None;self.setEnabled(True)
        while not self.messages.empty():
            try:self.messages.get_nowait()
            except queue.Empty:break
        if self.store and not (self.store.root/'campaign.json').exists():
            self.store=None;self.surface=None;self.draw_surface();self.joint_result=None;self.j_figure.clear();self.poses.setRowCount(0);self.set_status('This run was discarded.')
        if self.store:self.refresh_poses();self.draw()

    def shutdown_wait(self):
        self.cancel.set()
        if self.simulation_dialog:self.simulation_dialog.cancel.set()
        for thread in (self.capture_worker,self.worker,self.simulation_dialog.worker if self.simulation_dialog else None):
            if thread and thread.is_alive():thread.join()
        # No writer can race the session's Save/Discard operation afterward.

    def open(self):
        if self.active or self.busy:return
        path,_=QFileDialog.getOpenFileName(self,'Open campaign.json inside a saved run',str(Path('logs/runs').resolve()),'Campaign (campaign.json)')
        if path:self.load(path)

    def load(self,path):
        if self.quiescing:return
        try:
            self.store=CampaignStore(Path(path).parent);self.runner=None;self.store.verify_model();self.poses.setRowCount(0);self.refresh_poses();self.draw();self.load_surface();self.load_joints()
            self.j_k2.setValue(self.store.doc['fit']['candidate']['parameters']['k2'])
            self.tabs.setCurrentIndex(1)
            self.set_status(f'Opened {len(self.store.doc["poses"])}/{len(self.store.doc["plan"]["poses"])} poses. Motion never resumes from a saved campaign.')
        except Exception as exc:self.set_status('Could not open campaign: '+str(exc))

    def open_folder(self):
        if self.store:QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.store.root)))

    def refresh_poses(self):
        if not self.store:return
        selected=self.poses.currentRow();poses=self.store.doc['poses'];self.poses.setRowCount(len(poses))
        fitted={p['number']:p for p in self.store.doc.get('analysis',{}).get('poses',[])}
        for i,p in enumerate(poses):
            old_icon=self.poses.item(i,0).icon() if self.poses.item(i,0) else None
            detail=p.get('exclusion_reason') if p['excluded'] else p['reason']
            fit=fitted.get(p['number'])
            if fit:
                detail=f'Shared RMSE {fit["shared_rmse_px"]:.3f} px'+(f'; pose k₂ {fit["minimum"]["k2"]:.6g}' if fit.get('minimum') else '')
            for j,v in enumerate([p['number'],p['command']['label'],'Excluded' if p['excluded'] else 'Analysed' if fit else p['status'],
                ', '.join(f'{n:.2f}' for n in p['command']['lengths_mm']),detail]):
                cell=QTableWidgetItem(str(v));cell.setToolTip(str(v));self.poses.setItem(i,j,cell)
            if old_icon is None:
                pix=QPixmap(str(self.store.path(p['path'])/'frame.jpg')).scaled(100,56,Qt.KeepAspectRatio,Qt.SmoothTransformation)
                old_icon=QIcon(pix)
            self.poses.item(i,0).setIcon(old_icon);self.poses.setRowHeight(i,60)
        if poses:self.poses.selectRow(max(0,min(selected,len(poses)-1)))

    def edit_selected(self):
        if self.quiescing:return
        if self.active or self.busy or not self.store:return
        index=self.poses.currentRow()
        if index<0:return
        self.busy=True;self.set_status(f'Preparing pose {index+1} for joint placement…')
        def work():
            try:self.messages.put(('prepared',(index,self.store.prepared(index))))
            except Exception as exc:self.messages.put(('error',str(exc)))
        self.worker=threading.Thread(target=work,daemon=True);self.worker.start()

    def next_unfinished(self):
        if self.quiescing:return
        if not self.store:return
        index=next((i for i,p in enumerate(self.store.doc['poses']) if not p['excluded'] and not p.get('annotations')),None)
        if index is None:self.set_status('Every included pose has manual annotations. Fit shared k₂ first, then each pose.');return
        self.poses.selectRow(index);self.edit_selected()

    def saved_pose(self,index):
        self.refresh_poses();self.draw()
        QTimer.singleShot(50,self.next_unfinished)

    def exclude(self):
        if self.quiescing:return
        if not self.store or self.active or self.busy:return
        index=self.poses.currentRow()
        if index>=0:self.store.exclude(index,not self.store.doc['poses'][index]['excluded']);self.refresh_poses();self.draw()

    def analyse(self,automatic=False):
        if self.quiescing:return
        if not self.store or self.active or self.busy:return
        self.busy=True;self.cancel.clear();self.tabs.setCurrentIndex(2);self.set_status('Processing saved poses; hardware motion is complete.')
        store=self.store;args=(self.low.value(),self.high.value(),self.count.value())
        def progress(n,total,stage):self.messages.put(('progress',f'{stage}: {n}/{total}'))
        def work():
            try:
                if automatic:auto_measure(store,progress,self.cancel)
                result=fit_campaign(store,*args,progress=progress,cancel=self.cancel)
                self.messages.put(('fit',result))
            except Exception as exc:self.messages.put(('error',str(exc)))
        self.worker=threading.Thread(target=work,daemon=True);self.worker.start()

    def measure_dots(self):
        if self.quiescing or not self.store or self.active or self.busy:return
        self.busy=True;self.cancel.clear();self.set_status('Measuring marker dots on saved photos; no hardware is used.')
        store,replace=self.store,self.dots_replace.isChecked()
        def progress(n,total,stage):self.messages.put(('progress',f'{stage}: {n}/{total}'))
        def work():
            try:
                from campaign_dots import measure_campaign_dots
                self.messages.put(('dots',measure_campaign_dots(store,replace,progress,self.cancel)))
            except Exception as exc:self.messages.put(('error',str(exc)))
        self.worker=threading.Thread(target=work,daemon=True);self.worker.start()

    def sweep_surface(self):
        if self.quiescing:return
        if not self.store or self.active or self.busy:return
        self.busy=True;self.cancel.clear();self.tabs.setCurrentIndex(3);self.set_status('Sweeping k₂ × stiffness ratio over saved poses; no hardware is used.')
        store=self.store
        kwargs=dict(k_lower=self.s_klow.value(),k_upper=self.s_khigh.value(),k_count=self.s_kcount.value(),
                    r_lower=self.s_rlow.value(),r_upper=self.s_rhigh.value(),r_count=self.s_rcount.value(),
                    cable_kp=self.s_kp.value() if self.s_kp_on.isChecked() else None,
                    pixels_per_m=self.s_ppm.value() if self.s_ppm_on.isChecked() else None,
                    model_path=str(Path(self.host.config['model']).resolve()) if self.s_labmodel.isChecked() else None)
        def progress(n,total,stage):self.messages.put(('progress',f'{stage}: {n}/{total} pose evaluations'))
        def work():
            try:self.messages.put(('surface',fit_surface(store,progress=progress,cancel=self.cancel,**kwargs)))
            except Exception as exc:self.messages.put(('error',str(exc)))
        self.worker=threading.Thread(target=work,daemon=True);self.worker.start()

    def overlay_surface(self):
        if self.quiescing or not self.store or not self.surface or self.active or self.busy:return
        self.busy=True;self.set_status('Drawing the surface minimum on each pose photo…')
        store,result=self.store,self.surface
        def work():
            try:self.messages.put(('overlay',render_overlays(store,result)))
            except Exception as exc:self.messages.put(('error',str(exc)))
        self.worker=threading.Thread(target=work,daemon=True);self.worker.start()

    def fit_joints(self):
        if self.quiescing or not self.store or self.active or self.busy:return
        self.busy=True;self.cancel.clear();self.tabs.setCurrentIndex(4);self.set_status('Fitting each joint stiffness to the saved poses; no hardware is used.')
        store=self.store
        kwargs=dict(k2_reference=self.j_k2.value(),cable_kp=self.j_kp.value() if self.j_kp_on.isChecked() else None,
                    smooth=self.j_smooth.value(),fit_offset=self.j_offset.isChecked(),start=self.j_start.currentText(),
                    profile=self.j_profile.currentData(),fit_gain=self.j_gain.isChecked(),
                    optimizer=self.j_opt.currentData(),cma_evaluations=self.j_cma.value(),slack_antagonist=self.j_slack.isChecked(),
                    max_evaluations=self.j_evals.value(),
                    model_path=str(Path(self.host.config['model']).resolve()) if self.j_labmodel.isChecked() else None)
        def progress(n,total,stage):self.messages.put(('progress',f'{stage}: {n}/{total} evaluations'))
        def work():
            try:
                from joint_fit import fit_joints
                self.messages.put(('joints',fit_joints(store,progress=progress,cancel=self.cancel,**kwargs)))
            except Exception as exc:self.messages.put(('error',str(exc)))
        self.worker=threading.Thread(target=work,daemon=True);self.worker.start()

    def overlay_joints(self):
        if self.quiescing or not self.store or not self.joint_result or self.active or self.busy:return
        self.busy=True;self.set_status('Drawing the independent-joint fit on each pose photo…')
        store,result=self.store,self.joint_result
        def work():
            try:
                from joint_fit import render_overlays as joint_overlays
                self.messages.put(('overlay',joint_overlays(store,result)))
            except Exception as exc:self.messages.put(('error',str(exc)))
        self.worker=threading.Thread(target=work,daemon=True);self.worker.start()

    def export_joints(self):
        if not self.joint_result:return
        src=Path(self.joint_result['model_xml'])
        name,_=QFileDialog.getSaveFileName(self,'Export XML with fitted joint stiffness',str(src.with_name(src.stem+'_joints_fitted.xml')),'MuJoCo XML (*.xml)')
        if not name:return
        try:
            from joint_fit import export_xml
            path=export_xml(self.joint_result,name)
            note='' if Path(name).parent==src.parent else ' Keep it beside the original XML so the meshes resolve.'
            self.set_status(f'Exported {path.name} (joint stiffness only; damping unchanged).'+note)
        except Exception as exc:self.set_status('Export failed: '+str(exc))

    def load_joints(self):
        self.joint_result=None
        path=self.store.root/'joint_fit.json' if self.store else None
        if path and path.exists():
            try:self.joint_result=json.loads(path.read_text())
            except (OSError,ValueError):self.joint_result=None
        self.draw_joints()

    def draw_joints(self,*_):
        self.j_figure.clear();res=self.joint_result
        if not res:self.j_canvas.draw_idle();return
        a,b=self.j_figure.subplots(1,2)
        js=res['joints'];idx=np.arange(2,2+len(js));w=np.array([j['weight'] for j in js]);sd=np.array([j['sigma_log'] for j in js])
        det=np.array([j['determined'] for j in js]);expw=np.array([j['exponential_weight'] for j in js])
        a.errorbar(idx,w,yerr=[w-w*np.exp(-sd),w*np.exp(sd)-w],fmt='none',ecolor='#90a4ae',lw=1,capsize=2)
        a.plot(idx[det],w[det],'o',color='#1e88e5',label='Fitted, pinned by data')
        a.plot(idx[~det],w[~det],'o',mfc='none',color='#1e88e5',label='Fitted, NOT pinned (σ > factor 2)')
        a.plot(idx,expw,'--',color='#e53935',label='Campaign exponential')
        a.set(yscale='log',xlabel='Joint index i (j_00i)',ylabel='k_i / k₂',title=f"Joint stiffness relative to k₂ = {res['k2_reference']:.4g} N m/rad");a.legend(fontsize=7);a.grid(alpha=.3,which='both')
        nums=[p['number'] for p in res['poses']]
        b.plot(nums,[p['start_rmse_px'] or np.nan for p in res['poses']],'s--',color='#e53935',ms=4,label='Start (campaign values)')
        b.plot(nums,[p['rmse_px'] or np.nan for p in res['poses']],'o-',color='#1e88e5',ms=4,label='Fitted')
        b.set(xlabel='Pose',ylabel='RMSE [px]',title='Error per pose');b.legend(fontsize=7);b.grid(alpha=.3)
        n_det=int(det.sum());stale='' if all(self.store.doc['poses'][p['number']-1]['annotation_revision']==p['annotation_revision'] for p in res['poses']) else ' · STALE: points changed, fit again'
        t=res.get('transmission') or {};prof=res.get('profile','independent')
        tx=(f" · transmission gain {t['gain'][0]:.3f}±{t['gain_sigma'][0]:.3f} / {t['gain'][1]:.3f}±{t['gain_sigma'][1]:.3f}" if t.get('fitted_gain') else '')+(f" · offsets {t['offset_mm'][0]:+.2f} / {t['offset_mm'][1]:+.2f} mm" if t.get('fitted_offset') else '')
        model=('independent joints' if prof=='independent' else f"exponential r = {res['ratio']:.4f}±{res['ratio_sigma']:.4f}")
        self.joint_text.setText(f"{len(res['poses'])} poses · {model} · cable kp {res['cable_kp_N_per_m']:.6g} N/m{tx} · "
            f"RMSE {res['start_rmse_px']:.1f} → {res['rmse_px']:.1f} px · {n_det}/{len(js)} joints pinned by the data{stale}.\n"
            +(f"CMA-ES {res['cma']['evaluations']} evaluations, then " if res.get('cma') else '')+f"{'converged' if res['converged'] else 'stopped at iteration limit'} after {res['evaluations']} least-squares iterations. Hollow points: the data do not determine that joint; add poses or smoothness. Values in joint_fit.csv.")
        from appearance import theme_figure
        theme_figure(self.j_figure,self.host.theme);self.j_canvas.draw_idle()
        self.j_figure.savefig(self.store.root/'joint_fit.png',dpi=140)

    def load_surface(self):
        self.surface=None
        path=self.store.root/'fit_surface.json' if self.store else None
        if path and path.exists():
            try:self.surface=json.loads(path.read_text())
            except (OSError,ValueError):self.surface=None
        self.draw_surface()

    def draw_surface(self,*_):
        self.s_figure.clear();res=self.surface
        if not res:
            self.s_canvas.draw_idle();return
        rmse=self.s_metric.currentIndex()==0
        ks=np.array(res['k2']);rs=np.array(res['ratio'])
        J=np.array([[np.nan if v is None else v for v in row] for row in res['shared_cost_px2']],float)
        Z=np.sqrt(J) if rmse else J;label='Shared RMSE [px]' if rmse else 'Shared J [px²]'
        a,b=self.s_figure.subplots(1,2)
        import matplotlib;cmap=matplotlib.colormaps['viridis_r'].copy();cmap.set_bad('#9e9e9e')
        mesh=a.pcolormesh(ks,rs,np.ma.masked_invalid(Z),shading='nearest',cmap=cmap);self.s_figure.colorbar(mesh,ax=a,label=label)
        if np.isfinite(Z).sum()>=4:
            try:a.contour(ks,rs,Z,levels=10,colors='white',linewidths=.5,alpha=.6)
            except (ValueError,TypeError):pass
        m=res['minimum'];a.scatter(m['k2'],m['ratio'],marker='*',s=180,color='#e36b36',edgecolor='k',label='Minimum',zorder=5)
        base=res.get('baseline_exponential')
        if base:a.scatter(base['k2'],base['ratio'],marker='o',s=60,facecolor='none',edgecolor='#d32f2f',lw=1.5,label='Campaign k₂, r',zorder=5)
        a.axhline(1.0,color='w',lw=.6,ls='--')
        a.set(xscale='log',xlabel='k₂ [N m/rad]',ylabel='Adjacent stiffness ratio r',title='Grey = invalid for ≥1 pose');a.legend(loc='best',fontsize=8)
        v=res['valley']
        if v:
            vr=[p['ratio'] for p in v];vy=[p['rmse_px'] if rmse else p['cost_px2'] for p in v]
            h=b.plot(vr,vy,'o-',ms=3,color='#087e8b',label=f'Lowest {"RMSE" if rmse else "J"} at each r (left axis)')
            b.set(xlabel='Adjacent stiffness ratio r',ylabel=label,title='Valley: best k₂ for each ratio')
            b2=b.twinx();h+=b2.plot(vr,[p['k2'] for p in v],'--',color='#9e9e9e',lw=1,label='k₂ giving that minimum (right axis)')
            b2.set_ylabel('k₂ at valley [N m/rad]');b2.set_yscale('log')
            if min(p['k2'] for p in v)<=ks[0]*1.0001:
                b2.axhline(ks[0],color='#9e9e9e',lw=.6,ls=':');h.append(b2.plot([],[],':',color='#9e9e9e',lw=.6,label='k₂ grid floor: valley clipped where it lies on this line')[0])
            h.append(b.axvline(m['ratio'],color='#e36b36',lw=1.2,label=f"Overall minimum r = {m['ratio']:.3f}"))
            if base:h.append(b.axvline(base['ratio'],color='#d32f2f',lw=1,ls=':',label=f"Campaign model r = {base['ratio']:.3f}"))
            b.legend(handles=h,loc='upper center',fontsize=7,framealpha=.85)
        kp=f"cable kp {res['cable_kp_N_per_m']:.6g} N/m"+(' (overridden)' if res['cable_kp_overridden'] else ' (model)')
        if res.get('model_source')=='external':kp+=f" · Lab model {Path(res['model_xml']).parent.name}/{Path(res['model_xml']).name} ({res['model_xml_sha256'][:8]})"
        elif 'model_source' in res:kp+=' · campaign frozen model'
        if 'pixels_per_m' in res:kp+=f" · scale {res['pixels_per_m']:.6g} px/m"+(' (overridden)' if res.get('pixels_per_m_overridden') else ' (campaign)')
        stale='' if (self.store and surface_is_current(self.store,res)) else ' · STALE: points changed since this sweep, run again'
        edge=f" · minimum on {', '.join(m['boundary_axes'])} grid edge: widen range" if m['at_boundary'] else ''
        old=f" · campaign (k₂ {base['k2']:.4g}, r {base['ratio']:.4g}): RMSE {base['rmse_px']:.3f} px" if base else ''
        self.surface_text.setText(f"{res['included_poses']} poses · {kp} · valid {res['valid_points']}/{res['total_points']} grid points.\n"
            f"Minimum: k₂ = {m['k2']:.5g} N m/rad, r = {m['ratio']:.4g}, J = {m['cost_px2']:.2f} px², RMSE = {m['rmse_px']:.3f} px{old}{edge}{stale}.\n"
            'k₂ is relative to cable kp and gravity: report it as an effective value. Per-pose minima are in fit_surface.json.')
        from appearance import theme_figure
        theme_figure(self.s_figure,self.host.theme);self.s_canvas.draw_idle()
        if self.store:self.s_figure.savefig(self.store.root/'fit_surface.png',dpi=140)

    def individual(self):
        if self.quiescing:return
        if not self.store or self.active or self.busy:return
        try:add_per_pose(self.store);self.refresh_poses();self.draw();self.set_status('Separate k₂ values added using the same physical measurements and k₂ grid. Results saved.')
        except Exception as exc:self.set_status(str(exc))

    def draw(self,*_):
        self.figure.clear();result=self.store.doc.get('analysis') if self.store else None
        if not result:
            self.result_text.setText('No current shared baseline. Complete measurements, then Fit shared k₂.');self.canvas.draw_idle();return
        a,b=self.figure.subplots(1,2);grid=result['k2']
        for p in result['poses']:a.plot(grid,[np.nan if v is None else v for v in p['cost_px2']],color='#78909c',alpha=.2,lw=.7)
        a.plot(grid,[np.nan if v is None else v for v in result['shared_cost_px2']],color='#087e8b',lw=2,label='Shared mean J')
        best=result['shared_minimum'];a.scatter(best['k2'],best['cost_px2'],color='#e36b36');a.set(xscale='log',xlabel='k₂ [N m/rad]',ylabel='J [px² / point]',title='Equal weight per pose');a.legend()
        key=['difference_mm','measured_lateral_mm','number'][self.xaxis.currentIndex()]
        if result['per_pose']:
            xs=[p[key] for p in result['poses']];ys=[p['minimum']['k2'] for p in result['poses']]
            b.plot(xs,ys,'o-',lw=.6,ms=4,label='Separate k₂ (pose order)')
            for p,x,y in zip(result['poses'],xs,ys):
                if p['minimum']['at_boundary']:b.scatter(x,y,marker='x',s=60,color='#c62828')
        b.axhline(best['k2'],color='#087e8b',label='Shared k₂');b.set(xlabel=self.xaxis.currentText(),ylabel='k₂ [N m/rad]',title='Red × = grid boundary');b.legend()
        n=result['included_poses'];warning=' COMPLETE' if result['complete'] else ' INCOMPLETE / SUBSET'
        boundary=' · minimum at grid boundary: widen range' if best['at_boundary'] else ''
        expected=result.get('expected_fit_poses',result['expected_poses'])
        self.result_text.setText(f'{warning}: {n}/{expected} fit poses; {result["captured_poses"]}/{result["expected_poses"]} images · shared k₂ = {best["k2"]:.6g}; J = {best["cost_px2"]:.3f} px²; RMSE = {best["rmse_px"]:.3f} px{boundary}.\n'
            f'Grid {grid[0]:.4g}–{grid[-1]:.4g} ({len(grid)} samples). Inspect forward/reverse repeats; lower per-pose error alone does not prove variable stiffness. CSV/JSON contain each pose and all samples.')
        from appearance import theme_figure
        theme_figure(self.figure,self.host.theme);self.canvas.draw_idle()
        self.figure.savefig(self.store.root/'fit_results.png',dpi=140)

    def poll(self):
        if self.quiescing:return
        while True:
            try:kind,value=self.messages.get_nowait()
            except queue.Empty:break
            if kind=='captured':
                if self.runner:self.runner.captured(*value)
                self.refresh_poses()
            elif kind=='progress':self.set_status(value)
            elif kind=='fit':self.busy=False;self.refresh_poses();self.draw();self.set_status('Shared baseline finished and saved. Review it, then choose 2. Fit each pose.')
            elif kind=='dots':
                self.busy=False;self.refresh_poses();self.draw()
                ok=[r for r in value if r['links']];cmp=[r['mean_px'] for r in value if r.get('mean_px') is not None]
                self.set_status(f'Marker dots: {len(ok)}/{len(value)} poses measured'+(f'; mean distance to your element centres {np.mean(cmp):.1f} px over {len(cmp)} poses' if cmp else '')
                    +'. Review dots_overlay.jpg in each pose folder and dots_summary.csv.')
            elif kind=='joints':self.busy=False;self.joint_result=value;self.draw_joints();self.set_status('Independent joint fit finished and saved to joint_fit.json/.csv/.png.')
            elif kind=='surface':self.busy=False;self.surface=value;self.draw_surface();self.set_status('k₂ × ratio surface finished and saved to fit_surface.json/.csv/.png.')
            elif kind=='overlay':
                self.busy=False;self.set_status(f'Saved {len(value)} overlays ({value[0].name} …) in the campaign folder.')
                QDesktopServices.openUrl(QUrl.fromLocalFile(str(value[0])))
            elif kind=='error':self.busy=False;self.refresh_poses();self.set_status('Analysis: '+value)
            elif kind=='prepared':
                self.busy=False
                if self.host.closing or self.host.close_when_done:continue
                from campaign_review import CampaignInspector
                if self.inspector:self.inspector.close()
                index,item=value;self.inspector=CampaignInspector(self.store,index,item,self,self.saved_pose);self.inspector.show()
        active=self.active
        editing=bool(self.inspector and self.inspector.isVisible())
        self.start_button.setEnabled(bool(self.approved_review) and not active and not self.busy and not editing and not self.host.demo)
        self.start_button.setToolTip('Simulate / review and approve the plan first.' if not self.approved_review else 'Run the approved journey from its reviewed starting lengths.')
        self.stop_button.setEnabled(bool(self.host.session));self.plan_page.setEnabled(not active and not self.busy and not editing)
        self.open_button.setEnabled(not active and not self.busy and not editing)
        for b in (self.fit_button,self.surface_button,self.joint_button,self.edit_button,self.next_button,self.exclude_button,self.dots_button):b.setEnabled(bool(self.store) and not active and not self.busy and not editing)
        self.individual_button.setEnabled(bool(self.store and self.store.doc.get('analysis')) and not active and not self.busy and not editing)
        self.cancel_button.setEnabled(self.busy);self.surface_cancel.setEnabled(self.busy)
        self.overlay_button.setEnabled(bool(self.store and self.surface) and not active and not self.busy and not editing)
        self.joint_overlay.setEnabled(bool(self.store and self.joint_result) and not active and not self.busy and not editing);self.joint_export.setEnabled(bool(self.joint_result) and not self.busy)
        self.joint_cancel.setEnabled(self.busy)

    def closeEvent(self,event):
        self.abort('Campaign window closed; held and retained completed poses.')
        self.cancel.set()
        if self.simulation_dialog:self.simulation_dialog.close()
        if self.inspector:self.inspector.close()
        event.accept()
