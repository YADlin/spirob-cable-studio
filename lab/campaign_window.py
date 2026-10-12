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


class CampaignWindow(QDialog):
    def __init__(self,host):
        super().__init__(host);self.host=host;self.store=None;self.runner=None;self.inspector=None
        self.quiescing=False
        self.messages=queue.Queue();self.worker=None;self.capture_worker=None;self.cancel=threading.Event();self.busy=False
        self.setWindowTitle('SpiRob — campaigns');self.setMinimumSize(960,620)
        screen=QApplication.primaryScreen().availableGeometry();self.resize(min(1160,max(960,screen.width()-48)),min(730,max(620,screen.height()-64)))
        root=QVBoxLayout(self);self.status=QLabel('Draft a plan, inspect Preflight, then Start after Connect and Enable at rest.')
        self.status.setWordWrap(True);self.status.setFixedHeight(50);root.addWidget(self.status)
        bar=QHBoxLayout();root.addLayout(bar)
        self.start_button=QPushButton('Start campaign');self.start_button.clicked.connect(self.start);bar.addWidget(self.start_button)
        self.stop_button=QPushButton('Stop campaign + hold');self.stop_button.clicked.connect(lambda:self.abort('User stopped campaign'));bar.addWidget(self.stop_button)
        self.open_button=QPushButton('Open captured campaign…');self.open_button.clicked.connect(self.open);bar.addWidget(self.open_button)
        self.folder_button=QPushButton('Open data folder');self.folder_button.clicked.connect(self.open_folder);bar.addWidget(self.folder_button)
        self.tabs=QTabWidget();root.addWidget(self.tabs,1)
        plan_page=QWidget();pl=QVBoxLayout(plan_page);self.tabs.addTab(plan_page,'Plan / preflight')
        self.plan_page=plan_page
        row=QHBoxLayout();pl.addLayout(row)
        self.mode=QComboBox();self.mode.addItems(['Automatic shared k₂ after journey','Collect now; place joints afterward']);row.addWidget(self.mode,1)
        self.dwell=self.number(2,0,30,1);row.addWidget(QLabel('Extra dwell [s]'));row.addWidget(self.dwell)
        self.preflight_button=QPushButton('Preflight');self.preflight_button.clicked.connect(self.preflight);row.addWidget(self.preflight_button)
        note=QLabel('Difference: x = full D, y = mean. Both: x = L₁, y = L₂. Cable 1/2 changes only its own length. Speeds in mm/s.\nIntervals = 1: waypoint only; >1: capture each interpolated step from the preceding row. First row is always one capture.')
        note.setWordWrap(True);pl.addWidget(note)
        self.table=QTableWidget(0,7);self.table.setHorizontalHeaderLabels(['Motion','x [mm]','y [mm]','Speed 1','Speed 2','Intervals','Label'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch);pl.addWidget(self.table,1)
        buttons=QHBoxLayout();pl.addLayout(buttons)
        for title,handler in [('Default 73 poses',self.defaults),('Add row',self.add_row),('Remove row',self.remove_row),('Load plan…',self.load_plan),('Save plan…',self.save_plan)]:
            b=QPushButton(title);b.clicked.connect(handler);buttons.addWidget(b)
        self.preflight_text=QLabel('Preflight expands every row and checks the session motion envelope, storage budget and remaining time.');self.preflight_text.setWordWrap(True);self.preflight_text.setFixedHeight(65);pl.addWidget(self.preflight_text)
        poses_page=QWidget();pv=QVBoxLayout(poses_page);self.tabs.addTab(poses_page,'Captured poses / batch review')
        controls=QHBoxLayout();pv.addLayout(controls)
        self.edit_button=QPushButton('Place / review joints');self.edit_button.clicked.connect(self.edit_selected);controls.addWidget(self.edit_button)
        self.next_button=QPushButton('Review next unfinished');self.next_button.clicked.connect(self.next_unfinished);controls.addWidget(self.next_button)
        self.exclude_button=QPushButton('Exclude / include selected');self.exclude_button.clicked.connect(self.exclude);controls.addWidget(self.exclude_button)
        self.poses=QTableWidget(0,5);self.poses.setHorizontalHeaderLabels(['Pose / image','Label','State','L₁, L₂ targets','Details'])
        self.poses.setSelectionBehavior(QTableWidget.SelectRows);self.poses.setSelectionMode(QTableWidget.SingleSelection)
        self.poses.setEditTriggers(QTableWidget.NoEditTriggers);self.poses.setIconSize(__import__('PySide6.QtCore',fromlist=['QSize']).QSize(100,56))
        self.poses.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch);pv.addWidget(self.poses,1)
        self.poses.cellDoubleClicked.connect(lambda *_:self.edit_selected())
        review_note=QLabel('Frames and drafts are retained in this run. Save & next opens the next unfinished pose. Explicitly exclude obscured points.\nPrevious-image optical-flow suggestions are drafts for your review; they never use MuJoCo as the physical measurement.')
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
        self.defaults();self.timer=QTimer(self);self.timer.timeout.connect(self.poll);self.timer.start(100)

    @staticmethod
    def number(value,low,high,decimals):
        box=QDoubleSpinBox();box.setRange(low,high);box.setDecimals(decimals);box.setValue(value);box.setKeyboardTracking(False);return box

    @property
    def active(self):return bool(self.runner and self.runner.active)

    def set_status(self,text):self.status.setText(text);self.status.setToolTip(text)

    def defaults(self):
        rests=self.host.remote.get('rest_lengths_mm') or [a['rest_length_mm'] for a in (self.host.rig or {}).get('wsl',{}).get('axes',[{'rest_length_mm':220.},{'rest_length_mm':220.}])]
        self.set_rows(default_plan(rests))

    def set_rows(self,rows):
        self.table.setRowCount(0)
        for row in rows:self.add_row(row)

    def add_row(self,row=None):
        if not isinstance(row,dict):row=dict(mode='Both cables',x=220,y=220,s1=1,s2=1,steps=1,label='')
        i=self.table.rowCount();self.table.insertRow(i);mode=QComboBox();mode.addItems(MODES);mode.setCurrentText(row['mode']);self.table.setCellWidget(i,0,mode)
        for j,key in enumerate(('x','y','s1','s2','steps','label'),1):self.table.setItem(i,j,QTableWidgetItem(str(row[key])))

    def remove_row(self):
        if self.table.currentRow()>=0:self.table.removeRow(self.table.currentRow())

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
            except Exception as exc:self.set_status('Plan could not be loaded: '+str(exc))

    def save_plan(self):
        path,_=QFileDialog.getSaveFileName(self,'Save campaign plan','campaign_plan.json','JSON (*.json)')
        if path:
            try:atomic(path,encoded(dict(rows=self.rows(),dwell_s=self.dwell.value())))
            except Exception as exc:self.set_status('Plan could not be saved: '+str(exc))

    def build_plan(self,require_live=False):
        from analysis import inputs_at
        item=self.host.manual_source();rests=self.host.remote.get('rest_lengths_mm') or [a['rest_length_mm'] for a in (self.host.rig or {}).get('wsl',{}).get('axes',[{'rest_length_mm':220.},{'rest_length_mm':220.}])]
        initial=rests[:]
        if item and item['packet'].get('motor_active'):
            initial,_=inputs_at(item['packet'],item['packet']['t_s'],self.host.config['live']['max_input_gap_s']);initial=initial.tolist()
        if require_live and (self.host.demo or not self.host.session or self.host.closing or self.host.connecting or
            self.host.remote.get('phase')!='HOLDING' or self.host.remote.get('pending') or not item or self.host.pending_comparison_change):
            raise ValueError('Connect, enable/declare rest and wait for fresh HOLDING feedback before starting.')
        plan=expand_plan(self.rows(),initial,rests,self.host.difference_limit.value(),self.dwell.value(),self.host.config['live']['settle_window_s'])
        remaining=self.host.remote.get('session_limit_s',self.host.duration.currentData())-self.host.remote.get('elapsed_s',0)
        estimate=(len(item['packet']['image_b64'])*3/4 if item else 300000)*len(plan['poses'])
        text=f'{len(plan["poses"])} captures · estimate {plan["estimated_seconds"]/60:.1f} min; reserve {plan["reserved_seconds"]/60:.1f} min · remaining {remaining/60:.1f} min. Images ≈ {estimate/1048576:.1f} MiB at current JPEG size; hard cap 256 MiB. '
        text+=('TIME TOO SHORT: select a longer session before connecting.' if remaining<plan['reserved_seconds'] else 'Time budget OK.')
        self.preflight_text.setText(text)
        if require_live and remaining<plan['reserved_seconds']:raise ValueError(text)
        if estimate>256*1048576:raise ValueError('Estimated images exceed 256 MiB. Split the campaign.')
        if require_live and (self.host.scan_busy or self.host.manual_capture_busy):raise ValueError('Wait for the current single-pose analysis to finish.')
        return plan,item

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
            plan,item=self.build_plan(True)
            if self.host.remote.get('session',item['packet']['session'])!=item['packet']['session']:
                raise ValueError('Camera and motor session identities do not match.')
            plan['session']=item['packet']['session']
            self.store=CampaignStore.create(Path(self.host.session.log_dir)/'campaigns',plan,self.rows(),item,self.host.config,self.host.fit,
                'automatic' if self.mode.currentIndex()==0 else 'manual')
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
            else:self.set_status('Journey complete. All images retained; ready for one sitting of joint placement. Then fit shared k₂, followed by each pose.')

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
        if self.inspector:self.inspector.close()
        self.setEnabled(False)

    def end_shutdown(self):
        self.quiescing=False;self.busy=False;self.runner=None;self.setEnabled(True)
        while not self.messages.empty():
            try:self.messages.get_nowait()
            except queue.Empty:break
        if self.store and not (self.store.root/'campaign.json').exists():
            self.store=None;self.poses.setRowCount(0);self.set_status('This run was discarded.')
        if self.store:self.refresh_poses();self.draw()

    def shutdown_wait(self):
        self.cancel.set()
        for thread in (self.capture_worker,self.worker):
            if thread and thread.is_alive():thread.join()
        # No writer can race the session's Save/Discard operation afterward.

    def open(self):
        if self.active or self.busy:return
        path,_=QFileDialog.getOpenFileName(self,'Open campaign.json inside a saved run',str(Path('logs/runs').resolve()),'Campaign (campaign.json)')
        if path:self.load(path)

    def load(self,path):
        if self.quiescing:return
        try:
            self.store=CampaignStore(Path(path).parent);self.runner=None;self.store.verify_model();self.poses.setRowCount(0);self.refresh_poses();self.draw()
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
        self.result_text.setText(f'{warning}: {n}/{result["expected_poses"]} poses · shared k₂ = {best["k2"]:.6g}; J = {best["cost_px2"]:.3f} px²; RMSE = {best["rmse_px"]:.3f} px{boundary}.\n'
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
            elif kind=='error':self.busy=False;self.refresh_poses();self.set_status('Analysis: '+value)
            elif kind=='prepared':
                self.busy=False
                if self.host.closing or self.host.close_when_done:continue
                from campaign_review import CampaignInspector
                if self.inspector:self.inspector.close()
                index,item=value;self.inspector=CampaignInspector(self.store,index,item,self,self.saved_pose);self.inspector.show()
        active=self.active
        editing=bool(self.inspector and self.inspector.isVisible())
        self.start_button.setEnabled(not active and not self.busy and not editing and not self.host.demo)
        self.stop_button.setEnabled(active);self.plan_page.setEnabled(not active and not self.busy and not editing)
        self.open_button.setEnabled(not active and not self.busy and not editing)
        for b in (self.fit_button,self.edit_button,self.next_button,self.exclude_button):b.setEnabled(bool(self.store) and not active and not self.busy and not editing)
        self.individual_button.setEnabled(bool(self.store and self.store.doc.get('analysis')) and not active and not self.busy and not editing)
        self.cancel_button.setEnabled(self.busy)

    def closeEvent(self,event):
        self.abort('Campaign window closed; held and retained completed poses.')
        self.cancel.set()
        if self.inspector:self.inspector.close()
        event.accept()
