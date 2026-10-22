"""Frozen, memory-only landmark inspection. No motor or fitting commands.

Manual points replace the automatic image curve and initialize image tracking.
"""
import copy
import csv
import json
from pathlib import Path
import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox, QDoubleSpinBox, QCheckBox, QTableWidget,
    QTableWidgetItem, QAbstractItemView, QFileDialog, QMessageBox, QSplitter, QWidget,
    QApplication, QTabWidget, QHeaderView, QSizePolicy)
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
from appearance import theme_figure


from landmarks import correspondences
from joint_tracking import manual_profile


class PointInspector(QDialog):
    profile_applied = Signal(object)
    def __init__(self, supplier, parent=None, apply_callback=None, manual_mode=False, manual_callback=None, recapture_callback=None):
        super().__init__(parent)
        self.supplier = supplier
        self.apply_callback = apply_callback
        self.manual_mode=manual_mode;self.manual_callback=manual_callback
        self.source_item = None
        self.snapshot = None
        self.applied_profile = None
        self.manual = {}; self.excluded = set(); self.rows = []; self.updating = False
        self._view_initialized=False
        self.setWindowTitle('SpiRob — joint / element point inspector (frozen image)')
        self.setMinimumSize(980,640)
        available=QApplication.primaryScreen().availableGeometry()
        self.resize(min(1440,max(980,available.width()-48)),min(860,max(640,available.height()-64)))
        root=QVBoxLayout(self);root.setContentsMargins(8,8,8,8);root.setSpacing(5)
        self.caption=QLabel();self.caption.setWordWrap(True);self.caption.setMinimumWidth(0)
        self.caption.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Fixed);self.caption.setFixedHeight(40);root.addWidget(self.caption)
        tools=QHBoxLayout();root.addLayout(tools)
        self.capture_button=QPushButton('Capture current frame');tools.addWidget(self.capture_button)
        self.capture_button.clicked.connect(recapture_callback if manual_mode and recapture_callback else self.capture)
        self.mode=QComboBox();self.mode.addItems(['Joints + tip','Element centres']);tools.addWidget(self.mode)
        self.mode.currentIndexChanged.connect(self.mode_changed)
        self.full_frame=QCheckBox('Full frame');self.full_frame.setChecked(manual_mode);tools.addWidget(self.full_frame)
        self.full_frame.toggled.connect(lambda _checked:self.fit_view())
        self.fit_button=QPushButton('Fit view');self.fit_button.clicked.connect(self.fit_view);tools.addWidget(self.fit_button)
        self.place_button=QPushButton('Place selected');self.place_button.clicked.connect(self.begin_placement);tools.addWidget(self.place_button)
        self.dots_button=QPushButton('Auto-place marker dots');self.dots_button.clicked.connect(lambda:self.auto_place_dots())
        self.dots_button.setToolTip('Find the black dot at each element centre and place link_002…link_021 (Element centres mode). Edit any point afterwards.')
        tools.addWidget(self.dots_button);self.dot_base=None
        tools.addStretch()
        self.save_button=QPushButton('Save image + table…');tools.addWidget(self.save_button);self.save_button.clicked.connect(self.save)
        help_button=QPushButton('Help…');tools.addWidget(help_button)
        help_text=('MANUAL FROZEN POSE — no extraction or tracking. Select a point row and place its physical feature. At least four points are required; unplaced joints are omitted. Confirm the photographed pose was stationary, then sweep. Capture current frame starts another pose.' if manual_mode else
            'Select a point row and place its physical hinge centre. Apply joints uses your exact coordinates and replaces the green curve. In rigid mode, identify every joint through the tip of the visible chain. Save setup retains calibration and image references. For a settled capture, Apply joints + sweep freezes these points and encoder inputs. Live tracking uses image features and fixed adjacent link lengths; keep the camera fixed and motion planar. If tracking fails, capture and place joints again.')
        help_text+='\n\nUse the toolbar to zoom or pan, then click Place selected to return to placement without changing the view. Point edits, row selection and exclusions preserve zoom/pan. Fit view, the Full frame option or a new capture resets the view. Detailed columns expose model coordinates and residual components. Clicks are never snapped to the automatic trace.'
        help_button.clicked.connect(lambda:QMessageBox.information(self,'Point placement',help_text))

        self.split=QSplitter(Qt.Horizontal);self.split.setChildrenCollapsible(False);root.addWidget(self.split,1)
        left=QWidget();lv=QVBoxLayout(left);lv.setContentsMargins(0,0,0,0);lv.setSpacing(3)
        self.figure=Figure(figsize=(9,7),layout='constrained');self.canvas=FigureCanvasQTAgg(self.figure);self.ax=self.figure.subplots()
        self.canvas.setMinimumSize(320,200);self.canvas.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Expanding)
        self.toolbar=NavigationToolbar2QT(self.canvas,self);lv.addWidget(self.toolbar);lv.addWidget(self.canvas,1)
        self.canvas.mpl_connect('button_press_event',self.clicked_image)
        self.edit_hint=QLabel('Zoom / pan with the toolbar → Place selected → click the physical point. Fit view resets zoom.')
        self.edit_hint.setWordWrap(True);self.edit_hint.setMinimumWidth(0);self.edit_hint.setFixedHeight(34);lv.addWidget(self.edit_hint)
        self.split.addWidget(left)

        right=QWidget();right.setMinimumWidth(410);rv=QVBoxLayout(right);rv.setContentsMargins(0,0,0,0);rv.setSpacing(4)
        self.detail_columns=QCheckBox('Show model coordinates and residual components');rv.addWidget(self.detail_columns)
        self.table=QTableWidget(0,9);self.table.setMinimumHeight(170)
        self.table.setHorizontalHeaderLabels(['Use','Point','Physical x,y','MuJoCo x,y','Δx','Δy','Distance','Source','Visible'])
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows);self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers);self.table.verticalHeader().hide()
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Interactive)
        self.table.horizontalHeader().setStretchLastSection(True)
        for i,width in enumerate((34,76,108,108,60,60,76,112,60)):self.table.setColumnWidth(i,width)
        self.table.itemSelectionChanged.connect(self.selected);self.table.itemChanged.connect(self.inclusion_changed)
        self.detail_columns.toggled.connect(self.show_details);self.show_details(False);rv.addWidget(self.table,1)
        self.summary=QLabel();self.summary.setWordWrap(True);self.summary.setFixedHeight(34);self.summary.setMinimumWidth(0);rv.addWidget(self.summary)
        self.selected_name=QLabel('Select a point row');rv.addWidget(self.selected_name)
        self.x=self.number(0.,0.,1919.);self.y=self.number(0.,0.,1079.)
        xy=QHBoxLayout();rv.addLayout(xy)
        xy.addWidget(QLabel('x [px]'));xy.addWidget(self.x,1);xy.addWidget(QLabel('y [px]'));xy.addWidget(self.y,1)
        self.x.valueChanged.connect(self.adjust_selected);self.y.valueChanged.connect(self.adjust_selected)
        self.place=QCheckBox('Click image to place selected physical point');self.place.setChecked(manual_mode);rv.addWidget(self.place)
        reset=QHBoxLayout();rv.addLayout(reset)
        selected_reset=QPushButton('Reset selected point');reset.addWidget(selected_reset);selected_reset.clicked.connect(self.reset_selected)
        all_reset=QPushButton('Reset all points');reset.addWidget(all_reset);all_reset.clicked.connect(self.reset_all)

        self.profile_tabs=QTabWidget();self.profile_tabs.setFixedHeight(182 if not manual_mode else 160);rv.addWidget(self.profile_tabs)
        apply_page=QWidget();apply_layout=QVBoxLayout(apply_page);apply_layout.setContentsMargins(7,5,7,5);apply_layout.setSpacing(3)
        self.profile_tabs.addTab(apply_page,'Manual sweep' if manual_mode else 'Apply / sweep')
        self.score_profile=QCheckBox('Use verified points for J / k₂ sweep');self.score_profile.setChecked(True)
        self.score_profile.setToolTip('Uses the exact frozen coordinates and matching MuJoCo anchors. Subsequent frames need valid image tracking.')
        apply_layout.addWidget(self.score_profile);self.score_profile.setVisible(not manual_mode)
        self.rigid=QCheckBox('Keep joint-to-joint distances fixed (planar motion)');self.rigid.setChecked(True)
        apply_layout.addWidget(self.rigid);self.rigid.setVisible(not manual_mode)
        self.stationary=QCheckBox('Physical SpiRob was stationary in this captured frame')
        apply_layout.addWidget(self.stationary);self.stationary.setVisible(manual_mode);self.stationary.toggled.connect(self.manual_readiness)
        self.apply_profile_button=QPushButton('Apply joints to live view + J');apply_layout.addWidget(self.apply_profile_button)
        self.apply_profile_button.clicked.connect(lambda:self.apply_profile());self.apply_profile_button.setVisible(not manual_mode)
        self.save_profile_button=QPushButton('Apply joints + save setup for next run');apply_layout.addWidget(self.save_profile_button)
        self.save_profile_button.clicked.connect(lambda:self.apply_profile(save=True));self.save_profile_button.setVisible(not manual_mode)
        self.sweep_profile_button=QPushButton('Sweep k₂ using these manual points' if manual_mode else 'Apply joints + sweep this frozen pose')
        apply_layout.addWidget(self.sweep_profile_button);self.sweep_profile_button.clicked.connect(lambda:self.apply_profile(sweep=True))
        self.manual_status=QLabel();self.manual_status.setWordWrap(True);self.manual_status.setMinimumWidth(0);self.manual_status.setFixedHeight(50)
        apply_layout.addWidget(self.manual_status);self.manual_status.setVisible(manual_mode)
        spacing_page=QWidget();knobs=QGridLayout(spacing_page);self.profile_tabs.addTab(spacing_page,'Image spacing')
        self.tip_offset=self.number(0.,-200.,200.);self.arc_scale=self.number(1.,.5,1.5,4);self.arc_scale.setSingleStep(.005)
        knobs.addWidget(QLabel('Tip offset [px]'),0,0);knobs.addWidget(self.tip_offset,0,1)
        knobs.addWidget(QLabel('Spacing multiplier'),1,0);knobs.addWidget(self.arc_scale,1,1)
        spacing_note=QLabel('Adjust automatic correspondence only. Exact manual clicks stay at their chosen pixels.');spacing_note.setWordWrap(True);knobs.addWidget(spacing_note,2,0,1,2)
        self.profile_tabs.setTabVisible(1,not manual_mode)
        self.tip_offset.valueChanged.connect(self.rebuild);self.arc_scale.valueChanged.connect(self.rebuild)
        self.split.addWidget(right);self.split.setSizes([880,470])
        self.capture()

    def show_details(self, checked):
        for column in (3,4,5,8):self.table.setColumnHidden(column,not checked)

    def begin_placement(self):
        # Leave the navigation tool without undoing its zoom/pan operation.
        if self.toolbar.mode == 'pan/zoom':self.toolbar.pan()
        elif self.toolbar.mode == 'zoom rect':self.toolbar.zoom()
        self.place.setChecked(True)
        self.edit_hint.setText('Click the selected physical point. Zoom is retained; Fit view resets it.')

    def fit_view(self, *_):
        self._view_initialized=False
        self.redraw()

    def apply_profile(self, save=False, sweep=False):
        if self.snapshot is None:return
        try:
            if self.manual_mode:
                self.manual_callback(self.source_item,self.rows,'centres' if self.mode.currentIndex() else 'joints',self.stationary.isChecked())
                self.caption.setText('Sweeping this manually measured frozen pose. Save the combined screenshot in the main window when the sweep finishes.')
                return
            profile=manual_profile(self.rows,
                'centres' if self.mode.currentIndex() else 'joints',self.snapshot['frame_index'],
                self.snapshot['rgb'],self.score_profile.isChecked(),rigid=self.rigid.isChecked())
            if sweep and (not self.source_item.get('result') or not self.source_item['result']['settled']):
                raise ValueError('Capture a settled connected pose to calculate a k₂ sweep. Setup preview has no encoder inputs.')
            if self.apply_callback:self.apply_callback(profile,save,self.source_item if sweep else None,reference=self.source_item)
            else:self.profile_applied.emit(profile)
            self.caption.setText('Exact joint calibration applied'+(' and saved.' if save else '.')+(' Sweeping this frozen image; save the combined pose screenshot in the main window.' if sweep else ' Live placement now uses image tracking; no snapping to the automatic curve.'))
        except (ValueError,OSError) as exc:QMessageBox.warning(self,'Placement not applied',str(exc))

    @staticmethod
    def number(value, low, high, decimals=1):
        box=QDoubleSpinBox();box.setRange(low,high);box.setDecimals(decimals);box.setValue(value)
        box.setKeyboardTracking(False);return box

    def capture(self):
        item = self.supplier()
        if item is None or item.get('rgb') is None:
            self.caption.setText('No fresh camera frame available. Capture current frame when video resumes. Any previous snapshot below is unchanged.')
            self.save_button.setEnabled(self.snapshot is not None)
            return
        # Copy only one frame and bounded metadata; never retain packet histories.
        result = item.get('result')
        self.source_item=copy.deepcopy(item)
        self.snapshot = dict(rgb=item['rgb'].copy(), observed=copy.deepcopy(item['observed']),
            layout=copy.deepcopy(result['landmarks'] if result else item['landmarks']),
            session=item['packet']['session'], frame_index=item['packet']['frame_index'],
            t_s=item['packet']['t_s'], parameters=copy.deepcopy(item['parameters']), d2=item['d2'],
            model_status=item['message'], recorded_demo=bool(item.get('recorded_demo')),
            lengths_mm=None if result is None else result['lengths_mm'],
            setup=copy.deepcopy(result.get('setup',{}) if result else item.get('setup',{})),
            frame_age_s=item['displayed_age_s'], clock_uncertainty_s=item['packet'].get('clock_uncertainty_s'))
        h,w=self.snapshot['rgb'].shape[:2]; self.x.setMaximum(w-1); self.y.setMaximum(h-1)
        self.save_button.setEnabled(True)
        self.sweep_profile_button.setEnabled(bool(result and result.get('settled')))
        self.stationary.setChecked(False)
        self._view_initialized=False
        self.reset_all()
        self.applied_profile=None if self.manual_mode else copy.deepcopy(self.snapshot['setup'].get('profile'))
        if not self.applied_profile:self.auto_place_dots(silent=True)
        if self.applied_profile:
            self.mode.blockSignals(True);self.mode.setCurrentIndex(int(self.applied_profile['mode']=='centres'));self.mode.blockSignals(False)
            self.score_profile.setChecked(self.applied_profile.get('use_for_score',False));self.rebuild()
            self.rigid.setChecked(self.applied_profile['mode']=='joints')
        model_label = 'Setup reference pose (no encoder feedback)' if item.get('setup_preview') else 'Time-matched model' if result and not self.snapshot['recorded_demo'] else 'Recorded static model' if result else 'Model unavailable: '+item['message']
        self.caption.setText(f'FROZEN — {self.snapshot["session"]} · frame {self.snapshot["frame_index"]} · t={self.snapshot["t_s"]:.3f} s\n{model_label}. The live camera and motor controls continue in the main window.')
        self.caption.setToolTip(self.caption.text())
        self.manual_readiness()

    def dot_spacing(self):
        """Expected element-centre spacing [px] from the model in use and the image scale."""
        from spirob_analysis.model import SpiRobModel
        from dot_tracking import expected_spacing_px
        store=getattr(self,'store',None)
        if store is not None:path=store.config()['model']
        else:
            host=self.parent()
            path=(getattr(host,'config',None) or {}).get('model')
            if not path:raise ValueError('No model available for dot spacing.')
        scale=self.snapshot['setup'].get('pixels_per_m') or self.snapshot['parameters'].get('pixels_per_m')
        return expected_spacing_px(SpiRobModel(path),float(scale))

    def auto_place_dots(self, silent=False):
        """Image-only: dots → element-centre points the user can then edit."""
        if self.snapshot is None:return False
        from dot_tracking import measure_dots
        try:result=measure_dots(self.snapshot['rgb'],self.dot_spacing())
        except (ValueError,OSError) as exc:
            if not silent:QMessageBox.warning(self,'Marker dots not placed',str(exc))
            else:self.edit_hint.setText('Marker dots not found automatically ('+str(exc)+'). Place points manually or press Auto-place marker dots.')
            return False
        self.mode.blockSignals(True);self.mode.setCurrentIndex(1);self.mode.blockSignals(False)
        self.rigid.setChecked(False);self.rigid.setEnabled(False)
        self.applied_profile=None
        self.manual=dict(zip(result['landmark_ids'],result['points_px']))
        self.excluded=set(result['missing_links'])
        self.dot_base=result['base_dots_px']
        self.rebuild()
        missing=', '.join(n.replace('link_0','L') for n in result['missing_links'])
        self.edit_hint.setText(f'Marker dots placed automatically: {len(result["landmark_ids"])}/20 element centres'
            +(f'; not found (unchecked): {missing}. Tick a row and place it to add it.' if missing else '.')+' Select any row and click to correct it.')
        return True

    def manual_readiness(self, *_):
        if not self.manual_mode or self.snapshot is None:return
        count=sum(r['included'] and r['physical_px'] is not None for r in self.rows)
        error=self.source_item.get('capture_error')
        if error:reason=error
        elif count<4:reason=f'{count} physical points placed. Place at least four matching visible joints or centres.'
        elif not self.stationary.isChecked():reason='Points are ready. Confirm that the physical robot was stationary at capture.'
        else:reason=f'Ready to sweep {count} exact manual points. Automatic extraction and tracking are not used.'
        self.manual_status.setText(reason)
        self.sweep_profile_button.setEnabled(not error and count>=4 and self.stationary.isChecked())
        self.sweep_profile_button.setToolTip(reason)

    def mode_changed(self):
        self.rigid.setEnabled(not self.mode.currentIndex())
        self.rigid.setChecked(not self.mode.currentIndex())
        self.applied_profile=None;self.manual.clear(); self.excluded.clear(); self.rebuild()

    def reset_all(self):
        self.applied_profile=None
        self.manual.clear();self.excluded.clear()
        self.tip_offset.blockSignals(True);self.tip_offset.setValue(0);self.tip_offset.blockSignals(False)
        self.arc_scale.blockSignals(True);self.arc_scale.setValue(1);self.arc_scale.blockSignals(False)
        self.rebuild()

    def reset_selected(self):
        i=self.table.currentRow()
        if 0<=i<len(self.rows): self.manual.pop(self.rows[i]['id'],None);self.rebuild()

    def rebuild(self, *_):
        if self.snapshot is None: return
        selected=max(0,self.table.currentRow())
        scroll=(self.table.verticalScrollBar().value(),self.table.horizontalScrollBar().value())
        self.rows=correspondences(self.snapshot['observed'],self.snapshot['layout'],
            'centres' if self.mode.currentIndex() else 'joints', self.tip_offset.value(),self.arc_scale.value(),self.manual,self.excluded,self.applied_profile)
        if self.manual_mode:
            for row in self.rows:
                if row['physical_px'] is None:row['source']='not placed'
        self.updating=True
        self.table.setRowCount(len(self.rows))
        for i,row in enumerate(self.rows):
            xy=lambda p:'—' if p is None else f'{p[0]:.1f}, {p[1]:.1f}'
            values=['',row['id'],xy(row['physical_px']),xy(row['model_px']),
                    *['—' if row[k] is None else f'{row[k]:.2f}' for k in ('dx_px','dy_px','distance_px')],row['source'], 'yes' if row['physical_px'] is not None else 'no']
            for j,value in enumerate(values):
                cell=QTableWidgetItem(value)
                if j==0:
                    cell.setFlags(Qt.ItemIsEnabled|Qt.ItemIsUserCheckable|Qt.ItemIsSelectable)
                    cell.setCheckState(Qt.Checked if row['included'] else Qt.Unchecked)
                self.table.setItem(i,j,cell)
        self.table.selectRow(min(selected,len(self.rows)-1))
        self.table.verticalScrollBar().setValue(scroll[0]);self.table.horizontalScrollBar().setValue(scroll[1])
        distances=[r['distance_px'] for r in self.rows if r['included'] and r['distance_px'] is not None]
        self.summary.setText(f'{len(distances)} paired points included · diagnostic RMSE = {np.sqrt(np.mean(np.square(distances))):.2f} px' if distances else 'No included point pairs. Physical points can still be inspected.')
        self.updating=False;self.selected()
        self.manual_readiness()

    def inclusion_changed(self, item):
        if self.updating or item.column()!=0:return
        name=self.rows[item.row()]['id']
        if item.checkState()==Qt.Checked:self.excluded.discard(name)
        else:self.excluded.add(name)
        self.rebuild()

    def selected(self):
        if self.updating:return
        i=self.table.currentRow()
        self.selected_name.setText('Selected: '+self.rows[i]['id'] if 0<=i<len(self.rows) else 'Select a point row')
        point=self.rows[i]['physical_px'] if 0<=i<len(self.rows) else None
        self.updating=True
        self.x.setEnabled(point is not None);self.y.setEnabled(point is not None)
        if point is not None:self.x.setValue(point[0]);self.y.setValue(point[1])
        self.updating=False;self.redraw()

    def adjust_selected(self, *_):
        if self.updating:return
        i=self.table.currentRow()
        if 0<=i<len(self.rows):self.manual[self.rows[i]['id']]=[self.x.value(),self.y.value()];self.rebuild()

    def clicked_image(self, event):
        if not self.place.isChecked() or event.inaxes!=self.ax or event.button!=1 or self.canvas.toolbar.mode:return
        i=self.table.currentRow()
        if not 0<=i<len(self.rows):return
        h,w=self.snapshot['rgb'].shape[:2]
        if 0<=event.xdata<w and 0<=event.ydata<h:
            self.manual[self.rows[i]['id']]=[float(event.xdata),float(event.ydata)];self.rebuild()

    def redraw(self, *_):
        if self.snapshot is None:return
        view=(self.ax.get_xlim(),self.ax.get_ylim()) if self._view_initialized else None
        ax=self.ax;ax.clear();ax.imshow(self.snapshot['rgb'])
        backbone=np.asarray((self.snapshot['observed'] or {}).get('curve_px', [])).reshape(-1,2)
        if len(backbone):ax.plot(backbone[:,0],backbone[:,1],color='#888888',ls='--',lw=.6,alpha=.6)
        placed=np.array([r['physical_px'] for r in self.rows if r['physical_px'] is not None]).reshape(-1,2)
        if len(placed):ax.plot(placed[:,0],placed[:,1],color='#14eeb5',lw=1.2)
        if self.snapshot['layout']['model_px'] is not None:
            virtual=np.asarray(self.snapshot['layout']['model_px'])
            ax.plot(virtual[:,0],virtual[:,1],color='#ff4538',lw=.6,alpha=.55)
        selected=self.table.currentRow()
        for i,row in enumerate(self.rows):
            p,q=row['physical_px'],row['model_px'];alpha=1 if row['included'] else .25
            if p is not None and q is not None:ax.plot([p[0],q[0]],[p[1],q[1]],color='#ffdb75',lw=.7,alpha=alpha)
            label=row['id'].replace('j_0','J').replace('link_0','L')
            if p is not None:
                ax.scatter(*p,s=20,facecolors='#14eeb5',edgecolors='#072c24',linewidths=.5,alpha=alpha,zorder=4)
                ax.annotate(label,p,xytext=(4,-10),textcoords='offset points',color='#003d2f',fontsize=7,
                    bbox=dict(facecolor='white',alpha=.7,pad=.2,edgecolor='none'),zorder=5)
            if q is not None:
                ax.scatter(*q,s=36,facecolors='none',edgecolors='#ff4538',linewidths=1,alpha=alpha,zorder=4)
                ax.annotate(label,q,xytext=(4,5),textcoords='offset points',color='#a91208',fontsize=7,
                    bbox=dict(facecolor='white',alpha=.7,pad=.2,edgecolor='none'),zorder=5)
            if i==selected and p is not None:ax.scatter(*p,s=100,facecolors='none',edgecolors='#1682ff',linewidths=1.7,zorder=6)
        h,w=self.snapshot['rgb'].shape[:2]
        if view is not None:ax.set(xlim=view[0],ylim=view[1])
        elif self.full_frame.isChecked():ax.set(xlim=(-.5,w-.5),ylim=(h-.5,-.5))
        elif len(backbone) or len(placed):
            path=np.vstack([backbone,placed]);lo=path.min(0)-80;hi=path.max(0)+80
            ax.set(xlim=(max(0,lo[0]),min(w,hi[0])),ylim=(min(h,hi[1]),max(0,lo[1])))
        else:ax.set(xlim=(-.5,w-.5),ylim=(h-.5,-.5))
        ax.set(title='Green: physical · Red: MuJoCo · Thin lines: paired distance',xlabel='Native image x [px]',ylabel='Native image y [px]')
        if self.dot_base:
            b=np.asarray(self.dot_base);ax.scatter(b[:,0],b[:,1],marker='x',s=40,color='#ff9800',zorder=6)
        if self.snapshot['layout'].get('base_px'):
            base=self.snapshot['layout']['base_px'];ax.scatter(*base,marker='+',s=90,color='#ff4538')
            ax.annotate('j_001 base',base,xytext=(8,5),textcoords='offset points',color='black',
                        bbox=dict(facecolor='white',alpha=.85,edgecolor='none'))
        from PySide6.QtWidgets import QApplication
        theme_figure(self.figure,QApplication.instance().property('spirob_theme') or 'light')
        if view is None:
            self._view_initialized=True
            self.toolbar.update();self.toolbar.push_current()
        self.canvas.draw_idle()

    def export(self, destination):
        """Called only by explicit Save (or verification in a temporary folder)."""
        path=Path(destination).with_suffix('.png')
        self.figure.savefig(path,dpi=180)
        with path.with_suffix('.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['point','included','physical_x_px','physical_y_px','model_x_px','model_y_px','dx_px','dy_px','distance_px','source'])
            for r in self.rows:writer.writerow([r['id'],r['included'],*(r['physical_px'] or ['', '']),*(r['model_px'] or ['', '']),r['dx_px'],r['dy_px'],r['distance_px'],r['source']])
        metadata={k:v for k,v in self.snapshot.items() if k!='rgb'}
        metadata.update(version='0.8.0',mode=self.mode.currentText(),tip_offset_px=self.tip_offset.value(),arc_scale=self.arc_scale.value(),
            manual=self.manual,excluded=sorted(self.excluded),rows=self.rows,
            meaning='Green polyline uses the displayed physical coordinates exactly. Apply joints transfers these measurements to live tracking and optionally J; it does not change the MuJoCo mechanics.')
        path.with_suffix('.json').write_text(json.dumps(metadata,indent=2,allow_nan=False)+'\n')
        return path

    def save(self):
        if self.snapshot is None:return
        name,_=QFileDialog.getSaveFileName(self,'Save frozen point comparison',f'points_{self.snapshot["frame_index"]}.png','PNG image (*.png)')
        if not name:return
        path=Path(name).with_suffix('.png')
        if any(path.with_suffix(s).exists() for s in ('.csv','.json')):
            if QMessageBox.question(self,'Replace companion files?','The CSV/JSON companion already exists. Replace it?')!=QMessageBox.Yes:return
        try:
            self.export(path);self.caption.setText(self.caption.text().split('\nSaved:')[0]+'\nSaved: '+str(path)+' + CSV + JSON')
        except (OSError,ValueError) as exc:QMessageBox.warning(self,'Save failed',str(exc))
