"""Optional rotatable 3D stiffness review, separate from the default J(k2) plot."""
import copy
import csv
import json
from pathlib import Path
import queue
import threading
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (QApplication, QDialog, QVBoxLayout, QGridLayout,
    QHBoxLayout, QLabel, QDoubleSpinBox, QSpinBox, QComboBox, QPushButton,
    QProgressBar, QFileDialog, QMessageBox)
from appearance import theme_figure
from stiffness_sweep import sample_surface, PROFILE_AXES, FORMULA, SurfaceCancelled


def draw_surface(figure, scan, theme='light'):
    figure.clear()
    ax=figure.add_subplot(111,projection='3d',computed_zorder=False)
    x=np.log10(np.asarray(scan['k2']))
    y=np.asarray(scan['constant_values'])
    X,Y=np.meshgrid(x,y)
    Z=np.array([[np.nan if z is None else z for z in row] for row in scan['cost_px2']])
    ax.plot_surface(X,Y,Z,cmap='viridis',alpha=.85,linewidth=.2,edgecolor='#666666',antialiased=True,zorder=1)
    valid=np.isfinite(Z)
    ax.scatter(X[valid],Y[valid],Z[valid],s=5,color='#555555',alpha=.55,zorder=2)
    best=scan['minimum']
    ax.scatter([np.log10(best['k2'])],[best['constant_value']],[best['cost_px2']],
               s=160,marker='*',color='#ee6800',edgecolor='black',label='Lowest sampled J',depthshade=False,zorder=10)
    ticks=np.linspace(x[0],x[-1],3)
    ax.set_xticks(ticks, [f'{10**v:.3g}' for v in ticks])
    ax.set_yticks(np.linspace(y[0],y[-1],3))
    ax.tick_params(labelsize=8)
    ax.set_box_aspect((1.2,1.,.8),zoom=.80)
    ax.set(xlabel='k₂ [N m/rad]',ylabel='Ratio r' if scan['parameter']=='stiffness_ratio' else 'Multiplier b',zlabel='J [px² / point]',
           title='Frozen pose: lowest valid sampled J (logarithmic k₂ axis)')
    ax.xaxis.label.set_fontsize(9);ax.yaxis.label.set_fontsize(9);ax.zaxis.label.set_fontsize(9)
    ax.view_init(elev=28,azim=-125)
    ax.legend(loc='upper left',fontsize=9)
    theme_figure(figure,theme)
    fg='#ffffff' if theme=='dark' else '#000000'
    ax.zaxis.label.set_color(fg);ax.zaxis.get_offset_text().set_color(fg)
    for axis in (ax.xaxis,ax.yaxis,ax.zaxis):
        axis.set_pane_color((.13,.13,.13,1.) if theme=='dark' else (.95,.95,.95,1.))
    return ax


def export_surface(path, scan):
    path=Path(path).with_suffix('.png')
    figure=Figure(figsize=(10,8),layout='constrained')
    draw_surface(figure,scan)
    best=scan['minimum'];source=scan['source']
    figure.suptitle(f"{source.get('session','')} · frame {source.get('frame_index','')}\n"
        f"Jmin={best['cost_px2']:.6g}; k₂={best['k2']:.7g}; {scan['parameter_label']}={best['constant_value']:.7g}\n"
        +('Boundary minimum; extend/refine the grid. ' if best['at_boundary'] else '')
        +f"Valid samples {scan['valid_sample_count']}/{scan['actual_sample_count']}",fontsize=10)
    figure.savefig(path,dpi=180)
    with path.with_suffix('.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['k2_Nm_per_rad','stiffness_ratio','stiffness_exponent','effective_adjacent_ratio',
                         'J_px2_per_point','lowest_sample','error'])
        for r,value in enumerate(scan['constant_values']):
            p=dict(scan['fixed_parameters'],**{scan['parameter']:value})
            for c,k in enumerate(scan['k2']):
                writer.writerow([k,p['stiffness_ratio'],p['stiffness_exponent'],
                    p['stiffness_ratio']**p['stiffness_exponent'],scan['cost_px2'][r][c],
                    r==best['row'] and c==best['column'],scan['errors'][r][c] or ''])
    path.with_suffix('.json').write_text(json.dumps(scan,indent=2,allow_nan=False)+'\n')
    return path


class SurfaceReview(QDialog):
    def __init__(self, model_path, source, use_minimum, parent=None):
        super().__init__(parent)
        self.model_path=model_path;self.source=copy.deepcopy(source);self.use_minimum=use_minimum
        self.result=None;self.running=False;self.messages=queue.Queue();self.cancel=threading.Event()
        self.setWindowTitle('3D stiffness profile review — one frozen pose')
        self.setMinimumSize(860,600)
        screen=QApplication.primaryScreen().availableGeometry()
        self.resize(min(1100,screen.width()-60),min(850,screen.height()-80))
        layout=QVBoxLayout(self)
        info=QLabel(f"Frozen frame {source['frame_index']} · {source['session']}\n"
            'Uses exactly the image points and measured cable inputs from the completed 1D sweep.');info.setWordWrap(True);layout.addWidget(info)
        form=QGridLayout();layout.addLayout(form)
        self.kmin=self.number(source['lower'],.001,10.);self.kmax=self.number(source['upper'],.001,10.)
        self.kcount=QSpinBox();self.kcount.setRange(3,31);self.kcount.setValue(11)
        self.parameter=QComboBox()
        for key,(label,_,_) in PROFILE_AXES.items():self.parameter.addItem(label,key)
        self.low=self.number(.85,.5,1.5);self.high=self.number(1.,.5,1.5)
        self.count=QSpinBox();self.count.setRange(3,31);self.count.setValue(9)
        for col,(label,widget) in enumerate([('k₂ minimum',self.kmin),('k₂ maximum',self.kmax),('k₂ samples',self.kcount)]):
            form.addWidget(QLabel(label),0,col);form.addWidget(widget,1,col)
        for col,(label,widget) in enumerate([('Vary ONE constant',self.parameter),('Constant minimum',self.low),('Constant maximum',self.high),('Constant samples',self.count)]):
            form.addWidget(QLabel(label),2,col);form.addWidget(widget,3,col)
        self.parameter.currentIndexChanged.connect(self.axis_changed)
        self.formula=QLabel();self.formula.setWordWrap(True);layout.addWidget(self.formula)
        self.axis_changed()
        actions=QHBoxLayout();layout.addLayout(actions)
        self.run_button=QPushButton('Calculate surface');self.run_button.clicked.connect(self.start);actions.addWidget(self.run_button)
        self.cancel_button=QPushButton('Cancel calculation');self.cancel_button.clicked.connect(self.cancel.set);self.cancel_button.setEnabled(False);actions.addWidget(self.cancel_button)
        self.use_button=QPushButton('Use minimum in parameter fields');self.use_button.clicked.connect(self.use);self.use_button.setEnabled(False);actions.addWidget(self.use_button)
        self.save_button=QPushButton('Save surface…');self.save_button.clicked.connect(self.save);self.save_button.setEnabled(False);actions.addWidget(self.save_button)
        self.progress=QProgressBar();self.progress.setRange(0,100);self.progress.setValue(0);layout.addWidget(self.progress)
        self.status=QLabel('Choose ranges, then Calculate surface. Drag the graph to rotate; scroll to zoom.');self.status.setWordWrap(True);self.status.setFixedHeight(58);layout.addWidget(self.status)
        self.figure=Figure(figsize=(9,6),layout='constrained');self.canvas=FigureCanvasQTAgg(self.figure);layout.addWidget(self.canvas,1)
        self.toolbar=NavigationToolbar2QT(self.canvas,self);layout.addWidget(self.toolbar)
        self.figure.text(.5,.5,'Optional J(k₂, one profile constant) review',ha='center')
        theme_figure(self.figure,QApplication.instance().property('spirob_theme') or 'light')
        self.timer=QTimer(self);self.timer.timeout.connect(self.poll)

    @staticmethod
    def number(value,lo,hi):
        box=QDoubleSpinBox();box.setDecimals(10);box.setRange(lo,hi);box.setValue(value);box.setKeyboardTracking(False)
        return box

    def axis_changed(self):
        key=self.parameter.currentData();_,lo,hi=PROFILE_AXES[key]
        for box in (self.low,self.high):box.setRange(lo,hi)
        fixed=self.source['fixed_parameters'];value=fixed[key]
        width=.08 if key=='stiffness_ratio' else .5
        self.low.setValue(max(lo,value-width));self.high.setValue(min(hi,value+width))
        other='stiffness_exponent' if key=='stiffness_ratio' else 'stiffness_ratio'
        self.formula.setText(FORMULA+f". Fixed {PROFILE_AXES[other][0]} = {fixed[other]:.10g}.\n"
            'rᵇ sets the same profile slope: these two constants cannot be identified independently. '
            'Alignment, cable offsets and damping remain fixed.')

    def start(self):
        if self.running:return
        key=self.parameter.currentData()
        values=(self.low.value(),self.high.value(),self.count.value(),self.kmin.value(),self.kmax.value(),self.kcount.value())
        if values[0]>=values[1] or values[3]>=values[4]:
            self.status.setText('Each minimum must be below its maximum.');return
        self.running=True;self.cancel.clear();self.run_button.setEnabled(False);self.cancel_button.setEnabled(True)
        self.use_button.setEnabled(False);self.save_button.setEnabled(False);self.progress.setValue(0)
        for w in (self.parameter,self.low,self.high,self.count,self.kmin,self.kmax,self.kcount):w.setEnabled(False)
        self.status.setText('Calculating the frozen-pose grid. Live camera and motor controls continue. Cancel stops after the current equilibrium solve.')
        self.timer.start(100)
        source=copy.deepcopy(self.source);model_path=self.model_path;messages=self.messages;cancel=self.cancel
        def work():
            try:
                result=sample_surface(model_path,source,key,*values,
                    progress=lambda done,total:messages.put(('progress',(done,total))),cancel=cancel)
                messages.put(('done',result))
            except SurfaceCancelled as exc:messages.put(('cancelled',str(exc)))
            except Exception as exc:messages.put(('failed',str(exc)))
        threading.Thread(target=work,daemon=True).start()

    def poll(self):
        while not self.messages.empty():
            kind,value=self.messages.get_nowait()
            if kind=='progress':self.progress.setValue(round(100*value[0]/value[1]));continue
            self.running=False;self.timer.stop();self.run_button.setEnabled(True);self.cancel_button.setEnabled(False)
            for w in (self.parameter,self.low,self.high,self.count,self.kmin,self.kmax,self.kcount):w.setEnabled(True)
            if kind=='done':
                self.result=value;best=value['minimum']
                draw_surface(self.figure,value,QApplication.instance().property('spirob_theme') or 'light');self.canvas.draw_idle()
                self.status.setText(f"Lowest sampled J = {best['cost_px2']:.6g} at k₂ = {best['k2']:.7g}, "
                    f"{value['parameter_label']} = {best['constant_value']:.7g}. "
                    +('Minimum lies at the grid boundary; extend/refine the ranges. ' if best['at_boundary'] else '')
                    +f"{value['valid_sample_count']}/{value['actual_sample_count']} valid equilibria; invalid cells are gaps. "
                    'This is the sampled minimum for this pose, not a guaranteed global optimum.')
            else:self.status.setText(value+(' The displayed surface is the previous completed result.' if self.result else ''))
            self.status.setToolTip(self.status.text())
            self.use_button.setEnabled(self.result is not None);self.save_button.setEnabled(self.result is not None)

    def use(self):
        if not self.result or self.running:return
        try:
            self.use_minimum(self.result)
            self.status.setText('Minimum copied to k₂, r and b fields in the main window. Click Apply to simulation there to use it. No physical motor command was sent.')
        except ValueError as exc:self.status.setText(str(exc))

    def save(self):
        if not self.result or self.running:return
        name,_=QFileDialog.getSaveFileName(self,'Save 3D stiffness surface','spirob_stiffness_surface.png','PNG image (*.png)')
        if not name:return
        path=Path(name).with_suffix('.png')
        if any(path.with_suffix(s).exists() for s in ('.csv','.json')):
            if QMessageBox.question(self,'Replace companion files?','Replace existing surface CSV/JSON companions?')!=QMessageBox.Yes:return
        try:
            export_surface(path,self.result);self.status.setText('Saved surface PNG, all grid samples in CSV, and frozen observation/provenance in JSON: '+str(path))
        except (OSError,ValueError) as exc:QMessageBox.warning(self,'Save failed',str(exc))

    def reject(self):
        self.cancel.set()
        if not self.running:self.timer.stop()
        super().reject()
