"""Offline campaign generation, cancellable simulation and explicit approval."""
import copy
import queue
import threading
import time
import numpy as np
from PySide6.QtCore import Qt,QTimer
from PySide6.QtWidgets import (QDialog,QVBoxLayout,QHBoxLayout,QGridLayout,QLabel,QPushButton,
    QComboBox,QSpinBox,QDoubleSpinBox,QCheckBox,QSlider,QDialogButtonBox)
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg,NavigationToolbar2QT
from campaign_preview import random_plan,difference_sweep,mixed_difference_range,simulate,digest


def number(v,lo,hi):
    b=QDoubleSpinBox();b.setDecimals(3);b.setRange(lo,hi);b.setValue(v);b.setKeyboardTracking(False);return b


class RandomCampaign(QDialog):
    def __init__(self,parent,profile):
        super().__init__(parent);self.profile=copy.deepcopy(profile);self.rows=None
        self.setWindowTitle('Random campaign — create draft only');self.resize(670,460)
        root=QVBoxLayout(self);grid=QGridLayout();root.addLayout(grid)
        self.mode=QComboBox();self.mode.addItems(['Mixed','Both cables','Difference'])
        grid.addWidget(QLabel('Motion mode'),0,0);grid.addWidget(self.mode,0,1,1,2)
        self.bounds=[]
        for i,(lo,hi) in enumerate(profile['absolute_bounds_mm']):
            r=profile['rest_lengths_mm'][i];a=number(max(lo,r-20),lo,hi);b=number(min(hi,r+20),lo,hi)
            grid.addWidget(QLabel(f'Cable {i+1} min / max [mm]'),i+1,0);grid.addWidget(a,i+1,1);grid.addWidget(b,i+1,2);self.bounds.append([a,b])
        self.dlow=number(-40,-profile['difference_mm'],profile['difference_mm']);self.dhigh=number(40,-profile['difference_mm'],profile['difference_mm'])
        grid.addWidget(QLabel('Difference min / max [mm]'),3,0);grid.addWidget(self.dlow,3,1);grid.addWidget(self.dhigh,3,2)
        self.mean=number(sum(profile['rest_lengths_mm'])/2,.001,10000)
        grid.addWidget(QLabel('Difference-mode mean [mm]'),4,0);grid.addWidget(self.mean,4,1)
        cap=profile['max_speed_mm_s'];self.slow=number(min(1,cap),.05,cap);self.shigh=number(min(4,cap),.05,cap)
        grid.addWidget(QLabel('Speed min / max [mm/s]'),5,0);grid.addWidget(self.slow,5,1);grid.addWidget(self.shigh,5,2)
        self.linked=QCheckBox('Same speed for both cables at each pose');self.linked.setChecked(True);grid.addWidget(self.linked,6,0,1,3)
        self.count=QSpinBox();self.count.setRange(2,200);self.count.setValue(12)
        self.seed=QSpinBox();self.seed.setRange(0,2147483647);self.seed.setValue(20261013)
        grid.addWidget(QLabel('Measurement poses (+ 1 return)'),7,0);grid.addWidget(self.count,7,1);grid.addWidget(QLabel('Reproducible seed'),8,0);grid.addWidget(self.seed,8,1)
        self.note=QLabel();self.note.setWordWrap(True);root.addWidget(self.note)
        self.status=QLabel('Generates a draft only. Review the simulation, approve, then explicitly start on hardware.');self.status.setWordWrap(True);root.addWidget(self.status)
        self.mode.currentIndexChanged.connect(self.update_mode);self.update_mode()
        buttons=QDialogButtonBox(QDialogButtonBox.Ok|QDialogButtonBox.Cancel);buttons.button(QDialogButtonBox.Ok).setText('Generate draft')
        buttons.accepted.connect(self.generate);buttons.rejected.connect(self.reject);root.addWidget(buttons)

    def update_mode(self):
        diff=self.mode.currentText()=='Difference';mixed=self.mode.currentText()=='Mixed'
        for pair in self.bounds:
            for box in pair:box.setEnabled(not diff)
        for box in (self.dlow,self.dhigh,self.mean):box.setEnabled(diff or mixed)
        self.linked.setEnabled(not diff)
        self.count.setMinimum(4 if mixed else 2)
        self.note.setText('Mixed includes difference, cable 1 only, cable 2 only and independent two-cable moves. '
            'It starts with both cables inside the selected ranges; every four poses cover all four modes. '
            'Difference targets must also fit both cable ranges.' if mixed else 'One seeded random target per measurement pose. The final return is added to the reviewed journey.')

    def generate(self):
        try:
            ranges=[[self.dlow.value(),self.dhigh.value()]] if self.mode.currentText()=='Difference' else [[b.value() for b in pair] for pair in self.bounds]
            self.rows=random_plan(self.profile,self.mode.currentText(),ranges,[self.slow.value(),self.shigh.value()],
                self.count.value(),self.seed.value(),mean=self.mean.value(),linked=self.linked.isChecked(),
                difference_range=[self.dlow.value(),self.dhigh.value()])
            self.accept()
        except Exception as exc:self.status.setText(str(exc))


class DifferenceSweep(QDialog):
    def __init__(self,parent,profile):
        super().__init__(parent);self.profile=copy.deepcopy(profile);self.rows=None
        self.setWindowTitle('Difference sweep — create draft only');self.resize(540,340)
        root=QVBoxLayout(self);grid=QGridLayout();root.addLayout(grid)
        cap=profile['difference_mm'];self.mean=number(sum(profile['rest_lengths_mm'])/2,.001,10000)
        lo,hi=mixed_difference_range(profile,profile['absolute_bounds_mm'],self.mean.value())
        self.start=number(max(lo,-40),-cap,cap);self.end=number(min(hi,40),-cap,cap)
        self.speed=number(min(1,profile['max_speed_mm_s']),.05,profile['max_speed_mm_s'])
        self.intervals=QSpinBox();self.intervals.setRange(1,100);self.intervals.setValue(15)
        for i,(label,box) in enumerate([('Start extreme D [mm]',self.start),('End extreme D [mm]',self.end),
            ('Fixed mean [mm]',self.mean),('Speed [mm/s]',self.speed),('Intervals between extremes',self.intervals)]):
            grid.addWidget(QLabel(label),i,0);grid.addWidget(box,i,1)
        self.status=QLabel();self.status.setWordWrap(True);root.addWidget(self.status)
        self.intervals.valueChanged.connect(self.describe);self.describe()
        buttons=QDialogButtonBox(QDialogButtonBox.Ok|QDialogButtonBox.Cancel);buttons.button(QDialogButtonBox.Ok).setText('Generate sweep')
        buttons.accepted.connect(self.generate);buttons.rejected.connect(self.reject);root.addWidget(buttons)

    def describe(self):
        self.status.setText(f'{self.intervals.value()+1} equally spaced measurement poses, including both extremes, plus one return capture. '
            'Reverse the two extremes to change direction. Simulate and approve before starting.')

    def generate(self):
        try:
            self.rows=difference_sweep(self.profile,self.start.value(),self.end.value(),self.mean.value(),self.speed.value(),self.intervals.value())
            self.accept()
        except Exception as exc:self.status.setText(str(exc))


class CampaignPreview(QDialog):
    def __init__(self,parent,plan,xml,context,approved):
        super().__init__(parent);self.plan=copy.deepcopy(plan);self.context=copy.deepcopy(context);self.approved=approved
        self.result=None;self.review=None;self.cancel=threading.Event();self.messages=queue.Queue();self.closed=False
        self.setWindowTitle('MuJoCo campaign preview — no hardware commands');self.resize(1030,730)
        root=QVBoxLayout(self);self.status=QLabel('Simulating at the model timestep. Cancel remains available.');self.status.setWordWrap(True);root.addWidget(self.status)
        self.figure=Figure(figsize=(9,4.4),layout='constrained');self.canvas=FigureCanvasQTAgg(self.figure)
        root.addWidget(NavigationToolbar2QT(self.canvas,self));root.addWidget(self.canvas,1)
        self.slider=QSlider(Qt.Horizontal);self.slider.setEnabled(False);self.slider.valueChanged.connect(self.draw);root.addWidget(self.slider)
        row=QHBoxLayout();root.addLayout(row)
        self.play=QPushButton('Play');self.play.setCheckable(True);self.play.setEnabled(False);row.addWidget(self.play)
        row.addWidget(QLabel('Playback ×'));self.rate=number(5,.25,100);row.addWidget(self.rate)
        self.cancel_button=QPushButton('STOP / cancel');self.cancel_button.clicked.connect(self.stop);self.cancel_button.setStyleSheet('background:#a32121;color:white;font-weight:bold');row.addWidget(self.cancel_button)
        self.approve_button=QPushButton('Approve this simulated plan');self.approve_button.setEnabled(False);self.approve_button.clicked.connect(self.approve);row.addWidget(self.approve_button)
        note=QLabel('Review all poses with the slider or playback. Lines show commanded physical lengths and simulated tendon lengths. '
                    'The final step returns to the starting cable lengths at the selected return speed. '
                    'Simulation starts from a conditioned model pose; motor ramps, backlash and actual loading can differ. Approval never starts motors.')
        note.setWordWrap(True);root.addWidget(note)
        def work():
            try:
                result=simulate(self.plan,xml,self.context['parameters'],self.context['d2'],self.context['setup'],self.cancel,
                    lambda fraction,pose,total:self.messages.put(('progress',f'Simulating {100*fraction:.1f}% · pose {pose}/{total}. No hardware commands.')))
                self.messages.put(('complete',result))
            except Exception as exc:self.messages.put(('error',str(exc)))
        self.worker=threading.Thread(target=work,daemon=True);self.worker.start()
        self.last_tick=time.monotonic();self.timer=QTimer(self);self.timer.timeout.connect(self.poll);self.timer.start(100)

    def stop(self):
        self.cancel.set()
        host=self.parent().host
        if host.session:host.command('hold')

    def poll(self):
        now=time.monotonic();elapsed=now-self.last_tick;self.last_tick=now
        while not self.messages.empty():
            kind,value=self.messages.get_nowait()
            if kind=='progress':self.status.setText(value)
            elif kind=='error':self.status.setText(value)
            elif kind=='complete':
                if self.cancel.is_set():self.status.setText('Simulation cancelled.');continue
                self.result=value;s=value['summary']
                self.status.setStyleSheet('color:#a32121' if s['max_contacts'] else '')
                self.status.setText(f"{s['joints']} joints · {len(s['poses'])} poses. Observed ten_length: "
                    f"cable 1 {s['tendon_min_mm'][0]:.2f}–{s['tendon_max_mm'][0]:.2f} mm; "
                    f"cable 2 {s['tendon_min_mm'][1]:.2f}–{s['tendon_max_mm'][1]:.2f} mm. "
                    f"Maximum contacts: {s['max_contacts']}. These are observed simulation ranges, not hardware limits."+
                    (' Contact occurred: the current free-space k₂ solver cannot fit affected poses.' if s['max_contacts'] else ''))
                self.slider.setRange(0,len(value['frames'])-1);self.slider.setEnabled(True);self.play.setEnabled(True)
                self.approve_button.setEnabled(True);self.prepare_plot();self.draw()
        if self.result and self.play.isChecked():
            frames=self.result['frames'];current=frames[self.slider.value()]['t_s']
            target=current+elapsed*self.rate.value();times=[f['t_s'] for f in frames]
            index=min(len(frames)-1,max(self.slider.value()+1,int(np.searchsorted(times,target))))
            self.slider.setValue(index)
            if index==len(frames)-1:self.play.setChecked(False)

    def prepare_plot(self):
        self.figure.clear();self.pose_ax,self.length_ax=self.figure.subplots(1,2)
        curves=np.array([f['curve_mm'] for f in self.result['frames']]);lo=curves.min(axis=(0,1))-10;hi=curves.max(axis=(0,1))+10
        self.line,=self.pose_ax.plot([],[],'o-',color='#ee7b22',ms=3)
        self.pose_ax.set(xlim=(lo[0],hi[0]),ylim=(hi[1],lo[1]),xlabel='Lateral [mm]',ylabel='Along neutral backbone [mm]')
        self.pose_ax.set_aspect('equal',adjustable='box')
        frames=self.result['frames'];times=[f['t_s'] for f in frames]
        mapping=self.context['setup'].get('mapping',[0,1])
        for j,color in enumerate(('#087e8b','#9b3c8a')):
            self.length_ax.plot(times,[f['lengths_mm'][mapping[j]] for f in frames],color=color,label=f'Physical command {mapping[j]+1}')
            self.length_ax.plot(times,[f['tendon_mm'][j] for f in frames],'--',color=color,label=f'Model tendon {j+1}')
        self.marker=self.length_ax.axvline(0,color='#555');self.length_ax.set(xlabel='Journey time [s]',ylabel='Length [mm]');self.length_ax.legend(fontsize=8)

    def draw(self,*_):
        if not self.result:return
        frame=self.result['frames'][self.slider.value()];points=np.asarray(frame['curve_mm'])
        pose=self.plan['poses'][frame['pose']-1]
        title='Return to start' if pose.get('role')=='return' else f"Pose {frame['pose']} · {pose['mode']}"
        self.line.set_data(points[:,0],points[:,1]);self.pose_ax.set_title(f"{title}\n{frame['t_s']:.2f} s")
        self.marker.set_xdata([frame['t_s']]*2);self.canvas.draw_idle()

    def approve(self):
        if not self.result or self.closed:return
        self.review=dict(fingerprint=digest(self.context),context=copy.deepcopy(self.context),plan=copy.deepcopy(self.plan),
                         summary=copy.deepcopy(self.result['summary']))
        self.approved(self.review);self.approve_button.setEnabled(False);self.approve_button.setText('Approved')

    def reject(self):
        self.cancel.set();self.closed=True;self.timer.stop();super().reject()

    def closeEvent(self,event):
        self.cancel.set();self.closed=True;self.timer.stop();event.accept()
