"""Explicit measured cable references and per-run bounds in physical millimetres."""
import copy
import json
import subprocess
import numpy as np
from PySide6.QtWidgets import QDialog,QVBoxLayout,QGridLayout,QLabel,QDoubleSpinBox,QPushButton,QDialogButtonBox
from motion_profile import validate_profile


def model_bounds(model, rests, setup=None, parameters=None):
    setup=setup or {};parameters=parameters or {}
    mapping=setup.get('mapping',[0,1]);signs=setup.get('signs',[1,1])
    mean=parameters.get('mean_offset_mm',0.);diff=parameters.get('difference_offset_mm',0.)
    offset=[mean+diff/2,mean-diff/2];bounds=[None,None]
    for j,physical in enumerate(mapping):
        values=rests[physical]+signs[j]*(1000*(model.m.actuator_ctrlrange[j]-model.neutral_lengths[j])-offset[j])
        bounds[physical]=sorted(values.tolist())
    return bounds


def model_profile(model):
    rests=(1000*model.neutral_lengths).tolist()
    return validate_profile(dict(rest_lengths_mm=rests,absolute_bounds_mm=model_bounds(model,rests),
        max_speed_mm_s=20.,setup_travel_mm=150.,difference_mm=150.))


class MotionSettings(QDialog):
    def __init__(self,host):
        super().__init__(host);self.host=host;self.profile=None
        from spirob_analysis.model import SpiRobModel
        self.model=SpiRobModel(host.config['model'])
        self.setWindowTitle('Cable references and motion limits — next connection');self.resize(690,430)
        layout=QVBoxLayout(self);grid=QGridLayout();layout.addLayout(grid)
        self.fields=[]
        for col,text in enumerate(('Cable','Physical rest / reference [mm]','Minimum [mm]','Maximum [mm]')):grid.addWidget(QLabel(text),0,col)
        p=host.motion_profile
        for i in range(2):
            grid.addWidget(QLabel(str(i+1)),i+1,0);boxes=[]
            for col,v in enumerate([p['rest_lengths_mm'][i],*p['absolute_bounds_mm'][i]],1):
                box=QDoubleSpinBox();box.setDecimals(6);box.setRange(.001,10000);box.setValue(v);grid.addWidget(box,i+1,col);boxes.append(box)
            self.fields.append(boxes)
        self.speed=QDoubleSpinBox();self.speed.setRange(.05,20);self.speed.setValue(p['max_speed_mm_s'])
        self.setup=QDoubleSpinBox();self.setup.setRange(.5,1000);self.setup.setValue(p['setup_travel_mm'])
        grid.addWidget(QLabel('Speed ceiling [mm/s]'),3,0,1,2);grid.addWidget(self.speed,3,2)
        grid.addWidget(QLabel('Unreferenced setup travel ± [mm]'),4,0,1,2);grid.addWidget(self.setup,4,2)
        note=QLabel(f"Model: {self.model.m.njnt} joints. Neutral tendon lengths: {1000*self.model.neutral_lengths[0]:.6f}, {1000*self.model.neutral_lengths[1]:.6f} mm.\n"
            'Position-command bounds are 150–300 mm in the model. They are not measured hardware travel ratings.\n'
            'Enter the physical cable lengths at the pose you will declare as rest. These GUI values override WSL config for this session; the WSL file is not modified. Limits lock at Connect.')
        note.setWordWrap(True);layout.addWidget(note)
        for title,handler in [('Use model neutral references + bounds',self.model_defaults),('Map model bounds to entered physical references',self.map_bounds),('Read reference lengths from WSL config',self.remote_references)]:
            b=QPushButton(title);b.clicked.connect(handler);layout.addWidget(b)
        self.status=QLabel('');self.status.setWordWrap(True);layout.addWidget(self.status)
        buttons=QDialogButtonBox(QDialogButtonBox.Save|QDialogButtonBox.Cancel);buttons.accepted.connect(self.save);buttons.rejected.connect(self.reject);layout.addWidget(buttons)

    def model_defaults(self):
        for row,value in zip(self.fields,1000*self.model.neutral_lengths):row[0].setValue(float(value))
        self.map_bounds()

    def map_bounds(self):
        bounds=model_bounds(self.model,[r[0].value() for r in self.fields],self.host.comparison_setup,self.host.fit['candidate']['parameters'])
        for row,pair in zip(self.fields,bounds):
            for box,value in zip(row[1:],pair):box.setValue(value)

    def remote_references(self):
        if not self.host.rig:self.status.setText('No hardware rig configured.');return
        from connection import wsl_command
        try:
            result=subprocess.run(['ssh','-T','-o','ConnectTimeout=5',self.host.rig['ssh_host'],wsl_command(self.host.rig,'actuation.py','--check-only')],capture_output=True,text=True,timeout=12,check=True)
            data=json.loads(result.stdout.strip().splitlines()[-1])
            for row,a in zip(self.fields,data['axes']):row[0].setValue(a['rest_length_mm'])
            self.map_bounds();self.status.setText('Loaded WSL references. Review and Save to use at the next connection.')
        except Exception as exc:self.status.setText('Could not read WSL configuration: '+str(exc))

    def save(self):
        try:
            p=dict(rest_lengths_mm=[r[0].value() for r in self.fields],absolute_bounds_mm=[[b.value() for b in r[1:]] for r in self.fields],
                max_speed_mm_s=self.speed.value(),setup_travel_mm=self.setup.value(),difference_mm=self.host.difference_limit.value())
            self.profile=validate_profile(p);self.accept()
        except Exception as exc:self.status.setText(str(exc))
