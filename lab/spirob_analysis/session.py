"""Read existing experiment outputs without connecting to hardware."""
import csv
import io
import json
from pathlib import Path
import zipfile
import numpy as np


class Session:
    def __init__(self, source):
        self.source = Path(source).resolve()
        self.zip = zipfile.ZipFile(self.source) if self.source.is_file() else None
        if self.zip:
            candidates = [n for n in self.zip.namelist() if n.endswith('/run.json') or n == 'run.json']
            if len(candidates) != 1:
                raise ValueError("Expected one session root containing run.json.")
            self.prefix = candidates[0][:-len('run.json')]
        else:
            self.prefix = ''
        self.run = self.json('run.json')
        self.trial = self.json('trial.json')

    def read(self, relative):
        if self.zip:
            return self.zip.read(self.prefix + relative)
        return (self.source / relative).read_bytes()

    def json(self, relative):
        return json.loads(self.read(relative))

    def exists(self, relative):
        return self.prefix + relative in self.zip.namelist() if self.zip else (self.source / relative).is_file()

    def held_frame(self, step):
        # Review bundle full-resolution images were selected near the END of each hold.
        filename = f'review/hold_{step:03d}/full_resolution.jpg'
        if not self.exists(filename):
            raise ValueError(f"Missing {filename}; supply a hold-end review bundle for static fitting.")
        entries = [f for f in self.json('review/index.json')['frames'] if f['step'] == step]
        last = max(entries, key=lambda f: f['t_s'])
        return self.read(filename), {'file': filename, 'frame_index': last['frame_index'],
                                     't_s': last['t_s'], 'review_window_frames': len(entries)}

    def displacement_mm(self, step):
        # Steps are expanded across repetitions in the same order as the coordinator.
        steps = self.trial['steps'] * self.trial.get('repetitions', 1)
        target = steps[step - 1]
        return np.array([target['cable_1_mm'], target['cable_2_mm']]) - self.trial['rest_lengths_mm']

    def motor_series(self):
        rows = list(csv.DictReader(io.StringIO(self.read('derived/motor_inputs.csv').decode())))
        series = []
        for axis in [0, 1]:
            valid = [r for r in rows if int(r['axis']) == axis and r['actual_length_m']
                     and r['state'] in ('HOLDING', 'MOVING')]
            if not valid:
                raise ValueError('No referenced motor length data.')
            t = np.array([(float(r['read_start_s']) + float(r['read_end_s'])) / 2 for r in valid])
            length = np.array([float(r['actual_length_m']) * 1000 for r in valid])
            series.append((t, length - self.trial['rest_lengths_mm'][axis]))
        return series
