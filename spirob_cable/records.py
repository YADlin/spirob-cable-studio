"""Timestamp every individual serial sample; never pretend two reads are simultaneous."""
import csv
from datetime import datetime, timezone
from importlib.metadata import version
import json
from pathlib import Path
import platform
import time
from . import __version__

COLUMNS = ['axis', 'sample_index', 'read_start_s', 'read_end_s', 'state', 'count',
           'connection_count', 'rest_count', 'rest_length_mm', 'reference_epoch',
           'target_count', 'connection_mm', 'target_connection_mm', 'offset_mm',
           'target_mm', 'absolute_mm', 'target_absolute_mm', 'error_mm',
           'motor_rpm_command', 'velocity_mm_s', 'read_duration_s']


class Recorder:
    def __init__(self, root, settings, demo, publish=lambda *a, **k: None, name=''):
        self.path = Path(root)/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+('_DEMO' if demo else '_HARDWARE'))
        self.path.mkdir(parents=True)
        self.started = time.monotonic()
        self.publish, self.previous, self.indices = publish, {}, {}
        meta = {'schema_version': 4, 'software_version': __version__, 'demo': demo,
                'created_utc': datetime.now(timezone.utc).isoformat(), 'name': name,
                'configuration': settings.data(), 'python': platform.python_version(),
                'dependencies': {p: version(p) for p in ('PySide6','pyqtgraph','numpy','pymodbus','pyserial')},
                'coordinates': 'A=(L1+L2)/2; D=(L1-L2)/2; L1=A+D; L2=A-D; all log lengths mm',
                'timing': 'Sequential per-axis host acquisition times; no simultaneous start/measurement guarantee',
                'transport': {'framer':'ASCII','baud':9600,'data_bits':8,'parity':'N','stop_bits':1,'timeout_s':0.3,'retries':0}}
        (self.path/'session.json').write_text(json.dumps(meta, indent=2)+'\n')
        self.events = (self.path/'events.jsonl').open('x')
        self.samples = (self.path/'samples.csv').open('x', newline='')
        self.writer = csv.DictWriter(self.samples, fieldnames=COLUMNS)
        self.writer.writeheader(); self.samples.flush()
        self.pairs = (self.path/'pairs.csv').open('x', newline='')
        self.pair_writer = csv.DictWriter(self.pairs, fieldnames=['host_s','c1_read_end_s','c2_read_end_s','skew_s','mean_mm','half_difference_mm'])
        self.pair_writer.writeheader(); self.pairs.flush()

    def event(self, event, **data):
        value = {'t_s':time.monotonic()-self.started, 'event':event, **data}
        self.events.write(json.dumps(value, allow_nan=False)+'\n'); self.events.flush()
        self.publish('event', data=value)

    def sample(self, axis, row):
        self.indices[axis] = self.indices.get(axis, 0)+1
        row = dict(row, axis=axis, sample_index=self.indices[axis])
        row['read_start_s'] -= self.started; row['read_end_s'] -= self.started
        prev = self.previous.get(axis)
        row['velocity_mm_s'] = None
        if prev:
            dt = row['read_end_s']-prev['read_end_s']
            if 0 < dt <= 2:
                row['velocity_mm_s'] = (row['connection_mm']-prev['connection_mm'])/dt
        self.writer.writerow({k:row.get(k) for k in COLUMNS}); self.samples.flush()
        self.previous[axis] = row
        self.publish('sample', data=row)

    def pair(self, coords):
        if coords is None:
            return
        self.pair_writer.writerow({'host_s':time.monotonic()-self.started,
             'c1_read_end_s':coords['times'][0]-self.started, 'c2_read_end_s':coords['times'][1]-self.started,
             'skew_s':coords['skew_s'], 'mean_mm':coords['mean_mm'], 'half_difference_mm':coords['half_difference_mm']})
        self.pairs.flush()

    def close(self):
        for stream in (self.events, self.samples, self.pairs):
            stream.close()


class AxisLog:
    def __init__(self, recorder, axis):
        self.recorder, self.axis = recorder, axis
    def event(self, event, **data):
        self.recorder.event(event, axis=self.axis, **data)


class NullRecorder:
    path = 'test'
    def event(self, *args, **kwargs): pass
    def sample(self, *args, **kwargs): pass
    def pair(self, *args, **kwargs): pass
    def close(self): pass
