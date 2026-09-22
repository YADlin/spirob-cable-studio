"""Read long-form v3 recordings as well as earlier single-cable CSVs."""
import csv
import json
import math
from pathlib import Path
import numpy as np
from .analysis import Run, load_run
from .model import Rejected, finite


def load_runs(path, frame='connection'):
    path = Path(path); path = path/'samples.csv' if path.is_dir() else path
    if path.stat().st_size > 100_000_000:
        raise Rejected('Use CSV files up to 100 MB for interactive review')
    actual_field,target_field = {'connection':('connection_mm','target_connection_mm'),
              'rest':('offset_mm','target_mm'),'absolute':('absolute_mm','target_absolute_mm')}[frame]
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        if 'axis' not in (reader.fieldnames or []):
            return {'Cable 1 (legacy rest frame)':load_run(path)}
        if not {'read_end_s','axis',actual_field,target_field,'reference_epoch'} <= set(reader.fieldnames):
            raise Rejected('Not a supported two-cable samples.csv')
        meta_path = path.parent/'session.json'
        metadata = json.loads(meta_path.read_text()) if meta_path.exists() else {}
        data = {}
        def num(value): return math.nan if value in ('',None) else finite(value,'CSV value')
        for n,row in enumerate(reader):
            if n >= 500_000: raise Rejected('At most 500000 rows for interactive review')
            axis = int(row['axis'])
            if axis not in (0,1): raise Rejected('Invalid cable number in CSV')
            cols = data.setdefault(axis, {k:[] for k in ('time','actual','target','error','reference')})
            t = finite(row['read_end_s'],'Sample time')
            if cols['time'] and t < cols['time'][-1]: raise Rejected('Decreasing sample timestamps')
            actual,target = num(row[actual_field]),num(row[target_field])
            for key,val in zip(cols,(t,actual,target,target-actual,0 if frame=='connection' else num(row['reference_epoch']))):
                cols[key].append(val)
    if not data: raise Rejected('No samples in CSV')
    return {f'Cable {axis+1}':Run(path,metadata,**{k:np.asarray(v,dtype=float) for k,v in cols.items()}) for axis,cols in sorted(data.items())}
