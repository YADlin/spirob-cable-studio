"""Offline session review. Importing or reading a run never accesses a motor."""
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import numpy as np
from .model import COUNTS_PER_REV, Rejected, finite


def measured_circumference(start, end, travel):
    start, end = finite(start, "Start count"), finite(end, "End count")
    travel = finite(travel, "Measured travel")
    if start != int(start) or end != int(end) or start == end or travel <= 0:
        raise Rejected("Use distinct integer counts and a positive measured travel")
    value = travel * COUNTS_PER_REV / abs(end-start)
    if not 1 <= value <= 1000:
        raise Rejected("Result is outside 1..1000 mm/rev; check measurement units")
    return value


@dataclass
class Run:
    path: Path
    metadata: dict
    time: np.ndarray
    actual: np.ndarray
    target: np.ndarray
    error: np.ndarray
    reference: np.ndarray

    def metrics(self):
        mask = np.isfinite(self.error)
        errors = self.error[mask]
        dt = np.diff(self.time)
        dt = dt[np.isfinite(dt) & (dt > 0)]
        return {"samples": len(self.time),
                "rmse_mm": float(np.sqrt(np.mean(errors**2))) if len(errors) else None,
                "peak_abs_error_mm": float(np.max(np.abs(errors))) if len(errors) else None,
                "median_rate_hz": float(1 / np.median(dt)) if len(dt) else None}


def load_run(path):
    path = Path(path)
    path = path / 'samples.csv' if path.is_dir() else path
    if path.stat().st_size > 100_000_000:
        raise Rejected("Review supports CSV files up to 100 MB; analyse larger runs offline")
    meta_path = path.parent / 'session.json'
    metadata = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    columns = {name: [] for name in ('time', 'actual', 'target', 'error', 'reference')}
    def number(value):
        if value in (None, ''):
            return math.nan
        return finite(value, 'CSV value')
    settings = metadata.get('settings', {})
    mm_rev = metadata.get('mm_per_rev_used')
    sign = settings.get('payout_sign')
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        if not {'read_end_s', 'offset_mm', 'rest_count'} <= set(reader.fieldnames or ()):
            raise Rejected('Choose a samples.csv from a cable-control session')
        for i, row in enumerate(reader):
            if i >= 500_000:
                raise Rejected('Review supports at most 500000 samples')
            t, actual, rest = number(row['read_end_s']), number(row['offset_mm']), number(row['rest_count'])
            if not math.isfinite(t) or (columns['time'] and t < columns['time'][-1]):
                raise Rejected('CSV has invalid or decreasing read timestamps')
            target = number(row.get('target_mm'))
            if math.isnan(target) and mm_rev and sign in (-1, 1):
                count = number(row.get('target_count'))
                target = sign * (count-rest) * finite(mm_rev, 'Calibration') / COUNTS_PER_REV
            columns['time'].append(t)
            columns['actual'].append(actual)
            columns['target'].append(target)
            columns['error'].append(target-actual)
            columns['reference'].append(rest)
    if not columns['time']:
        raise Rejected('Session has no samples')
    return Run(path, metadata, **{key: np.asarray(value, dtype=float) for key, value in columns.items()})


def plot_series(run, aligned=False):
    """Break the line whenever a new physical rest reference is established."""
    t, a, target = run.time.copy(), run.actual.copy(), run.target.copy()
    good = np.flatnonzero(np.isfinite(a))
    if aligned and len(good):
        t -= t[good[0]]
    changes = np.flatnonzero(run.reference[1:] != run.reference[:-1]) + 1
    a[changes] = np.nan
    target[changes] = np.nan
    return t, a, target
