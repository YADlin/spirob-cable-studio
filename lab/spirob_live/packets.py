"""Windows side: read completed files; never renew the coordinator's camera lease.

Only Python's standard library is used here. The acquisition code stays unchanged.
All t_s values are seconds after the WSL coordinator started this session.
"""
import base64
import csv
import io
import json
from pathlib import Path
import time


def csv_tail(path, limit=262144):
    """Read a bounded tail and discard a partially written final row."""
    with Path(path).open('rb') as f:
        header = f.readline().decode('utf-8-sig').strip()
        start = f.tell()
        f.seek(0, 2)
        offset = max(start, f.tell() - limit)
        f.seek(offset)
        data = f.read(limit)
    if offset > start:
        data = data.partition(b'\n')[2]
    if not data.endswith(b'\n'):
        data = data.rpartition(b'\n')[0] + b'\n'
    return list(csv.DictReader(io.StringIO(header + '\n' + data.decode('utf-8'))))


def inside(root, relative):
    path = (Path(root) / relative).resolve()
    if not path.is_relative_to(Path(root).resolve()):
        raise ValueError('Log path leaves session directory')
    return path


def make_packet(capture, now=None):
    if capture is None:
        return {'state': 'WAITING', 'message': 'Camera service ready; start a trial or camera check.'}
    status = capture.status(renew=False)
    session = capture.path.parent
    result = {'state': status['state'], 'session': capture.session_id,
              'message': 'Waiting for recorded image and referenced motor samples.'}
    now = time.monotonic() if now is None else now
    frames, run = capture.snapshot()
    if run is None:
        # Preview is available before the motor is connected. No shared run.json reads.
        if not frames: return result
        latest_frame = frames[-1]
        result.update(image_b64=base64.b64encode(latest_frame['jpeg']).decode('ascii'),
            frame_index=latest_frame['frame_index'], t_s=latest_frame['read_end_s']-capture.started, motor=[], motor_active=False,
            motor_age_s=[None,None], frame_age_s=now-latest_frame['read_end_s'],
            phase='CAMERA READY', clock_uncertainty_s=None, rest_lengths_mm=[220.,220.])
        return result
    if not frames:
        return result
    offset = run['clock_before']['best']['offset_s']
    origin = run['coordinator_monotonic_s']
    motor = []
    states = [None, None]
    if 'motor_log' in run and 'motor_origin_s' in run:
        rows = csv_tail(inside(session, run['motor_log']) / 'samples.csv')
        for row in rows:
            states[int(row['axis'])] = row['state']
            if row['absolute_mm'] and row['state'] in ('HOLDING', 'MOVING'):
                motor.append({'axis': int(row['axis']),
                    't_s': run['motor_origin_s'] + (float(row['read_start_s']) + float(row['read_end_s'])) / 2,
                    'length_mm': float(row['absolute_mm']),
                    'target_mm': float(row['target_absolute_mm']) if row['target_absolute_mm'] else None,
                    'state': row['state'],
                    'velocity_mm_s': float(row['velocity_mm_s']) if row['velocity_mm_s'] else None})
    latest = [max((r['t_s'] for r in motor if r['axis'] == axis), default=None) for axis in (0, 1)]
    def frame_t(row):
        return float(row['read_end_s']) - offset - origin
    # Pick a frame bracketed by both encoder streams rather than extrapolating.
    eligible = [f for f in frames if all(t is not None and frame_t(f) <= t for t in latest)]
    feedback_fresh = all(v is not None and now-(origin+offset+v)<.65 for v in latest)
    chosen = eligible[-1] if eligible and feedback_fresh else frames[-1]
    t = frame_t(chosen)
    image = chosen['jpeg']
    if len(image) > 8 * 1024 * 1024:
        raise ValueError('Preview image exceeds the 8 MiB limit')
    result.update(phase=('HOLDING / MOVING' if all(s in ('HOLDING','MOVING') for s in states) else 'MOTORS DISABLED'), frame_index=int(chosen['frame_index']), t_s=t,
        image_b64=base64.b64encode(image).decode('ascii'),
        frame_age_s=now - float(chosen['read_end_s']),
        motor_age_s=[None if v is None else now - (origin + offset + v) for v in latest],
        motor=[r for r in motor if r['t_s'] >= t-8],
        motor_active=all(state in ('HOLDING','MOVING') for state in states), motor_states=states,
        rest_lengths_mm=run['trial']['rest_lengths_mm'],
        clock_uncertainty_s=run['clock_before']['network_uncertainty_s'],
        timing='Periodically refreshed host clock offset; exposure and buffering delay unmeasured.',
        message='Memory-buffered JPEG + encoder lengths; observer does not control hardware.')
    return result
