"""Marker-dot measurement of saved campaign photos (E1). No hardware is used.

For each included pose the dots are measured from the retained frame.jpg.
* Poses without points receive the dot observation (landmark_mode='centres').
* Poses that already have manual points keep them unless replace=True; the dot
  result is saved beside them as dots.json for comparison.
A review image dots_overlay.jpg is written in every pose folder and a summary
dots_summary.csv (dot vs manual element-centre distance) in the campaign folder.
"""
import copy
import csv
import json
import numpy as np
from PIL import Image
from analysis import Comparator, inputs_at
from spirob_analysis.model import SpiRobModel
from dot_tracking import measure_dots, expected_spacing_px, draw
from campaign_storage import atomic, encoded


class DotsCancelled(Exception):
    pass


def manual_centres(observation):
    """Midpoints of consecutive manual joint points → {'link_NNN': [x, y]}."""
    ids = observation.get('landmark_ids') or []
    if observation.get('landmark_mode') != 'joints' or not ids:
        return {}
    pts = dict(zip(ids, observation['points_px']))
    order = [f'j_{i:03d}' for i in range(2, 22)] + ['tip']
    out = {}
    for a, b in zip(order[:-1], order[1:]):
        if a in pts and b in pts:
            out['link_' + a[-3:]] = ((np.array(pts[a]) + pts[b]) / 2).tolist()
    return out


def measure_campaign_dots(store, replace=False, progress=None, cancel=None):
    store.verify_model()
    config = store.config()
    engine = Comparator(config, store.doc['fit'])
    spacing = expected_spacing_px(SpiRobModel(config['model']), engine.p['pixels_per_m'])
    rows, base_hint = [], config.get('marker_base_px')
    poses = list(enumerate(store.doc['poses']))
    for n, (i, pose) in enumerate(poses):
        if cancel is not None and cancel.is_set():
            raise DotsCancelled('Dot measurement stopped; poses already processed are saved.')
        if pose['excluded']:
            continue
        row = dict(pose=pose['number'], label=pose['command']['label'], links=0, missing='', result='')
        try:
            item = store.load_item(i)
            packet = item['packet']
            measured = measure_dots(item['rgb'], spacing, base_hint)
            base_hint = np.mean(measured['base_dots_px'], axis=0)
            lengths, _ = inputs_at(packet, packet['t_s'], config['live']['max_input_gap_s'])
            obs = engine.observation(measured, lengths, packet)
            obs.update(stationary_confirmed=False)
            folder = store.path(pose['path'])
            atomic(folder / 'dots.json', encoded(obs))
            Image.fromarray(draw(item['rgb'], measured)).save(folder / 'dots_overlay.jpg', quality=90)
            row.update(links=len(obs['landmark_ids']), missing=' '.join(m[-3:] for m in obs['missing_links']))
            manual = manual_centres(pose['observation'] or {})
            if manual:
                common = [k for k in obs['landmark_ids'] if k in manual]
                dots = dict(zip(obs['landmark_ids'], obs['points_px']))
                err = [float(np.linalg.norm(np.subtract(dots[k], manual[k]))) for k in common]
                row.update(compared=len(common), mean_px=float(np.mean(err)) if err else None,
                           max_px=float(np.max(err)) if err else None,
                           worst_link=common[int(np.argmax(err))] if err else '')
            if pose['observation'] is None or replace:
                store.set_observation(i, obs, 'Marker dots (automatic)')
                row['result'] = 'observation set from dots'
            else:
                row['result'] = 'manual points kept; dots saved for comparison'
        except (ValueError, RuntimeError, OSError) as exc:
            row['result'] = 'failed: ' + str(exc)
        rows.append(row)
        if progress:
            progress(n + 1, len(poses), 'Measuring marker dots')
    fields = ['pose', 'label', 'links', 'missing', 'compared', 'mean_px', 'max_px', 'worst_link', 'result']
    with (store.root / 'dots_summary.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in fields})
    return rows
