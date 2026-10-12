import argparse
import csv
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys
import time
import mujoco
import numpy as np
from .model import SpiRobModel
from .session import Session


def resolve_config(path):
    path = Path(path).resolve()
    config = json.loads(path.read_text())
    return config, path.parent / config['model'], path.parent / config['session']


def main():
    parser = argparse.ArgumentParser(description='Offline SpiRob analysis; joint 2 is the flexible reference.')
    parser.add_argument('--config', default='config.json')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('inspect')
    p = sub.add_parser('export')
    p.add_argument('--stiffness2', type=float, required=True)
    p.add_argument('--damping2', type=float)
    p.add_argument('--out', default='custom_model')
    for name in ['demo', 'extract', 'fit']:
        p = sub.add_parser(name)
        p.add_argument('--out', default='results_local')
        if name == 'fit':
            p.add_argument('--observations', required=True)
    p = sub.add_parser('view')
    p.add_argument('--fit', default='results_local/fit.json')
    p.add_argument('--step', type=int, default=2)
    p.add_argument('--stiffness2', type=float)
    p.add_argument('--damping2', type=float)
    p = sub.add_parser('replay')
    p.add_argument('--fit', default='results_local/fit.json')
    p.add_argument('--out', default='results_local/replay.csv')
    p.add_argument('--damping2', type=float)
    p.add_argument('--sample-hz', type=float, default=30.)
    args = parser.parse_args()
    config, xml, session_path = resolve_config(args.config)
    model = SpiRobModel(xml)
    if args.command == 'inspect':
        print(json.dumps(model.audit(), indent=2))
        return
    if args.command == 'export':
        path = model.export(args.out, args.stiffness2, args.damping2)
        print(f'Exported {path.resolve()}; joint 1 preserved; joints 2 onward scaled from joint 2.')
        return
    session = Session(session_path)
    if args.command in ('demo', 'extract', 'fit'):
        from .vision import extract
        from .fitting import fit
        from .report import report
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        if args.command == 'fit':
            observations = json.loads(Path(args.observations).read_text())
            image_dir = Path(args.observations).parent
        else:
            print('Extracting held-pose image curves...', flush=True)
            observations = extract(session, config, out)
            image_dir = out
        if args.command != 'extract':
            print('Fitting joint-2 stiffness with fixed exponential ratios...', flush=True)
            result = fit(model, observations, config, out)
            report(result, observations, out, image_dir)
            print(f"Uploaded k2: {model.k2:.8g}; candidate k2: {result['candidate']['parameters']['k2']:.8g} N m/rad")
            print(f"Point RMS: {result['baseline']['point_rmse_px']:.2f} -> {result['candidate']['point_rmse_px']:.2f} px")
            print('Candidate is conditional on the cable-zero assumption; inspect the sensitivity plot.')
            print(f'Report: {(out / "report.html").resolve()}')
        provenance = {'python': sys.version, 'model_sha256': hashlib.sha256(xml.read_bytes()).hexdigest(),
                      'session_sha256': hashlib.sha256(session_path.read_bytes()).hexdigest() if session_path.is_file() else None,
                      'config': config,
                      'versions': {n: importlib.metadata.version(n) for n in ['mujoco','numpy','scipy','Pillow','opencv-python-headless','scikit-image']}}
        (out / 'provenance.json').write_text(json.dumps(provenance, indent=2))
        return
    result = json.loads(Path(args.fit).read_text())
    p = result['candidate']['parameters']
    if args.command == 'view':
        from mujoco import viewer as mjviewer
        poses = [x for x in result['candidate']['poses'] if x['step'] == args.step]
        if len(poses) != 1:
            raise ValueError('Step not found in fitted poses.')
        pose = poses[0]
        k2 = p['k2'] if args.stiffness2 is None else args.stiffness2
        state = model.equilibrium(pose['controls_m'], k2)
        model.scale(k2, args.damping2)
        model.d.qpos[:] = state['qpos']
        model.d.ctrl[:] = pose['controls_m']
        mujoco.mj_forward(model.m, model.d)
        with mjviewer.launch_passive(model.m, model.d) as viewer:
            viewer.cam.lookat[:] = [0, .12, .09]
            viewer.cam.distance = .42
            viewer.cam.azimuth = 90
            viewer.cam.elevation = -90
            while viewer.is_running():
                start = time.monotonic()
                mujoco.mj_step(model.m, model.d, nstep=100)
                viewer.sync()
                time.sleep(max(0, .01-(time.monotonic()-start)))
        return
    series = session.motor_series()
    start = max(t[0] for t, _ in series)
    stop = min(t[-1] for t, _ in series)
    if args.sample_hz <= 0:
        raise ValueError('--sample-hz must be positive.')
    control_period = .01  # Interpolate measured cable inputs at 100 Hz.
    nstep = max(1, round(control_period / model.m.opt.timestep))
    period = nstep * model.m.opt.timestep
    length = [np.interp(start, t, x) for t, x in series]
    model.equilibrium(model.controls(length, p['mean_offset_mm'], p['difference_offset_mm']), p['k2'])
    model.scale(p['k2'], args.damping2)
    model.d.time = 0
    path = Path(args.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    next_sample = 0.
    max_contacts = 0
    with path.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['coordinator_t_s','simulation_t_s','input_0_m','input_1_m','tip_x_m','tip_y_m','tip_z_m',
                         'max_joint_speed_rad_s','contacts'] + [f'q_{i+1:03d}_rad' for i in range(model.m.nq)])
        while model.d.time <= stop-start:
            t_now = start + model.d.time
            length = [np.interp(t_now, t, x) for t, x in series]
            model.d.ctrl[:] = model.controls(length, p['mean_offset_mm'], p['difference_offset_mm'])
            mujoco.mj_forward(model.m, model.d)
            max_contacts = max(max_contacts, model.d.ncon)
            if model.d.time + 1e-10 >= next_sample:
                writer.writerow([t_now, model.d.time, *model.d.ctrl, *model.d.site('tip_site').xpos,
                                 np.max(np.abs(model.d.qvel)), model.d.ncon, *model.d.qpos])
                next_sample += 1/args.sample_hz
            before = model.d.time
            mujoco.mj_step(model.m, model.d, nstep=nstep)
            if model.d.time <= before or not np.all(np.isfinite(model.d.qpos)) or np.any(model.d.warning.number):
                raise RuntimeError('Simulation warning or state reset during replay; CSV is incomplete.')
    metadata = {'status': 'COMPLETE', 'k2': p['k2'], 'd2': model.d2 if args.damping2 is None else args.damping2,
                'damping_fitted': False, 'input': 'interpolated encoder-derived cable displacement',
                'control_update_period_s': period, 'max_contacts': max_contacts}
    path.with_suffix('.json').write_text(json.dumps(metadata, indent=2))
    print(f'Replay complete: {path.resolve()} (damping has not been fitted)')


if __name__ == '__main__':
    main()
