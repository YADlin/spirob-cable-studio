"""Does the Lab model XML describe the rig as it is built?

The physical SpiRob swings in a horizontal plane (global XY) with gravity along −Z,
its base clamped about 9 cm above the base plate. A replacement XML must keep that
set-up, otherwise gravity acts as a fake bending stiffness and every fit is wrong
(this happened with the 13-Oct XML before its robot_base wrapper was added).

Run:  python model_check.py [path/to/spirob_robot.xml]
"""
import json
import sys
from pathlib import Path
import numpy as np
import mujoco

DEFAULTS = dict(plane_height_m=.09, height_tolerance_m=.005, flatness_tolerance_m=.001)


def check_model(xml, settings=None):
    """Return (ok, lines). Each line is 'OK …' or 'FAIL …'."""
    s = dict(DEFAULTS, **(settings or {}))
    lines = []
    def item(ok, text):
        lines.append(('OK   ' if ok else 'FAIL ') + text)
        return ok
    try:
        m = mujoco.MjModel.from_xml_path(str(xml))
    except Exception as exc:
        return False, ['FAIL model does not load: ' + str(exc)]
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    ok = True
    g = m.opt.gravity
    ok &= item(np.allclose(g, [0, 0, -9.81], atol=.05), f'gravity points down (−Z): {np.round(g, 3).tolist()} m/s²')
    names = [m.joint(i).name for i in range(m.njnt)]
    expected = [f'j_{i:03d}' for i in range(1, m.njnt + 1)]
    ok &= item(names == expected and np.all(m.jnt_type == int(mujoco.mjtJoint.mjJNT_HINGE)),
               f'{m.njnt} ordered hinge joints j_001 … j_{m.njnt:03d}')
    ok &= item(m.nu == 2, f'two cable actuators (found {m.nu})')
    axes = d.xaxis[:m.njnt]
    vertical = np.abs(axes @ [0, 0, 1])
    ok &= item(np.all(vertical > .999), f'all hinge axes vertical, so bending is in the horizontal plane (min |axis·Z| = {vertical.min():.4f})')
    points = np.vstack([d.xanchor[:m.njnt], d.site('tip_site').xpos]) if m.nsite and 'tip_site' in [m.site(i).name for i in range(m.nsite)] else d.xanchor[:m.njnt]
    spread = float(points[:, 2].max() - points[:, 2].min())
    ok &= item(spread <= s['flatness_tolerance_m'], f'robot lies flat at rest: joint/tip heights vary by {spread*1000:.2f} mm (limit {s["flatness_tolerance_m"]*1000:.1f} mm)')
    height = float(np.mean(points[:, 2]))
    ok &= item(abs(height - s['plane_height_m']) <= s['height_tolerance_m'],
               f'bending plane height {height*1000:.1f} mm (expected {s["plane_height_m"]*1000:.0f} ± {s["height_tolerance_m"]*1000:.0f} mm)')
    length = float(np.linalg.norm(points[-1, :2] - points[0, :2]))
    ok &= item(length > .05, f'base-to-tip span {length*1000:.1f} mm in the horizontal plane')
    k = m.jnt_stiffness
    ok &= item(k[0] >= 10 * k[1:].max(), f'joint 1 is the stiff base clamp (k₁ = {k[0]:g}, largest other k = {k[1:].max():.4g})')
    return bool(ok), lines


def check_config(config_path='config.json'):
    config = json.loads(Path(config_path).read_text())
    return check_model(Path(config_path).parent / config['model'], config.get('model_check'))


if __name__ == '__main__':
    if len(sys.argv) > 1:
        cfg = json.loads(Path('config.json').read_text()) if Path('config.json').is_file() else {}
        ok, lines = check_model(sys.argv[1], cfg.get('model_check'))
    else:
        ok, lines = check_config()
    print('\n'.join(lines))
    print('Model matches the rig set-up.' if ok else 'Model does NOT match the rig set-up; fix the XML before fitting or training.')
    sys.exit(0 if ok else 1)
