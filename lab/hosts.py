"""Which machine owns the motors and camera: the laptop (Windows + WSL) or a Raspberry Pi.

Standard library only, with no Lab imports, so every Lab script can use it.
rig.local.json chooses with "host": "laptop" (the default) or "pi".
"""


def is_pi(rig):
    return rig.get('host', 'laptop') == 'pi'


def remote_section(rig):
    """Settings for the motor owner. Pi paths derive from one 'root' folder:
    root/spirob_cable and root/spirob_experiment (motor + capture code), root/config.json
    (motor configuration), root/lab (Lab runtime), root/.venv (Python)."""
    if not is_pi(rig):
        return rig['wsl']
    pi = dict(rig['pi'])
    root = pi['root'].rstrip('/')
    pi.setdefault('motor_repo', root)
    pi.setdefault('runtime', root + '/lab')
    pi.setdefault('capture_source', root)
    pi.setdefault('python', root + '/.venv/bin/python')
    return pi
