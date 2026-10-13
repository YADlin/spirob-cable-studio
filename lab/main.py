"""Start one window. Experiment logic lives in connection, actuation and analysis."""
import argparse
import json
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', action='store_true', help='Recorded held images; no SSH or motors')
    parser.add_argument('--rig', type=Path, default=Path(__file__).parent/'rig.local.json')
    args = parser.parse_args()
    rig_path = args.rig.expanduser().resolve()
    os.chdir(Path(__file__).resolve().parent)
    if not args.demo and not rig_path.is_file():
        raise SystemExit('First run: python3 setup.py import --from /path/to/old/project/rig.local.json')
    from interface import QApplication, Window
    config = json.loads(Path('config.json').read_text())
    if Path('lab.local.json').is_file():
        preferences=json.loads(Path('lab.local.json').read_text())
        for key in ('theme','comparison_setup','analysis_mode','motion_profile'):
            if key in preferences:config[key]=preferences[key]
    if args.demo:
        # The bundled photographs belong to the previous robot, not the new XML.
        config.update(model='model/spirob_robot.xml',fit='calibration/pilot_fit.json')
        config.pop('comparison_setup',None);config.pop('motion_profile',None)
    fit = json.loads(Path(config.get('fit','calibration/pilot_fit.json')).read_text())
    rig = None if args.demo else json.loads(rig_path.read_text())
    application = QApplication([])
    window = Window(config, fit, rig, args.demo)
    window.show()
    application.exec()


if __name__ == '__main__':
    main()
