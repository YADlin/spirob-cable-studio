"""Read-only checks: never enable, disable, configure, or write drive registers."""
import argparse
import json
from pathlib import Path
import time
from .config import RigSettings
from .records import NullRecorder
from .worker import make_drives


def run(config_path):
    cfg = RigSettings.load(config_path)
    cfg.validate()
    drives = make_drives(cfg, False, NullRecorder())
    failed = False
    print('READ ONLY | ASCII 9600 8N1 | 0.300 s/request | no register writes', flush=True)
    try:
        for index, drive in enumerate(drives, 1):
            print(f'\nCable {index}: ID {drive.settings.device_id} on {drive.settings.port}', flush=True)
            try:
                drive.open()
                start = time.monotonic()
                lpr = drive.read(10)
                print(f'  LPR={lpr}, request took {time.monotonic()-start:.3f} s', flush=True)
                start = time.monotonic()
                count = drive.position()
                duration = time.monotonic()-start
                print(f'  Encoder={count}, three-word read took {duration:.3f} s', flush=True)
                if lpr != 334 or duration > .65:
                    failed = True
                    print('  CHECK: application expects LPR=334 and encoder read <=0.650 s.', flush=True)
                else:
                    print('  PASS: valid register responses. Enable/disable writes were NOT tested.', flush=True)
            except Exception as exc:
                failed = True
                print(f'  FAIL: {type(exc).__name__}: {exc}', flush=True)
            print('  Last request: '+json.dumps(drive.client.last_io), flush=True)
    finally:
        for drive in drives:
            drive.close()
    return 1 if failed else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parent.parent/'config.json')
    args = parser.parse_args()
    try:
        return run(args.config)
    except KeyboardInterrupt:
        print('\nInterrupted; no motor commands were sent.', flush=True)
        return 130
    except Exception as exc:
        print(f'Configuration/diagnostic error: {exc}', flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
