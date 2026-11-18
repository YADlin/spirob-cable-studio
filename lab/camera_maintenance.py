"""Check or stop the camera service on this machine (Raspberry Pi). Standard library only.

Addresses only the authenticated SpiRob service; never kills whatever owns the port.
The laptop does the same through PowerShell in setup.py.
"""
import argparse
import json
from pathlib import Path
import socket
import sys
import time
from urllib.request import Request, build_opener, ProxyHandler

from hosts import remote_section

URL = 'http://127.0.0.1:8765'


def listening():
    with socket.socket() as sock:
        return sock.connect_ex(('127.0.0.1', 8765)) == 0


def call(route, token, data=None):
    request = Request(URL + route, None if data is None else json.dumps(data).encode(),
                      {'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'})
    with build_opener(ProxyHandler({})).open(request, timeout=4) as reply:
        body = reply.read(65536)
    return json.loads(body) if body else {}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rig', type=Path, required=True)
    parser.add_argument('--stop', action='store_true', help='close the service if it is idle')
    args = parser.parse_args()
    token = Path(remote_section(json.loads(args.rig.read_text()))['token_file']).read_text().strip()
    if not listening():
        print('No camera service is listening')
        return 0
    health = call('/health', token)
    if health.get('service') != 'spirob-live':
        raise RuntimeError('Port 8765 is not the authenticated SpiRob camera service')
    print(f"Camera version {health.get('version')}; capture {health.get('capture_state')}")
    if not args.stop:
        return 0
    if health.get('capture_state') in ('STARTING', 'STREAMING', 'RECORDING'):
        raise RuntimeError('Capture is active. Use Disconnect in its control window, then retry.')
    call('/shutdown-service', token, {})
    for _ in range(30):
        if not listening():
            print('Idle camera service closed')
            return 0
        time.sleep(.1)
    raise RuntimeError('Service shutdown requested but port remains occupied; inspect camera-status')


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
