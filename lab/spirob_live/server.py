"""Add authenticated GET /health and /packet to the existing Windows camera server.

Run this INSTEAD OF camera-serve. Its original POST control API is inherited intact.
"""
import argparse
import hmac
import json
from pathlib import Path
import signal
import sys
import threading
from urllib.parse import urlsplit
from .packets import make_packet, make_feedback


def add_observer(server, service):
    original = server.RequestHandlerClass
    class Handler(original):
        def do_POST(self):
            route = urlsplit(self.path).path
            if route in ('/preview/start','/preview/status','/preview/stop'):
                if not hmac.compare_digest(self.headers.get('Authorization',''), 'Bearer '+service.token):
                    self.send_error(401); return
                try:
                    size=int(self.headers.get('Content-Length',0))
                    if not 0 < size <= 4096: raise ValueError('Invalid request size')
                    data=json.loads(self.rfile.read(size))
                    if route == '/preview/start': result=service.preview_start(data)
                    elif route == '/preview/stop': result=service.preview_stop(data)
                    else: result=service.preview_current(data).status()
                    code=200
                except Exception as exc: result,code={'error':str(exc)},409
                payload=json.dumps(result,allow_nan=False).encode()
                self.send_response(code); self.send_header('Content-Length',str(len(payload)))
                self.end_headers(); self.wfile.write(payload); return
            if urlsplit(self.path).path in ('/context', '/record'):
                if not hmac.compare_digest(self.headers.get('Authorization', ''), 'Bearer '+service.token):
                    self.send_error(401); return
                try:
                    size = int(self.headers.get('Content-Length', 0))
                    if not 0 < size <= 16384: raise ValueError('Invalid request size')
                    data = json.loads(self.rfile.read(size))
                    capture = service.current(data)
                    if urlsplit(self.path).path == '/context':
                        with capture.lock: capture.context = data['context']
                        result = {'context_received': True}
                    else:
                        result = capture.record(data['enabled'])
                    code = 200
                except Exception as exc:
                    result, code = {'error': str(exc)}, 409
                payload = json.dumps(result).encode()
                self.send_response(code); self.send_header('Content-Length', str(len(payload)))
                self.end_headers(); self.wfile.write(payload); return
            if urlsplit(self.path).path != '/shutdown-service':
                return super().do_POST()
            if not hmac.compare_digest(self.headers.get('Authorization', ''), 'Bearer '+service.token):
                self.send_error(401)
                return
            if service.capture and service.capture.status(renew=False)['state'] in ('STARTING', 'RECORDING', 'STREAMING'):
                self.send_error(409, 'Close acquisition before closing the service')
                return
            self.send_response(200)
            self.send_header('Content-Length', '0')
            self.end_headers()
            threading.Thread(target=server.shutdown, daemon=True).start()

        def do_GET(self):
            if not hmac.compare_digest(self.headers.get('Authorization', ''), 'Bearer '+service.token):
                self.send_error(401)
                return
            route = urlsplit(self.path).path
            if route not in ('/health', '/packet', '/rl-packet', '/rl-feedback'):
                self.send_error(404)
                return
            try:
                result = ({'service': 'spirob-live', 'version': '0.7.0', 'read_only_preview': True,
                            'capture_state': service.capture.status(renew=False)['state'] if service.capture else 'IDLE'}
                          if route == '/health' else make_feedback(service.capture) if route == '/rl-feedback'
                          else make_packet(service.capture, latest_frame=route == '/rl-packet'))
                if route == '/rl-packet':
                    result['frame_selection'] = 'latest'
                code = 200
            except (FileNotFoundError, json.JSONDecodeError, KeyError):
                result, code = {'state': 'WAITING', 'message': 'Session files are being initialized.'}, 200
            except Exception as exc:
                result, code = {'state': 'UNAVAILABLE', 'message': str(exc)}, 409
            payload = json.dumps(result, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass
    server.RequestHandlerClass = Handler
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rig', type=Path, required=True)
    args = parser.parse_args()
    rig = json.loads(args.rig.read_text())
    camera = rig['windows']
    addon = Path(camera['capture_source'])
    if not (addon / 'spirob_experiment/camera.py').is_file():
        raise SystemExit(f'Cannot find existing capture source at {addon}')
    sys.path.insert(0, str(addon))
    from spirob_experiment.camera import OpenCVCamera, serve
    from .capture import LiveCameraService
    token = Path(camera['token_file']).read_text().strip()
    factory = lambda: OpenCVCamera(camera['device'], camera['backend'], 1920, 1080, 30)
    # Pay module/provenance cold-start cost before advertising a ready service.
    import cv2
    from spirob_experiment.storage import software_provenance
    software_provenance()
    service = LiveCameraService(camera['data_root'], token, factory)
    server = add_observer(serve(service, '127.0.0.1', 8765), service)
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    print('SpiRob camera + observer ready on Windows loopback:8765. Waiting for the interactive session.', flush=True)
    print(f'Reusing capture implementation: {addon}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if service.capture:
            status = service.capture.stop()
            print('Camera writer exited:', status['thread_exited'], flush=True)
        if getattr(service,'preview_directory',None): service.preview_directory.cleanup()
        server.server_close()


if __name__ == '__main__':
    main()
