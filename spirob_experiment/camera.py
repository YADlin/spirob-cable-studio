"""Camera sources and a leased, authenticated HTTP camera service.

The service has no motor commands. Frames are individual JPEG files plus an
arrival-time index, so a crash does not require repairing a video container.
"""
from collections import deque
import csv
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import shutil
import threading
import time
from urllib.parse import urlsplit
from .storage import atomic_json, software_provenance


class SyntheticCamera:
    def __init__(self, width=640, height=360, fps=20):
        self.width, self.height, self.fps = width, height, fps
        self.next_frame = time.monotonic()
        self.index = 0
        self.info = {"source": "synthetic", "width": width, "height": height,
                     "requested_fps": fps, "reported_fps": fps}

    def read(self):
        import cv2
        import numpy as np
        time.sleep(max(0, self.next_frame-time.monotonic()))
        self.next_frame = time.monotonic()+1/self.fps
        frame = np.full((self.height, self.width, 3), 240, dtype=np.uint8)
        cv2.putText(frame, f"OFFLINE CAMERA / {self.index}", (15, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, .7, (20, 20, 20), 2)
        self.index += 1
        return True, frame

    def close(self):
        pass


class OpenCVCamera:
    def __init__(self, device=0, backend="auto", width=1920, height=1080, fps=30):
        import cv2
        api = {"auto": cv2.CAP_ANY, "dshow": cv2.CAP_DSHOW,
               "msmf": cv2.CAP_MSMF, "v4l2": cv2.CAP_V4L2}[backend]
        self.cap = cv2.VideoCapture(device, api)
        try:
            if not self.cap.isOpened():
                raise RuntimeError(f"Camera {device} did not open using {backend}")
            requests = [(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")),
                        (cv2.CAP_PROP_FRAME_WIDTH, width),
                        (cv2.CAP_PROP_FRAME_HEIGHT, height),
                        (cv2.CAP_PROP_FPS, fps)]
            accepted = [bool(self.cap.set(prop, value)) for prop, value in requests]
            fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
            self.info = {"source": "opencv", "device": device,
                         "backend": self.cap.getBackendName(), "opencv": cv2.__version__,
                         "requested_width": width, "requested_height": height,
                         "requested_fps": fps, "set_return_values": accepted,
                         "reported_width": self.cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                         "reported_height": self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                         "reported_fps": self.cap.get(cv2.CAP_PROP_FPS),
                         "reported_fourcc": "".join(chr((fourcc >> (8*i)) & 255) for i in range(4)),
                         "exposure_reported": self.cap.get(cv2.CAP_PROP_EXPOSURE),
                         "auto_exposure_reported": self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE),
                         "timing": "Host read-call times; driver buffering/exposure latency unknown"}
        except BaseException:
            self.cap.release()
            raise

    def read(self):
        return self.cap.read()

    def close(self):
        self.cap.release()


class Capture:
    def __init__(self, root, session_id, nonce, factory, lease_s=10, max_s=3700):
        if not re.fullmatch(r"[A-Za-z0-9_-]{1,100}", session_id):
            raise ValueError("Invalid session id")
        if not re.fullmatch(r"[a-f0-9]{32}", nonce):
            raise ValueError("Invalid session nonce")
        self.path = Path(root)/session_id/"camera"
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path/"frames").mkdir()
        self.session_id, self.nonce = session_id, nonce
        self.factory, self.lease_s, self.max_s = factory, lease_s, max_s
        self.cancel = threading.Event()
        self.lock = threading.Lock()
        self.lease = time.monotonic()
        self.started = self.lease
        self.arrivals = deque(maxlen=60)
        self.data = {"schema_version": 1, "session_id": session_id, "nonce": nonce,
                     "state": "STARTING", "frame_count": 0, "error": None,
                     "bytes_written": 0,
                     "last_frame_s": None, "encoding": "JPEG quality 95; decoded camera images, not sensor raw"}
        atomic_json(self.path/"capture.json", self.data)
        self.thread = threading.Thread(target=self._run, daemon=True, name="camera-acquisition")
        self.thread.start()

    def status(self, renew=True):
        with self.lock:
            if renew:
                self.lease = time.monotonic()
            result = dict(self.data)
            result["last_frame_age_s"] = (None if result["last_frame_s"] is None
                                         else time.monotonic()-result["last_frame_s"])
            result["observed_fps"] = ((len(self.arrivals)-1)/(self.arrivals[-1]-self.arrivals[0])
                                      if len(self.arrivals) > 1 and self.arrivals[-1] > self.arrivals[0] else None)
            return result

    def _run(self):
        source = None
        try:
            import cv2
            source = self.factory()
            source.info["software"] = software_provenance()
            with self.lock:
                self.data["source"] = source.info
            atomic_json(self.path/"source.json", source.info)
            columns = ["frame_index", "file", "read_start_s", "read_end_s", "write_end_s",
                       "width", "height"]
            with (self.path/"frames.csv").open("x", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=columns)
                writer.writeheader()
                index = 0
                while not self.cancel.is_set():
                    start = time.monotonic()
                    with self.lock:
                        lease = self.lease
                    if start-lease > self.lease_s:
                        raise RuntimeError("Coordinator lease expired; acquisition stopped")
                    if start-self.started > self.max_s:
                        raise RuntimeError("Camera duration limit exceeded")
                    if index % 100 == 0 and shutil.disk_usage(self.path).free < 128*1024*1024:
                        raise RuntimeError("Less than 128 MiB free; camera recording stopped")
                    ok, frame = source.read()
                    arrival = time.monotonic()
                    if not ok or frame is None:
                        raise RuntimeError("Camera read failed")
                    encoded, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    if not encoded:
                        raise RuntimeError("JPEG encoding failed")
                    relative = f"frames/{index:08d}.jpg"
                    with (self.path/relative).open("xb") as output:
                        output.write(jpeg.tobytes())
                    writer.writerow({"frame_index": index, "file": relative, "read_start_s": start,
                                     "read_end_s": arrival, "write_end_s": time.monotonic(),
                                     "width": frame.shape[1], "height": frame.shape[0]})
                    stream.flush()
                    with self.lock:
                        self.arrivals.append(arrival)
                        self.data.update(state="RECORDING", frame_count=index+1, last_frame_s=arrival)
                        self.data["bytes_written"] += len(jpeg)
                    index += 1
            with self.lock:
                self.data["state"] = "COMPLETE"
        except Exception as exc:
            with self.lock:
                self.data.update(state="FAILED", error=f"{type(exc).__name__}: {exc}")
        finally:
            if source:
                try:
                    source.close()
                except Exception as exc:
                    with self.lock:
                        self.data.update(state="FAILED", error=f"Camera close failed: {exc}")
            atomic_json(self.path/"capture.json", self.status(renew=False))

    def stop(self):
        self.cancel.set()
        self.thread.join(2)
        result = self.status(renew=False)
        result["thread_exited"] = not self.thread.is_alive()
        return result


class CameraService:
    def __init__(self, root, token, factory):
        if len(token) < 32:
            raise ValueError("Camera token must have at least 32 characters")
        self.root, self.token, self.factory = Path(root), token, factory
        self.capture = None
        self.lock = threading.Lock()

    def start(self, data):
        with self.lock:
            if self.capture and self.capture.thread.is_alive():
                raise RuntimeError("A camera session is already active")
            self.capture = Capture(self.root, data["session_id"], data["nonce"], self.factory)
            return self.capture.status()

    def current(self, data):
        if not self.capture or data.get("session_id") != self.capture.session_id or data.get("nonce") != self.capture.nonce:
            raise ValueError("Camera session identity does not match")
        return self.capture


def serve(service, host="127.0.0.1", port=8765):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # Never place the authentication header in an access log.

        def do_POST(self):
            if not hmac.compare_digest(self.headers.get("Authorization", ""), "Bearer "+service.token):
                self.send_error(401)
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 4096:
                    raise ValueError("Expected a small JSON request")
                data = json.loads(self.rfile.read(size))
                route = urlsplit(self.path).path
                if route == "/clock":
                    result = {"camera_monotonic_s": time.monotonic(), "utc_ns": time.time_ns()}
                elif route == "/start":
                    result = service.start(data)
                elif route == "/status":
                    result = service.current(data).status()
                elif route == "/stop":
                    result = service.current(data).stop()
                else:
                    self.send_error(404)
                    return
                status = 200
            except Exception as exc:
                result, status = {"error": f"{type(exc).__name__}: {exc}"}, 409
            payload = json.dumps(result, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server
