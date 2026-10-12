"""Live JPEGs stay in memory. Only an explicit Record command writes images.

Reuse the installed camera source and HTTP protocol. This adapter is deployed
with SpiRob Lab; the existing experiment addon is not edited.
"""
from collections import deque
import csv
import json
from pathlib import Path
import shutil
import threading
import time
import tempfile
from spirob_experiment.camera import Capture, CameraService
from spirob_experiment.storage import atomic_json, software_provenance


class LiveCapture(Capture):
    buffer_limit = 32 * 1024 * 1024
    record_limit = 250 * 1024 * 1024
    record_seconds = 60.

    def __init__(self, *args, **kwargs):
        self.frames = deque()
        self.buffer_bytes = 0
        self.context = None
        self.ready_to_open = threading.Event()
        self.opening_started = None
        self.record_lock = threading.Lock()
        self.record_on = False
        self.record_reason = 'Not requested'
        self.record_started = 0.
        self.record_elapsed = 0.
        self.record_bytes = 0
        self.record_frames = 0
        super().__init__(*args, **kwargs)

    def recording(self):
        with self.record_lock:
            elapsed = self.record_elapsed + (time.monotonic()-self.record_started if self.record_on else 0.)
            return dict(active=self.record_on, reason=self.record_reason,
                        seconds=elapsed, bytes=self.record_bytes, frames=self.record_frames,
                        max_seconds=self.record_seconds, max_bytes=self.record_limit)

    def record(self, enabled):
        if type(enabled) is not bool:
            raise ValueError('Recording enabled must be true or false')
        with self.record_lock:
            if enabled and not self.record_on:
                if self.record_elapsed >= self.record_seconds or self.record_bytes >= self.record_limit:
                    raise ValueError('Recording limit reached for this session')
                self.record_started = time.monotonic()
            if not enabled and self.record_on:
                self.record_elapsed += time.monotonic()-self.record_started
            self.record_on = enabled
            self.record_reason = 'User recording' if enabled else 'Stopped by user'
        return self.recording()

    def status(self, renew=True):
        result = super().status(renew)
        result['startup_stage'] = ('waiting_to_open' if self.opening_started is None else
                                   'opening_camera' if result['frame_count']==0 else 'frames_arriving')
        result['recording'] = self.recording()
        result['bytes_written'] = result['recording']['bytes']
        result['camera_monotonic_s'] = time.monotonic()
        result['image_storage'] = 'Preview in memory; saved JPEGs only during explicit recording'
        return result

    def snapshot(self):
        with self.lock:
            return list(self.frames), self.context

    def _save_recorded(self, item, writer, stream):
        with self.record_lock:
            if not self.record_on:
                return
            now = time.monotonic()
            elapsed = self.record_elapsed + now-self.record_started
            reason = None
            if elapsed >= self.record_seconds:
                reason = '60-second recording limit reached'
            elif self.record_bytes + len(item['jpeg']) > self.record_limit:
                reason = '250-MiB recording limit reached'
            elif shutil.disk_usage(self.path).free < 256*1024*1024:
                reason = 'Less than 256 MiB free; recording stopped'
            if reason:
                self.record_on = False
                self.record_elapsed = elapsed
                self.record_reason = reason
                return
            relative = f"frames/{item['frame_index']:08d}.jpg"
            try:
                with (self.path/relative).open('xb') as f:
                    f.write(item['jpeg'])
                writer.writerow({k: item[k] for k in ('frame_index','read_start_s','read_end_s','width','height')} |
                                dict(file=relative, write_end_s=time.monotonic()))
                stream.flush()
            except OSError as exc:
                self.record_on = False
                self.record_elapsed = elapsed
                self.record_reason = 'Recording write failed: '+str(exc)
                return
            self.record_bytes += len(item['jpeg'])
            self.record_frames += 1

    def _run(self):
        source = None
        try:
            self.ready_to_open.wait(1.)
            if self.cancel.is_set():
                with self.lock: self.data['state']='COMPLETE'
                return
            self.opening_started = time.monotonic()
            import cv2
            source = self.factory()
            source.info['software'] = software_provenance()
            with self.lock:
                self.data['source'] = source.info
                self.data['encoding'] = 'JPEG quality 95; preview buffered in memory'
            atomic_json(self.path/'source.json', source.info)
            columns = ['frame_index','file','read_start_s','read_end_s','write_end_s','width','height']
            with (self.path/'frames.csv').open('x', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=columns)
                writer.writeheader(); stream.flush()
                index = 0
                while not self.cancel.is_set():
                    start = time.monotonic()
                    with self.lock:
                        lease = self.lease
                    if start-lease > self.lease_s or start-self.started > self.max_s:
                        raise RuntimeError('Camera coordinator lease or session duration expired')
                    ok, frame = source.read()
                    arrival = time.monotonic()
                    if not ok or frame is None:
                        raise RuntimeError('Camera read failed')
                    ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY,95])
                    if not ok:
                        raise RuntimeError('JPEG encoding failed')
                    data = jpeg.tobytes()
                    if len(data) > 8*1024*1024:
                        raise RuntimeError('Camera JPEG exceeds 8 MiB')
                    item = dict(frame_index=index, jpeg=data, read_start_s=start, read_end_s=arrival,
                                width=frame.shape[1], height=frame.shape[0])
                    with self.lock:
                        self.frames.append(item); self.buffer_bytes += len(data)
                        while len(self.frames)>90 or self.buffer_bytes>self.buffer_limit:
                            self.buffer_bytes -= len(self.frames.popleft()['jpeg'])
                        self.arrivals.append(arrival)
                        self.data.update(state='STREAMING', frame_count=index+1, last_frame_s=arrival)
                    self._save_recorded(item, writer, stream)
                    index += 1
            with self.lock:
                self.data['state'] = 'COMPLETE'
        except Exception as exc:
            with self.lock:
                self.data.update(state='FAILED', error=f'{type(exc).__name__}: {exc}')
        finally:
            with self.record_lock:
                if self.record_on:
                    self.record_elapsed += time.monotonic()-self.record_started
                    self.record_on = False
                    self.record_reason = 'Session ended'
            if source:
                try:
                    source.close()
                except Exception as exc:
                    with self.lock:
                        self.data.update(state='FAILED', error='Camera close failed: '+str(exc))
            with self.lock:
                self.data['bytes_written'] = self.record_bytes
            atomic_json(self.path/'capture.json', self.status(renew=False))


class LiveCameraService(CameraService):
    def preview_start(self, data):
        with self.lock:
            if self.capture and self.capture.thread.is_alive():
                raise RuntimeError('A camera session is already active')
            if getattr(self, 'preview_directory', None): self.preview_directory.cleanup()
            self.preview_directory = tempfile.TemporaryDirectory(prefix='spirob-setup-')
            self.capture = SetupCapture(self.preview_directory.name, data['session_id'], data['nonce'],
                                        self.factory, max_s=float('inf'))
            self.capture.ready_to_open.set()
            return self.capture.status()

    def preview_current(self, data):
        capture = self.current(data)
        if not isinstance(capture, SetupCapture): raise ValueError('Not a setup preview')
        return capture

    def preview_stop(self, data):
        capture = self.preview_current(data)
        result = capture.stop()
        if result['thread_exited']: self.preview_directory.cleanup()
        return result

    def start(self, data):
        with self.lock:
            if self.capture and self.capture.thread.is_alive():
                raise RuntimeError('A camera session is already active')
            self.capture = LiveCapture(self.root, data['session_id'], data['nonce'], self.factory, max_s=3720)
            result = self.capture.status()
            # Defer heavy driver work until the start response can leave this handler.
            timer = threading.Timer(.1, self.capture.ready_to_open.set)
            timer.daemon = True; timer.start()
            return result


class SetupCapture(LiveCapture):
    """Frames stay in bounded RAM; startup metadata lives in a temporary folder."""
    def record(self, enabled):
        raise ValueError('Connect a run before recording video. Setup snapshots can be exported explicitly.')
