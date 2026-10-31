"""Read-only RL camera worker: newest image, one detection, one latest result.

The comparison feed still brackets older images with encoder samples. This feed
keeps the newest image and a separate encoder timestamp: no invented samples,
no camera resizing and no motor commands. All Qt/control work stays on the GUI
thread. Ages include HTTP, decode, detection, and time waiting for the GUI.
"""
from __future__ import annotations

import base64
import io
import math
import threading
import time

import numpy as np
from PIL import Image

from connection import camera_get


def age_item(item, now=None):
    """Copy age-bearing fields; never mutate a worker result or stop its clock."""
    if item is None:
        return None
    now = time.monotonic() if now is None else now
    delay = max(0.0, now - item['received'])
    packet = dict(item['packet'])
    packet['frame_age_s'] += delay
    packet['motor_age_s'] = [None if x is None else x + delay for x in packet['motor_age_s']]
    if packet.get('encoder_age_s') is not None:
        packet['encoder_age_s'] += delay
    timing = dict(item.get('rl_timing', {}))
    timing['result_wait_s'] = timing.get('result_wait_s', 0.0) + delay
    return dict(item, packet=packet, received=now,
                displayed_age_s=item['displayed_age_s'] + delay, rl_timing=timing)


def encoder_packet(item, max_age=0.65):
    """Use a real time bracketed by both encoders, separate from image time."""
    packet = item['packet']
    if packet.get('frame_selection') != 'latest':
        return packet  # Compatibility with existing offline/legacy tests.
    ages = packet.get('motor_age_s', [])
    age = packet.get('encoder_age_s')
    t = packet.get('encoder_t_s')
    if (not packet.get('motor_active') or len(ages) != 2 or
            any(x is None or not math.isfinite(x) or x < 0 or x > max_age for x in ages) or
            age is None or not math.isfinite(age) or not 0 <= age <= max_age or
            t is None or not math.isfinite(t)):
        def show(x):
            return f'{x:.3f}' if isinstance(x, (int, float)) else str(x)
        raise ValueError('Encoder feedback unavailable: ages=[' + ', '.join(show(x) for x in ages) +
                         f'] s; common={show(age)} s; limit={max_age:.3f} s; '
                         f'active={packet.get("motor_active")}; states={packet.get("motor_states")}')
    return dict(packet, t_s=t)


def age_feedback(value, now):
    packet, received, request_s = value
    delay = max(0.0, now-received)
    packet = dict(packet)
    packet['motor_age_s'] = [None if x is None else x+delay for x in packet['motor_age_s']]
    if packet.get('encoder_age_s') is not None:
        packet['encoder_age_s'] += delay
    return packet, request_s, delay


def accept_feedback(packet, previous, began, arrived):
    """Accept a newly generated server response, not a cached/replayed response.

    The same real encoder sample may occur in several freshly generated replies:
    each reply recomputes its age on the server. A previous slow HTTP request is
    an upper bound, not an extra age to carry forward into a faster new reply.
    Only an unchanged/regressing RESPONSE timestamp retains the old receipt time.
    """
    generated = packet.get('feedback_generated_s')
    if (not isinstance(generated, (float, int)) or not math.isfinite(generated)):
        raise ValueError('Timestamped encoder service unavailable; deploy the feedback-age patch')
    if previous is not None and previous[0]['session'] == packet['session']:
        if generated <= previous[0]['feedback_generated_s']:
            return previous  # Do not refresh ages or receipt time for a replay.
    ages = packet.get('motor_age_s')
    common = packet.get('encoder_age_s')
    if not isinstance(ages, list) or len(ages) != 2:
        raise ValueError('Expected two encoder ages')
    if any(x is not None and (not isinstance(x, (int, float)) or not math.isfinite(x) or x < 0) for x in ages):
        raise ValueError('Invalid per-axis encoder ages')
    expected = max(ages) if all(x is not None for x in ages) else None
    if expected is None:
        if common is not None:
            raise ValueError('Common encoder age supplied without both axes')
    elif (not isinstance(common, (float, int)) or not math.isfinite(common) or
          abs(common-expected) > 1e-6):
        raise ValueError('Common encoder age must equal the oldest per-axis age')
    elapsed = arrived-began
    if not math.isfinite(elapsed) or elapsed < 0:
        raise ValueError('Invalid encoder request clock interval')
    packet = dict(packet)
    packet['motor_age_s'] = [None if x is None else x+elapsed for x in ages]
    packet['encoder_age_s'] = None if expected is None else max(packet['motor_age_s'])
    return packet, arrived, elapsed


class LiveFeed:
    """One daemon reader; replacing configuration invalidates all old results."""
    def __init__(self, get=camera_get, clock=time.monotonic):
        self.get, self.clock = get, clock
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.stopped = threading.Event()
        self.generation = 0
        self.job = self.result = None
        self.feedback = None
        self.feedback_error = 'Waiting for independent encoder feedback'
        self.error = 'Waiting for the RL camera feed'
        self.thread = threading.Thread(target=self._run, name='rl-camera-detection', daemon=True)
        self.thread.start()
        self.feedback_thread = threading.Thread(target=self._feedback_run, name='rl-encoder-feedback', daemon=True)
        self.feedback_thread.start()

    def configure(self, url, token, camera, egg, cup):
        with self.lock:
            self.generation += 1
            self.job = (url, token, camera, egg, cup)
            self.result = None
            self.feedback = None
            self.feedback_error = 'Waiting for independent encoder feedback'
            self.error = 'Waiting for five fresh detected frames'
        self.wake.set()

    def pause(self):
        with self.lock:
            if self.job is not None:
                self.generation += 1
                self.job = self.result = None
                self.feedback = None
                self.error = 'Connect the camera or motor session'
        self.wake.set()

    def close(self):
        self.stopped.set()
        self.pause()
        self.wake.set()

    def snapshot(self):
        with self.lock:
            item, error, feedback, feedback_error = self.result, self.error, self.feedback, self.feedback_error
        now = self.clock()
        item = age_item(item, now)
        if item is None:
            return None, error
        packet = item['packet']
        item['rl_timing']['camera_packet_encoder_age_s'] = packet.get('encoder_age_s')
        if feedback is not None and feedback[0]['session'] == packet['session']:
            fresh, request_s, wait_s = age_feedback(feedback, now)
            for key in ('motor', 'motor_active', 'motor_states', 'motor_age_s', 'encoder_t_s',
                        'encoder_age_s', 'rest_lengths_mm', 'feedback_generated_s'):
                if key in fresh:
                    packet[key] = fresh[key]
            item['rl_timing'].update(feedback_request_s=request_s, feedback_wait_s=wait_s,
                                    feedback_generated_s=fresh.get('feedback_generated_s'))
        else:
            # Never silently fall back to encoders that waited behind JPEG processing.
            packet.update(motor=[], motor_active=False, motor_states=[None, None],
                          motor_age_s=[None, None], encoder_t_s=None, encoder_age_s=None)
        packet['feedback_source'] = 'independent'
        item['rl_feedback_error'] = feedback_error
        return item, error or feedback_error

    def _feedback_run(self):
        while not self.stopped.is_set():
            with self.lock:
                job, generation = self.job, self.generation
            if job is None:
                self.stopped.wait(.05)
                continue
            began = self.clock()
            try:
                packet = self.get(job[0], job[1], '/rl-feedback', timeout=.5)
                arrived = self.clock()
                if packet.get('feedback_only') is not True or not packet.get('session'):
                    raise ValueError('Independent encoder service unavailable; deploy the feedback patch')
                with self.lock:
                    if generation != self.generation or self.stopped.is_set():
                        continue
                    self.feedback = accept_feedback(packet, self.feedback, began, arrived)
                    self.feedback_error = ''
            except Exception as exc:
                with self.lock:
                    if generation == self.generation:
                        self.feedback_error = f'RL encoders: {type(exc).__name__}: {exc}'
            self.stopped.wait(max(.01, .1-(self.clock()-began)))

    def _run(self):
        previous_key = None
        while not self.stopped.is_set():
            with self.lock:
                job, generation = self.job, self.generation
            if job is None:
                self.wake.wait(.1)
                self.wake.clear()
                continue
            began = self.clock()
            url, token, camera, egg, cup = job
            try:
                packet = self.get(url, token, '/rl-packet', timeout=1)
                arrived = self.clock()
                if packet.get('frame_selection') != 'latest':
                    raise ValueError('Latest-frame camera service unavailable; deploy the camera patch')
                if 'image_b64' not in packet:
                    raise ValueError(packet.get('message', 'Waiting for camera frames'))
                key = (generation, packet['session'], packet['frame_index'])
                if key == previous_key:
                    # A repeat response is not a new frame and cannot refresh ages.
                    self.stopped.wait(.02)
                    continue
                rgb = np.array(Image.open(io.BytesIO(base64.b64decode(packet['image_b64']))).convert('RGB'))
                decoded = self.clock()
                det = egg.detect(rgb)
                pedestal = cup.detect(rgb) if cup is not None else None
                finished = self.clock()
                elapsed = finished - began
                packet = dict(packet)
                source_age = float(packet['frame_age_s'])
                if not math.isfinite(source_age) or source_age < 0:
                    raise ValueError('Invalid camera frame age')
                packet['frame_age_s'] = source_age + elapsed
                packet['motor_age_s'] = [None if x is None else x + elapsed for x in packet['motor_age_s']]
                if packet.get('encoder_age_s') is not None:
                    packet['encoder_age_s'] += elapsed
                item = dict(rgb=rgb, packet=packet, displayed_age_s=packet['frame_age_s'],
                            received=finished, rl_detected=True, rl_egg=det, rl_cup=pedestal,
                            rl_camera=camera, rl_timing=dict(server_frame_age_s=source_age,
                            request_s=arrived-began, decode_s=decoded-arrived,
                            detection_s=finished-decoded, result_wait_s=0.0))
                with self.lock:
                    if generation == self.generation and not self.stopped.is_set():
                        self.result, self.error = item, ''
                        previous_key = key
            except Exception as exc:
                with self.lock:
                    if generation == self.generation:
                        # A retained frame continues ageing; the existing guard stops
                        # an active episode when it is no longer fresh.
                        self.error = f'RL camera: {type(exc).__name__}: {exc}'
            self.stopped.wait(max(.01, .1 - (self.clock() - began)))
