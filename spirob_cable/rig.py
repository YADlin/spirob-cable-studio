"""One owner for both drives. Validate all endpoints before writing either target.

The RMCS protocol has no documented synchronized multi-drive commit. Commands
and feedback are sequential; this class promises bounded endpoints, not exact
A/D trajectories between them.
"""
from collections import deque
from dataclasses import dataclass
import math
import time
from .model import Rejected, finite, signed_count, TRAVEL_MM, COUNTS_PER_REV
from .config import lengths_from_modes, modes_from_lengths
from .drive import DriveFault

FRESH_SECONDS = 2.0


class Cancelled(Exception):
    """Stop/release/close requested during a sequence; do not dispatch more targets."""


@dataclass(frozen=True)
class Move:
    axis: int
    count: int
    speed: float
    kind: str


class Axis:
    def __init__(self, index, cfg, drive, recorder):
        self.index, self.cfg, self.drive, self.recorder = index, cfg, drive, recorder
        self.count = self.connection = self.rest = self.rest_length = self.target = None
        self.enabled = False
        self.state = 'DISABLED'
        self.last_read = None
        self.stable = deque(maxlen=3)
        self.deadline = None
        self.rpm = cfg.motor_rpm
        self.speed_verified = False
        self.epoch = 0
        self.disable_confirmed = None
        self.message = 'Torque off. Enable to jog; rest is optional for setup.'

    @property
    def tolerance(self):
        return max(1, int(.03/self.cfg.mm_per_count))

    def fresh(self):
        if self.last_read is None or time.monotonic()-self.last_read > FRESH_SECONDS:
            raise DriveFault(f'Cable {self.index+1}: feedback is stale')

    def stationary(self):
        if len(self.stable) < 3 or max(self.stable)-min(self.stable) > max(1, self.tolerance//3):
            raise Rejected(f'Cable {self.index+1}: waiting for three stable samples')

    def idle(self):
        self.fresh()
        if not self.enabled or self.state != 'HOLDING':
            raise Rejected(f'Cable {self.index+1}: enable and wait for the previous move to finish')
        self.stationary()

    def envelope(self):
        return (self.rest, TRAVEL_MM) if self.rest is not None else (self.connection, self.cfg.setup_travel_mm)

    def validate_count(self, count):
        signed_count(count)
        reference, limit = self.envelope()
        if reference is None or abs(self.cfg.offset_mm(count, reference)) > limit+1e-9:
            origin = 'physical rest' if self.rest is not None else 'connection position'
            raise Rejected(f'Cable {self.index+1}: target exceeds ±{limit:g} mm from {origin}')

    def absolute(self, count=None):
        if self.rest is None or self.rest_length is None:
            return None
        return self.rest_length+self.cfg.offset_mm(self.count if count is None else count, self.rest)

    def poll(self):
        t0 = time.monotonic(); count = self.drive.position(); t1 = time.monotonic()
        signed_count(count)
        if t1-t0 > .65:
            raise DriveFault(f'Cable {self.index+1}: feedback read exceeded 0.65 s')
        if self.enabled and self.last_read is not None:
            dt = t1-self.last_read
            if dt > FRESH_SECONDS:
                raise DriveFault(f'Cable {self.index+1}: feedback gap exceeded {FRESH_SECONDS:g} s')
            distance = abs(count-self.count)*self.cfg.mm_per_count
            if distance > max(.1, 2*self.cfg.speed_for_rpm(self.rpm)*dt+.05):
                raise DriveFault(f'Cable {self.index+1}: unexpected encoder jump')
            ref, limit = self.envelope()
            if abs(self.cfg.offset_mm(count, ref)) > limit+1e-9:
                raise DriveFault(f'Cable {self.index+1}: measured travel exceeded ±{limit:g} mm')
        if self.connection is None:
            self.connection = count
        self.count, self.last_read = count, t1
        self.stable.append(count)
        if self.state in ('MOVING', 'STOPPING'):
            if t1 > self.deadline:
                raise DriveFault(f'Cable {self.index+1}: move/stop settling timeout')
            still = len(self.stable) == 3 and max(self.stable)-min(self.stable) <= max(1, self.tolerance//3)
            if still and (self.target is None or abs(self.target-count) <= self.tolerance):
                self.state, self.deadline = 'HOLDING', None
                if self.target is None:
                    self.target = count
                self.recorder.event('settled', axis=self.index, count=count, target_count=self.target)
        row = self.snapshot()
        row.update(read_start_s=t0, read_end_s=t1, read_duration_s=t1-t0)
        self.recorder.sample(self.index, row)

    def snapshot(self):
        def offset(count, reference):
            return self.cfg.offset_mm(count, reference) if count is not None and reference is not None else None
        return {'state':self.state, 'enabled':self.enabled, 'count':self.count,
            'connection_count':self.connection, 'rest_count':self.rest, 'rest_length_mm':self.rest_length,
            'reference_epoch':self.epoch, 'target_count':self.target,
            'connection_mm':offset(self.count,self.connection), 'target_connection_mm':offset(self.target,self.connection),
            'offset_mm':offset(self.count,self.rest), 'target_mm':offset(self.target,self.rest),
            'absolute_mm':self.absolute(), 'target_absolute_mm':self.absolute(self.target) if self.target is not None else None,
            'error_mm':offset(self.target,self.count), 'motor_rpm_command':self.rpm if self.enabled and self.speed_verified else None,
            'last_read':self.last_read, 'disable_confirmed':self.disable_confirmed, 'message':self.message,
            'turns_from_connection':(self.count-self.connection)/COUNTS_PER_REV if self.count is not None else None}


class Rig:
    def __init__(self, settings, drives, recorder, cancelled=lambda:False):
        settings.validate()
        if len(drives) != settings.active_axes:
            raise ValueError('Drive count differs from active cable count')
        self.settings, self.recorder, self.cancelled = settings, recorder, cancelled
        self.axes = [Axis(i,cfg,d,recorder) for i,(cfg,d) in enumerate(zip(settings.selected,drives))]
        self.faulted = False
        self.fault_reason = ''
        self.pending_reference = {}

    def checkpoint(self):
        if self.cancelled():
            raise Cancelled()
        if self.faulted:
            raise Rejected('Fault latched: disconnect and inspect before restarting')

    def connect(self):
        for axis in self.axes:
            self.checkpoint(); axis.drive.open()
        for axis in self.axes:
            axis.drive.disable(); axis.disable_confirmed = True
        self.poll_all()
        self.recorder.event('connected_disabled', active_axes=len(self.axes))

    def axis(self, index):
        if type(index) is not int or not 0 <= index < len(self.axes):
            raise Rejected('Invalid cable selection')
        return self.axes[index]

    def poll_all(self):
        if self.faulted:
            return
        for axis in self.axes:
            axis.poll()
        if self.pending_reference:
            try:
                for index in self.pending_reference:
                    self.axis(index).idle()
            except Rejected:
                pass
            else:
                refs = self.pending_reference.copy()
                for index, length in refs.items():
                    self.set_rest(index, length, from_pending=True)
                self.pending_reference.clear()
        self.recorder.pair(self.coordinates())

    def coordinates(self):
        if len(self.axes) != 2 or self.faulted:
            return None
        values = [a.absolute() for a in self.axes]
        if any(v is None for v in values) or any(time.monotonic()-a.last_read > FRESH_SECONDS for a in self.axes):
            return None
        mean, diff = modes_from_lengths(*values)
        times = [a.last_read for a in self.axes]
        return {'mean_mm':mean, 'half_difference_mm':diff, 'times':times, 'skew_s':abs(times[0]-times[1])}

    def enable(self, indices, reference_lengths=None):
        self.checkpoint()
        selected = [self.axis(i) for i in indices]
        for a in selected:
            a.fresh(); a.stationary()
            if a.state != 'DISABLED':
                raise Rejected(f'Cable {a.index+1}: already enabled or faulted')
            a.validate_count(a.count)
        if reference_lengths is not None:
            for length in reference_lengths.values():
                self.validate_rest_length(length, required=True)
        for a in selected:
            self.checkpoint(); a.drive.configure()
            a.rpm, a.speed_verified = a.cfg.motor_rpm, True
            self.poll_all()  # Refresh both axes between configuration sequences.
            a.stationary(); self.checkpoint()
            if not a.drive.target(a.count,self.cancelled):
                raise Cancelled()
            self.checkpoint(); a.drive.enable()
            a.enabled, a.disable_confirmed, a.state = True, False, 'HOLDING'
            a.target = a.count; a.stable.clear()
            a.message = 'Enabled: jog now. Set rest only for referenced/absolute targets.'
            self.recorder.event('enabled_hold_here', axis=a.index, count=a.count)
            self.poll_all()
        if reference_lengths is not None:
            self.pending_reference = dict(reference_lengths)
            self.recorder.event('reference_after_enable_requested', lengths_mm=reference_lengths)

    @staticmethod
    def validate_rest_length(length, required=False):
        if length is None and not required:
            return
        if length is None or finite(length,'Physical rest length') <= TRAVEL_MM:
            raise Rejected('Enter the measured physical rest length in mm, greater than 70 mm')

    def set_rest(self, index, length=None, from_pending=False):
        self.checkpoint(); a = self.axis(index)
        a.idle(); self.validate_rest_length(length)
        a.cfg.target_count(TRAVEL_MM,a.count); a.cfg.target_count(-TRAVEL_MM,a.count)
        if not a.drive.target(a.count,self.cancelled):
            raise Cancelled()
        a.rest, a.rest_length, a.target = a.count, length, a.count
        a.epoch += 1
        a.message = 'Rest recorded. ±70 mm window is now centred here.'
        self.recorder.event('rest_set', axis=index, count=a.rest, length_mm=length, epoch=a.epoch)

    def validate_moves(self, moves):
        self.checkpoint()
        if not moves or len({m.axis for m in moves}) != len(moves):
            raise Rejected('Expected one target per selected cable')
        if self.pending_reference:
            raise Rejected('Finishing the requested rest capture; wait or Stop')
        for move in moves:
            a = self.axis(move.axis); a.idle()
            a.validate_count(move.count); a.cfg.rpm_for_speed(move.speed)

    def dispatch(self, moves):
        self.validate_moves(moves)  # Every endpoint validated before any write.
        for move in moves:  # Prepare and verify every speed before committing targets.
            a = self.axis(move.axis); rpm = a.cfg.rpm_for_speed(move.speed)
            self.checkpoint()
            if not a.speed_verified or a.rpm != rpm:
                a.speed_verified = False
                if not a.drive.set_speed(rpm,self.cancelled):
                    raise Cancelled()
                a.rpm, a.speed_verified = rpm, True
                self.recorder.event('speed_applied', axis=a.index, motor_rpm=rpm,
                                    requested_mm_s=move.speed, nominal_mm_s=a.cfg.speed_for_rpm(rpm))
            self.poll_all()  # Do not let dual preparation starve feedback.
        self.validate_moves(moves)
        dispatch_times = []
        for move in moves:
            self.checkpoint(); a = self.axis(move.axis)
            distance = abs(move.count-a.count)*a.cfg.mm_per_count
            self.recorder.event('move_requested', axis=a.index, kind=move.kind, from_count=a.count,
                target_count=move.count, requested_mm_s=move.speed, motor_rpm=a.rpm,
                nominal_mm_s=a.cfg.speed_for_rpm(a.rpm), rest_count=a.rest)
            a.target, a.state = move.count, 'MOVING'
            a.deadline = time.monotonic()+2*distance/a.cfg.speed_for_rpm(a.rpm)+6
            a.stable.clear()
            started = time.monotonic()
            if not a.drive.target(move.count,self.cancelled):
                raise Cancelled()
            self.checkpoint(); a.drive.enable()
            dispatch_times.append((a.index, started, time.monotonic()))
            a.message = 'Moving; selected speed applies to this move.'
        if len(moves) == 2:
            self.recorder.event('pair_dispatched', axis_order=[x[0] for x in dispatch_times],
                 first_to_second_dispatch_s=dispatch_times[1][1]-dispatch_times[0][1],
                 note='Host sequential dispatch timing, not motor start skew')

    def jog(self, index, delta, speed):
        a = self.axis(index); a.idle()
        delta = finite(delta,'Jog step')
        if delta == 0:
            raise Rejected('Jog step must be nonzero')
        reference, limit = a.envelope()
        target = a.cfg.target_count(a.cfg.offset_mm(a.count,reference)+delta,reference,limit)
        self.dispatch([Move(index,target,speed,'jog')])

    def move_axis(self, index, value, speed, absolute=False):
        a = self.axis(index)
        if a.rest is None or (absolute and a.rest_length is None):
            raise Rejected('Set a physical rest reference before length targets')
        offset = finite(value,'Length target')-(a.rest_length if absolute else 0)
        self.dispatch([Move(index,a.cfg.target_count(offset,a.rest),speed,'absolute' if absolute else 'relative')])

    def plan_modes(self, mean, difference, speed):
        if len(self.axes) != 2:
            raise Rejected('Mean/difference control requires two active cables')
        lengths = lengths_from_modes(mean,difference)
        result = []
        for a,length in zip(self.axes,lengths):
            if a.rest is None or a.rest_length is None:
                raise Rejected(f'Cable {a.index+1}: set rest and its measured length first')
            try:
                target=a.cfg.target_count(length-a.rest_length,a.rest)
            except Rejected as exc:
                raise Rejected(f'Cable {a.index+1}: requested length {length:g} mm; allowed {a.rest_length-TRAVEL_MM:g}..{a.rest_length+TRAVEL_MM:g} mm') from exc
            result.append(Move(a.index,target,speed,'mean_difference'))
        self.validate_moves(result)
        return result

    def move_modes(self, mode, mean, difference, speed):
        current = self.coordinates()
        if current is None:
            raise Rejected('Both cables need physical rest lengths and fresh feedback')
        if mode == 'mean':
            difference = current['half_difference_mm']
        elif mode == 'difference':
            mean = current['mean_mm']
        elif mode != 'both':
            raise Rejected('Unknown coordinate mode')
        moves = self.plan_modes(mean,difference,speed)
        self.recorder.event('coordinate_request', mode=mode, mean_mm=mean,
                            half_difference_mm=difference, speed_mm_s=speed)
        self.dispatch(moves)

    def stop_all(self):
        self.pending_reference.clear()
        errors = []
        for a in self.axes:
            if a.enabled:
                try:
                    a.drive.hold(); a.target = None; a.state = 'STOPPING'
                    a.deadline = time.monotonic()+6; a.stable.clear()
                    a.message = 'Stop requested; waiting for stationary feedback.'
                except Exception as exc:
                    errors.append(str(exc))
        if errors:
            raise DriveFault('Stop unconfirmed: '+'; '.join(errors))
        self.recorder.event('stop_all_requested')

    def release(self, indices):
        self.pending_reference.clear()
        errors = []
        for index in indices:
            a = self.axis(index)
            try:
                a.drive.disable(); a.disable_confirmed = True
            except Exception as exc:
                a.disable_confirmed = False; errors.append(str(exc))
            a.enabled = False; a.rest = a.rest_length = a.target = None
            a.epoch += 1; a.stable.clear(); a.deadline = None
            a.state = 'FAULT' if self.faulted else 'DISABLED'
            a.message = 'Torque off. Rest cleared; connection displacement is retained.'
        if errors:
            raise DriveFault('Disable unconfirmed: '+'; '.join(errors))
        self.recorder.event('released', axes=indices)

    def fault(self, reason):
        self.faulted, self.fault_reason = True, str(reason)
        self.pending_reference.clear()
        # Attempt BOTH disables even when one drive or the log is unreachable.
        for a in self.axes:
            try:
                a.drive.disable(); a.disable_confirmed = True
            except Exception:
                a.disable_confirmed = False
            a.enabled = False; a.rest = a.rest_length = a.target = None
            a.state = 'FAULT'; a.epoch += 1; a.stable.clear()
        try:
            self.recorder.event('fault', reason=str(reason), disable_confirmed=[a.disable_confirmed for a in self.axes])
        except Exception:
            pass

    def snapshot(self):
        return {'axes':[a.snapshot() for a in self.axes], 'coordinates':self.coordinates(),
                'faulted':self.faulted, 'fault_reason':self.fault_reason,
                'pending_reference':bool(self.pending_reference), 'log_path':str(self.recorder.path)}
