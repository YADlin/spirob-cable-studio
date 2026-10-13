"""Explicit per-run motion envelope, using the existing controller's checks.

The external motor repository is unchanged. A scoped Rig subclass supplies
the selected envelope to endpoint conversion AND encoder polling. The worker,
drive protocol, speed checks, stop priority and watchdog are inherited intact.
"""
from contextlib import contextmanager
import math
from types import MethodType
from motion_profile import validate_profile


def limits(rests, difference_limit_mm=70.):
    if isinstance(difference_limit_mm, dict):return validate_profile(difference_limit_mm, rests)
    if (len(rests) != 2 or any(not math.isfinite(r) or r <= 70 for r in rests) or
        isinstance(difference_limit_mm, bool) or not isinstance(difference_limit_mm, (int,float)) or
        not math.isfinite(difference_limit_mm) or not 1 <= difference_limit_mm <= min(rests)):
        raise ValueError('Maximum full difference must be 1 mm to the shorter configured rest length.')
    travel=max(70.,difference_limit_mm/2)
    return dict(difference_mm=float(difference_limit_mm), travel_mm=travel,
                absolute_bounds_mm=[[max(0.,float(r)-travel),float(r)+travel] for r in rests], max_speed_mm_s=20., setup_travel_mm=70.)


@contextmanager
def worker_limits(rests, difference_limit_mm):
    from spirob_cable import worker
    from spirob_cable.rig import Move
    from spirob_cable.model import Rejected, finite
    policy=limits(rests,difference_limit_mm)
    original=worker.Rig
    original_worker=worker.Worker
    class SessionRig(original):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            for axis in self.axes:
                def envelope(a):
                    return ((a.rest,policy['travel_mm']) if a.rest is not None
                            else (a.connection,a.cfg.setup_travel_mm))
                axis.envelope=MethodType(envelope,axis)
                old_validate=axis.validate_count
                old_poll=axis.poll
                def bounded(a,count,original_validate=old_validate):
                    original_validate(count)
                    if a.rest is not None:
                        lo,hi=policy['absolute_bounds_mm'][a.index]
                        value=a.absolute(count)
                        if not lo-a.cfg.mm_per_count <= value <= hi+a.cfg.mm_per_count:
                            raise Rejected(f'Cable {a.index+1}: target outside {lo:g}..{hi:g} mm')
                def checked_poll(a,original_poll=old_poll):
                    original_poll()
                    if a.enabled and a.rest is not None:
                        lo,hi=policy['absolute_bounds_mm'][a.index]
                        if not lo-a.cfg.mm_per_count <= a.absolute() <= hi+a.cfg.mm_per_count:
                            from spirob_cable.drive import DriveFault
                            raise DriveFault(f'Cable {a.index+1}: measured length outside session bounds')
                axis.validate_count=MethodType(bounded,axis)
                axis.poll=MethodType(checked_poll,axis)

        def jog_pair(self,deltas,speed):
            moves=[]
            for a,delta in zip(self.axes,deltas):
                a.idle();ref,travel=a.envelope()
                count=a.cfg.target_count(a.cfg.offset_mm(a.count,ref)+finite(delta,'Jog'),ref,travel)
                moves.append(Move(a.index,count,speed,'jog'))
            self.dispatch(moves)

        def move_axis(self,index,value,speed,absolute=False):
            a=self.axis(index)
            if a.rest is None or (absolute and a.rest_length is None):
                raise Rejected('Declare rest before absolute targets')
            offset=finite(value,'Length target')-(a.rest_length if absolute else 0)
            self.dispatch([Move(index,a.cfg.target_count(offset,a.rest,policy['travel_mm']),speed,'independent_absolute' if absolute else 'relative')])

        def move_independent(self,lengths,speeds):
            from actuation import independent_targets
            if any(a.rest is None or a.rest_length is None for a in self.axes):raise Rejected('Declare both rest lengths first')
            try:independent_targets(lengths,speeds,[a.rest_length for a in self.axes],difference_limit_mm)
            except ValueError as exc:raise Rejected(str(exc)) from exc
            moves=[]
            for a,length,speed in zip(self.axes,lengths,speeds):
                if a.rest is None or a.rest_length is None:raise Rejected('Declare both rest lengths first')
                moves.append(Move(a.index,a.cfg.target_count(length-a.rest_length,a.rest,policy['travel_mm']),speed,'independent_absolute'))
            # The inherited dispatcher validates BOTH endpoints/speeds before
            # preparing either target and retains stop/watchdog/fault handling.
            self.dispatch(moves)

        def plan_modes(self,mean,difference,speed):
            from spirob_cable.config import lengths_from_modes
            lengths=lengths_from_modes(mean,difference)
            moves=[]
            for a,length in zip(self.axes,lengths):
                if a.rest is None or a.rest_length is None: raise Rejected('Declare both rest lengths first')
                moves.append(Move(a.index,a.cfg.target_count(length-a.rest_length,a.rest,policy['travel_mm']),speed,'mean_difference'))
            self.validate_moves(moves)
            return moves

        def validate_moves(self,moves):
            super().validate_moves(moves)
            if any(m.speed>policy['max_speed_mm_s'] for m in moves):raise Rejected('Speed exceeds session ceiling')
            target={m.axis:m.count for m in moves}
            values=[a.absolute(target.get(a.index,a.count)) for a in self.axes]
            if all(v is not None for v in values):
                tolerance=sum(a.cfg.mm_per_count for a in self.axes)
                independent=all(m.kind=='independent_absolute' for m in moves)
                if min(values)<=0 or (not independent and abs(values[0]-values[1])>policy['difference_mm']+tolerance):
                    raise Rejected('Targets exceed the selected full difference or nonnegative length limit')

        def set_rest(self,index,length=None,from_pending=False):
            a=self.axis(index)
            for offset in (-policy['travel_mm'],policy['travel_mm']):
                a.cfg.target_count(offset,a.count,policy['travel_mm'])
            super().set_rest(index,length,from_pending)
            a.message=f"Rest recorded. ±{policy['travel_mm']:g} mm travel; maximum full difference {policy['difference_mm']:g} mm."

    class SessionWorker(original_worker):
        def execute(self,name,data):
            if name=='jog_pair':
                if self.procedure.active:raise Rejected('Stop the experiment before jogging')
                self.rig.checkpoint();self.rig.jog_pair(data['deltas_mm'],data['speed_mm_s']);return
            if name != 'independent':return super().execute(name,data)
            if self.procedure.active:raise Rejected('Stop the experiment before manual motion')
            self.rig.checkpoint()
            self.rig.move_independent(data['lengths_mm'],data['speeds_mm_s'])

    worker.Rig=SessionRig
    worker.Worker=SessionWorker
    try: yield policy
    finally:
        worker.Rig=original
        worker.Worker=original_worker
