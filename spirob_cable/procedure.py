"""Finite endpoint experiments, owned by the serial worker."""
import time
from .model import Rejected, finite
from .rig import Move
from .drive import DriveFault


class Procedure:
    def __init__(self, rig):
        self.rig = rig; self.active = False; self.phase = 'IDLE'
        self.index = 0; self.steps = []; self.hold_until = None; self.epochs = []

    def start(self, data):
        if self.active:
            raise Rejected('An experiment is already active')
        rows, repeats, kind = data['rows'], data['repeats'], data['kind']
        speed = finite(data['speed_mm_s'],'Experiment speed')
        if type(repeats) is not int or not 1 <= repeats <= 20 or not isinstance(rows,list) or not 1 <= len(rows) <= 100 or len(rows)*repeats > 200:
            raise Rejected('Use 1..100 rows, 1..20 repeats and at most 200 moves')
        if kind not in ('pair','cable1','cable2'):
            raise Rejected('Unknown experiment coordinate type')
        steps = []
        for row in rows:
            if not isinstance(row,list) or len(row) != (3 if kind == 'pair' else 2):
                raise Rejected('Incorrect experiment row shape')
            hold = finite(row[-1],'Hold time')
            if not 0 <= hold <= 120:
                raise Rejected('Hold time must be 0..120 seconds')
            if kind == 'pair':
                moves = self.rig.plan_modes(row[0],row[1],speed)
            else:
                a = self.rig.axis(int(kind[-1])-1)
                if a.rest is None:
                    raise Rejected('Set rest before referenced experiments')
                moves = [Move(a.index,a.cfg.target_count(row[0],a.rest),speed,'experiment')]
                self.rig.validate_moves(moves)
            steps.append((moves,hold))
        self.rig.recorder.event('experiment_started', procedure=data)
        self.steps = steps*repeats; self.index = 0
        self.epochs = [a.epoch for a in self.rig.axes]
        self.active, self.phase = True, 'NEXT'

    def abort(self, reason):
        was_active = self.active
        self.active = False
        if was_active:
            self.phase = 'ABORTED'
            self.rig.recorder.event('experiment_aborted',reason=reason,step=self.index+1)

    def tick(self):
        if not self.active or self.rig.cancelled():
            return
        if self.epochs != [a.epoch for a in self.rig.axes] or self.rig.faulted:
            raise DriveFault('Experiment reference changed')
        if self.phase == 'NEXT':
            if self.index == len(self.steps):
                self.active, self.phase = False, 'COMPLETE'
                self.rig.recorder.event('experiment_completed',steps=len(self.steps)); return
            moves, _ = self.steps[self.index]
            self.rig.dispatch(moves); self.phase = 'MOVING'
        elif self.phase == 'MOVING':
            if all(self.rig.axes[m.axis].state == 'HOLDING' for m in self.steps[self.index][0]):
                self.hold_until = time.monotonic()+self.steps[self.index][1]
                self.phase = 'HOLDING'
                self.rig.recorder.event('experiment_hold',step=self.index+1,duration_s=self.steps[self.index][1])
        elif self.phase == 'HOLDING':
            for move in self.steps[self.index][0]:
                a = self.rig.axes[move.axis]; a.fresh()
                if abs(a.count-a.target) > a.tolerance:
                    raise DriveFault('Position drift during experiment hold')
            if time.monotonic() >= self.hold_until:
                self.index += 1; self.phase = 'NEXT'

    def snapshot(self):
        return {'active':self.active,'phase':self.phase,'step':min(self.index+1,len(self.steps)),'total':len(self.steps)}
