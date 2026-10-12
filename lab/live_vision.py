"""One vision worker with one waiting frame; never blocks MuJoCo or camera I/O."""
import queue
import threading
import time


class VisionWorker(threading.Thread):
    def __init__(self, measure, config):
        super().__init__(daemon=True, name='spirob-image-measurement')
        self.measure, self.config = measure, config
        self.pending = queue.Queue(maxsize=1)
        self.finished = queue.Queue(maxsize=1)
        self.stop = threading.Event()

    @staticmethod
    def replace(queue_, item):
        try:queue_.get_nowait()
        except queue.Empty:pass
        try:queue_.put_nowait(item)
        except queue.Full:pass

    def submit(self, item, tracker, epoch):
        self.replace(self.pending,(item,tracker,epoch))

    def clear_pending(self):
        try:self.pending.get_nowait()
        except queue.Empty:pass

    def run(self):
        while not self.stop.is_set():
            try:item,tracker,epoch=self.pending.get(timeout=.1)
            except queue.Empty:continue
            began=time.monotonic();observed=None;error=None
            try:
                packet=item['packet'];key=(packet['session'],packet['frame_index'])
                if tracker:observed=tracker.update(item['rgb'],key)
                else:_,observed=self.measure(item['rgb'],self.config)
            except Exception as exc:error=type(exc).__name__+': '+str(exc)
            self.replace(self.finished,(item,observed,error,time.monotonic()-began,epoch))
