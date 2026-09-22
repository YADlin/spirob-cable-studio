"""One serial worker for both IDs, with one client per distinct serial port."""
import queue
import threading
import time
from .drive import SerialDrive, DemoDrive, DriveFault
from .records import Recorder, AxisLog
from .rig import Rig, Cancelled
from .model import Rejected
from .procedure import Procedure


def make_drives(settings, demo, recorder):
    clients, drives = {}, []
    for i, cfg in enumerate(settings.selected):
        log = AxisLog(recorder,i)
        if demo:
            d = DemoDrive(cfg,log)
        else:
            d = SerialDrive(cfg,log,client=clients.get(cfg.port))
            clients[cfg.port] = d.client
        drives.append(d)
    return drives


class Worker(threading.Thread):
    def __init__(self, settings, demo, log_root, messages, name=''):
        super().__init__(daemon=True, name='dual-drive-owner')
        self.settings, self.demo, self.log_root, self.messages, self.run_name = settings,demo,log_root,messages,name
        self.commands = queue.Queue(maxsize=1)
        self.stop_request = threading.Event(); self.close_request = threading.Event(); self.release_request = threading.Event()
        self.heartbeat = time.monotonic()
        self.rig = self.procedure = None

    def cancelled(self):
        return self.stop_request.is_set() or self.release_request.is_set() or self.close_request.is_set()

    def clear(self):
        while True:
            try: self.commands.get_nowait()
            except queue.Empty: return

    def submit(self, name, data=None):
        priority = {'stop':self.stop_request,'release_all':self.release_request,'close':self.close_request}
        if name in priority:
            priority[name].set(); self.clear(); return
        if self.cancelled():
            raise Rejected('Stop/release/disconnect pending')
        try: self.commands.put_nowait((name,data or {}))
        except queue.Full as exc: raise Rejected('Wait for the previous command') from exc

    def publish(self, kind, **data):
        self.messages.put({'kind':kind,**data})

    def execute(self, name, data):
        r = self.rig
        if self.procedure.active and name not in ('note','demo_fault'):
            raise Rejected('Stop the experiment before manual motion')
        if r.faulted and name != 'note':
            raise Rejected('Fault latched: disconnect before restarting')
        if name == 'enable':
            r.enable(data['indices'],data.get('reference_lengths'))
        elif name == 'rest': r.set_rest(data['axis'],data.get('length_mm'))
        elif name == 'release': r.release([data['axis']])
        elif name == 'jog': r.jog(data['axis'],data['delta_mm'],data['speed_mm_s'])
        elif name == 'move': r.move_axis(data['axis'],data['value_mm'],data['speed_mm_s'],data.get('absolute',False))
        elif name == 'modes': r.move_modes(data['mode'],data['mean_mm'],data['difference_mm'],data['speed_mm_s'])
        elif name == 'experiment': self.procedure.start(data)
        elif name == 'note':
            note = str(data.get('text','')).strip()
            if not note or len(note)>1000: raise Rejected('Use 1..1000 characters')
            r.recorder.event('annotation',text=note)
        elif name == 'demo_fault' and self.demo: r.axis(data['axis']).drive.failed = True
        else: raise Rejected('Unknown command')

    def run(self):
        recorder = None; drives = []
        try:
            recorder = Recorder(self.log_root,self.settings,self.demo,self.publish,self.run_name)
            drives = make_drives(self.settings,self.demo,recorder)
            self.rig = Rig(self.settings,drives,recorder,self.cancelled)
            self.procedure = Procedure(self.rig)
            self.rig.connect()
            while not self.close_request.is_set():
                try:
                    if self.release_request.is_set():
                        self.clear(); self.procedure.abort('release')
                        self.rig.release(list(range(len(drives))))
                        self.release_request.clear(); self.stop_request.clear()
                    elif self.stop_request.is_set():
                        self.clear(); self.procedure.abort('stop'); self.rig.stop_all()
                        self.stop_request.clear()
                    if not self.rig.faulted:
                        if any(a.enabled for a in self.rig.axes) and time.monotonic()-self.heartbeat > 1.5:
                            raise DriveFault('GUI heartbeat lost; both references invalidated')
                        self.rig.poll_all()
                    try: name,data = self.commands.get_nowait()
                    except queue.Empty: name = None
                    if name:
                        self.execute(name,data)
                        self.publish('command_done')
                    self.procedure.tick()
                except Rejected as exc:
                    self.publish('rejected',text=str(exc))
                except Cancelled:
                    self.publish('command_done')
                except Exception as exc:
                    self.clear()
                    self.stop_request.clear(); self.release_request.clear()
                    try: self.procedure.abort('fault')
                    except Exception: pass
                    self.rig.fault(str(exc)); self.publish('command_done')
                self.publish('snapshot',**self.rig.snapshot(),procedure=self.procedure.snapshot())
                self.close_request.wait(.05)
        except Cancelled:
            pass
        except Exception as exc:
            if self.rig:
                self.rig.fault(str(exc)); self.publish('snapshot',**self.rig.snapshot())
            self.publish('rejected',text=str(exc))
        finally:
            if self.procedure:
                try: self.procedure.abort('disconnect')
                except Exception: pass
            confirmed = []
            for drive in drives:
                try: drive.disable(); confirmed.append(True)
                except Exception: confirmed.append(False)
            # Only close ports once every disable attempt has been made.
            for drive in drives:
                try: drive.close()
                except Exception: pass
            if recorder:
                try: recorder.event('disconnected',disable_confirmed=confirmed); recorder.close()
                except Exception: pass
            self.publish('closed',disable_confirmed=confirmed)
