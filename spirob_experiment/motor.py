"""Reuse the existing controller worker. This module never imports Qt widgets."""
import queue
import time


class Motor:
    def __init__(self, cfg, demo, session):
        from spirob_cable.worker import Worker
        self.messages = queue.Queue()
        self.worker = Worker(cfg, demo, session.path/"motor", self.messages, session.data["trial"]["name"])
        self.session = session
        self.snapshot = {}
        self.events = []
        self.pending = False
        self.closed = None
        self.origin = None
        self.samples_seen = [0, 0]
        self.worker.start()

    def tick(self, strict=True):
        self.worker.heartbeat = time.monotonic()
        errors = []
        while True:
            try:
                item = self.messages.get_nowait()
            except queue.Empty:
                break
            kind = item["kind"]
            if kind == "snapshot":
                self.snapshot = item
                if self.origin is None:
                    # The recorder origin is immutable after construction.
                    self.origin = self.worker.rig.recorder.started
                    self.session.data["motor_origin_s"] = self.origin-self.session.started
                    self.session.data["motor_log"] = str(
                        self.worker.rig.recorder.path.relative_to(self.session.path))
                if item.get("faulted"):
                    errors.append(item["fault_reason"])
            elif kind == "sample":
                self.samples_seen[item["data"]["axis"]] += 1
            elif kind == "event":
                self.events.append(item["data"])
            elif kind == "command_done":
                self.pending = False
            elif kind == "rejected":
                self.pending = False
                errors.append(item["text"])
            elif kind == "closed":
                self.closed = item
                if strict:
                    errors.append("Motor worker closed unexpectedly")
        if not self.worker.is_alive() and self.closed is None:
            errors.append("Motor worker exited without a close acknowledgement")
        if strict and errors:
            raise RuntimeError("; ".join(errors))

    def submit(self, name, data=None):
        if self.pending:
            raise RuntimeError("Motor command still pending")
        self.worker.submit(name, data)
        self.pending = True

    @property
    def connected(self):
        return (len(self.snapshot.get("axes", [])) == 2 and min(self.samples_seen) >= 3
                and all(a["state"] == "DISABLED" for a in self.snapshot["axes"]))

    @property
    def referenced(self):
        return (not self.pending and self.snapshot.get("coordinates") is not None
                and not self.snapshot.get("pending_reference")
                and all(a["state"] == "HOLDING" for a in self.snapshot["axes"]))

    def shutdown(self):
        """Best effort bounded shutdown; never claim an acknowledgement not received."""
        self.worker.submit("stop")
        deadline = time.monotonic()+7
        while self.worker.is_alive() and time.monotonic() < deadline:
            self.tick(strict=False)
            axes = self.snapshot.get("axes", [])
            if axes and all(a["state"] in ("HOLDING", "DISABLED", "FAULT") for a in axes):
                # stop may still be pending: give its event priority over close.
                if not self.worker.stop_request.is_set():
                    break
            time.sleep(.02)
        self.worker.submit("close")
        deadline = time.monotonic()+5
        while self.worker.is_alive() and time.monotonic() < deadline:
            self.tick(strict=False)
            self.worker.join(.02)
        self.tick(strict=False)
        return {"worker_exited": not self.worker.is_alive(),
                "disable_confirmed": (self.closed or {}).get("disable_confirmed", [False, False])}
