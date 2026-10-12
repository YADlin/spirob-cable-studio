"""Bounded camera calls and explicit host-to-host clock mapping."""
import json
import time
from urllib.request import Request, build_opener, ProxyHandler
from urllib.error import HTTPError


class CameraClient:
    def __init__(self, url, token, timeout=.6):
        if not url.startswith("http://"):
            raise ValueError("Use an http:// camera address on a trusted local link")
        if len(token) < 32:
            raise ValueError("Camera token must have at least 32 characters")
        self.url, self.token, self.timeout = url.rstrip("/"), token, timeout
        self.identity = {}
        self.opener = build_opener(ProxyHandler({}))

    def call(self, endpoint, data=None):
        request = Request(self.url+endpoint, json.dumps(data or {}).encode(),
                          {"Authorization": "Bearer "+self.token, "Content-Type": "application/json"})
        try:
            with self.opener.open(request, timeout=self.timeout) as response:
                return json.load(response)
        except HTTPError as exc:
            raise RuntimeError(exc.read(4096).decode(errors="replace")) from exc

    def clock(self, count=7):
        samples = []
        for _ in range(count):
            start = time.monotonic()
            reply = self.call("/clock")
            end = time.monotonic()
            samples.append({"coordinator_mid_s": (start+end)/2, "round_trip_s": end-start,
                            "offset_s": reply["camera_monotonic_s"]-(start+end)/2})
        best = min(samples, key=lambda s: s["round_trip_s"])
        return {"best": best, "samples": samples,
                "offset_definition": "camera_monotonic - coordinator_monotonic",
                "network_uncertainty_s": best["round_trip_s"]/2,
                "limitation": "Network interval only; excludes exposure, USB buffering and scheduling latency"}

    def start(self, session_id, nonce):
        self.identity = {"session_id": session_id, "nonce": nonce}
        return self.call("/start", self.identity)

    def status(self):
        return self.call("/status", self.identity)

    def stop(self):
        # Stopping can take up to two seconds while a read finishes.
        timeout = self.timeout
        try:
            self.timeout = 3
            return self.call("/stop", self.identity)
        finally:
            self.timeout = timeout
