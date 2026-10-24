"""Offscreen check of the RL window (contract v2): dry run, armed run at a chosen speed, auto-continue
with a human replacing the egg, ownership/Stop.

usage: QT_QPA_PLATFORM=offscreen uv run --frozen --extra live python tests/verify_rl_window.py
"""
import json, sys, tempfile, time
from pathlib import Path
from unittest.mock import patch
import numpy as np
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT))
from PySide6.QtWidgets import QApplication, QMessageBox
app = QApplication.instance() or QApplication([])
from rl import contract as C
from rl.hardware_io import CableMap, CameraMap
from rl.rl_window import RLWindow
from rl.twin import Twin

sys.argv = [sys.argv[0], "--quick"]
src = (ROOT / "tests" / "verify_rl_knockdown.py").read_text().split("# ------------------------------------------------------------------ 4-7. closed loop")[0]
g = {"__file__": str(ROOT / "tests" / "verify_rl_knockdown.py")}
exec(compile(src, "verify_rl_knockdown", "exec"), g)          # fixtures: synthetic bundle, SETUP, LIMITS, colours
results = []
def check(name, ok, detail=""):
    results.append(ok); print(("PASS " if ok else "FAIL ") + name + (f"  [{detail}]" if detail else ""), flush=True)

class Session:
    def __init__(self, twin, log_dir): self.twin, self.log_dir = twin, log_dir
    def send(self, action, **kw): return self.twin.send(action, **kw)

class Host:
    def __init__(self, twin, tmp):
        self.twin = twin; self.config = json.loads((ROOT / "config.json").read_text())
        self.fit = json.loads((ROOT / "calibration" / "new_model_initial.json").read_text())
        self.fit["candidate"]["parameters"].update(mean_offset_mm=0.0, difference_offset_mm=0.0)
        self.comparison_setup = dict(g["SETUP"]); self.motion_profile = dict(g["LIMITS"])
        self.session = Session(twin, tmp); self.remote = {}; self.remote_received = time.monotonic()
    def manual_source(self): return self.twin.item()
    def campaign_active(self): return False
    def command(self, action, **kw):
        if action == "hold": w.abort("User pressed Stop / Hold")

tmp = Path(tempfile.mkdtemp())
cable = CableMap.from_lab(g["SETUP"], g["PARAMS"], g["RESTS"], (C.MEAN_PAYOUT_M, C.MEAN_PAYOUT_M))
twin = Twin(cable, CameraMap.from_lab(g["SETUP"]), (0.08, 0.11))
host = Host(twin, tmp)
with patch("rl.rl_window.SETTINGS", tmp / "rl_vision.json"):
    w = RLWindow(host); w.show()
    w.settings = {"colour": g["egg_colour"].__dict__, "pedestal_colour": g["cup_colour"].__dict__}
    w.bundle = g["bundle"]
    def pump(seconds):
        for _ in range(int(round(seconds / 0.1))):
            twin.advance(0.1)
            host.remote = dict(twin.remote(), motion_limits=g["LIMITS"]); host.remote_received = time.monotonic()
            for e in twin.events: w.motor_event(e)
            twin.events.clear(); w.tick(); app.processEvents()
    pump(0.8)
    w.placement.setCurrentText("P01"); pump(0.3); w.run_button.click()
    check("start refused when the egg is not on the selected placement", w.runner is None or not w.active, w.status.text()[:90])
    w.placement.setCurrentText("free"); pump(0.3)
    w.run_button.click(); pump(5.0)
    check("dry-run episode logs decisions without motor commands",
          w.runner.log is not None and len(w.runner.log.decisions) >= 3 and not any(c["action"] == "independent" for c in twin.commands),
          f"{len(w.runner.log.decisions)} decisions")
    w.stop_button.click(); app.processEvents()
    check("Stop / Hold ends the dry-run episode", not w.active and w.runner.outcome == "ABORTED", str(w.runner.outcome))
    w.dry.setChecked(False); w.speed.setCurrentText("10 mm/s"); w.auto.setChecked(True)
    with patch.object(QMessageBox, "question", return_value=QMessageBox.Yes):
        w.arm_button.click()
    check("arming requires explicit confirmation", w.armed)
    w.run_button.click(); pump(25.0)
    speeds = [c["speeds_mm_s"][0] for c in twin.commands if c["action"] == "independent"]
    check("armed episode at 10 mm/s knocks the egg down and then waits", w.history[-1]["outcome"] == "SUCCESS" and w.runner.waiting
          and max(speeds) <= 10.0 + 1e-9, f"{w.history[-1]['outcome']}, max {max(speeds):.1f} mm/s")
    check("host treats the waiting runner as the motion owner", w.active)
    twin.place_new_egg((-0.09, 0.15)); pump(4.0)
    check("auto-continue starts the next episode after a still scene", len(w.runner.episodes) == 1 and w.runner.active,
          w.status.text()[:80])
    pump(30.0)
    check("auto-started episode succeeds and shows in the table", w.table.rowCount() >= 3 and w.history[-1]["placement"] == "auto"
          and w.history[-1]["outcome"] == "SUCCESS", f"{w.history[-1]}")
    w.stop_button.click(); app.processEvents()
    check("Stop ends auto-continue", not w.active)
    w.grab().save(str(tmp / "rl_window.png"))
print(f"{sum(results)}/{len(results)} checks passed; screenshot {tmp/'rl_window.png'}")
sys.exit(0 if all(results) else 1)
