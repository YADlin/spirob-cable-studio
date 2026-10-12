"""No serial hardware. Real HTTP, capture threads, files and existing demo worker."""
import csv
import json
from pathlib import Path
import secrets
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from spirob_cable.config import RigSettings
from spirob_experiment.camera import CameraService, Capture, SyntheticCamera, serve
from spirob_experiment.camera_client import CameraClient
from spirob_experiment.schema import Trial
from spirob_experiment.runner import acquire
from spirob_experiment.derive import camera_to_coordinator
from spirob_experiment.storage import checksums
from spirob_experiment.analysis import stiffness_profile, keypoint_cost


def plan():
    return {"schema_version": 1, "name": "test", "rest_lengths_mm": [220, 220],
            "speed_mm_s": 10, "repetitions": 2, "pre_roll_s": .05, "post_roll_s": .05,
            "timeout_s": 15, "steps": [
                {"cable_1_mm": 220.1, "cable_2_mm": 219.9, "hold_s": .05},
                {"cable_1_mm": 220, "cable_2_mm": 220, "hold_s": .05}]}


class SchemaTests(unittest.TestCase):
    def test_explicit_lengths_translate_once_to_legacy_half_difference(self):
        trial = Trial.from_dict(plan())
        a, d, _ = trial.procedure()["rows"][0]
        self.assertAlmostEqual(a+d, 220.1)
        self.assertAlmostEqual(a-d, 219.9)
        self.assertEqual(len(trial.expanded()), 4)
        self.assertEqual(Trial.from_dict(trial.data()), trial)

    def test_invalid_last_target_nan_boolean_or_extra_field_rejected(self):
        for key, value in (("cable_1_mm", 291), ("cable_2_mm", float("nan")), ("hold_s", True)):
            data = plan()
            data["steps"][-1][key] = value
            with self.assertRaises(ValueError):
                Trial.from_dict(data)
        data = plan()
        data["difference"] = 1
        with self.assertRaises(ValueError):
            Trial.from_dict(data)

    def test_config_reference_mismatch_rejected(self):
        data = plan()
        data["rest_lengths_mm"][0] = 221
        with self.assertRaisesRegex(ValueError, "rest lengths"):
            Trial.from_dict(data).validate_rig(RigSettings.demo())

    def test_two_clock_endpoints_map_offset_and_drift(self):
        a = {"best": {"coordinator_mid_s": 100., "offset_s": 10.}}
        b = {"best": {"coordinator_mid_s": 200., "offset_s": 10.1}}
        self.assertAlmostEqual(camera_to_coordinator(160.05, a, b), 150)
        self.assertEqual(camera_to_coordinator(110, a), 100)
        b["best"]["offset_s"] = 20
        with self.assertRaises(ValueError):
            camera_to_coordinator(160, a, b)

    def test_stiffness_coordinate_convention_explicit(self):
        values = stiffness_profile(2, 1, [0, .5, 1])
        self.assertEqual(values[0], 2)
        self.assertGreater(values[1], values[2])
        for coords in ([1, 2], [0, 2, 1], []):
            with self.assertRaises(ValueError):
                stiffness_profile(2, 1, coords)


class AcquisitionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.servers = []

    def tearDown(self):
        for server, thread, service in self.servers:
            if service.capture:
                service.capture.stop()
            server.shutdown()
            server.server_close()
            thread.join(3)
        self.tmp.cleanup()

    def client(self, root=None, factory=None):
        token = secrets.token_hex(32)
        service = CameraService(root or self.root, token, factory or (lambda: SyntheticCamera(320, 200, 60)))
        server = serve(service, port=0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.servers.append((server, thread, service))
        return CameraClient(f"http://127.0.0.1:{server.server_port}", token), service

    def test_complete_repeated_trial_has_all_inputs_frames_and_confirmed_cleanup(self):
        client, service = self.client()
        path = acquire(Trial.from_dict(plan()), self.root, client, RigSettings.demo(), demo=True, min_fps=10)
        run = json.loads((path/"run.json").read_text())
        self.assertEqual(run["state"], "COMPLETE")
        self.assertIn("spirob_experiment/runner.py", run["software"]["source_sha256"])
        self.assertIn("opencv-python-headless", run["camera_ready"]["source"]["software"]["dependencies"])
        self.assertEqual(run["motor_shutdown"]["disable_confirmed"], [True, True])
        self.assertTrue(run["camera_shutdown"]["thread_exited"])
        self.assertFalse(service.capture.thread.is_alive())
        events = [json.loads(x) for x in (path/run["motor_log"]/"events.jsonl").read_text().splitlines()]
        self.assertIn({"event": "experiment_completed", "steps": 4},
                      [{k: v for k, v in e.items() if k != "t_s"} for e in events])
        with (path/"derived/selected_frames.csv").open() as f:
            frames = list(csv.DictReader(f))
        self.assertEqual(len(frames), 6)
        for frame in frames:
            self.assertTrue((path/frame["file"]).exists())
            self.assertLess(abs(float(frame["selection_error_s"])), .08)
        with (path/"derived/commands.csv").open() as f:
            commands = list(csv.DictReader(f))
        self.assertEqual(len(commands), 8)
        self.assertAlmostEqual(float(commands[0]["target_length_m"]), .2201, places=5)
        hashed = checksums(path)
        self.assertIn("trial.json", hashed)
        self.assertEqual(len(hashed["trial.json"]), 64)

    def test_camera_failure_during_trial_stops_worker_and_marks_failed(self):
        class FailingSource(SyntheticCamera):
            def read(self):
                if self.index >= 100:
                    raise OSError("Injected camera disconnect")
                return super().read()
        client, _ = self.client(factory=lambda: FailingSource(320, 200, 60))
        data = plan()
        data["speed_mm_s"] = 1
        data["steps"][0]["cable_1_mm"] = 230
        with self.assertRaisesRegex(RuntimeError, "Camera stopped"):
            acquire(Trial.from_dict(data), self.root, client, RigSettings.demo(), demo=True, min_fps=10)
        path = next(self.root.glob("*DEMO*"))
        run = json.loads((path/"run.json").read_text())
        self.assertEqual(run["state"], "FAILED")
        self.assertEqual(run["motor_shutdown"]["disable_confirmed"], [True, True])
        events = (path/run["motor_log"]/"events.jsonl").read_text()
        self.assertIn("experiment_started", events)
        self.assertIn("experiment_aborted", events)
        self.assertNotIn("experiment_completed", events)

    def test_shared_root_mismatch_prevents_motor_start(self):
        camera_root = self.root/"different"
        client, _ = self.client(root=camera_root)
        with patch("spirob_experiment.runner.Motor") as motor:
            with self.assertRaisesRegex(RuntimeError, "storage root"):
                acquire(Trial.from_dict(plan()), self.root/"coordinator", client,
                        RigSettings.demo(), demo=True, min_fps=10)
            motor.assert_not_called()

    def test_hardware_rejects_synthetic_camera_before_opening_motor(self):
        client, _ = self.client()
        with patch("spirob_experiment.runner.Motor") as motor:
            with self.assertRaisesRegex(RuntimeError, "real camera"):
                acquire(Trial.from_dict(plan()), self.root, client,
                        RigSettings.demo(), demo=False, min_fps=10)
            motor.assert_not_called()

    def test_unauthorized_request_and_path_traversal_rejected(self):
        client, _ = self.client()
        client.token = "wrong"*8
        with self.assertRaises(RuntimeError):
            client.clock(1)
        with self.assertRaises(ValueError):
            Capture(self.root, "../escape", "a"*32, SyntheticCamera)

    def test_camera_lease_expiry_and_reused_session_are_not_silent(self):
        capture = Capture(self.root, "lease_test", "b"*32,
                          lambda: SyntheticCamera(320, 200, 60), lease_s=.1)
        capture.thread.join(3)
        self.assertEqual(capture.status(False)["state"], "FAILED")
        self.assertIn("lease expired", capture.status(False)["error"])
        with self.assertRaises(FileExistsError):
            Capture(self.root, "lease_test", "b"*32, SyntheticCamera)

    def test_lost_start_response_still_stops_camera(self):
        client, service = self.client()
        original = client.start
        def lost_response(*args):
            original(*args)
            raise TimeoutError("Injected lost start response")
        client.start = lost_response
        with self.assertRaisesRegex(RuntimeError, "lost start response"):
            acquire(Trial.from_dict(plan()), self.root, client, demo=True, min_fps=10)
        self.assertFalse(service.capture.thread.is_alive())


class CostTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.meta = self.root/"metadata.json"
        self.meta.write_text(json.dumps({"units": "m", "coordinate_frame": "base_xy",
                                         "time_origin": "session", "required_points": ["tip"]}))
        self.obs, self.sim = self.root/"observed.csv", self.root/"sim.csv"

    def tearDown(self):
        self.tmp.cleanup()

    def test_sse_and_rmse_use_measured_times(self):
        self.obs.write_text("t_s,point,x,y,valid\n0,tip,0,0,1\n0.05,tip,1,0,1\n")
        self.sim.write_text("t_s,point,x,y,valid\n0,tip,0,0,1\n0.1,tip,4,0,1\n")
        result = keypoint_cost(self.obs, self.sim, self.meta)
        self.assertEqual(result["sse"], 1)
        self.assertAlmostEqual(result["point_distance_rmse"], 2**-.5)

    def test_missing_simulation_cannot_improve_cost(self):
        self.obs.write_text("t_s,point,x,y,valid\n0,tip,0,0,1\n0.05,tip,1,0,1\n")
        self.sim.write_text("t_s,point,x,y,valid\n0,tip,0,0,1\n")
        with self.assertRaisesRegex(ValueError, "no extrapolation"):
            keypoint_cost(self.obs, self.sim, self.meta)

    def test_invalid_physical_points_fail_coverage_gate(self):
        self.obs.write_text("t_s,point,x,y,valid\n0,tip,0,0,1\n0.05,tip,,,0\n")
        self.sim.write_text("t_s,point,x,y,valid\n0,tip,0,0,1\n")
        with self.assertRaisesRegex(ValueError, "coverage"):
            keypoint_cost(self.obs, self.sim, self.meta)

    def test_large_interpolation_gap_rejected(self):
        self.obs.write_text("t_s,point,x,y,valid\n0.5,tip,1,0,1\n")
        self.sim.write_text("t_s,point,x,y,valid\n0,tip,0,0,1\n1,tip,2,0,1\n")
        with self.assertRaisesRegex(ValueError, "gap"):
            keypoint_cost(self.obs, self.sim, self.meta)


if __name__ == "__main__":
    unittest.main()
