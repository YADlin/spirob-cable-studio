import math
import unittest
from dataclasses import replace
from unittest.mock import Mock
from spirob_cable.model import Settings, Rejected, COUNTS_PER_REV, I32_MAX, turns
from spirob_cable.drive import SerialDrive, DriveFault
from spirob_cable.records import NullRecorder as NullJournal
CFG=Settings(device_id=11,diameter_basis="core",cable_diameter_mm=0.405,payout_sign=1)

class GeometryTests(unittest.TestCase):
    def test_27_mm_core_and_0405_mm_cable_conversion(self):
        CFG.validate()
        self.assertAlmostEqual(CFG.mm_per_rev, 86.09534667162828)
        self.assertAlmostEqual(1 / CFG.mm_per_count, 1551.7679545394794)
        self.assertEqual(CFG.motor_rpm, 34)
        self.assertLessEqual(CFG.actual_speed_mm_s, CFG.speed_mm_s)

    def test_core_diameter_includes_cable_centreline(self):
        cfg = replace(CFG, diameter_basis="core", cable_diameter_mm=0.405)
        cfg.validate()
        self.assertAlmostEqual(cfg.mm_per_rev, math.pi * 27.405)

    def test_calibrated_circumference_overrides_geometry(self):
        cfg = replace(CFG, calibrated_mm_per_rev=86, diameter_basis=None)
        cfg.validate()
        self.assertEqual(cfg.mm_per_rev, 86)

    def test_effective_diameter_does_not_add_cable_twice(self):
        cfg = replace(CFG, diameter_basis="effective")
        cfg.validate()
        self.assertAlmostEqual(cfg.mm_per_rev, math.pi * 27)

    def test_both_signs_and_signed_origins_round_trip(self):
        for sign in (-1, 1):
            cfg = replace(CFG, payout_sign=sign)
            for ref in (-3 * COUNTS_PER_REV, 4 * COUNTS_PER_REV):
                for mm in (-70, -1, 0, 1, 70):
                    target = cfg.target_count(mm, ref)
                    self.assertLessEqual(abs(cfg.offset_mm(target, ref)), 70)
                    self.assertAlmostEqual(cfg.offset_mm(target, ref), mm, delta=cfg.mm_per_count)

    def test_full_span_exceeds_one_turn_without_wrapping(self):
        ref = 4 * COUNTS_PER_REV + 123
        low, high = CFG.target_count(-70, ref), CFG.target_count(70, ref)
        self.assertGreater(high-low, COUNTS_PER_REV)
        total, whole, angle = turns(high, low)
        self.assertEqual(whole, 1)
        self.assertAlmostEqual(total, 140 / CFG.mm_per_rev, delta=2 / COUNTS_PER_REV)
        self.assertAlmostEqual(angle, (140 / CFG.mm_per_rev - 1) * 360,
                               delta=720 / COUNTS_PER_REV)
        self.assertEqual(turns(ref - 2 * COUNTS_PER_REV - COUNTS_PER_REV // 4, ref), (-2.25, -2, -90))

    def test_out_of_range_and_nonfinite_targets_rejected(self):
        for mm in (-70.001, 70.001, math.nan, math.inf, True, "bad"):
            with self.subTest(mm=mm), self.assertRaises(Rejected):
                CFG.target_count(mm, 0)
        with self.assertRaises(Rejected):
            CFG.target_count(1, I32_MAX)

    def test_missing_or_invalid_calibration_rejected(self):
        for changes in ({"diameter_basis": None}, {"diameter_basis": "core", "cable_diameter_mm": None},
                        {"payout_sign": None}, {"payout_sign": True},
                        {"spool_diameter_mm": 0}, {"spool_diameter_mm": math.nan},
                        {"calibrated_mm_per_rev": -2}, {"speed_mm_s": 20.01},
                        {"device_id": 0}, {"rest_length_mm": 70}):
            with self.subTest(changes=changes), self.assertRaises(Rejected):
                replace(CFG, **changes).validate()

class ProtocolTests(unittest.TestCase):
    def setUp(self):
        # No PyModbus import and no serial connection: test register handling.
        self.d = SerialDrive.__new__(SerialDrive)
        self.d.settings, self.d.journal = CFG, NullJournal()

    def test_signed_target_order_and_commit(self):
        self.d.write = Mock()
        self.d.target(-1336)
        self.assertEqual(self.d.write.call_args_list[0].args, (16, (-1336) & 0xFFFF))
        self.assertEqual(self.d.write.call_args_list[1].args, (18, 65535))

    def test_low_word_error_or_stop_prevents_high_word_commit(self):
        self.d.write = Mock(side_effect=DriveFault("missing ack"))
        with self.assertRaises(DriveFault):
            self.d.target(100)
        self.assertEqual(self.d.write.call_count, 1)
        stop = Mock(side_effect=[False, True])
        self.d.write = Mock()
        self.assertFalse(self.d.target(100, stop))
        self.assertEqual(self.d.write.call_count, 1)

    def test_high_low_high_retry_and_signed_feedback(self):
        self.d.read = Mock(side_effect=[0, 65535, 1, 1, 4, 1])
        self.assertEqual(self.d.position(), 65540)
        self.d.read = Mock(side_effect=[65535, 65535, 65535])
        self.assertEqual(self.d.position(), -1)
        self.d.read = Mock(side_effect=[0, 65535, 1, 1, 65535, 2])
        with self.assertRaises(DriveFault):
            self.d.position()

    def test_ack_checked_and_failed_logging_does_not_block_disable(self):
        self.d.client = Mock(connected=True)
        response = Mock(address=2, registers=[0x0700])
        response.isError.return_value = False
        self.d.client.write_register.return_value = response
        self.d.journal = Mock()
        self.d.journal.event.side_effect = OSError("disk full")
        self.d.disable()
        self.d.client.write_register.assert_called_once_with(address=2, value=0x0700, device_id=11)
        self.d.journal = NullJournal()
        response.registers = [0]
        with self.assertRaises(DriveFault):
            self.d.disable()

    def test_serial_loss_prevents_implicit_reconnect(self):
        self.d.client = Mock(connected=False)
        with self.assertRaises(DriveFault):
            self.d.read(20)
        with self.assertRaises(DriveFault):
            self.d.enable()
        self.d.client.read_holding_registers.assert_not_called()
        self.d.client.write_register.assert_not_called()
