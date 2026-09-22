"""Two-cable endpoints, setup movement, transport ownership and failure handling."""
from dataclasses import replace
import csv
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch
from spirob_cable.config import RigSettings, modes_from_lengths, lengths_from_modes, difference_bounds
from spirob_cable.model import Settings, Rejected, COUNTS_PER_REV, I32_MAX
from spirob_cable.drive import DemoDrive, SerialDrive, DriveFault
from spirob_cable.records import Recorder, NullRecorder, AxisLog
from spirob_cable.rig import Rig, Move, Cancelled
from spirob_cable.worker import Worker, make_drives
from spirob_cable.procedure import Procedure
from spirob_cable.review import load_runs


class RigTests(unittest.TestCase):
    def setUp(self):
        self.now=100.; self.timer=patch('time.monotonic',lambda:self.now); self.timer.start(); self.addCleanup(self.timer.stop)
        self.cfg=RigSettings.demo()
        self.log=Mock(wraps=NullRecorder())
        self.drives=[DemoDrive(c,AxisLog(self.log,i)) for i,c in enumerate(self.cfg.selected)]
        self.r=Rig(self.cfg,self.drives,self.log)
        self.r.connect(); self.poll(3)

    def poll(self,n=1,dt=.1):
        for _ in range(n): self.now+=dt; self.r.poll_all()

    def enable(self):
        self.r.enable([0,1]); self.poll(3)

    def ready(self):
        self.enable(); self.r.set_rest(0,220); self.r.set_rest(1,220); self.poll()

    def settle(self):
        for _ in range(600):
            self.poll()
            if all(a.state in ('HOLDING','DISABLED') for a in self.r.axes): return
        self.fail('Failed to settle')

    def test_connection_records_plot_coordinates_without_enable_or_rest(self):
        self.assertTrue(all(not a.enabled and a.rest is None for a in self.r.axes))
        self.assertEqual([a.snapshot()['connection_mm'] for a in self.r.axes],[0,0])
        self.assertIsNone(self.r.coordinates())
        self.assertGreater(self.log.sample.call_count,0)
        self.assertFalse(any(d.enabled for d in self.drives))

    def test_jog_more_than_half_mm_and_two_mm_without_rest(self):
        self.enable(); self.r.jog(0,10,10); self.settle()
        self.assertAlmostEqual(self.r.axes[0].snapshot()['connection_mm'],10,delta=.001)
        self.r.jog(0,5,10); self.settle()
        self.assertAlmostEqual(self.r.axes[0].snapshot()['connection_mm'],15,delta=.002)
        self.assertIsNone(self.r.axes[0].rest)
        self.assertAlmostEqual(self.r.axes[1].snapshot()['connection_mm'],0)
        self.assertEqual(self.drives[0].motor_rpm,self.cfg.axes[0].rpm_for_speed(10))

    def test_setup_envelope_is_from_connection_and_survives_reenable(self):
        self.enable(); self.r.jog(0,60,20); self.settle()
        self.r.release([0]); self.poll(3); self.r.enable([0]); self.poll(3)
        with self.assertRaises(Rejected): self.r.jog(0,20,20)
        self.assertAlmostEqual(self.r.axes[0].snapshot()['connection_mm'],60,delta=.001)

    def test_setup_envelope_is_configurable(self):
        cfg=RigSettings(tuple(replace(a,setup_travel_mm=150) for a in self.cfg.axes),2)
        ds=[DemoDrive(c,NullRecorder()) for c in cfg.selected]; r=Rig(cfg,ds,NullRecorder()); r.connect()
        for _ in range(3): self.now+=.1; r.poll_all()
        r.enable([0,1])
        for _ in range(3): self.now+=.1; r.poll_all()
        r.jog(0,100,20)
        for _ in range(80): self.now+=.1; r.poll_all()
        self.assertAlmostEqual(r.axes[0].snapshot()['connection_mm'],100,delta=.001)

    def test_enable_preloads_current_count_before_each_enable(self):
        events=[]
        for i,d in enumerate(self.drives):
            target,enable=d.target,d.enable
            d.target=lambda c,cancel,i=i,fn=target: events.append(('target',i,c)) or fn(c,cancel)
            d.enable=lambda i=i,fn=enable: events.append(('enable',i)) or fn()
        self.enable()
        self.assertEqual([e[:2] for e in events],[('target',0),('enable',0),('target',1),('enable',1)])
        self.assertTrue(all(a.rest is None for a in self.r.axes))

    def test_enable_at_rest_captures_only_after_stationary_feedback(self):
        self.r.enable([0,1],{0:220,1:220})
        self.assertTrue(self.r.pending_reference)
        self.assertTrue(all(a.rest is None for a in self.r.axes))
        self.poll(4)
        self.assertFalse(self.r.pending_reference)
        self.assertEqual(self.r.coordinates()['mean_mm'],220)

    def test_stop_cancels_pending_rest_capture(self):
        self.r.enable([0,1],{0:220,1:220}); self.r.stop_all(); self.poll(5)
        self.assertTrue(all(a.rest is None for a in self.r.axes))

    def test_independent_jogs_can_overlap_with_separate_speeds_and_signs(self):
        self.enable(); self.r.jog(0,20,10); self.r.jog(1,-5,5)
        self.assertEqual([a.state for a in self.r.axes],['MOVING','MOVING'])
        self.settle()
        self.assertAlmostEqual(self.r.axes[0].snapshot()['connection_mm'],20,delta=.001)
        self.assertAlmostEqual(self.r.axes[1].snapshot()['connection_mm'],-5,delta=.001)
        self.assertGreater(self.drives[1].count,self.r.axes[1].connection)  # Reversed payout sign.

    def test_mean_and_difference_examples_and_independent_adjustments(self):
        self.ready(); self.r.move_modes('both',220,10,10); self.settle()
        self.assertAlmostEqual(self.r.axes[0].absolute(),230,delta=.001)
        self.assertAlmostEqual(self.r.axes[1].absolute(),210,delta=.001)
        self.r.move_modes('mean',180,999,10); self.settle()
        self.assertAlmostEqual(self.r.axes[0].absolute(),190,delta=.002)
        self.assertAlmostEqual(self.r.axes[1].absolute(),170,delta=.002)
        self.r.move_modes('difference',999,-5,10); self.settle()
        self.assertAlmostEqual(self.r.axes[0].absolute(),175,delta=.003)
        self.assertAlmostEqual(self.r.axes[1].absolute(),185,delta=.003)

    def test_invalid_second_target_rejects_both_before_writes(self):
        self.ready()
        for d in self.drives:
            d.target=Mock(wraps=d.target); d.set_speed=Mock(wraps=d.set_speed)
        with self.assertRaises(Rejected): self.r.move_modes('both',180,40,10)  # L2 140 <150.
        for d in self.drives: d.target.assert_not_called(); d.set_speed.assert_not_called()

    def test_new_rest_moves_window_and_full_turns_are_retained(self):
        self.enable(); self.r.jog(0,20,20); self.settle(); self.r.set_rest(0,220)
        self.r.move_axis(0,-70,20); self.settle(); low=self.r.axes[0].count
        self.r.move_axis(0,70,20); self.settle(); high=self.r.axes[0].count
        self.assertGreater(abs(high-low),COUNTS_PER_REV)
        self.assertAlmostEqual(self.r.axes[0].snapshot()['connection_mm'],90,delta=.002)
        with self.assertRaises(Rejected): self.r.jog(0,1,20)

    def test_speed_readback_failure_prevents_both_target_commits(self):
        self.ready()
        self.drives[0].target=Mock(); self.drives[1].target=Mock()
        self.drives[1].set_speed=Mock(side_effect=DriveFault('readback'))
        with self.assertRaises(DriveFault): self.r.move_modes('both',230,0,10)
        self.drives[0].target.assert_not_called(); self.drives[1].target.assert_not_called()

    def test_partial_target_failure_attempts_both_disables_and_latches_fault(self):
        self.ready(); self.drives[1].target=Mock(side_effect=DriveFault('second target lost'))
        for d in self.drives: d.disable=Mock(wraps=d.disable)
        try: self.r.move_modes('both',230,0,10)
        except DriveFault as exc: self.r.fault(exc)
        self.assertTrue(self.r.faulted)
        for d in self.drives: d.disable.assert_called_once()
        self.assertTrue(all(a.rest is None and a.state=='FAULT' for a in self.r.axes))

    def test_disable_failure_on_first_drive_does_not_skip_second(self):
        self.ready(); self.drives[0].disable=Mock(side_effect=DriveFault('port failed'))
        self.drives[1].disable=Mock(wraps=self.drives[1].disable)
        self.r.fault('feedback failed')
        self.drives[1].disable.assert_called_once()
        self.assertEqual([a.disable_confirmed for a in self.r.axes],[False,True])

    def test_stop_between_target_commits_does_not_dispatch_second(self):
        self.ready(); stop=[False]; self.r.cancelled=lambda:stop[0]
        target=self.drives[0].target
        def first(count,cancel):
            result=target(count,cancel); stop[0]=True; return result
        self.drives[0].target=first; self.drives[1].target=Mock()
        with self.assertRaises(Cancelled): self.r.move_modes('both',230,0,10)
        self.drives[1].target.assert_not_called()
        stop[0]=False; self.r.stop_all(); self.settle()
        self.assertTrue(all(a.state=='HOLDING' for a in self.r.axes))

    def test_stop_during_speed_change_invalidates_cache(self):
        self.ready(); stop=[False]; self.r.cancelled=lambda:stop[0]
        setter=self.drives[0].set_speed
        def interrupted(rpm,cancel): setter(rpm,cancel); stop[0]=True; return False
        self.drives[0].set_speed=interrupted
        with self.assertRaises(Cancelled): self.r.jog(0,5,10)
        self.assertFalse(self.r.axes[0].speed_verified)
        stop[0]=False; self.r.stop_all(); self.settle()
        self.drives[0].set_speed=Mock(wraps=setter); self.r.jog(0,1,5)
        self.drives[0].set_speed.assert_called_once()

    def test_unknown_physical_length_blocks_mean_control_but_allows_jog(self):
        self.enable(); self.r.set_rest(0,None); self.r.set_rest(1,None)
        with self.assertRaises(Rejected): self.r.move_modes('both',220,0,5)
        self.r.jog(0,1,5); self.settle()
        self.assertAlmostEqual(self.r.axes[0].snapshot()['offset_mm'],1,delta=.001)

    def test_single_motor_mode_can_jog_and_move_without_second_device(self):
        cfg=replace(self.cfg,active_axes=1); r=Rig(cfg,[self.drives[0]],self.log); r.connect()
        for _ in range(3): self.now+=.1; r.poll_all()
        r.enable([0])
        for _ in range(3): self.now+=.1; r.poll_all()
        r.jog(0,5,10)
        for _ in range(20): self.now+=.1; r.poll_all()
        self.assertAlmostEqual(r.axes[0].snapshot()['connection_mm'],5,delta=.001)
        with self.assertRaises(Rejected): r.move_modes('both',220,0,5)

    def test_feedback_loss_and_stale_feedback_fail(self):
        self.ready(); self.drives[1].failed=True
        with self.assertRaises(DriveFault): self.r.poll_all()
        self.r.fault('lost cable2'); self.assertIsNone(self.r.coordinates())
        self.assertTrue(all(not a.enabled for a in self.r.axes))

    def test_full_experiment_then_abort(self):
        self.ready(); p=Procedure(self.r)
        data={'kind':'pair','rows':[[220,2,.1],[210,2,.1],[220,0,.1]],'repeats':1,'speed_mm_s':10}
        p.start(data)
        for _ in range(150):
            p.tick(); self.poll()
            if not p.active: break
        self.assertEqual(p.phase,'COMPLETE'); self.assertAlmostEqual(self.r.coordinates()['mean_mm'],220,delta=.001)
        p.start(data); p.tick(); p.abort('stop'); self.r.stop_all(); self.settle(); p.tick()
        self.assertEqual(p.phase,'ABORTED')

    def test_late_invalid_experiment_row_prevents_first_movement(self):
        self.ready(); p=Procedure(self.r)
        for d in self.drives: d.target=Mock()
        with self.assertRaises(Rejected): p.start({'kind':'pair','rows':[[220,1,1],[180,40,1]],'repeats':1,'speed_mm_s':10})
        for d in self.drives: d.target.assert_not_called()
        self.assertFalse(p.active)


class ConfigAndRecordingTests(unittest.TestCase):
    def test_coordinate_inversion_and_unequal_rest_limits(self):
        self.assertEqual(lengths_from_modes(220,10),(230,210))
        self.assertEqual(modes_from_lengths(230,210),(220,10))
        self.assertEqual(difference_bounds(180,220,220),(-30,30))
        self.assertEqual(difference_bounds(220,230,210),(-60,80))

    def test_duplicate_unset_ids_and_bad_parameters_rejected(self):
        cfg=RigSettings.demo()
        with self.assertRaises(Rejected): replace(cfg,axes=(cfg.axes[0],cfg.axes[0])).validate()
        with self.assertRaises(Rejected): RigSettings().validate()
        for bad in (0,float('nan'),1001):
            with self.assertRaises(Rejected): replace(cfg.axes[0],setup_travel_mm=bad).validate()

    def test_old_config_import_never_guesses_second_id(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'config.json'; Settings(device_id=47,diameter_basis='core',payout_sign=-1,speed_mm_s=10).save(p)
            cfg=RigSettings.load(p)
            self.assertEqual(cfg.active_axes,1); self.assertEqual(cfg.axes[0].device_id,47); self.assertIsNone(cfg.axes[1].device_id)
            cfg.save(p); self.assertEqual(RigSettings.load(p),cfg)

    def test_shared_port_uses_one_modbus_client_distinct_ids(self):
        cfg=RigSettings.demo()
        with patch('pymodbus.client.ModbusSerialClient') as cls:
            drives=make_drives(cfg,False,NullRecorder())
            self.assertEqual(cls.call_count,1); self.assertIs(drives[0].client,drives[1].client)
            self.assertTrue(drives[0].owns_client); self.assertFalse(drives[1].owns_client)
            drives[1].close(); cls.return_value.close.assert_not_called()
            drives[0].close(); cls.return_value.close.assert_called_once()

    def test_separate_ports_use_separate_clients(self):
        cfg=RigSettings.demo(); cfg=replace(cfg,axes=(cfg.axes[0],replace(cfg.axes[1],port='SECOND')))
        with patch('pymodbus.client.ModbusSerialClient') as cls:
            make_drives(cfg,False,NullRecorder()); self.assertEqual(cls.call_count,2)

    def test_recording_before_rest_and_pair_skew_review(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg=RigSettings.demo(); events=[]; rec=Recorder(folder,cfg,True,lambda kind,**data:events.append((kind,data)))
            ds=make_drives(cfg,True,rec); r=Rig(cfg,ds,rec); r.connect()
            for _ in range(3): r.poll_all()
            r.enable([0,1])
            for _ in range(3): r.poll_all()
            r.set_rest(0,220); r.set_rest(1,220); r.poll_all(); rec.close()
            with (rec.path/'samples.csv').open() as stream: rows=list(csv.DictReader(stream))
            self.assertEqual(rows[0]['connection_mm'],'0.0'); self.assertEqual(rows[0]['absolute_mm'],'')
            self.assertTrue(any(row['absolute_mm']=='220.0' for row in rows))
            with (rec.path/'pairs.csv').open() as stream: pairs=list(csv.DictReader(stream))
            self.assertTrue(pairs); self.assertGreaterEqual(float(pairs[-1]['skew_s']),0)
            runs=load_runs(rec.path)
            self.assertEqual(len(runs),2); self.assertTrue(all(run.metrics()['samples']>0 for run in runs.values()))
            self.assertTrue(all(run.actual[0]==0 for run in runs.values()))

    def test_worker_stop_priority_clears_queue(self):
        import queue
        worker=Worker(RigSettings.demo(),True,'unused',queue.Queue()); worker.submit('jog',{'axis':0})
        worker.submit('stop'); self.assertTrue(worker.commands.empty())
        with self.assertRaises(Rejected): worker.submit('jog',{'axis':1})

    def test_serial_speed_readback_mismatch(self):
        drive=SerialDrive.__new__(SerialDrive); drive.write=Mock(); drive.read=Mock(return_value=1)
        with self.assertRaises(DriveFault): drive.set_speed(348)
        drive.write.assert_called_once_with(14,348)


if __name__=='__main__': unittest.main()

class HeartbeatTests(unittest.TestCase):
    def test_worker_gui_heartbeat_loss_disables_both(self):
        import queue
        with tempfile.TemporaryDirectory() as folder:
            w=Worker(RigSettings.demo(),True,folder,queue.Queue()); w.start()
            def until(fn,heartbeat=True):
                end=time.monotonic()+5
                while time.monotonic()<end:
                    if heartbeat: w.heartbeat=time.monotonic()
                    if fn(): return
                    time.sleep(.02)
                self.fail('Worker condition timed out')
            try:
                until(lambda:w.rig is not None and all(len(a.stable)==3 for a in w.rig.axes))
                w.submit('enable',{'indices':[0,1]})
                until(lambda:all(a.enabled for a in w.rig.axes))
                until(lambda:w.rig.faulted,heartbeat=False)
                self.assertTrue(all(a.disable_confirmed for a in w.rig.axes))
                self.assertTrue(all(not a.enabled and a.rest is None for a in w.rig.axes))
            finally:
                w.submit('close'); w.join(3)
            self.assertFalse(w.is_alive())
