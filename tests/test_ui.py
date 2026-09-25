"""Real Qt event loops and a two-drive simulator. No hardware is opened."""
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
import numpy as np
from PySide6 import QtWidgets as Q, QtTest
from spirob_cable.ui import Studio
from spirob_cable.config import RigSettings
from spirob_cable.model import Settings

APP=Q.QApplication.instance() or Q.QApplication([]); APP.setStyle('Fusion')


class UITests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.root=Path(self.tmp.name)
        self.errors=[]; self.hook=sys.excepthook; sys.excepthook=lambda *args:self.errors.append(args)
        self.w=Studio(True,self.root/'config.json',self.root/'logs')
        self.w.confirm=lambda *args:True; self.w.show(); APP.processEvents()

    def tearDown(self):
        if self.w.worker:
            self.w.worker.submit('close'); self.until(lambda:self.w.worker is None)
        self.w.close(); APP.processEvents(); sys.excepthook=self.hook
        self.tmp.cleanup(); self.assertFalse(self.errors,str(self.errors))

    def until(self,fn,seconds=10):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            APP.processEvents()
            if fn(): return
            QtTest.QTest.qWait(20)
        self.fail(f'Timeout: {self.w.snapshot} errors={self.errors}')

    def connect(self):
        self.w.connect_button.click()
        self.until(lambda:len(self.w.snapshot.get('axes',[]))==2 and all(len(a.stable)==3 for a in self.w.worker.rig.axes))

    def ready(self):
        self.connect(); self.w.enable_rest_button.click()
        self.until(lambda:self.w.snapshot.get('coordinates') is not None and not self.w.snapshot.get('pending_reference') and not self.w.pending)

    def shot(self,name):
        if os.environ.get('STUDIO_SCREENSHOT_DIR'):
            path=Path(os.environ['STUDIO_SCREENSHOT_DIR']); path.mkdir(parents=True,exist_ok=True)
            APP.processEvents(); self.w.grab().save(str(path/name))

    def test_plot_visible_before_enable_and_flexible_jog_before_rest(self):
        self.connect(); QtTest.QTest.qWait(150); self.w.redraw()
        for curve in self.w.plots.curves:
            x,y=curve.getData(); self.assertGreaterEqual(len(y),3); self.assertTrue(np.all(np.isfinite(y)))
        self.assertFalse(self.w.manual[0]['plus'].isEnabled())
        self.shot('connected.png')
        self.w.manual[0]['enable'].click()
        self.until(lambda:self.w.snapshot['axes'][0]['enabled'] and len(self.w.worker.rig.axes[0].stable)==3 and not self.w.pending)
        self.w.manual[0]['step'].setValue(5); self.w.manual[0]['speed'].setValue(10)
        self.w.manual[0]['plus'].click()
        self.until(lambda:not self.w.pending and self.w.snapshot['axes'][0]['state']=='HOLDING' and self.w.snapshot['axes'][0]['connection_mm']>4.9)
        self.assertIsNone(self.w.snapshot['axes'][0]['rest_count'])
        self.assertFalse(self.w.snapshot['axes'][1]['enabled'])
        self.assertEqual(self.w.manual[0]['step'].value(),5)
        self.assertTrue(self.w.manual[0]['plus'].isEnabled())
        self.shot('setup_jog.png')

    def test_enable_at_rest_modes_limits_layout_and_review(self):
        self.ready(); self.w.controls.setCurrentIndex(1); self.w.pair.speed.setValue(20)
        self.w.pair.mean.setValue(.220); self.w.pair.difference.setValue(20); self.w.pair.apply.click()
        self.until(lambda:not self.w.pending and all(a['state']=='HOLDING' for a in self.w.snapshot['axes']) and self.w.snapshot['coordinates']['half_difference_mm']>9.9)
        self.w.pair.mean.setValue(.180); self.w.pair.apply.click()
        self.until(lambda:not self.w.pending and all(a['state']=='HOLDING' for a in self.w.snapshot['axes']) and self.w.snapshot['coordinates']['mean_mm']<180.1)
        lengths=[a['absolute_mm'] for a in self.w.snapshot['axes']]
        self.assertAlmostEqual(lengths[0],190,delta=.002); self.assertAlmostEqual(lengths[1],170,delta=.002)
        self.w.frame.setCurrentIndex(2); self.w.plots.auto_range(); self.shot('mean_difference.png')
        self.w.pair.difference.setValue(80)
        self.assertFalse(self.w.pair.apply.isEnabled())
        self.assertIn('Cable 2',self.w.pair.status.text().replace('cable 2', 'Cable 2'))
        self.assertFalse(self.w.snapshot.get('faulted'))
        self.w.resize(1080,760); QtTest.QTest.qWait(80); self.shot('small_layout.png')
        self.assertGreaterEqual(self.w.plots.graph.height(),430)
        self.assertLessEqual(self.w.height(),780)
        self.w.release_all_button.click(); self.until(lambda:all(not a['enabled'] for a in self.w.snapshot['axes']))
        self.w.connect_button.click(); self.until(lambda:self.w.worker is None)
        self.w.load_review(self.w.session_path)
        self.assertEqual(len(self.w.review_runs[0]),2)
        self.assertEqual(self.w.views.currentIndex(),1)
        self.shot('review.png')

    def test_loss_of_one_axis_faults_both_and_clears_references(self):
        self.ready(); self.w.demo_fault_buttons[1].click()
        self.until(lambda:self.w.snapshot.get('faulted'))
        self.assertTrue(all(a['rest_count'] is None and not a['enabled'] for a in self.w.snapshot['axes']))
        self.assertFalse(self.w.manual[0]['plus'].isEnabled()); self.assertFalse(self.w.pair.apply.isEnabled())
        self.assertIn('UNCONFIRMED',self.w.notice_label.text())

    def test_slider_drag_previews_once_and_release_starts_both_with_full_difference(self):
        self.ready(); p=self.w.pair; p.speed.setValue(20)
        self.w.controls.setCurrentIndex(1)
        with patch.object(self.w.worker,'submit',wraps=self.w.worker.submit) as submit:
            p.difference_slider.setSliderDown(True)
            for value in (50,100,200): p.difference_slider.setValue(value)
            self.assertEqual(p.difference.value(),20)
            self.assertAlmostEqual(p.lengths[0].value(),.230)
            self.assertAlmostEqual(p.lengths[1].value(),.210)
            submit.assert_not_called()
            p.difference_slider.setSliderDown(False)
            submit.assert_called_once()
            self.assertEqual(submit.call_args.args[1]['difference_mm'],10)
        self.until(lambda:not self.w.pending and all(a['state']=='HOLDING' for a in self.w.snapshot['axes']) and self.w.snapshot['coordinates']['half_difference_mm']>9.9)
        self.assertAlmostEqual(float(p.live[0].text()),.230,delta=.00002)
        self.assertIn('Live difference',p.live_summary.text())
        self.assertIn('ms between',p.timing.text())

    def test_unequal_lengths_update_average_and_repeated_feedback_preserves_draft(self):
        self.ready(); p=self.w.pair
        p.lengths[0].setValue(.240); p.lengths[1].setValue(.200)
        self.assertAlmostEqual(p.mean.value(),.220)
        self.assertAlmostEqual(p.difference.value(),40)
        self.w.refresh()
        self.assertAlmostEqual(p.difference.value(),40)
        p.mean.setValue(.180)
        self.assertAlmostEqual(p.lengths[0].value(),.200)
        self.assertAlmostEqual(p.lengths[1].value(),.160)
        p.equal.click()
        self.assertEqual(p.difference.value(),0)
        self.assertAlmostEqual(p.lengths[0].value(),.180)
        self.assertAlmostEqual(p.lengths[1].value(),.180)
        self.assertFalse(self.w.pending)

    def test_unequal_rest_lengths_initialize_targets_and_slider_limits(self):
        self.w.manual[0]['rest_length'].setValue(230)
        self.w.manual[1]['rest_length'].setValue(210)
        self.ready(); p=self.w.pair
        self.assertAlmostEqual(p.mean.value(),.220)
        self.assertAlmostEqual(p.difference.value(),20)
        # At mean 220: L1 >=160 and L2 >=140, so difference spans -120..160.
        self.assertEqual((p.difference_slider.minimum(),p.difference_slider.maximum()),(-1200,1600))
        p.auto.setChecked(False)
        with patch.object(self.w.worker,'submit') as submit:
            p.mean_slider.setSliderDown(True); p.mean_slider.setValue(1800)
            p.mean_slider.setSliderDown(False)
            submit.assert_not_called()
        self.assertAlmostEqual(p.lengths[0].value(),.190)
        self.assertAlmostEqual(p.lengths[1].value(),.170)

    def test_duplicate_ids_block_connect_and_old_config_imports_single(self):
        self.w.setup[1]['device_id'].setValue(1)
        with patch('spirob_cable.ui.Worker') as worker:
            self.w.connect_button.click(); worker.assert_not_called()
        self.assertIn('distinct',self.w.notice_label.text())
        path=self.root/'old.json'; Settings(device_id=47,diameter_basis='core',payout_sign=-1,speed_mm_s=10).save(path)
        self.w.demo=False
        with patch.object(Q.QFileDialog,'getOpenFileName',return_value=(str(path),'')):
            self.w.import_config()
        self.assertEqual(self.w.axis_count.currentIndex(),0)
        self.assertEqual(self.w.setup[0]['device_id'].value(),47)
        self.assertEqual(self.w.setup[1]['device_id'].value(),0)
        self.assertEqual(RigSettings.load(self.root/'config.json').axes[0].device_id,47)

    def test_pair_procedure_completion_and_priority_abort(self):
        self.ready(); self.w.trial_speed.setValue(20)
        for i,d in enumerate((0,1,0)):
            self.w.trial_table.cellWidget(i,0).setValue(.220)
            self.w.trial_table.cellWidget(i,1).setValue(d)
            self.w.trial_table.cellWidget(i,2).setValue(.1)
        self.w.start_trial_button.click()
        self.until(lambda:self.w.snapshot.get('procedure',{}).get('phase')=='COMPLETE')
        self.w.trial_table.cellWidget(0,1).setValue(20)
        self.w.start_trial_button.click(); self.until(lambda:self.w.snapshot.get('procedure',{}).get('active'))
        self.w.stop_button.click(); self.until(lambda:self.w.snapshot.get('procedure',{}).get('phase')=='ABORTED' and all(a['state']=='HOLDING' for a in self.w.snapshot['axes']))
        self.assertTrue(all(a['enabled'] for a in self.w.snapshot['axes']))

    def test_legacy_trial_half_difference_round_trips_through_full_difference_table(self):
        path=self.root/'legacy_trial.json'
        data={'kind':'pair','rows':[[220,10,1],[180,-5,2]],'repeats':1,'speed_mm_s':5}
        path.write_text(json.dumps(data))
        with patch.object(Q.QFileDialog,'getOpenFileName',return_value=(str(path),'')):
            self.w.load_trial()
        self.assertEqual(self.w.trial_table.cellWidget(0,1).value(),20)
        self.assertEqual(self.w.trial_table.cellWidget(1,1).value(),-10)
        self.assertEqual(self.w.trial_data(),data)

    def test_startup_fault_survives_worker_close_and_notice_expiry(self):
        from spirob_cable.drive import DemoDrive, DriveFault
        with patch.object(DemoDrive,'position',side_effect=DriveFault('ID 5: request deadline; RX bytes=12')):
            self.w.connect_button.click()
            self.until(lambda:self.w.worker is None)
        self.w.notice_until=0; self.w.refresh()
        self.assertIn('ID 5',self.w.readout.text())
        self.assertIn('ID 5',self.w.notice_label.text())
        self.assertTrue(self.w.dock.isVisible())
        self.assertFalse(self.w.manual[0]['enable'].isEnabled())
        self.assertFalse(self.w.manual[1]['enable'].isEnabled())
        self.connect()
        self.assertTrue(self.w.connection_ready)
        self.assertEqual(self.w.session_error,'')
        self.assertTrue(self.w.manual[0]['enable'].isEnabled())


if __name__=='__main__': unittest.main()
