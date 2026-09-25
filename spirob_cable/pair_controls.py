"""One coherent target pair, expressed as lengths or mean/full difference.

This widget emits requests only; it never accesses a drive. Slider movement
previews targets. An optional release action submits one finite paired move.
"""
import math
from PySide6 import QtCore, QtWidgets as Q
from .model import TRAVEL_MM


def spin(value, low, high, decimals, suffix, step):
    widget = Q.QDoubleSpinBox()
    widget.setRange(low, high); widget.setDecimals(decimals); widget.setSuffix(suffix)
    widget.setSingleStep(step); widget.setKeyboardTracking(False); widget.setValue(value)
    return widget


def text(value=''):
    widget = Q.QLabel(value); widget.setWordWrap(True)
    return widget


class PairControls(Q.QWidget):
    move_requested = QtCore.Signal()

    def __init__(self):
        super().__init__()
        self.rests = None
        self.ready = False
        self.live_values = None
        self.reference_key = None
        self.target_valid = False
        layout = Q.QVBoxLayout(self)
        layout.addWidget(text('TWO-CABLE LENGTH CONTROL'))
        layout.addWidget(text('Set the average length, then make one cable longer and the other shorter. Difference 0 gives equal lengths.'))

        box = Q.QGroupBox('Mean and difference targets'); form = Q.QVBoxLayout(box)
        row = Q.QHBoxLayout(); row.addWidget(text('Mean length'))
        self.mean = spin(.220, -10, 20, 6, ' m', .001); row.addWidget(self.mean)
        form.addLayout(row)
        self.mean_slider = Q.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.mean_slider.setSingleStep(1); self.mean_slider.setPageStep(10)
        form.addWidget(self.mean_slider)
        self.mean_range = text(); form.addWidget(self.mean_range)
        row = Q.QHBoxLayout(); row.addWidget(text('Difference L₁ − L₂'))
        self.difference = spin(0, -40000, 40000, 3, ' mm', 1); row.addWidget(self.difference)
        form.addLayout(row)
        self.difference_slider = Q.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.difference_slider.setSingleStep(1); self.difference_slider.setPageStep(10)
        form.addWidget(self.difference_slider)
        self.difference_range = text(); form.addWidget(self.difference_range)
        form.addWidget(text('Positive difference: cable 1 longer, cable 2 shorter.'))
        layout.addWidget(box)

        box = Q.QGroupBox('Cable lengths'); grid = Q.QGridLayout(box)
        grid.addWidget(text('Cable'), 0, 0)
        grid.addWidget(text('Live · m'), 0, 1)
        grid.addWidget(text('Target · editable'), 0, 2)
        self.live = []; self.lengths = []
        for index in range(2):
            grid.addWidget(text(str(index+1)), index+1, 0)
            live = text('—'); self.live.append(live); grid.addWidget(live, index+1, 1)
            value = spin(.220, -10, 20, 6, ' m', .001)
            self.lengths.append(value); grid.addWidget(value, index+1, 2)
            value.valueChanged.connect(self.lengths_edited)
        self.live_summary = text('Live lengths require a physical rest reference.')
        self.live_summary.setToolTip('Lengths are estimates from encoder counts and spool calibration, not direct cable measurements.')
        grid.addWidget(self.live_summary, 3, 0, 1, 3)
        layout.addWidget(box)

        row = Q.QHBoxLayout()
        self.equal = Q.QPushButton('Equal lengths')
        self.copy = Q.QPushButton('Copy live lengths')
        row.addWidget(self.equal); row.addWidget(self.copy); layout.addLayout(row)
        row = Q.QHBoxLayout(); row.addWidget(text('Cable speed'))
        self.speed = spin(5, .05, 20, 2, ' mm/s', 1); row.addWidget(self.speed)
        layout.addLayout(row)
        self.auto = Q.QCheckBox('Move when a slider is released'); self.auto.setChecked(True)
        layout.addWidget(self.auto)
        self.apply = Q.QPushButton('Move both cables'); self.apply.setObjectName('primary')
        layout.addWidget(self.apply)
        self.status = text(); layout.addWidget(self.status)
        self.timing = text('Both targets start in quick succession; exact simultaneous start is not available on this link.')
        layout.addWidget(self.timing)
        layout.addWidget(text('Numeric edits and Equal lengths update the preview. Click Move both cables to execute. Live values are encoder estimates; slider steps are 0.1 mm.'))
        layout.addStretch()

        self.mean.valueChanged.connect(self.modes_edited)
        self.difference.valueChanged.connect(self.modes_edited)
        self.mean_slider.valueChanged.connect(lambda value: self.mean.setValue(value/10000))
        self.difference_slider.valueChanged.connect(lambda value: self.difference.setValue(value/10))
        for slider in (self.mean_slider, self.difference_slider):
            slider.sliderReleased.connect(self.slider_released)
        self.equal.clicked.connect(lambda: self.difference.setValue(0))
        self.copy.clicked.connect(self.copy_live)
        self.apply.clicked.connect(self.move_requested.emit)
        self.set_targets(220, 0)

    @staticmethod
    def set_value(widget, value):
        with QtCore.QSignalBlocker(widget):
            widget.setValue(value)

    def targets(self):
        return self.mean.value()*1000, self.difference.value()

    def set_targets(self, mean_mm, difference_mm):
        self.set_value(self.mean, mean_mm/1000)
        self.set_value(self.difference, difference_mm)
        self.modes_edited()

    def modes_edited(self, *_):
        mean, difference = self.targets()
        self.set_value(self.lengths[0], (mean+difference/2)/1000)
        self.set_value(self.lengths[1], (mean-difference/2)/1000)
        self.update_limits()

    def lengths_edited(self, *_):
        first, second = (widget.value()*1000 for widget in self.lengths)
        self.set_targets((first+second)/2, first-second)

    def update_limits(self):
        mean, difference = self.targets()
        # Offline preview uses a clearly labelled illustrative 220 mm rest.
        r1, r2 = self.rests if self.rests is not None else (220, 220)
        low1, high1 = r1-TRAVEL_MM, r1+TRAVEL_MM
        low2, high2 = r2-TRAVEL_MM, r2+TRAVEL_MM
        mean_low, mean_high = max(low1-difference/2, low2+difference/2), min(high1-difference/2, high2+difference/2)
        diff_low, diff_high = 2*max(low1-mean, mean-high2), 2*min(high1-mean, mean-low2)
        for slider, low, high, value in ((self.mean_slider, mean_low, mean_high, mean),
                                        (self.difference_slider, diff_low, diff_high, difference)):
            lo, hi = math.ceil(low*10-1e-8), math.floor(high*10+1e-8)
            with QtCore.QSignalBlocker(slider):
                slider.setRange(lo, max(lo, hi)); slider.setValue(round(value*10))
            slider.setEnabled(self.ready and hi >= lo)
        first, second = mean+difference/2, mean-difference/2
        self.target_valid = (self.rests is not None and low1-1e-8 <= first <= high1+1e-8
                             and low2-1e-8 <= second <= high2+1e-8)
        suffix = '' if self.rests is not None else ' · preview only'
        self.mean_range.setText(f'{mean_low/1000:.4f} … {mean_high/1000:.4f} m'+suffix if mean_low <= mean_high else 'No valid mean for this difference.')
        self.difference_range.setText(f'{diff_low:+.1f} … {diff_high:+.1f} mm'+suffix if diff_low <= diff_high else 'No valid difference for this mean.')
        self.apply.setEnabled(self.ready and self.target_valid)
        if self.rests is None:
            message = 'Enable both cables and set their measured physical rest lengths to control absolute lengths.'
        elif not self.target_valid:
            message = f'Outside travel limits. Cable 1: {low1/1000:.4f}–{high1/1000:.4f} m; cable 2: {low2/1000:.4f}–{high2/1000:.4f} m. No move will be sent.'
        elif not self.ready:
            message = 'Waiting for both cables to be enabled, stationary and referenced.'
        else:
            message = f'Ready: cable 1 → {first/1000:.5f} m; cable 2 → {second/1000:.5f} m.'
        self.status.setText(message)
        self.status.setStyleSheet('color:#9c3026;' if self.rests is not None and not self.target_valid else '')

    def slider_released(self):
        # No queue of old slider positions, and never issue commands in drag.
        if self.auto.isChecked() and self.ready and self.target_valid:
            self.move_requested.emit()

    def copy_live(self):
        if self.live_values is not None:
            first, second = self.live_values
            self.set_targets((first+second)/2, first-second)

    def update_feedback(self, axes, valid, ready, editable):
        self.ready = bool(ready)
        self.live_values = None
        self.rests = None
        if len(axes) == 2 and all(axis.get('rest_length_mm') is not None for axis in axes):
            self.rests = tuple(axis['rest_length_mm'] for axis in axes)
        if valid and len(axes) == 2 and all(axis.get('absolute_mm') is not None for axis in axes):
            self.live_values = tuple(axis['absolute_mm'] for axis in axes)
            key = tuple((axis['rest_count'], axis['rest_length_mm'], axis['reference_epoch']) for axis in axes)
            if key != self.reference_key:
                self.reference_key = key
                self.copy_live()
        elif not axes or self.rests is None:
            self.reference_key = None
        for index, live in enumerate(self.live):
            live.setText(f'{self.live_values[index]/1000:.5f}' if self.live_values is not None else '—')
        if self.live_values is not None:
            first, second = self.live_values
            self.live_summary.setText(f'Live mean {(first+second)/2000:.5f} m\nLive difference L₁ − L₂: {first-second:+.2f} mm')
        else:
            self.live_summary.setText('Live lengths unavailable: reference both cables and check feedback.')
        for widget in (self.mean, self.difference, *self.lengths, self.speed, self.equal):
            widget.setEnabled(bool(editable))
        self.copy.setEnabled(bool(editable) and self.live_values is not None)
        self.auto.setEnabled(bool(editable))
        self.update_limits()
