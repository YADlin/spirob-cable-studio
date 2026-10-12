"""One explicit setup dialog: camera alignment and input wiring to MuJoCo."""
import copy
import numpy as np
from PySide6.QtWidgets import (QDialog, QVBoxLayout, QGridLayout, QHBoxLayout,
    QLabel, QDoubleSpinBox, QComboBox, QPushButton, QMessageBox)


class DisplaySetup(QDialog):
    def __init__(self, setup, base_m, apply_callback, pick_callback, reset_setup, parent=None, scale_callback=None):
        super().__init__(parent)
        self.current = copy.deepcopy(setup)
        self.base_m = np.asarray(base_m)
        self.apply_callback, self.pick_callback = apply_callback, pick_callback
        self.reset_setup = copy.deepcopy(reset_setup)
        self.scale_callback = scale_callback
        self.setWindowTitle('Camera alignment and MuJoCo cable mapping')
        root = QVBoxLayout(self)
        note = QLabel('Align the fixed joint-1 base once using the resting robot.\n'
                      'Keep this calibration fixed when checking another deformation.\n'
                      'Mapping changes the simulation inputs; physical motor commands retain their cable numbers.')
        note.setWordWrap(True); root.addWidget(note)
        grid = QGridLayout(); root.addLayout(grid)
        self.x = self.number(0, -4000, 8000, 2)
        self.y = self.number(0, -4000, 8000, 2)
        self.rotation = self.number(0, -180, 180, 3)
        self.scale = self.number(4000, 100, 20000, 3)
        for i,(label,box) in enumerate([('Physical base x [native px]',self.x),('Physical base y [native px]',self.y),
                                       ('Image rotation [degrees]',self.rotation),('Image scale [px/m]',self.scale)]):
            grid.addWidget(QLabel(label),i,0);grid.addWidget(box,i,1)
        self.mapping = QComboBox()
        self.mapping.addItems(['cable_0 ← physical cable 1; cable_1 ← physical cable 2',
                               'cable_0 ← physical cable 2; cable_1 ← physical cable 1 (swap)'])
        grid.addWidget(QLabel('MuJoCo tendon ← physical cable'),4,0);grid.addWidget(self.mapping,4,1)
        self.signs = []
        for i in range(2):
            box=QComboBox();box.addItems(['+1: increasing physical length increases model target',
                                        '−1: reverse displacement sign (advanced)'])
            grid.addWidget(QLabel(f'cable_{i} displacement sign'),5+i,0);grid.addWidget(box,5+i,1);self.signs.append(box)
        helptext=QLabel('For opposite bending, check the swap setting first. A sign reversal changes length conventions, not motor direction.\n'
                       'The base is j_001; the fitted curve begins at j_002, one element farther along the robot.\n'
                       'Changing alignment or mapping invalidates an earlier sweep selection.')
        helptext.setWordWrap(True);root.addWidget(helptext)
        row=QHBoxLayout();root.addLayout(row)
        pick=QPushButton('Pick physical base in video');row.addWidget(pick);pick.clicked.connect(self.pick)
        apply=QPushButton('Apply setup');row.addWidget(apply);apply.clicked.connect(self.apply)
        save=QPushButton('Apply + save setup');row.addWidget(save);save.clicked.connect(lambda:self.apply(True))
        reset=QPushButton('Load supplied image alignment');root.addWidget(reset)
        reset.clicked.connect(self.load_supplied_alignment)
        match=QPushButton('Match model length to calibrated physical joints');root.addWidget(match)
        match.clicked.connect(self.match_length)
        scale_note=QLabel('For automatic length matching, place the physical joints in the inspector and Apply joints first. The button compares the sum of those exact joint-to-joint lengths with the same model links.\nYou can also edit Image scale [px/m]: increase it to lengthen the red backbone, decrease it to shorten it. A 10% increase makes the overlay 10% longer. The picked base stays fixed. Calibrate in a resting, nearly straight pose, then Apply + save setup.')
        scale_note.setWordWrap(True);root.addWidget(scale_note)
        clear=QPushButton('Clear transferred joint spacing');root.addWidget(clear);clear.clicked.connect(self.clear_profile)
        self.notice=QLabel('Settings remain in memory until Apply + save setup.');self.notice.setWordWrap(True);root.addWidget(self.notice)
        self.load(setup)

    @staticmethod
    def number(value,low,high,decimals):
        box=QDoubleSpinBox();box.setRange(low,high);box.setDecimals(decimals);box.setValue(value)
        box.setKeyboardTracking(False);return box

    def load(self, setup):
        self.current=copy.deepcopy(setup)
        angle=setup['rotation_rad'];c,s=np.cos(angle),np.sin(angle)
        base=setup['pixels_per_m']*(self.base_m @ np.array([[c,s],[-s,c]]))
        base += [setup['translation_x_px'],setup['translation_y_px']]
        self.x.setValue(base[0]);self.y.setValue(base[1])
        self.rotation.setValue(np.degrees(angle));self.scale.setValue(setup['pixels_per_m'])
        self.mapping.setCurrentIndex(int(setup.get('mapping',[0,1])==[1,0]))
        for i,value in enumerate(setup.get('signs',[1,1])): self.signs[i].setCurrentIndex(int(value<0))

    def values(self):
        angle=np.radians(self.rotation.value());c,s=np.cos(angle),np.sin(angle)
        translation=np.array([self.x.value(),self.y.value()])-self.scale.value()*(self.base_m @ np.array([[c,s],[-s,c]]))
        return dict(self.current, translation_x_px=float(translation[0]),translation_y_px=float(translation[1]),
                    rotation_rad=float(angle),pixels_per_m=self.scale.value(),
                    mapping=[1,0] if self.mapping.currentIndex() else [0,1],
                    signs=[-1 if b.currentIndex() else 1 for b in self.signs])

    def pick(self):
        self.hide();self.pick_callback(self)

    def load_supplied_alignment(self):
        draft=self.values()
        for key in ('translation_x_px','translation_y_px','pixels_per_m','rotation_rad'):
            draft[key]=self.reset_setup[key]
        self.load(draft)
        self.notice.setText('Supplied image alignment loaded; cable mapping preserved. Apply to view, then pick the physical base and save.')

    def clear_profile(self):
        self.current['profile']=None
        self.notice.setText('Transferred spacing cleared from this draft. Apply setup to use nominal spacing again.')

    def match_length(self):
        try:
            value=self.scale_callback(self.values())
            if not self.scale.minimum() <= value <= self.scale.maximum():raise ValueError('Matched scale is outside the supported range.')
            self.scale.setValue(value)
            self.notice.setText(f'Scale matched to physical joint lengths: {value:.3f} px/m. Apply + save setup to keep this calibration fixed.')
        except (ValueError,KeyError,TypeError) as exc:QMessageBox.warning(self,'Scale not changed',str(exc))

    def picked(self,x,y):
        self.x.setValue(x);self.y.setValue(y);self.show();self.raise_()
        self.notice.setText('Base position selected. Press Apply setup to use it.')

    def apply(self, save=False):
        try:
            self.apply_callback(self.values(),bool(save))
            self.current=self.values()
            self.notice.setText('Setup saved for later sessions.' if save else 'Setup applied to this session.')
        except (ValueError,OSError) as exc: QMessageBox.warning(self,'Setup not applied',str(exc))
