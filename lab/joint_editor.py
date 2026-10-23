"""Hand-edit per-joint stiffness and damping and see the MuJoCo backbone over the camera.

Joint 1 (the base clamp) is not edited. Apply sends the values to the running
simulation only; Save writes them into an XML (the Lab model by default, with a
timestamped backup) so they persist.
"""
import re
import shutil
from datetime import datetime
from pathlib import Path
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView, QDoubleSpinBox, QFileDialog, QMessageBox)


def write_joint_values(xml_path, names, stiffness, damping, destination=None):
    """Replace stiffness/damping attributes of the named joints, keeping the rest of the file as written."""
    text = Path(xml_path).read_text()
    for name, k, d in zip(names, stiffness, damping):
        pattern = re.compile(r'<joint\b[^>]*\bname="%s"[^>]*>' % re.escape(name))
        match = pattern.search(text)
        if not match:
            raise ValueError(f'Joint {name} not found in {xml_path}.')
        tag = match.group(0)
        for attr, value in (('stiffness', k), ('damping', d)):
            if re.search(r'\b%s="[^"]*"' % attr, tag):
                tag = re.sub(r'\b%s="[^"]*"' % attr, f'{attr}="{float(value)!r}"', tag)
            else:
                tag = tag.replace('<joint', f'<joint {attr}="{float(value)!r}"', 1)
        text = text[:match.start()] + tag + text[match.end():]
    out = Path(destination or xml_path)
    out.write_text(text)
    return out


def save_with_backup(model_path, names, stiffness, damping, target):
    """Write joint values to target. Overwriting the Lab model keeps a timestamped backup.

    Returns a one-line note for the user.
    """
    model_path = Path(model_path).resolve(); target = Path(target)
    if target.resolve() == model_path:
        backup = model_path.with_name(model_path.stem + datetime.now().strftime('_backup_%Y%m%d_%H%M%S.xml'))
        shutil.copy2(model_path, backup)
        note = f'Previous file kept as {backup.name}. The Lab reads it at the next start.'
    elif target.resolve().parent != model_path.parent:
        note = 'Keep it beside a meshes/ folder (or copy the meshes) so MuJoCo finds the meshes.'
    else:
        note = 'Saved beside the Lab model; meshes resolve.'
    write_joint_values(model_path, names, stiffness, damping, target)
    return note


class JointEditor(QDialog):
    def __init__(self, host, model_path, apply_callback):
        super().__init__(host)
        from spirob_analysis.model import SpiRobModel
        self.model_path = Path(model_path).resolve()
        self.apply_callback = apply_callback
        model = SpiRobModel(self.model_path)
        self.names = model.names[1:]
        self.xml_k, self.xml_d = model.joint_values()
        self.setWindowTitle('SpiRob — joint stiffness and damping')
        self.resize(560, 760)
        root = QVBoxLayout(self)
        note = QLabel('Edit any value, then Apply to simulation: the red MuJoCo backbone in the main window updates over the live camera. '
                      'Joint 1 (base clamp) is fixed. Save writes the values into the XML so they are used next time.')
        note.setWordWrap(True); root.addWidget(note)
        self.table = QTableWidget(len(self.names), 4)
        self.table.setHorizontalHeaderLabels(['Joint', 'k [N m/rad]', 'd [N m s/rad]', 'k / k₂'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().hide(); root.addWidget(self.table, 1)
        self.table.itemChanged.connect(self.update_ratio)
        grid = QGridLayout(); root.addLayout(grid)
        self.k2 = self.number(self.xml_k[0], .0001, 10, 6); self.r = self.number(.915, .5, 1.5, 4)
        self.d2 = self.number(self.xml_d[0], 0, 2, 6); self.rd = self.number(.915, .5, 1.5, 4)
        for col, (label, box) in enumerate([('k₂', self.k2), ('k ratio r', self.r), ('d₂', self.d2), ('d ratio', self.rd)]):
            grid.addWidget(QLabel(label), 0, col); grid.addWidget(box, 1, col)
        fill = QPushButton('Fill exponential'); fill.clicked.connect(self.fill_exponential); grid.addWidget(fill, 1, 4)
        self.scale = self.number(1., .01, 100, 3)
        grid.addWidget(QLabel('Scale all k by'), 2, 0); grid.addWidget(self.scale, 2, 1)
        scale_button = QPushButton('Scale k'); scale_button.clicked.connect(self.scale_k); grid.addWidget(scale_button, 2, 2)
        ratio_d = QPushButton('d = ζ-like: d_i ∝ √k_i'); ratio_d.setToolTip('Keep d₂, set d_i = d₂·√(k_i/k₂)')
        ratio_d.clicked.connect(self.damping_from_k); grid.addWidget(ratio_d, 2, 3, 1, 2)
        buttons = QHBoxLayout(); root.addLayout(buttons)
        for title, handler in [('Reset to XML', self.reset), ('Load joint_fit.csv…', self.load_csv),
                               ('Apply to simulation', self.apply), ('Save to XML…', self.save)]:
            b = QPushButton(title); b.clicked.connect(handler); buttons.addWidget(b)
        self.status = QLabel(); self.status.setWordWrap(True); root.addWidget(self.status)
        self.set_values(self.xml_k, self.xml_d)
        self.status.setText(f'Loaded from {self.model_path.name}.')

    @staticmethod
    def number(value, low, high, decimals):
        box = QDoubleSpinBox(); box.setRange(low, high); box.setDecimals(decimals); box.setValue(float(value)); return box

    def set_values(self, k, d):
        self.table.blockSignals(True)
        for i, (name, kv, dv) in enumerate(zip(self.names, k, d)):
            item = QTableWidgetItem(name); item.setFlags(Qt.ItemIsEnabled); self.table.setItem(i, 0, item)
            self.table.setItem(i, 1, QTableWidgetItem(f'{kv:.6g}'))
            self.table.setItem(i, 2, QTableWidgetItem(f'{dv:.6g}'))
            r = QTableWidgetItem(f'{kv / k[0]:.4f}'); r.setFlags(Qt.ItemIsEnabled); self.table.setItem(i, 3, r)
        self.table.blockSignals(False)

    def values(self):
        k, d = [], []
        for i, name in enumerate(self.names):
            try:
                k.append(float(self.table.item(i, 1).text())); d.append(float(self.table.item(i, 2).text()))
            except ValueError as exc:
                raise ValueError(f'{name}: not a number') from exc
        k, d = np.array(k), np.array(d)
        if np.any(k <= 0) or np.any(d < 0) or d[0] <= 0:
            raise ValueError('Stiffness must be positive; damping non-negative (joint 2 damping positive).')
        return k, d

    def update_ratio(self, item):
        if item.column() != 1:
            return
        try:
            k, _ = self.values()
        except ValueError:
            return
        self.table.blockSignals(True)
        for i in range(len(self.names)):
            self.table.item(i, 3).setText(f'{k[i] / k[0]:.4f}')
        self.table.blockSignals(False)

    def fill_exponential(self):
        i = np.arange(len(self.names))
        self.set_values(self.k2.value() * self.r.value() ** i, self.d2.value() * self.rd.value() ** i)

    def scale_k(self):
        try:
            k, d = self.values()
        except ValueError as exc:
            self.status.setText(str(exc)); return
        self.set_values(k * self.scale.value(), d)

    def damping_from_k(self):
        try:
            k, d = self.values()
        except ValueError as exc:
            self.status.setText(str(exc)); return
        self.set_values(k, d[0] * np.sqrt(k / k[0]))

    def reset(self):
        self.set_values(self.xml_k, self.xml_d); self.status.setText('Reset to the XML values (not yet applied).')

    def load_csv(self):
        import csv
        name, _ = QFileDialog.getOpenFileName(self, 'Load joint_fit.csv from a campaign folder', '', 'CSV (*.csv)')
        if not name:
            return
        try:
            rows = {r['joint']: float(r['k_Nm_per_rad']) for r in csv.DictReader(open(name))}
            k = np.array([rows[n] for n in self.names])
        except (KeyError, ValueError, OSError) as exc:
            self.status.setText('Could not read joint stiffness: ' + str(exc)); return
        _, d = self.values()
        self.set_values(k, d)
        self.status.setText('Stiffness loaded from the joint fit; damping unchanged. Apply to see it.')

    def apply(self):
        try:
            k, d = self.values()
            self.apply_callback(k, d)
            self.status.setText('Sent to the simulation. Watch the red backbone in the main window.')
        except ValueError as exc:
            self.status.setText(str(exc))

    def save(self):
        try:
            k, d = self.values()
        except ValueError as exc:
            self.status.setText(str(exc)); return
        name, _ = QFileDialog.getSaveFileName(self, 'Save joint values into an XML', str(self.model_path), 'MuJoCo XML (*.xml)')
        if not name:
            return
        target = Path(name)
        try:
            note = save_with_backup(self.model_path, self.names, k, d, target)
            self.status.setText(f'Saved {target.name}. {note}')
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, 'Not saved', str(exc))
