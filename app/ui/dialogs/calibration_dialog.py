from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel,
    QSpinBox, QDoubleSpinBox, QComboBox, QPushButton
)
from PyQt5.QtCore import Qt

class CalibrationSettingsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Calibration Settings")
        self.setModal(True)
        layout = QVBoxLayout(self)

        # Interval input
        interval_row = QHBoxLayout()
        interval_row.addWidget(QLabel("Step Interval:"))
        self.minutes_spin = QSpinBox()
        self.minutes_spin.setRange(0,59); self.minutes_spin.setSuffix(" min")
        interval_row.addWidget(self.minutes_spin)
        self.seconds_spin = QSpinBox()
        self.seconds_spin.setRange(0,59); self.seconds_spin.setSuffix(" sec")
        interval_row.addWidget(self.seconds_spin)
        layout.addLayout(interval_row)

        # Units dropdown
        unit_row = QHBoxLayout()
        unit_row.addWidget(QLabel("Units:"))
        self.unit_combo = QComboBox()
        self.unit_combo.addItems(["Degrees","Millimeters"])
        unit_row.addWidget(self.unit_combo)
        layout.addLayout(unit_row)

        # Step value
        value_row = QHBoxLayout()
        value_row.addWidget(QLabel("Step Value:"))
        self.step_spin = QDoubleSpinBox()
        self.step_spin.setRange(0.1,360.0); self.step_spin.setSingleStep(0.1)
        self.step_spin.setValue(90.0)
        value_row.addWidget(self.step_spin)
        layout.addLayout(value_row)

        # Direction
        dir_row = QHBoxLayout()
        dir_row.addWidget(QLabel("Direction:"))
        self.dir_combo = QComboBox()
        self.dir_combo.addItems(["Up","Down"])
        dir_row.addWidget(self.dir_combo)
        layout.addLayout(dir_row)

        # Buttons
        btn_row = QHBoxLayout()
        self.apply_btn = QPushButton("Apply Settings")
        self.skip_btn  = QPushButton("Skip Step")
        self.cancel_btn = QPushButton("Cancel")
        btn_row.addWidget(self.apply_btn)
        btn_row.addWidget(self.skip_btn)
        btn_row.addWidget(self.cancel_btn)
        layout.addLayout(btn_row)

        self.apply_btn.clicked.connect(self.accept)
        self.cancel_btn.clicked.connect(self.reject)
        self.skip_btn.clicked.connect(self._skip)
        self._skip_requested = False

    def get_interval_ms(self):
        return (self.minutes_spin.value()*60 + self.seconds_spin.value())*1000

    def get_units(self):
        return self.unit_combo.currentText()

    def get_step_value(self):
        return self.step_spin.value()

    def get_direction(self):
        return 1 if self.dir_combo.currentText()=="Up" else -1

    def skip_requested(self):
        return self._skip_requested

    def _skip(self):
        self._skip_requested = True
        self.accept()
