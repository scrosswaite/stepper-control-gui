from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QFormLayout,
    QDoubleSpinBox, QSpinBox, QPushButton, QHBoxLayout
)

class SettingsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Settings")
        self.setModal(True)
        layout = QVBoxLayout(self)
        form = QFormLayout()
        layout.addLayout(form)

        # Lead screw pitch
        self.lead_spin = QDoubleSpinBox()
        self.lead_spin.setRange(0.1,100.0); self.lead_spin.setSingleStep(0.1)
        self.lead_spin.setValue(parent._lead_mm)
        form.addRow("Lead (mm/rev):", self.lead_spin)

        # Steps per rev
        self.steps_spin = QSpinBox()
        self.steps_spin.setRange(1,10000)
        self.steps_spin.setValue(parent._steps_per_rev)
        form.addRow("Steps per rev:", self.steps_spin)

        # Click distance
        self.click_spin = QDoubleSpinBox()
        self.click_spin.setRange(0.01,100.0); self.click_spin.setSingleStep(0.01)
        self.click_spin.setValue(parent._click_mm)
        form.addRow("Click distance (mm):", self.click_spin)

        # Buttons
        btns = QHBoxLayout(); btns.addStretch()
        apply = QPushButton("Apply"); cancel = QPushButton("Cancel")
        btns.addWidget(apply); btns.addWidget(cancel)
        layout.addLayout(btns)

        apply.clicked.connect(self.accept)
        cancel.clicked.connect(self.reject)

    def get_lead(self):
        return self.lead_spin.value()

    def get_steps(self):
        return self.steps_spin.value()

    def get_click(self):
        return self.click_spin.value()
