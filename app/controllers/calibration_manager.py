from PyQt5.QtCore import QObject, QTimer
from app.ui.dialogs.calibration_dialog import CalibrationSettingsDialog

class CalibrationManager(QObject):
    def __init__(self, serial, update_progress, update_depth,
                 set_status, set_leds,
                 steps_per_rev=200, lead_mm=4.0, click_mm=0.5):
        super().__init__()
        self.serial = serial
        self.update_progress = update_progress
        self.update_depth    = update_depth
        self.set_status      = set_status
        self.led_ready, self.led_moving, self.led_error = set_leds

        # mechanical params
        self.steps_per_rev = steps_per_rev
        self.lead_mm       = lead_mm
        self.click_mm      = click_mm
        self.click_deg     = (click_mm/lead_mm)*360.0
        self.comp_factor   = 300/9.35

        # state
        self._interval_ms = 600_000
        self.step_value   = 90.0
        self.step_units   = 'Millimeters'
        self.calib_dir    = 1
        self.calib_settings_applied = False
        self._stop_requested = False
        self._calib_step = 0
        self._calib_total_mm = 0.0
        self._seconds_remaining = 0

        # timers
        self.calib_timer = QTimer(self)
        self.calib_timer.timeout.connect(self._run_next_calib_step)
        self.countdown_timer = QTimer(self)
        self.countdown_timer.timeout.connect(self._tick_countdown)

    def start(self):
        if not self.calib_settings_applied:
            dlg = CalibrationSettingsDialog()
            if dlg.exec_() != dlg.Accepted:
                return
            self._interval_ms = dlg.get_interval_ms()
            self.step_value   = dlg.get_step_value()
            self.step_units   = dlg.get_units()
            self.calib_dir    = dlg.get_direction()
            self.calib_settings_applied = True
            self.set_status("Calibration settings applied. Press Calibrate to start.")
            return

        # begin sequence
        self.set_status("Calibrating: homing…")
        self.serial.send("HOME\n")
        QTimer.singleShot(1000, self._start_timer)

    def _start_timer(self):
        self._calib_step = 0
        self._calib_total_mm = 0.0
        self._stop_requested = False
        self.update_progress(0)
        self.update_depth(0.0)

        self.calib_timer.setInterval(self._interval_ms)
        self.countdown_timer.setInterval(1000)
        self._seconds_remaining = self._interval_ms // 1000

        self.set_status(f"Calib step 0/10: ready")
        self.calib_timer.start()
        self.countdown_timer.start()

    def _run_next_calib_step(self):
        if self._stop_requested:
            return self._finish()

        self._calib_step += 1
        self._seconds_remaining = self._interval_ms // 1000

        if self.step_units == "Degrees":
            deg = self.step_value * self.calib_dir
            mm  = (deg/360.0)*self.lead_mm
        else:
            mm = self.step_value
            commanded_mm = mm * self.comp_factor
            deg = (commanded_mm/self.lead_mm)*360.0*self.calib_dir

        self._calib_total_mm += abs(mm)
        self.update_depth(self._calib_total_mm)
        self.update_progress(self._calib_step)
        self.set_status(f"Calib step {self._calib_step}/10: {mm:.2f} mm")

        self.serial.send(f"MOVE {deg}\n")
        self.led_ready.off(); self.led_moving.on(); self.led_error.off()

        if self._calib_step >= 10:
            self.calib_timer.stop()
            self.set_status("Calibrating: homing back…")
            self.serial.send("HOME\n")
            QTimer.singleShot(1000, self._finish)

    def cancel(self):
        self._stop_requested = True
        self.calib_timer.stop()
        self.countdown_timer.stop()
        self.set_status("Calibration canceled")
        self.serial.send("HOME\n")

    def _finish(self):
        self.calib_timer.stop(); self.countdown_timer.stop()
        self.set_status("Calibration complete")
        self.led_moving.off(); self.led_ready.on()

    def _tick_countdown(self):
        if self._seconds_remaining > 0:
            self._seconds_remaining -= 1
        else:
            self.countdown_timer.stop()
