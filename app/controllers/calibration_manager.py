from PyQt5.QtCore import QObject, QTimer

class CalibrationManager(QObject):
    def __init__(self, serial, update_progress, update_depth, set_status, set_leds, unlock_ui_callback, steps_per_rev=200, lead_mm=4.0, click_mm=0.5):
        super().__init__()
        self.serial = serial
        self.update_progress = update_progress
        self.update_depth    = update_depth
        self.set_status      = set_status
        self.led_ready, self.led_moving, self.led_error = set_leds
        self.unlock_ui = unlock_ui_callback

        self.steps_per_rev = steps_per_rev
        self.lead_mm       = lead_mm
        self.click_mm      = click_mm
        self.click_deg     = (click_mm / lead_mm) * 360.0
        self.comp_factor   = 300 / 9.35

        self._interval_ms = 10000
        self.step_value   = 90.0
        self.step_units   = 'Degrees'
        self.calib_dir    = 1

        self._stop_requested = False
        self._calib_step = 0
        self._calib_total_mm = 0.0
        self._seconds_remaining = 0

        self.calib_timer = QTimer(self)
        self.calib_timer.timeout.connect(self._run_next_calib_step)
        self.countdown_timer = QTimer(self)
        self.countdown_timer.timeout.connect(self._tick_countdown)

    def start(self):
        self._calib_step = 0
        self._calib_total_mm = 0.0
        self._stop_requested = False
        self.update_progress(0)
        self.update_depth(0.0)

        self.set_status("Calibrating: homing…")
        # --- CORRECTED LINE ---
        self.serial.send("HOME\n".encode())
        QTimer.singleShot(2000, self._start_timer)

    def _start_timer(self):
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
            mm  = (deg / 360.0) * self.lead_mm
        else:
            mm = self.step_value
            commanded_mm = mm * self.comp_factor
            deg = (commanded_mm / self.lead_mm) * 360.0 * self.calib_dir

        self._calib_total_mm += abs(mm)
        self.update_depth(self._calib_total_mm)
        self.update_progress(self._calib_step)
        self.set_status(f"Calib step {self._calib_step}/10: Moving {mm:.2f} mm")

        # --- CORRECTED LINE ---
        self.serial.send(f"MOVE {deg}\n".encode())

        if self._calib_step >= 10:
            self.calib_timer.stop()
            self.set_status("Calibrating: homing back…")
            # --- CORRECTED LINE ---
            self.serial.send("HOME\n".encode())
            QTimer.singleShot(2000, self._finish)

    def cancel(self):
        self._stop_requested = True
        self.calib_timer.stop()
        self.countdown_timer.stop()
        self.set_status("Calibration canceled, returning home.")
        # --- CORRECTED LINE ---
        self.serial.send("HOME\n".encode())

    def _finish(self):
        self.calib_timer.stop()
        self.countdown_timer.stop()
        self.set_status("Calibration complete")
        self.unlock_ui()

    def _tick_countdown(self):
        if self._seconds_remaining > 0:
            self._seconds_remaining -= 1
            self.set_status(f"Next step in: {self._seconds_remaining}s")
        else:
            self.countdown_timer.stop()