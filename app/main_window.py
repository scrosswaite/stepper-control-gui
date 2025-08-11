import time
import csv
from collections import deque
from datetime import datetime

from PyQt5.QtWidgets import QMainWindow, QMessageBox, QDialog, QFileDialog
from PyQt5.QtCore import Qt, QTimer

from app.ui.main_ui import setup_ui
from app.ui.dialogs.settings_dialog import SettingsDialog
from app.ui.dialogs.calibration_dialog import CalibrationSettingsDialog
from app.controllers.serial_controller import SerialController
from app.controllers.calibration_manager import CalibrationManager
from app.utils.led_indicator import LedIndicator
from app.utils.config_manager import ConfigManager

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Stepper Control GUI")

        self.config_manager = ConfigManager()
        self.is_busy = False
        self._load_settings()

        self._tilt_buffer_len = 50
        self._tilt_buffer_x = deque(maxlen=self._tilt_buffer_len)
        self._tilt_buffer_y = deque(maxlen=self._tilt_buffer_len)

        setup_ui(self)
        QTimer.singleShot(0, self._update_settings_tab_fields)

        self._plot_start = time.time()
        self._times, self._tilt_xs, self._tilt_ys = deque(maxlen=100), deque(maxlen=100), deque(maxlen=100)
        self._all_times, self._all_tilt_xs, self._all_tilt_ys = [], [], []

        self.export_btn.clicked.connect(self._export_tilt_data)

        self.led_power  = LedIndicator(self.led_power)
        self.led_ready  = LedIndicator(self.led_ready)
        self.led_moving = LedIndicator(self.led_moving)
        self.led_error  = LedIndicator(self.led_error)
        self.led_power.on()
        self.led_ready.off()
        self.led_moving.off()
        self.led_error._widget.setStyleSheet("background-color: red; border:1px solid black; border-radius:6px;")

        self.serial = SerialController(
            on_data=self._on_serial_data,
            on_error=self._on_serial_error,
            on_disconnect=self._on_serial_disconnect
        )
        self.update_ports()

        self.calibration = CalibrationManager(
            serial=self.serial,
            update_progress=self._update_calib_progress,
            update_depth=self._update_depth_label,
            set_status=self._set_status,
            set_leds=(self.led_ready, self.led_moving, self.led_error),
            unlock_ui_callback=self._unlock_ui,
            steps_per_rev=self._steps_per_rev,
            lead_mm=self._lead_mm,
            click_mm=self._click_mm
        )

        self.refresh_btn.clicked.connect(self.update_ports)
        self.connect_btn.clicked.connect(self._toggle_connection)
        self.estop_btn.clicked.connect(self._send_estop)
        self.dunk_btn.clicked.connect(self._send_dunk)
        self.zero_btn.clicked.connect(self._send_zero)
        self.up_btn.clicked.connect(self._send_up)
        self.down_btn.clicked.connect(self._send_down)
        self.calib_btn.clicked.connect(self._start_calibration_dialog)
        self.cancel_btn.clicked.connect(self.calibration.cancel)
        self.begin_lvl_btn.clicked.connect(self._send_level)
        self.settings_btn.clicked.connect(self.open_settings)
        self.apply_settings_btn.clicked.connect(self._apply_settings)

    def _log_message(self, message, direction="RX"):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        prefix = ">>" if direction == "TX" else "<<"
        log_entry = f"[{timestamp}] {prefix} {message}"
        self.command_log.append(log_entry)
        self.command_log.verticalScrollBar().setValue(self.command_log.verticalScrollBar().maximum())

    def _lock_ui(self, message="Busy..."):
        self.is_busy = True
        self._set_status(message)
        self.led_moving.on()
        self.led_ready.off()
        for btn in (self.dunk_btn, self.zero_btn, self.calib_btn, self.up_btn, self.down_btn, self.connect_btn, self.begin_lvl_btn, self.settings_btn, self.apply_settings_btn):
            btn.setEnabled(False)

    def _unlock_ui(self, message="Ready"):
        self.is_busy = False
        self._set_status(message)
        self.led_moving.off()
        self.led_ready.on()
        for btn in (self.dunk_btn, self.zero_btn, self.calib_btn, self.up_btn, self.down_btn, self.connect_btn, self.begin_lvl_btn, self.settings_btn, self.apply_settings_btn):
            btn.setEnabled(True)

    def _on_serial_data(self, line: str):
        self._log_message(line, "RX")
        if line == "HOMED" or line.startswith("MOVED"):
            self._unlock_ui("Task complete.")
        if line == "CALIBRATE":
            self._start_calibration_dialog()
        if line.startswith("TILT"):
            data = line[4:].strip().split()
            if len(data) >= 2:
                try:
                    pitch, roll = float(data[0]), float(data[1])
                    self._tilt_buffer_x.append(pitch)
                    self._tilt_buffer_y.append(roll)
                    avg_p = sum(self._tilt_buffer_x) / len(self._tilt_buffer_x)
                    avg_r = sum(self._tilt_buffer_y) / len(self._tilt_buffer_y)
                    self.tilt_x_label.setText(f"{avg_p:.2f}")
                    self.tilt_y_label.setText(f"{avg_r:.2f}")
                    t = time.time() - self._plot_start
                    self._times.append(t)
                    self._tilt_xs.append(avg_p)
                    self._tilt_ys.append(avg_r)
                    self._all_times.append(t)
                    self._all_tilt_xs.append(avg_p)
                    self._all_tilt_ys.append(avg_r)
                    self._refresh_tilt_plot()
                except ValueError:
                    pass
        elif line.startswith("LIMIT"):
            QMessageBox.warning(self, "Limit Switch Hit", "A limit switch was activated!")
            self._unlock_ui("Limit reached — stopped")
            self.led_error.on()

    def _on_serial_error(self, err: Exception):
        self._set_status(f"Serial error: {err}")
        self._log_message(f"ERROR: {err}", "RX")
        self._unlock_ui("Serial error.")

    def _on_serial_disconnect(self):
        self.connect_btn.setText("Connect")
        self._set_arduino_indicator(False)
        self._unlock_ui("Disconnected")

    def _start_calibration_dialog(self):
        if not self._check_connection() or self.is_busy: return
        dlg = CalibrationSettingsDialog(self)
        if dlg.exec_() == QDialog.Accepted:
            self.calibration._interval_ms = dlg.get_interval_ms()
            self.calibration.step_value = dlg.get_step_value()
            self.calibration.step_units = dlg.get_units()
            self.calibration.calib_dir = dlg.get_direction()
            self._lock_ui("Starting calibration...")
            self.calibration.start()

    def _send_up(self):
        if not self._check_connection() or self.is_busy: return
        self._lock_ui("Moving up...")
        command = f"MOVE {self._click_deg}\n"
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())

    def _send_down(self):
        if not self._check_connection() or self.is_busy: return
        self._lock_ui("Moving down...")
        command = f"MOVE {-self._click_deg}\n"
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())

    def _send_zero(self):
        if not self._check_connection() or self.is_busy: return
        self._lock_ui("Homing...")
        command = "HOME\n"
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())

    def _send_dunk(self):
        if not self._check_connection() or self.is_busy: return
        self._lock_ui("Dunking...")
        command = "DUNK\n"
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())

    def _send_level(self):
        if not self._check_connection() or self.is_busy: return
        self._lock_ui("Starting levelling...")
        command = "LEVEL\n"
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())

    def _send_estop(self):
        if not self._check_connection(): return
        self._lock_ui("Sending E-Stop...")
        command = "ESTOP\n"
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())
        self._unlock_ui("E-STOP Sent.")

    def _load_settings(self):
        settings = self.config_manager.load_settings()
        self._lead_mm = float(settings.get("lead_mm", 4.0))
        self._steps_per_rev = int(settings.get("steps_per_rev", 200))
        self._click_mm = float(settings.get("click_mm", 0.5))
        self._click_deg = (self._click_mm / self._lead_mm) * 360.0
        self._comp_factor = 300 / 9.35

    def _update_settings_tab_fields(self):
        self.lead_spin.setValue(self._lead_mm)
        self.steps_spin.setValue(self._steps_per_rev)
        self.click_spin.setValue(self._click_mm)

    def _save_settings(self):
        settings_to_save = {"lead_mm": self._lead_mm, "steps_per_rev": self._steps_per_rev, "click_mm": self._click_mm}
        self.config_manager.save_settings(settings_to_save)

    def _apply_settings(self):
        self._lead_mm = self.lead_spin.value()
        self._steps_per_rev = self.steps_spin.value()
        self._click_mm = self.click_spin.value()
        self._click_deg = (self._click_mm / self._lead_mm) * 360.0
        self._set_status("Settings applied and saved.")
        self._save_settings()

    def open_settings(self):
        dlg = SettingsDialog(self)
        if dlg.exec_() == QDialog.Accepted:
            self._lead_mm = dlg.get_lead()
            self._steps_per_rev = dlg.get_steps()
            self._click_mm = dlg.get_click()
            self._click_deg = (self._click_mm / self._lead_mm) * 360.0
            self._update_settings_tab_fields()
            self._set_status("Settings updated.")
            self._save_settings()

    def closeEvent(self, event):
        reply = QMessageBox.question(self, "Confirm Exit", "Are you sure you want to quit?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            self._save_settings()
            event.accept()
        else:
            event.ignore()

    def update_ports(self):
        self.port_combo.clear()
        self.port_combo.addItems(self.serial.list_ports())

    def _check_connection(self):
        if not self.serial.is_connected:
            QMessageBox.critical(self, "Connection Error", "Arduino is not connected!")
            return False
        return True

    def _toggle_connection(self):
        if self.serial.is_connected:
            self.serial.disconnect()
        else:
            port = self.port_combo.currentText()
            if not port:
                QMessageBox.critical(self, "Connection Failed", "No serial port selected.")
                return
            try:
                self.serial.connect(port)
                self.connect_btn.setText("Disconnect")
                self._set_status(f"Connected to {port}")
                self.led_power.on()
                self._set_arduino_indicator(True)
            except Exception as e:
                QMessageBox.critical(self, "Connection Failed", str(e))
                self._set_status(f"Error: {e}")

    def _refresh_tilt_plot(self):
        self.tilt_ax.clear()
        self.tilt_ax.plot(self._times, self._tilt_xs, label="Tilt X", linewidth=2)
        self.tilt_ax.plot(self._times, self._tilt_ys, label="Tilt Y", linewidth=2)
        self.tilt_ax.grid(which="major", linestyle="--", linewidth=0.5)
        self.tilt_ax.legend(loc="upper right")
        self.tilt_ax.set_xlabel("Time (s)")
        self.tilt_ax.set_ylabel("Tilt (°)")
        self.tilt_ax.set_title("Tilt X / Y over Time")
        self.tilt_fig.tight_layout()
        self.tilt_canvas.draw()

    def _set_status(self, text: str):
        self.status.setText(text)

    def _update_calib_progress(self, step: int):
        self.calib_progress.setValue(step)

    def _update_depth_label(self, mm: float):
        self.depth_label.setText(f"{mm:.2f}")

    def _set_arduino_indicator(self, connected: bool):
        color = "green" if connected else "red"
        self.arduino_indicator.setStyleSheet(f"background-color: {color}; border: 1px solid black;")

    def _export_tilt_data(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export Tilt Data", "", "CSV Files (*.csv)")
        if not path: return
        try:
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Time (s)", "Tilt X (°)", "Tilt Y (°)"])
                for t, x, y in zip(self._all_times, self._all_tilt_xs, self._all_tilt_ys):
                    writer.writerow([f"{t:.3f}", f"{x:.3f}", f"{y:.3f}"])
            QMessageBox.information(self, "Export Successful", f"Data exported to:\n{path}")
        except Exception as e:
            QMessageBox.critical(self, "Export Failed", f"Could not write file:\n{e}")
    
    def _send_manual_command(self):
        """Sends a command from the manual input box."""
        if not self._check_connection() or  self.is_busy:
            return
        
        command_text = self.manual_cmd_input.text().strip()
        if not command_text:
            self._set_status("Manual command is empty.")
            return
        
        #UI is not lockde as assuming user knows what they are doing
        self._log_message(command_text, "TX")
        self.serial.send(f"{command_text}\n".encode())
        self.manual_cmd_input.clear() # Clear the input box after sending