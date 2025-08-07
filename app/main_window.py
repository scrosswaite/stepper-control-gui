import time
import csv
from collections import deque

from PyQt5.QtWidgets import QMainWindow, QMessageBox, QDialog, QFileDialog
from PyQt5.QtCore import Qt, QTimer

from app.ui.main_ui import setup_ui
from app.ui.dialogs.settings_dialog import SettingsDialog
from app.controllers.serial_controller import SerialController
from app.controllers.calibration_manager import CalibrationManager
from app.utils.led_indicator import LedIndicator
from app.utils.config_manager import ConfigManager

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Stepper Control GUI")

        # Config Manager
        self.config_manager = ConfigManager()

        # Load settings from file or use defaults
        self._load_settings()

        # Buffers for smoothing tilt readings
        self._tilt_buffer_len = 50
        self._tilt_buffer_x = deque(maxlen=self._tilt_buffer_len)
        self._tilt_buffer_y = deque(maxlen=self._tilt_buffer_len)

        # Build the UI (needs _lead_mm, etc.)
        setup_ui(self)
        
        # Update UI fields with loaded values
        QTimer.singleShot(0, self._update_settings_tab_fields)

        # Live Plotting
        self._plot_start = time.time()
        self._times   = deque(maxlen=100)
        self._tilt_xs = deque(maxlen=100)
        self._tilt_ys = deque(maxlen=100)
        
        # For full export history:
        self._all_times   = []
        self._all_tilt_xs = []
        self._all_tilt_ys = []

        self.export_btn.clicked.connect(self._export_tilt_data)

        # Wrap status LEDs
        self.led_power  = LedIndicator(self.led_power)
        self.led_ready  = LedIndicator(self.led_ready)
        self.led_moving = LedIndicator(self.led_moving)
        self.led_error  = LedIndicator(self.led_error)

        self.led_power.on()
        self.led_ready.off()
        self.led_moving.off()
        self.led_error._widget.setStyleSheet(
            "background-color: red; border:1px solid black; border-radius:6px;"
        )

        # Serial comms controller
        self.serial = SerialController(
            on_data=self._on_serial_data,
            on_error=self._on_serial_error
        )
        self.update_ports()

        # Calibration manager
        self.calibration = CalibrationManager(
            serial=self.serial,
            update_progress=self._update_calib_progress,
            update_depth=self._update_depth_label,
            set_status=self._set_status,
            set_leds=(self.led_ready, self.led_moving, self.led_error),
            steps_per_rev=self._steps_per_rev,
            lead_mm=self._lead_mm,
            click_mm=self._click_mm
        )

        # UI signal wiring
        self.refresh_btn.clicked.connect(self.update_ports)
        self.connect_btn.clicked.connect(self._toggle_connection)
        self.estop_btn.clicked.connect(self._send_estop)
        self.dunk_btn.clicked.connect(self._send_dunk)
        self.zero_btn.clicked.connect(self._send_zero)
        self.up_btn.clicked.connect(self._send_up)
        self.down_btn.clicked.connect(self._send_down)
        self.calib_btn.clicked.connect(self.calibration.start)
        self.cancel_btn.clicked.connect(self.calibration.cancel)
        self.begin_lvl_btn.clicked.connect(self._send_level)
        self.settings_btn.clicked.connect(self.open_settings)
        self.apply_settings_btn.clicked.connect(self._apply_settings)

    def _load_settings(self):
        """Load settings from config.json and apply them."""
        settings = self.config_manager.load_settings()
        self._lead_mm = float(settings.get("lead_mm", 4.0))
        self._steps_per_rev = int(settings.get("steps_per_rev", 200))
        self._click_mm = float(settings.get("click_mm", 0.5))
        self._click_deg = (self._click_mm / self._lead_mm) * 360.0
        self._comp_factor = 300 / 9.35

    def _update_settings_tab_fields(self):
        """Updates the spinboxes in the settings tab with current values."""
        self.lead_spin.setValue(self._lead_mm)
        self.steps_spin.setValue(self._steps_per_rev)
        self.click_spin.setValue(self._click_mm)

    def _save_settings(self):
        """Gather current settings and save them to the config file."""
        settings_to_save = {
            "lead_mm": self._lead_mm,
            "steps_per_rev": self._steps_per_rev,
            "click_mm": self._click_mm,
        }
        self.config_manager.save_settings(settings_to_save)
        self._set_status("Settings saved to config.json")

    def _apply_settings(self):
        self._lead_mm = self.lead_spin.value()
        self._steps_per_rev = self.steps_spin.value()
        self._click_mm = self.click_spin.value()
        self._click_deg = (self._click_mm / self._lead_mm) * 360.0
        self._set_status(
            f"Settings applied: lead={self._lead_mm:.2f} mm/rev, "
            f"steps={self._steps_per_rev}, click={self._click_mm:.2f} mm"
        )
        self._save_settings()

    def closeEvent(self, event):
        reply = QMessageBox.question(
            self, "Confirm Exit",
            "Are you sure you want to quit?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self._save_settings()
            event.accept()
        else:
            event.ignore()
            
    # --- The rest of your methods are below ---

    def update_ports(self):
        self.port_combo.clear()
        for p in self.serial.list_ports():
            self.port_combo.addItem(p)

    def _check_connection(self):
        if not self.serial.is_connected:
            QMessageBox.critical(
                self, "Connection Error",
                "Arduino is not connected!\nPlease connect before sending commands."
            )
            return False
        return True

    def _toggle_connection(self):
        if self.serial.is_connected:
            self.serial.disconnect()
            self.connect_btn.setText("Connect")
            self._set_status("Disconnected")
            self.led_power.off()
            self._set_arduino_indicator(False)
        else:
            port = self.port_combo.currentText()
            try:
                self.serial.connect(port)
                self.connect_btn.setText("Disconnect")
                self._set_status(f"Connected to {port}")
                self.led_power.on()
                self._set_arduino_indicator(True)
            except Exception as e:
                QMessageBox.critical(self, "Connection Failed", str(e))
                self._set_status(f"Error: {e}")
                self.led_power.off()
                self._set_arduino_indicator(False)

    def _on_serial_data(self, line: str):
        if line == "CALIBRATE":
            self.calibration.start()
        elif line == "ESTOP":
            self._set_status("Emergency Stop!")
            self.led_error.on(); self.led_ready.off(); self.led_moving.off()
        elif line.startswith("TILT"):
            data = line[4:].strip().split()
            if len(data) >= 2:
                try:
                    pitch = float(data[0]); roll = float(data[1])
                except ValueError:
                    return
                self._tilt_buffer_x.append(pitch)
                self._tilt_buffer_y.append(roll)
                avg_p = sum(self._tilt_buffer_x)/len(self._tilt_buffer_x)
                avg_r = sum(self._tilt_buffer_y)/len(self._tilt_buffer_y)
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
        elif line.startswith("LIMIT"):
            QMessageBox.warning(
                self, "Limit Switch Hit",
                "A limit switch was activated!\nMotor movement has stopped."
            )
            self._set_status("Limit reached — stopped")
            self.led_error.on(); self.led_moving.off()

    def _refresh_tilt_plot(self):
            ax = self.tilt_ax
            ax.clear()
            ax.plot(self._times, self._tilt_xs, label="Tilt X", linewidth=2)
            ax.plot(self._times, self._tilt_ys, label="Tilt Y", linewidth=2)
            ax.grid(which="major", linestyle="--", linewidth=0.5)
            ax.grid(which="minor", linestyle=":",  linewidth=0.3)
            ax.legend(loc="upper right", frameon=False)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Tilt (°)")
            ax.set_title("Tilt X / Y over Time")
            self.tilt_fig.tight_layout()
            self.tilt_canvas.draw()

    def _on_serial_error(self, err: Exception):
        self._set_status(f"Serial error: {err}")

    def _set_status(self, text: str):
        self.status.setText(text)

    def _update_calib_progress(self, step: int):
        self.calib_progress.setValue(step)

    def _update_depth_label(self, mm: float):
        self.depth_label.setText(f"{mm:.2f}")

    def _send_estop(self):
        if not self._check_connection(): return
        self.serial.send("ESTOP\n")

    def _send_dunk(self):
        if not self._check_connection(): return
        desired_mm = 50.0
        commanded_mm = desired_mm * self._comp_factor
        dunk_deg = (commanded_mm / self._lead_mm) * 360.0
        self.serial.send("DUNK\n")
        self.serial.send(f"MOVE {dunk_deg}\n")

    def _send_zero(self):
        if not self._check_connection(): return
        self.serial.send("HOME\n")
        self._set_status("Zero position set")

    def _send_up(self):
        if not self._check_connection(): return
        deg = self._click_deg
        self.serial.send(f"MOVE {deg}\n")

    def _send_down(self):
        if not self._check_connection(): return
        deg = -self._click_deg
        self.serial.send(f"MOVE {deg}\n")

    def _send_level(self):
        if not self._check_connection(): return
        self.serial.send("LEVEL\n")

    def open_settings(self):
        dlg = SettingsDialog(self)
        if dlg.exec_() == QDialog.Accepted:
            self._lead_mm = dlg.get_lead()
            self._steps_per_rev = dlg.get_steps()
            self._click_mm = dlg.get_click()
            self._click_deg = (self._click_mm / self._lead_mm) * 360.0
            self._set_status(
                f"Settings updated: lead={self._lead_mm:.2f} mm/rev, "
                f"steps={self._steps_per_rev}, click={self._click_mm:.2f} mm"
            )
            self._save_settings()

    def _set_arduino_indicator(self, connected: bool):
        color = "green" if connected else "red"
        self.arduino_indicator.setStyleSheet(
            f"background-color: {color}; border: 1px solid black;"
        )

    def _export_tilt_data(self):
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Export Tilt Data",
            "",
            "CSV Files (*.csv)"
        )
        if not path:
            return
        try:
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Time (s), Tilt X (°), Tilt Y (°)"])
                for t, x, y in zip(self._all_times, self._all_tilt_xs, self._all_tilt_ys):
                    writer.writerow([f"{t:.3f}", f"{x:.3f}", f"{y:3f}"])
        except Exception as e:
            QMessageBox.critical(self, "Export Failed", f"Could not write file:\n{e}")
            return
        QMessageBox.information(self, "Export Successful", f"Tilt data exported to :\n{path}")