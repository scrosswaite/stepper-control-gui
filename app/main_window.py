import time
import csv
import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore
from collections import deque
from datetime import datetime

from PyQt5.QtWidgets import QMainWindow, QMessageBox, QDialog, QFileDialog, QLabel, QSlider
from PyQt5.QtCore import Qt

from app.ui.main_ui import setup_ui
from app.ui.dialogs.settings_dialog import SettingsDialog
from app.ui.dialogs.calibration_dialog import CalibrationSettingsDialog
from app.controllers.serial_controller import SerialController
from app.controllers.calibration_manager import CalibrationManager
from app.utils.led_indicator import LedIndicator
from app.utils.config_manager import ConfigManager
from app.controllers.modbus_controller import ModbusWorker, ModbusConfig


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Stepper Control GUI")

        # -------------------------------
        # Core state & safe defaults (so setup_ui can read them)
        # -------------------------------
        self.config_manager = ConfigManager()
        self.is_busy = False

        # Motion / config defaults (overridden by _load_settings)
        self._lead_mm = 4.0
        self._steps_per_rev = 200
        self._click_mm = 0.5
        self._click_deg = (self._click_mm / self._lead_mm) * 360.0
        self._comp_factor = 300 / 9.35  # keep existing factor
        self.tilt_x_label = QLabel("0.00")
        self.tilt_y_label = QLabel("0.00")
        self.tilt_z_label = QLabel("0.00")

        # Position & presets
        self._preset_positions = {"1": None, "2": None, "3": None}
        self._current_position = 0
        self._waiting_for_pos_to_save = None

        # Tilt buffers (logic-only; UI comes later)
        self._tilt_buffer_len = 50
        self._tilt_buffer_x = deque(maxlen=self._tilt_buffer_len)
        self._tilt_buffer_y = deque(maxlen=self._tilt_buffer_len)
        self._tilt_buffer_z = deque(maxlen=self._tilt_buffer_len)

        # Plotting buffers
        self._plot_start = time.time()
        self._times, self._tilt_xs, self._tilt_ys = deque(maxlen=100), deque(maxlen=100), deque(maxlen=100)
        self._tilt_zs = deque(maxlen=100)
        self._all_times, self._all_tilt_xs, self._all_tilt_ys, self._all_tilt_zs = [], [], [], []

        # -------------------------------
        # Load persisted settings (NO UI calls here)
        # -------------------------------
        self._load_settings()

        # -------------------------------
        # Build UI (can safely read _lead_mm/_steps_per_rev/_click_mm)
        # -------------------------------
        setup_ui(self)

        # 3D view depends on UI widgets created above
        self._setup_3d_view()

        # Now it's safe to touch widgets
        self._update_settings_tab_fields()
        self._update_preset_labels()

        # -------------------------------
        # LEDs
        # -------------------------------
        self.led_power  = LedIndicator(self.led_power)
        self.led_ready  = LedIndicator(self.led_ready)
        self.led_moving = LedIndicator(self.led_moving)
        self.led_error  = LedIndicator(self.led_error)
        self.led_power.on()
        self.led_ready.off()
        self.led_moving.off()
        # Keep error LED red by default
        self.led_error._widget.setStyleSheet("background-color: red; border:1px solid black; border-radius:6px;")

        # -------------------------------
        # Serial & calibration
        # -------------------------------
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

        # -------------------------------
        # Signals
        # -------------------------------
        self.export_btn.clicked.connect(self._export_tilt_data)
        self.manual_cmd_send_btn.clicked.connect(self._send_manual_command)
        self.cmd_send_btn.clicked.connect(self._send_command_from_fields)
        self.cancel_cmd_btn.clicked.connect(self._cancel_command)
        self.zero_cmd_btn.clicked.connect(self._set_zero_here)


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

        self.save_pos_1_btn.clicked.connect(lambda: self._save_preset_position("1"))
        self.go_pos_1_btn.clicked.connect(lambda: self._go_to_preset_position("1"))
        self.save_pos_2_btn.clicked.connect(lambda: self._save_preset_position("2"))
        self.go_pos_2_btn.clicked.connect(lambda: self._go_to_preset_position("2"))
        self.save_pos_3_btn.clicked.connect(lambda: self._save_preset_position("3"))
        self.go_pos_3_btn.clicked.connect(lambda: self._go_to_preset_position("3"))

        self.send_motor_command_btn.clicked.connect(self._send_motor_command)

        # Speed Control Signals

        self.apply_speed_btn.clicked.connect(self._send_speed_settings)
        self.speed_slider.valueChanged.connect(self.max_speed_spin.setValue)
        self.max_speed_spin.valueChanged.connect(self.speed_slider.setValue)

        # ---- Viscometer data buffers ----
        self._visco_times = []
        self._visco_vl = []
        self._visco_tC = []
        self._visco_rho   = []
        self._visco_start = time.time()
        self._visco_max_points = 1200  # keep last ~20 min at 1 Hz

        # ---- STYLING FOR VISCOMETER PLOT ----
        # 1. Colors & Fonts (easy to customize here)
        self.BG_COLOR = "#1e1e1e" # Dark gray background
        self.TEXT_COLOR = "#d0d0d0" # Light gray text
        self.GRID_COLOR = "#404040" # Muted grid lines
        self.LINE_COLORS = {
            'temp': '#3498db', # Blue for temperature
            'visc': '#2ecc71', # Green for viscosity
            'rho': '#e67e22' # Orange for density
        }
        self.AXIS_LABEL_FONT = {'fontsize': 12, 'fontweight': 'bold', 'color': self.TEXT_COLOR}
        self.TITLE_FONT = {'fontsize': 14, 'fontweight': 'bold', 'color': self.TEXT_COLOR}

        # 2. Apply background colors
        self.visco_fig.set_facecolor(self.BG_COLOR)
        self.visco_ax.set_facecolor(self.BG_COLOR)
        
        # 3. Style spines (the plot border)
        for spine in self.visco_ax.spines.values():
            spine.set_edgecolor(self.GRID_COLOR)
        self.visco_ax2.spines['right'].set_edgecolor(self.LINE_COLORS['temp'])
        
        # 4. Style ticks
        self.visco_ax.tick_params(axis='x', colors=self.TEXT_COLOR)
        self.visco_ax.tick_params(axis='y', colors=self.TEXT_COLOR)
        self.visco_ax2.tick_params(axis='y', colors=self.LINE_COLORS['temp'])
        
        # 5. Pre-create line objects with new styles
        self._line_v = self.visco_ax.plot([], [], 'o-', label="Viscosity (cP)", linewidth=2, markersize=4, color=self.LINE_COLORS['visc'])[0]
        self._line_rho = self.visco_ax.plot([], [], 's--', label="Density (kg/m³)", linewidth=1.8, markersize=4, color=self.LINE_COLORS['rho'])[0]
        self._line_t = self.visco_ax2.plot([], [], '^-', label="Temperature (°C)", linewidth=2, markersize=4, color=self.LINE_COLORS['temp'])[0]

        # 6. Set axis labels and title with styled fonts
        self.visco_ax.set_title("Live Viscometer Data", fontdict=self.TITLE_FONT)
        self.visco_ax.set_xlabel("Time (s)", fontdict=self.AXIS_LABEL_FONT)
        self.visco_ax.set_ylabel("Viscosity (cP) / Density (kg/m³)", fontdict=self.AXIS_LABEL_FONT)
        self.visco_ax2.set_ylabel("Temperature (°C)", fontdict=self.AXIS_LABEL_FONT, rotation=270, labelpad=15)
        
        # 7. Style the grid
        self.visco_ax.grid(True, which='major', linestyle=':', linewidth=0.5, color=self.GRID_COLOR)
        
        # 8. Legend + toggles (connect ONCE)
        self._legend = None
        self.cb_v.stateChanged.connect(lambda s: self._toggle_line(self._line_v,   s))
        self.cb_t.stateChanged.connect(lambda s: self._toggle_line(self._line_t,   s))
        self.cb_rho.stateChanged.connect(lambda s: self._toggle_line(self._line_rho, s))

        # Set initial visibility to match checkboxes
        self._toggle_line(self._line_v,   self.cb_v.isChecked())
        self._toggle_line(self._line_t,   self.cb_t.isChecked())
        self._toggle_line(self._line_rho, self.cb_rho.isChecked())

        # ---- Start Modbus worker (set your working COM & mode) ----
        self._visco_worker = ModbusWorker(
            ModbusConfig(
                port="COM9", method="rtu", unit_id=1,
                baudrate=9600, parity="E", bytesize=8, stopbits=1,
                poll_ms=1000, byteorder="big", wordorder="big",
                density_float_addr=0x100B
                # If known, uncomment ONE of these lines:
                # density_float_addr=0x1020,
                # density_u16_addr=0x0020, density_scale_lo=900.0, density_scale_hi=1200.0,
            )
        )
        self._visco_worker.data.connect(self._on_visco_data)
        self._visco_worker.error.connect(lambda e: self._set_status(f"Viscometer: {e}"))
        self._visco_worker.connection_changed.connect(lambda ok: self._set_status(
            "Viscometer connected" if ok else "Viscometer disconnected"))
        self._visco_worker.start()


    # --------------------------------------------------------------------
    # Helpers / UI state
    # --------------------------------------------------------------------
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
        for btn in (self.dunk_btn, self.zero_btn, self.calib_btn, self.up_btn, self.down_btn,
                    self.connect_btn, self.begin_lvl_btn, self.settings_btn, self.apply_settings_btn):
            btn.setEnabled(False)

    def _unlock_ui(self, message="Ready"):
        self.is_busy = False
        self._set_status(message)
        self.led_moving.off()
        self.led_ready.on()
        for btn in (self.dunk_btn, self.zero_btn, self.calib_btn, self.up_btn, self.down_btn,
                    self.connect_btn, self.begin_lvl_btn, self.settings_btn, self.apply_settings_btn):
            btn.setEnabled(True)

    # --------------------------------------------------------------------
    # Serial data handling
    # --------------------------------------------------------------------
    def _on_serial_data(self, line: str):
        self._log_message(line, "RX")
        line = line.strip()

        if line == "HOMED":
            self._current_position = 0
            self._update_preset_labels()  # UI exists now, safe
            self._unlock_ui("Homed.")
            return

        if line.startswith("POS"):
            parts = line.split()
            if len(parts) >= 2:
                try:
                    steps = int(float(parts[1]))  # tolerate "123.0"
                    self._current_position = steps

                    # If we were saving a preset, store it now
                    if self._waiting_for_pos_to_save is not None:
                        slot = self._waiting_for_pos_to_save
                        self._preset_positions[slot] = steps
                        self._waiting_for_pos_to_save = None
                        self._update_preset_labels()
                        self._save_settings()
                        self._set_status(f"Preset {slot} saved.")
                except ValueError:
                    pass
            return

        # Completed move (do not adjust position here; rely on POS)
        if line.startswith("MOVED"):
            self._unlock_ui("Move complete.")
            return

        if line in ("STOPPING", "STOPPED" "ESTOP"):
            self._unlock_ui("Motion stopped.")
            return

        if line == "CALIBRATE":
            self._start_calibration_dialog()
            return

        if line.startswith("TILT"):
            data = line[4:].strip().split()
            if len(data) >= 2:
                try:
                    pitch = float(data[0])
                    roll  = float(data[1])
                    yaw   = 0.0  # placeholder

                    # Rotate body and axes
                    for item in [self.accel_body, self.x_axis, self.y_axis, self.z_axis]:
                        item.resetTransform()
                        item.rotate(pitch, 1, 0, 0)
                        item.rotate(roll,  0, 1, 0)

                    # Update buffers and labels
                    self._tilt_buffer_x.append(pitch)
                    self._tilt_buffer_y.append(roll)
                    self._tilt_buffer_z.append(yaw)
                    avg_p = sum(self._tilt_buffer_x) / len(self._tilt_buffer_x)
                    avg_r = sum(self._tilt_buffer_y) / len(self._tilt_buffer_y)
                    avg_y = sum(self._tilt_buffer_z) / len(self._tilt_buffer_z)
                    self.tilt_x_label.setText(f"{avg_p:.2f}")
                    self.tilt_y_label.setText(f"{avg_r:.2f}")
                    self.tilt_z_label.setText(f"{avg_y:.2f}")

                    # excel stuff
                    with open("tilt_data.csv", "w") as f:
                        f.write(f"{avg_p:.2f},{avg_r:.2f},{avg_y:.2f}")

                    # Update plots
                    t = time.time() - self._plot_start
                    self._times.append(t)
                    self._tilt_xs.append(avg_p)
                    self._tilt_ys.append(avg_r)
                    self._tilt_zs.append(avg_y)
                    self._all_times.append(t)
                    self._all_tilt_xs.append(avg_p)
                    self._all_tilt_ys.append(avg_r)
                    self._all_tilt_zs.append(avg_y)
                    self._refresh_tilt_plot()
                except ValueError:
                    pass
            return

        if line.startswith("LIMIT"):
            QMessageBox.warning(self, "Limit Switch Hit", "A limit switch was activated!")
            self._unlock_ui("Limit reached — stopped")
            self.led_error.on()
            return

        if any(line.upper().startswith(s) for s in ("CANCEL", "STOP", "ABORT")):
            self._unlock_ui("Command cancelled.")
            return

    def _on_serial_error(self, err: Exception):
        self._set_status(f"Serial error: {err}")
        self._log_message(f"ERROR: {err}", "RX")
        self._unlock_ui("Serial error.")

    def _on_serial_disconnect(self):
        self.connect_btn.setText("Connect")
        self._set_arduino_indicator(False)
        self._unlock_ui("Disconnected")

    # --------------------------------------------------------------------
    # Calibration
    # --------------------------------------------------------------------
    def _start_calibration_dialog(self):
        if not self._check_connection() or self.is_busy:
            return
        dlg = CalibrationSettingsDialog(self)
        if dlg.exec_() == QDialog.Accepted:
            self.calibration._interval_ms = dlg.get_interval_ms()
            self.calibration.step_value = dlg.get_step_value()
            self.calibration.step_units = dlg.get_units()
            self.calibration.calib_dir = dlg.get_direction()
            self._lock_ui("Starting calibration...")
            self.calibration.start()

    # --------------------------------------------------------------------
    # Commands
    # --------------------------------------------------------------------
    def _send_up(self):
        if not self._check_connection() or self.is_busy:
            return
        self._lock_ui("Moving up...")
        command = f"MOVE {self._click_deg}\n"
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())

    def _send_down(self):
        if not self._check_connection() or self.is_busy:
            return
        self._lock_ui("Moving down...")
        command = f"MOVE {-self._click_deg}\n"
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())

    def _send_zero(self):
        if not self._check_connection() or self.is_busy:
            return
        self._lock_ui("Homing...")
        command = "HOME\n"
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())

    def _send_dunk(self):
        if not self._check_connection() or self.is_busy:
            return
        self._lock_ui("Dunking...")
        command = "DUNK\n"
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())

    def _send_level(self):
        if not self._check_connection() or self.is_busy:
            return
        self._lock_ui("Starting levelling...")
        command = "LEVEL\n"
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())

    def _send_estop(self):
        if not self._check_connection():
            return
        self._lock_ui("Sending E-Stop...")
        command = "ESTOP\n"
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())
        self._unlock_ui("E-STOP Sent.")

    # --------------------------------------------------------------------
    # Settings & persistence
    # --------------------------------------------------------------------
    def _load_settings(self):
        """Load config values only. DO NOT touch UI here."""
        settings = self.config_manager.load_settings()
        self._lead_mm = float(settings.get("lead_mm", self._lead_mm))
        self._steps_per_rev = int(settings.get("steps_per_rev", self._steps_per_rev))
        self._click_mm = float(settings.get("click_mm", self._click_mm))
        self._click_deg = (self._click_mm / self._lead_mm) * 360.0
        self._comp_factor = settings.get("comp_factor", self._comp_factor)

        presets = settings.get("presets", {})
        for k in ("1", "2", "3"):
            v = presets.get(k, None)
            self._preset_positions[k] = int(v) if isinstance(v, (int, float)) else None

    def _update_settings_tab_fields(self):
        # Safe: widgets exist after setup_ui
        self.lead_spin.setValue(self._lead_mm)
        self.steps_spin.setValue(self._steps_per_rev)
        self.click_spin.setValue(self._click_mm)

    def _save_settings(self):
        settings_to_save = {
            "lead_mm": self._lead_mm,
            "steps_per_rev": self._steps_per_rev,
            "click_mm": self._click_mm,
            "comp_factor": self._comp_factor,
            "presets": self._preset_positions
        }
        self.config_manager.save_settings(settings_to_save)

    def _apply_settings(self):
        self._lead_mm = self.lead_spin.value()
        self._steps_per_rev = self.steps_spin.value()
        self._click_mm = self.click_spin.value()
        self._click_deg = (self._click_mm / self._lead_mm) * 360.0
        self._set_status("Settings applied and saved.")
        self._save_settings()
        self._update_preset_labels()

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
            self._update_preset_labels()

    def closeEvent(self, event):
        reply = QMessageBox.question(self, "Confirm Exit",
                                    "Are you sure you want to quit?",
                                    QMessageBox.Yes | QMessageBox.No,
                                    QMessageBox.No)
        if reply == QMessageBox.Yes:
            # stop viscometer worker if running
            try:
                if getattr(self, "_visco_worker", None):
                    self._visco_worker.stop()
                    self._visco_worker.wait(1500)
            except Exception:
                pass
            super().closeEvent(event)
            self._save_settings()
            event.accept()
        else:
            event.ignore()


    # --------------------------------------------------------------------
    # Connection
    # --------------------------------------------------------------------
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

                # Optional: sync position immediately
                self._log_message("GET_POS", "TX")
                self.serial.send(b"GET_POS\n")

                # self.position_bar.setRange(0, self.MAX_STEPS)
            except Exception as e:
                QMessageBox.critical(self, "Connection Failed", str(e))
                self._set_status(f"Error: {e}")

    # --------------------------------------------------------------------
    # Plotting
    # --------------------------------------------------------------------
    def _refresh_tilt_plot(self):
        self.tilt_ax.clear()
        self.tilt_ax.plot(self._times, self._tilt_xs, label="Tilt X", linewidth=2)
        self.tilt_ax.plot(self._times, self._tilt_ys, label="Tilt Y", linewidth=2)
        self.tilt_ax.plot(self._times, self._tilt_zs, label="Tilt Z", linewidth=2)
        self.tilt_ax.grid(which="major", linestyle="--", linewidth=0.5)
        self.tilt_ax.legend(loc="upper right")
        self.tilt_ax.set_xlabel("Time (s)")
        self.tilt_ax.set_ylabel("Tilt (°)")
        self.tilt_ax.set_title("Tilt X / Y over Time")
        self.tilt_fig.tight_layout()
        self.tilt_canvas.draw()

    # --------------------------------------------------------------------
    # Status & indicators
    # --------------------------------------------------------------------
    def _set_status(self, text: str):
        self.status.setText(text)

    def _update_calib_progress(self, step: int):
        self.calib_progress.setValue(step)

    def _update_depth_label(self, mm: float):
        self.depth_label.setText(f"{mm:.2f}")

    def _set_arduino_indicator(self, connected: bool):
        color = "green" if connected else "red"
        self.arduino_indicator.setStyleSheet(f"background-color: {color}; border: 1px solid black;")

    # --------------------------------------------------------------------
    # Export & manual command
    # --------------------------------------------------------------------
    def _export_tilt_data(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export Tilt Data", "", "CSV Files (*.csv)")
        if not path:
            return
        try:
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Time (s)", "Tilt X (°)", "Tilt Y (°)", "Tilt Z (°)"])
                for t, x, y, z in zip(self._all_times, self._all_tilt_xs, self._all_tilt_ys, self._all_tilt_zs):
                    writer.writerow([f"{t:.3f}", f"{x:.3f}", f"{y:.3f}", f"{z:.3f}"])
            QMessageBox.information(self, "Export Successful", f"Data exported to:\n{path}")
        except Exception as e:
            QMessageBox.critical(self, "Export Failed", f"Could not write file:\n{e}")

    def _send_manual_command(self):
        if not self._check_connection() or self.is_busy:
            return
        command_text = self.manual_cmd_input.text().strip()
        if not command_text:
            self._set_status("Manual command is empty.")
            return
        self._log_message(command_text, "TX")
        self.serial.send(f"{command_text}\n".encode())
        self.manual_cmd_input.clear()

    # --------------------------------------------------------------------
    # 3D view
    # --------------------------------------------------------------------
    def _setup_3d_view(self):
        # Create the rectangular body of the accelerometer
        verts = np.array([
            [-10, -5, -1], [10, -5, -1], [10, 5, -1], [-10, 5, -1],
            [-10, -5, 1], [10, -5, 1], [10, 5, 1], [-10, 5, 1]
        ])
        faces = np.array([
            [0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
            [2, 3, 7], [2, 7, 6], [0, 3, 7], [0, 7, 4], [1, 2, 6], [1, 6, 5]
        ])
        colors = np.array([[0.3, 0.8, 0.3, 0.8]] * 12)  # Semi-transparent green

        self.accel_body = gl.GLMeshItem(
            vertexes=verts, faces=faces, faceColors=colors,
            smooth=False, drawEdges=True, edgeColor=(0, 0, 0, 1)
        )
        self.gl_view.addItem(self.accel_body)

        # Create lines for the X, Y, and Z axes originating from the center
        origin = [0, 0, 0]
        self.x_axis = gl.GLLinePlotItem(pos=np.array([origin, [15, 0, 0]]), color=(1, 0, 0, 1), width=3)  # Red X
        self.y_axis = gl.GLLinePlotItem(pos=np.array([origin, [0, 10, 0]]), color=(0, 1, 0, 1), width=3)  # Green Y
        self.z_axis = gl.GLLinePlotItem(pos=np.array([origin, [0, 0, 10]]), color=(0, 0, 1, 1), width=3)  # Blue Z

        self.gl_view.addItem(self.x_axis)
        self.gl_view.addItem(self.y_axis)
        self.gl_view.addItem(self.z_axis)

        self.x_label = gl.GLTextItem(pos=[17, 0, 0], text='X')
        self.y_label = gl.GLTextItem(pos=[0, 12, 0], text='Y')
        self.z_label = gl.GLTextItem(pos=[0, 0, 12], text='Z')

        self.gl_view.addItem(self.x_label)
        self.gl_view.addItem(self.y_label)
        self.gl_view.addItem(self.z_label)

    # --------------------------------------------------------------------
    # Presets
    # --------------------------------------------------------------------
    def _update_preset_labels(self):
        """Updates the text labels for the preset positions.
        Safe-guarded so it won't crash if called early."""
        if not hasattr(self, "preset_1_label"):
            return
        steps_per_mm = self._steps_per_rev / self._lead_mm
        for num, pos in self._preset_positions.items():
            label = getattr(self, f"preset_{num}_label")
            if pos is not None:
                pos_mm = pos / steps_per_mm
                label.setText(f"{pos_mm:.2f} mm")
            else:
                label.setText("Not Set")

    def _save_preset_position(self, preset_num):
        """Asks the Arduino for its position to save it."""
        if not self._check_connection() or self.is_busy:
            return
        self._log_message(f"GET_POS (save preset {preset_num})", "TX")
        self._waiting_for_pos_to_save = preset_num
        self.serial.send(b"GET_POS\n")

    def _go_to_preset_position(self, preset_num):
        """Sends a command to move to a saved preset position."""
        if not self._check_connection() or self.is_busy:
            return
        target_pos = self._preset_positions.get(preset_num)
        if target_pos is None:
            self._set_status(f"Preset {preset_num} is not set.")
            return
        steps_to_move = target_pos - self._current_position
        degs_to_move = (steps_to_move / self._steps_per_rev) * 360.0

        # Optional: avoid tiny no-op moves
        if abs(degs_to_move) < 0.01:
            self._set_status(f"Already at Preset {preset_num}.")
            return

        command = f"MOVE {degs_to_move:.2f}\n"
        self._log_message(command.strip(), "TX")
        self._lock_ui(f"Moving to Preset {preset_num}...")
        self.serial.send(command.encode())

    # --------------------------------------------------------------------
    # Command entry
    # --------------------------------------------------------------------
    def _send_command_from_fields(self):
        if not self._check_connection() or self.is_busy:
            return
        sign = 1 if self.cmd_dir_combo.currentText().lower().startswith("up") else -1
        value = self.cmd_value_spin.value()
        if self.cmd_unit_combo.currentText().lower().startswith("mill"):
            value = (value / self._lead_mm) * 360.0
        command = f"MOVE {sign * value:.2f}\n"
        self._lock_ui("Moving...")
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())

    def _cancel_command(self):
        if not self._check_connection():
            return
        self._set_status("Cancelling command...")
        self._log_message("STOP", "TX")
        self.serial.send(b"STOP\n")

    def _set_zero_here(self):
        """Set current position as zero WITHOUT moving the motor."""
        if not self._check_connection() or self.is_busy:
            return
        self._set_status("Setting zero (no motion)...")
        self._log_message("SET_ZERO", "TX")
        self.serial.send(b"SET_ZERO\n")

    def _on_visco_data(self, d: dict):
        # Update styled info box labels
        try:
            self.vp_vl_label.setText(f"Viscosity: <span style='color:{self.LINE_COLORS['visc']};'>{d['vl']:.2f} cP</span>")
            self.vp_temp_label.setText(f"Temperature: <span style='color:{self.LINE_COLORS['temp']};'>{d['t']:.1f} °C</span>")
            if d.get("rho") is not None:
                self.vp_rho_label.setText(f"Density: <span style='color:{self.LINE_COLORS['rho']};'>{d['rho']:.1f} kg/m³</span>")
            else:
                self.vp_rho_label.setText(f"Density: <span style='color:{self.LINE_COLORS['rho']};'>-- kg/m³</span>")
        except Exception:
            pass

        # Buffers (use NaN for missing density so line gaps display cleanly)
        t = time.time() - self._visco_start
        self._visco_times.append(t)
        self._visco_vl.append(d["vl"])
        self._visco_tC.append(d["t"])
        self._visco_rho.append(d["rho"] if d.get("rho") is not None else float("nan"))

        # Trim window
        if len(self._visco_times) > self._visco_max_points:
            self._visco_times = self._visco_times[-self._visco_max_points:]
            self._visco_vl    = self._visco_vl[-self._visco_max_points:]
            self._visco_tC    = self._visco_tC[-self._visco_max_points:]
            self._visco_rho   = self._visco_rho[-self._visco_max_points:]

        self._refresh_visco_plot()

    def _refresh_visco_plot(self):
        if not hasattr(self, "visco_ax"):
            return
        x = self._visco_times

        # Update line data
        self._line_v.set_data(x, self._visco_vl)
        self._line_rho.set_data(x, self._visco_rho)
        self._line_t.set_data(x, self._visco_tC)

        # Rescale axes to data currently visible
        self.visco_ax.relim();  self.visco_ax.autoscale_view()
        self.visco_ax2.relim(); self.visco_ax2.autoscale_view()

        # Keep last window in view
        if x:
            xmin = max(0, x[-1] - self._visco_max_points)
            self.visco_ax.set_xlim(left=xmin, right=x[-1] + 1)

        self._rebuild_legend()
        self.visco_canvas.draw_idle()


    def _rebuild_legend(self):
        lines_visc = [l for l in (self._line_v, self._line_rho) if l.get_visible()]
        lines_temp = [self._line_t] if self._line_t.get_visible() else []
        
        # Place legend outside the plot area
        self.visco_ax.legend(lines_visc, [l.get_label() for l in lines_visc], loc='upper left', bbox_to_anchor=(1.05, 1),
                             facecolor=self.BG_COLOR, edgecolor=self.GRID_COLOR, labelcolor=self.TEXT_COLOR)
        self.visco_ax2.legend(lines_temp, [l.get_label() for l in lines_temp], loc='upper left', bbox_to_anchor=(1.05, 0.8),
                              facecolor=self.BG_COLOR, edgecolor=self.GRID_COLOR, labelcolor=self.TEXT_COLOR)


    def _toggle_line(self, line, state):
        line.set_visible(bool(state))
        self._rebuild_legend()
        self.visco_canvas.draw_idle()


    def _send_motor_command(self):
        if not self._check_connection() or self.is_busy:
            return

        actuator = self.actuator_combo.currentText()
        direction = self.motor_direction_combo.currentText()
        distance = self.motor_distance_spin.value()
        units = self.motor_unit_combo.currentText()

        # Determine the target motor index (0-2 for individual, 3 for all)
        if "Actuator 1" in actuator:
            motor_index = 0
        elif "Actuator 2" in actuator:
            motor_index = 1
        elif "Actuator 3" in actuator:
            motor_index = 2
        else: # All
            motor_index = 3

        # Determine the sign for the direction
        sign = 1 if direction == "Forward" else -1
        value = sign * distance

        # Create the command string
        # Format: "MOVE_M <motor_index> <value> <units>"
        command = f"MOVE_M {motor_index} {value} {units}\n"

        self._lock_ui(f"Moving {actuator}...")
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())

    def _send_speed_settings(self):
        if not self._check_connection():
            return
            
        max_speed = self.max_speed_spin.value()
        acceleration = self.accel_spin.value()

        # New command format: "CONFIG_SPEED <max_speed> <acceleration>"
        command = f"CONFIG_SPEED {max_speed} {acceleration}\n"

        self._set_status("Updating speed settings...")
        self._log_message(command.strip(), "TX")
        self.serial.send(command.encode())