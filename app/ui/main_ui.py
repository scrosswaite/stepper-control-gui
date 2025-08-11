import os
import pyqtgraph.opengl as gl
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure 
from PyQt5.QtCore    import Qt
from PyQt5.QtGui     import QFontMetrics, QPixmap
from PyQt5.QtWidgets import (
    QWidget, QGroupBox, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLabel, QComboBox, QLineEdit, QDoubleSpinBox, QSpinBox,
    QPushButton, QSizePolicy, QGridLayout, QTabWidget, QProgressBar,
    QTextEdit
)

def setup_ui(window):
    # ── Hydramotion Logo ──
    window.logo_label = QLabel()
    ui_dir        = os.path.dirname(os.path.abspath(__file__))
    resources_dir = os.path.abspath(os.path.join(ui_dir, "..", "..", "resources"))
    logo_path     = os.path.join(resources_dir, "logo.png")
    pix = QPixmap(logo_path)
    if pix.isNull():
        print(f"[ui_main] failed to load logo at {logo_path}")
    else:
        pix = pix.scaledToHeight(40, Qt.SmoothTransformation)
        window.logo_label.setPixmap(pix)

    # — Widgets —
    window.port_combo        = QComboBox()
    window.refresh_btn       = QPushButton("Refresh")
    window.connect_btn       = QPushButton("Connect")
    window.refresh_btn.setObjectName("refreshButton")
    window.connect_btn.setObjectName("connectButton")

    window.arduino_indicator = QLabel()
    window.arduino_indicator.setFixedSize(16, 16)
    window.arduino_indicator.setStyleSheet(
        "background-color: red; border: 1px solid black;"
    )

    window.status = QLabel("Disconnected")

    window.fluid_input    = QLineEdit()
    window.fluid_input.setPlaceholderText("Enter fluid type…")
    window.distance_label = QLabel("0.00")
    window.depth_label    = QLabel("0.00")

    window.state_label = QLabel("stationary")
    fnt = window.state_label.font()
    fnt.setBold(True)
    window.state_label.setFont(fnt)

    window.motor_indicator = QLabel()
    window.motor_indicator.setFixedSize(16, 16)
    window.motor_indicator.setStyleSheet(
        "background-color: green; border: 1px solid black;"
    )

    window.unit_combo = QComboBox()
    window.unit_combo.addItems(["Degrees", "Millimeters"])
    window.value_spin = QDoubleSpinBox()
    window.value_spin.setRange(-360, 360)
    window.value_spin.setSingleStep(1)
    window.value_spin.setValue(60)

    # — Action buttons —
    def mk(name, text):
        b = QPushButton(text)
        b.setObjectName(name)
        return b

    window.estop_btn  = mk("estopButton",  "E‑Stop")
    window.dunk_btn   = mk("dunkButton",   "Dunk")
    window.zero_btn   = mk("zeroButton",   "Zero")
    window.calib_btn  = mk("calibButton",  "Calibrate")
    window.up_btn     = mk("upDownButton", "▲")
    window.down_btn   = mk("upDownButton", "▼")
    window.cancel_btn = mk("cancelButton", "Cancel Calibration")

    # — Command Input Section —
    window.cmd_unit_combo = QComboBox()
    window.cmd_unit_combo.addItems(["Degrees", "Millimeters"])
    window.cmd_value_spin = QDoubleSpinBox()
    window.cmd_value_spin.setRange(-360, 360)
    window.cmd_value_spin.setSingleStep(1)
    window.cmd_value_spin.setValue(45)
    window.cmd_send_btn = QPushButton("Send")
    window.cmd_send_btn.setObjectName("cmdSendButton")
    window.cmd_dir_combo = QComboBox()
    window.cmd_dir_combo.addItems(["Up", "Down"])
    window.settings_btn = QPushButton("Settings")
    window.settings_btn.setObjectName("settingsButton")
    window.cancel_cmd_btn = QPushButton("Cancel Command")
    window.cancel_cmd_btn.setObjectName("cancelCommandButton")
    window.zero_cmd_btn   = QPushButton("Set Zero Position")
    window.zero_cmd_btn.setObjectName("zeroCommandButton")

    # Make *all* of these horizontally expanding
    for btn in (
        window.estop_btn, window.dunk_btn, window.zero_btn,
        window.calib_btn, window.up_btn, window.down_btn,
        window.cancel_btn, window.cmd_send_btn,
        window.settings_btn, window.cancel_cmd_btn, window.zero_cmd_btn
    ):
        btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    # — Communications Group —
    comms = QGroupBox("Communications")
    c_lay = QHBoxLayout(comms)
    c_lay.addWidget(QLabel("Serial Port:"))
    c_lay.addWidget(window.port_combo)
    c_lay.addWidget(window.refresh_btn)
    c_lay.addWidget(window.connect_btn)
    c_lay.addStretch()
    c_lay.addWidget(QLabel("Arduino:"))
    c_lay.addWidget(window.arduino_indicator)

    # — Fluid Info Group —
    fluid = QGroupBox("Fluid Information")
    f_lay = QFormLayout(fluid)
    f_lay.addRow("Fluid:",                   window.fluid_input)
    f_lay.addRow("Distance from Zero (mm):", window.distance_label)
    f_lay.addRow("Bob Depth (mm):",          window.depth_label)

    # Motor visual indicator
    #f_lay.addRow(QLabel("Motor Position:"))
    #window.position_bar = QProgressBar()
    #window.position_bar.setTextVisible(True)
    #window.position_bar.setFormat("%v steps")
    #f_lay.addRow(window.position_bar)



    # — Controls Group —
    ctrl = QGroupBox("Controls")
    ctrl_layout = QVBoxLayout(ctrl)
    ctrl_layout.setSpacing(8)

    window.calib_progress        = QProgressBar()
    window.calib_progress.setRange(0, 10)
    window.calib_progress.setValue(0)
    window.calib_progress.setTextVisible(True)
    window.calib_countdown_label = QLabel("Next step in: 0s")
    window.calib_countdown_label.setAlignment(Qt.AlignCenter)

    # First row: E‑Stop, Dunk, Zero – each with equal stretch
    row1 = QHBoxLayout()
    for b in (window.estop_btn, window.dunk_btn, window.zero_btn):
        row1.addWidget(b, 1)
    ctrl_layout.addLayout(row1)

    # Second row: Calibrate, up/down panel, Cancel Calibration
    row2 = QHBoxLayout()
    window.dir_panel = QWidget()
    dir_layout = QVBoxLayout(window.dir_panel)
    dir_layout.setContentsMargins(0, 0, 0, 0)
    dir_layout.setSpacing(4)
    dir_layout.setAlignment(Qt.AlignCenter)
    dir_layout.addWidget(window.up_btn)
    dir_layout.addWidget(window.down_btn)
    window.dir_panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    for w in (window.calib_btn, window.dir_panel, window.cancel_btn):
        row2.addWidget(w, 1)
    ctrl_layout.addLayout(row2)


    ctrl_layout.addWidget(window.calib_progress)
    ctrl_layout.addWidget(window.calib_countdown_label)

    # — Status Group —
    status = QGroupBox("Status")
    st = QGridLayout(status)
    window.led_power  = QLabel(); window.led_power .setFixedSize(12,12)
    window.led_ready  = QLabel(); window.led_ready .setFixedSize(12,12)
    window.led_moving = QLabel(); window.led_moving.setFixedSize(12,12)
    window.led_error  = QLabel(); window.led_error .setFixedSize(12,12)
    st.addWidget(QLabel("Power"),  0,0); st.addWidget(window.led_power,  0,1)
    st.addWidget(QLabel("Ready"),  1,0); st.addWidget(window.led_ready,  1,1)
    st.addWidget(QLabel("Moving"), 2,0); st.addWidget(window.led_moving, 2,1)
    st.addWidget(QLabel("Error"),  3,0); st.addWidget(window.led_error,  3,1)

    # — Command Input Group —
    command = QGroupBox("Command Input")
    cmd_layout = QFormLayout(command)
    cmd_layout.addRow("Direction:", window.cmd_dir_combo)
    cmd_layout.addRow("Units:",     window.cmd_unit_combo)
    cmd_layout.addRow("Value:",     window.cmd_value_spin)
    cmd_layout.addRow(window.cmd_send_btn)
    cmd_layout.addRow(window.settings_btn)
    cmd_layout.addRow(window.cancel_cmd_btn)
    cmd_layout.addRow(window.zero_cmd_btn)

    # Manual command input
    cmd_layout.addRow(QLabel("-----------"))
    window.manual_cmd_input = QLineEdit()
    window.manual_cmd_input.setPlaceholderText("e.g., MOVE -90")
    cmd_layout.addRow("Manual Cmd:", window.manual_cmd_input)
    window.manual_cmd_send_btn = QPushButton("Send Manual Command")
    cmd_layout.addRow(window.manual_cmd_send_btn)

    # Command Log Group
    log_group = QGroupBox("Command Log")
    log_layout = QVBoxLayout(log_group)
    window.command_log = QTextEdit()
    window.command_log.setReadOnly(True)
    #window.command_log.setStyleSheet("QTextEdit { background-color: #222; color #eee; font-family: Consolas, monospace;}")
    log_layout.addWidget(window.command_log)

    # — Tabs assembly —
    tabs = QTabWidget()
    # Calibration tab
    cal_tab = QWidget()
    cal_layout = QVBoxLayout(cal_tab)
    cal_layout.addWidget(window.logo_label, alignment=Qt.AlignLeft)
    cal_layout.addWidget(comms)
    cal_layout.addWidget(log_group)
    cal_layout.addWidget(fluid)
    cal_layout.addWidget(command)
    bottom = QHBoxLayout()
    bottom.addWidget(ctrl)
    bottom.addWidget(status)
    cal_layout.addLayout(bottom)
    cal_layout.addStretch()
    tabs.addTab(cal_tab, "Calibration")

    # Create UI
        
    # 1) Create the levelling‐system controls
    window.begin_lvl_btn = QPushButton("Begin Levelling")
    window.tilt_x_label  = QLabel("0.00")
    window.tilt_y_label  = QLabel("0.00")

    lev_group = QGroupBox("Levelling System")
    lev_form  = QFormLayout(lev_group)
    lev_form.addRow("Tilt X (°):", window.tilt_x_label)
    lev_form.addRow("Tilt Y (°):", window.tilt_y_label)
    lev_form.addRow("Tilt Z (°):", window.tilt_z_label) # unavaliable until 9DoF accelerometer used
    lev_form.addRow(window.begin_lvl_btn)

    # 2) Create the Matplotlib canvas
    window.tilt_fig    = Figure(figsize=(4, 2))
    window.tilt_canvas = FigureCanvas(window.tilt_fig)
    window.tilt_ax     = window.tilt_fig.add_subplot(111)
    window.tilt_ax.set_xlabel("Time (s)")
    window.tilt_ax.set_ylabel("Tilt (°)")
    window.tilt_ax.set_title("Tilt X / Y over Time")
    window.tilt_ax.minorticks_on()

    # 3) Build the Levelling tab
    lev_tab    = QWidget()
    lev_layout = QVBoxLayout(lev_tab)
    lev_layout.addWidget(lev_group)
    lev_layout.addWidget(window.tilt_canvas)

    # Add 3D view
    view_group = QGroupBox("Live Platform View")
    view_layout = QVBoxLayout(view_group)
    window.gl_view = gl.GLViewWidget()
    window.gl_view.setCameraPosition(distance=40)
    view_layout.addWidget(window.gl_view, 1)

    # controls
    top_layout = QHBoxLayout()
    top_layout.addWidget(view_group, 2)
    top_layout.addWidget(lev_group, 1)

    lev_layout.addLayout(top_layout)

    lev_layout.addWidget(window.tilt_canvas)

    
    # Export Button
    window.export_btn = QPushButton("Export Tilt Data")
    window.export_btn.setObjectName = ("exportButton")
    window.export_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
    lev_layout.addWidget(window.export_btn)

    lev_layout.addStretch()
    tabs.addTab(lev_tab, "Levelling")  
  

    # Settings tab
    setup_tab = QWidget()
    setup_layout = QVBoxLayout(setup_tab)

    # Wrap fields in a titled group box
    specs_group = QGroupBox("Stepper Motor Specifications")
    specs_form = QFormLayout(specs_group)
    setup_layout.addWidget(specs_group)

    # Preset positions 
    preset_group = QGroupBox("Preset Positions")
    preset_layout = QGridLayout(preset_group)

    window.save_pos_1_btn = QPushButton("Save 1")
    window.go_pos_1_btn = QPushButton("Go to 1")
    window.preset_1_label = QLabel("Not Set")
    preset_layout.addWidget(window.save_pos_1_btn, 0, 0)
    preset_layout.addWidget(window.go_pos_1_btn, 0, 1)
    preset_layout.addWidget(window.preset_1_label, 0, 2)

    window.save_pos_2_btn = QPushButton("Save 2")
    window.go_pos_2_btn = QPushButton("Go to 2")
    window.preset_2_label = QLabel("Not Set")
    preset_layout.addWidget(window.save_pos_2_btn, 1, 0)
    preset_layout.addWidget(window.go_pos_2_btn, 1, 1)
    preset_layout.addWidget(window.preset_2_label, 1, 2)

    window.save_pos_3_btn = QPushButton("Save 3")
    window.go_pos_3_btn = QPushButton("Go to 3")
    window.preset_3_label = QLabel("Not Set")
    preset_layout.addWidget(window.save_pos_3_btn, 2, 0)
    preset_layout.addWidget(window.go_pos_3_btn, 2, 1)
    preset_layout.addWidget(window.preset_3_label, 2, 2)

    setup_layout.addWidget(preset_group)

    # Create and add spinboxes in that group
    window.lead_spin = QDoubleSpinBox()
    window.lead_spin.setRange(0.1, 100.0)
    window.lead_spin.setSingleStep(0.1)
    window.lead_spin.setValue(window._lead_mm)
    specs_form.addRow("Lead (mm/rev):", window.lead_spin)

    window.steps_spin = QSpinBox()
    window.steps_spin.setRange(1, 10000)
    window.steps_spin.setValue(window._steps_per_rev)
    specs_form.addRow("Steps per rev:", window.steps_spin)

    window.click_spin = QDoubleSpinBox()
    window.click_spin.setRange(0.01, 100.0)
    window.click_spin.setSingleStep(0.01)
    window.click_spin.setValue(window._click_mm)
    specs_form.addRow("Click distance (mm):", window.click_spin)

    # 3) Your Apply button at the bottom, right‐aligned
    window.apply_settings_btn = QPushButton("Apply")
    setup_layout.addWidget(window.apply_settings_btn, alignment=Qt.AlignRight)

    # 4) Stretch to push everything up
    setup_layout.addStretch()

    # 5) Finally add this tab to the tab widget
    tabs.addTab(setup_tab, "Settings")

    # Ensure tabs are central widget
    window.setCentralWidget(tabs)

    # — Button Width Matching — (uses window.dir_panel)
    fm = QFontMetrics(window.cancel_btn.font())
    text_w = fm.horizontalAdvance(window.cancel_btn.text())
    base = text_w + 24 + 2
    for w in (
        window.estop_btn,
        window.dunk_btn,
        window.zero_btn,
        window.calib_btn,
        window.cancel_btn,
        window.dir_panel
    ):
        w.setMinimumWidth(base)
    base_width = window.cancel_btn.sizeHint().width()
    for w in (
        window.estop_btn,
        window.dunk_btn,
        window.zero_btn,
        window.calib_btn,
        window.cancel_btn,
        window.dir_panel
    ):
        w.setMinimumWidth(base_width)

    # — Style Sheet — (identical to your original)
    window.setStyleSheet("""
    QGroupBox {
      font-weight: bold;
      border: 1px solid #666;
      border-radius: 6px;
      background: #f9f9f9;
      margin-top: 10px;
    }
    QGroupBox::title {
      subcontrol-origin: margin;
      subcontrol-position: top center;
      padding: 0 4px;
    }
    QPushButton {
      background-color: #444;
      border: 1px solid #333;
      border-radius: 8px;
      padding: 6px 12px;
      color: white;
      font: 11pt "Segoe UI";
    }
    QPushButton:hover {
      background-color: #555;
    }
    QPushButton:pressed {
      background-color: #222;
    }
    QPushButton#estopButton {
      background-color: #c0392b;
    }
    QPushButton#estopButton:hover {
      background-color: #992d22;
    }
    QPushButton#dunkButton {
      background-color: #000;
      color: white;
    }
    QPushButton#dunkButton:hover {
      background-color: #333;
    }
    QPushButton#zeroButton {
      background-color: #27ae60;
    }
    QPushButton#zeroButton:hover {
      background-color: #1e8449;
    }
    QPushButton#calibButton {
      background-color: #3498db;
    }
    QPushButton#calibButton:hover {
      background-color: #2980b9;
    }
    QPushButton#upDownButton {
      background-color: #000;
      color: white;
      font: bold 16pt "Segoe UI";
    }
    QPushButton#upDownButton:hover {
      background-color: #222;
    }
    QPushButton#cancelButton {
      background-color: #f39c12;
    }
    QPushButton#cancelButton:hover {
      background-color: #e67e22;
    }
    QPushButton#refreshButton,
    QPushButton#connectButton {
      background-color: #ddd;
      color: black;
      border: 1px solid #aaa;
      font-weight: normal;
    }
    QPushButton#refreshButton:hover,
    QPushButton#connectButton:hover {
      background-color: #ccc;
    }
    QPushButton#cmdSendButton {
      background-color: #8e44ad;
    }
    QPushButton#cmdSendButton:hover {
      background-color: #732d91;
    }
    QPushButton#settingsButton {
      background-color: #888;
    }
    QPushButton#settingsButton:hover {
      background-color: #aaa;                 
    }
    """)
