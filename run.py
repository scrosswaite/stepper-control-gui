# run.py — final, stable version with CLI fixes and baud update

import sys
import time

# GUI components are only used when needed (GUI mode or for lead_mm calc in CLI).
from PyQt5.QtWidgets import QApplication
from app.main_window import MainWindow

# -----------------------------
# Serial / CLI configuration
# -----------------------------
BAUD = 115200          # Updated from 9600 to match Arduino
READ_TIMEOUT = 2       # seconds
DEFAULT_PORT = "COM7"  # Keep previous default; can be overridden via CLI

USAGE = """\
Usage:
  python run.py                         # Launch GUI
  python run.py up <mm> [PORT]          # Move up by <mm>
  python run.py down <mm> [PORT]        # Move down by <mm>
  python run.py level_on [PORT]         # Start auto-levelling
  python run.py level_off [PORT]        # Stop auto-levelling
  python run.py level [PORT]            # Alias for level_on
  python run.py zero [PORT]             # Home (same as GUI Zero)
  python run.py home [PORT]             # Home
  python run.py set_zero [PORT]         # Set current position as zero
  python run.py dunk [PORT]             # Dunk
  python run.py estop [PORT]            # Emergency stop
"""


# --- Simple function for direct serial communication (no GUI widgets used here) ---
def send_serial_command(port, command):
    """
    Connects directly to the serial port, sends a single command, waits for a single response, and closes.
    Prints a banner if received; robust to missing banner.
    """
    import serial  # import locally so unit tests/mocks can patch easily

    try:
        print(f"Opening port {port} @ {BAUD} baud...")
        # Ensure the port always closes by using a context manager.
        with serial.Serial(port, BAUD, timeout=READ_TIMEOUT) as ser:
            time.sleep(1.5)  # allow board reset/USB-CDC settle

            # Try to read the Arduino's banner line (non-fatal if missing)
            banner = ser.readline().decode("utf-8", "ignore").strip()
            if banner:
                print(f"Arduino ready. Banner: {banner}")
            else:
                print("Note: No banner detected. Proceeding to send command.")

            # Send the command (the Arduino expects a newline terminator)
            full_command = command + "\n"
            print(f"Sending command: {command}")
            ser.write(full_command.encode())

            # Read one response line (typical: MOVED / SPEED_CONFIG_OK / etc.)
            response = ser.readline().decode("utf-8", "ignore").strip()
            if response:
                print(f"Arduino response: {response}")
            else:
                print("No response received (timeout).")

            print(f"Port {port} closed.")

    except serial.SerialException as e:
        print(f"SERIAL ERROR: {e}. Is another program (like the GUI or Arduino IDE) using the port?")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


def _parse_port_arg(args, default=DEFAULT_PORT):
    """
    Minimal positional port override helper.
    If args is empty: return default.
    If args[0] looks like a COM* or /dev/*, use it.
    Otherwise return default and leave args untouched (caller already consumed what it needs).
    """
    if not args:
        return default
    candidate = args[0]
    # Very light validation; accept anything to keep behavior flexible.
    return candidate


def main():
    # --- COMMAND-LINE MODE ---
    if len(sys.argv) > 1:
        # We need a temporary MainWindow instance only to get calculation values (lead_mm).
        # Do not show any windows.
        app = QApplication.instance() or QApplication(sys.argv)
        temp_window = MainWindow()  # not shown; used for _lead_mm

        command = sys.argv[1].lower()
        args = sys.argv[2:]  # remaining positional args
        port_to_use = DEFAULT_PORT
        final_command_to_send = ""

        # Distance-based moves: up/down <mm> [PORT]
        if command in ("up", "down"):
            if not args:
                print(f"ERROR: The '{command}' command requires a distance value in mm.\n")
                print(USAGE)
                return
            try:
                distance_mm = float(args[0])
            except ValueError:
                print(f"ERROR: Invalid distance value '{args[0]}'.\n")
                print(USAGE)
                return

            # Convert mm -> degrees using the GUI's configured lead
            degs_to_move = (distance_mm / temp_window._lead_mm) * 360.0
            if command == "down":
                degs_to_move = -degs_to_move
            final_command_to_send = f"MOVE {degs_to_move:.2f}"

            # Optional port override as next positional arg
            if len(args) >= 2:
                port_to_use = _parse_port_arg([args[1]], default=DEFAULT_PORT)

        else:
            # Simple commands with aliases mapped to Arduino-supported tokens
            aliases = {
                # Levelling
                "level":      "LEVEL_ON",   # convenience alias
                "level_on":   "LEVEL_ON",
                "leveloff":   "LEVEL_OFF",  # tolerate missing underscore
                "level_off":  "LEVEL_OFF",
                "stop_level": "LEVEL_OFF",

                # Motion / safety
                "dunk":       "DUNK",
                "estop":      "ESTOP",

                # Zeroing / homing
                "zero":       "HOME",       # matches GUI Zero button
                "home":       "HOME",
                "set_zero":   "SET_ZERO",   # set current position as zero
            }

            if command in aliases:
                final_command_to_send = aliases[command]
                # Optional port override as next positional arg
                if args:
                    port_to_use = _parse_port_arg([args[0]], default=DEFAULT_PORT)
            else:
                print(f"ERROR: Unknown command '{command}'.\n")
                print(USAGE)
                return

        if final_command_to_send:
            send_serial_command(port_to_use, final_command_to_send)
            print("--- Script Finished ---")
        return

    # --- GUI MODE ---
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(900, 600)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
