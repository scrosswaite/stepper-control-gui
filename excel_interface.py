import sys
import time # <-- Add this import
from app.main_window import MainWindow
from PyQt5.QtWidgets import QApplication

def main():
    # This function now prints every step it takes.
    print("--- Script Started ---")

    app = QApplication(sys.argv)
    print("QApplication instance created.")

    window = MainWindow()
    print("MainWindow instance created.")

    print(f"Received arguments: {sys.argv}")

    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        print(f"Command identified: '{command}'")

        try:
            print("Attempting to list serial ports...")
            available_ports = window.serial.list_ports()
            if not available_ports:
                print("ERROR: No serial ports found.")
                sys.exit(1)

            print(f"Found ports: {available_ports}. Connecting to '{available_ports[0]}'...")
            window.serial.connect(available_ports[0])
            print(f"SUCCESS: Connected to {available_ports[0]}")

        except Exception as e:
            print(f"ERROR: Failed to connect to serial port. Exception: {e}")
            sys.exit(1)

        print(f"Executing command: {command}")

        # --- Command execution ---
        if command == "level":
            window._send_level()
        elif command == "up":
            window._send_up()
        elif command == "down":
            window._send_down()
        elif command == "zero":
            window._send_zero()
        elif command == "dunk":
            window._send_dunk()
        elif command == "estop":
            window._send_estop()
        else:
            print(f"ERROR: Unknown command '{command}'")

        print("Command sent. Waiting for 0.5s before exit...")
        time.sleep(0.5)
        print("--- Script Finished ---")
        sys.exit(0)

    else:
        # This part runs if you don't provide a command (i.e., you want the GUI)
        print("No command argument found, starting GUI.")
        window.resize(900, 600)
        window.show()
        sys.exit(app.exec_())

if __name__ == '__main__':
            main()