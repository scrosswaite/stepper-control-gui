import sys
import time
from PyQt5.QtWidgets import QApplication
from app.main_window import MainWindow

def main():
    """
    This script now checks for command-line arguments.
    - If an argument (like 'up', 'down') is provided, it runs in command-line mode.
    - If no arguments are provided, it starts the GUI as normal.
    """
    # We need a QApplication instance to create the MainWindow, even in command-line mode.
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    # We only want to run commands if an argument is provided.
    if len(sys.argv) > 1:
        # --- COMMAND-LINE MODE ---
        command = sys.argv[1].lower()
        print(f"Command received: {command}")

        # Create the main window object to access its functions, but don't show it.
        window = MainWindow()

        try:
            # --- Specify the COM port to use ---
            port_to_use = "COM7"  # <-- CHANGE THIS if your port is different
            # ------------------------------------

            print(f"Attempting to connect to specified port: {port_to_use}...")
            
            # Check if the desired port is actually available
            available_ports = window.serial.list_ports()
            if port_to_use not in available_ports:
                print(f"ERROR: Port {port_to_use} not found.")
                print(f"Available ports are: {available_ports}")
                return # Exit the function
            
            # Connect to the specified port
            window.serial.connect(port_to_use)
            print(f"Successfully connected to {port_to_use}.")

            # Execute the command
            if command == "up":
                window._send_up()
            elif command == "down":
                window._send_down()
            elif command == "zero":
                window._send_zero()
            elif command == "dunk":
                window._send_dunk()
            elif command == "estop":
                window._send_estop()
            elif command == "level":
                window._send_level()
            else:
                print(f"ERROR: Unknown command '{command}'")

            print("Command sent to actuator.")
            # Give the command a moment to process before the script closes.
            time.sleep(0.5)

        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            # Always ensure disconnection
            if window.serial.is_connected:
                window.serial.disconnect()
                print("Disconnected from serial port.")
            print("--- Script Finished ---")

    else:
        # --- GUI MODE ---
        # If no command was given, run the visual application.
        print("No command received, starting GUI...")
        window = MainWindow()
        window.resize(900, 600)
        window.show()
        sys.exit(app.exec_())


if __name__ == '__main__':
    main()
