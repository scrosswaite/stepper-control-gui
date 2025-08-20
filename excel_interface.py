import sys
from app.main_window import MainWindow
from PyQt5.QtWidgets import QApplication

def main():
    # Create a QApplication instance
    app = QApplication(sys.argv)

    # Create the main window
    window = MainWindow()

    # Check for command-line arguments
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()

        # You might need to establish a connection first
        # This is a simplified example. You may need to adapt this.
        # a better approach would be to have connect/disconnect commands.
        if not window.serial.is_connected:
            print("Not connected to a serial port.")
            # You could attempt to auto-connect here if you have a default port
            # For now, we'll exit if not connected.
            return


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
            print(f"Unknown command: {command}")

        # This is a simple way to allow the command to be sent
        # and then exit. You might need a more robust solution
        # depending on the asynchronous nature of your app.
        # For now, we'll just exit.
        sys.exit(0)

    else:
        # If no arguments are provided, run the GUI as normal
        window.resize(900, 600)
        window.show()
        sys.exit(app.exec_())


if __name__ == '__main__':
    main()