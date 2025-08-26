#
# This is the final, stable version of run.py
#
import sys
import time

# We only import GUI components when we know we need them.
from PyQt5.QtWidgets import QApplication
from app.main_window import MainWindow

# --- A new, simple function for direct serial communication ---
def send_serial_command(port, command):
    """
    This function connects directly to the serial port, sends a single command,
    waits for a single response, and closes. It uses no GUI components.
    """
    import serial # Import pyserial locally so the GUI doesn't need it.
    try:
        print(f"Opening port {port}...")
        # Use a 'with' statement to ensure the port always closes.
        with serial.Serial(port, 9600, timeout=2) as ser:
            time.sleep(1.5) # Wait a bit longer for the connection to fully establish.
            
            # Read the Arduino's welcome message to confirm it's ready.
            response = ser.readline().decode('utf-8', 'ignore').strip()
            if not response:
                print("ERROR: No response from Arduino on connection. Check wiring and Arduino code.")
                return
            print(f"Arduino ready. Banner: {response}")

            # Send the command
            full_command = command + '\n'
            print(f"Sending command: {command}")
            ser.write(full_command.encode())

            # Wait for the confirmation message (e.g., "MOVED")
            response = ser.readline().decode('utf-8', 'ignore').strip()
            print(f"Arduino response: {response}")
        
        print(f"Port {port} closed.")

    except serial.SerialException as e:
        print(f"SERIAL ERROR: {e}. Is another program (like the GUI or Arduino IDE) using the port?")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


def main():
    # Check if we are running in command-line mode.
    if len(sys.argv) > 1:
        # --- COMMAND-LINE MODE ---
        # We need a temporary MainWindow instance ONLY to get calculation values.
        # This is done without showing any windows.
        app = QApplication.instance() or QApplication(sys.argv)
        temp_window = MainWindow()

        command = sys.argv[1].lower()
        port_to_use = "COM7" # Make sure this is your correct port
        final_command_to_send = ""

        # --- Calculate the command string ---
        if command in ["up", "down"]:
            if len(sys.argv) > 2:
                try:
                    distance_mm = float(sys.argv[2])
                    degs_to_move = (distance_mm / temp_window._lead_mm) * 360.0
                    if command == "down":
                        degs_to_move = -degs_to_move
                    final_command_to_send = f"MOVE {degs_to_move:.2f}"
                except ValueError:
                    print(f"ERROR: Invalid distance value '{sys.argv[2]}'.")
                    return
            else:
                print(f"ERROR: The '{command}' command requires a distance value.")
                return
        elif command in ["zero", "dunk", "estop", "level"]:
            final_command_to_send = command.upper()
        else:
            print(f"ERROR: Unknown command '{command}'")
            return

        # --- Send the final command using our simple, direct function ---
        if final_command_to_send:
            send_serial_command(port_to_use, final_command_to_send)
        
        print("--- Script Finished ---")

    else:
        # --- GUI MODE ---
        app = QApplication(sys.argv)
        window = MainWindow()
        window.resize(900, 600)
        window.show()
        sys.exit(app.exec_())

if __name__ == '__main__':
    main()