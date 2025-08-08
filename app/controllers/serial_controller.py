import serial
import serial.tools.list_ports
from PyQt5.QtCore import QObject, QTimer

class SerialController(QObject):
    def __init__(self, on_data, on_error, on_disconnect):
        super().__init__()
        self.on_data = on_data
        self.on_error = on_error
        self.on_disconnect = on_disconnect
        self._serial = None
        self._timer  = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._read_loop)

    @property
    def is_connected(self):
        return self._serial and self._serial.is_open

    def list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port: str, baud=9600, timeout=5):
        ser = serial.Serial(port, baud, timeout=timeout)
        banner = ser.readline().decode('utf-8', 'ignore').strip()
        if not banner:
            ser.close()
            raise ConnectionError("No response from Arduino")
        self._serial = ser
        self._timer.start()

    def disconnect(self):
        self._timer.stop()
        if self._serial:
            self._serial.close()
            self._serial = None
        self.on_disconnect()

    def send(self, cmd: str):
        if self.is_connected:
            self._serial.write(cmd.encode())

    def _read_loop(self):
        try:
            while self._serial and self._serial.in_waiting:
                line = self._serial.readline().decode('utf-8', 'ignore').strip()
                self.on_data(line)
        except Exception as e:
            self.on_error(e)