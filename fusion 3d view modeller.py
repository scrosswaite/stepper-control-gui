import sys, serial
import pyvista as pv
from PyQt5 import QtWidgets
from pyvistaqt import QtInteractor

class FusionViewer(QtWidgets.QMainWindow):
    def __init__(self, filename, port="COM7", baud=115200):
        super().__init__()
        self.setWindowTitle("Live IMU 3D Model Viewer")

        # Serial connection
        self.ser = serial.Serial(port, baud, timeout=1)

        # PyVista plotter widget
        self.plotter = QtInteractor(self)
        self.setCentralWidget(self.plotter)

        # Load STL/OBJ model
        self.mesh = pv.read(filename)
        self.actor = self.plotter.add_mesh(self.mesh, color="lightgreen", show_edges=True)

        self.plotter.show_axes()
        self.plotter.reset_camera()

        # Timer to update orientation
        self.timer = self.startTimer(5)  # every 50ms

    def timerEvent(self, event):
        if self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()
            try:
                roll, pitch, yaw = map(float, line.split(","))
                # Apply orientation (experiment with order/signs to match your setup)
                self.actor.SetOrientation(pitch, roll, yaw)
                self.plotter.render()
            except:
                pass

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    viewer = FusionViewer("model.stl", port="COM7")  # replace model.stl + COM port
    viewer.show()
    sys.exit(app.exec_())
