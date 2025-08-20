import sys
from PyQt5.QtWidgets import QApplication
from app.main_window import MainWindow
from excel_interface import main

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = MainWindow()
    window.resize(900, 600)
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
