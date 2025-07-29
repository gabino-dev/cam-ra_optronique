import sys
from PySide6.QtWidgets import QApplication
from ui.mainwindow import MainWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)
    with open("style.qss") as f:
        app.setStyleSheet(f.read())
    win = MainWindow()
    win.show()
    sys.exit(app.exec()) 