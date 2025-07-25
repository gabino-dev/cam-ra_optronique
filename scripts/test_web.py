import os
# éviter de charger les libs Snap qui causent le symbol lookup error
os.environ.pop("LD_LIBRARY_PATH", None)
# forcer X11 même si la session est en Wayland
os.environ["QT_QPA_PLATFORM"] = "xcb"

from PyQt6.QtWidgets import QApplication, QVBoxLayout, QWidget
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl
import sys

class FreeboardWindow(QWidget):
    def __init__(self, signalk_port=3000):
        super().__init__()
        self.setWindowTitle("Signal K Freeboard (Qt6)")
        self.resize(1200, 800)
        layout = QVBoxLayout(self)
        web = QWebEngineView(self)
        web.setUrl(QUrl(f"http://localhost:{signalk_port}/@signalk/freeboard-sk"))
        layout.addWidget(web)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = FreeboardWindow(3000)
    win.show()
    sys.exit(app.exec())
