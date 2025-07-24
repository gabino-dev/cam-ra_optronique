import sys
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl

app = QApplication(sys.argv)
win = QWidget()
win.setWindowTitle("Test OSM WebEngine (Qt5)")
win.setMinimumSize(800, 600)
layout = QVBoxLayout(win)
web = QWebEngineView()
web.setUrl(QUrl("http://localhost:3000"))
layout.addWidget(web)
win.show()
sys.exit(app.exec_())
