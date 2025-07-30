from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel

class AboutPage(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("<h2>À propos</h2>"))
        layout.addWidget(QLabel("VIGY - Version 2.0<br>© 2025 Gabin Laussu"))
        layout.addStretch() 