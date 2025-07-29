from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QLineEdit, QFormLayout, QPushButton

class SettingsPage(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("<h2>Réglages</h2>"))
        form = QFormLayout()
        self.port_line = QLineEdit("/dev/pts/3")
        self.can_line = QLineEdit("can0")
        self.arduino_line = QLineEdit("/dev/ttyUSB0")
        form.addRow("Port NMEA:", self.port_line)
        form.addRow("CAN Interface:", self.can_line)
        form.addRow("Port Arduino:", self.arduino_line)
        layout.addLayout(form)
        self.save_btn = QPushButton("Enregistrer")
        layout.addWidget(self.save_btn)
        layout.addStretch() 