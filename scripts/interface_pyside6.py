import sys
import math
import struct
import subprocess
import threading
import can

from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QLineEdit,
    QGroupBox, QTextEdit, QGridLayout
)
from PySide6.QtCore import QTimer, Qt, QPoint
from PySide6.QtGui import QPainter, QColor, QPen, QPixmap

ELEV_FACTOR = 0.0573
BEAR_FACTOR = 0.0573
NEG_OFFSET = 31415.5

class DirectionWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.bearing = 0
        self.setFixedSize(150, 150)

    def set_bearing(self, deg):
        self.bearing = deg
        self.update()

    def paintEvent(self, event):
        center = self.rect().center()
        radius = min(self.width(), self.height()) // 2 - 10

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw ship icon (triangle)
        ship_size = 20
        painter.setBrush(QColor("blue"))
        painter.setPen(Qt.NoPen)
        points = [
            center + QPoint(0, -ship_size),
            center + QPoint(ship_size // 2, ship_size // 2),
            center + QPoint(-ship_size // 2, ship_size // 2)
        ]
        painter.drawPolygon(points)

        # Draw direction line
        angle_rad = math.radians(-self.bearing + 90)
        x = center.x() + radius * math.cos(angle_rad)
        y = center.y() - radius * math.sin(angle_rad)
        pen = QPen(QColor("red"), 2)
        painter.setPen(pen)
        painter.drawLine(center.x(), center.y(), int(x), int(y))

class CameraControl(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VIGY - PySide6 Control")

        self.layout = QHBoxLayout(self)

        # === CAN SEND GROUP ===
        send_group = QGroupBox("Envoi CAN (0x20C)")
        send_layout = QVBoxLayout()

        self.bearing_input = QLineEdit("70")
        self.elev_input = QLineEdit("-10")
        self.result_label = QLabel("")
        self.indicator_label = QLabel("Cap visé : --°")

        send_layout.addWidget(QLabel("Bearing (°):"))
        send_layout.addWidget(self.bearing_input)
        send_layout.addWidget(QLabel("Élévation (°):"))
        send_layout.addWidget(self.elev_input)

        send_btn = QPushButton("Envoyer")
        send_btn.clicked.connect(self.send_command)
        send_layout.addWidget(send_btn)

        unlock_btn = QPushButton("Unlock")
        unlock_btn.clicked.connect(self.send_unlock)
        send_layout.addWidget(unlock_btn)

        send_layout.addWidget(self.result_label)
        send_layout.addWidget(self.indicator_label)
        send_layout.addWidget(QLabel("Direction visée:"))
        self.direction_widget = DirectionWidget()
        send_layout.addWidget(self.direction_widget)

        send_group.setLayout(send_layout)
        self.layout.addWidget(send_group)

        # === FOV GROUP ===
        fov_group = QGroupBox("Calculateur FOV")
        fov_layout = QVBoxLayout()
        self.distance_input = QLineEdit()
        self.fov_result = QLabel("")

        calc_btn = QPushButton("Calculer")
        calc_btn.clicked.connect(self.compute_fov)

        fov_layout.addWidget(QLabel("Distance à la cible (m):"))
        fov_layout.addWidget(self.distance_input)
        fov_layout.addWidget(calc_btn)
        fov_layout.addWidget(self.fov_result)

        fov_group.setLayout(fov_layout)
        self.layout.addWidget(fov_group)

        # === POSITION LIVE ===
        live_group = QGroupBox("Position actuelle (0x105)")
        live_layout = QVBoxLayout()
        self.elevation_live = QLabel("Élévation : -- °")
        self.bearing_live = QLabel("Bearing : -- °")
        live_layout.addWidget(self.elevation_live)
        live_layout.addWidget(self.bearing_live)
        live_group.setLayout(live_layout)
        self.layout.addWidget(live_group)

        # CAN
        self.bus = can.interface.Bus(channel='can0', interface='socketcan')
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_can)
        self.timer.start(40)

    def send_command(self):
        try:
            bearing = float(self.bearing_input.text())
            elev = float(self.elev_input.text())
            if elev < -30 or elev >= 70:
                self.result_label.setText("Erreur : élévation hors plage [-30°, 70°).")
                return

            if elev >= 0:
                raw_elev = int(round(elev / ELEV_FACTOR))
            else:
                raw_elev = int(round(elev / ELEV_FACTOR + NEG_OFFSET))
            raw_elev &= 0xFFFF

            raw_bear = int(round((360 - bearing) / BEAR_FACTOR)) & 0xFFFF

            data = bytearray([
                0x0C,
                raw_elev & 0xFF, (raw_elev >> 8) & 0xFF,
                raw_bear & 0xFF, (raw_bear >> 8) & 0xFF
            ])
            cmd = f"cansend can0 20C#{data.hex().upper()}"
            subprocess.call(cmd, shell=True)
            self.result_label.setText(f"Trame envoyée: {data.hex().upper()}")
        except Exception as e:
            self.result_label.setText(f"Erreur: {e}")

    def send_unlock(self):
        try:
            cmd = "cansend can0 205#00"
            subprocess.call(cmd, shell=True)
            self.result_label.setText("Trame UNLOCK envoyée (205#00)")
        except Exception as e:
            self.result_label.setText(f"Erreur UNLOCK: {e}")

    def compute_fov(self):
        try:
            distance = float(self.distance_input.text())
            field = 30 * 1.2
            fov_rad = 2 * math.atan(field / (2 * distance))
            fov_deg = math.degrees(fov_rad)

            fov_modes = [
                (5000, "NFOV 8x", 2.4, 8),
                (4000, "NFOV 4x", 2.4, 4),
                (3000, "NFOV 2x", 2.4, 2),
                (2000, "NFOV",    2.4, 1),
                (1000, "WFOV",    12,  1),
                (0,    "VWFOV",   41,  1)
            ]

            for value, name, fov, zoom in fov_modes:
                effective_fov = fov / zoom
                if fov_deg <= effective_fov:
                    self.fov_result.setText(f"Mode: {name}, Value: {value}, FOV: {effective_fov:.2f}°")
                    return
            self.fov_result.setText("Cible trop grande, utilisez VWFOV")
        except:
            self.fov_result.setText("Entrée invalide.")

    def read_can(self):
        try:
            msg = self.bus.recv(timeout=0.01)
            if msg and msg.arbitration_id == 0x105 and len(msg.data) >= 8:
                elev_rad = struct.unpack('<f', msg.data[0:4])[0]
                bear_rad = struct.unpack('<f', msg.data[4:8])[0]
                elev_deg = math.degrees(elev_rad)
                bear_deg = math.degrees(bear_rad)

                if elev_deg >= 180:
                    elev_deg -= 360

                self.elevation_live.setText(f"Élévation : {elev_deg:.2f} °")
                self.bearing_live.setText(f"Bearing : {bear_deg:.2f} °")
                self.indicator_label.setText(f"Cap visé : {bear_deg:.2f} °")
                self.direction_widget.set_bearing(bear_deg)
        except:
            pass

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CameraControl()
    window.show()
    sys.exit(app.exec())
