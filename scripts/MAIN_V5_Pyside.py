#!/home/gabin/venvs/pyside-env/bin/python
import sys
import math
import struct
import subprocess
import can

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QLineEdit, QGroupBox, QGridLayout, QFrame, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, QThread, Signal, QPoint
from PySide6.QtGui import QPainter, QColor, QPolygon, QFont

# === Constantes ===
ELEV_FACTOR = 0.0573
BEAR_FACTOR = 0.0573
NEG_OFFSET  = 31415.5

# === Thread de lecture CAN ===
class CANReader(QThread):
    update_data = Signal(float, float)

    def __init__(self):
        super().__init__()
        self.running = True
        self.bus = can.interface.Bus(channel='can0', interface='socketcan')

    def run(self):
        while self.running:
            msg = self.bus.recv(timeout=0.01)
            if msg and msg.arbitration_id == 0x105 and len(msg.data) >= 8:
                elev_rad = struct.unpack('<f', msg.data[0:4])[0]
                bear_rad = struct.unpack('<f', msg.data[4:8])[0]
                elev_deg = math.degrees(elev_rad)
                bear_deg = math.degrees(bear_rad)
                if elev_deg >= 180:
                    elev_deg -= 360
                self.update_data.emit(elev_deg, bear_deg)

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

# === Widget de direction (avec un bateau et trait rouge) ===
class DirectionWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.bearing = 0.0
        self.setFixedSize(120, 120)

    def set_bearing(self, bearing):
        self.bearing = bearing
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Fond
        painter.fillRect(self.rect(), QColor(240, 240, 240))

        center = QPoint(self.width() // 2, self.height() // 2)

        # Bateau fixe vers le haut (triangle noir)
        ship_size = 15
        points = [
            QPoint(center.x(), center.y() - ship_size),
            QPoint(center.x() + ship_size // 2, center.y() + ship_size // 2),
            QPoint(center.x() - ship_size // 2, center.y() + ship_size // 2)
        ]
        painter.setBrush(Qt.black)
        painter.drawPolygon(QPolygon(points))

        # Trait rouge direction (dynamique selon bearing)
        end_x = center.x() + 40 * math.sin(math.radians(self.bearing))
        end_y = center.y() - 40 * math.cos(math.radians(self.bearing))
        painter.setPen(QColor(255, 0, 0))
        painter.drawLine(center, QPoint(int(end_x), int(end_y)))

# === Fenêtre principale ===
class CameraInterface(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VIGY - PySide6 Interface")
        self.setFixedSize(850, 420)

        # === Zone Envoi CAN ===
        self.bearing_input = QLineEdit("70")
        self.elev_input = QLineEdit("-10")
        self.result_label = QLabel("")

        send_button = QPushButton("Envoyer")
        send_button.clicked.connect(self.send_can)

        unlock_button = QPushButton("Unlock")
        unlock_button.clicked.connect(self.send_unlock)

        self.indicator_label = QLabel("Cap visé : -- °")
        self.indicator_label.setFont(QFont("Arial", 12, QFont.Bold))

        self.direction_widget = DirectionWidget()

        indicator_layout = QHBoxLayout()
        indicator_layout.addWidget(self.direction_widget)
        indicator_layout.addWidget(self.indicator_label)
        indicator_layout.setAlignment(Qt.AlignLeft)

        send_group = QGroupBox("Envoi CAN (0x20C)")
        send_layout = QGridLayout()
        send_layout.addWidget(QLabel("Bearing (°) :"), 0, 0)
        send_layout.addWidget(self.bearing_input, 0, 1)
        send_layout.addWidget(QLabel("Elevation (°) :"), 1, 0)
        send_layout.addWidget(self.elev_input, 1, 1)
        send_layout.addWidget(send_button, 2, 0, 1, 2)
        send_layout.addWidget(unlock_button, 3, 0, 1, 2)
        send_layout.addWidget(self.result_label, 4, 0, 1, 2)
        send_layout.addLayout(indicator_layout, 5, 0, 1, 2)
        send_group.setLayout(send_layout)

        # === Zone Position Live ===
        self.elevation_live = QLabel("Élévation : -- °")
        self.elevation_live.setFont(QFont("Arial", 14))
        self.bearing_live = QLabel("Bearing : -- °")
        self.bearing_live.setFont(QFont("Arial", 14))

        live_group = QGroupBox("Position actuelle (0x105)")
        live_layout = QVBoxLayout()
        live_layout.addWidget(self.elevation_live)
        live_layout.addWidget(self.bearing_live)
        live_group.setLayout(live_layout)

        # === Layout principal ===
        main_layout = QHBoxLayout()
        main_layout.addWidget(send_group)
        main_layout.addWidget(live_group)
        self.setLayout(main_layout)

        # === Thread CAN ===
        self.can_thread = CANReader()
        self.can_thread.update_data.connect(self.update_live_position)
        self.can_thread.start()

    def closeEvent(self, event):
        self.can_thread.stop()
        event.accept()

    def deg_to_elev_bytes(self, angle_deg):
        if angle_deg >= 0:
            raw = int(round(angle_deg / ELEV_FACTOR))
        else:
            raw = int(round(angle_deg / ELEV_FACTOR + NEG_OFFSET))
        raw &= 0xFFFF
        return raw & 0xFF, (raw >> 8) & 0xFF

    def deg_to_bear_bytes(self, angle_deg):
        raw = int(round((360 - angle_deg) / BEAR_FACTOR)) & 0xFFFF
        return raw & 0xFF, (raw >> 8) & 0xFF

    def send_can(self):
        try:
            bearing = float(self.bearing_input.text())
            elev = float(self.elev_input.text())
            if elev < -30 or elev >= 70:
                self.result_label.setText("Erreur : élévation hors plage [-30°, 70°).")
                return
            byte0 = 0x0C
            elev_lsb, elev_msb = self.deg_to_elev_bytes(elev)
            bear_lsb, bear_msb = self.deg_to_bear_bytes(bearing)
            data_str = f"{byte0:02X}{elev_lsb:02X}{elev_msb:02X}{bear_lsb:02X}{bear_msb:02X}"
            cmd = f"cansend can0 20C#{data_str}"
            subprocess.call(cmd, shell=True)
            self.result_label.setText(f"Trame envoyée : {data_str}")
        except Exception as e:
            self.result_label.setText(f"Erreur : {e}")

    def send_unlock(self):
        try:
            cmd = "cansend can0 205#00"
            subprocess.call(cmd, shell=True)
            self.result_label.setText("Trame UNLOCK envoyée (205#00)")
        except Exception as e:
            self.result_label.setText(f"Erreur UNLOCK : {e}")

    def update_live_position(self, elev_deg, bear_deg):
        self.elevation_live.setText(f"Élévation : {elev_deg:.2f} °")
        self.bearing_live.setText(f"Bearing : {bear_deg:.2f} °")
        self.indicator_label.setText(f"Cap visé : {bear_deg:.2f} °")
        self.direction_widget.set_bearing(bear_deg)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CameraInterface()
    window.show()
    sys.exit(app.exec())
