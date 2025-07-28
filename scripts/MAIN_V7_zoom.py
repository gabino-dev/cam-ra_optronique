#!/home/gabin/venvs/pyside-env/bin/python
import sys
import math
import struct
import subprocess
import can
import cv2

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QLineEdit, QGroupBox, QGridLayout, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, QThread, Signal, QPoint
from PySide6.QtGui import QPainter, QColor, QPolygon, QFont, QImage, QPixmap

# === Constantes CAN ===
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
        painter.fillRect(self.rect(), QColor(240, 240, 240))
        center = QPoint(self.width() // 2, self.height() // 2)
        ship_size = 15
        points = [
            QPoint(center.x(), center.y() - ship_size),
            QPoint(center.x() + ship_size // 2, center.y() + ship_size // 2),
            QPoint(center.x() - ship_size // 2, center.y() + ship_size // 2)
        ]
        painter.setBrush(Qt.black)
        painter.drawPolygon(QPolygon(points))
        end_x = center.x() + 40 * math.sin(math.radians(self.bearing))
        end_y = center.y() - 40 * math.cos(math.radians(self.bearing))
        painter.setPen(QColor(255, 0, 0))
        painter.drawLine(center, QPoint(int(end_x), int(end_y)))

# === Fenêtre principale avec CAN, vidéo et Zoom ===
class CameraInterface(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VIGY - PySide6 Interface")
        self.setFixedSize(1000, 520)

        # -- Zone CAN envoi --
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

        send_group = QGroupBox("Envoi CAN (0x20C)")
        send_layout = QGridLayout()
        send_layout.addWidget(QLabel("Bearing (°) :"), 0, 0)
        send_layout.addWidget(self.bearing_input, 0, 1)
        send_layout.addWidget(QLabel("Elevation (°) :"), 1, 0)
        send_layout.addWidget(self.elev_input, 1, 1)
        send_layout.addWidget(send_button, 2, 0, 1, 2)
        send_layout.addWidget(unlock_button, 3, 0, 1, 2)
        send_layout.addWidget(self.result_label, 4, 0, 1, 2)
        indicator_layout = QHBoxLayout()
        indicator_layout.addWidget(self.direction_widget)
        indicator_layout.addWidget(self.indicator_label)
        send_layout.addLayout(indicator_layout, 5, 0, 1, 2)
        send_group.setLayout(send_layout)

        # -- Zone CAN live --
        live_group = QGroupBox("Position actuelle (0x105)")
        live_layout = QVBoxLayout()
        self.elevation_live = QLabel("Élévation : -- °")
        self.elevation_live.setFont(QFont("Arial", 14))
        self.bearing_live = QLabel("Bearing : -- °")
        self.bearing_live.setFont(QFont("Arial", 14))
        live_layout.addWidget(self.elevation_live)
        live_layout.addWidget(self.bearing_live)
        live_group.setLayout(live_layout)

        # -- Zone Zoom FoV --
        zoom_group = QGroupBox("Zoom FoV (0x202)")
        zoom_layout = QHBoxLayout()
        self.zoom_result = QLabel("")
        btn_vw = QPushButton("VWFoV")
        btn_w = QPushButton("WFoV")
        btn_n = QPushButton("NFoV")
        btn_vw.clicked.connect(lambda: self.send_zoom(0x04))
        btn_w.clicked.connect(lambda: self.send_zoom(0x08))
        btn_n.clicked.connect(lambda: self.send_zoom(0x0C))
        zoom_layout.addWidget(btn_vw)
        zoom_layout.addWidget(btn_w)
        zoom_layout.addWidget(btn_n)
        zoom_layout.addWidget(self.zoom_result)
        zoom_group.setLayout(zoom_layout)

        # -- Zone vidéo --
        video_group = QGroupBox("Flux vidéo caméra")
        v_layout = QVBoxLayout()
        self.video_label = QLabel("Flux en cours...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("Démarrer Vidéo")
        self.stop_btn = QPushButton("Arrêter Vidéo")
        self.stop_btn.setEnabled(False)
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        v_layout.addWidget(self.video_label)
        v_layout.addLayout(btn_layout)
        video_group.setLayout(v_layout)

        self.cap = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.start_btn.clicked.connect(self.start_capture)
        self.stop_btn.clicked.connect(self.stop_capture)

        # -- Layout principal --
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        left_layout.addWidget(send_group)
        left_layout.addWidget(live_group)
        left_layout.addWidget(zoom_group)
        main_layout.addLayout(left_layout)
        main_layout.addWidget(video_group)
        self.setLayout(main_layout)

        # -- Thread CAN démarrage --
        self.can_thread = CANReader()
        self.can_thread.update_data.connect(self.update_live_position)
        self.can_thread.start()

    def closeEvent(self, event):
        self.can_thread.stop()
        self.stop_capture()
        event.accept()

    # --- Utilities CAN ---
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
            subprocess.call(f"cansend can0 20C#{data_str}", shell=True)
            self.result_label.setText(f"Trame envoyée : {data_str}")
        except Exception as e:
            self.result_label.setText(f"Erreur : {e}")

    def send_unlock(self):
        try:
            subprocess.call("cansend can0 205#00", shell=True)
            self.result_label.setText("Trame UNLOCK envoyée (205#00)")
        except Exception as e:
            self.result_label.setText(f"Erreur UNLOCK : {e}")

    def send_zoom(self, code):
        try:
            # Data 8 bytes: 00, code, then 6 zeros
            data_bytes = [0x00, code] + [0x00] * 6
            data_str = ''.join(f"{b:02X}" for b in data_bytes)
            subprocess.call(f"cansend can0 202#{data_str}", shell=True)
            self.zoom_result.setText(f"Trame envoyée : 202#{data_str}")
        except Exception as e:
            self.zoom_result.setText(f"Erreur Zoom : {e}")

    def update_live_position(self, elev_deg, bear_deg):
        self.elevation_live.setText(f"Élévation : {elev_deg:.2f} °")
        self.bearing_live.setText(f"Bearing : {bear_deg:.2f} °")
        self.indicator_label.setText(f"Cap visé : {bear_deg:.2f} °")
        self.direction_widget.set_bearing(bear_deg)

    # --- Vidéo ---
    def start_capture(self):
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.timer.start(30)
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

    def update_frame(self):
        if self.cap:
            ret, frame = self.cap.read()
            if ret:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                bytes_per_line = ch * w
                image = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.video_label.setPixmap(QPixmap.fromImage(image).scaled(
                    self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
                ))

    def stop_capture(self):
        self.timer.stop()
        if self.cap:
            self.cap.release()
            self.cap = None
        self.video_label.clear()
        self.video_label.setText("Flux en cours...")
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CameraInterface()
    window.show()
    sys.exit(app.exec())
