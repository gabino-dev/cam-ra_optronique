#!/home/gabin/venvs/pyside-env/bin/python
import sys
import os
import math
import struct
import subprocess
import can
import cv2
from datetime import datetime

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QLineEdit, QGroupBox, QGridLayout, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, QThread, Signal, QPoint, QUrl
from PySide6.QtGui import QPainter, QColor, QPolygon, QFont, QImage, QPixmap, QIcon, QDesktopServices

# === Constantes CAN ===
ELEV_FACTOR = 0.0573
BEAR_FACTOR = 0.0573
NEG_OFFSET = 31415.5

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

# === Widget de direction ===
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

# === Interface principale ===
class CameraInterface(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VIGY - PySide6 Interface")
        self.setFixedSize(1000, 650)

        # Variables de contexte pour photo
        self.last_elev = None
        self.last_bear = None
        self.current_zoom = None

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
        send_layout = QGridLayout(send_group)
        send_layout.addWidget(QLabel("Bearing (°) :"), 0, 0)
        send_layout.addWidget(self.bearing_input, 0, 1)
        send_layout.addWidget(QLabel("Elevation (°) :"), 1, 0)
        send_layout.addWidget(self.elev_input, 1, 1)
        send_layout.addWidget(send_button, 2, 0, 1, 2)
        send_layout.addWidget(unlock_button, 3, 0, 1, 2)
        send_layout.addWidget(self.result_label, 4, 0, 1, 2)
        ind_layout = QHBoxLayout()
        ind_layout.addWidget(self.direction_widget)
        ind_layout.addWidget(self.indicator_label)
        send_layout.addLayout(ind_layout, 5, 0, 1, 2)

        # -- Zone CAN live --
        live_group = QGroupBox("Position actuelle (0x105)")
        live_layout = QVBoxLayout(live_group)
        self.elevation_live = QLabel("Élévation : -- °")
        self.elevation_live.setFont(QFont("Arial", 14))
        self.bearing_live = QLabel("Bearing : -- °")
        self.bearing_live.setFont(QFont("Arial", 14))
        live_layout.addWidget(self.elevation_live)
        live_layout.addWidget(self.bearing_live)

        # -- Zone Zoom FoV --
        zoom_group = QGroupBox("Zoom FoV (0x202)")
        zoom_layout = QHBoxLayout(zoom_group)
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

        # -- Zone vidéo et photo --
        video_group = QGroupBox("Flux vidéo caméra")
        v_layout = QVBoxLayout(video_group)
        self.video_label = QLabel("Flux en cours...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Boutons contrôle vidéo
        vid_btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("Démarrer Vidéo")
        self.stop_btn = QPushButton("Arrêter Vidéo")
        self.stop_btn.setEnabled(False)
        vid_btn_layout.addWidget(self.start_btn)
        vid_btn_layout.addWidget(self.stop_btn)

        # Boutons photo (photo + dossier)
        photo_btn_layout = QHBoxLayout()
        self.photo_btn = QPushButton("Prendre Photo")
        self.photo_btn.setIcon(QIcon.fromTheme("camera-photo"))
        self.open_folder_btn = QPushButton("Ouvrir Dossier Photos")
        photo_btn_layout.addWidget(self.photo_btn)
        photo_btn_layout.addWidget(self.open_folder_btn)

        v_layout.addWidget(self.video_label)
        v_layout.addLayout(vid_btn_layout)
        v_layout.addLayout(photo_btn_layout)

        # Connexions des signaux
        self.cap = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.start_btn.clicked.connect(self.start_capture)
        self.stop_btn.clicked.connect(self.stop_capture)
        self.photo_btn.clicked.connect(self.take_photo)
        self.open_folder_btn.clicked.connect(self.open_photo_folder)

        # -- Layout principal --
        main_layout = QHBoxLayout(self)
        left_col = QVBoxLayout()
        left_col.addWidget(send_group)
        left_col.addWidget(live_group)
        left_col.addWidget(zoom_group)
        main_layout.addLayout(left_col)
        main_layout.addWidget(video_group)

        # -- Thread CAN --
        self.can_thread = CANReader()
        self.can_thread.update_data.connect(self.update_live_position)
        self.can_thread.start()

    def closeEvent(self, event):
        self.can_thread.stop()
        self.stop_capture()
        event.accept()

    # --- CAN utils ---
    def deg_to_elev_bytes(self, angle_deg):
        if angle_deg >= 0:
            raw = int(round(angle_deg / ELEV_FACTOR))
        else:
            raw = int(round(angle_deg / ELEV_FACTOR + NEG_OFFSET))
        raw &= 0xFFFF
        return raw & 0xFF, raw >> 8

    def deg_to_bear_bytes(self, angle_deg):
        raw = int(round((360 - angle_deg) / BEAR_FACTOR)) & 0xFFFF
        return raw & 0xFF, raw >> 8

    def send_can(self):
        try:
            bearing = float(self.bearing_input.text())
            elev = float(self.elev_input.text())
            if elev < -30 or elev >= 70:
                self.result_label.setText("Erreur : élévation hors plage [-30°,70°)")
                return
            byte0 = 0x0C
            lsb_e, msb_e = self.deg_to_elev_bytes(elev)
            lsb_b, msb_b = self.deg_to_bear_bytes(bearing)
            data = f"{byte0:02X}{lsb_e:02X}{msb_e:02X}{lsb_b:02X}{msb_b:02X}"
            subprocess.call(f"cansend can0 20C#{data}", shell=True)
            self.result_label.setText(f"Trame 0x20C envoyée: {data}")
        except Exception as e:
            self.result_label.setText(f"Erreur CAN: {e}")

    def send_unlock(self):
        try:
            subprocess.call("cansend can0 205#00", shell=True)
            self.result_label.setText("Trame UNLOCK (0x205#00) envoyée")
        except Exception as e:
            self.result_label.setText(f"Erreur Unlock: {e}")

    def send_zoom(self, code):
        try:
            bytes_list = [0x00, code] + [0x00]*6
            data = ''.join(f"{b:02X}" for b in bytes_list)
            subprocess.call(f"cansend can0 202#{data}", shell=True)
            self.current_zoom = code
            zoom_names = {0x04: "VWFoV", 0x08: "WFoV", 0x0C: "NFoV"}
            name = zoom_names.get(code, "Unknown")
            self.zoom_result.setText(f"Zoom: {name} (0x{code:02X})")
        except Exception as e:
            self.zoom_result.setText(f"Erreur Zoom: {e}")

    def update_live_position(self, elev_deg, bear_deg):
        self.last_elev = elev_deg
        self.last_bear = bear_deg
        self.elevation_live.setText(f"Élévation : {elev_deg:.2f} °")
        self.bearing_live.setText(f"Bearing : {bear_deg:.2f} °")
        self.indicator_label.setText(f"Cap visé : {bear_deg:.2f} °")
        self.direction_widget.set_bearing(bear_deg)

    # --- Vidéo et photo ---
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
                img = QImage(rgb.data, w, h, ch*w, QImage.Format_RGB888)
                pix = QPixmap.fromImage(img).scaled(
                    self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.video_label.setPixmap(pix)

    def stop_capture(self):
        self.timer.stop()
        if self.cap:
            self.cap.release()
            self.cap = None
        self.video_label.setText("Flux en cours...")
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

    def take_photo(self):
        if not self.cap:
            return
        ret, frame = self.cap.read()
        if ret:
            # Crée un dossier spécifique pour cette photo
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            folder = os.path.join('photos', f'photo_{timestamp}')
            os.makedirs(folder, exist_ok=True)
            # Enregistre l'image
            img_path = os.path.join(folder, f'photo_{timestamp}.jpg')
            cv2.imwrite(img_path, frame)
            # Écrit les infos tactiques
            info_path = os.path.join(folder, 'info.txt')
            zoom_names = {0x04: 'VWFoV', 0x08: 'WFoV', 0x0C: 'NFoV'}
            zoom_name = zoom_names.get(self.current_zoom, 'Unknown') if self.current_zoom is not None else 'None'
            with open(info_path, 'w') as f:
                f.write(f"Timestamp: {timestamp}\n")
                f.write(f"Élévation: {self.last_elev:.2f} °\n")
                f.write(f"Bearing: {self.last_bear:.2f} °\n")
                f.write(f"Zoom: {zoom_name} (0x{self.current_zoom:02X})\n")
            self.result_label.setText(f"Photo et infos enregistrées dans {folder}")

    def open_photo_folder(self):
        path = os.path.abspath('photos')
        os.makedirs(path, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(path))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CameraInterface()
    window.show()
    sys.exit(app.exec())
