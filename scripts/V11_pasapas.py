import sys
import os
import math
import struct
import subprocess
import can
import cv2
import serial
import time
from datetime import datetime

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QLineEdit, QGroupBox, QGridLayout, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, QThread, Signal, QPoint, QUrl
from PySide6.QtGui import (
    QPainter, QColor, QPolygon, QFont, QImage, QPixmap, QIcon, QDesktopServices
)

# === Constants ===
ELEV_FACTOR = 0.0573
BEAR_FACTOR = 0.0573
NEG_OFFSET = 31415.5

FOV_MAP = {
    0x04: 'VWFoV',
    0x08: 'WFoV',
    0x0C: 'NFoV'
}


class CANReader(QThread):
    update_data = Signal(float, float)
    update_digital_zoom = Signal(int)

    def __init__(self):
        super().__init__()
        self.running = True
        self.bus = can.interface.Bus(channel='can0', interface='socketcan')

    def run(self):
        while self.running:
            msg = self.bus.recv(timeout=0.01)
            if not msg:
                continue
            if msg.arbitration_id == 0x105 and len(msg.data) >= 8:
                elev = math.degrees(struct.unpack('<f', msg.data[0:4])[0])
                bear = math.degrees(struct.unpack('<f', msg.data[4:8])[0])
                if elev >= 180:
                    elev -= 360
                self.update_data.emit(elev, bear)
            elif msg.arbitration_id == 0x101 and len(msg.data) >= 3:
                raw = msg.data[1] | (msg.data[2] << 8)
                self.update_digital_zoom.emit(raw)

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

class DirectionWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.bearing = 0.0
        self.setFixedSize(120, 120)

    def set_bearing(self, b):
        self.bearing = b
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(240,240,240))
        c = QPoint(self.width()//2, self.height()//2)
        sz = 15
        pts = [
            QPoint(c.x(), c.y()-sz),
            QPoint(c.x()+sz//2, c.y()+sz//2),
            QPoint(c.x()-sz//2, c.y()+sz//2)
        ]
        painter.setBrush(Qt.black)
        painter.drawPolygon(QPolygon(pts))
        ex = c.x() + 40 * math.sin(math.radians(self.bearing))
        ey = c.y() - 40 * math.cos(math.radians(self.bearing))
        painter.setPen(QColor(255,0,0))
        painter.drawLine(c, QPoint(int(ex), int(ey)))

class CameraInterface(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VIGY - PySide6 Interface")
        self.setFixedSize(1000, 700)

        self.last_elev = 0.0
        self.last_bear = 0.0
        self.fov_zoom_code = None
        self.digital_zoom_val = 0

        # Anti-tremblement moteur :
        self.last_sent_angle = None
        self.last_sent_time = time.time()

        try:
            self.arduino = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
            time.sleep(2)  # Laisser l'Arduino démarrer
            print("[INFO] Arduino connecté sur /dev/ttyUSB1")
        except Exception as e:
            print(f"[ERREUR] Impossible d’ouvrir le port série : {e}")
            self.arduino = None

        self.bearing_input = QLineEdit("70")
        self.elev_input = QLineEdit("-10")
        self.result_label = QLabel("")
        send_btn = QPushButton("Envoyer")
        send_btn.clicked.connect(self.send_can)
        unlock_btn = QPushButton("Unlock")
        unlock_btn.clicked.connect(self.send_unlock)
        self.ind_label = QLabel("Cap visé : -- °")
        self.ind_label.setFont(QFont("Arial", 12, QFont.Bold))
        self.dir_widget = DirectionWidget()

        send_group = QGroupBox("Envoi CAN (0x20C)")
        sg_layout = QGridLayout(send_group)
        sg_layout.addWidget(QLabel("Bearing (°):"), 0, 0)
        sg_layout.addWidget(self.bearing_input, 0, 1)
        sg_layout.addWidget(QLabel("Elevation (°):"), 1, 0)
        sg_layout.addWidget(self.elev_input, 1, 1)
        sg_layout.addWidget(send_btn, 2, 0, 1, 2)
        sg_layout.addWidget(unlock_btn, 3, 0, 1, 2)
        sg_layout.addWidget(self.result_label, 4, 0, 1, 2)
        sg_layout.addWidget(self.dir_widget, 5, 0)
        sg_layout.addWidget(self.ind_label, 5, 1)

        live_group = QGroupBox("Position actuelle (0x105)")
        lg_layout = QVBoxLayout(live_group)
        self.elev_live = QLabel("Élévation : 0.00 °")
        self.elev_live.setFont(QFont("Arial", 14))
        self.bear_live = QLabel("Bearing : 0.00 °")
        self.bear_live.setFont(QFont("Arial", 14))
        lg_layout.addWidget(self.elev_live)
        lg_layout.addWidget(self.bear_live)

        zoom_group = QGroupBox("Zoom FoV (0x202)")
        zg_layout = QHBoxLayout(zoom_group)
        self.zoom_lbl = QLabel("")
        for code, label in FOV_MAP.items():
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, c=code: self.send_fov_zoom(c))
            zg_layout.addWidget(btn)
        zg_layout.addWidget(self.zoom_lbl)

        video_group = QGroupBox("Flux vidéo caméra")
        vg_layout = QVBoxLayout(video_group)
        self.video_label = QLabel("Flux en cours...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        vid_ctrl = QHBoxLayout()
        self.start_btn = QPushButton("Démarrer Vidéo")
        self.stop_btn = QPushButton("Arrêter Vidéo")
        self.stop_btn.setEnabled(False)
        vid_ctrl.addWidget(self.start_btn)
        vid_ctrl.addWidget(self.stop_btn)

        photo_ctrl = QHBoxLayout()
        self.photo_btn = QPushButton("Prendre Photo")
        self.photo_btn.setIcon(QIcon.fromTheme("camera-photo"))
        self.open_btn = QPushButton("Ouvrir Dossier Photos")
        photo_ctrl.addWidget(self.photo_btn)
        photo_ctrl.addWidget(self.open_btn)

        vg_layout.addWidget(self.video_label)
        vg_layout.addLayout(vid_ctrl)
        vg_layout.addLayout(photo_ctrl)

        main_layout = QHBoxLayout(self)
        left_layout = QVBoxLayout()
        left_layout.addWidget(send_group)
        left_layout.addWidget(live_group)
        left_layout.addWidget(zoom_group)
        main_layout.addLayout(left_layout)
        main_layout.addWidget(video_group)

        self.can_thread = CANReader()
        self.can_thread.update_data.connect(self.on_position)
        self.can_thread.update_digital_zoom.connect(self.on_digital_zoom)
        self.can_thread.start()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.start_btn.clicked.connect(self.start_capture)
        self.stop_btn.clicked.connect(self.stop_capture)
        self.photo_btn.clicked.connect(self.take_photo)
        self.open_btn.clicked.connect(self.open_photos)
        self.send_unlock()


    def closeEvent(self, event):
        # Remettre la caméra à 0°, 0° via trame CAN
        try:
            lsb_e, msb_e = self.deg_to_bytes(0.0, ELEV_FACTOR)
            lsb_b, msb_b = self.deg_to_bytes(360.0, BEAR_FACTOR)  # 360° ≡ 0°
            data = f"0C{lsb_e:02X}{msb_e:02X}{lsb_b:02X}{msb_b:02X}"
            subprocess.call(f"cansend can0 20C#{data}", shell=True)
            print(f"[INFO] Remise à zéro envoyée : {data}")
            
        except Exception as e:
            print(f"[ERREUR] Remise à zéro CAN échouée : {e}")

        # Envoi position 0° au moteur via Arduino
        if self.arduino and self.arduino.is_open:
            try:
                self.arduino.write(b"0\n")
                print("[INFO] Remise à zéro envoyée au moteur (Arduino)")
            except Exception as e:
                print(f"[ERREUR] Envoi moteur échoué : {e}")
            self.arduino.close()

        # Stopper proprement les autres composants
        self.can_thread.stop()
        self.stop_capture()
        self.timer.stop()

        event.accept()



    def deg_to_bytes(self, angle, factor, neg_offset=False):
        raw = int(round(angle / factor + (NEG_OFFSET if neg_offset else 0))) & 0xFFFF
        return raw & 0xFF, raw >> 8

    def send_can(self):
        try:
            b = float(self.bearing_input.text())
            e = float(self.elev_input.text())
            if e < -30 or e >= 70:
                raise ValueError
            lsb_e, msb_e = self.deg_to_bytes(e, ELEV_FACTOR, neg_offset=e < 0)
            lsb_b, msb_b = self.deg_to_bytes(360 - b, BEAR_FACTOR)
            data = f"0C{lsb_e:02X}{msb_e:02X}{lsb_b:02X}{msb_b:02X}"
            subprocess.call(f"cansend can0 20C#{data}", shell=True)
            self.result_label.setText(f"Trame 0x20C envoyée: {data}")
        except:
            self.result_label.setText("Erreur: élévation hors plage")

    def send_unlock(self):
        subprocess.call("cansend can0 205#00", shell=True)
        self.result_label.setText("Trame 0x205#00 envoyée")

    def send_fov_zoom(self, code):
        db = [0x00, code] + [0x00] * 6
        d = ''.join(f"{b:02X}" for b in db)
        subprocess.call(f"cansend can0 202#{d}", shell=True)
        self.fov_zoom_code = code
        self.zoom_lbl.setText(f"FoV: {FOV_MAP.get(code, '')}")

    def on_position(self, elev, bear):
        self.last_elev = elev
        self.last_bear = bear
        self.elev_live.setText(f"Élévation : {elev:.2f} °")
        self.bear_live.setText(f"Bearing : {bear:.2f} °")
        self.ind_label.setText(f"Cap visé : {bear:.2f} °")
        self.dir_widget.set_bearing(bear)

        # --- Pilotage moteur pas-à-pas (bearing absolu 0-359) ---
        if self.arduino and self.arduino.is_open:
            bearing_int = int(round(bear)) % 360  # angle entre 0 et 359
            now = time.time()
            if (
                self.last_sent_angle is None or
                abs(bearing_int - self.last_sent_angle) > 2 or
                (now - self.last_sent_time) > 0.12
            ):
                try:
                    message = f"{bearing_int}\n"
                    self.arduino.write(message.encode())
                    print(f"[DEBUG] Envoi vers moteur pas-à-pas : {bearing_int}")
                    self.last_sent_angle = bearing_int
                    self.last_sent_time = now
                except Exception as e:
                    print(f"[ERREUR] Envoi série moteur : {e}")

    def on_digital_zoom(self, raw):
        self.digital_zoom_val = raw

    def start_capture(self):
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.timer.start(30)
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

    def update_frame(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            ret, frm = self.cap.read()
            if ret:
                rgb = cv2.cvtColor(frm, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                img = QImage(rgb.data, w, h, ch*w, QImage.Format_RGB888)
                self.video_label.setPixmap(
                    QPixmap.fromImage(img).scaled(
                        self.video_label.size(), Qt.KeepAspectRatio
                    )
                )

    def stop_capture(self):
        self.timer.stop()
        if hasattr(self, 'cap'):
            self.cap.release()
            del self.cap
        self.video_label.setText("Flux en cours...")
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

    def take_photo(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            ret, frm = self.cap.read()
            if not ret:
                return
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            fld = os.path.join('photos', f'photo_{ts}')
            os.makedirs(fld, exist_ok=True)
            cv2.imwrite(os.path.join(fld, f'{ts}.jpg'), frm)
            info = os.path.join(fld, 'info.txt')
            fov = FOV_MAP.get(self.fov_zoom_code, 'None')
            raw = self.digital_zoom_val or 0
            with open(info, 'w') as f:
                f.write(f"Timestamp: {ts}\n")
                f.write(f"Élévation: {self.last_elev:.2f} °\n")
                f.write(f"Bearing: {self.last_bear:.2f} °\n")
                f.write(f"FoV: {fov}\n")
                if raw >= 2000:
                    if raw < 3000:
                        dz = 2*(raw-2000)/1000
                    elif raw < 4000:
                        dz = 2+2*(raw-3000)/1000
                    else:
                        dz = 4+4*(raw-4000)/1000
                    f.write(f"Digital Zoom: NFoV + digital {dz:.2f}x ({raw})\n")
            self.result_label.setText(f"Info saved in {fld}")

    def open_photos(self):
        p = os.path.abspath('photos')
        os.makedirs(p, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(p))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = CameraInterface()
    w.show()
    sys.exit(app.exec())
