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
    QPainter, QColor, QPolygon, QFont, QImage, QPixmap, QIcon, QDesktopServices, QPen
)
from PySide6.QtWebEngineWidgets import QWebEngineView

import pynmea2

ELEV_FACTOR = 0.0573
BEAR_FACTOR = 0.0573
NEG_OFFSET = 31415.5

FOV_MAP = {
    0x04: 'VWFoV',
    0x08: 'WFoV',
    0x0C: 'NFoV'
}

class NMEAReader(QThread):
    nmea_update = Signal(str, str, float)
    def __init__(self, port='/dev/pts/2', baud=4800):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = True
    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
        except Exception as e:
            print(f"[NMEA] Erreur ouverture port : {e}")
            return
        while self.running:
            try:
                line = ser.readline().decode(errors='ignore').strip()
                if not line.startswith('$'):
                    continue
                try:
                    msg = pynmea2.parse(line)
                    if isinstance(msg, pynmea2.RMC):
                        lat = f"{msg.latitude:.5f}° {msg.lat_dir}"
                        lon = f"{msg.longitude:.5f}° {msg.lon_dir}"
                        heading = float(msg.true_course) if msg.true_course else 0.0
                        self.nmea_update.emit(lat, lon, heading)
                except pynmea2.ParseError:
                    continue
            except Exception as e:
                print("[NMEA] Erreur lecture :", e)
                continue
    def stop(self):
        self.running = False
        self.wait()

class CANReader(QThread):
    update_data = Signal(float, float)
    update_digital_zoom = Signal(int)
    update_lrf = Signal(int)
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
            elif msg.arbitration_id == 0x107 and len(msg.data) >= 2:
                dist_m = msg.data[0] | (msg.data[1] << 8)
                self.update_lrf.emit(dist_m)
    def stop(self):
        self.running = False
        self.quit()
        self.wait()

class CompassWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.heading = 0.0
        self.camera_bearing = 0.0
        self.setFixedSize(160, 160)
    def set_heading(self, heading):
        self.heading = heading
        self.update()
    def set_camera_bearing(self, bearing):
        self.camera_bearing = bearing
        self.update()
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        size = min(self.width(), self.height())
        center = QPoint(self.width() // 2, self.height() // 2)
        radius = size // 2 - 10
        painter.setPen(Qt.black)
        painter.setBrush(QColor(240, 240, 240))
        painter.drawEllipse(center, radius, radius)
        font = QFont("Arial", 12, QFont.Bold)
        painter.setFont(font)
        for angle, label in zip([0, 90, 180, 270], ["N", "E", "S", "O"]):
            rad = math.radians(angle)
            x = center.x() + (radius - 18) * math.sin(rad)
            y = center.y() - (radius - 18) * math.cos(rad)
            painter.drawText(int(x - 10), int(y + 10), label)
        painter.setPen(QColor(180, 180, 180))
        for a in range(0, 360, 30):
            rad = math.radians(a)
            x1 = center.x() + (radius - 7) * math.sin(rad)
            y1 = center.y() - (radius - 7) * math.cos(rad)
            x2 = center.x() + (radius - 18) * math.sin(rad)
            y2 = center.y() - (radius - 18) * math.cos(rad)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))
        painter.setPen(QPen(QColor(220, 20, 60), 4))
        rad = math.radians(self.camera_bearing)
        x = center.x() + (radius - 36) * math.sin(rad)
        y = center.y() - (radius - 36) * math.cos(rad)
        painter.drawLine(center, QPoint(int(x), int(y)))
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(50, 100, 255))
        rad = math.radians(self.heading)
        pts = [
            QPoint(center.x() + int((radius - 36) * math.sin(rad)),
                   center.y() - int((radius - 36) * math.cos(rad))),
            QPoint(center.x() + int(12 * math.sin(rad + math.radians(120))),
                   center.y() - int(12 * math.cos(rad + math.radians(120)))),
            QPoint(center.x() + int(12 * math.sin(rad + math.radians(240))),
                   center.y() - int(12 * math.cos(rad + math.radians(240))))
        ]
        painter.drawPolygon(QPolygon(pts))

class SignalKWidget(QGroupBox):
    def __init__(self):
        super().__init__("SIGNAL K")
        self.webview = QWebEngineView()
        self.webview.setUrl(QUrl("http://localhost:3000"))
        layout = QVBoxLayout(self)
        layout.addWidget(self.webview)
        self.setMinimumSize(420, 350)

class CameraInterface(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VIGY - PySide6 Interface")
        self.resize(1600, 900)

        # --- VIDEO ---
        video_group = QGroupBox("Flux vidéo caméra")
        vg_layout = QVBoxLayout(video_group)
        self.video_label = QLabel("Flux en cours...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(600, 350)
        vg_layout.addWidget(self.video_label)
        vid_ctrl = QHBoxLayout()
        self.start_btn = QPushButton("Démarrer Vidéo")
        self.stop_btn = QPushButton("Arrêter Vidéo")
        self.stop_btn.setEnabled(False)
        vid_ctrl.addWidget(self.start_btn)
        vid_ctrl.addWidget(self.stop_btn)
        vg_layout.addLayout(vid_ctrl)
        photo_ctrl = QHBoxLayout()
        self.photo_btn = QPushButton("Prendre Photo")
        self.open_btn = QPushButton("Ouvrir Dossier Photos")
        photo_ctrl.addWidget(self.photo_btn)
        photo_ctrl.addWidget(self.open_btn)
        vg_layout.addLayout(photo_ctrl)

        # --- AUTRES BLOCS ---
        self.bearing_input = QLineEdit("70")
        self.elev_input = QLineEdit("-10")
        self.result_label = QLabel("")
        send_btn = QPushButton("Envoyer")
        send_btn.clicked.connect(self.send_can)
        unlock_btn = QPushButton("Unlock")
        unlock_btn.clicked.connect(self.send_unlock)
        self.ind_label = QLabel("Cap visé : -- °")
        self.ind_label.setFont(QFont("Arial", 12, QFont.Bold))

        send_group = QGroupBox("Envoi CAN (0x20C)")
        sg_layout = QGridLayout(send_group)
        sg_layout.addWidget(QLabel("Bearing (°):"), 0, 0)
        sg_layout.addWidget(self.bearing_input, 0, 1)
        sg_layout.addWidget(QLabel("Elevation (°):"), 1, 0)
        sg_layout.addWidget(self.elev_input, 1, 1)
        sg_layout.addWidget(send_btn, 2, 0, 1, 2)
        sg_layout.addWidget(unlock_btn, 3, 0, 1, 2)
        sg_layout.addWidget(self.result_label, 4, 0, 1, 2)
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
        for code, label in FOV_MAP.items():
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, c=code: self.send_fov_zoom(c))
            zg_layout.addWidget(btn)
        self.zoom_lbl = QLabel("")
        zg_layout.addWidget(self.zoom_lbl)

        lrf_group = QGroupBox("Télémètre Laser")
        lrf_layout = QHBoxLayout(lrf_group)
        lrf_btn = QPushButton("LRF \U0001F52B")
        lrf_btn.setFixedWidth(100)
        lrf_btn.clicked.connect(self.send_lrf)
        self.lrf_label = QLabel("-- m")
        self.lrf_label.setFont(QFont("Arial", 14, QFont.Bold))
        self.target_label = QLabel("Lat cible : --\nLon cible : --")
        self.target_label.setFont(QFont("Arial", 12))
        lrf_layout.addWidget(lrf_btn)
        lrf_layout.addWidget(self.lrf_label)
        lrf_layout.addWidget(self.target_label)

        self.compass_widget = CompassWidget()
        gps_group = QGroupBox("Position bateau / cible")
        gps_layout = QVBoxLayout(gps_group)
        self.gps_label = QLabel("Lat : --\nLon : --\nCap : --")
        self.gps_label.setFont(QFont("Arial", 14, QFont.Bold))
        gps_layout.addWidget(self.gps_label)
        gps_layout.addWidget(self.compass_widget, alignment=Qt.AlignCenter)

        # --- ZONE BAS EN COLONNES ---
        bottom_layout = QHBoxLayout()

        # Colonne 1 : CAN + Live
        col1 = QVBoxLayout()
        col1.addWidget(send_group)
        col1.addWidget(live_group)
        bottom_layout.addLayout(col1)

        # Colonne 2 : Zoom + LRF
        col2 = QVBoxLayout()
        col2.addWidget(zoom_group)
        col2.addWidget(lrf_group)
        bottom_layout.addLayout(col2)

        # Colonne 3 : GPS/cible + Compass
        col3 = QVBoxLayout()
        col3.addWidget(gps_group)
        bottom_layout.addLayout(col3)

        autres_group = QGroupBox("AUTRES OPTIONS")
        autres_group.setLayout(bottom_layout)
        autres_group.setMinimumHeight(330)

        # --- HAUT ---
        self.signalk_widget = SignalKWidget()
        camera_main_box = QGroupBox("CAMERA")
        camera_main_layout = QVBoxLayout(camera_main_box)
        camera_main_layout.addWidget(video_group)
        camera_main_box.setMinimumSize(700, 500)

        top_layout = QHBoxLayout()
        top_layout.addWidget(self.signalk_widget, stretch=1)
        top_layout.addWidget(camera_main_box, stretch=2)

        main_layout = QVBoxLayout(self)
        main_layout.addLayout(top_layout, stretch=3)
        main_layout.addWidget(autres_group, stretch=2)

        # --- INIT DONNEES/THREADS ---
        self.last_elev = 0.0
        self.last_bear = 0.0
        self.last_head = 0.0
        self.lrf_dist_m = None
        self.last_lat = "--"
        self.last_lon = "--"
        self.last_lat_deg = None
        self.last_lon_deg = None
        self.target_lat = None
        self.target_lon = None

        try:
            self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)
        except:
            self.arduino = None
        self.last_sent_angle = None
        self.last_sent_time = time.time()

        self.can_thread = CANReader()
        self.can_thread.update_data.connect(self.on_position)
        self.can_thread.update_digital_zoom.connect(self.on_digital_zoom)
        self.can_thread.update_lrf.connect(self.on_lrf)
        self.can_thread.start()

        self.nmea_thread = NMEAReader(port='/dev/pts/2')
        self.nmea_thread.nmea_update.connect(self.on_nmea_update)
        self.nmea_thread.start()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.start_btn.clicked.connect(self.start_capture)
        self.stop_btn.clicked.connect(self.stop_capture)
        self.photo_btn.clicked.connect(self.take_photo)
        self.open_btn.clicked.connect(self.open_photos)
        self.send_unlock()

    def closeEvent(self, event):
        try:
            lsb_e, msb_e = self.deg_to_bytes(0.0, ELEV_FACTOR)
            lsb_b, msb_b = self.deg_to_bytes(360.0, BEAR_FACTOR)
            data = f"0C{lsb_e:02X}{msb_e:02X}{lsb_b:02X}{msb_b:02X}"
            subprocess.call(f"cansend can0 20C#{data}", shell=True)
        except:
            pass
        if getattr(self, 'arduino', None):
            try: self.arduino.write(b"0\n")
            except: pass
            self.arduino.close()
        self.can_thread.stop()
        self.nmea_thread.stop()
        self.stop_capture()
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
            lsb_e, msb_e = self.deg_to_bytes(e, ELEV_FACTOR, neg_offset=e<0)
            lsb_b, msb_b = self.deg_to_bytes(360 - b, BEAR_FACTOR)
            data = f"0C{lsb_e:02X}{msb_e:02X}{lsb_b:02X}{msb_b:02X}"
            subprocess.call(f"cansend can0 20C#{data}", shell=True)
            self.result_label.setText(f"Trame envoyée: {data}")
        except:
            self.result_label.setText("Erreur: élévation hors plage")

    def send_unlock(self):
        subprocess.call("cansend can0 205#00", shell=True)

    def send_fov_zoom(self, code):
        db = [0x00, code] + [0x00]*6
        d = ''.join(f"{byte:02X}" for byte in db)
        subprocess.call(f"cansend can0 202#{d}", shell=True)
        self.zoom_lbl.setText(f"FoV: {FOV_MAP.get(code,'')}")

    def send_lrf(self):
        subprocess.call("cansend can0 202#0000000000004000", shell=True)

    def on_position(self, elev, bear):
        self.last_elev, self.last_bear = elev, bear
        self.elev_live.setText(f"Élévation : {elev:.2f} °")
        self.bear_live.setText(f"Bearing : {bear:.2f} °")
        self.ind_label.setText(f"Cap visé : {bear:.2f} °")
        self.compass_widget.set_camera_bearing(bear)
        if self.arduino and self.arduino.is_open:
            bearing_int = int(round(bear)) % 360
            now = time.time()
            if (self.last_sent_angle is None or
                abs(bearing_int - self.last_sent_angle) > 2 or
                (now - self.last_sent_time) > 0.12):
                try:
                    message = f"{bearing_int}\n"
                    self.arduino.write(message.encode())
                    print(f"[DEBUG] Envoi vers moteur pas-à-pas : {bearing_int}")
                    self.last_sent_angle = bearing_int
                    self.last_sent_time = now
                except Exception as e:
                    print(f"[ERREUR] Envoi série moteur : {e}")

    def on_digital_zoom(self, raw):
        pass

    def on_nmea_update(self, lat, lon, heading):
        self.last_lat, self.last_lon, self.last_head = lat, lon, heading
        try:
            deg = float(lat.split('°')[0]); dir_ = lat.split()[-1]
            self.last_lat_deg = deg if dir_=='N' else -deg
            deg = float(lon.split('°')[0]); dir_ = lon.split()[-1]
            self.last_lon_deg = deg if dir_=='E' else -deg
        except:
            self.last_lat_deg = self.last_lon_deg = None
        self.gps_label.setText(f"Lat : {lat}\nLon : {lon}\nCap : {heading:.2f}°")
        self.compass_widget.set_heading(heading)

    def compute_target_latlon(self):
        if None in (
            self.last_lat_deg,
            self.last_lon_deg,
            self.last_bear,
            self.last_elev,
            self.last_head,
            self.lrf_dist_m
        ):
            return
        lat_cam = self.last_lat_deg
        lon_cam = self.last_lon_deg
        bearing_cam = self.last_bear
        heading_boat = self.last_head
        elevation_cam = self.last_elev
        distance_lrf_m = self.lrf_dist_m
        bearing_abs = (bearing_cam + heading_boat) % 360
        angle_h = -elevation_cam
        angle_h_rad = math.radians(angle_h)
        d_ground = math.cos(angle_h_rad) * distance_lrf_m
        bearing_rad = math.radians(bearing_abs)
        lat1_rad = math.radians(lat_cam)
        lon1_rad = math.radians(lon_cam)
        dx = d_ground * math.sin(bearing_rad)
        dy = d_ground * math.cos(bearing_rad)
        R = 6378137.0
        delta_lat = dy / R
        delta_lon = dx / (R * math.cos(lat1_rad))
        lat_target = -(lat_cam + math.degrees(delta_lat))
        lon_target = -(lon_cam + math.degrees(delta_lon))
        self.target_lat = lat_target
        self.target_lon = lon_target
        self.target_label.setText(
            f"Lat cible : {lat_target:.5f}°\n"
            f"Lon cible : {lon_target:.5f}°"
        )
        print(f"[DEBUG] Position caméra : {lat_cam}, {lon_cam}")
        print(f"[DEBUG] Cap bateau = {heading_boat:.2f}°, bearing caméra = {bearing_cam:.2f}° → absolu = {bearing_abs:.2f}°")
        print(f"[DEBUG] Élévation = {elevation_cam:.2f}°, d_sol = {d_ground:.2f} m")
        print(f"[DEBUG] Δlat = {math.degrees(delta_lat):.5f}, Δlon = {math.degrees(delta_lon):.5f}")
        print(f"[DEBUG] Cible : {lat_target:.5f}, {lon_target:.5f}")

    def on_lrf(self, dist_m):
        if dist_m == 0:
            self.lrf_label.setText("-- m")
            self.target_label.setText("Lat cible : --\nLon cible : --")
        else:
            self.lrf_dist_m = dist_m
            self.lrf_label.setText(f"{dist_m:.2f} m")
            self.compute_target_latlon()

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
            if not ret: return
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            fld = os.path.join('photos', f'photo_{ts}')
            os.makedirs(fld, exist_ok=True)
            cv2.imwrite(os.path.join(fld, f'{ts}.jpg'), frm)
            info = os.path.join(fld, 'info.txt')
            with open(info, 'w') as f:
                f.write(f"Timestamp: {ts}\n")
                f.write(f"Élévation: {self.last_elev:.2f} °\n")
                f.write(f"Bearing: {self.last_bear:.2f} °\n")
                f.write(f"Lat bateau: {self.last_lat}\n")
                f.write(f"Lon bateau: {self.last_lon}\n")
                if self.target_lat and self.target_lon:
                    f.write(f"Lat cible: {self.target_lat:.5f}°\n")
                    f.write(f"Lon cible: {self.target_lon:.5f}°\n")
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
