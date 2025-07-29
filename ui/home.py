from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QGroupBox, QGridLayout
from PySide6.QtCore import Qt, QTimer, QUrl, QPoint
from PySide6.QtGui import QImage, QPixmap, QFont, QDesktopServices, QPainter, QColor, QPolygon, QPen
import cv2
import os
import time
from datetime import datetime
from .logic import CANReader, NMEAReader, FOV_MAP, send_camera_as_cog_nmea, ELEV_FACTOR, BEAR_FACTOR
import serial
import math
from PySide6.QtWebEngineWidgets import QWebEngineView
import av

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
        # 1. Compas principal (fond)
        painter.setPen(Qt.black)
        painter.setBrush(QColor(240, 240, 240))
        painter.drawEllipse(center, radius, radius)
        font = QFont("Arial", 12, QFont.Bold)
        painter.setFont(font)
        for angle, label in zip([0, 90, 180, 270], ["N", "E", "S", "O"]):
            rad_label = math.radians(angle)
            x = center.x() + (radius - 18) * math.sin(rad_label)
            y = center.y() - (radius - 18) * math.cos(rad_label)
            painter.drawText(int(x - 10), int(y + 10), label)
        painter.setPen(QColor(180, 180, 180))
        for a in range(0, 360, 30):
            rad2 = math.radians(a)
            x1 = center.x() + (radius - 7) * math.sin(rad2)
            y1 = center.y() - (radius - 7) * math.cos(rad2)
            x2 = center.x() + (radius - 18) * math.sin(rad2)
            y2 = center.y() - (radius - 18) * math.cos(rad2)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))
        # 2. Cercle bleu (bateau)
        painter.setPen(QPen(QColor(50, 120, 255), 5))
        painter.setBrush(QColor(120, 170, 255, 180))
        painter.drawEllipse(center, 13, 13)
        # 3. Triangle cap bateau (bleu)
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
        # 4. TRAIT ROUGE (CAMERA) toujours RELIE au CAP du bateau !
        painter.setPen(QPen(QColor(220, 20, 60), 4))
        rad_cam = math.radians((self.heading + self.camera_bearing) % 360)
        x = center.x() + (radius - 5) * math.sin(rad_cam)
        y = center.y() - (radius - 5) * math.cos(rad_cam)
        painter.drawLine(center, QPoint(int(x), int(y)))

class HomePage(QWidget):
    def __init__(self, can_thread, nmea_thread):
        super().__init__()
        self.can_thread = can_thread
        self.nmea_thread = nmea_thread
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("<h2>Accueil - Vidéo</h2>"))
        # --- HAUT ---
        top_layout = QHBoxLayout()
        # Bloc Freeboard
        self.signalk_widget = QWebEngineView()
        self.signalk_widget.setUrl(QUrl("http://localhost:3000/@signalk/freeboard-sk/"))
        self.signalk_widget.setMinimumSize(420, 350)
        top_layout.addWidget(self.signalk_widget, stretch=1)
        # Bloc vidéo (refonte)
        video_col = QVBoxLayout()
        video_group = QGroupBox("Flux vidéo caméra")
        video_group.setStyleSheet("QGroupBox { margin-top: 18px; font-size: 18px; }")
        vg_layout = QVBoxLayout(video_group)
        self.video_label = QLabel("Flux en cours...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(800, 480)
        self.video_label.setStyleSheet("background: #e0e0e0; border-radius: 8px; margin: 8px;")
        vg_layout.addWidget(self.video_label, alignment=Qt.AlignCenter)
        # Boutons sous la vidéo
        btn_row = QHBoxLayout()
        self.start_btn = QPushButton("▶️ Démarrer Vidéo")
        self.stop_btn = QPushButton("⏹️ Arrêter Vidéo")
        self.stop_btn.setEnabled(False)
        self.photo_btn = QPushButton("📸 Prendre Photo")
        self.open_btn = QPushButton("🗂️ Ouvrir Dossier Photos")
        for btn in [self.start_btn, self.stop_btn, self.photo_btn, self.open_btn]:
            btn.setMinimumHeight(38)
            btn.setStyleSheet("font-size: 16px; padding: 0 18px;")
            btn_row.addWidget(btn)
        vg_layout.addLayout(btn_row)
        video_col.addWidget(video_group)
        top_layout.addLayout(video_col, stretch=2)
        layout.addLayout(top_layout)
        # --- INFOS LIVE ---
        info_group = QGroupBox("Infos Live")
        info_layout = QGridLayout(info_group)
        self.elev_live = QLabel("Élévation : 0.00 °")
        self.bear_live = QLabel("Bearing : 0.00 °")
        self.cap_live = QLabel("Cap visé : -- °")
        self.gps_label = QLabel("Lat : --\nLon : --\nCap : --")
        info_layout.addWidget(self.elev_live, 0, 0)
        info_layout.addWidget(self.bear_live, 0, 1)
        info_layout.addWidget(self.cap_live, 1, 0)
        info_layout.addWidget(self.gps_label, 1, 1)
        self.compass_widget = CompassWidget()
        info_layout.addWidget(self.compass_widget, 0, 2, 2, 1)
        layout.addWidget(info_group)
        # --- COMMANDES CAN ---
        can_group = QGroupBox("Envoi CAN (0x20C)")
        can_layout = QGridLayout(can_group)
        from PySide6.QtWidgets import QLineEdit
        self.bearing_input = QLineEdit("70")
        self.elev_input = QLineEdit("-10")
        self.result_label = QLabel("")
        send_btn = QPushButton("Envoyer")
        unlock_btn = QPushButton("Unlock")
        can_layout.addWidget(QLabel("Bearing (°):"), 0, 0)
        can_layout.addWidget(self.bearing_input, 0, 1)
        can_layout.addWidget(QLabel("Elevation (°):"), 1, 0)
        can_layout.addWidget(self.elev_input, 1, 1)
        can_layout.addWidget(send_btn, 2, 0, 1, 2)
        can_layout.addWidget(unlock_btn, 3, 0, 1, 2)
        can_layout.addWidget(self.result_label, 4, 0, 1, 2)
        layout.addWidget(can_group)
        # --- ZOOM FOV ---
        zoom_group = QGroupBox("Zoom FoV (0x202)")
        zg_layout = QHBoxLayout(zoom_group)
        for code, label in FOV_MAP.items():
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, c=code: self.send_fov_zoom(c))
            zg_layout.addWidget(btn)
        self.zoom_lbl = QLabel("")
        zg_layout.addWidget(self.zoom_lbl)
        layout.addWidget(zoom_group)
        # --- LRF ---
        lrf_group = QGroupBox("Télémètre Laser")
        lrf_layout = QHBoxLayout(lrf_group)
        lrf_btn = QPushButton("LRF \U0001F52B")
        lrf_btn.setFixedWidth(100)
        lrf_btn.clicked.connect(self.send_lrf)
        self.lrf_label = QLabel("-- m")
        self.lrf_label.setFont(QFont("Arial", 14, QFont.Bold))
        lrf_layout.addWidget(lrf_btn)
        lrf_layout.addWidget(self.lrf_label)
        layout.addWidget(lrf_group)
        layout.addStretch()
        # --- LOGIQUE ---
        self.last_elev = 0.0
        self.last_bear = 0.0
        self.last_head = 0.0
        self.lrf_dist_m = None
        self.last_lat = "--"
        self.last_lon = "--"
        self.last_lat_deg = None
        self.last_lon_deg = None
        self.cap_live.setText(f"Cap visé : -- °")
        self.last_sent_angle = None
        self.last_sent_time = time.time()
        # Connexion Arduino
        try:
            self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)
        except:
            self.arduino = None
        # Connexion signaux
        self.can_thread.update_data.connect(self.on_position)
        self.can_thread.update_digital_zoom.connect(self.on_digital_zoom)
        self.can_thread.update_lrf.connect(self.on_lrf)
        self.nmea_thread.nmea_update.connect(self.on_nmea_update)
        self.nmea_thread.ttm_trace.connect(self.on_ttm_trace)
        # Vidéo
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.start_btn.clicked.connect(self.start_capture)
        self.stop_btn.clicked.connect(self.stop_capture)
        self.photo_btn.clicked.connect(self.take_photo)
        self.open_btn.clicked.connect(self.open_photos)
        send_btn.clicked.connect(self.send_can)
        unlock_btn.clicked.connect(self.send_unlock)
    def start_capture(self):
        # Capture H264 natif via PyAV
        self.container = av.open('/dev/video3', format='v4l2', options={'input_format': 'h264'})
        self.video_stream = self.container.streams.video[0]
        self.video_stream.thread_type = 'AUTO'
        self.frame_iter = self.container.decode(self.video_stream)
        self.timer.start(30)
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

    def update_frame(self):
        try:
            frame = next(self.frame_iter)
            img = frame.to_ndarray(format='rgb24')
            h, w, ch = img.shape
            qimg = QImage(img.data, w, h, ch*w, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(qimg).scaled(
                self.video_label.size(), Qt.KeepAspectRatio))
        except StopIteration:
            pass
        except Exception as e:
            print(f"[PyAV] Erreur lecture frame: {e}")

    def stop_capture(self):
        self.timer.stop()
        if hasattr(self, 'container'):
            self.container.close()
            del self.container
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
            self.result_label.setText(f"Info saved in {fld}")
    def open_photos(self):
        p = os.path.abspath('photos')
        os.makedirs(p, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(p))
    def send_can(self):
        import subprocess
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
        import subprocess
        subprocess.call("cansend can0 205#00", shell=True)
    def send_fov_zoom(self, code):
        import subprocess
        db = [0x00, code] + [0x00]*6
        d = ''.join(f"{byte:02X}" for byte in db)
        subprocess.call(f"cansend can0 202#{d}", shell=True)
        self.zoom_lbl.setText(f"FoV: {FOV_MAP.get(code,'')}")
    def send_lrf(self):
        import subprocess
        subprocess.call("cansend can0 202#0000000000004000", shell=True)
    def on_position(self, elev, bear):
        self.last_elev, self.last_bear = elev, bear
        self.elev_live.setText(f"Élévation : {elev:.2f} °")
        self.bear_live.setText(f"Bearing : {bear:.2f} °")
        self.cap_live.setText(f"Cap visé : {bear:.2f} °")
        send_camera_as_cog_nmea(bear, self.last_head)
        self.compass_widget.set_camera_bearing(bear)
        # Envoi vers Arduino moteur pas à pas
        if hasattr(self, 'arduino') and self.arduino and self.arduino.is_open:
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
        except Exception as e:
            print(f"[NMEA] Erreur conversion lat/lon: {e} | lat={lat} lon={lon}")
            self.last_lat_deg = self.last_lon_deg = None
        print(f"[DEBUG] GPS: last_lat_deg={self.last_lat_deg}, last_lon_deg={self.last_lon_deg}, last_head={self.last_head}")
        self.gps_label.setText(f"Lat : {lat}\nLon : {lon}\nCap : {heading:.2f}°")
        self.compass_widget.set_heading(heading)
    def on_ttm_trace(self, ttm_sentence):
        pass
    def on_lrf(self, dist_m):
        if dist_m == 0:
            self.lrf_label.setText("-- m")
        else:
            self.lrf_label.setText(f"{dist_m:.2f} m")
    def deg_to_bytes(self, angle, factor, neg_offset=False):
        raw = int(round(angle / factor + (31415.5 if neg_offset else 0))) & 0xFFFF
        return raw & 0xFF, raw >> 8 