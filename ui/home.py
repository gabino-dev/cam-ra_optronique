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
        layout.setSpacing(15)
        layout.setContentsMargins(15, 15, 15, 15)
        
        # Titre avec meilleur contraste
        title = QLabel("<h2 style='color: #ffffff; background: #1a365d; padding: 15px; border-radius: 10px; margin: 0; border: 2px solid #3182ce;'>🎥 Accueil - Contrôle Caméra</h2>")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # --- LAYOUT PRINCIPAL : Caméra à gauche (2/3), Freeboard à droite (1/3) ---
        main_layout = QHBoxLayout()
        main_layout.setSpacing(20)
        
        # --- COLONNE GAUCHE (2/3) : Caméra et contrôles ---
        left_col = QVBoxLayout()
        left_col.setSpacing(15)
        
        # Bloc caméra avec meilleur style
        camera_group = QGroupBox("📹 Flux vidéo caméra")
        camera_group.setStyleSheet("""
            QGroupBox { 
                font-size: 18px; font-weight: bold; color: #ffffff;
                background: #2d3748; border: 3px solid #3182ce; border-radius: 10px;
                margin-top: 25px; padding-top: 15px; padding-bottom: 15px;
            }
            QGroupBox::title { 
                subcontrol-origin: margin; left: 15px; padding: 0 12px 0 12px; 
                background: #3182ce; color: white; border-radius: 6px;
                font-size: 16px; font-weight: bold;
            }
        """)
        cam_layout = QHBoxLayout(camera_group)
        cam_layout.setSpacing(15)
        
        # Vidéo à gauche avec meilleur contraste
        self.video_label = QLabel("Flux en cours...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(350, 280)
        self.video_label.setStyleSheet("""
            background: #1a202c; border: 3px solid #4a5568; border-radius: 8px; 
            color: #e2e8f0; font-size: 16px; font-weight: bold; padding: 20px;
        """)
        cam_layout.addWidget(self.video_label, stretch=2)
        
        # Colonne boutons à droite avec meilleur style
        btn_col = QVBoxLayout()
        btn_col.setSpacing(12)
        
        self.start_btn = QPushButton("▶️ Démarrer Vidéo")
        self.stop_btn = QPushButton("⏹️ Arrêter Vidéo")
        self.stop_btn.setEnabled(False)
        self.photo_btn = QPushButton("📸 Prendre Photo")
        self.open_btn = QPushButton("🗂️ Ouvrir Dossier Photos")
        
        for btn in [self.start_btn, self.stop_btn, self.photo_btn, self.open_btn]:
            btn.setMinimumHeight(50)
            btn.setStyleSheet("""
                QPushButton { 
                    font-size: 15px; font-weight: bold; padding: 10px 20px;
                    background: #3182ce; color: white; border: 2px solid #2c5aa0; border-radius: 8px;
                    margin-bottom: 10px;
                }
                QPushButton:hover { background: #2c5aa0; border: 2px solid #1e40af; }
                QPushButton:disabled { background: #4a5568; color: #a0aec0; border: 2px solid #2d3748; }
            """)
            btn_col.addWidget(btn)
        btn_col.addStretch()
        cam_layout.addLayout(btn_col, stretch=1)
        
        left_col.addWidget(camera_group)
        
        # --- INFOS LIVE avec meilleur contraste ---
        info_group = QGroupBox("📊 Infos Live")
        info_group.setStyleSheet("""
            QGroupBox { 
                font-size: 18px; font-weight: bold; color: #ffffff;
                background: #2d3748; border: 3px solid #805ad5; border-radius: 10px;
                margin-top: 25px; padding-top: 15px; padding-bottom: 15px;
            }
            QGroupBox::title { 
                subcontrol-origin: margin; left: 15px; padding: 0 12px 0 12px; 
                background: #805ad5; color: white; border-radius: 6px;
                font-size: 16px; font-weight: bold;
            }
        """)
        info_layout = QGridLayout(info_group)
        info_layout.setSpacing(12)
        
        self.elev_live = QLabel("Élévation : 0.00 °")
        self.bear_live = QLabel("Bearing : 0.00 °")
        self.cap_live = QLabel("Cap visé : -- °")
        self.gps_label = QLabel("Lat : --\nLon : --\nCap : --")
        
        # Style pour les labels d'info
        for label in [self.elev_live, self.bear_live, self.cap_live, self.gps_label]:
            label.setStyleSheet("""
                color: #f7fafc; font-size: 14px; font-weight: bold;
                background: #1a202c; padding: 12px; border-radius: 6px;
                border: 2px solid #4a5568;
            """)
        
        info_layout.addWidget(self.elev_live, 0, 0)
        info_layout.addWidget(self.bear_live, 0, 1)
        info_layout.addWidget(self.cap_live, 1, 0)
        info_layout.addWidget(self.gps_label, 1, 1)
        
        self.compass_widget = CompassWidget()
        info_layout.addWidget(self.compass_widget, 0, 2, 2, 1)
        
        left_col.addWidget(info_group)
        
        # --- COMMANDES CAN avec meilleur style ---
        can_group = QGroupBox("🎯 Envoi CAN (0x20C)")
        can_group.setStyleSheet("""
            QGroupBox { 
                font-size: 18px; font-weight: bold; color: #ffffff;
                background: #2d3748; border: 3px solid #38a169; border-radius: 10px;
                margin-top: 25px; padding-top: 15px; padding-bottom: 15px;
            }
            QGroupBox::title { 
                subcontrol-origin: margin; left: 15px; padding: 0 12px 0 12px; 
                background: #38a169; color: white; border-radius: 6px;
                font-size: 16px; font-weight: bold;
            }
        """)
        can_layout = QGridLayout(can_group)
        can_layout.setSpacing(10)
        
        from PySide6.QtWidgets import QLineEdit
        self.bearing_input = QLineEdit("70")
        self.elev_input = QLineEdit("-10")
        self.result_label = QLabel("")
        
        # Style pour les inputs
        for input_widget in [self.bearing_input, self.elev_input]:
            input_widget.setStyleSheet("""
                QLineEdit { 
                    background: #1a202c; color: #e2e8f0; border: 2px solid #4a5568;
                    border-radius: 6px; padding: 8px; font-size: 14px; font-weight: bold;
                }
                QLineEdit:focus { border: 3px solid #3182ce; background: #2d3748; }
            """)
        
        self.result_label.setStyleSheet("color: #f7fafc; font-size: 13px; padding: 8px; background: #1a202c; border-radius: 4px;")
        
        send_btn = QPushButton("🚀 Envoyer")
        unlock_btn = QPushButton("🔓 Unlock")
        
        for btn in [send_btn, unlock_btn]:
            btn.setStyleSheet("""
                QPushButton { 
                    background: #38a169; color: white; border: 2px solid #2f855a; border-radius: 6px;
                    padding: 10px 20px; font-weight: bold; font-size: 14px;
                }
                QPushButton:hover { background: #2f855a; border: 2px solid #276749; }
            """)
        
        can_layout.addWidget(QLabel("Bearing (°):"), 0, 0)
        can_layout.addWidget(self.bearing_input, 0, 1)
        can_layout.addWidget(QLabel("Elevation (°):"), 1, 0)
        can_layout.addWidget(self.elev_input, 1, 1)
        can_layout.addWidget(send_btn, 2, 0, 1, 2)
        can_layout.addWidget(unlock_btn, 3, 0, 1, 2)
        can_layout.addWidget(self.result_label, 4, 0, 1, 2)
        
        # Style pour les labels CAN
        for i in range(can_layout.rowCount()):
            item = can_layout.itemAtPosition(i, 0)
            if item and item.widget() and isinstance(item.widget(), QLabel):
                item.widget().setStyleSheet("color: #ffffff; font-size: 14px; font-weight: bold;")
        
        # --- ZOOM FOV avec meilleur style ---
        zoom_group = QGroupBox("🔍 Zoom FoV (0x202)")
        zoom_group.setStyleSheet("""
            QGroupBox { 
                font-size: 18px; font-weight: bold; color: #ffffff;
                background: #2d3748; border: 3px solid #d69e2e; border-radius: 10px;
                margin-top: 25px; padding-top: 15px; padding-bottom: 15px;
            }
            QGroupBox::title { 
                subcontrol-origin: margin; left: 15px; padding: 0 12px 0 12px; 
                background: #d69e2e; color: white; border-radius: 6px;
                font-size: 16px; font-weight: bold;
            }
        """)
        zg_layout = QVBoxLayout(zoom_group)
        zg_layout.setSpacing(12)
        
        for code, label in FOV_MAP.items():
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, c=code: self.send_fov_zoom(c))
            btn.setMinimumHeight(45)
            btn.setStyleSheet("""
                QPushButton { 
                    background: #d69e2e; color: white; border: 2px solid #b7791f; border-radius: 8px;
                    padding: 12px 20px; font-weight: bold; font-size: 15px;
                }
                QPushButton:hover { background: #b7791f; border: 2px solid #975a16; }
            """)
            zg_layout.addWidget(btn)
        
        # --- LRF avec meilleur style ---
        lrf_group = QGroupBox("🎯 Télémètre Laser")
        lrf_group.setStyleSheet("""
            QGroupBox { 
                font-size: 18px; font-weight: bold; color: #ffffff;
                background: #2d3748; border: 3px solid #e53e3e; border-radius: 10px;
                margin-top: 25px; padding-top: 15px; padding-bottom: 15px;
            }
            QGroupBox::title { 
                subcontrol-origin: margin; left: 15px; padding: 0 12px 0 12px; 
                background: #e53e3e; color: white; border-radius: 6px;
                font-size: 16px; font-weight: bold;
            }
        """)
        lrf_layout = QHBoxLayout(lrf_group)
        lrf_layout.setSpacing(12)
        
        lrf_btn = QPushButton("LRF 🔥")
        lrf_btn.setFixedWidth(130)
        lrf_btn.clicked.connect(self.send_lrf)
        lrf_btn.setStyleSheet("""
            QPushButton { 
                background: #e53e3e; color: white; border: 2px solid #c53030; border-radius: 6px;
                padding: 10px 20px; font-weight: bold; font-size: 14px;
            }
            QPushButton:hover { background: #c53030; border: 2px solid #9b2c2c; }
        """)
        
        self.lrf_label = QLabel("-- m")
        self.lrf_label.setFont(QFont("Arial", 16, QFont.Bold))
        self.lrf_label.setStyleSheet("color: #f7fafc; background: #1a202c; padding: 12px; border-radius: 6px; border: 2px solid #4a5568;")
        
        lrf_layout.addWidget(lrf_btn)
        lrf_layout.addWidget(self.lrf_label)
        lrf_layout.addStretch()
        
        # --- LAYOUT HORIZONTAL pour CAN, ZOOM et LRF ---
        controls_layout = QHBoxLayout()
        controls_layout.addWidget(can_group, stretch=1)
        controls_layout.addWidget(zoom_group, stretch=1)
        controls_layout.addWidget(lrf_group, stretch=1)
        left_col.addLayout(controls_layout)
        left_col.addStretch()
        
        # --- COLONNE DROITE (1/3) : Freeboard ---
        self.signalk_widget = QWebEngineView()
        self.signalk_widget.setUrl(QUrl("http://localhost:3000/@signalk/freeboard-sk/"))
        self.signalk_widget.setMinimumSize(400, 600)
        self.signalk_widget.setStyleSheet("border: 3px solid #4a5568; border-radius: 10px;")
        
        # Layout principal
        main_layout.addLayout(left_col, stretch=2)
        main_layout.addWidget(self.signalk_widget, stretch=1)
        
        layout.addLayout(main_layout)
        
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
                self.video_label.width(), self.video_label.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
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
        try:
            # Capture une frame PyAV
            frame = next(self.frame_iter)
            img = frame.to_ndarray(format='bgr24')
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            fld = os.path.join('photos', f'photo_{ts}')
            os.makedirs(fld, exist_ok=True)
            cv2.imwrite(os.path.join(fld, f'{ts}.jpg'), img)
            info = os.path.join(fld, 'info.txt')
            rel_bearing = (self.last_bear - self.last_head) % 360 if self.last_head is not None else None
            with open(info, 'w') as f:
                f.write(f"Timestamp: {ts}\n")
                f.write(f"Élévation: {self.last_elev:.2f} °\n")
                f.write(f"Bearing: {self.last_bear:.2f} °\n")
                f.write(f"Cap bateau: {self.last_head:.2f} °\n")
                if rel_bearing is not None:
                    f.write(f"Bearing relatif: {rel_bearing:.2f} °\n")
                f.write(f"Lat bateau: {self.last_lat}\n")
                f.write(f"Lon bateau: {self.last_lon}\n")
                if hasattr(self, 'target_lat') and hasattr(self, 'target_lon') and self.target_lat and self.target_lon:
                    f.write(f"Lat cible: {self.target_lat:.5f}°\n")
                    f.write(f"Lon cible: {self.target_lon:.5f}°\n")
            self.result_label.setText(f"Info saved in {fld}")
            # Rafraîchir la galerie si elle existe
            main_window = self.parent()
            while main_window and not hasattr(main_window, 'pages'):
                main_window = main_window.parent()
            if main_window and hasattr(main_window, 'pages'):
                gallery_page = main_window.pages.widget(2)
                if hasattr(gallery_page, 'refresh_gallery'):
                    gallery_page.refresh_gallery()
        except Exception as e:
            self.result_label.setText(f"Erreur capture photo: {e}")
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