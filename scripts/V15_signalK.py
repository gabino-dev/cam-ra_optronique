#!/usr/bin/env python3
import os
import sys
import math
import struct
import subprocess
import can
import cv2
import serial
import time
from datetime import datetime

# Pour forcer X11 si sous Wayland (VM)
os.environ.pop("LD_LIBRARY_PATH", None)
os.environ["QT_QPA_PLATFORM"] = "xcb"

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QLineEdit, QGroupBox, QGridLayout, QSizePolicy, QSplitter
)
from PySide6.QtCore import Qt, QTimer, QThread, Signal, QPoint, QUrl
from PySide6.QtGui import (
    QPainter, QColor, QPolygon, QFont, QImage, QPixmap, QIcon, QDesktopServices, QPen
)
from PySide6.QtWebEngineWidgets import QWebEngineView

import pynmea2  # pip install pynmea2

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

        # cadran
        painter.setPen(Qt.black)
        painter.setBrush(QColor(240, 240, 240))
        painter.drawEllipse(center, radius, radius)

        # points cardinaux
        painter.setFont(QFont("Arial", 12, QFont.Bold))
        for angle, label in zip([0,90,180,270], ["N","E","S","O"]):
            rad = math.radians(angle)
            x = center.x() + (radius-18) * math.sin(rad)
            y = center.y() - (radius-18) * math.cos(rad)
            painter.drawText(int(x-10), int(y+10), label)

        # traits secondaires
        painter.setPen(QColor(180,180,180))
        for a in range(0,360,30):
            rad = math.radians(a)
            x1 = center.x() + (radius-7)*math.sin(rad)
            y1 = center.y() - (radius-7)*math.cos(rad)
            x2 = center.x() + (radius-18)*math.sin(rad)
            y2 = center.y() - (radius-18)*math.cos(rad)
            painter.drawLine(int(x1),int(y1),int(x2),int(y2))

        # flèche caméra
        painter.setPen(QPen(QColor(220,20,60),4))
        rad = math.radians(self.camera_bearing)
        x = center.x() + (radius-36)*math.sin(rad)
        y = center.y() - (radius-36)*math.cos(rad)
        painter.drawLine(center, QPoint(int(x),int(y)))

        # triangle bateau
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(50,100,255))
        rad = math.radians(self.heading)
        pts = [
            QPoint(center.x()+int((radius-36)*math.sin(rad)), center.y()-int((radius-36)*math.cos(rad))),
            QPoint(center.x()+int(12*math.sin(rad+math.radians(120))), center.y()-int(12*math.cos(rad+math.radians(120)))),
            QPoint(center.x()+int(12*math.sin(rad+math.radians(240))), center.y()-int(12*math.cos(rad+math.radians(240))))
        ]
        painter.drawPolygon(QPolygon(pts))

class CameraInterface(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VIGY - Caméra & Freeboard")
        self.resize(1400, 900)

        # Web Freeboard
        self.web = QWebEngineView()
        self.web.setUrl(QUrl("http://localhost:3000/@signalk/freeboard-sk"))

        # Video/logo caméra
        self.video_label = QLabel("Flux en cours...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.start_btn = QPushButton("Démarrer Vidéo")
        self.stop_btn = QPushButton("Arrêter Vidéo")
        self.stop_btn.setEnabled(False)
        self.photo_btn = QPushButton("Prendre Photo")
        self.open_btn = QPushButton("Ouvrir Dossier Photos")

        # regrouper caméra
        cam_group = QGroupBox("Flux vidéo caméra")
        cam_layout = QVBoxLayout(cam_group)
        cam_layout.addWidget(self.video_label)
        btns = QHBoxLayout()
        btns.addWidget(self.start_btn)
        btns.addWidget(self.stop_btn)
        cam_layout.addLayout(btns)
        pic = QHBoxLayout()
        pic.addWidget(self.photo_btn)
        pic.addWidget(self.open_btn)
        cam_layout.addLayout(pic)

        # splitter haut
        top_split = QSplitter(Qt.Horizontal)
        top_split.addWidget(self.web)
        top_split.addWidget(cam_group)
        top_split.setStretchFactor(0,2)
        top_split.setStretchFactor(1,1)

        # autres contrôles et widgets (bottom)
        # ... réutiliser les widgets existants pour les commandes CAN, LRF, compass, etc.
        bottom = QWidget()
        # Créez un layout bottom avec tous les autres groupes (Envoi CAN, LRF, GPS & cible, Zoom FoV)
        bottom_layout = QVBoxLayout(bottom)
        # TODO: reconstruire les groupes comme dans le code d'origine

        # Layout principal
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(top_split)
        main_layout.addWidget(bottom)

        # Connecter signaux, threads, etc.
        # (reprise du code original pour CANReader, NMEAReader, envoi CAN, calcul position...)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = CameraInterface()
    w.show()
    sys.exit(app.exec())
