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
    QPainter, QColor, QPolygon, QFont, QImage, QPixmap, QIcon,
    QDesktopServices, QPen
)

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
                dist_cm = msg.data[0] | (msg.data[1] << 8)
                self.update_lrf.emit(dist_cm)

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

class CompassWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.heading = 0.0          # cap bateau NMEA
        self.camera_bearing = 0.0   # bearing caméra relatif
        self.setFixedSize(160, 160)

    def set_heading(self, heading):
        self.heading = heading
        self.update()

    def set_camera_bearing(self, bearing):
        self.camera_bearing = bearing % 360
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        center = QPoint(self.width()//2, self.height()//2)
        radius = min(self.width(), self.height())//2 - 10

        # cercle
        painter.setPen(Qt.black)
        painter.setBrush(QColor(240,240,240))
        painter.drawEllipse(center, radius, radius)

        # cardinaux
        painter.setFont(QFont("Arial",10,QFont.Bold))
        for ang, lbl in zip([0,90,180,270],["N","E","S","O"]):
            r = math.radians(ang)
            x = center.x() + (radius-15)*math.sin(r)
            y = center.y() - (radius-15)*math.cos(r)
            painter.drawText(int(x-5),int(y+5),lbl)

        # secondaire
        painter.setPen(QColor(180,180,180))
        for a in range(0,360,30):
            r=math.radians(a)
            x1=center.x()+(radius-5)*math.sin(r)
            y1=center.y()-(radius-5)*math.cos(r)
            x2=center.x()+(radius-15)*math.sin(r)
            y2=center.y()-(radius-15)*math.cos(r)
            painter.drawLine(int(x1),int(y1),int(x2),int(y2))

        # triangle bateau
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(50,100,255))
        r=math.radians(self.heading)
        pts=[QPoint(center.x()+int((radius-20)*math.sin(r)),
                    center.y()-int((radius-20)*math.cos(r))),
             QPoint(center.x()+int(8*math.sin(r+2.09)),
                    center.y()-int(8*math.cos(r+2.09))),
             QPoint(center.x()+int(8*math.sin(r-2.09)),
                    center.y()-int(8*math.cos(r-2.09)))]
        painter.drawPolygon(QPolygon(pts))

        # flèche caméra (même delta)
        painter.setPen(QPen(QColor(220,20,60),3))
        cr=r  # same direction as boat
        x=center.x()+int((radius-30)*math.sin(cr))
        y=center.y()-int((radius-30)*math.cos(cr))
        painter.drawLine(center, QPoint(x,y))

class CameraInterface(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VIGY")
        self.setFixedSize(800,600)

        self.prev_heading = None
        self.cam_bearing = 0.0

        # widgets
        self.compass = CompassWidget()
        self.gps_lbl = QLabel("Lat: --\nLon: --")
        self.bear_live = QLabel("Cam: --°")
        self.elev_live = QLabel("Elév: --°")
        self.bearing_input=QLineEdit("0")
        self.elev_input=QLineEdit("0")
        send_btn=QPushButton("Tourner Caméra")
        send_btn.clicked.connect(self.send_can)

        # layout
        left=QVBoxLayout()
        left.addWidget(self.gps_lbl)
        left.addWidget(self.bear_live)
        left.addWidget(self.elev_live)
        left.addWidget(self.bearing_input)
        left.addWidget(self.elev_input)
        left.addWidget(send_btn)
        main=QHBoxLayout(self)
        main.addLayout(left)
        main.addWidget(self.compass)

        # threads
        self.nmea=NMEAReader()
        self.nmea.nmea_update.connect(self.on_nmea)
        self.nmea.start()

        self.canth=CANReader()
        self.canth.update_data.connect(self.on_can)
        self.canth.start()

        # video timer stub
        self.timer=QTimer(self)
        self.timer.timeout.connect(self.update_frame)

    def closeEvent(self,e):
        self.nmea.stop()
        self.canth.stop()
        e.accept()

    def on_nmea(self,lat,lon,hdg):
        # init
        if self.prev_heading is None:
            self.prev_heading=hdg
            self.cam_bearing=hdg
        # delta
        delta = (hdg - self.prev_heading + 360) % 360
        self.cam_bearing = (self.cam_bearing + delta) % 360
        self.prev_heading = hdg

        self.compass.set_heading(hdg)
        self.compass.set_camera_bearing(self.cam_bearing)
        self.gps_lbl.setText(f"Lat: {lat}\nLon: {lon}")

    def on_can(self,elev,bear):
        # camera command overrides
        self.cam_bearing = bear % 360
        self.compass.set_camera_bearing(self.cam_bearing)
        self.bear_live.setText(f"Cam: {bear:.1f}°")
        self.elev_live.setText(f"Elév: {elev:.1f}°")

    def send_can(self):
        try:
            b=float(self.bearing_input.text())
            e=float(self.elev_input.text())
            # send CAN 0x20C elevation/bearing...
            subprocess.call(f"cansend can0 20C#{int(e)&0xFF:02X}{int(e)>>8:02X}{int(b)&0xFF:02X}{int(b)>>8:02X}",shell=True)
        except:
            pass

    def update_frame(self):
        pass  # vidéo pas implémentée ici

if __name__=="__main__":
    app=QApplication(sys.argv)
    w=CameraInterface()
    w.show()
    sys.exit(app.exec())
