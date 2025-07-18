import sys
import os
import math
import struct
import subprocess
import can
import shutil
from datetime import datetime

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QLineEdit, QGroupBox, QGridLayout, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, QThread, Signal, QPoint, QUrl
from PySide6.QtGui import QPainter, QColor, QPolygon, QFont, QIcon, QDesktopServices

# === Constants ===
ELEV_FACTOR = 0.0573
BEAR_FACTOR = 0.0573
NEG_OFFSET = 31415.5

# FoV Zoom mapping codes to labels
FOV_MAP = {0x04: 'VWFoV', 0x08: 'WFoV', 0x0C: 'NFoV'}

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
            # Position actuelle (0x105)
            if msg.arbitration_id == 0x105 and len(msg.data) >= 8:
                elev = math.degrees(struct.unpack('<f', msg.data[0:4])[0])
                bear = math.degrees(struct.unpack('<f', msg.data[4:8])[0])
                if elev >= 180:
                    elev -= 360
                self.update_data.emit(elev, bear)
            # Digital zoom (0x101)
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
        pts = [QPoint(c.x(),c.y()-sz), QPoint(c.x()+sz//2,c.y()+sz//2), QPoint(c.x()-sz//2,c.y()+sz//2)]
        painter.setBrush(Qt.black)
        painter.drawPolygon(QPolygon(pts))
        ex = c.x() + 40 * math.sin(math.radians(self.bearing))
        ey = c.y() - 40 * math.cos(math.radians(self.bearing))
        painter.setPen(QColor(255,0,0))
        painter.drawLine(c, QPoint(int(ex),int(ey)))

class CameraInterface(QWidget):
    def __init__(self):
        super().__init__()
        # Detect 'capture' CLI: PATH or known SDK directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            shutil.which('capture'),
            os.path.join(script_dir, '..', 'sdk_x53_linux_1.2.21', 'capture'),
            '/home/invite/Desktop/python/sdk_x53_linux_1.2.21/capture'
        ]
        self.capture_cmd = None
        for cmd in candidates:
            if cmd and os.path.isfile(cmd) and os.access(cmd, os.X_OK):
                self.capture_cmd = cmd
                break
        if not self.capture_cmd:
            print("Warning: 'capture' CLI not found in PATH or SDK folder.")

        self.setWindowTitle("VIGY - PySide6 Interface")
        self.setFixedSize(1000,700)
        # Context
        self.last_elev = 0.0
        self.last_bear = 0.0
        self.fov_zoom_code = None
        self.digital_zoom_val = 0

        # CAN controls
        self.bearing_input = QLineEdit("70")
        self.elev_input = QLineEdit("-10")
        self.result_label = QLabel("")
        send_btn = QPushButton("Envoyer")
        send_btn.clicked.connect(self.send_can)
        unlock_btn = QPushButton("Unlock")
        unlock_btn.clicked.connect(self.send_unlock)
        self.ind_label = QLabel("Cap visé : -- °")
        self.ind_label.setFont(QFont("Arial",12,QFont.Bold))
        self.dir_widget = DirectionWidget()
        send_group = QGroupBox("Envoi CAN (0x20C)")
        sl = QGridLayout(send_group)
        sl.addWidget(QLabel("Bearing (°):"),0,0)
        sl.addWidget(self.bearing_input,0,1)
        sl.addWidget(QLabel("Elevation (°):"),1,0)
        sl.addWidget(self.elev_input,1,1)
        sl.addWidget(send_btn,2,0,1,2)
        sl.addWidget(unlock_btn,3,0,1,2)
        sl.addWidget(self.result_label,4,0,1,2)
        sl.addWidget(self.dir_widget,5,0)
        sl.addWidget(self.ind_label,5,1)

        # Live CAN display
        live_group = QGroupBox("Position actuelle (0x105)")
        ll = QVBoxLayout(live_group)
        self.elev_live = QLabel("Élévation : 0.00 °")
        self.elev_live.setFont(QFont("Arial",14))
        self.bear_live = QLabel("Bearing : 0.00 °")
        self.bear_live.setFont(QFont("Arial",14))
        ll.addWidget(self.elev_live)
        ll.addWidget(self.bear_live)

        # FoV zoom group
        zoom_group = QGroupBox("Zoom FoV (0x202)")
        zl = QHBoxLayout(zoom_group)
        self.zoom_lbl = QLabel("")
        for code,label in FOV_MAP.items():
            btn = QPushButton(label)
            btn.clicked.connect(lambda _,c=code: self.send_fov_zoom(c))
            zl.addWidget(btn)
        zl.addWidget(self.zoom_lbl)

        # Video & photo controls
        video_group = QGroupBox("Flux vidéo caméra")
        vg = QVBoxLayout(video_group)
        self.video_label = QLabel("Flux en cours...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        ctrl = QHBoxLayout()
        self.start_btn = QPushButton("Démarrer Flux")
        self.stop_btn = QPushButton("Arrêter Flux")
        self.stop_btn.setEnabled(False)
        ctrl.addWidget(self.start_btn); ctrl.addWidget(self.stop_btn)
        photo_ctrl = QHBoxLayout()
        self.photo_btn = QPushButton("Prendre Photo")
        self.photo_btn.setIcon(QIcon.fromTheme("camera-photo"))
        self.open_btn = QPushButton("Ouvrir Dossier Photos")
        photo_ctrl.addWidget(self.photo_btn); photo_ctrl.addWidget(self.open_btn)
        vg.addWidget(self.video_label)
        vg.addLayout(ctrl)
        vg.addLayout(photo_ctrl)

        # Main layout arrangement
        m = QHBoxLayout(self)
        cl = QVBoxLayout()
        cl.addWidget(send_group); cl.addWidget(live_group); cl.addWidget(zoom_group)
        m.addLayout(cl); m.addWidget(video_group)

        # Initialize CAN thread and signals
        self.can_thread = CANReader()
        self.can_thread.update_data.connect(self.on_position)
        self.can_thread.update_digital_zoom.connect(self.on_digital_zoom)
        self.can_thread.start()
        self.start_btn.clicked.connect(self.start_stream)
        self.stop_btn.clicked.connect(self.stop_stream)
        self.photo_btn.clicked.connect(self.take_photo)
        self.open_btn.clicked.connect(self.open_photos)

    def send_can(self):
        try:
            b = float(self.bearing_input.text()); e = float(self.elev_input.text())
            if e < -30 or e >= 70:
                raise ValueError
            lsb_e, msb_e = self.deg_to_bytes(e, ELEV_FACTOR, neg_offset=e<0)
            lsb_b, msb_b = self.deg_to_bytes(360-b, BEAR_FACTOR)
            data = f"0C{lsb_e:02X}{msb_e:02X}{lsb_b:02X}{msb_b:02X}"
            subprocess.call(["cansend","can0","20C#"+data])
            self.result_label.setText(f"Trame 0x20C envoyée: {data}")
        except:
            self.result_label.setText("Erreur: élévation hors plage")

    def send_unlock(self):
        subprocess.call(["cansend","can0","205#00"]); self.result_label.setText("Trame 0x205#00 envoyée")

    def send_fov_zoom(self, code):
        data = ''.join(f"{b:02X}" for b in ([0x00, code] + [0x00]*6))
        subprocess.call(["cansend","can0","202#"+data])
        self.fov_zoom_code = code; self.zoom_lbl.setText(f"FoV: {FOV_MAP[code]}")

    def on_position(self, elev, bear):
        self.last_elev = elev; self.last_bear = bear
        self.elev_live.setText(f"Élévation : {elev:.2f} °")
        self.bear_live.setText(f"Bearing : {bear:.2f} °")
        self.ind_label.setText(f"Cap visé : {bear:.2f} °"); self.dir_widget.set_bearing(bear)

    def on_digital_zoom(self, raw):
        self.digital_zoom_val = raw

    def start_stream(self):
        if not self.capture_cmd:
            self.result_label.setText("Error: capture CLI not found"); return
        try:
            self.stream_proc = subprocess.Popen([self.capture_cmd, '-x', '-o', '/dev/stdout'], stdout=subprocess.PIPE)
            self.start_btn.setEnabled(False); self.stop_btn.setEnabled(True)
        except Exception as e:
            self.result_label.setText(f"Stream error: {e}")

    def stop_stream(self):
        if hasattr(self, 'stream_proc') and self.stream_proc:
            self.stream_proc.terminate(); self.stream_proc = None
        self.start_btn.setEnabled(True); self.stop_btn.setEnabled(False)

    def take_photo(self):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        folder = os.path.join('photos', f'photo_{ts}')
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, f'{ts}.jpg')
        if not self.capture_cmd:
            self.result_label.setText("Error: capture CLI not found"); return
        subprocess.call([self.capture_cmd, '-j', '-o', path])
        fov_label = FOV_MAP.get(self.fov_zoom_code, 'None')
        raw = self.digital_zoom_val or 0
        with open(os.path.join(folder, 'info.txt'), 'w') as f:
            f.write(f"Timestamp: {ts}\n")
            f.write(f"Élévation: {self.last_elev:.2f} °\n")
            f.write(f"Bearing: {self.last_bear:.2f} °\n")
            f.write(f"FoV Zoom: {fov_label}\n")
            if raw >= 2000:
                if raw < 3000:
                    dz = 2 * (raw - 2000) / 1000.0
                elif raw < 4000:
                    dz = 2 + 2 * (raw - 3000) / 1000.0
                else:
                    dz = 4 + 4 * (raw - 4000) / 1000.0
                f.write(f"Digital Zoom: NFoV + digital {dz:.2f}x ({raw})\n")
        self.result_label.setText(f"Photo saved: {path}")

    def open_photos(self):
        p = os.path.abspath('photos'); os.makedirs(p, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(p))

    def deg_to_bytes(self, angle, factor, neg_offset=False):
        raw = int(round(angle / factor + (NEG_OFFSET if neg_offset else 0))) & 0xFFFF
        return raw & 0xFF, raw >> 8

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = CameraInterface(); w.show(); sys.exit(app.exec())
