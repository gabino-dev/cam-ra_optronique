from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QGroupBox, QListWidget, QListWidgetItem, QDialog
from PySide6.QtCore import Qt, QTimer, Signal
from datetime import datetime
import subprocess
from .logic import parse_ttm, dest_point, relative_bearing, ELEV_FACTOR, BEAR_FACTOR
from PySide6.QtWebEngineWidgets import QWebEngineView
from .home import CompassWidget
from PySide6.QtCore import QUrl

class RadarTargetWidget(QWidget):
    def __init__(self, nom, ttm_sentence):
        super().__init__()
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(4)
        
        # Texte TTM en haut
        self.lab = QLabel(f"{ttm_sentence}")
        self.lab.setStyleSheet("color: #fff; font-size: 11px; font-family: monospace; background: #2a2a2a; padding: 4px; border-radius: 3px;")
        self.lab.setWordWrap(True)
        main_layout.addWidget(self.lab)
        
        # Boutons en dessous
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(6)
        
        self.cibler_btn = QPushButton("🎯 Cibler")
        self.cibler_btn.setFixedSize(80, 28)
        self.cibler_btn.setStyleSheet("QPushButton { background: #0078d7; color: white; border-radius: 4px; font-weight: bold; font-size: 10px; }")
        
        self.lock_btn = QPushButton("🔒 Lock")
        self.lock_btn.setFixedSize(70, 28)
        self.lock_btn.setToolTip("Verrouiller le suivi (🔒)")
        self.lock_btn.setStyleSheet("QPushButton { background: #28a745; color: white; border-radius: 4px; font-weight: bold; font-size: 10px; }")
        
        self.unlock_btn = QPushButton("🔓 Unlock")
        self.unlock_btn.setFixedSize(85, 28)
        self.unlock_btn.setToolTip("Déverrouiller le suivi (🔓)")
        self.unlock_btn.setEnabled(False)
        self.unlock_btn.setStyleSheet("QPushButton { background: #dc3545; color: white; border-radius: 4px; font-weight: bold; font-size: 10px; }")
        
        btn_layout.addWidget(self.cibler_btn)
        btn_layout.addWidget(self.lock_btn)
        btn_layout.addWidget(self.unlock_btn)
        btn_layout.addStretch()
        
        main_layout.addLayout(btn_layout)
        self.setMinimumHeight(60)

class RadarTTMWidget(QGroupBox):
    ttm_target_selected = Signal(str)
    ttm_lock_selected = Signal(str)
    ttm_unlock_selected = Signal(str)
    def __init__(self):
        super().__init__("Traces Radar TTM")
        self.list = QListWidget()
        layout = QVBoxLayout(self)
        layout.addWidget(self.list)
        self.setMinimumWidth(520)
        self.setMinimumHeight(300)
        self.traces = {}
        self.locked_target = None
    def add_ttm(self, ttm_sentence):
        fields = ttm_sentence.strip().split(',')
        nom = fields[7] if len(fields) > 7 else ttm_sentence
        now = datetime.now().strftime("%H:%M:%S")
        if nom in self.traces:
            ttm_old, widget = self.traces[nom]
            widget.lab.setText(f"{now} - {ttm_sentence[:60]}")
            widget.ttm_sentence = ttm_sentence
            return
        widget = RadarTargetWidget(nom, ttm_sentence)
        item = QListWidgetItem(self.list)
        item.setSizeHint(widget.sizeHint())
        self.list.addItem(item)
        self.list.setItemWidget(item, widget)
        self.traces[nom] = (ttm_sentence, widget)
        widget.cibler_btn.clicked.connect(lambda: self.ttm_target_selected.emit(ttm_sentence))
        widget.lock_btn.clicked.connect(lambda: self.handle_lock(nom))
        widget.unlock_btn.clicked.connect(lambda: self.handle_unlock(nom))
    def handle_lock(self, nom):
        for n, (_, w) in self.traces.items():
            w.lock_btn.setEnabled(True)
            w.unlock_btn.setEnabled(False)
        if nom in self.traces:
            _, widget = self.traces[nom]
            widget.lock_btn.setEnabled(False)
            widget.unlock_btn.setEnabled(True)
            self.locked_target = nom
            self.ttm_lock_selected.emit(widget.ttm_sentence)
    def handle_unlock(self, nom):
        if nom in self.traces:
            _, widget = self.traces[nom]
            widget.lock_btn.setEnabled(True)
            widget.unlock_btn.setEnabled(False)
            if self.locked_target == nom:
                self.locked_target = None
                self.ttm_unlock_selected.emit(widget.ttm_sentence)

class RadarTTMWindow(QDialog):
    def __init__(self, radar_widget):
        super().__init__()
        self.setWindowTitle("Radar TTM")
        self.resize(600, 300)
        layout = QVBoxLayout(self)
        layout.addWidget(radar_widget)
        self.setLayout(layout)

class RadarPage(QWidget):
    def __init__(self, nmea_thread, get_boat_info=None, send_can_ciblage=None):
        super().__init__()
        self.nmea_thread = nmea_thread
        self.get_boat_info = get_boat_info
        self.send_can_ciblage = send_can_ciblage
        main_layout = QHBoxLayout(self)
        # Freeboard à gauche (2/3)
        self.signalk_widget = QWebEngineView()
        self.signalk_widget.setUrl(QUrl("http://localhost:3000/@signalk/freeboard-sk/"))
        self.signalk_widget.setMinimumSize(600, 500)
        main_layout.addWidget(self.signalk_widget, stretch=2)
        # Colonne de droite (1/3) : boussole en haut, radar_ttm_widget en dessous
        right_col = QVBoxLayout()
        self.compass_widget = CompassWidget()
        self.compass_widget.setFixedSize(180, 180)
        right_col.addWidget(self.compass_widget, alignment=Qt.AlignTop | Qt.AlignHCenter)
        self.radar_ttm_widget = RadarTTMWidget()
        right_col.addWidget(self.radar_ttm_widget, stretch=1)
        main_layout.addLayout(right_col, stretch=1)
        self.sim_btn = QPushButton("Lancer simulateur radar")
        self.sim_btn.setFixedWidth(250)
        self.sim_btn.clicked.connect(self.start_radar_sim)
        right_col.addWidget(self.sim_btn, alignment=Qt.AlignLeft)
        self.sim_proc = None
        self.auto_target_timer = QTimer(self)
        self.auto_target_timer.setInterval(500)
        self.auto_target_timer.timeout.connect(self.cibler_auto)
        self.locked_sentence = None
        self.radar_ttm_widget.ttm_target_selected.connect(self.cibler_depuis_ttm)
        self.radar_ttm_widget.ttm_lock_selected.connect(self.start_auto_cibler)
        self.radar_ttm_widget.ttm_unlock_selected.connect(self.stop_auto_cibler)
        # Connexion du signal ttm_trace
        self.nmea_thread.ttm_trace.connect(self.radar_ttm_widget.add_ttm)
        # Connexion boussole
        self.nmea_thread.nmea_update.connect(self.on_nmea_update)
        self.can_bearing = 0.0
    def on_nmea_update(self, lat, lon, heading):
        self.compass_widget.set_heading(heading)
        self.compass_widget.set_camera_bearing(self.can_bearing)
    def on_position(self, elev, bear):
        self.can_bearing = bear
        self.compass_widget.set_camera_bearing(bear)
    def start_radar_sim(self):
        if self.sim_proc is None:
            try:
                self.sim_proc = subprocess.Popen([
                    'python3', 'scripts/radar.py'
                ])
                self.sim_btn.setText("Simulateur en cours...")
                self.sim_btn.setEnabled(False)
            except Exception as e:
                print("Erreur lancement simulateur:", e)
    def cibler_depuis_ttm(self, ttm_sentence):
        if self.get_boat_info is None or self.send_can_ciblage is None:
            print("Ciblage indisponible : infos bateau manquantes.")
            return
        lat, lon, head = self.get_boat_info()
        if lat is None or lon is None or head is None:
            print("Ciblage indisponible : infos bateau manquantes.")
            return
        try:
            distance_m, angle, _ = parse_ttm(ttm_sentence)
            target_lat, target_lon = dest_point(lat, lon, angle, distance_m)
            rel_bearing = relative_bearing(lat, lon, target_lat, target_lon, head)
            self.send_can_ciblage(rel_bearing)
        except Exception as e:
            print(f"Erreur ciblage: {e}")
    def start_auto_cibler(self, ttm_sentence):
        self.locked_sentence = ttm_sentence
        self.auto_target_timer.start()
    def stop_auto_cibler(self, ttm_sentence):
        self.auto_target_timer.stop()
        self.locked_sentence = None
    def cibler_auto(self):
        if self.locked_sentence:
            self.cibler_depuis_ttm(self.locked_sentence) 