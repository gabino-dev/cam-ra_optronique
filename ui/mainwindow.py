from PySide6.QtWidgets import QWidget, QHBoxLayout, QListWidget, QListWidgetItem, QStackedWidget, QFrame
from PySide6.QtGui import QIcon
from PySide6.QtCore import Qt
from .home import HomePage
from .radar import RadarPage
from .gallery import GalleryPage
from .settings import SettingsPage
from .about import AboutPage
from .logic import NMEAReader, CANReader, ELEV_FACTOR, BEAR_FACTOR, parse_ttm, dest_point, relative_bearing
import subprocess
import time

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VIGY - CAMERA CONTROL - Couach_2025")
        self.resize(1700, 1050)
        self.setMinimumWidth(1500)
        self.setMinimumHeight(900)

        self.sidebar = QListWidget()
        self.sidebar.setFixedWidth(180)
        self.sidebar.setFrameShape(QFrame.NoFrame)
        for name in ["Accueil", "Radar", "Galerie", "Réglages", "À propos"]:
            item = QListWidgetItem(QIcon(), name)
            self.sidebar.addItem(item)

        # Threads partagés
        self.nmea_thread = NMEAReader(port='/dev/pts/3')
        self.can_thread = CANReader()
        self.nmea_thread.start()
        self.can_thread.start()

        self.pages = QStackedWidget()
        self.home_page = HomePage(self.can_thread, self.nmea_thread)
        self.radar_page = RadarPage(
            self.nmea_thread,
            get_boat_info=self.get_boat_info,
            send_can_ciblage=self.send_can_ciblage
        )
        self.can_thread.update_data.connect(self.radar_page.on_position)
        self.pages.addWidget(self.home_page)
        self.pages.addWidget(self.radar_page)
        self.pages.addWidget(GalleryPage())
        self.pages.addWidget(SettingsPage())
        self.pages.addWidget(AboutPage())

        self.sidebar.currentRowChanged.connect(self.pages.setCurrentIndex)
        self.sidebar.setCurrentRow(0)

        main_layout = QHBoxLayout(self)
        main_layout.addWidget(self.sidebar)
        main_layout.addWidget(self.pages, stretch=1)
        self.setLayout(main_layout)

    def get_boat_info(self):
        # Retourne (lat, lon, head) en float, ou (None, None, None)
        try:
            lat = self.home_page.last_lat_deg
            lon = self.home_page.last_lon_deg
            head = self.home_page.last_head
            return lat, lon, head
        except Exception:
            return None, None, None

    def send_can_ciblage(self, rel_bearing):
        # Envoie la trame CAN de ciblage à l'angle rel_bearing
        try:
            lsb_e, msb_e = self.home_page.deg_to_bytes(0.0, ELEV_FACTOR)
            lsb_b, msb_b = self.home_page.deg_to_bytes(360 - rel_bearing, BEAR_FACTOR)
            data = f"0C{lsb_e:02X}{msb_e:02X}{lsb_b:02X}{msb_b:02X}"
            subprocess.call(f"cansend can0 20C#{data}", shell=True)
            print(f"Ciblage CAN envoyé: {rel_bearing:.2f}° (data={data})")
        except Exception as e:
            print(f"Erreur ciblage CAN: {e}")

    def closeEvent(self, event):
        # Slew to position 0,0 (CAN)
        try:
            lsb_e, msb_e = self.home_page.deg_to_bytes(0.0, ELEV_FACTOR)
            lsb_b, msb_b = self.home_page.deg_to_bytes(360.0, BEAR_FACTOR)
            data = f"0C{lsb_e:02X}{msb_e:02X}{lsb_b:02X}{msb_b:02X}"
            subprocess.call(f"cansend can0 20C#{data}", shell=True)
        except Exception as e:
            print("Erreur slew to 0/0:", e)
        # Arduino moteur pas à pas à 0
        if hasattr(self.home_page, 'arduino') and self.home_page.arduino and self.home_page.arduino.is_open:
            try:
                self.home_page.arduino.write(b"0\n")
                self.home_page.arduino.close()
            except Exception as e:
                print("Erreur Arduino close:", e)
        # Arrêt threads
        self.can_thread.stop()
        self.nmea_thread.stop()
        event.accept() 