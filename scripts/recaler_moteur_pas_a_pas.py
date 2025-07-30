#!/home/gabin/venvs/pyside-env/bin/python
import serial
import time
import sys
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QFont

class MotorControlWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Contrôle Moteur Pas à Pas")
        self.setFixedSize(400, 300)
        
        # Configuration série
        self.PORT = "/dev/ttyUSB0"
        self.BAUDRATE = 9600
        self.ser = None
        
        # Position actuelle
        self.current_position = 0
        self.target_position = 0
        self.zero_offset = 0  # Offset pour le nouveau 0°
        
        # Timers pour répétition automatique
        self.left_timer = QTimer()
        self.left_timer.timeout.connect(self.decrement_position)
        self.left_timer.setInterval(100)  # 100ms entre chaque envoi
        
        self.right_timer = QTimer()
        self.right_timer.timeout.connect(self.increment_position)
        self.right_timer.setInterval(100)  # 100ms entre chaque envoi
        
        self.setup_ui()
        self.connect_serial()
        
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Groupe position
        pos_group = QGroupBox("Position du Moteur")
        pos_layout = QVBoxLayout(pos_group)
        
        self.position_label = QLabel("Position actuelle: 0°")
        self.position_label.setFont(QFont("Arial", 16, QFont.Bold))
        self.position_label.setAlignment(Qt.AlignCenter)
        pos_layout.addWidget(self.position_label)
        
        self.target_label = QLabel("Position cible: 0°")
        self.target_label.setFont(QFont("Arial", 12))
        self.target_label.setAlignment(Qt.AlignCenter)
        pos_layout.addWidget(self.target_label)
        
        layout.addWidget(pos_group)
        
        # Groupe contrôles
        control_group = QGroupBox("Contrôles")
        control_layout = QVBoxLayout(control_group)
        
        # Boutons directionnels
        btn_layout = QHBoxLayout()
        
        self.left_btn = QPushButton("← GAUCHE (-1°)")
        self.left_btn.setFont(QFont("Arial", 12, QFont.Bold))
        self.left_btn.setMinimumHeight(50)
        self.left_btn.pressed.connect(self.start_left)
        self.left_btn.released.connect(self.stop_left)
        btn_layout.addWidget(self.left_btn)
        
        self.right_btn = QPushButton("DROITE (+1°) →")
        self.right_btn.setFont(QFont("Arial", 12, QFont.Bold))
        self.right_btn.setMinimumHeight(50)
        self.right_btn.pressed.connect(self.start_right)
        self.right_btn.released.connect(self.stop_right)
        btn_layout.addWidget(self.right_btn)
        
        control_layout.addLayout(btn_layout)
        
        # Bouton reset
        self.reset_btn = QPushButton("RESET à 0°")
        self.reset_btn.setFont(QFont("Arial", 10))
        self.reset_btn.clicked.connect(self.reset_position)
        control_layout.addWidget(self.reset_btn)
        
        # Bouton set 0°
        self.set_zero_btn = QPushButton("SET 0° (Position actuelle)")
        self.set_zero_btn.setFont(QFont("Arial", 10))
        self.set_zero_btn.setStyleSheet("background: #ff6b35; color: white; font-weight: bold;")
        self.set_zero_btn.clicked.connect(self.set_current_as_zero)
        control_layout.addWidget(self.set_zero_btn)
        
        layout.addWidget(control_group)
        
        # Instructions
        instructions = QLabel("Utilisez les flèches ou cliquez sur les boutons.\nMaintenez appuyé pour un décalage continu.")
        instructions.setAlignment(Qt.AlignCenter)
        instructions.setStyleSheet("color: #666; font-style: italic;")
        layout.addWidget(instructions)
        
    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.PORT, self.BAUDRATE, timeout=1)
            time.sleep(2)  # Attendre l'initialisation
            print(f"[INFO] Connecté au port {self.PORT}")
        except Exception as e:
            print(f"[ERREUR] Connexion série: {e}")
            self.ser = None
    
    def send_position(self, position):
        if self.ser and self.ser.is_open:
            try:
                # Garder la position dans [0, 359] avec l'offset
                adjusted_position = (position + self.zero_offset) % 360
                self.ser.write(f"{adjusted_position}\n".encode())
                self.current_position = position
                self.position_label.setText(f"Position actuelle: {position}° (réel: {adjusted_position}°)")
                print(f"[INFO] Position envoyée: {adjusted_position}° (référence: {position}°)")
            except Exception as e:
                print(f"[ERREUR] Envoi position: {e}")
    
    def increment_position(self):
        self.target_position = (self.target_position + 1) % 360
        self.target_label.setText(f"Position cible: {self.target_position}°")
        self.send_position(self.target_position)
    
    def decrement_position(self):
        self.target_position = (self.target_position - 1) % 360
        self.target_label.setText(f"Position cible: {self.target_position}°")
        self.send_position(self.target_position)
    
    def set_current_as_zero(self):
        # Définir la position actuelle comme nouveau 0°
        self.zero_offset = (self.zero_offset + self.current_position) % 360
        self.target_position = 0
        self.current_position = 0
        self.target_label.setText("Position cible: 0°")
        self.position_label.setText("Position actuelle: 0° (nouveau 0° défini)")
        print(f"[INFO] Nouveau 0° défini. Offset total: {self.zero_offset}°")
        self.send_position(0)
    
    def reset_position(self):
        self.target_position = 0
        self.current_position = 0
        self.zero_offset = 0  # Remettre l'offset à 0
        self.target_label.setText("Position cible: 0°")
        self.position_label.setText("Position actuelle: 0°")
        self.send_position(0)
    
    def start_left(self):
        self.left_timer.start()
    
    def stop_left(self):
        self.left_timer.stop()
    
    def start_right(self):
        self.right_timer.start()
    
    def stop_right(self):
        self.right_timer.stop()
    
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Left:
            self.start_left()
        elif event.key() == Qt.Key_Right:
            self.start_right()
        elif event.key() == Qt.Key_0:
            self.reset_position()
    
    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Left:
            self.stop_left()
        elif event.key() == Qt.Key_Right:
            self.stop_right()
    
    def closeEvent(self, event):
        # Fermer la connexion série
        if self.ser and self.ser.is_open:
            try:
                # Envoyer la position actuelle avec l'offset au lieu de 0
                current_real_position = (self.current_position + self.zero_offset) % 360
                self.ser.write(f"{current_real_position}\n".encode())
                print(f"[INFO] Fermeture - Position finale: {current_real_position}° (référence: {self.current_position}°)")
                self.ser.close()
            except:
                pass
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = MotorControlWidget()
    widget.show()
    sys.exit(app.exec())
