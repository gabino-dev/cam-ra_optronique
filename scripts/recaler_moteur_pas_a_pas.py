#!/home/gabin/venvs/pyside-env/bin/python
import serial
import time

# Port série de l’Arduino
PORT = "/dev/ttyUSB0"
BAUDRATE = 9600

# Dernière position connue (tu peux aussi la lire depuis un fichier si besoin)


# Nouvelle position : on décale de -1°, et on reste dans [0, 359]
new_position = 2

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(2)  # Attendre l'initialisation d'Arduino
    ser.write(f"{new_position}\n".encode())
    print(f"[INFO] Position envoyée : {new_position}°")
    ser.close()
except Exception as e:
    print(f"[ERREUR] {e}")
