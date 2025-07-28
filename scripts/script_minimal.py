#!/home/gabin/venvs/pyside-env/bin/python
import serial
import time

try:
    print("[INFO] Connexion au port /dev/ttyUSB0...")
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(2)  # attendre l'initialisation de l'Arduino
    print("[OK] Connecté.")

    # Test d'envoi d'un bearing à 90°
    bearing = 90
    message = f"{bearing}\n"
    print(f"[DEBUG] Envoi vers Arduino : {message.strip()}")
    ser.write(message.encode())

    ser.close()
    print("[FIN] Test terminé.")

except Exception as e:
    print(f"[ERREUR] Impossible d'envoyer la donnée : {e}")
