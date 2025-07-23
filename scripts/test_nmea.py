import serial
import time

try:
    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=4800,
        timeout=1,
        dsrdtr=False,   # désactive DSR/DTR
        rtscts=False    # désactive RTS/CTS
    )
except Exception as e:
    print(f"[ERREUR] Impossible d'ouvrir le port série : {e}")
    exit(1)

while True:
    hdt = "$GPHDT,123.4,T*32\r\n"
    
    ser.write(hdt.encode())
    
    print("Trames envoyées.")
    time.sleep(1)
