import serial

ser = serial.Serial('/dev/pts/7', baudrate=4800, timeout=1)

print("[INFO] Lecture des trames NMEA sur /dev/pts/7...")
try:
    while True:
        if ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            if line.startswith('$'):
                print("Trame NMEA :", line)
except KeyboardInterrupt:
    ser.close()
    print("\n[INFO] Lecture arrêtée.")
