import time

# Chemin vers le fichier .nmea
nmea_file = "/home/invite/Desktop/python/cam-ra_optronique/scripts/mydata.nmea"
# Le port vers lequel on simule l'envoi (ex: /dev/pts/5)
gps_port_path = "/dev/pts/3"

with open(nmea_file, "r") as file:
    lines = file.readlines()

while True:
    for line in lines:
        with open(gps_port_path, "w") as gps_port:
            gps_port.write(line)
        print(f"Sent: {line.strip()}")
        time.sleep(1)  # Une ligne NMEA par seconde (comme un vrai GPS)
