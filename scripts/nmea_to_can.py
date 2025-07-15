import time
import subprocess

nmea_file = "/home/invite/Desktop/python/cam-ra_optronique/scripts/mydata.nmea"
can_iface = "can0"
can_id = "220"  # ID CAN en hexadécimal (0x220)

FACTOR = 10000  # pour encoder les minutes

def parse_gpgga(line):
    if not line.startswith("$GPGGA"):
        return None
    parts = line.split(",")
    try:
        lat_raw = parts[2]
        lat_dir = parts[3]
        lon_raw = parts[4]
        lon_dir = parts[5]

        lat_deg = int(lat_raw[:2])
        lat_min = float(lat_raw[2:])
        lon_deg = int(lon_raw[:3])
        lon_min = float(lon_raw[3:])

        return (lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir)
    except:
        return None

def encode_to_hex(lat_d, lat_m, lat_dir, lon_d, lon_m, lon_dir):
    lat_m_enc = int(lat_m * FACTOR)
    lon_m_enc = int(lon_m * FACTOR)

    ns = "00" if lat_dir == "N" else "01"
    ew = "00" if lon_dir == "E" else "01"

    data = [
        f"{lat_d:02X}",
        f"{(lat_m_enc >> 8) & 0xFF:02X}",
        f"{lat_m_enc & 0xFF:02X}",
        ns,
        f"{lon_d:02X}",
        f"{(lon_m_enc >> 8) & 0xFF:02X}",
        f"{lon_m_enc & 0xFF:02X}",
        ew
    ]
    return "".join(data)

# Boucle infinie
while True:
    with open(nmea_file, "r") as f:
        for line in f:
            data = parse_gpgga(line)
            if data:
                hex_payload = encode_to_hex(*data)
                cmd = f"cansend {can_iface} {can_id}#{hex_payload}"
                print(">>", cmd)
                subprocess.run(cmd, shell=True)
                time.sleep(1)
