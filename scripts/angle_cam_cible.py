#!/home/gabin/venvs/pyside-env/bin/python
import math
import serial


def parse_ttm(ttm_sentence):
    """
    Parse une phrase NMEA $TTM et retourne (distance_m, angle_deg)
    """
    fields = ttm_sentence.strip().split(',')
    if len(fields) < 5 or fields[0][-3:] != 'TTM':
        raise ValueError("Phrase NMEA n'est pas de type TTM")
    distance = float(fields[1])
    distance_unit = fields[2]
    angle = float(fields[3])
    # Conversion de la distance en mètres
    if distance_unit == 'N':
        distance_m = distance * 1852
    elif distance_unit == 'K':
        distance_m = distance * 1000
    else:
        distance_m = distance  # en mètres si pas d'unité
    return distance_m, angle

def dest_point(lat1, lon1, bearing_deg, distance_m):
    """
    Calcule la latitude et longitude d'un point à partir d'une position de départ
    """
    R = 6371000  # Rayon moyen de la Terre en mètres
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    bearing_rad = math.radians(bearing_deg)
    d_div_r = distance_m / R

    lat2_rad = math.asin(
        math.sin(lat1_rad) * math.cos(d_div_r) +
        math.cos(lat1_rad) * math.sin(d_div_r) * math.cos(bearing_rad)
    )
    lon2_rad = lon1_rad + math.atan2(
        math.sin(bearing_rad) * math.sin(d_div_r) * math.cos(lat1_rad),
        math.cos(d_div_r) - math.sin(lat1_rad) * math.sin(lat2_rad)
    )
    return math.degrees(lat2_rad), math.degrees(lon2_rad)

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calcule le bearing (azimut vrai) entre deux points GPS
    """
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)

    x = math.sin(delta_lon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
    bearing = (math.degrees(math.atan2(x, y)) + 360) % 360
    return bearing

def relative_bearing(camera_lat, camera_lon, target_lat, target_lon, ship_heading):
    """
    Bearing relatif à l'axe du bateau
    """
    bearing_abs = calculate_bearing(camera_lat, camera_lon, target_lat, target_lon)
    rel_bearing = (bearing_abs - ship_heading + 360) % 360
    return rel_bearing

if __name__ == "__main__":
    camera_lat = 48.1173  # 48°07.038'N
    camera_lon = 11.5167  # 11°31.000'E
    ship_heading = 74  # Cap du bateau en degrés

    # Ouverture du port série virtuel
    ser = serial.Serial('/dev/pts/7', baudrate=4800, timeout=1)
    print("Lecture des trames TTM sur /dev/pts/7 ...")

    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if line.startswith('$TTM'):
            try:
                distance_m, angle = parse_ttm(line)
                target_lat, target_lon = dest_point(camera_lat, camera_lon, angle, distance_m)
                rel_bearing = relative_bearing(camera_lat, camera_lon, target_lat, target_lon, ship_heading)
                print(f"TTM: {line}")
                print(f"Coordonnées cible : {target_lat:.6f}, {target_lon:.6f}")
                print(f"Bearing relatif caméra : {rel_bearing:.2f}°\n")
            except Exception as e:
                print(f"Erreur de parsing : {e}")
