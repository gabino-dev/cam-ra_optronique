import math
import struct
import socket
import time
import serial
import can
import pynmea2
from datetime import datetime
from PySide6.QtCore import QThread, Signal

ELEV_FACTOR = 0.0573
BEAR_FACTOR = 0.0573
NEG_OFFSET = 31415.5
FOV_MAP = {
    0x04: 'VWFoV',
    0x08: 'WFoV',
    0x0C: 'NFoV'
}

def parse_ttm(ttm_sentence):
    fields = ttm_sentence.strip().split(',')
    if len(fields) < 5 or not fields[0].endswith('TTM'):
        raise ValueError("Phrase NMEA n'est pas de type TTM")
    distance = float(fields[1])
    distance_unit = fields[2]
    angle = float(fields[3])
    if distance_unit == 'N':
        distance_m = distance * 1852
    elif distance_unit == 'K':
        distance_m = distance * 1000
    else:
        distance_m = distance
    return distance_m, angle, fields[7] if len(fields) > 7 else "Target"

def dest_point(lat1, lon1, bearing_deg, distance_m):
    R = 6371000
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
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)
    x = math.sin(delta_lon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
    bearing = (math.degrees(math.atan2(x, y)) + 360) % 360
    return bearing

def relative_bearing(camera_lat, camera_lon, target_lat, target_lon, ship_heading):
    bearing_abs = calculate_bearing(camera_lat, camera_lon, target_lat, target_lon)
    rel_bearing = (bearing_abs - ship_heading + 360) % 360
    return rel_bearing

def send_camera_as_cog_nmea(camera_bearing, ship_heading, udp_port=10110):
    cog = (ship_heading + camera_bearing) % 360
    sog = 4.2
    now = time.strftime("%H%M%S", time.gmtime())
    date = time.strftime("%d%m%y", time.gmtime())
    def nmea_checksum(sentence):
        cs = 0
        for c in sentence:
            cs ^= ord(c)
        return f"{cs:02X}"
    gprmc_body = f"GPRMC,{now}.000,A,3459.715914,S,13830.130353,E,{sog:.1f},{cog:.1f},{date},,,A"
    csum = nmea_checksum(gprmc_body)
    gprmc = f"${gprmc_body}*{csum}\r\n"
    gpvtg_body = f"GPVTG,{cog:.1f},T,{cog:.1f},M,{sog:.1f},N,7.8,K"
    csum2 = nmea_checksum(gpvtg_body)
    gpvtg = f"${gpvtg_body}*{csum2}\r\n"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(gprmc.encode(), ('127.0.0.1', udp_port))
    sock.sendto(gpvtg.encode(), ('127.0.0.1', udp_port))
    sock.close()

class NMEAReader(QThread):
    nmea_update = Signal(str, str, float)
    ttm_trace = Signal(str)
    def __init__(self, port='/dev/pts/3', baud=4800):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = True
        self.sock = None
    def connect_tcp(self):
        while self.running and self.sock is None:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect(("127.0.0.1", 10110))
                self.sock = s
            except Exception:
                time.sleep(2)
    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
        except Exception:
            return
        self.connect_tcp()
        while self.running:
            try:
                line = ser.readline().decode(errors='ignore').strip()
                if not line.startswith('$'):
                    continue
                if self.sock:
                    try:
                        self.sock.sendall((line + "\r\n").encode())
                    except Exception:
                        try:
                            self.sock.close()
                        except:
                            pass
                        self.sock = None
                        self.connect_tcp()
                if line.startswith('$TTM') or line.startswith('$GPTTM'):
                    self.ttm_trace.emit(line)
                try:
                    msg = pynmea2.parse(line)
                    if isinstance(msg, pynmea2.RMC):
                        lat = f"{msg.latitude:.5f}° {msg.lat_dir}"
                        lon = f"{msg.longitude:.5f}° {msg.lon_dir}"
                        heading = float(msg.true_course) if msg.true_course else 0.0
                        self.nmea_update.emit(lat, lon, heading)
                except pynmea2.ParseError:
                    continue
            except Exception:
                continue
    def stop(self):
        self.running = False
        self.wait()
        if self.sock:
            try:
                self.sock.close()
            except:
                pass

class CANReader(QThread):
    update_data = Signal(float, float)
    update_digital_zoom = Signal(int)
    update_lrf = Signal(int)
    def __init__(self):
        super().__init__()
        self.running = True
        self.bus = can.interface.Bus(channel='can0', interface='socketcan')
    def run(self):
        while self.running:
            msg = self.bus.recv(timeout=0.01)
            if not msg:
                continue
            if msg.arbitration_id == 0x105 and len(msg.data) >= 8:
                elev = math.degrees(struct.unpack('<f', msg.data[0:4])[0])
                bear = math.degrees(struct.unpack('<f', msg.data[4:8])[0])
                if elev >= 180:
                    elev -= 360
                self.update_data.emit(elev, bear)
            elif msg.arbitration_id == 0x101 and len(msg.data) >= 3:
                raw = msg.data[1] | (msg.data[2] << 8)
                self.update_digital_zoom.emit(raw)
            elif msg.arbitration_id == 0x107 and len(msg.data) >= 2:
                dist_m = msg.data[0] | (msg.data[1] << 8)
                self.update_lrf.emit(dist_m)
    def stop(self):
        self.running = False
        self.quit()
        self.wait() 