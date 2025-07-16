import tkinter as tk
from tkinter import ttk
import subprocess
import math
import can
import struct
import threading

# Facteurs (° ↔ unité brute)
ELEV_FACTOR = 0.0573
BEAR_FACTOR = 0.0573
NEG_OFFSET  = 31415.5

arrow_symbols = ['↑', '↗', '→', '↘', '↓', '↙', '←', '↖']


def deg_to_elev_bytes(angle_deg: float):
    if angle_deg >= 0:
        raw = int(round(angle_deg / ELEV_FACTOR))
    else:
        raw = int(round(angle_deg / ELEV_FACTOR + NEG_OFFSET))
    raw &= 0xFFFF
    return raw & 0xFF, (raw >> 8) & 0xFF

def deg_to_bear_bytes(angle_deg: float):
    raw = int(round((360 - angle_deg) / BEAR_FACTOR)) & 0xFFFF
    return raw & 0xFF, (raw >> 8) & 0xFF

def calc_fov(distance, target_height=30, margin=1.2):
    field = target_height * margin
    fov_rad = 2 * math.atan(field / (2 * distance))
    return math.degrees(fov_rad)

fov_modes = [
    (5000, "NFOV 8x", 2.4, 8),
    (4000, "NFOV 4x", 2.4, 4),
    (3000, "NFOV 2x", 2.4, 2),
    (2000, "NFOV",    2.4, 1),
    (1000, "WFOV",    12,  1),
    (0,    "VWFOV",   41,  1)
]

root = tk.Tk()
root.title("VIGY - Slew to Position, FOV & Live Position")
main_frame = ttk.Frame(root, padding=10)
main_frame.grid()

# === CAN Frame (angle_to_can) ===
can_frame = ttk.LabelFrame(main_frame, text="Envoi CAN (0x20C)", padding=20)
can_frame.grid(column=0, row=0, padx=10, pady=10, sticky="N")

bearing_var = tk.StringVar(value="70")
elev_var = tk.StringVar(value="-10")
result_var = tk.StringVar()
indicator_var = tk.StringVar(value="Cap visé : --°")
arrow_var = tk.StringVar(value="")

indicator_label = ttk.Label(can_frame, textvariable=indicator_var, font=("Courier", 14), foreground="red")
indicator_label.grid(column=0, row=4, columnspan=2, pady=(10, 0))

direction_label = ttk.Label(can_frame, textvariable=arrow_var, font=("Helvetica", 30))
direction_label.grid(column=0, row=5, columnspan=2)

ttk.Label(can_frame, text="Bearing (°) :").grid(column=0, row=0, sticky="W")
ttk.Entry(can_frame, textvariable=bearing_var, width=10).grid(column=1, row=0)

ttk.Label(can_frame, text="Elevation (°) :").grid(column=0, row=1, sticky="W")
ttk.Entry(can_frame, textvariable=elev_var, width=10).grid(column=1, row=1)

def compute_can():
    try:
        bearing = float(bearing_var.get())
        elev    = float(elev_var.get())
        if elev < -30 or elev >= 70:
            result_var.set("Erreur : élévation hors plage [-30°, 70°).")
            return
        byte0 = 0x0C
        elev_lsb, elev_msb = deg_to_elev_bytes(elev)
        bear_lsb, bear_msb = deg_to_bear_bytes(bearing)
        data_str = f"{byte0:02X}{elev_lsb:02X}{elev_msb:02X}{bear_lsb:02X}{bear_msb:02X}"
        cmd = f"cansend can0 20C#{data_str}"
        subprocess.call(cmd, shell=True)
        result_var.set(f"Trame envoyée : {data_str}")
    except Exception as e:
        result_var.set(f"Erreur : {e}")

ttk.Button(can_frame, text="Envoyer", command=compute_can).grid(column=0, row=2, columnspan=2, pady=10)
ttk.Label(can_frame, textvariable=result_var, background="#eee", padding=10).grid(column=0, row=3, columnspan=2, sticky="W")

# === FOV Frame ===
fov_frame = ttk.LabelFrame(main_frame, text="Calculateur FOV", padding=20)
fov_frame.grid(column=1, row=0, padx=10, pady=10, sticky="N")

distance_var = tk.StringVar()
fov_result_var = tk.StringVar()

ttk.Label(fov_frame, text="Target distance (m):").grid(column=0, row=0, sticky="W")
ttk.Entry(fov_frame, textvariable=distance_var, width=20).grid(column=1, row=0)

def compute_fov():
    try:
        distance = float(distance_var.get())
        fov_calc = calc_fov(distance)
        for value, name, fov, zoom in fov_modes:
            effective_fov = fov / zoom
            if fov_calc <= effective_fov:
                fov_result_var.set(f"Mode: {name}, Value: {value}, FOV: {effective_fov:.2f}°")
                return
        fov_result_var.set("Cible trop grande, utilisez VWFOV")
    except:
        fov_result_var.set("Entrée invalide.")

ttk.Button(fov_frame, text="Calculer", command=compute_fov).grid(column=0, row=1, columnspan=2, pady=10)
ttk.Label(fov_frame, textvariable=fov_result_var, background="#eee", padding=10).grid(column=0, row=2, columnspan=2, sticky="W")


def send_unlock():
    try:
        cmd = "cansend can0 205#00"
        subprocess.call(cmd, shell=True)
        result_var.set("Trame UNLOCK envoyée (205#00)")
    except Exception as e:
        result_var.set(f"Erreur UNLOCK : {e}")

ttk.Button(can_frame, text="Unlock", command=send_unlock).grid(column=0, row=6, columnspan=2, pady=10)
       
# === LIVE POSITION Frame ===
live_frame = ttk.LabelFrame(main_frame, text="Position actuelle (0x105)", padding=20)
live_frame.grid(column=2, row=0, padx=10, pady=10, sticky="N")

elevation_live_var = tk.StringVar(value="Élévation : -- °")
bearing_live_var = tk.StringVar(value="Bearing : -- °")

label_elev = ttk.Label(live_frame, textvariable=elevation_live_var, font=("Helvetica", 16))
label_bear = ttk.Label(live_frame, textvariable=bearing_live_var, font=("Helvetica", 16))
label_elev.grid(row=0, column=0, sticky="W", pady=5)
label_bear.grid(row=1, column=0, sticky="W", pady=5)

# === Interface CAN ===
can_interface = 'can0'
bus = can.interface.Bus(channel=can_interface, interface='socketcan')

stop_thread = False

def get_arrow_from_bearing(deg):
    idx = int(((deg % 360) + 22.5) // 45) % 8
    return arrow_symbols[idx]

def can_loop():
    global stop_thread
    while not stop_thread:
        try:
            msg = bus.recv(timeout=0.01)
            if msg and msg.arbitration_id == 0x105 and len(msg.data) >= 8:
                elev_rad = struct.unpack('<f', msg.data[0:4])[0]
                bear_rad = struct.unpack('<f', msg.data[4:8])[0]
                elev_deg = math.degrees(elev_rad)
                bear_deg = math.degrees(bear_rad)

                # Affichage dans l'intervalle [-180, 180[
                if elev_deg >= 180:
                    elev_deg -= 360

                elevation_live_var.set(f"Élévation : {elev_deg:.2f} °")
                bearing_live_var.set(f"Bearing : {bear_deg:.2f} °")
                indicator_var.set(f"Cap visé : {bear_deg:.2f} °")
                arrow_var.set(get_arrow_from_bearing(bear_deg))
        except Exception as e:
            print("Erreur lecture CAN :", e)

def on_close():
    global stop_thread
    stop_thread = True
    root.after(100, shutdown_bus)

def shutdown_bus():
    try:
        bus.shutdown()
        print("Bus CAN fermé proprement.")
    except Exception as e:
        print(f"Erreur à la fermeture du bus : {e}")
    root.destroy()

threading.Thread(target=can_loop, daemon=True).start()
root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
