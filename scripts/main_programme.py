#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import subprocess
import math

# Facteurs (° ↔ unité brute)
ELEV_FACTOR = 0.0573      # 1 unité ≈ 0.0573°
BEAR_FACTOR = 0.0573      # idem pour le bearing

# Décalage pour la zone négative (zéro expérimental ≈ 31 415)
NEG_OFFSET  = 31415       # valeur brute correspondant à ≈ 0°

def deg_to_elev_bytes(angle_deg: float) -> tuple[int, int]:
    if angle_deg >= 0:
        raw = int(round(angle_deg / ELEV_FACTOR))
    else:
        raw = int(round(angle_deg / ELEV_FACTOR + NEG_OFFSET))
    raw &= 0xFFFF  # UInt16
    return raw & 0xFF, (raw >> 8) & 0xFF

def deg_to_bear_bytes(angle_deg: float) -> tuple[int, int]:
    raw = int(round((360 - angle_deg) / BEAR_FACTOR)) & 0xFFFF
    return raw & 0xFF, (raw >> 8) & 0xFF

def calc_fov(distance, target_height=30, margin=1.2):
    field = target_height * margin
    fov_rad = 2 * math.atan(field / (2 * distance))
    return math.degrees(fov_rad)

fov_modes = [
    (5000, "NFOV 8x", 2.4, 8),   # Effective FOV:0.30°
    (4000, "NFOV 4x", 2.4, 4),   # Effective FOV:0.60°
    (3000, "NFOV 2x", 2.4, 2),   # Effective FOV:1.20°
    (2000, "NFOV",    2.4, 1),   # Effective FOV:2.40°
    (1000, "WFOV",    12,  1),   # Effective FOV:12.00°
    (0,    "VWFOV",   41,  1)    # Effective FOV:41.00°
]

root = tk.Tk()
root.title("VIGY - Slew to Position & FOV Calculator")

main_frame = ttk.Frame(root, padding=10)
main_frame.grid()

# === CAN Frame (angle_to_can) ===
can_frame = ttk.LabelFrame(main_frame, text="Envoi CAN (0x20C)", padding=20)
can_frame.grid(column=0, row=0, padx=10, pady=10, sticky="N")

ttk.Label(can_frame, text="Bearing (°) :").grid(column=0, row=0, sticky="W")
bearing_var = tk.StringVar(value="70")
ttk.Entry(can_frame, textvariable=bearing_var, width=10).grid(column=1, row=0)

ttk.Label(can_frame, text="Elevation (°) :").grid(column=0, row=1, sticky="W")
elev_var = tk.StringVar(value="-10")
ttk.Entry(can_frame, textvariable=elev_var, width=10).grid(column=1, row=1)

result_var = tk.StringVar()

def compute_can():
    try:
        bearing = float(bearing_var.get())
        elev    = float(elev_var.get())

        if elev < -30 or elev >= 70:
            result_var.set("Erreur : élévation hors plage [-30°, 70°).")
            return

        byte0 = 0x0C  # stabilisation + slewing deux axes

        elevation_lsb, elevation_msb = deg_to_elev_bytes(elev)
        bearing_lsb, bearing_msb = deg_to_bear_bytes(bearing)

        data_str = f"{byte0:02X}{elevation_lsb:02X}{elevation_msb:02X}{bearing_lsb:02X}{bearing_msb:02X}"
        cmd = f"cansend can0 20C#{data_str}"

        print(f"Envoi : {cmd}")
        ret = subprocess.call(cmd, shell=True)

        if ret != 0:
            result_var.set(f"Erreur d'envoi (code {ret})")
        else:
            trame_hex = f"{byte0:02X} {elevation_lsb:02X} {elevation_msb:02X} {bearing_lsb:02X} {bearing_msb:02X}"
            result_var.set(f"Trame envoyée : {trame_hex}")

    except ValueError:
        result_var.set("Entrée invalide : chiffres uniquement.")
    except Exception as e:
        result_var.set(f"Erreur inattendue : {e}")

ttk.Button(can_frame, text="Envoyer", command=compute_can).grid(column=0, row=2, columnspan=2, pady=10)
ttk.Label(can_frame, textvariable=result_var, justify="left", background="#eee", padding=10).grid(
    column=0, row=3, columnspan=2, sticky="W"
)

# === FOV Frame (fov_deter_interface) ===
fov_frame = ttk.LabelFrame(main_frame, text="Calculateur FOV", padding=20)
fov_frame.grid(column=1, row=0, padx=10, pady=10, sticky="N")

ttk.Label(fov_frame, text="Target distance (m):").grid(column=0, row=0, sticky="W")
distance_var = tk.StringVar()
ttk.Entry(fov_frame, textvariable=distance_var, width=20).grid(column=1, row=0)

fov_result_var = tk.StringVar()

def compute_fov():
    try:
        distance = float(distance_var.get())
        fov_calc = calc_fov(distance)

        widest_value, widest_name, widest_fov, widest_zoom = fov_modes[-1]
        if fov_calc > widest_fov:
            effective_fov = widest_fov / widest_zoom
            result = (
                f"Distance: {distance:.1f} m\n"
                f"Target Height: 30.0 m (by default)\n"
                f"Required FOV: {fov_calc:.2f}°\n\n"
                f"Recommended mode: {widest_name}\n"
                f"Value to send: {widest_value}\n"
                f"Optic FOV: {widest_fov:.2f}°\n"
                f"Numeric ZOOM: x{widest_zoom}\n"
                f"Effective FOV: {effective_fov:.2f}°"
            )
            fov_result_var.set(result)
            return

        for value, name, fov, zoom in fov_modes:
            effective_fov = fov / zoom
            if fov_calc <= effective_fov:
                result = (
                    f"Distance: {distance:.1f} m\n"
                    f"Target Height: 30.0 m (by default)\n"
                    f"Required FOV: {fov_calc:.2f}°\n\n"
                    f"Recommended mode: {name}\n"
                    f"Value to send: {value}\n"
                    f"Optic FOV: {fov:.2f}°\n"
                    f"Numeric ZOOM: x{zoom}\n"
                    f"Effective FOV: {effective_fov:.2f}°"
                )
                fov_result_var.set(result)
                return

        value, name, fov, zoom = fov_modes[0]
        effective_fov = fov / zoom
        result = (
            f"Distance: {distance:.1f} m\n"
            f"Target Height: 30.0 m (by default)\n"
            f"Required FOV: {fov_calc:.2f}°\n\n"
            f"WARNING: Target will not fully fit!\n"
            f"Max zoom mode used: {name}\n"
            f"Value to send: {value}\n"
            f"Optic FOV: {fov:.2f}°\n"
            f"Numeric ZOOM: x{zoom}\n"
            f"Effective FOV: {effective_fov:.2f}°"
        )
        fov_result_var.set(result)
    except Exception as e:
        fov_result_var.set("Error: invalid input.")

ttk.Button(fov_frame, text="Calculer", command=compute_fov).grid(column=0, row=1, columnspan=2, pady=10)
ttk.Label(fov_frame, textvariable=fov_result_var, justify="left", background="#eee", padding=10).grid(
    column=0, row=2, columnspan=2, sticky="W"
)

root.mainloop()