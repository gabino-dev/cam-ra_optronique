#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import can 
import struct
import math

# === Fenêtre Tkinter ===
root = tk.Tk()
root.title("VIGY - Position actuelle (0x105)")

frame = ttk.Frame(root, padding=20)
frame.grid()

elevation_var = tk.StringVar(value="Élévation : -- °")
bearing_var = tk.StringVar(value="Azimut : -- °")

ttk.Label(frame, textvariable=elevation_var, font=("Helvetica", 16)).grid(row=0, column=0, sticky="W", pady=5)
ttk.Label(frame, textvariable=bearing_var, font=("Helvetica", 16)).grid(row=1, column=0, sticky="W", pady=5)

# === Interface CAN ===
can_interface = 'can0'
bus = can.interface.Bus(channel=can_interface, interface='socketcan')

def update_position():
    try:
        msg = bus.recv(timeout=0.01)  # max 40 ms d'attente
        if msg and msg.arbitration_id == 0x105 and len(msg.data) >= 8:
            elev_rad = struct.unpack('<f', msg.data[0:4])[0]
            bear_rad = struct.unpack('<f', msg.data[4:8])[0]

            elev_deg = math.degrees(elev_rad)
            bear_deg = math.degrees(bear_rad)

            # Correction pour afficher les angles dans [-180°, +180°[
            if elev_deg >= 180:
                elev_deg -= 360
            if bear_deg >= 180:
                bear_deg -= 360

            elevation_var.set(f"Élévation : {elev_deg:.2f} °")
            bearing_var.set(f"Azimut : {bear_deg:.2f} °")
    except Exception as e:
        print("Erreur lecture CAN :", e)

    def on_close():

        try:
            bus.shutdown()  # méthode python-can
        except Exception as e:
            print(f"Erreur à la fermeture du bus : {e}")
        root.destroy()

        # Associer la fermeture de la fenêtre à on_close()
        root.protocol("WM_DELETE_WINDOW", on_close)


    # Replanifier appel dans 40 ms
    root.after(10, update_position)

# Lancement boucle