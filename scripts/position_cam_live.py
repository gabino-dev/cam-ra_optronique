#!/home/gabin/venvs/pyside-env/bin/python
#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import can
import struct
import math
import threading

# === Variables globales ===
stop_thread = False

# === Fenêtre Tkinter ===
root = tk.Tk()
root.title("VIGY - Position actuelle (0x105)")

frame = ttk.Frame(root, padding=20)
frame.grid()

elevation_var = tk.StringVar(value="Élévation : -- °")
bearing_var = tk.StringVar(value="Bearing : -- °")

ttk.Label(frame, textvariable=elevation_var, font=("Helvetica", 16)).grid(row=0, column=0, sticky="W", pady=5)
ttk.Label(frame, textvariable=bearing_var, font=("Helvetica", 16)).grid(row=1, column=0, sticky="W", pady=5)

# === Interface CAN ===
can_interface = 'can0'
bus = can.interface.Bus(channel=can_interface, bustype='socketcan')

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

                elevation_var.set(f"Élévation : {elev_deg:.2f} °")
                bearing_var.set(f"Bearing : {bear_deg:.2f} °")
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

# === Lancer thread CAN séparé ===
threading.Thread(target=can_loop, daemon=True).start()

# Gestion fermeture propre
root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
