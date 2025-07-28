#!/home/gabin/venvs/pyside-env/bin/python
import tkinter as tk
from tkinter import ttk
import struct

def decode_can_data():
    try:
        input_str = entry.get()
        if "Data=" not in input_str:
            result_var.set("Format invalide. Exemple : ID=..., Data=8C7AC64036BE4940")
            return

        hex_data = input_str.split("Data=")[-1].strip()
        if len(hex_data) != 16:
            result_var.set("La section Data doit contenir exactement 8 octets (16 caractères hexadécimaux).")
            return

        raw_bytes = bytes.fromhex(hex_data)

        # [0:3] = elevation, [4:7] = bearing
        elevation_rad, bearing_rad = struct.unpack("<ff", raw_bytes)

        elevation_deg = (elevation_rad * 180) / 3.14159265
        bearing_deg = (bearing_rad * 180) / 3.14159265

        # error elevation corrected
        if elevation_deg > 180:
            elevation_deg -= 360

        result = (
            f"Elevation : {elevation_deg:.2f}° ({elevation_rad:.4f} rad)\n"
            f"Bearing   : {bearing_deg:.2f}° ({bearing_rad:.4f} rad)"
        )
        result_var.set(result)

    except Exception as e:
        result_var.set(f"Erreur : {e}")

root = tk.Tk()
root.title("CAN LOS Decoder")

mainframe = ttk.Frame(root, padding="10")
mainframe.grid()

ttk.Label(mainframe, text="Trame CAN (ex : ID=...,Data=XXXXXXXXXXXXXXX):").grid(column=0, row=0, sticky="W")
entry = ttk.Entry(mainframe, width=50)
entry.grid(column=0, row=1, sticky="W")


ttk.Button(mainframe, text="Decode", command=decode_can_data).grid(column=0, row=2, pady=10, sticky="W")

result_var = tk.StringVar()
ttk.Label(mainframe, textvariable=result_var, justify="left").grid(column=0, row=3, sticky="W")

root.mainloop()
