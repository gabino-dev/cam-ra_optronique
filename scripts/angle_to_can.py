import tkinter as tk
from tkinter import ttk

ELEV_FACTOR = 0.0573      
BEAR_FACTOR = 0.0573      

def deg_to_elev_bytes(angle_deg: float) -> tuple[int, int]:
    raw = int(round(angle_deg / ELEV_FACTOR))  # élévation en raw int16
    raw &= 0xFFFF  # on garde le format binaire sur 2 octets
    return raw & 0xFF, (raw >> 8) & 0xFF


def deg_to_bear_bytes(angle_deg: float) -> tuple[int, int]:
    raw = int(round((360 - angle_deg) / BEAR_FACTOR)) & 0xFFFF
    return raw & 0xFF, (raw >> 8) & 0xFF

# === Tkinter UI ===
root = tk.Tk()
root.title("VIGY - Slew to Position (CAN 0x20C)")

frm = ttk.Frame(root, padding=20)
frm.grid()

ttk.Label(frm, text="Bearing (°) :").grid(column=0, row=0, sticky="W")
bearing_var = tk.StringVar(value="70")
ttk.Entry(frm, textvariable=bearing_var, width=10).grid(column=1, row=0)

ttk.Label(frm, text="Elevation (°) :").grid(column=0, row=1, sticky="W")
elev_var = tk.StringVar(value="-10")
ttk.Entry(frm, textvariable=elev_var, width=10).grid(column=1, row=1)

result_var = tk.StringVar()

def compute():
    try:
        bearing = float(bearing_var.get())
        elev    = float(elev_var.get())

        if not (-69.9 <= elev <= 69.9):
            result_var.set("Erreur : élévation hors plage [-69.9°, 69.9°].")
            return

        byte0 = 0x0C

        el_lsb, el_msb = deg_to_elev_bytes(elev)
        be_lsb, be_msb = deg_to_bear_bytes(bearing)

        trame_hex = f"{byte0:02X} {el_lsb:02X} {el_msb:02X} {be_lsb:02X} {be_msb:02X}"
        cansend   = f"cansend can0 20C#{byte0:02X}{el_lsb:02X}{el_msb:02X}{be_lsb:02X}{be_msb:02X}"

        result_var.set(
            f"Trame CAN  (bytes) : {trame_hex}\n"
            f"Commande    cansend : {cansend}"
        )
    except ValueError:
        result_var.set("Entrée invalide : chiffres uniquement.")

ttk.Button(frm, text="Calculer", command=compute).grid(column=0, row=2, columnspan=2, pady=10)
ttk.Label(frm, textvariable=result_var, justify="left", background="#eee", padding=10).grid(
    column=0, row=3, columnspan=2, sticky="W"
)

root.mainloop()
