import tkinter as tk
from tkinter import ttk
import math

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

def compute():
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
            result_var.set(result)
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
                result_var.set(result)
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
        result_var.set(result)
    except Exception as e:
        result_var.set("Error: invalid input.")


root = tk.Tk()
root.title("FOV Calculator")

frame = ttk.Frame(root, padding=20)
frame.grid()

ttk.Label(frame, text="Target distance (m):").grid(column=0, row=0, sticky="W")
distance_var = tk.StringVar()
ttk.Entry(frame, textvariable=distance_var, width=20).grid(column=1, row=0)

ttk.Button(frame, text="compute", command=compute).grid(column=0, row=1, columnspan=2, pady=10)

result_var = tk.StringVar()
result_label = ttk.Label(frame, textvariable=result_var, justify="left", background="#eee", padding=10)
result_label.grid(column=0, row=2, columnspan=2, sticky="W")

root.mainloop()
