#!/home/gabin/venvs/pyside-env/bin/python
import tkinter as tk
from tkinter import ttk
import math

def horizon_distance(height_m):
    """Returns the horizon distance in km from a given height in meters."""
    return 3.86 * math.sqrt(height_m)

def calculate():
    try:
        h_camera = float(camera_height_var.get())
        h_target = float(target_height_var.get())

        d_cam = horizon_distance(h_camera)
        d_top = horizon_distance(h_target)

        d_total = d_cam + d_top
        partial_min = d_cam
        partial_max = d_cam + d_top

        result = (
            f"RESULTS:\n"
            f"- Horizon distance (camera): {d_cam:.1f} km\n\n"
            
    
            f"- Partial visibility zone: {partial_min:.1f} km to {partial_max:.1f} km\n"
            f"  > Target visibility is reducing.\n\n"
            f"- Full visibility (entire target visible): {d_cam:.1f} km or less\n"
            f"  > Entire target is fully visible within this range."
        )
        result_var.set(result)

    except Exception as e:
        result_var.set(f"Error: {e}")

# GUI setup
root = tk.Tk()
root.title("Horizon Visibility Calculator")

mainframe = ttk.Frame(root, padding=20)
mainframe.grid()

ttk.Label(mainframe, text="Camera height (in meters):").grid(column=0, row=0, sticky="W")
camera_height_var = tk.StringVar()
ttk.Entry(mainframe, textvariable=camera_height_var, width=20).grid(column=1, row=0)

ttk.Label(mainframe, text="Target height (in meters):").grid(column=0, row=1, sticky="W")
target_height_var = tk.StringVar()
ttk.Entry(mainframe, textvariable=target_height_var, width=20).grid(column=1, row=1)

ttk.Button(mainframe, text="Calculate", command=calculate).grid(column=0, row=2, columnspan=2, pady=10)

result_var = tk.StringVar()
ttk.Label(mainframe, textvariable=result_var, justify="left", background="#eef", padding=10).grid(column=0, row=3, columnspan=2, sticky="W")

root.mainloop()
