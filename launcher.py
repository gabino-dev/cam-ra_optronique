import tkinter as tk
from tkinter import ttk
import os
import subprocess

SCRIPTS_FOLDER = "scripts"

def refresh_script_list():
    listbox.delete(0, tk.END)
    for filename in os.listdir(SCRIPTS_FOLDER):
        if filename.endswith(".py"):
            listbox.insert(tk.END, filename)

def run_selected_script():
    selection = listbox.curselection()
    if not selection:
        return
    selected_script = listbox.get(selection[0])
    script_path = os.path.join(SCRIPTS_FOLDER, selected_script)
    subprocess.Popen(["python", script_path])

# Création de l'interface
root = tk.Tk()
root.title("Python Script Launcher")

frame = ttk.Frame(root, padding=20)
frame.pack(fill="both", expand=True)

ttk.Label(frame, text="Available Scripts:").pack(anchor="w")

listbox = tk.Listbox(frame, height=10, width=40)
listbox.pack(pady=5, fill="both", expand=True)

ttk.Button(frame, text="Run Selected Script", command=run_selected_script).pack(pady=10)

refresh_script_list()

root.mainloop()
