"""Tiny offline window: pick log(s) -> PDF + 3D replay open automatically."""
from __future__ import annotations

import os
import threading
import webbrowser


def run_gui():
    import tkinter as tk
    from tkinter import filedialog, messagebox, scrolledtext

    from .cli import load_config, process

    root = tk.Tk()
    root.title("Drone Log QC (offline)")
    root.geometry("720x520")
    frm = tk.Frame(root, padx=10, pady=8)
    frm.pack(fill="x")
    fields = {}
    for i, (lab, key, default) in enumerate([
            ("Roll/Pitch b0 used (optional)", "tuning.roll_pitch_b0", ""),
            ("Yaw b0 used (optional)", "tuning.yaw_b0", ""),
            ("Take-off mass kg", "airframe.mass_kg", "2.0")]):
        tk.Label(frm, text=lab).grid(row=i, column=0, sticky="w")
        e = tk.Entry(frm, width=12); e.insert(0, default); e.grid(row=i, column=1, sticky="w")
        fields[key] = e
    out = scrolledtext.ScrolledText(root, height=20)
    out.pack(fill="both", expand=True, padx=10, pady=6)

    def log(msg):
        out.insert("end", msg + "\n"); out.see("end"); root.update_idletasks()

    def go():
        paths = filedialog.askopenfilenames(title="Select flight log(s)",
                                            filetypes=[("Flight logs", "*.rlog *.tlog *.csv *.txt"), ("All", "*.*")])
        if not paths:
            return
        ov = {}
        for k, e in fields.items():
            v = e.get().strip()
            if v:
                try:
                    ov[k] = float(v)
                except ValueError:
                    messagebox.showerror("Input", f"{k}: not a number"); return
        cfg = load_config(None, ov)

        def work():
            for p in paths:
                try:
                    _, pdf, html = process(p, None, cfg, log)
                    webbrowser.open("file://" + os.path.abspath(pdf))
                    if html:
                        webbrowser.open("file://" + os.path.abspath(html))
                except Exception as ex:
                    log(f"FAILED {p}: {ex}")
            log("Done.")
        threading.Thread(target=work, daemon=True).start()

    tk.Button(root, text="Select log file(s) and analyse", command=go, height=2).pack(fill="x", padx=10, pady=6)
    log("Supported: RFly .rlog, MAVLink .tlog, CSV export. Reports are saved next to the log in 'qc_reports'.")
    root.mainloop()
