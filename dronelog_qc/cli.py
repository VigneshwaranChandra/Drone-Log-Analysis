"""Command line: python -m dronelog_qc <log files...> [-o outdir] [--config cfg.json]"""
from __future__ import annotations

import argparse
import warnings
import copy
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CFG = os.path.join(HERE, "..", "config", "default.json")


def load_config(path=None, overrides=None):
    with open(path or DEFAULT_CFG, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    for k, v in (overrides or {}).items():
        sect, key = k.split(".", 1)
        cfg[sect][key] = v
    return cfg


def process(log_path, out_dir=None, cfg=None, log=print):
    warnings.filterwarnings("ignore", category=RuntimeWarning)
    from .loaders import load_any
    from .analysis.core import analyze
    from .report.pdf import build_pdf
    from .viz.replay3d import build_replay

    cfg = copy.deepcopy(cfg or load_config())
    out_dir = out_dir or os.path.join(os.path.dirname(os.path.abspath(log_path)), "qc_reports")
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(log_path))[0]
    log(f"Reading {log_path} ...")
    fd = load_any(log_path)
    log(f"  {len(fd.series)} channels, {fd.time_range()[1]:.1f} s")
    res = analyze(fd, cfg)
    pdf = build_pdf(res, os.path.join(out_dir, base + "_QC.pdf"))
    log(f"  PDF    -> {pdf}")
    html = build_replay(res, os.path.join(out_dir, base + "_3D.html"), title=base)
    if html:
        log(f"  3D     -> {html}")
    js = os.path.join(out_dir, base + "_findings.json")
    with open(js, "w", encoding="utf-8") as f:
        json.dump({"summary": res.summary, "findings": [vars(x) for x in res.findings],
                   "tuning": res.tuning, "upset": res.upset}, f, indent=1, default=float)
    for x in res.findings:
        if x.severity in ("BAD", "WARN"):
            log(f"  [{x.severity}] {x.title}")
    return res, pdf, html


def main(argv=None):
    ap = argparse.ArgumentParser(description="Offline drone flight-log QC (RFly .rlog, MAVLink .tlog, CSV) -> PDF")
    ap.add_argument("logs", nargs="*", help="log files")
    ap.add_argument("-o", "--out", help="output folder (default: <log folder>/qc_reports)")
    ap.add_argument("-c", "--config", help="config JSON (default: config/default.json)")
    ap.add_argument("--rp-b0", type=float, help="roll/pitch ADRC b0 used on this flight")
    ap.add_argument("--yaw-b0", type=float, help="yaw ADRC b0 used on this flight")
    ap.add_argument("--mass", type=float, help="take-off mass kg")
    ap.add_argument("--gui", action="store_true", help="open the window instead")
    ap.add_argument("--handbook", metavar="PDF", help="write the reference handbook PDF and exit")
    a = ap.parse_args(argv)
    if a.handbook:
        from .report.handbook import build_handbook
        print(build_handbook(a.handbook, load_config(a.config)))
        return 0
    if a.gui or not a.logs:
        from .gui import run_gui
        return run_gui()
    ov = {}
    if a.rp_b0 is not None: ov["tuning.roll_pitch_b0"] = a.rp_b0
    if a.yaw_b0 is not None: ov["tuning.yaw_b0"] = a.yaw_b0
    if a.mass is not None: ov["airframe.mass_kg"] = a.mass
    cfg = load_config(a.config, ov)
    rc = 0
    for p in a.logs:
        try:
            process(p, a.out, cfg)
        except Exception as e:  # keep going with the other logs
            print(f"FAILED {p}: {e}", file=sys.stderr)
            rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
