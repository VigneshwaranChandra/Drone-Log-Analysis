"""CSV / TXT loader with a configurable column-name mapping.

Use this for data exported from any log viewer (including the RflySim log
replayer if it has an export option, or MATLAB/Excel dumps).

Column names are matched against regular expressions in
config/channel_map.json (case-insensitive, first match wins). Add a line there
when a new viewer uses different column names - no code change needed.

The ENU blocks from the RFly viewer (MSPositionENU / MSVelocityENU) are
converted to this tool's N/E/U naming:  ENU.x = East, ENU.y = North, ENU.z = Up.
"""
from __future__ import annotations

import json
import os
import re

import numpy as np
import pandas as pd

from ..data import FlightData

DEFAULT_MAP = os.path.join(os.path.dirname(__file__), "..", "..", "config", "channel_map.json")


def _load_map(path=None):
    with open(path or DEFAULT_MAP, "r", encoding="utf-8") as f:
        raw = json.load(f)
    rules = []
    for channel, patterns in raw["channels"].items():
        for p in patterns:
            rules.append((re.compile(p, re.IGNORECASE), channel))
    return raw, rules


def _time_seconds(col: pd.Series) -> np.ndarray:
    t = pd.to_numeric(col, errors="coerce").to_numpy(float)
    if np.all(np.isnan(t)):
        dt = pd.to_datetime(col, errors="coerce")
        t = (dt - dt.iloc[0]).dt.total_seconds().to_numpy(float)
    span = np.nanmax(t) - np.nanmin(t)
    step = np.nanmedian(np.diff(t))
    # Heuristic unit detection: a real flight log step is 1 ms .. 1 s
    if step > 500 or span > 1e7:          # microseconds
        t = t / 1e6
    elif step >= 1.0 and span > 5000:      # milliseconds
        t = t / 1000.0
    return t


def load_csv(path: str, map_path: str | None = None) -> FlightData:
    raw, rules = _load_map(map_path)
    sep = None
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        head = f.read(4096)
    if head.count("\t") > head.count(","):
        sep = "\t"
    df = pd.read_csv(path, sep=sep, engine="python")
    df.columns = [str(c).strip() for c in df.columns]

    time_col = None
    for c in df.columns:
        if re.search(raw.get("time_pattern", r"^(time|timestamp|t)\b"), c, re.IGNORECASE):
            time_col = c
            break
    if time_col is None:
        time_col = df.columns[0]
    t = _time_seconds(df[time_col])

    fd = FlightData(source=path)
    mapping = {}
    for c in df.columns:
        if c == time_col:
            continue
        for rx, ch in rules:
            if rx.search(c):
                if ch in mapping.values():
                    break
                mapping[c] = ch
                break
    angle_unit = raw.get("angle_unit", "auto")
    for col, ch in mapping.items():
        v = pd.to_numeric(df[col], errors="coerce").to_numpy(float)
        if ch in ("roll", "pitch", "yaw", "roll_sp", "pitch_sp", "yaw_sp") and angle_unit != "deg":
            if angle_unit == "rad" or np.nanmax(np.abs(v)) <= 3.2 * 1.05:
                v = np.degrees(v)
        if ch == "yaw" or ch == "yaw_sp":
            v = v % 360.0
        fd.add(ch, t, v)
    if fd.series:
        t0, _ = fd.time_range()
        fd.shift_time(t0)
    fd.meta.update({"format": "CSV", "column_mapping": mapping,
                    "unmapped_columns": [c for c in df.columns if c not in mapping and c != time_col],
                    "motor_units": "PWM_us" if any(
                        np.nanmax(fd.series[m][1]) > 200 for m in fd.motor_names()) else "percent"})
    return fd
