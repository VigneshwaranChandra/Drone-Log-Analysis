"""Pick a loader from the file extension / content."""
import os

from ..data import FlightData


def load_any(path: str) -> FlightData:
    ext = os.path.splitext(path)[1].lower()
    with open(path, "rb") as f:
        head = f.read(16)
    if ext == ".rlog" or head[:2] == b"AC" and b"RLog" in head:
        from .rlog import load_rlog
        return load_rlog(path)
    if ext == ".tlog":
        from .tlog import load_tlog
        return load_tlog(path)
    if ext in (".csv", ".txt", ".tsv"):
        from .csvlog import load_csv
        return load_csv(path)
    raise ValueError(f"Unsupported log type: {path} (supported: .rlog .tlog .csv)")
