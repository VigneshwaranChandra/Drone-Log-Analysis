# Drone Log QC (offline)

Drop a flight log in, get a **PDF QC report** + an **offline 3D replay**.
Works fully offline. Built for the RFly autopilot (`.rlog`, viewed in the RflySim log replayer),
also reads MAVLink `.tlog` (Mission Planner / QGC) and CSV exports.

## What the report tells you
- **QC verdict** (PASS / PASS WITH WARNINGS / FAIL) and every finding with evidence + confidence.
- **Crash / upset reconstruction**: when control was lost, whether the controller fought it
  (motor split / saturation) or not, impact time and g-force, motors still running after the crash.
- **Movement timeline**: ground / climb / hover / cruise / turn / braking / descent / UPSET, with speed,
  course vs heading, altitude.
- **Motors**: balance, saturation, throttle reserve, CG trim, diagonal (yaw) imbalance, and a
  **physics check of the motor layout** (does M1..M4 in the config match how the model really rotated?).
- **Vibration**: gyro noise on the ground and in flight, spectrum with the dominant frequency.
- **ADRC**: reads back the b0 values from the log and gives **+/-0.25 / +/-0.5** b0 advice per axis.
- **Power**: battery voltage per cell, sag.
- **Wind** (when the log has steady hover).

## Use
Windows: run `build_windows_exe.bat` once -> `dist\DroneLogQC.exe`. Double-click, pick log(s),
PDF + 3D replay open automatically (saved in `qc_reports\` next to the log).

Any OS with Python 3.10+:
```
pip install -r requirements.txt
python -m dronelog_qc                                  # window
python -m dronelog_qc flight.rlog --rp-b0 8.0 --yaw-b0 -1.4 --mass 2.0   # command line
```

Settings (motor layout, PWM range, thresholds, tuning step sizes): `config/default.json`.

## Docs
- `docs/RLOG_FORMAT.md` - the reverse-engineered `.rlog` format and how each field was verified.
- `docs/ADRC_TUNING_GUIDE.md` - what b0 means, how to start a new model, how to read the advice.
- `docs/SAMPLE_LOG_202511081618.md` - findings on the first sample log (a crash).
- `docs/AUTOPILOT_ORIGIN_CHECK.md` - what the log says about where the autopilot software comes from,
  and how to verify it.

## Tests
`python -W ignore tests/test_loaders.py`
