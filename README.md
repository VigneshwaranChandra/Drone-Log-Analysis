# Drone Log QC (offline)

Drop a flight log in, get a **PDF QC report** + an **offline 3D replay**.
Works fully offline. Built for the RFly autopilot (`.rlog`, viewed in the RflySim log replayer),
also reads MAVLink `.tlog` (Mission Planner / QGC) and CSV exports.

## Handbook
`docs/DroneLogQC_Handbook.pdf` - the full reference: how to use the tool, what every log channel means,
quad physics and motor patterns, ADRC b0 tuning (incl. starting a new model), crash investigation procedure,
symptom -> where-to-look guide, the complete issue library, checklists, the sample crash, autopilot origin
check. Rebuild with `python -m dronelog_qc --handbook docs/DroneLogQC_Handbook.pdf`.

## What the report tells you
- **Where is the problem?** ranking: propulsion / GCS parameters / firmware / autopilot sensors / power /
  outside / airframe.
- **Diagnosis & action plan**: one card per detected issue - evidence, meaning, ranked causes with the area
  to look at, tick-box physical checks, fix.
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

## Detected issues (all verified on simulated faults)
Crash types: uncommanded upset, motor/prop failure (names the motor), power loss in air, collision, hard
landing, no crash-disarm. Motors: single weak motor, CG offset, yaw/diagonal imbalance, saturation high/low,
low thrust reserve, ESC desync, layout mismatch. Control: fast wobble, wobble only when braking, sluggish.
Sensors: vibration (flight / ground idle), estimator mismatch, GPS glitch, weak GPS, hover drift, altitude
hold. Environment: wind speed + direction. Power: low voltage, battery sag. Data: log gaps.

## Tests
`python -W ignore tests/test_loaders.py` and `python tests/test_detection.py` (simulates 14 faults with
`tools/simulate_rlog.py` and checks each is detected, and that a healthy flight gives no warnings).
