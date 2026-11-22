@echo off
REM Builds a single offline DroneLogQC.exe (double-click -> pick log -> PDF + 3D open).
REM Needs Python 3.10+ for Windows (python.org installer includes tkinter).
python -m pip install -r requirements.txt pyinstaller
pyinstaller --noconfirm --onefile --windowed --name DroneLogQC ^
  --add-data "config;config" --collect-submodules pymavlink run_gui.py
echo.
echo Done: dist\DroneLogQC.exe  (copy the config folder next to it to change settings)
pause
