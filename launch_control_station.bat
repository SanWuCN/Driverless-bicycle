@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv-control-station\Scripts\python.exe" (
    py -3 -m venv .venv-control-station
    if errorlevel 1 goto failed
)
".venv-control-station\Scripts\python.exe" -m pip install -r requirements-control-station.txt
if errorlevel 1 goto failed
".venv-control-station\Scripts\python.exe" tools\control_station.py --open-browser
exit /b %errorlevel%
:failed
echo Launch failed. Install Python 3 and check the dependency installation above.
pause
exit /b 1
