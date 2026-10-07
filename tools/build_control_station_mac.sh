#!/bin/zsh
set -e

BIKE_ROOT="${0:A:h:h}"
BIKE_PYTHON="$BIKE_ROOT/.venv-control-station/bin/python"
if [[ ! -x "$BIKE_PYTHON" ]]; then
  if command -v python3.13 >/dev/null 2>&1; then
    python3.13 -m venv "$BIKE_ROOT/.venv-control-station"
  else
    python3 -m venv "$BIKE_ROOT/.venv-control-station"
  fi
fi

"$BIKE_PYTHON" -m pip install -r "$BIKE_ROOT/requirements-control-station.txt"
"$BIKE_PYTHON" -m pip install 'pyinstaller>=6,<7'
"$BIKE_PYTHON" -m PyInstaller \
  --noconfirm --clean --windowed --name BikeControlStation \
  --add-data "$BIKE_ROOT/tools/control_station_web:tools/control_station_web" \
  --collect-all odrive --collect-all fibre \
  --paths "$BIKE_ROOT/tools" \
  --distpath "$BIKE_ROOT/dist" --workpath "$BIKE_ROOT/build/pyinstaller" \
  --specpath "$BIKE_ROOT/build/pyinstaller" \
  "$BIKE_ROOT/tools/control_station.py"
print "构建完成：$BIKE_ROOT/dist/BikeControlStation.app"
