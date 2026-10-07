#!/bin/zsh
set -e

BIKE_ROOT="${0:A:h}"
BIKE_PYTHON="$BIKE_ROOT/.venv-control-station/bin/python"

if [[ ! -x "$BIKE_PYTHON" ]]; then
  if command -v python3.13 >/dev/null 2>&1; then
    python3.13 -m venv "$BIKE_ROOT/.venv-control-station"
  else
    python3 -m venv "$BIKE_ROOT/.venv-control-station"
  fi
fi

"$BIKE_PYTHON" -m pip install -r "$BIKE_ROOT/requirements-control-station.txt"
exec "$BIKE_PYTHON" "$BIKE_ROOT/tools/control_station.py" --open-browser
