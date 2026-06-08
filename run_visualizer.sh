#!/usr/bin/env bash
# Lanza el visualizador IMU usando el entorno virtual del proyecto
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/.venv/bin/python" "$SCRIPT_DIR/imu_visualizer.py" "$@"
