#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="$ROOT_DIR/.digital_twin_pids"

stop_pid() {
  local name="$1"
  local pid="$2"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    echo "Stopping $name (PID $pid)..."
    kill "$pid" 2>/dev/null || true
  fi
}

echo "Stopping Digital Twin Environment..."

if [[ -f "$PID_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$PID_FILE"
  stop_pid "ngrok" "${NGROK_PID:-}"
  stop_pid "rosbridge" "${ROSBRIDGE_PID:-}"
  stop_pid "phase1 stack" "${ROS_PID:-}"
  rm -f "$PID_FILE"
else
  echo "No PID file found. Running targeted fallback cleanup..."
  pkill -f "rosbridge_websocket" 2>/dev/null || true
  pkill -f "phase1_simulation.launch.py" 2>/dev/null || true
  pkill -x ngrok 2>/dev/null || true
fi

sleep 1

# Final safety cleanup for known process names from this stack only.
pkill -f "ros2 launch robotic_control rosbridge_websocket.launch.py" 2>/dev/null || true
pkill -f "ros2 launch robotic_control phase1_simulation.launch.py" 2>/dev/null || true

echo "Environment stop sequence complete."
