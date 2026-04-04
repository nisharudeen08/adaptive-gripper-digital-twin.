#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

PID_FILE="$ROOT_DIR/.digital_twin_pids"
PHASE1_LAUNCH="${PHASE1_LAUNCH:-robotic_control phase1_simulation.launch.py}"
PHASE1_EXTRA_ARGS="${PHASE1_EXTRA_ARGS:-use_gazebo:=false}"
ROSBRIDGE_LAUNCH="${ROSBRIDGE_LAUNCH:-robotic_control rosbridge_websocket.launch.py}"
ROSBRIDGE_EXTRA_ARGS="${ROSBRIDGE_EXTRA_ARGS:-port:=9090 address:=0.0.0.0 max_message_size:=10000000 unregister_timeout:=10.0}"
NGROK_TUNNEL_NAME="${NGROK_TUNNEL_NAME:-rosbridge}"
START_NGROK="${START_NGROK:-true}"

if [[ -f "$PID_FILE" ]]; then
  echo "Found existing PID file: $PID_FILE"
  echo "Run ./stop_digital_twin.sh first if a stack is already running."
  exit 1
fi

if [[ ! -f "/opt/ros/humble/setup.bash" ]]; then
  echo "ROS 2 Humble setup not found at /opt/ros/humble/setup.bash"
  exit 1
fi

# ROS setup scripts can reference unset vars internally; disable nounset while sourcing.
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  source "$ROOT_DIR/install/setup.bash"
else
  echo "Workspace overlay not found at $ROOT_DIR/install/setup.bash"
  echo "Run: colcon build && source install/setup.bash"
  exit 1
fi
set -u

export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROOT_DIR/.ros_logs}"
mkdir -p "$ROS_LOG_DIR"

cleanup_on_error() {
  local ec=$?
  if [[ $ec -ne 0 ]]; then
    echo "Startup failed. Cleaning up any started processes..."
    [[ -n "${ROS_PID:-}" ]] && kill "$ROS_PID" 2>/dev/null || true
    [[ -n "${ROSBRIDGE_PID:-}" ]] && kill "$ROSBRIDGE_PID" 2>/dev/null || true
    [[ -n "${NGROK_PID:-}" ]] && kill "$NGROK_PID" 2>/dev/null || true
    rm -f "$PID_FILE"
  fi
  exit $ec
}
trap cleanup_on_error ERR

echo "Starting Digital Twin Environment..."

echo "Launching Phase-1 stack: ros2 launch $PHASE1_LAUNCH $PHASE1_EXTRA_ARGS"
# shellcheck disable=SC2086
ros2 launch $PHASE1_LAUNCH $PHASE1_EXTRA_ARGS > "$ROOT_DIR/.phase1.log" 2>&1 &
ROS_PID=$!
echo "Phase-1 stack PID: $ROS_PID"

sleep 4

echo "Launching rosbridge: ros2 launch $ROSBRIDGE_LAUNCH $ROSBRIDGE_EXTRA_ARGS"
# shellcheck disable=SC2086
ros2 launch $ROSBRIDGE_LAUNCH $ROSBRIDGE_EXTRA_ARGS > "$ROOT_DIR/.rosbridge.log" 2>&1 &
ROSBRIDGE_PID=$!
echo "rosbridge PID: $ROSBRIDGE_PID"

sleep 2

NGROK_PID=""
if [[ "$START_NGROK" == "true" ]]; then
  if command -v ngrok >/dev/null 2>&1; then
    echo "Launching ngrok tunnel: ngrok start $NGROK_TUNNEL_NAME"
    ngrok start "$NGROK_TUNNEL_NAME" > "$ROOT_DIR/.ngrok.log" 2>&1 &
    NGROK_PID=$!
    echo "ngrok PID: $NGROK_PID"
  else
    echo "ngrok not found; skipping tunnel startup."
  fi
else
  echo "START_NGROK=false; skipping tunnel startup."
fi

{
  echo "ROS_PID=$ROS_PID"
  echo "ROSBRIDGE_PID=$ROSBRIDGE_PID"
  echo "NGROK_PID=$NGROK_PID"
} > "$PID_FILE"

echo ""
echo "DIGITAL TWIN BRIDGE ACTIVE"
echo "Local WebSocket: ws://localhost:9090"
echo "PID file: $PID_FILE"
echo "Logs: .phase1.log | .rosbridge.log | .ngrok.log"
echo ""
echo "Use ./stop_digital_twin.sh to stop all managed processes."
