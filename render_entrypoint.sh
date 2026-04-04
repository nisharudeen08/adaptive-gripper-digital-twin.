#!/usr/bin/env bash
set -euo pipefail

# Render provides $PORT; default to 10000 for local Docker runs.
PORT="${PORT:-10000}"

# ROS setup scripts can reference unset vars internally; disable nounset while sourcing.
set +u
source /opt/ros/humble/setup.bash
source /app/install/setup.bash
set -u

mkdir -p "${ROS_LOG_DIR:-/tmp/ros_logs}"

exec ros2 launch robotic_control rosbridge_websocket.launch.py \
  "port:=${PORT}" \
  "address:=0.0.0.0" \
  "max_message_size:=10000000" \
  "unregister_timeout:=10.0"
