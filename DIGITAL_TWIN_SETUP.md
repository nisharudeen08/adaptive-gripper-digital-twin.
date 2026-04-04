# Adaptive Gripper Digital Twin Setup (ROS2 + Unity)

This workspace now includes rosbridge launch and automation scripts for the architecture:

ROS2 nodes -> rosbridge websocket (`ws://localhost:9090`) -> optional ngrok tunnel -> Unity WebSocket client.

## 1) One-time Ubuntu setup

Install rosbridge:

```bash
sudo apt update
sudo apt install ros-humble-rosbridge-suite
```

Optional (for tunnel): install ngrok and configure your token/domain in `~/.config/ngrok/ngrok.yml`.

Minimal tunnel config example:

```yaml
version: "2"
authtoken: YOUR_TOKEN_HERE
tunnels:
  rosbridge:
    proto: tcp
    addr: 9090
```

## 2) Build and source workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 3) Start full digital twin bridge stack

```bash
cd ~/ros2_ws
./start_digital_twin.sh
```

Default behavior:
- Starts `robotic_control phase1_simulation.launch.py` with `use_gazebo:=false`
- Starts `robotic_control rosbridge_websocket.launch.py` on `ws://localhost:9090`
- Starts `ngrok start rosbridge` if `ngrok` is installed

Logs written to:
- `.phase1.log`
- `.rosbridge.log`
- `.ngrok.log`

## 4) Stop stack

```bash
cd ~/ros2_ws
./stop_digital_twin.sh
```

## 5) Common runtime overrides

Run with Gazebo enabled:

```bash
PHASE1_EXTRA_ARGS="use_gazebo:=true sim_mode:=ramp" ./start_digital_twin.sh
```

Skip ngrok (LAN/local testing):

```bash
START_NGROK=false ./start_digital_twin.sh
```

Use different rosbridge port:

```bash
ROSBRIDGE_EXTRA_ARGS="port:=9191 address:=0.0.0.0" ./start_digital_twin.sh
```

## 6) Verification checklist

Local rosbridge smoke test:

```bash
wscat -c ws://localhost:9090
```

In `wscat`, subscribe:

```json
{"op":"subscribe","id":"1","topic":"/gripper/fsr/force","type":"std_msgs/Float32"}
```

You should receive publish JSON messages repeatedly.

Check rosbridge process log:

```bash
tail -f .rosbridge.log
```

Expected line includes:

`Rosbridge WebSocket server started on port 9090`

## 7) Unity URL notes

- Local/LAN: `ws://<ubuntu-ip>:9090`
- ngrok TCP tunnel: use exact host and port from ngrok forwarding output
- If ngrok gives a TCP endpoint, include the port in Unity (`ws://host:port`)

## 8) Files added

- `src/robotic_control/launch/rosbridge_websocket.launch.py`
- `start_digital_twin.sh`
- `stop_digital_twin.sh`
