#!/usr/bin/env python3
"""
ros2_to_relay_publisher.py
══════════════════════════════════════════════════════
Bridge node between ROS2 topics and Render.com
WebSocket relay server.

Subscribes to all gripper and arm topics,
packages into single JSON message,
sends to Render.com relay at 10Hz.

How to run:
  ros2 run robotic_control ros2_to_relay_publisher \
    --ros-args \
    -p relay_url:=wss://yourapp.onrender.com \
    -p auth_token:=YOUR_TOKEN_HERE \
    -p publish_hz:=10.0
══════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32, String, Float32MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import json
import time
import threading
import asyncio
import websockets


class Ros2ToRelayPublisher(Node):
    """
    Bridges ROS2 topics to Render.com
    WebSocket relay server.
    """

    def __init__(self):
        super().__init__('ros2_to_relay_publisher')

        # ── Parameters ──────────────────────────────
        self.declare_parameter('relay_url',
            'wss://yourapp.onrender.com')
        self.declare_parameter('auth_token',
            'YOUR_TOKEN_HERE')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('joint_names', [
            'arm_joint1',
            'arm_joint2',
            'arm_joint3',
            'gripper_joint'
        ])

        self.relay_url = self.get_parameter(
            'relay_url').get_parameter_value().string_value
        self.auth_token = self.get_parameter(
            'auth_token').get_parameter_value().string_value
        self.publish_hz = self.get_parameter(
            'publish_hz').get_parameter_value().double_value
        self.joint_names = self.get_parameter(
            'joint_names').get_parameter_value().string_array_value

        # ── Internal State ───────────────────────────
        # Latest values from each ROS2 topic
        # Default safe values used until first message
        self.fsr_force        = 0.0
        self.servo_pos_rad    = 0.0
        self.servo_pos_deg    = 0.0
        self.grip_state       = 'OPEN'
        self.pid_debug        = [0.0, 0.0, 0.0, 0.0]
        self.twin_delta       = 0.0
        self.hw_status        = 'SIMULATION'
        self.sim_mode         = 'unknown'
        self.joint_positions  = [0.0, 0.0, 0.0, 0.0]
        self.joint_velocities = [0.0, 0.0, 0.0, 0.0]

        # WebSocket connection state
        self.websocket        = None
        self.connected        = False
        self.reconnect_delay  = 2.0  # seconds

        # ── Subscriptions ────────────────────────────
        self.create_subscription(
            Float32,
            '/gripper/fsr/force',
            self.fsr_callback, 10)

        self.create_subscription(
            JointState,
            '/gripper/servo/position',
            self.servo_callback, 10)

        self.create_subscription(
            String,
            '/gripper/grip_state',
            self.grip_state_callback, 10)

        self.create_subscription(
            Float32MultiArray,
            '/gripper/control/debug',
            self.pid_debug_callback, 10)

        self.create_subscription(
            Float32,
            '/gripper/twin_delta',
            self.twin_delta_callback, 10)

        self.create_subscription(
            String,
            '/gripper/hardware/status',
            self.hw_status_callback, 10)

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback, 10)

        # ── Publish Timer ────────────────────────────
        period = 1.0 / self.publish_hz
        self.create_timer(period, self.publish_to_relay)

        # ── Start WebSocket in background thread ─────
        self.ws_thread = threading.Thread(
            target=self.run_websocket_loop,
            daemon=True)
        self.ws_thread.start()

        self.get_logger().info(
            f'ros2_to_relay_publisher started')
        self.get_logger().info(
            f'Relay URL  : {self.relay_url}')
        self.get_logger().info(
            f'Publish Hz : {self.publish_hz}')
        self.get_logger().info(
            f'Connecting to Render.com relay...')

    # ── ROS2 Topic Callbacks ─────────────────────────

    def fsr_callback(self, msg):
        self.fsr_force = msg.data

    def servo_callback(self, msg):
        if msg.position:
            self.servo_pos_rad = msg.position[0]
            self.servo_pos_deg = (
                msg.position[0] * 57.2958)

    def grip_state_callback(self, msg):
        self.grip_state = msg.data

    def pid_debug_callback(self, msg):
        self.pid_debug = list(msg.data)

    def twin_delta_callback(self, msg):
        self.twin_delta = msg.data

    def hw_status_callback(self, msg):
        self.hw_status = msg.data

    def joint_states_callback(self, msg):
        """
        Reads ALL 4 joint positions:
          arm_joint1 (shoulder)
          arm_joint2 (elbow)
          arm_joint3 (wrist)
          gripper_joint
        """
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                if idx < len(self.joint_positions):
                    self.joint_positions[idx] = (
                        msg.position[i]
                        if i < len(msg.position)
                        else 0.0)

    # ── WebSocket Logic ──────────────────────────────

    def run_websocket_loop(self):
        """
        Runs asyncio WebSocket loop in
        background thread. Auto-reconnects
        on disconnect.
        """
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(
            self.websocket_connect_loop())

    async def websocket_connect_loop(self):
        """
        Keeps trying to connect to Render.com.
        Reconnects automatically on disconnect.
        """
        attempt = 0
        while rclpy.ok():
            try:
                attempt += 1
                self.get_logger().info(
                    f'Connection attempt {attempt}...')

                async with websockets.connect(
                    self.relay_url,
                    ping_interval=20,
                    ping_timeout=10
                ) as ws:
                    self.websocket = ws
                    self.connected = True
                    self.reconnect_delay = 2.0

                    # Send auth handshake first
                    handshake = json.dumps({
                        'type': 'publisher',
                        'auth': self.auth_token
                    })
                    await ws.send(handshake)

                    self.get_logger().info(
                        '✅ Connected to Render relay')

                    # Keep connection alive
                    async for message in ws:
                        pass  # listen for server pings

            except Exception as e:
                self.connected = False
                self.websocket = None
                self.get_logger().warn(
                    f'⚠️  Relay disconnected: {e}')
                self.get_logger().info(
                    f'Retrying in '
                    f'{self.reconnect_delay}s...')

                await asyncio.sleep(
                    self.reconnect_delay)

                # Exponential backoff (max 16s)
                self.reconnect_delay = min(
                    self.reconnect_delay * 2, 16.0)

    def publish_to_relay(self):
        """
        Timer callback at 10Hz.
        Packages all latest ROS2 data into
        JSON and sends to Render.com relay.
        """
        if not self.connected or \
           self.websocket is None:
            return

        # Build JSON message
        # arm_joint1=shoulder, 2=elbow, 3=wrist
        payload = {
            'auth'          : self.auth_token,
            'timestamp'     : time.time(),
            'fsr_force'     : round(
                                self.fsr_force, 3),

            # Gripper servo (FSR controlled)
            'gripper_rad'   : round(
                                self.servo_pos_rad, 4),
            'gripper_deg'   : round(
                                self.servo_pos_deg, 2),

            # Arm servos (manually controlled)
            'arm_joint1_rad': round(
                                self.joint_positions[0], 4),
            'arm_joint2_rad': round(
                                self.joint_positions[1], 4),
            'arm_joint3_rad': round(
                                self.joint_positions[2], 4),

            # Degrees for Unity UI display
            'arm_joint1_deg': round(
                                self.joint_positions[0]
                                * 57.2958, 2),
            'arm_joint2_deg': round(
                                self.joint_positions[1]
                                * 57.2958, 2),
            'arm_joint3_deg': round(
                                self.joint_positions[2]
                                * 57.2958, 2),

            # Control state
            'grip_state'    : self.grip_state,
            'pid_debug'     : self.pid_debug,
            'twin_delta'    : round(
                                self.twin_delta, 3),
            'hw_status'     : self.hw_status,
            'sim_mode'      : self.sim_mode,
        }

        # Send asynchronously (non-blocking)
        try:
            asyncio.run_coroutine_threadsafe(
                self.websocket.send(
                    json.dumps(payload)),
                asyncio.get_event_loop()
            )
        except Exception as e:
            self.get_logger().warn(
                f'Send failed: {e}')
            self.connected = False


def main(args=None):
    rclpy.init(args=args)
    node = Ros2ToRelayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            'Shutting down relay publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
