#!/usr/bin/env python3
"""
fake_sensor_publisher.py
Publishes fake sensor data for ALL joints:
  - FSR force (gripper only)
  - finger_left_joint position (FSR linked)
  - shoulder_joint, elbow_joint, wrist_joint
    (user will control via rqt in real use,
     but here we simulate slow movements
     so Gazebo and Unity show full arm motion)
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String


class FakeSensorPublisher(Node):

    def __init__(self):
        super().__init__("fake_sensor_publisher")

        # Parameters
        self.declare_parameter("sim_mode", "step")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("noise_std_fsr", 0.05)
        self.declare_parameter("force_max", 5.0)

        self.sim_mode = self.get_parameter("sim_mode").get_parameter_value().string_value
        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

        # Publishers
        self.fsr_pub = self.create_publisher(Float32, "/gripper/fsr/force", 10)
        self.servo_pub = self.create_publisher(JointState, "/gripper/servo/position", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states_fake", 10)
        self.hw_pub = self.create_publisher(String, "/gripper/hardware/status", 10)
        self.state_pub = self.create_publisher(String, "/gripper/grip_state", 10)

        # Internal state
        self.start_time = time.time()
        self.t = 0.0
        self.shoulder_pos = 0.0
        self.elbow_pos = 0.0
        self.wrist_pos = 0.0
        self.gripper_pos = 0.0
        self.fsr_force = 0.0
        self.grip_state = "OPEN"

        self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info("fake_sensor_publisher started")
        self.get_logger().info(f"Mode: {self.sim_mode}")

    def timer_callback(self):
        self.t = time.time() - self.start_time
        self.sim_mode = self.get_parameter("sim_mode").get_parameter_value().string_value

        if self.sim_mode == "step":
            self.run_step_mode()
        elif self.sim_mode == "ramp":
            self.run_ramp_mode()
        elif self.sim_mode == "noise":
            self.run_noise_mode()
        elif self.sim_mode == "overforce":
            self.run_overforce_mode()
        elif self.sim_mode == "realistic":
            self.run_realistic_mode()
        else:
            self.run_step_mode()

        self.publish_fsr()
        self.publish_servo()
        self.publish_joint_states()
        self.publish_hw_status()
        self.publish_grip_state()

    def run_step_mode(self):
        cycle = self.t % 8.0
        self.shoulder_pos = 0.3
        self.elbow_pos = 0.5
        self.wrist_pos = -0.1

        if cycle < 2.0:
            self.fsr_force = 0.0
            self.gripper_pos = 0.0
            self.grip_state = "OPEN"
        elif cycle < 4.0:
            self.fsr_force = 1.0
            self.gripper_pos = 0.5
            self.grip_state = "CONTACT"
        elif cycle < 6.0:
            self.fsr_force = 3.0
            self.gripper_pos = 1.2
            self.grip_state = "GRASPING"
        else:
            self.fsr_force = 0.0
            self.gripper_pos = 0.0
            self.grip_state = "OPEN"

    def run_ramp_mode(self):
        cycle = self.t % 8.0
        self.shoulder_pos = 0.4 * math.sin(self.t * 0.3)
        self.elbow_pos = 0.3 * math.sin(self.t * 0.2 + 1.0)
        self.wrist_pos = 0.2 * math.sin(self.t * 0.4 + 0.5)

        if cycle < 5.0:
            self.fsr_force = (cycle / 5.0) * 5.0
        else:
            self.fsr_force = max(0.0, 5.0 - ((cycle - 5.0) / 3.0) * 5.0)

        self.gripper_pos = (self.fsr_force / 5.0) * 1.5708
        self.grip_state = self.get_grip_state()

    def run_noise_mode(self):
        self.shoulder_pos = 0.3 + random.gauss(0, 0.02)
        self.elbow_pos = 0.5 + random.gauss(0, 0.02)
        self.wrist_pos = -0.1 + random.gauss(0, 0.01)

        self.fsr_force = 2.0 + random.gauss(0, 0.3)
        self.fsr_force = max(0.0, self.fsr_force)
        self.gripper_pos = 1.0 + random.gauss(0, 0.05)
        self.gripper_pos = max(0.0, min(1.5708, self.gripper_pos))
        self.grip_state = self.get_grip_state()

    def run_overforce_mode(self):
        self.shoulder_pos = 0.3
        self.elbow_pos = 0.5
        self.wrist_pos = -0.1

        self.fsr_force = min(12.0, self.t * 1.5)
        self.gripper_pos = min(1.5708, (self.fsr_force / 10.0) * 1.5708)

        if self.fsr_force > 10.0:
            self.grip_state = "EMERGENCY"
        else:
            self.grip_state = self.get_grip_state()

    def run_realistic_mode(self):
        cycle = self.t % 10.0

        if cycle < 2.0:
            self.shoulder_pos = 0.0
            self.elbow_pos = 0.0
            self.wrist_pos = 0.0
            self.fsr_force = 0.0
            self.gripper_pos = 0.0
            self.grip_state = "OPEN"
        elif cycle < 4.0:
            p = (cycle - 2.0) / 2.0
            self.shoulder_pos = 0.3 * p
            self.elbow_pos = 0.8 * p
            self.wrist_pos = -0.2 * p
            self.fsr_force = 0.0
            self.gripper_pos = 0.0
            self.grip_state = "OPEN"
        elif cycle < 5.0:
            p = cycle - 4.0
            self.shoulder_pos = 0.3
            self.elbow_pos = 0.8
            self.wrist_pos = -0.2
            self.fsr_force = 0.5 * p
            self.gripper_pos = 0.3 * p
            self.grip_state = "CONTACT"
        elif cycle < 7.0:
            self.shoulder_pos = 0.3
            self.elbow_pos = 0.8
            self.wrist_pos = -0.2
            self.fsr_force = 2.5 + random.gauss(0, 0.1)
            self.gripper_pos = 1.2
            self.grip_state = "GRASPING"
        elif cycle < 9.0:
            p = (cycle - 7.0) / 2.0
            self.shoulder_pos = 0.3 + 0.3 * p
            self.elbow_pos = 0.8 - 0.3 * p
            self.wrist_pos = -0.2 + 0.1 * p
            self.fsr_force = 2.5
            self.gripper_pos = 1.2
            self.grip_state = "GRASPING"
        else:
            self.shoulder_pos = 0.6
            self.elbow_pos = 0.5
            self.wrist_pos = -0.1
            self.fsr_force = 0.0
            self.gripper_pos = 0.0
            self.grip_state = "OPEN"

    def get_grip_state(self):
        if self.fsr_force < 0.1:
            return "OPEN"
        if self.fsr_force < 2.0:
            return "CONTACT"
        if self.fsr_force > 10.0:
            return "EMERGENCY"
        return "GRASPING"

    def publish_fsr(self):
        msg = Float32()
        msg.data = float(max(0.0, self.fsr_force))
        self.fsr_pub.publish(msg)

    def publish_servo(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["finger_left_joint"]
        msg.position = [float(self.gripper_pos)]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        self.servo_pub.publish(msg)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["shoulder_joint", "elbow_joint", "wrist_joint", "finger_left_joint"]
        msg.position = [
            float(self.shoulder_pos),
            float(self.elbow_pos),
            float(self.wrist_pos),
            float(self.gripper_pos),
        ]
        msg.velocity = [0.0, 0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0, 0.0]
        self.joint_pub.publish(msg)

    def publish_hw_status(self):
        msg = String()
        msg.data = "SIMULATION"
        self.hw_pub.publish(msg)

    def publish_grip_state(self):
        msg = String()
        msg.data = self.grip_state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down fake sensor publisher")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
