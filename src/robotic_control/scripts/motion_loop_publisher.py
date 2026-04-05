#!/usr/bin/env python3
import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MotionLoopPublisher(Node):
    def __init__(self):
        super().__init__("motion_loop_publisher")

        self.arm_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10
        )

        self.t = 0.0
        self.dt = 0.1
        self.create_timer(self.dt, self.timer_cb)
        self.get_logger().info("motion_loop_publisher started")

    def timer_cb(self):
        self.t += self.dt

        # Staged, constrained motion for realistic behavior:
        # home -> reach -> grasp hold -> return.
        # Shoulder and elbow are explicitly coupled.
        # Wrist is held fixed to avoid unnatural extra movement.
        cycle_s = 16.0
        phase = self.t % cycle_s

        if phase < 3.0:
            # Home
            shoulder = 0.0
            elbow = 0.45
            wrist = -0.10
            finger = 0.20
        elif phase < 7.0:
            # Reach: shoulder drives elbow (coupled)
            p = (phase - 3.0) / 4.0
            shoulder = 0.05 + 0.45 * p          # 0.05 -> 0.50
            elbow = 0.45 + 0.70 * p             # 0.45 -> 1.15
            wrist = -0.10                        # fixed
            finger = 0.20
        elif phase < 9.5:
            # Grasp
            p = (phase - 7.0) / 2.5
            shoulder = 0.50
            elbow = 1.15
            wrist = -0.10                        # fixed
            finger = 0.20 + 0.22 * p             # 0.20 -> 0.42
        elif phase < 12.5:
            # Lift/transport: coordinated shoulder-elbow return while grasp held
            p = (phase - 9.5) / 3.0
            shoulder = 0.50 - 0.20 * p           # 0.50 -> 0.30
            elbow = 1.15 - 0.35 * p              # 1.15 -> 0.80
            wrist = -0.10                        # fixed
            finger = 0.42
        else:
            # Return and release
            p = (phase - 12.5) / 3.5
            shoulder = 0.30 * (1.0 - p)          # 0.30 -> 0.00
            elbow = 0.80 - 0.35 * p              # 0.80 -> 0.45
            wrist = -0.10                        # fixed
            finger = 0.42 - 0.34 * p

        # Safety clamps
        shoulder = max(-0.45, min(0.55, shoulder))
        elbow = max(0.40, min(1.20, elbow))
        wrist = -0.10
        finger = max(0.20, min(0.42, finger))

        arm_msg = JointTrajectory()
        arm_msg.header.stamp = self.get_clock().now().to_msg()
        arm_msg.joint_names = ["shoulder_joint", "elbow_joint", "wrist_joint"]

        arm_point = JointTrajectoryPoint()
        arm_point.positions = [shoulder, elbow, wrist]
        arm_point.velocities = [0.0, 0.0, 0.0]
        arm_point.time_from_start = Duration(sec=0, nanosec=400_000_000)
        arm_msg.points.append(arm_point)
        self.arm_pub.publish(arm_msg)

        grip_msg = JointTrajectory()
        grip_msg.header.stamp = self.get_clock().now().to_msg()
        grip_msg.joint_names = ["finger_left_joint"]

        grip_point = JointTrajectoryPoint()
        grip_point.positions = [finger]
        grip_point.velocities = [0.0]
        grip_point.time_from_start = Duration(sec=0, nanosec=400_000_000)
        grip_msg.points.append(grip_point)
        self.gripper_pub.publish(grip_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionLoopPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
