#!/usr/bin/env python3
"""
gripper_mover.py — ROS 2 Python node to test the gripper controller.

It publishes a trajectory to open and close the gripper.
Because of the <mimic> tag in the URDF, driving 'finger_left_joint' automatically
causes Gazebo to drive 'finger_right_joint'.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# =============================================================================
# CONFIGURATION
# =============================================================================
JOINT_NAMES = [
    "finger_left_joint",  # The exact joint used in controllers.yaml
]

CONTROLLER_TOPIC = "/gripper_controller/joint_trajectory"

WAYPOINTS = [
    ([ 0.0 ],    2.0),    # Gripper Open (0 rad) at t=2.0s
    ([ 1.5708 ], 5.0),    # Gripper Close (~90 degrees) at t=5.0s
    ([ 0.0 ],    8.0),    # Back to Open at t=8.0s
]


class GripperMover(Node):
    def __init__(self):
        super().__init__("gripper_mover")
        self.publisher_ = self.create_publisher(JointTrajectory, CONTROLLER_TOPIC, 10)
        self.timer = self.create_timer(1.0, self.send_trajectory)
        self.get_logger().info("GripperMover started. Will test open/close sequence...")

    def send_trajectory(self):
        self.timer.cancel()
        msg = JointTrajectory()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.joint_names = JOINT_NAMES

        for positions, t_sec in WAYPOINTS:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = []
            point.time_from_start = Duration(
                sec=int(t_sec),
                nanosec=int((t_sec - int(t_sec)) * 1e9)
            )
            msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info("Gripper trajectory published! Watch Gazebo.")


def main(args=None):
    rclpy.init(args=args)
    node = GripperMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
