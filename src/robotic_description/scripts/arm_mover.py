#!/usr/bin/env python3
"""
arm_mover.py — ROS2 Python node that moves the robotic arm through a sequence
of waypoints by publishing JointTrajectory messages.

PACKAGE: robotic_description
TOPIC:   /arm_controller/joint_trajectory
TYPE:    trajectory_msgs/JointTrajectory

HOW TO RUN (after building the workspace):
------------------------------------------
Terminal 1 — Start the simulation (if not already running):
    cd ~/ros2_ws
    source install/setup.bash
    ros2 launch robotic_description gazebo.launch.py

Terminal 2 — Run this node:
    cd ~/ros2_ws
    source install/setup.bash
    ros2 run robotic_description arm_mover

WHAT IT DOES:
-------------
Sends a single JointTrajectory message containing 4 waypoints.
The controller_manager interpolates between waypoints and drives the arm
through each pose in sequence. time_from_start is cumulative: each waypoint
gives the controller a deadline (seconds from NOW) by which to reach that pose.

ADJUSTING JOINT NAMES:
-----------------------
If your joint names differ from the defaults below, change JOINT_NAMES.
They must exactly match the <joint name="..."> entries in your URDF and
the "joints:" list in controllers.yaml.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# =============================================================================
# CONFIGURATION — Change these to match your URDF joint names
# =============================================================================
JOINT_NAMES = [
    "shoulder_joint",   # Joint 1: Base yaw rotation (Z axis, ±90°)
    "elbow_joint",      # Joint 2: Upper arm pitch (Y axis, ±90°)
    "wrist_joint",      # Joint 3: Forearm pitch (Y axis, ±90°)
]

# Controller topic — matches the controller name in controllers.yaml
CONTROLLER_TOPIC = "/arm_controller/joint_trajectory"

# Waypoints: list of (positions_rad, time_from_start_sec)
# Each entry is a full pose for all joints in JOINT_NAMES order.
# time_from_start is cumulative — it's the time from trajectory START, not
# from the previous waypoint. Budget extra time for slower joints.
WAYPOINTS = [
    #  shoulder   elbow     wrist      time(s)
    ([  0.0,      0.0,      0.0  ],    2.0),   # Home position — all zeros
    ([  0.785,   -0.523,    0.3  ],    5.0),   # Pose 1: reach right-forward
    ([ -0.785,    0.523,   -0.3  ],    9.0),   # Pose 2: reach left-back
    ([  0.5,      0.785,   -0.785],   13.0),   # Pose 3: arm raised high
    ([  0.0,      0.0,      0.0  ],   16.0),   # Return to home
]


class ArmMover(Node):
    """
    ROS2 node that publishes a pre-planned trajectory to the arm_controller.

    Publishing model:
      - We publish once after a short delay (to ensure controller_manager is ready).
      - The controller buffers the full trajectory and executes it autonomously.
      - We do NOT need to publish at 100 Hz; one message triggers the full motion.
    """

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__("arm_mover")

        # Create a publisher on the arm controller's trajectory topic.
        # queue_size=10: buffer up to 10 messages if subscriber is slow (never an issue here).
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            CONTROLLER_TOPIC,
            10
        )

        # Wait 2 seconds before sending — gives controller_manager time to become active.
        # Without this delay, the message may be published before the controller is ready
        # and will be silently dropped.
        self.timer = self.create_timer(2.0, self.send_trajectory)

        self.get_logger().info(
            f"ArmMover node started. Will publish to '{CONTROLLER_TOPIC}' in 2 seconds."
        )

    def send_trajectory(self):
        """
        Build a JointTrajectory message from WAYPOINTS and publish it once.
        After publishing, cancel the timer so this method is never called again.
        """
        # Cancel the one-shot timer immediately so we only publish once
        self.timer.cancel()

        # ---- Build the JointTrajectory message --------------------------------
        msg = JointTrajectory()

        # header.stamp: Leave as zero (0,0). The controller interprets this as
        # "start executing this trajectory immediately." If you wanted a delayed
        # start, you'd set this to rclpy.clock.Clock().now().to_msg() + offset.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        # joint_names: Must exactly match JOINT_NAMES — order matters!
        # The positions in each JointTrajectoryPoint must be in this same order.
        msg.joint_names = JOINT_NAMES

        # ---- Build each waypoint (JointTrajectoryPoint) ----------------------
        for positions, t_sec in WAYPOINTS:
            point = JointTrajectoryPoint()

            # positions: List of joint angles in RADIANS, one per joint.
            # Index 0 = shoulder_joint, 1 = elbow_joint, 2 = wrist_joint.
            point.positions = positions

            # velocities: Target velocity at this waypoint. An empty list []
            # tells the controller to auto-compute velocities for smooth motion.
            # If you specify velocities, you must provide one per joint.
            point.velocities = []

            # time_from_start: Deadline (from trajectory start) to reach this pose.
            # We split seconds and nanoseconds because Duration uses integer fields.
            point.time_from_start = Duration(
                sec=int(t_sec),
                nanosec=int((t_sec - int(t_sec)) * 1e9)  # fractional seconds → ns
            )

            msg.points.append(point)

        # ---- Publish the complete trajectory ----------------------------------
        self.publisher_.publish(msg)

        # Log a human-readable summary of what was sent
        self.get_logger().info(
            f"Published trajectory with {len(WAYPOINTS)} waypoints to '{CONTROLLER_TOPIC}'."
        )
        for i, (pos, t) in enumerate(WAYPOINTS):
            pos_str = ", ".join(f"{p:+.3f}" for p in pos)
            self.get_logger().info(f"  Waypoint {i+1} @ t={t:.1f}s → [{pos_str}] rad")

        self.get_logger().info(
            "Trajectory sent. Watch Gazebo — arm should move through all poses. "
            f"Total motion time: {WAYPOINTS[-1][1]:.1f} seconds."
        )


def main(args=None):
    """
    Entry point. Standard ROS2 Python node lifecycle:
      rclpy.init() → create node → spin (process callbacks) → shutdown.
    """
    rclpy.init(args=args)           # Initialize ROS2 communication layer
    node = ArmMover()               # Create our node (timer starts here)
    rclpy.spin(node)                # Block here, processing callbacks until Ctrl-C
    node.destroy_node()             # Clean up node resources
    rclpy.shutdown()                # Shut down ROS2 communication layer


if __name__ == "__main__":
    main()
