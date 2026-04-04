# =============================================================================
# FILE: src/robotic_description/launch/gazebo.launch.py
# PURPOSE: Launches Gazebo Classic + robot_state_publisher + all controllers
#          for the adaptive_gripper_arm digital twin.
#
# LAUNCH ORDER (critical — wrong order causes controllers to fail):
#   1. robot_state_publisher    → publishes /robot_description
#   2. gzserver                 → starts Gazebo physics + loads ros2_control plugin
#   3. gzclient (after 3s)      → opens the GUI
#   4. spawn_entity (after 5s)  → spawns robot into Gazebo, starts controller_manager
#   5. joint_state_broadcaster  → MUST be first controller (others need it)
#   6. arm_controller           → trajectory controller for shoulder/elbow/wrist
#   7. gripper_controller       → trajectory controller for finger_left_joint
#
# WHY ORDER MATTERS:
#   - controller_manager is started by the gazebo_ros2_control plugin INSIDE
#     Gazebo. It does not exist until spawn_entity completes and the plugin loads.
#   - joint_state_broadcaster must be active before arm_controller, because
#     JointTrajectoryController reads feedback from /joint_states to close the loop.
#   - spawner nodes exit with code 0 on success — we use OnProcessExit to chain them.
#
# USAGE:
#   ros2 launch robotic_description gazebo.launch.py
#   ros2 launch robotic_description gazebo.launch.py use_sim_time:=true
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # =========================================================================
    # PACKAGE PATH — resolved at launch time, not at import time
    # =========================================================================
    pkg_path = get_package_share_directory("robotic_description")

    # =========================================================================
    # LAUNCH ARGUMENTS — overridable from the command line
    # =========================================================================
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation clock (/clock) instead of wall-clock. "
                    "Always true when running in Gazebo.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # =========================================================================
    # LOAD URDF — read the file as a plain string and pass it as a parameter.
    # We use the URDF that already has <ros2_control> and <gazebo> plugin blocks.
    # NOTE: We read the file at launch-time (Python), not at node-start-time,
    # because ParameterValue(Command(["xacro", ...])) can fail if there are
    # xacro calls that depend on package paths not yet on the ROS_PACKAGE_PATH.
    # For a plain .urdf (not .xacro), reading directly is simpler and safer.
    # =========================================================================
    import re
    urdf_file = os.path.join(pkg_path, "urdf", "adaptive_gripper_arm.urdf")
    with open(urdf_file, "r") as f:
        # Minify URDF by removing newlines and comments to prevent gazebo_ros2_control
        # parser crash when it forwards the parameter to controller_manager.
        # (Colons inside XML comments break the rcl parameter parser in Humble).
        raw_urdf = f.read()
        no_comments = re.sub(r'<!--.*?-->', '', raw_urdf, flags=re.DOTALL)
        robot_description_content = no_comments.replace('\n', ' ')

    # =========================================================================
    # STEP 1: robot_state_publisher
    # Publishes /robot_description (used by spawn_entity to get the URDF)
    # and /tf + /tf_static (joint transforms) using /joint_states feedback.
    # Must start FIRST so Gazebo spawn can find the URDF on the topic.
    # =========================================================================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            # Passing robot_description as a parameter string (not a file path)
            # avoids file-not-found errors if the node starts before install completes.
            {"robot_description": ParameterValue(robot_description_content, value_type=str)},
            {"use_sim_time": use_sim_time},
        ],
    )

    # =========================================================================
    # STEP 2: Gazebo Classic server (gzserver)
    # -s flags load Gazebo system plugins:
    #   libgazebo_ros_init.so    → initializes the ROS2-Gazebo bridge (/clock etc.)
    #   libgazebo_ros_factory.so → enables spawn_entity.py to work
    # --verbose prints full Gazebo startup logs — essential for debugging.
    # =========================================================================
    world_file = os.path.join(
        get_package_share_directory("gazebo_ros"), "worlds", "empty.world"
    )
    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            world_file,
        ],
        output="screen",
    )

    # =========================================================================
    # STEP 3: Gazebo Classic GUI (gzclient)
    # Delayed 3 seconds after gzserver starts to let the physics engine
    # fully initialize before the GUI tries to connect.
    # =========================================================================
    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
    )

    # =========================================================================
    # STEP 4: Spawn the robot into Gazebo
    # spawn_entity.py reads /robot_description topic (published by rsp above)
    # and inserts the robot model into the running Gazebo simulation.
    # Delayed 5 seconds so gzserver has time to boot and load the world.
    # =========================================================================
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        arguments=[
            "-entity", "adaptive_gripper_arm",   # Name of the model in Gazebo
            "-topic", "robot_description",         # Topic to read URDF from
            # Optional: set spawn pose if needed
            # "-x", "0.0", "-y", "0.0", "-z", "0.0",
        ],
        output="screen",
    )

    # =========================================================================
    # STEP 5: Spawner — joint_state_broadcaster
    # Uses the ROS2 Humble "spawner" executable (replaces old spawner.py).
    # This loads AND activates the controller in one step.
    # MUST run first — arm_controller needs /joint_states to be published.
    # =========================================================================
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # =========================================================================
    # STEP 6: Spawner — arm_controller (shoulder + elbow + wrist)
    # Started only AFTER joint_state_broadcaster exits with success (code 0).
    # =========================================================================
    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_arm_controller",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # =========================================================================
    # STEP 7: Spawner — gripper_controller (finger_left_joint)
    # Started only AFTER arm_controller exits with success.
    # =========================================================================
    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_gripper_controller",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # =========================================================================
    # EVENT HANDLERS — Chain the launch sequence
    #
    # OnProcessStart: triggers when a process STARTS (used to delay GUI)
    # OnProcessExit:  triggers when a process EXITS with code 0 (success)
    #
    # This chain ensures perfect ordering:
    #   gzserver starts → (3s) gzclient opens
    #   gzserver starts → (5s) spawn_robot runs
    #   spawn_robot exits → spawn_jsb runs
    #   spawn_jsb exits → spawn_arm_controller runs
    #   spawn_arm_controller exits → spawn_gripper_controller runs
    # =========================================================================

    # Delay gzclient 3s after gzserver process starts
    launch_gzclient_after_server = RegisterEventHandler(
        OnProcessStart(
            target_action=gzserver,
            on_start=[
                TimerAction(period=3.0, actions=[gzclient]),
            ],
        )
    )

    # Delay spawn_robot 5s after gzserver process starts
    spawn_robot_after_server = RegisterEventHandler(
        OnProcessStart(
            target_action=gzserver,
            on_start=[
                TimerAction(period=5.0, actions=[spawn_robot]),
            ],
        )
    )

    # Start joint_state_broadcaster immediately after spawn_entity finishes
    spawn_jsb_after_robot = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawn_jsb],
        )
    )

    # Start arm_controller after joint_state_broadcaster is confirmed active
    spawn_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_arm_controller],
        )
    )

    # Start gripper_controller after arm_controller is confirmed active
    spawn_gripper_after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_arm_controller,
            on_exit=[spawn_gripper_controller],
        )
    )

    # =========================================================================
    # RETURN the full launch description.
    # Only include "root" actions here — chained actions are registered via
    # event handlers above and will trigger automatically.
    # =========================================================================
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,       # Start immediately
        gzserver,                    # Start immediately
        launch_gzclient_after_server,  # Registers event: gzserver→gzclient
        spawn_robot_after_server,      # Registers event: gzserver→spawn_robot
        spawn_jsb_after_robot,         # Registers event: spawn_robot→jsb
        spawn_arm_after_jsb,           # Registers event: jsb→arm_controller
        spawn_gripper_after_arm,       # Registers event: arm→gripper_controller
    ])
