import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_path = get_package_share_directory("robotic_description")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo clock if true.",
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            get_package_share_directory("gazebo_ros"), "worlds", "empty.world"
        ),
        description="Gazebo world file.",
    )
    declare_model = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(pkg_path, "urdf", "adaptive_gripper_arm.urdf"),
        description="URDF model path.",
    )
    robot_description_content = ParameterValue(
        Command(["xacro ", model]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time},
        ],
    )

    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
        ],
        output="screen",
    )

    gzclient = ExecuteProcess(cmd=["gzclient"], output="screen")

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "adaptive_gripper_arm", "-topic", "robot_description"],
        output="screen",
    )

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_jsb_after_robot = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[spawn_jsb])
    )
    spawn_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_arm_controller])
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_world,
            declare_model,
            robot_state_publisher,
            gzserver,
            gzclient,
            spawn_robot,
            spawn_jsb_after_robot,
            spawn_arm_after_jsb,
        ]
    )
