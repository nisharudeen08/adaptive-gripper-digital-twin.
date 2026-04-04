from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("adaptive_gripper_description"))
    urdf_path = package_share / "urdf" / "adaptive_gripper_arm.urdf"
    controller_config = package_share / "config" / "joint_limits.yaml"
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description = urdf_path.read_text()

    use_sim_time = {"use_sim_time": True}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[use_sim_time, {"robot_description": robot_description}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_adaptive_gripper",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "adaptive_gripper_arm", "-z", "0.0"],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            str(controller_config),
        ],
        output="screen",
    )

    arm_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            str(controller_config),
        ],
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[use_sim_time, {"robot_description": robot_description}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(Path(get_package_share_directory("rviz2")) / "default.rviz"),
                description="RViz config file",
            ),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[arm_position_controller],
                )
            ),
            rviz2,
        ]
    )
