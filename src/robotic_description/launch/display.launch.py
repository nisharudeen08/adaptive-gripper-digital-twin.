from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("robotic_description")
    urdf_file = Path(pkg_share) / "urdf" / "robotic_arm.urdf"
    controllers_file = Path(pkg_share) / "config" / "controllers.yaml"
    entity_name = "robotic_arm"
    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=str(Path(get_package_share_directory("rviz2")) / "default.rviz"),
        description="Path to RViz configuration file",
    )

    robot_description = urdf_file.read_text()
    use_sim_time = {"use_sim_time": True}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("gazebo_ros")) / "launch" / "gazebo.launch.py")
        ),
        launch_arguments={"world": "empty.world"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[use_sim_time, {"robot_description": robot_description}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            entity_name,
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.02",
        ],
    )

    # Note: Controller spawning requires ros2_control packages. 
    # Comment out these nodes if controller_manager is not available.
    # Uncomment when ros2-humble-ros2-control and ros2-humble-ros2-controllers are installed:
    # sudo apt-get install -y ros2-humble-ros2-control ros2-humble-ros2-controllers
    
    # joint_state_broadcaster = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     output="screen",
    #     arguments=[
    #         "joint_state_broadcaster",
    #         "--controller-manager",
    #         "/controller_manager",
    #         "--param-file",
    #         str(controllers_file),
    #     ],
    # )

    controller_spawners = []
    # Uncomment controller spawners after installing ros2_control packages
    # for controller in [
    #     "shoulder_position_controller",
    #     "elbow_position_controller",
    #     "wrist_position_controller",
    #     "gripper_position_controller",
    # ]:
    #     controller_spawners.append(
    #         Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             output="screen",
    #             arguments=[
    #                 controller,
    #                 "--controller-manager",
    #                 "/controller_manager",
    #                 "--param-file",
    #                 str(controllers_file),
    #             ],
    #         )
    #     )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[use_sim_time, {"robot_description": robot_description}],
    )

    # Removed controller event handler since controller_manager is not available
    # controller_start_handler = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=spawn_entity,
    #         on_start=[joint_state_broadcaster] + controller_spawners,
    #     )
    # )

    return LaunchDescription(
        [
            SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1"),
            SetEnvironmentVariable("GAZEBO_MASTER_URI", "http://127.0.0.1:11345"),
            rviz_config,
            gazebo,
            robot_state_publisher,
            spawn_entity,
            # controller_start_handler,  # Uncomment after installing ros2_control
            rviz,
        ]
    )
