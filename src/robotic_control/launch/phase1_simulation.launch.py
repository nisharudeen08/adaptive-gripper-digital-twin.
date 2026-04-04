import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory('robotic_description')
    pkg_control = get_package_share_directory('robotic_control')

    # Arguments
    sim_mode = LaunchConfiguration('sim_mode', default='realistic')
    target_force = LaunchConfiguration('target_force', default='2.0')
    num_hz = LaunchConfiguration('num_hz', default='100.0')

    # Load parameter file
    param_file = os.path.join(pkg_control, 'config', 'gripper_params.yaml')

    use_gazebo = LaunchConfiguration('use_gazebo', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('sim_mode', default_value='realistic', description='Simulation Mode'),
        DeclareLaunchArgument('use_gazebo', default_value='false', description='Whether to launch heavy Gazebo GUI'),
        
        # 1-3. Simulated robot environment (Example, hooks into your existing Gazebo setup)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_description, 'launch', 'gazebo.launch.py')),
            condition=IfCondition(use_gazebo)
        ),
        
        # 4-5. Controller Spawners (These are now cleanly handled inside gazebo.launch.py directly!)

        # 6. Fake Sensor Publisher (PHASE 1 SPECIFIC)
        Node(
            package='robotic_control',
            executable='fake_sensor_publisher',
            name='fake_sensor_publisher',
            parameters=[param_file, {'sim_mode': sim_mode}]
        ),

        # 7. Adaptive Grip Controller
        Node(
            package='robotic_control',
            executable='adaptive_grip_controller',
            name='adaptive_grip_controller',
            parameters=[param_file, {'target_force': target_force}]
        ),

        # 8. FSR Gazebo Sync
        Node(
            package='robotic_control',
            executable='fsr_gazebo_sync',
            name='fsr_gazebo_sync',
            parameters=[param_file]
        ),

        # 9. Gazebo Contact Bridge
        Node(
            package='robotic_control',
            executable='gazebo_contact_bridge',
            name='gazebo_contact_bridge'
        ),

        # 10. Gripper Monitor Dashboard
        Node(
            package='robotic_control',
            executable='gripper_monitor',
            name='gripper_monitor',
            output='screen' # Ensure rich dashboard draws to terminal
        ),

        # 11. Data Exporter (CSV Logging)
        Node(
            package='robotic_control',
            executable='data_exporter',
            name='data_exporter'
        )
    ])
