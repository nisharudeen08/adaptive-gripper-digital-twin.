import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_control = get_package_share_directory('robotic_control')
    param_file = os.path.join(pkg_control, 'config', 'gripper_params.yaml')

    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        
        # 6. Real Serial Bridge (PHASE 2 SPECIFIC - Replaces Fake Sensor)
        Node(
            package='robotic_control',
            executable='fsr_serial_bridge',
            name='fsr_serial_bridge',
            parameters=[param_file, {'serial_port': serial_port}]
        ),

        # 7. Adaptive Grip Controller (Uses strictly the exact same params!)
        Node(
            package='robotic_control',
            executable='adaptive_grip_controller',
            name='adaptive_grip_controller',
            parameters=[param_file]
        ),

        # 8, 9, 10, 11 (Same identical nodes as Phase 1...)
        Node(package='robotic_control', executable='fsr_gazebo_sync', name='fsr_gazebo_sync'),
        Node(package='robotic_control', executable='gazebo_contact_bridge', name='gazebo_contact_bridge'),
        Node(package='robotic_control', executable='gripper_monitor', name='gripper_monitor', output='screen'),
        Node(package='robotic_control', executable='data_exporter', name='data_exporter')
    ])
