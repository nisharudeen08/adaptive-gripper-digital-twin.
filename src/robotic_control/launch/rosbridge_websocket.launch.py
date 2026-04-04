from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    address = LaunchConfiguration('address')
    max_message_size = LaunchConfiguration('max_message_size')
    unregister_timeout = LaunchConfiguration('unregister_timeout')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('address', default_value='0.0.0.0'),
        DeclareLaunchArgument('max_message_size', default_value='10000000'),
        DeclareLaunchArgument('unregister_timeout', default_value='10.0'),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': port,
                'address': address,
                'require_clients': False,
                'ssl': False,
                'max_message_size': max_message_size,
                'unregister_timeout': unregister_timeout,
            }],
        ),
    ])
