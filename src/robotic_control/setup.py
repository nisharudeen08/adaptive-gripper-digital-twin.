from setuptools import setup
import os
from glob import glob

package_name = 'robotic_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=['scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Senior Engineer',
    maintainer_email='engineer@robotics.com',
    description='Adaptive Gripper Digital Twin',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_sensor_publisher = scripts.fake_sensor_publisher:main',
            'adaptive_grip_controller = scripts.adaptive_grip_controller:main',
            'ros2_to_relay_publisher = scripts.ros2_to_relay_publisher:main',
            'fsr_gazebo_sync = scripts.fsr_gazebo_sync:main',
            'gazebo_contact_bridge = scripts.gazebo_contact_bridge:main',
            'gripper_monitor = scripts.gripper_monitor:main',
            'data_exporter = scripts.data_exporter:main',
            'fsr_serial_bridge = scripts.fsr_serial_bridge:main',
            'motion_loop_publisher = scripts.motion_loop_publisher:main'
        ],
    },
)
