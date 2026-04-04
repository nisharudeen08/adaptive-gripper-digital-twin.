#!/usr/bin/env python3

import rclpy
from gazebo_msgs.srv import SpawnEntity
import os

def spawn_robot():
    rclpy.init()
    
    # Wait for spawn service
    client = rclpy.create_node('spawn_robot_node')
    spawn_client = client.create_client(SpawnEntity, '/spawn_entity')
    
    while not spawn_client.wait_for_service(timeout_sec=5.0):
        print("Waiting for spawn_entity service...")
    
    # Read URDF
    urdf_path = os.path.expanduser('~/ros2_ws/install/robotic_description/share/robotic_description/urdf/robotic_arm.urdf')
    with open(urdf_path, 'r') as f:
        robot_urdf = f.read()
    
    # Create spawn request
    request = SpawnEntity.Request()
    request.name = 'robotic_arm'
    request.xml = robot_urdf
    request.robot_namespace = ''
    request.initial_pose.position.x = 0.0
    request.initial_pose.position.y = 0.0
    request.initial_pose.position.z = 0.0
    request.initial_pose.orientation.w = 1.0
    
    # Call spawn service
    future = spawn_client.call_async(request)
    rclpy.spin_until_future_complete(client, future)
    
    result = future.result()
    if result and result.success:
        print(f"✓ Robot spawned: {result.status_message}")
    else:
        print(f"✗ Spawn failed: {result.status_message if result else 'No response'}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    spawn_robot()
