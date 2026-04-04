#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gazebo_msgs.msg import ContactsState # Built-in Gazebo bumper msg

class GazeboContactBridge(Node):
    def __init__(self):
        super().__init__('gazebo_contact_bridge')
        self.pub = self.create_publisher(Float32, '/gazebo/gripper_contact_force', 10)
        self.sub = self.create_subscription(ContactsState, '/gazebo/fingertip_bumper', self.cb, 10)
        self.filtered = 0.0
        
    def cb(self, msg):
        force = 0.0
        if msg.states:
            for state in msg.states:
                for wz in state.wrenches:
                    force += abs(wz.force.z) # Normal force
        
        self.filtered = 0.2 * force + 0.8 * self.filtered
        if force > 0.1 and self.filtered <= 0.1:
            self.get_logger().info("Contact detected in Gazebo!")
            
        self.pub.publish(Float32(data=float(self.filtered)))

def main():
    rclpy.init()
    rclpy.spin(GazeboContactBridge())
    rclpy.shutdown()

if __name__ == '__main__': main()
