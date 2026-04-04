#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class FsrGazeboSync(Node):
    def __init__(self):
        super().__init__('fsr_gazebo_sync')
        
        self.declare_parameter('lp_filter_alpha', 0.3)
        self.declare_parameter('warn_threshold_N', 0.5)
        self.declare_parameter('error_threshold_N', 2.0)
        self.declare_parameter('gazebo_timeout_sec', 2.0)
        
        self.real_fsr = 0.0
        self.gazebo_force = 0.0
        self.filtered_gazebo = 0.0
        self.last_gazebo_time = self.get_clock().now()
        
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(Float32, '/gripper/fsr/force', self.fsr_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, '/gazebo/gripper_contact_force', self.gazebo_cb, qos_profile_sensor_data)
        
        self.sim_force_pub = self.create_publisher(Float32, '/gripper/simulated_contact_force', 10)
        self.delta_pub = self.create_publisher(Float32, '/gripper/twin_delta', 10)
        
        self.timer = self.create_timer(1.0/50.0, self.loop)

    def fsr_cb(self, msg):
        self.real_fsr = msg.data

    def gazebo_cb(self, msg):
        self.gazebo_force = msg.data
        alpha = self.get_parameter('lp_filter_alpha').value
        self.filtered_gazebo = alpha * self.gazebo_force + (1 - alpha) * self.filtered_gazebo
        self.last_gazebo_time = self.get_clock().now()

    def loop(self):
        timeout = self.get_parameter('gazebo_timeout_sec').value
        dt = (self.get_clock().now() - self.last_gazebo_time).nanoseconds / 1e9
        
        delta_msg = Float32()
        if dt > timeout:
            delta_msg.data = -1.0
        else:
            delta = self.real_fsr - self.filtered_gazebo
            delta_msg.data = float(delta)
            
            warn_th = self.get_parameter('warn_threshold_N').value
            err_th = self.get_parameter('error_threshold_N').value
            
            if abs(delta) > err_th:
                self.get_logger().error(f"Twin accuracy failure! Delta: {delta:.2f}N")
            elif abs(delta) > warn_th:
                self.get_logger().warn(f"Twin diverging! Delta: {delta:.2f}N")
                
        self.delta_pub.publish(delta_msg)
        self.sim_force_pub.publish(Float32(data=float(self.filtered_gazebo)))

def main(args=None):
    rclpy.init(args=args)
    node = FsrGazeboSync()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
