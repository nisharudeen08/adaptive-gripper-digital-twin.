#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import random
import math

class FakeSensorPublisher(Node):
    def __init__(self):
        super().__init__('fake_sensor_publisher')
        
        # Parameters
        self.declare_parameter('sim_mode', 'realistic')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('noise_std_fsr', 0.05)
        self.declare_parameter('noise_std_servo', 0.01)
        self.declare_parameter('force_max', 5.0)
        self.declare_parameter('cycle_duration', 8.0)
        
        self.sim_mode = self.get_parameter('sim_mode').value
        self.rate = self.get_parameter('publish_rate_hz').value
        self.noise_fsr = self.get_parameter('noise_std_fsr').value
        self.noise_servo = self.get_parameter('noise_std_servo').value
        self.force_max = self.get_parameter('force_max').value
        self.cycle_dur = self.get_parameter('cycle_duration').value
        
        # QoS setup for sensor data
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.force_pub = self.create_publisher(Float32, '/gripper/fsr/force', qos_profile)
        self.pos_pub = self.create_publisher(JointState, '/gripper/servo/position', qos_profile)
        
        # Timer
        self.timer = self.create_timer(1.0/self.rate, self.timer_cb)
        self.time_start = self.get_clock().now().nanoseconds / 1e9
        
        # Mode dictionary mapping strings to handler functions
        self.modes = {
            'ramp': self.mode_ramp,
            'step': self.mode_step,
            'noise': self.mode_noise,
            'overforce': self.mode_overforce,
            'realistic': self.mode_realistic
        }
        
        self.get_logger().info(f"Started Fake Sensor Publisher. Mode: [{self.sim_mode}]")
        self.last_mode = self.sim_mode

    def timer_cb(self):
        # Handle param updates dynamically
        current_mode = self.get_parameter('sim_mode').value
        if current_mode != self.last_mode and current_mode in self.modes:
            self.get_logger().info(f"Transitioning mode: {self.last_mode} -> {current_mode}")
            self.last_mode = current_mode
            self.time_start = self.get_clock().now().nanoseconds / 1e9
            self.sim_mode = current_mode
            
        t = (self.get_clock().now().nanoseconds / 1e9) - self.time_start
        
        func = self.modes.get(self.sim_mode, self.mode_realistic)
        force, pos = func(t)
        
        # Add noise
        force += random.gauss(0, self.noise_fsr)
        pos += random.gauss(0, self.noise_servo)
        
        # Ensure force >= 0
        force = max(0.0, force)
        
        # Publish
        msg_f = Float32()
        msg_f.data = float(force)
        self.force_pub.publish(msg_f)
        
        msg_p = JointState()
        msg_p.header.stamp = self.get_clock().now().to_msg()
        msg_p.name = ['gripper_joint']
        msg_p.position = [float(pos)]
        self.pos_pub.publish(msg_p)

    def mode_ramp(self, t):
        t_mod = t % 7.0
        if t_mod < 5.0:
            f = (t_mod / 5.0) * self.force_max
        else:
            f = self.force_max - ((t_mod - 5.0) / 2.0) * self.force_max
        pos = (f / 5.0) * 1.57
        return f, pos

    def mode_step(self, t):
        t_mod = t % 8.0
        if t_mod < 2.0: return 0.0, 0.0
        elif t_mod < 4.0: return 1.0, 0.3
        elif t_mod < 6.0: return 3.0, 0.9
        else: return 0.0, 0.0

    def mode_noise(self, t):
        return 2.0, 1.0

    def mode_overforce(self, t):
        t_mod = t % 10.0
        if t_mod < 8.0:
            return (t_mod / 8.0) * 12.0, 1.57
        return 0.0, 0.0

    def mode_realistic(self, t):
        t_mod = t % self.cycle_dur
        if t_mod < 1.0: return 0.0, 0.0
        elif t_mod < 2.0: return (t_mod - 1.0) * 0.5, (t_mod - 1.0) * 0.5
        elif t_mod < 5.0: return 2.0, 0.8
        elif t_mod < 6.0: return 2.0 * (1.0 - (t_mod - 5.0)), 0.8 * (1.0 - (t_mod - 5.0))
        else: return 0.0, 0.0

def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
