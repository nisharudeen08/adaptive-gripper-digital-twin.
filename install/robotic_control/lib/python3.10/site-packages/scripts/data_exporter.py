#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Float32MultiArray
from sensor_msgs.msg import JointState
import os
import csv
import time
from datetime import datetime
import math

class DataExporter(Node):
    def __init__(self):
        super().__init__('data_exporter')
        
        # State variables
        self.fsr = 0.0
        self.servo = 0.0
        self.state = "UNKNOWN"
        self.target = 0.0
        self.pid_out = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.twin_delta = 0.0
        self.hw_status = "SIMULATION"
        
        home = os.path.expanduser('~')
        log_dir = os.path.join(home, 'gripper_logs')
        os.makedirs(log_dir, exist_ok=True)
        
        phase = 'phase1' # By default
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(log_dir, f"{phase}_session_{ts}.csv")
        self.get_logger().info(f"Logging to: {self.filepath}")
        
        self.file = open(self.filepath, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['ros_timestamp', 'wall_time', 'fsr_force_N', 'servo_pos_rad', 
                              'servo_pos_deg', 'grip_state', 'target_force_N', 'pid_output',
                              'p_term', 'i_term', 'd_term', 'twin_delta_N', 'hw_status'])
        self.buffer = []
        
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(Float32, '/gripper/fsr/force', self.fsr_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, '/gripper/servo/position', self.servo_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/gripper/grip_state', self.state_cb, 10)
        self.create_subscription(Float32MultiArray, '/gripper/control/debug', self.debug_cb, 10)
        self.create_subscription(Float32, '/gripper/twin_delta', self.twin_cb, 10)
        self.create_subscription(String, '/gripper/hardware/status', self.hw_cb, 10)
        
        self.timer = self.create_timer(0.05, self.log_row) # 20Hz logging

    def fsr_cb(self, msg): self.fsr = msg.data
    def servo_cb(self, msg): 
        if msg.position: self.servo = msg.position[0]
    def state_cb(self, msg): self.state = msg.data
    def debug_cb(self, msg):
        if len(msg.data) >= 6:
            self.target, _, _, self.p_term, self.i_term, self.d_term = msg.data[0:6]
            self.pid_out = self.p_term + self.i_term + self.d_term
    def twin_cb(self, msg): self.twin_delta = msg.data
    def hw_cb(self, msg): 
        self.hw_status = msg.data
        if "HARDWARE" in self.hw_status and "phase1" in self.filepath:
            pass

    def log_row(self):
        ros_time = self.get_clock().now().nanoseconds / 1e9
        wall_time = time.time()
        self.buffer.append([ros_time, wall_time, self.fsr, self.servo, math.degrees(self.servo),
                            self.state, self.target, self.pid_out, self.p_term, self.i_term, 
                            self.d_term, self.twin_delta, self.hw_status])
                            
        if len(self.buffer) >= 100:
            self.writer.writerows(self.buffer)
            self.file.flush()
            self.buffer.clear()

    def destroy_node(self):
        if self.buffer:
            self.writer.writerows(self.buffer)
        self.file.close()
        self.get_logger().info(f"Session saved to: {self.filepath}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
