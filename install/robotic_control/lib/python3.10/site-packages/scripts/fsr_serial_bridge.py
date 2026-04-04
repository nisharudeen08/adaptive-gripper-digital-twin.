#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import JointState
import serial
import json
import time

class SerialBridge(Node):
    def __init__(self):
        super().__init__('fsr_serial_bridge')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        self.force_pub = self.create_publisher(Float32, '/gripper/fsr/force', 10)
        self.pos_pub = self.create_publisher(JointState, '/gripper/servo/position', 10)
        self.stat_pub = self.create_publisher(String, '/gripper/hardware/status', 10)
        
        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.get_logger().info("Waiting for hardware handshake...")
            start = time.time()
            ready = False
            while time.time() - start < 5.0:
                line = self.ser.readline().decode('utf-8').strip()
                if "READY" in line:
                    self.get_logger().info("Hardware connected!")
                    ready = True
                    break
            if not ready: self.get_logger().error("🔴 ERROR: Handshake failed")
        except Exception as e:
            self.get_logger().error(f"Serial port err: {e}")
            
        self.timer = self.create_timer(0.005, self.read_serial) # > 100Hz
        self.last_msg = time.time()

    def read_serial(self):
        if not hasattr(self, 'ser'): return
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                data = json.loads(line)
                
                self.force_pub.publish(Float32(data=float(data['fsr'])))
                
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = ['gripper_joint']
                js.position = [float(data['pos'])]
                self.pos_pub.publish(js)
                
                self.stat_pub.publish(String(data="HARDWARE_OK"))
                self.last_msg = time.time()
                
            except Exception:
                pass
                
        if time.time() - self.last_msg > 2.0:
            self.stat_pub.publish(String(data="HARDWARE_ERROR"))

def main():
    rclpy.init()
    node = SerialBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': main()
