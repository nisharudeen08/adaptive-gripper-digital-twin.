#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Float32MultiArray
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor
import time
import math

from rich.live import Live
from rich.table import Table
from rich import box
from rich.panel import Panel
from rich.align import Align

class GripperMonitor(Node):
    def __init__(self):
        super().__init__('gripper_monitor')
        
        self.fsr = 0.0
        self.servo = 0.0
        self.state = "UNKNOWN"
        self.target = 0.0
        self.error = 0.0
        self.pid_out = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.twin_delta = 0.0
        self.hw_status = "SIMULATION"
        self.sim_mode = "unknown"
        
        self.msg_count = 0
        self.start_time = time.time()
        self.last_hz_time = time.time()
        self.hz = 0.0
        
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(Float32, '/gripper/fsr/force', self.fsr_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, '/gripper/servo/position', self.servo_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/gripper/grip_state', self.state_cb, 10)
        self.create_subscription(Float32MultiArray, '/gripper/control/debug', self.debug_cb, 10)
        self.create_subscription(Float32, '/gripper/twin_delta', self.twin_cb, 10)
        self.create_subscription(String, '/gripper/hardware/status', self.hw_cb, 10)
        
    def fsr_cb(self, msg): 
        self.fsr = msg.data
        self.msg_count += 1
    def servo_cb(self, msg): 
        if msg.position: self.servo = msg.position[0]
    def state_cb(self, msg): self.state = msg.data
    def debug_cb(self, msg):
        if len(msg.data) >= 6:
            self.target, _, self.error, self.p_term, self.i_term, self.d_term = msg.data[0:6]
            self.pid_out = self.p_term + self.i_term + self.d_term
    def twin_cb(self, msg): self.twin_delta = msg.data
    def hw_cb(self, msg): self.hw_status = msg.data

    def generate_table(self):
        now = time.time()
        if now - self.last_hz_time >= 1.0:
            self.hz = self.msg_count / (now - self.last_hz_time)
            self.msg_count = 0
            self.last_hz_time = now
            
        uptime = int(now - self.start_time)
        up_s = f"{uptime//3600:02d}:{(uptime%3600)//60:02d}:{uptime%60:02d}"
        
        t = Table(box=box.DOUBLE, show_header=False, expand=True)
        t.add_column("Key")
        t.add_column("Value")
        
        # Color state
        sc = "white"
        if self.state == "OPEN": sc = "blue"
        elif self.state == "CONTACT": sc = "yellow"
        elif self.state == "GRASPING": sc = "green"
        elif self.state == "EMERGENCY": sc = "red"
        
        # Force Bar
        f_pct = min(100, int((self.fsr/10.0)*100))
        bars = int(f_pct / 10)
        bar_str = "▓"*bars + "░"*(10-bars)
        fc = "green" if self.fsr < 3 else ("yellow" if self.fsr <= 8 else "red")
        
        hwc = "green"
        if self.hw_status != "SIMULATION": hwc = "cyan"
        
        t.add_row("FSR Force", f"[{fc}]{self.fsr:.2f} N  [{bar_str}] {f_pct}%[/{fc}]")
        t.add_row("Servo Position", f"{math.degrees(self.servo):.1f}°  ({self.servo:.3f} rad)")
        t.add_row("Grip State", f"[{sc}]{self.state}[/{sc}]")
        t.add_section()
        t.add_row("Target Force", f"{self.target:.2f} N")
        t.add_row("Force Error", f"{self.error:+.2f} N")
        t.add_row("PID Output", f"{self.pid_out:.3f} rad")
        t.add_row(" └─ P term", f"{self.p_term:.3f}")
        t.add_row(" └─ I term", f"{self.i_term:.3f}")
        t.add_row(" └─ D term", f"{self.d_term:.3f}")
        t.add_section()
        
        tc = "green" if abs(self.twin_delta)<0.5 else ("yellow" if abs(self.twin_delta)<=2.0 else "red")
        if self.twin_delta == -1.0:
            t.add_row("Twin Delta", "Gazebo Disconnected")
        else:
            t.add_row("Twin Delta", f"[{tc}]{self.twin_delta:+.2f} N[/{tc}]")
            
        t.add_row("HW Status", f"[{hwc}]{self.hw_status}[/{hwc}]")
        t.add_section()
        t.add_row("Uptime", up_s)
        t.add_row("Msg Rate", f"{self.hz:.1f} Hz")
        
        title = "Phase 1 - Simulation Mode" if self.hw_status == "SIMULATION" else "Phase 2 - HARDWARE MODE"
        return Panel(Align.center(t), title=f"ADAPTIVE GRIPPER DIGITAL TWIN\n{title}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperMonitor()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    
    with Live(node.generate_table(), refresh_per_second=2.0) as live:
        try:
            while rclpy.ok():
                exec_.spin_once(timeout_sec=0.1)
                live.update(node.generate_table())
        except KeyboardInterrupt:
            pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
