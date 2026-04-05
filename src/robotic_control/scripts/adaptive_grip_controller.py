#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Float32MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class AdaptiveGripController(Node):
    def __init__(self):
        super().__init__('adaptive_grip_controller')
        
        # Parameters
        self.declare_parameter('target_force', 2.0)
        self.declare_parameter('force_deadband', 0.1)
        self.declare_parameter('kp', 0.05)
        self.declare_parameter('ki', 0.001)
        self.declare_parameter('kd', 0.01)
        self.declare_parameter('max_integral', 0.5)
        self.declare_parameter('deriv_filter_alpha', 0.2)
        self.declare_parameter('max_grip_pos', 1.57)
        self.declare_parameter('min_grip_pos', 0.0)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('joint_names', ['finger_left_joint'])
        
        # State vars
        self.fsr_force = 0.0
        self.servo_pos = 0.0
        self.grip_state = "OPEN"
        
        # PID vars
        self.integral = 0.0
        self.prev_error = 0.0
        self.deriv_filtered = 0.0
        
        # Subs
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(Float32, '/gripper/fsr/force', self.force_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, '/gripper/servo/position', self.servo_cb, qos_profile_sensor_data)
        
        # Pubs
        self.traj_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.state_pub = self.create_publisher(String, '/gripper/grip_state', 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '/gripper/control/debug', 10)
        
        rate = self.get_parameter('control_rate_hz').value
        self.timer = self.create_timer(1.0/rate, self.control_loop)
        
    def force_cb(self, msg):
        self.fsr_force = msg.data

    def servo_cb(self, msg):
        if len(msg.position) > 0:
            self.servo_pos = msg.position[0]

    def reset_pid(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.deriv_filtered = 0.0

    def control_loop(self):
        target_f = self.get_parameter('target_force').value
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        max_i = self.get_parameter('max_integral').value
        alpha = self.get_parameter('deriv_filter_alpha').value
        p_max = self.get_parameter('max_grip_pos').value
        p_min = self.get_parameter('min_grip_pos').value
        joint_names = self.get_parameter('joint_names').value
        
        new_state = self.grip_state
        
        # State Machine Transitions
        if self.fsr_force > 10.0:
            new_state = "EMERGENCY"
        elif self.grip_state == "EMERGENCY" and self.fsr_force > 0.5:
            new_state = "EMERGENCY" # Latch
        elif self.fsr_force < 0.1:
            new_state = "OPEN"
        elif self.fsr_force >= 2.0:
            new_state = "GRASPING"
        else:
            new_state = "CONTACT"
            
        if new_state != self.grip_state:
            self.get_logger().info(f"State transition: {self.grip_state} -> {new_state}")
            if new_state == "EMERGENCY":
                self.get_logger().error("CRITICAL: Emergency stop triggered! Force > 10N")
            self.reset_pid()
            self.grip_state = new_state
            
        self.state_pub.publish(String(data=self.grip_state))
        
        # Action execution based on state
        cmd_pos = self.servo_pos
        error = 0.0
        p_term = i_term = d_term = 0.0
        
        if self.grip_state == "OPEN":
            cmd_pos = p_min
        elif self.grip_state == "CONTACT":
            cmd_pos = 0.5 # Pre-grasp rad
        elif self.grip_state == "GRASPING":
            error = target_f - self.fsr_force
            p_term = kp * error
            self.integral += error
            self.integral = max(-max_i, min(max_i, self.integral)) # Anti-windup
            i_term = ki * self.integral
            
            raw_d = error - self.prev_error
            self.deriv_filtered = alpha * raw_d + (1 - alpha) * self.deriv_filtered # Low pass D
            d_term = kd * self.deriv_filtered
            
            pid_out = p_term + i_term + d_term
            cmd_pos = self.servo_pos + pid_out
            self.prev_error = error
            
        elif self.grip_state == "EMERGENCY":
            cmd_pos = p_min
            
        cmd_pos = max(p_min, min(p_max, cmd_pos))
        
        # Debug pub
        dbg = Float32MultiArray()
        dbg.data = [float(target_f), float(self.fsr_force), float(error), 
                    float(p_term), float(i_term), float(d_term), 
                    float(cmd_pos - self.servo_pos), float(self.servo_pos)]
        self.debug_pub.publish(dbg)
        
        # Trajectory command (Option 1: left finger only)
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(cmd_pos)]
        point.time_from_start = Duration(sec=0, nanosec=100000000) # 0.1s
        traj.points.append(point)
        self.traj_pub.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveGripController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
