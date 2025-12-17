#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class YardSparrowDriver(Node):
    def __init__(self):
        super().__init__('yard_sparrow_driver')

        # --- Parameters ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_diameter_m', 0.1651) # 6.5 inches
        self.declare_parameter('track_width_m', 0.523)      # Distance between wheels (523mm)
        self.last_print_time = self.get_clock().now()

        # --- Setup Serial ---
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to ESP32 on {port} at {baud}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial: {e}")
            self.ser = None

        # --- Subscribers ---
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # --- Timers ---
        self.create_timer(0.05, self.read_serial_loop) # 20 Hz read loop

    def cmd_vel_callback(self, msg):
        if not self.ser:
            return

        # 1. Get Parameters
        wheel_dia = self.get_parameter('wheel_diameter_m').value # 0.1651
        track_width = self.get_parameter('track_width_m').value  # 0.523
        
        # 2. Extract ROS Velocities
        linear_x = msg.linear.x   # m/s
        angular_z = msg.angular.z # rad/s

        # 3. Calculate Speed (RPM)
        # RPM = (m/s * 60) / Circumference
        
        linear_rpm = (linear_x * 6.0) / (math.pi * wheel_dia)
        speed_cmd = int(linear_rpm)

        # 4. Calculate Turn (Differential RPM)
        # Diff_Velocity = Angular * Track_Width
        
        diff_vel_mps = angular_z * track_width
        diff_rpm = (diff_vel_mps * 6.0) / (math.pi * wheel_dia)
        turn_cmd = int(diff_rpm)

        # 5. Clamp and Send
        speed_cmd = max(min(speed_cmd, 1000), -1000)
        turn_cmd = max(min(turn_cmd, 1000), -1000)

        command = f"S:{speed_cmd},T:{turn_cmd}\n"
        
        try:
            self.ser.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Serial Write Error: {e}")

    def read_serial_loop(self):
        if not self.ser:
            return

        # 1. Flush buffer
        lines = []
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline()
                lines.append(line)
            except Exception:
                pass
        
        if not lines:
            return

        # 2. Get newest line
        try:
            last_line = lines[-1].decode('utf-8', errors='ignore').strip()
        except:
            return # Decode error, skip

        # 3. Print every 0.5 seconds
        now = self.get_clock().now()
        if (now - self.last_print_time).nanoseconds > 0.2 * 1e9:
            self.get_logger().info(f"RX: {last_line}")
            self.last_print_time = now

def main(args=None):
    rclpy.init(args=args)
    node = YardSparrowDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()