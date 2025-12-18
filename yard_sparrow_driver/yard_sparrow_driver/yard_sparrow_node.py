#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math
import time

from yard_sparrow_driver.imu_utils import get_toggle_command


class YardSparrowDriver(Node):
    def __init__(self):
        super().__init__('yard_sparrow_driver')

        # --- Parameters ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_diameter_m', 0.1651) # 6.5 inches
        self.declare_parameter('track_width_m', 0.523) # Distance between wheels (523mm)

        # --- Robot State (Odometry) ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_odom_time = self.get_clock().now()
        
        # --- Internal State ---
        self.target_speed = 0
        self.target_turn = 0
        self.last_cmd_time = self.get_clock().now()
        
        # --- Extraneous IMU Output Tracking ---
        # Queue of (prefix_char, timestamp) for non-quaternion IMU lines
        self.extraneous_lines = []  # List of tuples: (cmd_char, timestamp_ns)
        self.last_toggle_sent = {}  # Dict: {cmd_char: last_sent_timestamp_ns} for debouncing
        self.debounce_time_ns = 5.0 * 1e9  # 5 seconds debounce
        self.detection_window_ns = 10.0 * 1e9  # 10 seconds window
        self.detection_threshold = 2  # More than 2 occurrences triggers toggle
        
        # --- Quaternion Data Monitoring ---
        self.last_quaternion_time = None  # Timestamp of last received quaternion data
        self.quaternion_timeout_ns = 2.0 * 1e9  # 2 seconds timeout
        self.startup_grace_period_ns = 3.0 * 1e9  # 3 seconds to detect if quaternion is already on
        self.node_start_time = self.get_clock().now()
        self.quaternion_enable_sent = False  # Track if we've sent the enable command
        self.last_quaternion_enable_time_ns = 0  # Debounce for quaternion enable (nanoseconds)
        self.quaternion_enable_debounce_ns = 1.0 * 1e9  # 1 second debounce for quaternion enable
        
        # --- Setup Serial ---
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to Base Station on {port}")
            
            # Wait for serial to settle, then check if quaternion data is already coming
            time.sleep(2)
            # Don't send quaternion enable immediately, prevents toggling it off if it's already on
            self.get_logger().info("Serial connected, monitoring for quaternion data...")
            
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial: {e}")
            self.ser = None

        # --- Subscribers & Publishers ---
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) # For rviz

        # --- Timers ---
        self.create_timer(0.02, self.write_serial_loop) # Heartbeat to wheels at 50Hz
        self.create_timer(0.02, self.read_serial_loop) # Reads feedback from Wheels and IMU at 50Hz
        self.create_timer(1.0, self.monitor_quaternion_data) # Checks if quaternion data is being received at 1Hz

    def send_imu_cmd(self, char_cmd):
        """Helper to send commands to IMU"""
        if self.ser:
            # Format: "[I] x\n"
            cmd_str = f"[I] {char_cmd}\n"
            try:
                self.ser.write(cmd_str.encode('utf-8'))
                self.get_logger().info(f"Sent IMU Command: {cmd_str.strip()}")
            except Exception as e:
                self.get_logger().warn(f"Serial Error: {e}")
    
    def handle_extraneous_imu_output(self, cmd_char):
        """
        Tracks extraneous IMU output lines and sends toggle commands when
        the same prefix appears more than threshold times within the detection window.
        Includes debouncing to prevent multiple toggles.
        """
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        
        # Clean old entries (older than detection window)
        cutoff_time = now_ns - self.detection_window_ns
        self.extraneous_lines = [(char, ts) for char, ts in self.extraneous_lines if ts > cutoff_time]
        
        # Add current line to queue
        self.extraneous_lines.append((cmd_char, now_ns))
        
        # Count occurrences of this prefix in the last 10 seconds
        recent_count = sum(1 for char, ts in self.extraneous_lines 
                          if char == cmd_char and ts > cutoff_time)
        
        # Check if we should send toggle command
        if recent_count > self.detection_threshold:
            # Check debounce: don't send if we sent this command recently
            last_sent = self.last_toggle_sent.get(cmd_char, 0)
            time_since_last = now_ns - last_sent
            
            if time_since_last > self.debounce_time_ns:
                # Send toggle command
                self.send_imu_cmd(cmd_char)
                self.last_toggle_sent[cmd_char] = now_ns
                self.get_logger().warn(
                    f"Detected {recent_count} occurrences of '{cmd_char}' output "
                    f"in last 10s, sending toggle command"
                )

    def cmd_vel_callback(self, msg):
        # 1. Update the "Target" state (Don't send yet)
        self.last_cmd_time = self.get_clock().now()
        
        wheel_dia = self.get_parameter('wheel_diameter_m').value
        track_width = self.get_parameter('track_width_m').value
        
        linear_x = msg.linear.x   # m/s
        angular_z = msg.angular.z # rad/s
        
        linear_val = (linear_x * 6.0) / (math.pi * wheel_dia) 
        
        turn_val = (angular_z * track_width * 6.0) / (math.pi * wheel_dia)

        # Reverse wheels
        # linear_val = -linear_val
        turn_val = -turn_val

        self.target_speed = int(max(min(linear_val, 1000), -1000))
        self.target_turn = int(max(min(turn_val, 1000), -1000))

    def write_serial_loop(self):
        if not self.ser: return

        # Safety Timeout: If no cmd_vel for 0.5s, stop the robot
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds > 0.5 * 1e9:
            self.target_speed = 0
            self.target_turn = 0

        # FORMAT: "[W] S:{speed},T:{turn}\n"
        command = f"[W] S:{self.target_speed},T:{self.target_turn}\n"
        
        try:
            self.ser.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Serial Write Error: {e}")

    def read_serial_loop(self):
        if not self.ser: return

        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line: continue
                
                # --- CASE 1: Wheel Feedback ---
                # Format: "[W] V:42.0 L:100 R:100 T:30.0"
                if line.startswith("[W]"):
                    # self.get_logger().info(f"Odom: {line}")
                    self.calculate_odometry(line)

                # --- CASE 2: IMU Data ---
                # Format: "[I] qW:0.99 qX:0.01..."
                elif line.startswith("[I]"):
                    self.parse_imu(line)

                # else: # Print the broken lines for debugging
                #     self.get_logger().warn(f"Unknown line: {line}")
                    
            except Exception as e:
                pass

    def calculate_odometry(self, line):
        """
        Parses speed feedback and updates robot position (x, y, theta).
        """
        try:
            # DEBUG: Uncomment this to verify data is reaching this function
            # self.get_logger().info(f"Parsing Odom Line: {line}")

            # Parse Format: "[W] V:42.0V L:100 R:100 T:30.0C"
            # We strip unit characters 'V' and 'C' to prevent int() conversion errors
            clean_line = line.replace('V', '').replace('C', '')
            parts = clean_line[4:].split() # Remove "[W] " and split by spaces
            
            speed_l = 0
            speed_r = 0
            
            for p in parts:
                if p.startswith("L:"):
                    try:
                        speed_l = int(p.split(':')[1])
                    except ValueError:
                        pass
                elif p.startswith("R:"):
                    try:
                        speed_r = int(p.split(':')[1])
                    except ValueError:
                        pass

            # Invert right wheel speed
            speed_r = -speed_r

            # 1. Convert Raw Speed to m/s
            # Note: Ensure this divider (60.0 vs 6.0) matches your firmware scaling!
            wheel_dia = self.get_parameter('wheel_diameter_m').value
            track_width = self.get_parameter('track_width_m').value
            
            v_l = (speed_l * math.pi * wheel_dia) / 60.0
            v_r = (speed_r * math.pi * wheel_dia) / 60.0
            
            # 2. Calculate Robot Velocity
            v_x = (v_r + v_l) / 2.0  # Linear velocity
            v_th = (v_r - v_l) / track_width # Angular velocity

            # 3. Integrate over time (dt)
            current_time = self.get_clock().now()
            dt = (current_time - self.last_odom_time).nanoseconds / 1e9
            self.last_odom_time = current_time

            # Handle lag spikes (reset dt if too large)
            if dt > 1.0: 
                dt = 0.0

            delta_x = (v_x * math.cos(self.th)) * dt
            delta_y = (v_x * math.sin(self.th)) * dt
            delta_th = v_th * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            # 4. Create Quaternion for Odom
            q = self.euler_to_quaternion(0, 0, self.th)

            # 5. Publish Transform (odom -> base_link)
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

            # 6. Publish Odometry Message
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            
            # Position
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = q
            
            # Velocity
            odom.twist.twist.linear.x = v_x
            odom.twist.twist.angular.z = v_th
            
            
            self.odom_pub.publish(odom)

        except Exception as e:
            self.get_logger().warn(f"Odom Calc Error: {e}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def parse_imu(self, line):

        # Print the raw line
        # self.get_logger().info(f"{line}")

        # Remove "[I] " prefix
        raw = line[4:].strip()
        
        # Check if it's Quaternion data
        if "qW:" in raw:
            try:
                # Parse format: qW:12345 qX:23456...
                parts = raw.split()
                data = {}
                for p in parts:
                    if ':' in p:
                        key, val = p.split(':')
                        data[key] = int(val)
                
                # DMP Scaling Factor (2^30)
                scale = 1073741824.0
                
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"
                
                imu_msg.orientation.w = data.get('qW', 0) / scale
                imu_msg.orientation.x = data.get('qX', 0) / scale
                imu_msg.orientation.y = data.get('qY', 0) / scale
                imu_msg.orientation.z = data.get('qZ', 0) / scale
                
                # Publish
                self.imu_pub.publish(imu_msg)
                
                # Update quaternion reception timestamp
                self.last_quaternion_time = self.get_clock().now()
                
            except ValueError:
                pass

        # Track lines that match known prefixes but aren't quaternion data
        toggle_cmd = get_toggle_command(raw)
        if toggle_cmd is not None and toggle_cmd != 'q':
            self.get_logger().info(f"{raw}")
            self.handle_extraneous_imu_output(toggle_cmd)
    
    def monitor_quaternion_data(self):
        """
        Monitors quaternion data reception and automatically re-enables it if it stops.
        Handles startup case where quaternion might already be enabled.
        """
        if not self.ser:
            return
        
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        time_since_start = (now - self.node_start_time).nanoseconds
        
        # During startup grace period, check if quaternion data is already coming
        if time_since_start < self.startup_grace_period_ns:
            if self.last_quaternion_time is not None:
                # Quaternion data is already coming, don't send enable command
                self.get_logger().info("Quaternion data detected during startup, assuming already enabled")
                self.quaternion_enable_sent = True  # Mark as sent to prevent toggling
            elif not self.quaternion_enable_sent:
                # No quaternion data yet, send enable command after a short delay
                if time_since_start > 1.0 * 1e9:  # Wait 1 second into startup
                    self.send_imu_cmd('q')
                    self.quaternion_enable_sent = True
                    self.last_quaternion_enable_time_ns = now_ns
                    self.get_logger().info("No quaternion data detected, sending enable command")
            return
        
        # After startup grace period, monitor for missing quaternion data
        if self.last_quaternion_time is None:
            # Never received quaternion data, send enable command
            if not self.quaternion_enable_sent or \
               (now_ns - self.last_quaternion_enable_time_ns) > self.quaternion_enable_debounce_ns:
                self.send_imu_cmd('q')
                self.last_quaternion_enable_time_ns = now_ns
                self.get_logger().warn("No quaternion data ever received, sending enable command")
            return
        
        # Check if quaternion data has stopped
        time_since_last_quat = (now - self.last_quaternion_time).nanoseconds
        
        if time_since_last_quat > self.quaternion_timeout_ns:
            # Quaternion data has stopped, re-enable it (with debounce)
            if (now_ns - self.last_quaternion_enable_time_ns) > self.quaternion_enable_debounce_ns:
                self.send_imu_cmd('q')
                self.last_quaternion_enable_time_ns = now_ns
                self.get_logger().warn(
                    f"Quaternion data timeout ({time_since_last_quat / 1e9:.1f}s), "
                    f"re-enabling quaternion output"
                )



def main(args=None):
    rclpy.init(args=args)
    node = YardSparrowDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()