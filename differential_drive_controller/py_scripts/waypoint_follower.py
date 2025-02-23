#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        self.declare_parameter('waypoint_1_x', 2)
        self.declare_parameter('waypoint_1_y', 2)
        self.declare_parameter('waypoint_2_x', 3)
        self.declare_parameter('waypoint_2_y', -3)

        
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.02)
        # uncomment if want different kp ki and kd for angular PID
        # self.declare_parameter('kp_angular', 2.0)
        # self.declare_parameter('ki_angular', 0.0)
        # self.declare_parameter('kd_angular', 0.2)

        self.odom_received = False
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.waypoints = [(self.get_parameter('waypoint_1_x').value, self.get_parameter('waypoint_1_y').value),
                          (self.get_parameter('waypoint_2_x').value, self.get_parameter('waypoint_2_y').value)]
        self.linear_pid = PIDController(
            self.get_parameter('kp').value,
            self.get_parameter('ki').value,
            self.get_parameter('kd').value
            
            # uncomment below and comment above to use different PID constants
            # for linear and angular
            # self.get_parameter('kp_linear').value,
            # self.get_parameter('ki_linear').value,
            # self.get_parameter('kd_linear').value
        )
        self.angular_pid = PIDController(
            self.get_parameter('kp').value,
            self.get_parameter('ki').value,
            self.get_parameter('kd').value
            
            # uncomment below and comment above to use different PID constants
            # for linear and angular
            # self.get_parameter('kp_angular').value,
            # self.get_parameter('ki_angular').value,
            # self.get_parameter('kd_angular').value
        )
        
        self.current_waypoint_index = 0
        self.current_pose = (0.0, 0.0, 0.0)  # (x, y, theta)
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.get_logger().info('Subsrcibed to odom topic')
        self.odom_received = True
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        self.current_pose = (position.x, position.y, yaw)
    
    def quaternion_to_euler(self, x, y, z, w):
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return (0.0, 0.0, yaw)  # Only yaw is needed
    
    def control_loop(self):
        if self.current_waypoint_index >= len(self.waypoints) and not self.odom_received:
            self.stop_robot()
            return
        if self.current_waypoint_index <2:
            target_x, target_y = self.waypoints[self.current_waypoint_index]
            current_x, current_y, current_theta = self.current_pose
        else:
            self.stop_robot()
            return


        distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        angle_error = angle_to_target - current_theta
        self.get_logger().info(f'Remaining distance to next waypoint: {distance}')
        
        if distance < 0.1:  # Threshold for reaching the waypoint
            self.get_logger().info('Reached waypoint')
            self.current_waypoint_index += 1
        
        linear_velocity = self.linear_pid.compute(distance, 0.0)
        angular_velocity = self.angular_pid.compute(angle_error, 0.0)
        
        cmd_msg = Twist()
        cmd_msg.linear.x = max(min(linear_velocity, 0.2), -0.2)
        cmd_msg.angular.z = max(min(angular_velocity, 0.4), -0.4)
        
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info('Moving the robot')
    
    def stop_robot(self):
        cmd_msg = Twist()
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info("Waypoints completed. Robot stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
