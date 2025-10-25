#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import time
import math

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        # PID and motion parameters
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('kp', 0.01)
        self.declare_parameter('ki', 0.005)
        self.declare_parameter('kd', 0.02)
        self.declare_parameter('max_angular', 1.0)

        self.linear_speed : float = self.get_parameter('linear_speed').value
        self.kp : float = self.get_parameter('kp').value
        self.ki : float = self.get_parameter('ki').value
        self.kd : float = self.get_parameter('kd').value
        self.max_angular : float = self.get_parameter('max_angular').value

        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        self.line_lost_counter = 0
        self.line_lost_max = 300

        self.subscription = self.create_subscription(Float64, '/line_error', self.error_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def error_callback(self, msg: Float64):
        error = msg.data
        current_time = time.time()
        cmd = Twist()

        if not math.isnan(error):
            dt = current_time - self.last_time if self.last_time else 0.01
            self.integral += error * dt
            derivative = (error - self.last_error) / dt if dt > 0 else 0.0

            angular_z = -(self.kp * error + self.ki * self.integral + self.kd * derivative)
            # angular_z = max(min(angular_z, self.max_angular), -self.max_angular)

            cmd.linear.x = self.linear_speed
            cmd.angular.z = angular_z

            self.last_error = error
            self.last_time = current_time
            self.line_lost_counter = 0

            if abs(angular_z) > self.max_angular:
                cmd.linear.x = 0.0
                cmd.angular.z = max(min(angular_z, self.max_angular), -self.max_angular)
        else:
            # Lost line — spin to search
            self.line_lost_counter += 1
            if self.line_lost_counter < self.line_lost_max:
                self.get_logger().info("Line lost, searching...")
                cmd.linear.x = 0.0
                cmd.angular.z = - 0.3 * self.integral / abs(self.integral)
            elif self.line_lost_counter == self.line_lost_max:
                self.get_logger().info("Line lost too long! Stopping.")
                cmd = Twist()

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
