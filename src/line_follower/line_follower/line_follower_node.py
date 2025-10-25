#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info('Line Follower Node has been started.')
        self.declare_parameter("number", 2)
        self.declare_parameter("publish_period", 1.0)
        self.x = self.get_parameter("number").value
        self.y = self.get_parameter("publish_period").value
        self.get_logger().info(f"{self.x} : {self.y}")

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()