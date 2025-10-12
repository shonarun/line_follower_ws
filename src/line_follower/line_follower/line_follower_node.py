#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info('Line Follower Node has been started.')

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()