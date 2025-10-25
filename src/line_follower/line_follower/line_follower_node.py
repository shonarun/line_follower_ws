#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.subscription = self.create_subscription(Image, '/visuals/image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()

    def listener_callback(self, data: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            cv2.imshow("Visuals", frame)
            cv2.waitKey(1)
        except Exception as e:
            return

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()