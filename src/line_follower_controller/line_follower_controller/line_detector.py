#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        self.declare_parameter('threshold_value', 50)
        self.declare_parameter('cropping_ratio', 0.7)

        self.threshold_value : int = self.get_parameter('threshold_value').value
        self.cropping_ratio : float = self.get_parameter('cropping_ratio').value

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        self.error_pub = self.create_publisher(Float64, '/line_error', 10)

    def listener_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, self.threshold_value, 255, cv2.THRESH_BINARY_INV)

        height, width = frame.shape[:2]
        roi = binary[int(height * self.cropping_ratio):height, :]

        M = cv2.moments(roi)
        error_msg = Float64()

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M["m01"] / M["m00"]) + int(height*self.cropping_ratio)
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
            frame_center = width // 2
            error = float(cx - frame_center)
            error_msg.data = error
            self.error_pub.publish(error_msg)
            self.get_logger().info(f"Line center: {cx}, Error: {error}")
        else:
            self.get_logger().warn("Line not found!")
            error_msg.data = float('nan')
            self.error_pub.publish(error_msg)

        try:
            cv2.imshow('Line Detection', frame)
            cv2.waitKey(1)
        except Exception:
            pass

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
