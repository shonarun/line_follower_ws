#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist
import time

class CameraSub(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        self.declare_parameter('threshold_value', 50)
        self.declare_parameter('cropping_ratio', 0.7)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('kp', 0.01)
        self.declare_parameter('ki', 0.005)
        self.declare_parameter('kd', 0.02)
        self.declare_parameter('max_angular', 1.0)
        
        self.threshold_value : int = self.get_parameter('threshold_value').value
        self.cropping_ratio : float = self.get_parameter('cropping_ratio').value
        self.linear_speed : float = self.get_parameter('linear_speed').value
        self.kp : float = self.get_parameter('kp').value
        self.ki : float = self.get_parameter('ki').value
        self.kd : float = self.get_parameter('kd').value
        self.max_angular : float = self.get_parameter('max_angular').value

        self.line_lost_counter = 0
        self.line_lost_max = 60

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def listener_callback(self, data: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            
        
        except Exception as e:
            self.get_logger().info(f"Failed to convert image: {e}")
            return
    
        # Convert to grayscale and threshold
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, self.threshold_value, 255, cv2.THRESH_BINARY_INV)

        ## May use this instead.
        # binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)


        # Calculate the center of the white line (naive approach)
        height = frame.shape[0]
        width = frame.shape[1]

        roi = binary[int(height*self.cropping_ratio):height, 0:width]
        M = cv2.moments(roi)
        msg = Twist()

        current_time = time.time()

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"]) + int(height*self.cropping_ratio)

            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)

            frame_center = frame.shape[1] // 2
            error = cx - frame_center
            self.get_logger().info(f"Line center: {cx}, Error: {error}")

            dt = current_time - self.last_time if self.last_time else 0.01
            self.integral += error * dt
            derivative = (error - self.last_error) / dt if dt > 0 else 0.0

            angular_z = -(self.kp * error + self.ki * self.integral + self.kd * derivative)

            self.last_error = error
            self.last_time = current_time
            
            # angular_z = max(min(angular_z, self.max_angular), -self.max_angular)

            msg.linear.x = self.linear_speed
            msg.angular.z = angular_z
            
            if abs(angular_z) > self.max_angular:
                msg.linear.x = 0.0
                msg.angular.z = angular_z

        else:
            self.line_lost_counter += 1
            if self.line_lost_counter < self.line_lost_max: # 🔥 May consider this.
                self.get_logger().info("Line lost, searching...")
                msg.linear.x = 0.0
                msg.angular.z = 0.3  # spin to search
            else:
                self.get_logger().info("Line lost for too long! Stopping.")
                msg = Twist()  # stop
            
        self.cmd_publisher.publish(msg)

        try:
            cv2.imshow("Processed", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().info(f"Display error: {e}")

    # No particular reason here.
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
