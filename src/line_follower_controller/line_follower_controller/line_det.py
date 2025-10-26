#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf_transformations
from tf2_ros import Buffer, TransformListener
from tf2_ros.timeout_exception import TimeoutException
from builtin_interfaces.msg import Time

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        self.declare_parameter('threshold_value', 50)
        self.declare_parameter('cropping_ratio', 0.7)
        self.declare_parameter('target_frame', 'odom') 

        self.threshold_value = self.get_parameter('threshold_value').value
        self.cropping_ratio = self.get_parameter('cropping_ratio').value
        self.target_frame = self.get_parameter('target_frame').value

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.listener_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        
        
        self.error_pub = self.create_publisher(Float64, '/line_error', 10)
        self.line_point_pub = self.create_publisher(PointStamped, '/line_ground_point', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.camera_info = None
        self.logger = self.get_logger()

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def listener_callback(self, data):
        if self.camera_info is None:
            self.logger.warn("Camera info not received yet.")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.logger.info(f"Image conversion failed: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, self.threshold_value, 255, cv2.THRESH_BINARY_INV)

        height, width = frame.shape[:2]
        roi_y_start = int(height * self.cropping_ratio)
        roi = binary[roi_y_start:height, :]

        M = cv2.moments(roi)
        error_msg = Float64()

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            roi_cy = int(M['m01'] / M['m00'])
            cy = roi_y_start + roi_cy 
            
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
            
            frame_center = width // 2
            error = float(cx - frame_center)
            error_msg.data = error
            self.error_pub.publish(error_msg)
            self.logger.info(f"Line center: ({cx}, {cy}), Pixel Error: {error}")
            
            # Project to ground
            try:
                ground_point = self.project_to_ground(cx, cy, frame.shape[:2], data.header.stamp)
                if ground_point:
                    self.publish_ground_point(ground_point, data.header.stamp)
                    self.logger.info(f"Projected ground point: {ground_point}")
            except Exception as e:
                self.logger.warn(f"Projection failed: {e}")
        else:
            self.logger.info("Line not found!")
            error_msg.data = float('nan')
            self.error_pub.publish(error_msg)

        try:
            cv2.imshow('Line Detection', frame)
            cv2.waitKey(1)
        except Exception:
            pass

    def project_to_ground(self, u, v, img_shape, stamp):
        """Project pixel (u,v) to ground plane (z=0) in target_frame."""
        # Get camera intrinsics
        K_list = self.camera_info.K
        if len(K_list) != 9 or K_list[0] == 0:
            raise ValueError("Invalid camera intrinsics.")
        K = np.array(K_list).reshape(3, 3)
        
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame, self.camera_info.header.frame_id,
                stamp, rclpy.duration.Duration(seconds=0.1)
            )
        except TimeoutException:
            raise RuntimeError("TF lookup timeout.")
        
        # Extract rotation and translation (world to cam)
        quat = [trans.transform.rotation.x, trans.transform.rotation.y,
                trans.transform.rotation.z, trans.transform.rotation.w]
        R_w2c = tf_transformations.quaternion_matrix(quat)[:3, :3]
        t_w2c = np.array([trans.transform.translation.x,
                          trans.transform.translation.y,
                          trans.transform.translation.z])
        
        # Invert to get cam to world: R_c2w = R_w2c.T, t_c2w = -R_c2w @ t_w2c
        R_c2w = R_w2c.T
        t_c2w = -R_c2w @ t_w2c
        
        # Now project
        h, w = img_shape[:2]
        pix = np.array([u, v, 1.0])
        ray_dir_cam = np.linalg.inv(K) @ pix
        ray_dir_cam /= np.linalg.norm(ray_dir_cam)
        
        cam_center_cam = np.zeros(3)
        cam_center_world = R_c2w @ cam_center_cam + t_c2w
        ray_dir_world = R_c2w @ ray_dir_cam
        
        dz = ray_dir_world[2]
        if abs(dz) < 1e-6:
            raise RuntimeError("Ray parallel to ground plane.")
        
        t = -cam_center_world[2] / dz
        if t < 0:
            raise RuntimeError("Intersection behind camera.")
        
        ground_pt_world = cam_center_world + t * ray_dir_world
        return ground_pt_world[:2]  # (X, Y)

    def publish_ground_point(self, ground_xy, stamp):
        point_msg = PointStamped()
        point_msg.header.stamp = stamp
        point_msg.header.frame_id = self.target_frame
        point_msg.point.x = ground_xy[0]
        point_msg.point.y = ground_xy[1]
        point_msg.point.z = 0.0
        self.line_point_pub.publish(point_msg)

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