#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import numpy as np
from collections import deque 
class LineNavigator(Node):
    def __init__(self):
        super().__init__('line_navigator')
        self.declare_parameter('max_linear_vel', 0.2)  
        self.declare_parameter('k_p', 1.5) 
        self.declare_parameter('queue_max_size', 10) 
        self.declare_parameter('pop_distance', 0.3)
        self.declare_parameter('min_point_dist', 0.2)
        self.declare_parameter('lost_timeout', 2.0) 

        self.max_v = self.get_parameter('max_linear_vel').value
        self.k_p = self.get_parameter('k_p').value
        self.queue_max_size = self.get_parameter('queue_max_size').value
        self.pop_distance = self.get_parameter('pop_distance').value
        self.min_point_dist = self.get_parameter('min_point_dist').value
        self.lost_timeout = self.get_parameter('lost_timeout').value

        
        self.point_queue = deque(maxlen=self.queue_max_size)
        self.last_point_time = None 
        self.robot_pose = np.array([0.0, 0.0, 0.0]) 

        self.point_sub = self.create_subscription(PointStamped, '/line_ground_point', self.point_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("Line navigator with point queue initialized.")

    def odom_callback(self, msg):
        # Update robot pose (x, y, yaw)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat_z = msg.pose.pose.orientation.z
        quat_w = msg.pose.pose.orientation.w
        yaw = 2 * np.arctan2(quat_z, quat_w)
        self.robot_pose = np.array([x, y, yaw])

    def point_callback(self, msg):
        if msg.header.frame_id != 'odom':
            return

        current_time = self.get_clock().now()
        self.last_point_time = current_time  

        new_point = np.array([msg.point.x, msg.point.y])
        robot_pos = self.robot_pose[:2]

        
        if len(self.point_queue) == 0 or self._is_ahead(new_point, self.point_queue[-1], robot_pos):
            self.point_queue.append((msg.point.x, msg.point.y, current_time.nanoseconds))
            self.get_logger().info(f"Enqueued point: ({msg.point.x:.2f}, {msg.point.y:.2f}). Queue size: {len(self.point_queue)}")

        
        self._update_queue()

    def _is_ahead(self, new_point, last_queued, robot_pos):
        """Check if new point is further ahead than last in queue (along approximate path direction)."""
        last_pt = np.array([last_queued[0], last_queued[1]])
        robot_to_last = last_pt - robot_pos
        robot_to_new = new_point - robot_pos

        forward_dir = np.array([np.cos(self.robot_pose[2]), np.sin(self.robot_pose[2])])
        dist_last = np.dot(robot_to_last, forward_dir)
        dist_new = np.dot(robot_to_new, forward_dir)
        return dist_new > dist_last + self.min_point_dist

    def _update_queue(self):
        """Pop front points if robot is close enough."""
        if len(self.point_queue) == 0:
            return

        front_pt = np.array([self.point_queue[0][0], self.point_queue[0][1]])
        robot_pos = self.robot_pose[:2]
        dist_to_front = np.linalg.norm(front_pt - robot_pos)

        if dist_to_front < self.pop_distance:
            self.point_queue.popleft()
            self.get_logger().info(f"Reached front point! Queue size now: {len(self.point_queue)}")

    def _publish_commands(self):
        """Main control loop: Chase front of queue. Run via timer for steady updates."""
        current_time = self.get_clock().now()
        if self.last_point_time and (current_time - self.last_point_time).nanoseconds / 1e9 > self.lost_timeout:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().warn("Line lost! Stopping.")
            return

        if len(self.point_queue) == 0:
            twist = Twist()
            twist.linear.x = self.max_v
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        
        target_x, target_y, _ = self.point_queue[0]
        target_pt = np.array([target_x, target_y])
        robot_pos = self.robot_pose[:2]

        
        rel_vec = target_pt - robot_pos
        yaw = self.robot_pose[2]
        rel_x = np.cos(-yaw) * rel_vec[0] - np.sin(-yaw) * rel_vec[1]
        rel_y = np.sin(-yaw) * rel_vec[0] + np.cos(-yaw) * rel_vec[1]

        
        lateral_error = rel_y
        long_dist = rel_x

        
        twist = Twist()
        twist.linear.x = min(self.max_v, self.max_v * (long_dist / 1.0)) 
        twist.angular.z = -self.k_p * np.tanh(lateral_error)  

        self.cmd_pub.publish(twist)
        self.get_logger().debug(f"Chasing ({target_x:.2f}, {target_y:.2f}); lat_err: {lateral_error:.2f}, dist: {np.linalg.norm(rel_vec):.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = LineNavigator()
    timer = node.create_timer(0.05, node._publish_commands)

    rclpy.spin(node)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()