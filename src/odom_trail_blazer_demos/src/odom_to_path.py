#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPathNode(Node):
    """
    A node that subscribes to odometry messages and publishes them as a path
    """

    def __init__(self):
        super().__init__('odom_to_path_node')
        
        self.declare_parameter('odom_topic', '/lid_odom')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('frame_id', 'lid_odom')
        self.declare_parameter('max_path_length', 100)  # Maximum number of poses to keep in path
        
        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.max_path_length = self.get_parameter('max_path_length').value
        
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        
        self.path_publisher = self.create_publisher(
            Path,
            self.path_topic,
            10)
        
        # Subscribe to odometry topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10)
        
        self.get_logger().info(f'OdomToPath node initialized. Subscribing to: {self.odom_topic}')
        self.get_logger().info(f'Publishing path on: {self.path_topic}')
        self.get_logger().info(f'Maximum path length: {self.max_path_length}')

    def odom_callback(self, msg):
        """
        Callback function for odometry messages
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.path.poses.append(pose_stamped)
        if len(self.path.poses) > self.max_path_length:
            self.path.poses = self.path.poses[-self.max_path_length:]
        
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    
    node = OdomToPathNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
