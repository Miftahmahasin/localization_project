#!/usr/bin/env python3
"""
Set AMCL Initial Pose
Manually trigger AMCL to start localizing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Wait for publisher to be ready
        time.sleep(1.0)
        
        # Set initial pose at field center
        self.set_initial_pose(x=0.0, y=0.0, yaw=0.0)
        
    def set_initial_pose(self, x, y, yaw):
        """Set initial pose in map frame"""
        
        msg = PoseWithCovarianceStamped()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Orientation (yaw to quaternion)
        import math
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Covariance (relatively high uncertainty)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.5 * 0.5   # x variance
        msg.pose.covariance[7] = 0.5 * 0.5   # y variance
        msg.pose.covariance[35] = 0.3 * 0.3  # yaw variance
        
        # Publish
        self.publisher.publish(msg)
        
        self.get_logger().info('='*60)
        self.get_logger().info('📍 Initial Pose Published!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Position: ({x:.2f}, {y:.2f})')
        self.get_logger().info(f'Orientation: {math.degrees(yaw):.1f}°')
        self.get_logger().info(f'Frame: {msg.header.frame_id}')
        self.get_logger().info('='*60)
        self.get_logger().info('')
        self.get_logger().info('Check if AMCL responds:')
        self.get_logger().info('  ros2 topic echo /particlecloud --once')
        self.get_logger().info('')
        self.get_logger().info('If particles appear, AMCL is working!')
        self.get_logger().info('='*60)

def main(args=None):
    rclpy.init(args=args)
    
    node = InitialPoseSetter()
    
    # Keep alive for a moment
    time.sleep(2.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()