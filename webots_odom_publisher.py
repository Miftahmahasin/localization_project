#!/usr/bin/env python3
"""
Webots Odometry Publisher
Publishes odometry from Webots supervisor info
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class WebotsOdomPublisher(Node):
    def __init__(self):
        super().__init__('webots_odom_publisher')
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer - publish at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_odom)
        
        # Try to import Webots
        try:
            from controller import Supervisor
            self.supervisor = Supervisor()
            self.robot_node = self.supervisor.getFromDef('ROBOTIS_OP3')
            if self.robot_node is None:
                self.get_logger().error('Robot node ROBOTIS_OP3 not found!')
                self.use_webots = False
            else:
                self.use_webots = True
                self.get_logger().info('✅ Webots supervisor initialized')
        except:
            self.get_logger().warn('Not running in Webots, using static odometry')
            self.use_webots = False
            
        self.seq = 0
        
    def publish_odom(self):
        now = self.get_clock().now()
        
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        if self.use_webots:
            # Get position from Webots
            position = self.robot_node.getPosition()
            orientation = self.robot_node.getOrientation()
            
            # Position
            odom_msg.pose.pose.position.x = position[0]
            odom_msg.pose.pose.position.y = position[1]
            odom_msg.pose.pose.position.z = position[2]
            
            # Convert rotation matrix to quaternion
            # orientation is a 9-element rotation matrix [R11, R12, R13, R21, R22, R23, R31, R32, R33]
            R11, R12, R13 = orientation[0], orientation[1], orientation[2]
            R21, R22, R23 = orientation[3], orientation[4], orientation[5]
            R31, R32, R33 = orientation[6], orientation[7], orientation[8]
            
            # Convert to quaternion (only yaw matters for 2D)
            yaw = math.atan2(R21, R11)
            
            odom_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            odom_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Get velocity
            velocity = self.robot_node.getVelocity()
            odom_msg.twist.twist.linear.x = velocity[0]
            odom_msg.twist.twist.linear.y = velocity[1]
            odom_msg.twist.twist.angular.z = velocity[5]  # Angular velocity around Z
            
        else:
            # Static odometry fallback
            odom_msg.pose.pose.orientation.w = 1.0
            
        # Covariance (low values for simulation)
        odom_msg.pose.covariance = [
            0.001, 0, 0, 0, 0, 0,
            0, 0.001, 0, 0, 0, 0,
            0, 0, 0.001, 0, 0, 0,
            0, 0, 0, 0.001, 0, 0,
            0, 0, 0, 0, 0.001, 0,
            0, 0, 0, 0, 0, 0.001
        ]
        
        odom_msg.twist.covariance = odom_msg.pose.covariance
        
        # Publish
        self.odom_pub.publish(odom_msg)
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        
        self.seq += 1
        if self.seq % 50 == 0:  # Log every second
            self.get_logger().info(
                f'Odom: ({odom_msg.pose.pose.position.x:.2f}, '
                f'{odom_msg.pose.pose.position.y:.2f})',
                throttle_duration_sec=1.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = WebotsOdomPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()