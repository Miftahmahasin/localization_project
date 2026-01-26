#!/usr/bin/env python3
"""
Static Map to Odom Transform Publisher
Publishes initial map→odom transform to allow AMCL to start
AMCL will take over this transform once it starts localizing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class StaticMapOdomPublisher(Node):
    def __init__(self):
        super().__init__('static_map_odom_publisher')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer - 20 Hz
        self.timer = self.create_timer(0.05, self.publish_transform)
        
        self.get_logger().info('✅ Static map→odom transform publisher started')
        self.get_logger().info('   AMCL will override this once it starts localizing')
        
    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        # Identity transform (map and odom aligned initially)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = StaticMapOdomPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()