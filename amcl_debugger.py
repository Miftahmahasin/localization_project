#!/usr/bin/env python3
"""
AMCL Debug Checker
Check why AMCL is not localizing despite good scan data
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import time

class AMCLDebugger(Node):
    def __init__(self):
        super().__init__('amcl_debugger')
        
        self.topics_status = {
            '/map': False,
            '/field_scan': False,
            '/particlecloud': False,
            '/amcl_pose': False,
        }
        
        # Subscribe to all relevant topics
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            lambda msg: self.topic_callback('/map', msg),
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/field_scan',
            lambda msg: self.topic_callback('/field_scan', msg),
            10
        )
        
        self.particles_sub = self.create_subscription(
            PoseArray,
            '/particlecloud',
            lambda msg: self.topic_callback('/particlecloud', msg),
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            lambda msg: self.topic_callback('/amcl_pose', msg),
            10
        )
        
        # Check timer
        self.timer = self.create_timer(2.0, self.check_status)
        
        self.get_logger().info('🔍 AMCL Debugger Started')
        self.get_logger().info('Checking AMCL topics...\n')
        
    def topic_callback(self, topic_name, msg):
        if not self.topics_status[topic_name]:
            self.topics_status[topic_name] = True
            self.get_logger().info(f'✅ {topic_name}: RECEIVING DATA!')
            
            # Extra info for specific topics
            if topic_name == '/map':
                self.get_logger().info(f'   Map size: {msg.info.width}x{msg.info.height}')
                self.get_logger().info(f'   Resolution: {msg.info.resolution}m/cell')
                self.get_logger().info(f'   Frame: {msg.header.frame_id}')
                
            elif topic_name == '/field_scan':
                valid = sum(1 for r in msg.ranges if r > msg.range_min and r < msg.range_max)
                self.get_logger().info(f'   Valid ranges: {valid}/{len(msg.ranges)}')
                self.get_logger().info(f'   Frame: {msg.header.frame_id}')
                
            elif topic_name == '/particlecloud':
                self.get_logger().info(f'   Particles: {len(msg.poses)}')
                self.get_logger().info(f'   Frame: {msg.header.frame_id}')
                
    def check_status(self):
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('AMCL TOPIC STATUS CHECK')
        self.get_logger().info('='*60)
        
        all_good = True
        
        for topic, status in self.topics_status.items():
            icon = '✅' if status else '❌'
            self.get_logger().info(f'{icon} {topic}: {"OK" if status else "NOT RECEIVING"}')
            if not status:
                all_good = False
        
        self.get_logger().info('='*60 + '\n')
        
        if all_good:
            self.get_logger().info('🎉 ALL TOPICS OK! AMCL should be working!')
            self.get_logger().info('If localization still not working, check:')
            self.get_logger().info('  1. Initial pose is set')
            self.get_logger().info('  2. TF tree is correct (ros2 run tf2_tools view_frames)')
            self.get_logger().info('  3. AMCL parameters (especially base_frame_id)')
        elif not self.topics_status['/particlecloud']:
            self.get_logger().warn('❌ AMCL NOT PUBLISHING PARTICLES!')
            self.get_logger().warn('Possible causes:')
            
            if not self.topics_status['/map']:
                self.get_logger().warn('  1. ❌ No map - AMCL needs map to localize!')
            else:
                self.get_logger().info('  1. ✅ Map is available')
                
            if not self.topics_status['/field_scan']:
                self.get_logger().warn('  2. ❌ No scan data - AMCL needs laser scan!')
            else:
                self.get_logger().info('  2. ✅ Scan data is available')
                
            self.get_logger().warn('  3. Check AMCL is actually running: ros2 node list | grep amcl')
            self.get_logger().warn('  4. Check AMCL logs for errors')
            self.get_logger().warn('  5. Check TF frames match (base_frame_id in AMCL vs scan frame)')

def main(args=None):
    rclpy.init(args=args)
    node = AMCLDebugger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()