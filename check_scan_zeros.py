#!/usr/bin/env python3
"""
Check why scan has all zeros
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanChecker(Node):
    def __init__(self):
        super().__init__('scan_checker')
        
        self.sub = self.create_subscription(
            LaserScan,
            '/field_scan',
            self.callback,
            10
        )
        
        self.get_logger().info('Checking scan data...')
        self.count = 0
        
    def callback(self, msg):
        self.count += 1
        
        if self.count == 1:
            # Check first message
            self.get_logger().info(f'\n{"="*70}')
            self.get_logger().info(f'SCAN DATA CHECK')
            self.get_logger().info(f'{"="*70}')
            self.get_logger().info(f'Frame: {msg.header.frame_id}')
            self.get_logger().info(f'Total ranges: {len(msg.ranges)}')
            self.get_logger().info(f'Range min: {msg.range_min}')
            self.get_logger().info(f'Range max: {msg.range_max}')
            self.get_logger().info(f'Angle min: {msg.angle_min:.2f} rad')
            self.get_logger().info(f'Angle max: {msg.angle_max:.2f} rad')
            self.get_logger().info(f'Angle increment: {msg.angle_increment:.4f} rad')
            
            # Check range values
            import math
            valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min < r < msg.range_max]
            zeros = [r for r in msg.ranges if r == 0.0]
            infs = [r for r in msg.ranges if math.isinf(r)]
            
            self.get_logger().info(f'\nRange statistics:')
            self.get_logger().info(f'  Valid (finite, in range): {len(valid)}')
            self.get_logger().info(f'  Zeros: {len(zeros)}')
            self.get_logger().info(f'  Infinities: {len(infs)}')
            
            if len(valid) > 0:
                self.get_logger().info(f'  Min valid: {min(valid):.3f}m')
                self.get_logger().info(f'  Max valid: {max(valid):.3f}m')
                self.get_logger().info(f'  Avg valid: {sum(valid)/len(valid):.3f}m')
            
            # Show first 20 non-inf values
            self.get_logger().info(f'\nFirst 20 ranges (non-inf):')
            non_inf = [r for r in msg.ranges if not math.isinf(r)][:20]
            for i, r in enumerate(non_inf):
                self.get_logger().info(f'  [{i}]: {r:.3f}m')
            
            self.get_logger().info(f'{"="*70}\n')
            
            # Shutdown after first message
            self.create_timer(1.0, lambda: rclpy.shutdown())

def main(args=None):
    rclpy.init(args=args)
    node = ScanChecker()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()