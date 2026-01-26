#!/usr/bin/env python3
"""
Check LaserScan with Correct QoS
ros2 topic echo might have QoS mismatch - let's check properly
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
import math

class ScanChecker(Node):
    def __init__(self):
        super().__init__('scan_checker')
        
        # Try multiple QoS profiles
        qos_profiles = [
            ('RELIABLE', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)),
            ('BEST_EFFORT', QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)),
            ('SENSOR_DATA', QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, 
                                       durability=DurabilityPolicy.VOLATILE))
        ]
        
        self.get_logger().info('🔍 Checking /field_scan with different QoS profiles...')
        
        # Try each profile
        for name, qos in qos_profiles:
            try:
                self.get_logger().info(f'\nTrying QoS: {name}')
                
                sub = self.create_subscription(
                    LaserScan,
                    '/field_scan',
                    lambda msg, n=name: self.callback(msg, n),
                    qos
                )
                
                self.get_logger().info(f'  ✅ Subscription created with {name}')
                
            except Exception as e:
                self.get_logger().error(f'  ❌ Failed with {name}: {e}')
        
        self.msg_received = {}
        
    def callback(self, msg, qos_name):
        if qos_name not in self.msg_received:
            self.msg_received[qos_name] = 0
        
        self.msg_received[qos_name] += 1
        
        if self.msg_received[qos_name] == 1:
            # First message with this QoS
            self.get_logger().info(f'\n✅ RECEIVED MESSAGE WITH {qos_name}!')
            
            # Analyze
            valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min < r < msg.range_max]
            inf_count = sum(1 for r in msg.ranges if math.isinf(r))
            
            self.get_logger().info(f'Frame: {msg.header.frame_id}')
            self.get_logger().info(f'Total ranges: {len(msg.ranges)}')
            self.get_logger().info(f'Valid ranges: {len(valid)} ({100*len(valid)/len(msg.ranges):.1f}%)')
            self.get_logger().info(f'Infinite: {inf_count} ({100*inf_count/len(msg.ranges):.1f}%)')
            
            if valid:
                self.get_logger().info(f'Valid range: [{min(valid):.3f}, {max(valid):.3f}]m')
                
                # Show first 5 valid
                self.get_logger().info('First 5 valid ranges:')
                count = 0
                for i, r in enumerate(msg.ranges):
                    if math.isfinite(r) and msg.range_min < r < msg.range_max:
                        angle = msg.angle_min + i * msg.angle_increment
                        self.get_logger().info(f'  [{i}]: {r:.3f}m @ {math.degrees(angle):.1f}°')
                        count += 1
                        if count >= 5:
                            break
            else:
                self.get_logger().warn('❌ NO VALID RANGES in received message!')

def main(args=None):
    rclpy.init(args=args)
    node = ScanChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()