#!/usr/bin/env python3
"""
Detailed LaserScan Analyzer
Check what's really in the scan message vs what RViz shows
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanAnalyzer(Node):
    def __init__(self):
        super().__init__('scan_analyzer')
        
        self.sub = self.create_subscription(
            LaserScan,
            '/field_scan',
            self.callback,
            10
        )
        
        self.count = 0
        self.get_logger().info('🔍 Analyzing /field_scan...')
        
    def callback(self, msg):
        self.count += 1
        
        if self.count % 10 == 0:  # Every 10th message
            # Analyze ranges
            ranges = msg.ranges
            total = len(ranges)
            
            valid = [r for r in ranges if math.isfinite(r) and msg.range_min < r < msg.range_max]
            inf_count = sum(1 for r in ranges if math.isinf(r))
            nan_count = sum(1 for r in ranges if math.isnan(r))
            
            print(f"\n{'='*70}")
            print(f"LaserScan Analysis (message #{self.count})")
            print(f"{'='*70}")
            print(f"Frame: {msg.header.frame_id}")
            print(f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
            print(f"\nTotal ranges: {total}")
            print(f"  Valid ranges: {len(valid)} ({100*len(valid)/total:.1f}%)")
            print(f"  Infinite: {inf_count} ({100*inf_count/total:.1f}%)")
            print(f"  NaN: {nan_count} ({100*nan_count/total:.1f}%)")
            
            if valid:
                print(f"\nValid range statistics:")
                print(f"  Min: {min(valid):.3f}m")
                print(f"  Max: {max(valid):.3f}m")
                print(f"  Mean: {sum(valid)/len(valid):.3f}m")
                
                # Show first 10 valid
                print(f"\nFirst 10 valid ranges:")
                count = 0
                for i, r in enumerate(ranges):
                    if math.isfinite(r) and msg.range_min < r < msg.range_max:
                        angle = msg.angle_min + i * msg.angle_increment
                        print(f"  Index {i:3d}: {r:.3f}m @ {math.degrees(angle):6.1f}°")
                        count += 1
                        if count >= 10:
                            break
            else:
                print("\n❌ NO VALID RANGES!")
                print("   All values are .inf or NaN")
                
                # Sample first 20 values
                print("\nFirst 20 range values:")
                for i in range(min(20, len(ranges))):
                    print(f"  [{i}]: {ranges[i]}")
            
            print(f"\nScan parameters:")
            print(f"  Angle: [{math.degrees(msg.angle_min):.1f}°, {math.degrees(msg.angle_max):.1f}°]")
            print(f"  Increment: {math.degrees(msg.angle_increment):.3f}°")
            print(f"  Range limits: [{msg.range_min:.2f}, {msg.range_max:.2f}]m")
            print(f"{'='*70}\n")

def main(args=None):
    rclpy.init(args=args)
    node = ScanAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()