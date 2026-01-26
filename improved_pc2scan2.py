#!/usr/bin/env python3
"""
IMPROVED PointCloud to LaserScan Converter
Key improvements:
1. Finer angular resolution (0.5° instead of 1°)
2. Intelligent gap interpolation
3. Point averaging per bin
4. Better coverage statistics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import struct
import math

class ImprovedPC2Scan(Node):
    def __init__(self):
        super().__init__('improved_pc2scan')
        
        # Parameters
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', 0.0087266)  # 0.5° for finer resolution
        self.declare_parameter('range_min', 0.3)
        self.declare_parameter('range_max', 5.0)
        self.declare_parameter('interpolate_gaps', True)
        self.declare_parameter('max_gap_size', 10)  # Max gap to fill (bins)
        
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.interpolate = self.get_parameter('interpolate_gaps').value
        self.max_gap = self.get_parameter('max_gap_size').value
        
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        
        self.scan_pub = self.create_publisher(LaserScan, '/field_scan', 10)
        self.pc_sub = self.create_subscription(
            PointCloud2, '/field_point_cloud', self.convert_callback, 10
        )
        
        self.get_logger().info('🔄 Improved PC2Scan Started')
        self.get_logger().info(f'   Resolution: {math.degrees(self.angle_increment):.2f}°/bin')
        self.get_logger().info(f'   Total bins: {self.num_ranges}')
        self.get_logger().info(f'   Interpolation: {"ON" if self.interpolate else "OFF"}')
        
        self.scan_count = 0
        
    def convert_callback(self, pc_msg):
        points = []
        for i in range(0, len(pc_msg.data), pc_msg.point_step):
            try:
                x, y, z = struct.unpack('fff', bytes(pc_msg.data[i:i+pc_msg.point_step]))
                points.append((x, y, z))
            except:
                continue
        
        if not points:
            return
        
        # Initialize bins
        ranges = [float('inf')] * self.num_ranges
        counts = [0] * self.num_ranges
        sums = [0.0] * self.num_ranges
        
        # Fill bins with averaged points
        for x, y, z in points:
            angle = math.atan2(y, x)
            distance = math.sqrt(x*x + y*y)
            
            if angle < self.angle_min or angle > self.angle_max:
                continue
            if distance < self.range_min or distance > self.range_max:
                continue
            
            idx = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= idx < self.num_ranges:
                sums[idx] += distance
                counts[idx] += 1
        
        # Average points in each bin
        for i in range(self.num_ranges):
            if counts[i] > 0:
                ranges[i] = sums[i] / counts[i]
        
        # Interpolate gaps
        if self.interpolate:
            ranges = self._fill_gaps(ranges)
        
        # Publish
        scan = LaserScan()
        scan.header = pc_msg.header
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        
        self.scan_pub.publish(scan)
        
        # Stats
        self.scan_count += 1
        if self.scan_count % 50 == 0:
            valid = sum(1 for r in ranges if r != float('inf'))
            pct = 100 * valid / self.num_ranges
            self.get_logger().info(
                f'Scan #{self.scan_count}: {valid}/{self.num_ranges} ({pct:.1f}%) '
                f'from {len(points)} points'
            )
    
    def _fill_gaps(self, ranges):
        """Linear interpolation for small gaps"""
        filled = list(ranges)
        i = 0
        
        while i < len(filled):
            if filled[i] != float('inf'):
                i += 1
                continue
            
            # Found gap start
            gap_start = i
            while i < len(filled) and filled[i] == float('inf'):
                i += 1
            gap_end = i
            gap_size = gap_end - gap_start
            
            # Interpolate if small enough
            if gap_size <= self.max_gap:
                left = filled[gap_start - 1] if gap_start > 0 else float('inf')
                right = filled[gap_end] if gap_end < len(filled) else float('inf')
                
                if left != float('inf') and right != float('inf'):
                    for j in range(gap_start, gap_end):
                        alpha = (j - gap_start + 1) / (gap_size + 1)
                        filled[j] = left + alpha * (right - left)
        
        return filled

def main():
    rclpy.init()
    node = ImprovedPC2Scan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()