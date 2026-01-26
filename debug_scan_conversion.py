#!/usr/bin/env python3
"""
Debug Why LaserScan is All Inf
Check the complete conversion pipeline
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import struct
import math

class ScanDebugger(Node):
    def __init__(self):
        super().__init__('scan_debugger')
        
        self.pc_data = None
        self.scan_data = None
        
        # Subscribe to both
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/field_point_cloud',
            self.pc_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/field_scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info('🔍 Scan Debugger Started')
        self.get_logger().info('Collecting data...')
        
        # Timer to analyze
        self.timer = self.create_timer(5.0, self.analyze)
        
    def pc_callback(self, msg):
        if self.pc_data is None:
            self.pc_data = msg
            
    def scan_callback(self, msg):
        if self.scan_data is None:
            self.scan_data = msg
    
    def analyze(self):
        if self.pc_data is None or self.scan_data is None:
            self.get_logger().warn('Waiting for data...')
            return
            
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('ANALYSIS RESULTS')
        self.get_logger().info('='*70)
        
        # Decode pointcloud
        points = []
        point_step = self.pc_data.point_step
        
        for i in range(0, len(self.pc_data.data), point_step):
            point_data = self.pc_data.data[i:i+point_step]
            x, y, z = struct.unpack('fff', bytes(point_data))
            points.append((x, y, z))
        
        self.get_logger().info(f'\nPointCloud:')
        self.get_logger().info(f'  Total points: {len(points)}')
        self.get_logger().info(f'  Frame: {self.pc_data.header.frame_id}')
        
        if len(points) > 0:
            # Calculate what scan SHOULD be
            x_vals = [p[0] for p in points]
            y_vals = [p[1] for p in points]
            z_vals = [p[2] for p in points]
            
            self.get_logger().info(f'  X range: [{min(x_vals):.3f}, {max(x_vals):.3f}]')
            self.get_logger().info(f'  Y range: [{min(y_vals):.3f}, {max(y_vals):.3f}]')
            self.get_logger().info(f'  Z range: [{min(z_vals):.3f}, {max(z_vals):.3f}]')
            
            # Calculate angles and distances
            angles = []
            distances = []
            for x, y, z in points:
                angle = math.atan2(y, x)
                dist = math.sqrt(x*x + y*y)
                angles.append(angle)
                distances.append(dist)
            
            self.get_logger().info(f'  Angle range: [{math.degrees(min(angles)):.1f}°, {math.degrees(max(angles)):.1f}°]')
            self.get_logger().info(f'  Distance range: [{min(distances):.3f}, {max(distances):.3f}]m')
        
        # Check scan
        self.get_logger().info(f'\nLaserScan:')
        self.get_logger().info(f'  Total ranges: {len(self.scan_data.ranges)}')
        self.get_logger().info(f'  Frame: {self.scan_data.header.frame_id}')
        self.get_logger().info(f'  Angle: [{math.degrees(self.scan_data.angle_min):.1f}°, {math.degrees(self.scan_data.angle_max):.1f}°]')
        self.get_logger().info(f'  Range limits: [{self.scan_data.range_min:.2f}, {self.scan_data.range_max:.2f}]m')
        
        valid = [r for r in self.scan_data.ranges if math.isfinite(r) and self.scan_data.range_min < r < self.scan_data.range_max]
        inf_count = sum(1 for r in self.scan_data.ranges if math.isinf(r))
        
        self.get_logger().info(f'  Valid ranges: {len(valid)} ({100*len(valid)/len(self.scan_data.ranges):.1f}%)')
        self.get_logger().info(f'  Inf: {inf_count} ({100*inf_count/len(self.scan_data.ranges):.1f}%)')
        
        if len(valid) > 0:
            self.get_logger().info(f'  Valid range values: [{min(valid):.3f}, {max(valid):.3f}]m')
        
        # Diagnosis
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('DIAGNOSIS:')
        self.get_logger().info('='*70)
        
        if len(points) > 0 and len(valid) == 0:
            self.get_logger().error('❌ PointCloud has data but LaserScan is all .inf!')
            self.get_logger().error('')
            self.get_logger().error('Possible causes:')
            
            # Check if Z filtering issue
            z_nonzero = any(abs(z) > 0.01 for _, _, z in points)
            if z_nonzero:
                self.get_logger().error('  1. Z values not zero - height filtering issue')
                self.get_logger().error(f'     Z range: [{min(z_vals):.3f}, {max(z_vals):.3f}]')
            
            # Check if distance out of range
            if len(distances) > 0:
                if min(distances) < self.scan_data.range_min:
                    self.get_logger().error(f'  2. Points too close: {min(distances):.3f}m < {self.scan_data.range_min:.2f}m')
                if max(distances) > self.scan_data.range_max:
                    self.get_logger().error(f'  3. Points too far: {max(distances):.3f}m > {self.scan_data.range_max:.2f}m')
            
            # Check converter parameters
            self.get_logger().error('')
            self.get_logger().error('Check converter parameters:')
            self.get_logger().error('  ros2 param list /simple_pc2scan')
            self.get_logger().error('  ros2 param get /simple_pc2scan range_min')
            self.get_logger().error('  ros2 param get /simple_pc2scan range_max')
            
        elif len(points) > 0 and len(valid) > 0:
            self.get_logger().info(f'✅ Conversion working! {len(valid)} valid ranges from {len(points)} points')
        
        self.get_logger().info('='*70 + '\n')
        
        # Shutdown
        self.timer.cancel()
        self.create_timer(1.0, lambda: rclpy.shutdown())

def main(args=None):
    rclpy.init(args=args)
    node = ScanDebugger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()