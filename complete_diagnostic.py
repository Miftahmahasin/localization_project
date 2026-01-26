#!/usr/bin/env python3
"""
Complete Diagnostic for PointCloud to LaserScan Issue
Checks everything: data, transforms, parameters, timing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from tf2_ros import Buffer, TransformListener
import struct
import math
import time

class CompleteDiagnostic(Node):
    def __init__(self):
        super().__init__('complete_diagnostic')
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Track what we've seen
        self.pc_count = 0
        self.scan_count = 0
        self.pc_analyzed = False
        self.scan_analyzed = False
        
        # Subscribers
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
        
        self.get_logger().info('🔍 Starting Complete Diagnostic...')
        self.get_logger().info('Waiting for data...')
        
    def pc_callback(self, msg):
        self.pc_count += 1
        
        if not self.pc_analyzed:
            self.analyze_pointcloud(msg)
            self.pc_analyzed = True
    
    def scan_callback(self, msg):
        self.scan_count += 1
        
        if not self.scan_analyzed:
            self.analyze_laserscan(msg)
            self.scan_analyzed = True
            
        if self.pc_analyzed and self.scan_analyzed:
            self.final_diagnosis()
            
    def analyze_pointcloud(self, msg):
        print("\n" + "="*70)
        print("📊 POINTCLOUD ANALYSIS")
        print("="*70)
        
        # Decode points
        points = []
        point_step = msg.point_step
        
        for i in range(0, len(msg.data), point_step):
            point_data = msg.data[i:i+point_step]
            x, y, z = struct.unpack('fff', bytes(point_data))
            points.append((x, y, z))
        
        print(f"Frame: {msg.header.frame_id}")
        print(f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        print(f"Total points: {len(points)}")
        
        if not points:
            print("❌ NO POINTS!")
            return
        
        # Stats
        x_vals = [p[0] for p in points]
        y_vals = [p[1] for p in points]
        z_vals = [p[2] for p in points]
        
        print(f"\nCoordinates:")
        print(f"  X: [{min(x_vals):.3f}, {max(x_vals):.3f}]")
        print(f"  Y: [{min(y_vals):.3f}, {max(y_vals):.3f}]")
        print(f"  Z: [{min(z_vals):.3f}, {max(z_vals):.3f}]")
        
        # For LaserScan
        angles = [math.atan2(y, x) for x, y, _ in points]
        distances = [math.sqrt(x**2 + y**2) for x, y, _ in points]
        
        print(f"\nFor LaserScan conversion:")
        print(f"  Angles: [{math.degrees(min(angles)):.1f}°, {math.degrees(max(angles)):.1f}°]")
        print(f"  Distances: [{min(distances):.3f}, {max(distances):.3f}]m")
        
        # Sample
        print(f"\nFirst 3 points:")
        for i in range(min(3, len(points))):
            x, y, z = points[i]
            angle = math.atan2(y, x)
            dist = math.sqrt(x**2 + y**2)
            print(f"  {i}: ({x:.3f}, {y:.3f}, {z:.3f}) → {dist:.3f}m @ {math.degrees(angle):.1f}°")
    
    def analyze_laserscan(self, msg):
        print("\n" + "="*70)
        print("📡 LASERSCAN ANALYSIS")
        print("="*70)
        
        print(f"Frame: {msg.header.frame_id}")
        print(f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        print(f"Total ranges: {len(msg.ranges)}")
        
        # Count valid vs inf
        valid_ranges = [r for r in msg.ranges if math.isfinite(r) and r > msg.range_min]
        inf_count = len([r for r in msg.ranges if not math.isfinite(r)])
        
        print(f"\nRange distribution:")
        print(f"  Valid ranges: {len(valid_ranges)}/{len(msg.ranges)} ({100*len(valid_ranges)/len(msg.ranges):.1f}%)")
        print(f"  Infinite: {inf_count} ({100*inf_count/len(msg.ranges):.1f}%)")
        
        if valid_ranges:
            print(f"  Valid range values: [{min(valid_ranges):.3f}, {max(valid_ranges):.3f}]m")
            print(f"\nFirst 5 valid ranges:")
            count = 0
            for i, r in enumerate(msg.ranges):
                if math.isfinite(r) and r > msg.range_min:
                    angle = msg.angle_min + i * msg.angle_increment
                    print(f"    Index {i}: {r:.3f}m @ {math.degrees(angle):.1f}°")
                    count += 1
                    if count >= 5:
                        break
        else:
            print("  ❌ ALL RANGES ARE INVALID!")
        
        print(f"\nScan parameters:")
        print(f"  Angle range: [{math.degrees(msg.angle_min):.1f}°, {math.degrees(msg.angle_max):.1f}°]")
        print(f"  Angle increment: {math.degrees(msg.angle_increment):.3f}°")
        print(f"  Range limits: [{msg.range_min:.3f}, {msg.range_max:.3f}]m")
    
    def final_diagnosis(self):
        print("\n" + "="*70)
        print("🔬 FINAL DIAGNOSIS")
        print("="*70)
        
        print(f"\nMessage counts:")
        print(f"  PointCloud: {self.pc_count} messages")
        print(f"  LaserScan: {self.scan_count} messages")
        
        # Check TF
        print(f"\nTF Check:")
        try:
            # Try cam_link to base_link
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                'cam_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            print(f"  ✅ cam_link → base_link transform EXISTS")
            t = trans.transform.translation
            print(f"     Translation: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})")
        except Exception as e:
            print(f"  ⚠️  cam_link → base_link: {str(e)}")
        
        try:
            # Try map to cam_link
            trans = self.tf_buffer.lookup_transform(
                'map',
                'cam_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            print(f"  ✅ map → cam_link transform EXISTS")
        except Exception as e:
            print(f"  ℹ️  map → cam_link: {str(e)}")
        
        print("\n" + "="*70)
        print("💡 RECOMMENDATIONS:")
        print("="*70)
        
        if self.scan_count == 0:
            print("\n❌ NO LASERSCAN MESSAGES!")
            print("   → pointcloud_to_laserscan is not publishing")
            print("   → Check if node is running and has subscribers")
        elif self.pc_count > 0 and self.scan_count > 0:
            print("\n✅ Both PointCloud and LaserScan are publishing")
            print("   → Check LaserScan analysis above for details")
            print("   → If all ranges are .inf, problem is in conversion")
        
        print("\n")
        
        # Shutdown after a moment
        self.create_timer(2.0, self.shutdown)
    
    def shutdown(self):
        self.get_logger().info('Diagnostic complete. Shutting down...')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CompleteDiagnostic()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()