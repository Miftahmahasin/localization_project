#!/usr/bin/env python3
"""
PointCloud Binary Data Decoder
Decode the actual float32 values from binary data
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import math

class PCDecoder(Node):
    def __init__(self):
        super().__init__('pc_decoder')
        self.get_logger().info('🔍 Starting PointCloud Binary Decoder...')
        
        self.sub = self.create_subscription(
            PointCloud2, 
            '/field_point_cloud', 
            self.decode_callback, 
            10
        )
        
        self.analyzed = False
        
    def decode_callback(self, msg):
        if self.analyzed:
            return
            
        self.get_logger().info('📊 Decoding PointCloud Binary Data...')
        
        try:
            # Decode binary data manually
            points = []
            point_step = msg.point_step  # 12 bytes per point (4 bytes * 3 fields)
            
            for i in range(0, len(msg.data), point_step):
                # Extract 12 bytes for one point
                point_data = msg.data[i:i+point_step]
                
                # Unpack as 3 float32 values (x, y, z)
                x, y, z = struct.unpack('fff', bytes(point_data))
                
                points.append((x, y, z))
            
            if not points:
                self.get_logger().error('❌ No points decoded!')
                return
            
            # Analyze
            print("\n" + "="*70)
            print("📊 POINTCLOUD DECODED VALUES")
            print("="*70)
            print(f"Frame: {msg.header.frame_id}")
            print(f"Total points: {len(points)}")
            
            # Calculate stats
            x_vals = [p[0] for p in points]
            y_vals = [p[1] for p in points]
            z_vals = [p[2] for p in points]
            
            print("\n" + "-"*70)
            print("📏 COORDINATE RANGES:")
            print("-"*70)
            print(f"X: [{min(x_vals):.4f}, {max(x_vals):.4f}] (range: {max(x_vals)-min(x_vals):.4f}m)")
            print(f"Y: [{min(y_vals):.4f}, {max(y_vals):.4f}] (range: {max(y_vals)-min(y_vals):.4f}m)")
            print(f"Z: [{min(z_vals):.4f}, {max(z_vals):.4f}] (range: {max(z_vals)-min(z_vals):.4f}m)")
            
            # Calculate angles (for LaserScan)
            print("\n" + "-"*70)
            print("📐 ANGLE DISTRIBUTION (for LaserScan conversion):")
            print("-"*70)
            
            angles = []
            distances = []
            for x, y, z in points:
                # In camera frame: X=forward, Y=left/right
                # LaserScan angle = atan2(y, x)
                angle = math.atan2(y, x)
                distance = math.sqrt(x**2 + y**2)
                angles.append(angle)
                distances.append(distance)
            
            print(f"Angle range: [{min(angles):.3f}, {max(angles):.3f}] rad")
            print(f"           = [{math.degrees(min(angles)):.1f}°, {math.degrees(max(angles)):.1f}°]")
            print(f"Distance range: [{min(distances):.3f}, {max(distances):.3f}]m")
            
            # Count by angle bins
            angle_bins = {
                'front (±30°)': 0,
                'left (30-90°)': 0,
                'right (-30--90°)': 0,
                'back (±150-180°)': 0
            }
            
            for angle in angles:
                deg = math.degrees(angle)
                if -30 <= deg <= 30:
                    angle_bins['front (±30°)'] += 1
                elif 30 < deg <= 90:
                    angle_bins['left (30-90°)'] += 1
                elif -90 <= deg < -30:
                    angle_bins['right (-30--90°)'] += 1
                else:
                    angle_bins['back (±150-180°)'] += 1
            
            print("\n📊 Points by direction:")
            for direction, count in angle_bins.items():
                pct = 100.0 * count / len(points)
                print(f"  {direction:20s}: {count:3d} ({pct:5.1f}%)")
            
            # Sample points
            print("\n" + "-"*70)
            print("📍 SAMPLE POINTS (first 10):")
            print("-"*70)
            print("     X        Y        Z     | Distance  Angle")
            print("-"*70)
            for i in range(min(10, len(points))):
                x, y, z = points[i]
                dist = math.sqrt(x**2 + y**2)
                angle = math.atan2(y, x)
                print(f"{i:2d}: {x:7.3f}  {y:7.3f}  {z:7.3f}  | {dist:7.3f}m  {math.degrees(angle):6.1f}°")
            
            # Check pointcloud_to_laserscan compatibility
            print("\n" + "-"*70)
            print("🔍 LASERSCAN CONVERSION CHECK:")
            print("-"*70)
            
            # Standard angle range: -π to +π
            in_scan_range = sum(1 for a in angles if -math.pi <= a <= math.pi)
            print(f"Points in LaserScan angle range [-π, +π]: {in_scan_range}/{len(points)} ({100*in_scan_range/len(points):.1f}%)")
            
            # Check Z filtering (default -0.01 to 0.01)
            in_height_range = sum(1 for z in z_vals if -0.01 <= z <= 0.01)
            print(f"Points in default height range [-0.01, 0.01]: {in_height_range}/{len(points)} ({100*in_height_range/len(points):.1f}%)")
            
            # Check if all Z are exactly 0
            all_zero_z = all(z == 0.0 for z in z_vals)
            print(f"All Z values exactly 0.0? {all_zero_z}")
            
            if all_zero_z and in_scan_range == len(points):
                print("\n✅ Data looks good for LaserScan conversion!")
                print("   Problem must be elsewhere (transform, timing, etc.)")
            elif not all_zero_z:
                print(f"\n⚠️  WARNING: Z values not zero!")
                print(f"   This will cause height filtering issues!")
            
            print("\n" + "="*70)
            print("✅ Decode complete!")
            print("="*70 + "\n")
            
            self.analyzed = True
            self.create_timer(1.0, self.shutdown)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error decoding: {str(e)}')
            import traceback
            traceback.print_exc()
    
    def shutdown(self):
        self.get_logger().info('Shutting down...')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PCDecoder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()