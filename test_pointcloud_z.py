#!/usr/bin/env python3
"""
PointCloud Z-Value Analyzer
Diagnose why pointcloud_to_laserscan produces all .inf values
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PCAnalyzer(Node):
    def __init__(self):
        super().__init__('pc_analyzer')
        self.get_logger().info('🔍 Starting PointCloud Analysis...')
        
        self.sub = self.create_subscription(
            PointCloud2, 
            '/field_point_cloud', 
            self.analyze_callback, 
            10
        )
        
        self.analyzed = False
        
    def analyze_callback(self, msg):
        if self.analyzed:
            return
            
        self.get_logger().info('📊 Analyzing PointCloud...')
        
        try:
            # Read all points
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            
            if not points:
                self.get_logger().error('❌ No valid points found!')
                return
                
            # Convert to numpy for analysis
            points_array = np.array(points)
            x_vals = points_array[:, 0]
            y_vals = points_array[:, 1]
            z_vals = points_array[:, 2]
            
            # Print detailed analysis
            print("\n" + "="*60)
            print("📊 POINTCLOUD ANALYSIS RESULTS")
            print("="*60)
            print(f"Frame ID: {msg.header.frame_id}")
            print(f"Total points: {len(points)}")
            print(f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
            print("\n" + "-"*60)
            print("📏 COORDINATE RANGES:")
            print("-"*60)
            print(f"X: [{x_vals.min():.4f}, {x_vals.max():.4f}] (range: {x_vals.max()-x_vals.min():.4f}m)")
            print(f"Y: [{y_vals.min():.4f}, {y_vals.max():.4f}] (range: {y_vals.max()-y_vals.min():.4f}m)")
            print(f"Z: [{z_vals.min():.4f}, {z_vals.max():.4f}] (range: {z_vals.max()-z_vals.min():.4f}m)")
            
            print("\n" + "-"*60)
            print("📊 Z-VALUE STATISTICS:")
            print("-"*60)
            print(f"Mean Z: {z_vals.mean():.4f}m")
            print(f"Median Z: {np.median(z_vals):.4f}m")
            print(f"Std Dev Z: {z_vals.std():.4f}m")
            
            # Check if all Z values are the same
            z_unique = np.unique(z_vals)
            print(f"\nUnique Z values: {len(z_unique)}")
            if len(z_unique) <= 5:
                print(f"Z values: {z_unique}")
            
            # Check typical pointcloud_to_laserscan defaults
            print("\n" + "-"*60)
            print("🔍 POINTCLOUD_TO_LASERSCAN FILTER CHECK:")
            print("-"*60)
            
            default_min_height = -0.01
            default_max_height = 0.01
            
            in_default_range = np.sum((z_vals >= default_min_height) & (z_vals <= default_max_height))
            print(f"Default height filter: [{default_min_height}, {default_max_height}]")
            print(f"Points in default range: {in_default_range}/{len(points)} ({100*in_default_range/len(points):.1f}%)")
            
            if in_default_range == 0:
                print("\n⚠️  WARNING: NO POINTS IN DEFAULT HEIGHT RANGE!")
                print("This is why your laser scan is all .inf!")
                print("\n💡 SOLUTION:")
                print(f"Set min_height to: {z_vals.min() - 0.01:.4f}")
                print(f"Set max_height to: {z_vals.max() + 0.01:.4f}")
            
            # Show sample points
            print("\n" + "-"*60)
            print("📍 SAMPLE POINTS (first 10):")
            print("-"*60)
            print("    X        Y        Z")
            for i, (x, y, z) in enumerate(points[:10]):
                print(f"{i:2d}: {x:7.3f}  {y:7.3f}  {z:7.3f}")
            
            # Distance analysis
            print("\n" + "-"*60)
            print("📐 DISTANCE FROM ORIGIN:")
            print("-"*60)
            distances = np.sqrt(x_vals**2 + y_vals**2)
            print(f"Min distance: {distances.min():.3f}m")
            print(f"Max distance: {distances.max():.3f}m")
            print(f"Mean distance: {distances.mean():.3f}m")
            
            print("\n" + "="*60)
            print("✅ Analysis complete!")
            print("="*60 + "\n")
            
            self.analyzed = True
            
            # Shutdown after 1 second
            self.create_timer(1.0, self.shutdown)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error analyzing pointcloud: {str(e)}')
            import traceback
            traceback.print_exc()
    
    def shutdown(self):
        self.get_logger().info('Shutting down...')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PCAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()