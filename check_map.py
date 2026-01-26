#!/usr/bin/env python3
"""
Check Map with Correct QoS
Map typically uses TRANSIENT_LOCAL durability
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

class MapChecker(Node):
    def __init__(self):
        super().__init__('map_checker')
        
        # Map typically uses this QoS
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.get_logger().info('🗺️  Checking /map with TRANSIENT_LOCAL QoS...')
        
        self.map_received = False
        
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        
        # Timeout timer
        self.timer = self.create_timer(10.0, self.timeout)
        
    def map_callback(self, msg):
        if not self.map_received:
            self.map_received = True
            
            self.get_logger().info('\n' + '='*70)
            self.get_logger().info('✅ MAP RECEIVED!')
            self.get_logger().info('='*70)
            self.get_logger().info(f'Frame: {msg.header.frame_id}')
            self.get_logger().info(f'Size: {msg.info.width} x {msg.info.height} cells')
            self.get_logger().info(f'Resolution: {msg.info.resolution} m/cell')
            self.get_logger().info(f'Origin: ({msg.info.origin.position.x:.2f}, '
                                 f'{msg.info.origin.position.y:.2f}, '
                                 f'{msg.info.origin.position.z:.2f})')
            
            # Count occupied/free/unknown
            data = msg.data
            occupied = sum(1 for x in data if x > 50)
            free = sum(1 for x in data if 0 <= x <= 50)
            unknown = sum(1 for x in data if x < 0)
            
            total = len(data)
            self.get_logger().info(f'\nMap data:')
            self.get_logger().info(f'  Total cells: {total}')
            self.get_logger().info(f'  Occupied: {occupied} ({100*occupied/total:.1f}%)')
            self.get_logger().info(f'  Free: {free} ({100*free/total:.1f}%)')
            self.get_logger().info(f'  Unknown: {unknown} ({100*unknown/total:.1f}%)')
            
            self.get_logger().info('='*70)
            self.get_logger().info('\n✅ Map is valid and being published!')
            self.get_logger().info('   AMCL should now be able to use it')
            self.get_logger().info('\nCheck AMCL particles:')
            self.get_logger().info('  ros2 topic echo /particlecloud --once')
            self.get_logger().info('='*70 + '\n')
            
            # Shutdown
            self.timer.cancel()
            self.create_timer(1.0, lambda: rclpy.shutdown())
    
    def timeout(self):
        if not self.map_received:
            self.get_logger().error('\n' + '='*70)
            self.get_logger().error('❌ NO MAP RECEIVED AFTER 10 SECONDS!')
            self.get_logger().error('='*70)
            self.get_logger().error('Possible causes:')
            self.get_logger().error('  1. Map server not activated')
            self.get_logger().error('  2. Map file not found or invalid')
            self.get_logger().error('  3. Lifecycle manager failed')
            self.get_logger().error('\nTry:')
            self.get_logger().error('  bash /mnt/user-data/outputs/diagnose_map.sh')
            self.get_logger().error('  bash /mnt/user-data/outputs/activate_map.sh')
            self.get_logger().error('='*70 + '\n')
            
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MapChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()