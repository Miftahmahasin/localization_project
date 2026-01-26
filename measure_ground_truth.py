#!/usr/bin/env python3
"""
Measure ground truth distances in Webots
Compare with projected distances
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class GroundTruthMeasure(Node):
    def __init__(self):
        super().__init__('ground_truth_measure')
        self.br = CvBridge()
        
        self.sub = self.create_subscription(
            Image,
            '/robotis_op3/camera/image_raw',
            self.callback,
            10
        )
        
        self.frame_count = 0
        self.get_logger().info("Click on image to measure distances")
        self.get_logger().info("Known references in Webots:")
        self.get_logger().info("  - Center circle radius: 0.75m")
        self.get_logger().info("  - Penalty box: 1.65m from goal")
        self.get_logger().info("  - Goal width: 2.6m")
        
    def callback(self, msg):
        self.frame_count += 1
        if self.frame_count % 30 != 0:
            return
        
        cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
        
        # Draw reference lines
        height, width = cv_image.shape[:2]
        
        # Center vertical line
        cv2.line(cv_image, (width//2, 0), (width//2, height), (0, 255, 0), 2)
        
        # Horizon estimate (where ground meets sky)
        horizon_y = int(height * 0.35)  # Approximate
        cv2.line(cv_image, (0, horizon_y), (width, horizon_y), (255, 0, 0), 1)
        
        # Draw horizontal lines at different heights
        for y in [300, 400, 500, 600, 650, 700]:
            if y < height:
                cv2.line(cv_image, (0, y), (width, y), (0, 255, 255), 1)
                cv2.putText(cv_image, f"v={y}", (10, y-5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Add text
        cv2.putText(cv_image, "Center column: image X=640", 
                   (width//2 - 150, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow("Ground Truth Reference", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = GroundTruthMeasure()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()