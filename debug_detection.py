#!/usr/bin/env python3
"""
Debug script to check field line detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DebugDetection(Node):
    def __init__(self):
        super().__init__('debug_detection')
        self.br = CvBridge()
        
        self.sub = self.create_subscription(
            Image,
            '/robotis_op3/camera/image_raw',
            self.callback,
            10
        )
        
        self.get_logger().info("Debug detection started")
        self.frame_count = 0
        
    def callback(self, msg):
        self.frame_count += 1
        
        if self.frame_count % 30 != 0:  # Process every 30th frame
            return
        
        # Convert to OpenCV
        cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
        
        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Check image statistics
        mean_val = np.mean(gray)
        max_val = np.max(gray)
        
        # Count white pixels at different thresholds
        white_200 = np.sum(gray > 200)
        white_180 = np.sum(gray > 180)
        white_150 = np.sum(gray > 150)
        
        total_pixels = gray.shape[0] * gray.shape[1]
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Frame {self.frame_count}:")
        self.get_logger().info(f"  Image size: {gray.shape}")
        self.get_logger().info(f"  Mean brightness: {mean_val:.1f}")
        self.get_logger().info(f"  Max brightness: {max_val}")
        self.get_logger().info(f"  White pixels (>200): {white_200} ({100*white_200/total_pixels:.1f}%)")
        self.get_logger().info(f"  White pixels (>180): {white_180} ({100*white_180/total_pixels:.1f}%)")
        self.get_logger().info(f"  White pixels (>150): {white_150} ({100*white_150/total_pixels:.1f}%)")
        
        # Find brightest region
        if max_val > 200:
            brightest = np.where(gray == max_val)
            y = brightest[0][0]
            x = brightest[1][0]
            self.get_logger().info(f"  Brightest pixel at: ({x}, {y})")
        
        self.get_logger().info("=" * 60)

def main():
    rclpy.init()
    node = DebugDetection()
    rclpy.spin(node)

if __name__ == '__main__':
    main()