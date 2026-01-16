#!/usr/bin/env python3
"""Test projection manually"""

import numpy as np
import sys
sys.path.append('/home/miftah/basbot/install/soccer_object_localization/lib/python3.10/site-packages')

from soccer_object_localization.static_camera_pose import StaticCameraPose

# Create camera matrix
focal_length = 900.0
img_width = 1280
img_height = 720

camera_matrix = np.array([
    [focal_length, 0, img_width / 2],
    [0, focal_length, img_height / 2],
    [0, 0, 1]
], dtype=np.float32)

# Create static camera pose
camera = StaticCameraPose(
    camera_matrix=camera_matrix,
    camera_height=0.475,
    camera_tilt=-0.349,
    camera_offset_x=0.08,
    camera_offset_y=0.0
)

print("Testing projection...")
print(f"Camera: height={camera.camera_height}m, tilt={np.degrees(camera.camera_tilt):.1f}°")
print()

# Test various pixels
test_pixels = [
    (640, 500),  # Center bottom
    (640, 600),  # Lower
    (640, 400),  # Upper
    (800, 500),  # Right
    (480, 500),  # Left
]

for u, v in test_pixels:
    point = camera.project_pixel_to_ground(u, v)
    if point:
        distance = np.sqrt(point[0]**2 + point[1]**2)
        print(f"Pixel ({u:4d}, {v:4d}) → Point ({point[0]:6.3f}, {point[1]:6.3f}, {point[2]:6.3f})  dist={distance:.3f}m")
    else:
        print(f"Pixel ({u:4d}, {v:4d}) → REJECTED")

print()
print("If you see valid points above, projection works!")
print("If all REJECTED, there's a problem with projection math.")