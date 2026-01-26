#!/usr/bin/env python3
"""
Camera calibration helper
Uses known field features to estimate camera parameters
"""

import numpy as np
import sys
sys.path.append('/home/miftah/basbot/install/soccer_object_localization/lib/python3.10/site-packages')

from soccer_object_localization.static_camera_pose import StaticCameraPose

def test_parameters(height, tilt_deg, focal_length=900.0):
    """Test projection with given parameters"""
    
    camera_matrix = np.array([
        [focal_length, 0, 640],
        [0, focal_length, 360],
        [0, 0, 1]
    ], dtype=np.float32)
    
    camera = StaticCameraPose(
        camera_matrix=camera_matrix,
        camera_height=height,
        camera_tilt=np.radians(tilt_deg),
        camera_offset_x=0.08,
        camera_offset_y=0.0
    )
    
    print(f"\n{'='*60}")
    print(f"Testing: height={height}m, tilt={tilt_deg}°, focal={focal_length}px")
    print(f"{'='*60}")
    
    # Test various image rows (v coordinate)
    # Center column (u=640) only
    test_rows = [
        (640, 400, "Upper field"),
        (640, 500, "Mid field"),
        (640, 600, "Lower field"),
        (640, 650, "Very close"),
        (640, 700, "Bottom of image"),
    ]
    
    print(f"\n{'Pixel (u, v)':<20} {'Description':<15} {'Distance (m)':<12} {'Point (x, y, z)'}")
    print("-" * 75)
    
    for u, v, desc in test_rows:
        point = camera.project_pixel_to_ground(u, v)
        if point:
            distance = np.sqrt(point[0]**2 + point[1]**2)
            print(f"({u:4d}, {v:4d}){'':<8} {desc:<15} {distance:6.3f}m{'':<6} "
                  f"({point[0]:5.2f}, {point[1]:5.2f}, {point[2]:5.2f})")
        else:
            print(f"({u:4d}, {v:4d}){'':<8} {desc:<15} REJECTED")

# Test different parameter combinations
print("="*60)
print("CAMERA PARAMETER CALIBRATION")
print("="*60)

# Known ground truth from Webots/field:
print("\nKNOWN REFERENCES:")
print("  - Center circle radius: 0.75m")
print("  - Penalty area: 1.65m from goal line")
print("  - Goal width: 2.6m")
print("  - If robot at center, goal is ~4.5m away")

# Test current parameters
test_parameters(height=0.475, tilt_deg=-20.0)

# Test with higher camera
test_parameters(height=0.55, tilt_deg=-20.0)

# Test with more downward tilt
test_parameters(height=0.475, tilt_deg=-25.0)

# Test with both
test_parameters(height=0.55, tilt_deg=-25.0)

# Test with different focal length
test_parameters(height=0.475, tilt_deg=-20.0, focal_length=1000.0)

print("\n" + "="*60)
print("INSTRUCTIONS:")
print("="*60)
print("1. Look at Webots scene - note robot position")
print("2. In rqt_image_view, note pixel row (v) of known features:")
print("   - Bottom of penalty box (1.65m away)")
print("   - Center circle edge (0.75m away)")
print("   - Horizon line")
print("3. Compare with estimates above")
print("4. Choose parameters that match best")
print("="*60)