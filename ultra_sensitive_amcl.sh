#!/bin/bash
# Ultra-Sensitive AMCL for Pure Vision Tracking

echo "🔧 CONFIGURING AMCL FOR PURE VISION TRACKING"
echo "========================================================================"
echo ""

echo "Making AMCL ultra-sensitive to scan changes..."
echo ""

# CRITICAL: Very low update thresholds
ros2 param set /amcl update_min_d 0.01  # Update every 1cm!
ros2 param set /amcl update_min_a 0.01  # Update every ~0.5 degrees!

# CRITICAL: Completely ignore odometry
ros2 param set /amcl alpha1 0.000001
ros2 param set /amcl alpha2 0.000001
ros2 param set /amcl alpha3 0.000001
ros2 param set /amcl alpha4 0.000001
ros2 param set /amcl alpha5 0.000001

# More particles for better tracking
ros2 param set /amcl min_particles 1000
ros2 param set /amcl max_particles 3000

# Resample every update
ros2 param set /amcl resample_interval 1

# Very forgiving laser model
ros2 param set /amcl z_hit 0.5
ros2 param set /amcl z_rand 0.5

# Longer transform tolerance
ros2 param set /amcl transform_tolerance 1.0

echo "✅ Parameters set"
echo ""

echo "Restarting AMCL..."
ros2 lifecycle set /amcl deactivate
sleep 1
ros2 lifecycle set /amcl cleanup
sleep 1
ros2 lifecycle set /amcl configure
sleep 1
ros2 lifecycle set /amcl activate
sleep 2

echo ""
echo "Setting initial pose..."
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [
      0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.2
    ]
  }
}" --once &

sleep 5

echo ""
echo "========================================================================"
echo "TESTING"
echo "========================================================================"
echo ""

echo "1. Check covariance (should be non-zero):"
ros2 topic echo /amcl_pose --once 2>&1 | grep -A 2 "covariance:" | head -3

echo ""
echo "2. NOW MOVE ROBOT IN WEBOTS"
echo ""
echo "3. Watch pose update:"
echo "   ros2 topic echo /amcl_pose | grep -A 3 'position:'"
echo ""

echo "If pose still doesn't update:"
echo "  - Check scan is changing: ros2 topic echo /field_scan | head -50"
echo "  - AMCL may need even more sensitivity"
echo "  - OR scan quality too low for pure vision tracking"
echo ""
echo "========================================================================"