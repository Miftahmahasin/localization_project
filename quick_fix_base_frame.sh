#!/bin/bash
# Quick Fix: Change base_frame back to cam_link

echo "🔧 QUICK FIX: Changing base_frame to cam_link"
echo "========================================================================"
echo ""

echo "The issue: base_frame_id = base_link, but scan is in cam_link frame"
echo "Solution: Change base_frame_id back to cam_link"
echo ""

echo "Applying fix..."
ros2 param set /amcl base_frame_id cam_link

if [ $? -eq 0 ]; then
    echo "✅ Parameter changed"
else
    echo "❌ Failed - is AMCL running?"
    exit 1
fi

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
    pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}},
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]
  }
}" --once &

sleep 5

echo ""
echo "Checking result..."
ros2 topic echo /amcl_pose --once 2>&1 | grep -A 2 "covariance:" | head -3

echo ""
echo "========================================================================"
echo "DONE! Check RViz:"
echo "  - Scan should now be on field plane"
echo "  - Point cloud should be on field plane"
echo "  - Move robot and watch pose update"
echo "========================================================================"