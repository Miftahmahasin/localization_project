#!/bin/bash
# Test pointcloud_to_laserscan with correct parameters

echo "🔧 Starting pointcloud_to_laserscan test..."
echo "================================================"
echo ""
echo "This will convert /field_point_cloud to /field_scan"
echo "with EXPANDED height filtering to accept all Z values"
echo ""
echo "Press Ctrl+C to stop"
echo "================================================"
echo ""

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -p min_height:=-10.0 \
  -p max_height:=10.0 \
  -p range_min:=0.05 \
  -p range_max:=10.0 \
  -p angle_min:=-3.14159 \
  -p angle_max:=3.14159 \
  -p angle_increment:=0.0174533 \
  -p transform_tolerance:=1.0 \
  -p use_sim_time:=true \
  -r cloud_in:=/field_point_cloud \
  -r scan:=/field_scan \
  --log-level info