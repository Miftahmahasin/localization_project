#!/bin/bash
# Check pointcloud_to_laserscan actual parameters

echo "🔍 Checking pointcloud_to_laserscan parameters..."
echo "================================================================"
echo ""

ros2 param list /pointcloud_to_laserscan

echo ""
echo "================================================================"
echo "📊 Critical parameters:"
echo "================================================================"

echo ""
echo "Height filtering:"
ros2 param get /pointcloud_to_laserscan min_height
ros2 param get /pointcloud_to_laserscan max_height

echo ""
echo "Angle range:"
ros2 param get /pointcloud_to_laserscan angle_min
ros2 param get /pointcloud_to_laserscan angle_max
ros2 param get /pointcloud_to_laserscan angle_increment

echo ""
echo "Distance range:"
ros2 param get /pointcloud_to_laserscan range_min
ros2 param get /pointcloud_to_laserscan range_max

echo ""
echo "Transform settings:"
ros2 param get /pointcloud_to_laserscan target_frame
ros2 param get /pointcloud_to_laserscan transform_tolerance

echo ""
echo "================================================================"