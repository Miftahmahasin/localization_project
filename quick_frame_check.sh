#!/bin/bash
# Quick AMCL Frame Check

echo "🔍 QUICK AMCL FRAME CHECK"
echo "========================================================================"

echo ""
echo "1. AMCL Frame Configuration:"
echo ""
ros2 param get /amcl base_frame_id
ros2 param get /amcl odom_frame_id  
ros2 param get /amcl global_frame_id

echo ""
echo "2. Scan Frame:"
echo ""
timeout 2 ros2 topic echo /field_scan --once 2>&1 | grep "frame_id:" | head -1

echo ""
echo "3. Expected Configuration:"
echo "   base_frame_id: base_link"
echo "   odom_frame_id: odom"
echo "   global_frame_id: map"
echo "   scan frame_id: base_link (or cam_link with proper transform)"

echo ""
echo "========================================================================"
echo "IF MISMATCH DETECTED:"
echo "========================================================================"
echo ""

cat << 'EOF'
If base_frame_id is NOT base_link, fix with:

  ros2 param set /amcl base_frame_id base_link

If scan frame is cam_link but base_frame is base_link:
  
  You need transform: base_link → cam_link
  
  This should be published by static_tf_publisher in launch file
  
  Check it exists:
    ros2 run tf2_ros tf2_echo base_link cam_link
    
  If error, the static transform is missing!

EOF

echo "========================================================================"