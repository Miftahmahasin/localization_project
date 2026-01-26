#!/bin/bash
# AMCL Node Inspector
# Check if AMCL is configured correctly

echo "🔍 AMCL Node Inspector"
echo "========================================================================"
echo ""

# Check if AMCL node exists
echo "1. Checking if AMCL node is running..."
if ros2 node list | grep -q "amcl"; then
    echo "   ✅ AMCL node is running"
else
    echo "   ❌ AMCL node NOT FOUND!"
    exit 1
fi

echo ""
echo "2. Checking AMCL subscriptions..."
ros2 node info /amcl | grep "Subscribers:" -A 10

echo ""
echo "3. Checking AMCL publishers..."
ros2 node info /amcl | grep "Publishers:" -A 10

echo ""
echo "4. Checking AMCL parameters (critical ones)..."
echo "   Frame IDs:"
ros2 param get /amcl base_frame_id
ros2 param get /amcl odom_frame_id
ros2 param get /amcl global_frame_id

echo ""
echo "   Initial pose:"
ros2 param get /amcl set_initial_pose

echo ""
echo "   Particle filter:"
ros2 param get /amcl min_particles
ros2 param get /amcl max_particles

echo ""
echo "   Laser parameters:"
ros2 param get /amcl laser_max_range
ros2 param get /amcl laser_min_range

echo ""
echo "5. Checking topic connections..."
echo "   /field_scan publishers:"
ros2 topic info /field_scan | grep "Publisher count"

echo "   /field_scan subscribers:"
ros2 topic info /field_scan | grep "Subscription count"

echo ""
echo "   /map publishers:"
ros2 topic info /map | grep "Publisher count"

echo "   /map subscribers:"
ros2 topic info /map | grep "Subscription count"

echo ""
echo "6. Checking TF tree..."
echo "   Available frames:"
ros2 run tf2_ros tf2_monitor --all-frames 2>&1 | head -20

echo ""
echo "========================================================================"
echo "✅ Inspection complete!"
echo ""
echo "If AMCL is not localizing, common issues:"
echo "  - base_frame_id doesn't match scan frame (should be 'cam_link')"
echo "  - No transform between odom and base_frame_id"
echo "  - Initial pose not set"
echo "  - Map not received"
echo ""